/*
 * Simple Open EtherCAT Master Library 
 *
 * File    : nicdrv.c
 * Version : 1.2.5
 * Date    : 09-04-2011
 * Copyright (C) 2005-2011 Speciaal Machinefabriek Ketels v.o.f.
 * Copyright (C) 2005-2011 Arthur Ketels
 * Copyright (C) 2008-2009 TU/e Technische Universiteit Eindhoven 
 *
 * SOEM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the Free
 * Software Foundation.
 *
 * SOEM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * As a special exception, if other files instantiate templates or use macros
 * or inline functions from this file, or you compile this file and link it
 * with other works to produce a work based on this file, this file does not
 * by itself cause the resulting work to be covered by the GNU General Public
 * License. However the source code for this file must still be made available
 * in accordance with section (3) of the GNU General Public License.
 *
 * This exception does not invalidate any other reasons why a work based on
 * this file might be covered by the GNU General Public License.
 *
 * The EtherCAT Technology, the trade name and logo “EtherCAT” are the intellectual
 * property of, and protected by Beckhoff Automation GmbH. You can use SOEM for
 * the sole purpose of creating, using and/or selling or otherwise distributing
 * an EtherCAT network master provided that an EtherCAT Master License is obtained
 * from Beckhoff Automation GmbH.
 *
 * In case you did not receive a copy of the EtherCAT Master License along with
 * SOEM write to Beckhoff Automation GmbH, Eiserstraße 5, D-33415 Verl, Germany
 * (www.beckhoff.com).
 */

/** \file
 * \brief
 * EtherCAT RAW socket driver.
 *
 * Low level interface functions to send and receive EtherCAT packets.
 * EtherCAT has the property that packets are only send by the master,
 * and the send packets allways return in the receive buffer.
 * There can be multiple packets "on the wire" before they return.
 * To combine the received packets with the original send packets a buffer
 * system is installed. The identifier is put in the index item of the
 * EtherCAT header. The index is stored and compared when a frame is recieved.
 * If there is a match the packet can be combined with the transmit packet
 * and returned to the higher level function.
 *
 * The socket layer can exhibit a reversal in the packet order (rare).
 * If the Tx order is A-B-C the return order could be A-C-B. The indexed buffer
 * will reorder the packets automatically.
 *
 * The "redundant" option will configure two sockets and two NIC interfaces.
 * Slaves are connected to both interfaces, one on the IN port and one on the
 * OUT port. Packets are send via both interfaces. Any one of the connections
 * (also an interconnect) can be removed and the slaves are still serviced with
 * packets. The software layer will detect the possible failure modes and
 * compensate. If needed the packets from interface A are resend through interface B.
 * This layer if fully transparent for the higher layers.
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h> 
#include <sys/socket.h> 
#include <unistd.h>
#include <sys/time.h> 
#include <arpa/inet.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <netpacket/packet.h>
#include <pthread.h>

#include "ethercattype.h"
#include "nicdrv.h"

/** Redundancy modes */
enum
{
	/** No redundancy, single NIC mode */
	ECT_RED_NONE,
	/** Double redundant NIC connecetion */
	ECT_RED_DOUBLE
};

/** pointer structure to Tx and Rx stacks */
typedef struct
{
	/** socket connection used */
	int			*sock;
	/** tx buffer */
	ec_bufT		(*txbuf)[EC_MAXBUF];
	/** tx buffer lengths */
	int			(*txbuflength)[EC_MAXBUF];
	/** temporary receive buffer */
	ec_bufT		*tempbuf;
	/** rx buffers */
	ec_bufT		(*rxbuf)[EC_MAXBUF];
	/** rx buffer status fields */
	int			(*rxbufstat)[EC_MAXBUF];
	/** received MAC source address (middle word) */
	int			(*rxsa)[EC_MAXBUF];
} ec_stackT;	

/** primary rx buffers */
ec_bufT ec_rxbuf[EC_MAXBUF];
/** primary rx buffer status */
int ec_rxbufstat[EC_MAXBUF];
/** primary rx MAC source address */
static int ec_rxsa[EC_MAXBUF];
/** primary temporary rx buffer */
static ec_bufT ec_tempinbuf;
/** primary temporary rx buffer status */
static int ec_tempinbufs;

/** secondary rx buffers */
static ec_bufT ec_rxbuf2[EC_MAXBUF];
/** secondary rx buffer status */
static int ec_rxbufstat2[EC_MAXBUF];
/** secondary rx MAC source address */
static int ec_rxsa2[EC_MAXBUF];
/** secondary temporary rx buffer */
static ec_bufT ec_tempinbuf2;

/** transmit buffers */
ec_bufT ec_txbuf[EC_MAXBUF];
/** transmit buffer lenghts */
int ec_txbuflength[EC_MAXBUF];
/** temporary tx buffer */
ec_bufT ec_txbuf2;
/** temporary tx buffer length */
int ec_txbuflength2;

/** primary socket handle */
int sockhandle = -1;
/** secondary socket handle */
int sockhandle2 = -1;

/** primary and secondary tx/rx stack pointers */
static ec_stackT ec_stack[2]= 
		{{&sockhandle, &ec_txbuf, &ec_txbuflength, &ec_tempinbuf, &ec_rxbuf, &ec_rxbufstat, &ec_rxsa},
		 {&sockhandle2, &ec_txbuf, &ec_txbuflength, &ec_tempinbuf2, &ec_rxbuf2, &ec_rxbufstat2, &ec_rxsa2}};

/** last used frame index */
static uint8 ec_lastidx;
/** global rx packet counter, counts only EtherCAT packets */
int ec_incnt;
/** global error packet counter, ie non EtherCAT packets */
int ec_errcnt;
/** current redundancy state */
int ec_redstate;

/** global helper var to count time used in tx socket */
int hlp_txtime;
/** global helper var ri count time used in rx socket */
int hlp_rxtime;
/** Primary source MAC address used for EtherCAT.
 * This address is not the MAC address used from the NIC.
 * EtherCAT does not care about MAC addressing, but it is used here to
 * differentiate the route the packet traverses through the EtherCAT
 * segment. This is needed to find out the packet flow in redundant
 * configurations. */
const uint16 priMAC[3] = { 0x0101, 0x0101, 0x0101 };
/** Secondary source MAC address used for EtherCAT. */
const uint16 secMAC[3] = { 0x0404, 0x0404, 0x0404 };

/** second MAC word is used for identification */
#define RX_PRIM priMAC[1]
/** second MAC word is used for identification */
#define RX_SEC secMAC[1]

pthread_mutex_t ec_getindex_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ec_tx_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ec_rx_mutex = PTHREAD_MUTEX_INITIALIZER;

/** Basic setup to connect NIC to socket.
 * @param[in] ifname	    = Name of NIC device, f.e. "eth0"
 * @param[in] secondary		= if >0 then use secondary stack instead of primary
 * @return >0 if succeeded
 */
int ec_setupnic(const char * ifname, int secondary) 
{
	int i;
	int r, rval, ifindex, fl;
	struct timeval timeout;
	struct ifreq ifr;
	struct sockaddr_ll sll;
	int *psock;

	rval = 0;
	if (secondary)
	{
		/* when using secondary socket it is automatically a redundant setup */
		psock = &sockhandle2;
		ec_redstate = ECT_RED_DOUBLE;
	}
	else
	{
		psock = &sockhandle;
		ec_redstate = ECT_RED_NONE;
	}	
	/* we use RAW packet socket, with packet type ETH_P_ECAT */
	*psock = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ECAT));
	timeout.tv_sec =  0;
	timeout.tv_usec = 1;
	 
	r = setsockopt(*psock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	r = setsockopt(*psock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
	i = 1;
	r = setsockopt(*psock, SOL_SOCKET, SO_DONTROUTE, &i, sizeof(i));
	/* connect socket to NIC by name */
	strcpy(ifr.ifr_name, ifname);
	r = ioctl(*psock, SIOCGIFINDEX, &ifr);
	ifindex = ifr.ifr_ifindex;
	strcpy(ifr.ifr_name, ifname);
	ifr.ifr_flags = 0;
	/* reset flags of NIC interface */
	r = ioctl(*psock, SIOCGIFFLAGS, &ifr);
	/* set flags of NIC interface, here promiscuous and broadcast */
	ifr.ifr_flags = ifr.ifr_flags || IFF_PROMISC || IFF_BROADCAST;
	r = ioctl(*psock, SIOCGIFFLAGS, &ifr);
	/* bind socket to protocol, in this case RAW EtherCAT */
	sll.sll_family = AF_PACKET;
	sll.sll_ifindex = ifindex;
	sll.sll_protocol = htons(ETH_P_ECAT);
	r = bind(*psock, (struct sockaddr *)&sll, sizeof(sll));
	/* get flags */
	fl = fcntl(*psock, F_GETFL, 0);
	/* set nodelay option, so make socket non-blocking */
//	r = fcntl(*psock, F_SETFL, fl | O_NDELAY);
	/* setup ethernet headers in tx buffers so we don't have to repeat it */
	for (i = 0; i < EC_MAXBUF; i++) 
	{
		ec_setupheader(&ec_txbuf[i]);
		ec_rxbufstat[i] = EC_BUF_EMPTY;
	}
	ec_setupheader(&ec_txbuf2);
	ec_errcnt = ec_incnt = 0;
	if (r == 0) rval = 1;
	
	return rval;
}

/** Close sockets used
 * @return 0
 */
int ec_closenic(void) 
{
	if (sockhandle >= 0) close(sockhandle);
	if (sockhandle2 >= 0) close(sockhandle2);
	
	return 0;
}

/** Fill buffer with ethernet header structure.
 * Destination MAC is allways broadcast.
 * Ethertype is allways ETH_P_ECAT.
 * @param[out] p = buffer
 */
void ec_setupheader(void *p) 
{
	ec_etherheadert *bp;
	bp = p;
	bp->da0 = htons(0xffff);
	bp->da1 = htons(0xffff);
	bp->da2 = htons(0xffff);
	bp->sa0 = htons(priMAC[0]);
	bp->sa1 = htons(priMAC[1]);
	bp->sa2 = htons(priMAC[2]);
	bp->etype = htons(ETH_P_ECAT);
}

/** Get new frame identifier index and allocate corresponding rx buffer.
 * @return new index.
 */
uint8 ec_getindex(void)
{
	uint8 idx;
	int cnt;

	pthread_mutex_lock( &ec_getindex_mutex );

	idx = ec_lastidx + 1;
	/* index can't be larger than buffer array */
	if (idx >= EC_MAXBUF) 
	{
		idx = 0;
	}
	cnt = 0;
	/* try to find unused index */
	while ((ec_rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_MAXBUF))
	{
		idx++;
		cnt++;
		if (idx >= EC_MAXBUF) 
		{
			idx = 0;
		}
	}
	ec_rxbufstat[idx] = EC_BUF_ALLOC;
	ec_rxbufstat2[idx] = EC_BUF_ALLOC;
	ec_lastidx = idx;

	pthread_mutex_unlock( &ec_getindex_mutex );
	
	return idx;
}

/** Set rx buffer status.
 * @param[in] idx		= index in buffer array
 * @param[in] bufstat   = status to set
 */
void ec_setbufstat(uint8 idx, int bufstat)
{
    ec_rxbufstat[idx] = bufstat;
	ec_rxbufstat2[idx] = bufstat;
}

/** Transmit buffer over socket (non blocking).
 * @param[in] idx			= index in tx buffer array
 * @param[in] stacknumber   = 0=Primary 1=Secondary stack
 * @return socket send result
 */
int ec_outframe(uint8 idx, int stacknumber)
{
	int lp, rval;
	ec_stackT *stack;

	stack = &ec_stack[stacknumber];
	lp = (*stack->txbuflength)[idx];
	rval = send(*stack->sock, (*stack->txbuf)[idx], lp, 0);
	(*stack->rxbufstat)[idx] = EC_BUF_TX;

	return rval;
}

/** Transmit buffer over socket (non blocking).
 * @param[in] idx			= index in tx buffer array
 * @return socket send result
 */
int ec_outframe_red(uint8 idx)
{
	ec_comt *datagramP;
	ec_etherheadert *ehp;
	int rval;

	ehp = (ec_etherheadert *)&ec_txbuf[idx];
	/* rewrite MAC source address 1 to primary */
	ehp->sa1 = htons(priMAC[1]);
	/* transmit over primary socket*/
	rval = ec_outframe(idx, 0);
	if (ec_redstate != ECT_RED_NONE)
	{	
		pthread_mutex_lock( &ec_tx_mutex );
		ehp = (ec_etherheadert *)&ec_txbuf2;
		/* use dummy frame for secondary socket transmit (BRD) */
		datagramP = (ec_comt*)&ec_txbuf2[ETH_HEADERSIZE];
		/* write index to frame */
		datagramP->index = idx;
		/* rewrite MAC source address 1 to secondary */
		ehp->sa1 = htons(secMAC[1]);
		/* transmit over secondary socket */
		send(sockhandle2, &ec_txbuf2, ec_txbuflength2 , 0);
		pthread_mutex_unlock( &ec_tx_mutex );
		ec_rxbufstat2[idx] = EC_BUF_TX;
	}	
	
	return rval;
}

/** Non blocking read of socket. Put frame in temporary buffer.
 * @param[in] stacknumber = 0=primary 1=secondary stack
 * @return >0 if frame is available and read
 */
static int ec_recvpkt(int stacknumber)
{
	int lp, bytesrx;
	ec_stackT *stack;

	stack = &ec_stack[stacknumber];
	lp = sizeof(ec_tempinbuf);
	bytesrx = recv(*stack->sock, (*stack->tempbuf), lp, 0);
	ec_tempinbufs = bytesrx;
	
	return (bytesrx > 0);
}

/** Non blocking receive frame function. Uses RX buffer and index to combine
 * read frame with transmitted frame. To compensate for received frames that
 * are out-of-order all frames are stored in their respective indexed buffer.
 * If a frame was placed in the buffer previously, the function retreives it
 * from that buffer index without calling ec_recvpkt. If the requested index
 * is not already in the buffer it calls ec_recvpkt to fetch it. There are
 * three options now, 1 no frame read, so exit. 2 frame read but other
 * than requested index, store in buffer and exit. 3 frame read with matching
 * index, store in buffer, set completed flag in buffer status and exit.
 * 
 * @param[in] idx			= requested index of frame
 * @param[in] stacknumber  = 0=primary 1=secondary stack
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME or EC_OTHERFRAME.
 */
int ec_inframe(uint8 idx, int stacknumber)
{
	uint16  l;
	int		rval;
	uint8   idxf;
	ec_etherheadert *ehp;
	ec_comt *ecp;
	ec_stackT *stack;
	ec_bufT *rxbuf;

	stack = &ec_stack[stacknumber];
	rval = EC_NOFRAME;
	rxbuf = &(*stack->rxbuf)[idx];
	/* check if requested index is already in buffer ? */
	if ((idx < EC_MAXBUF) && (	(*stack->rxbufstat)[idx] == EC_BUF_RCVD)) 
	{
		l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
		/* return WKC */
		rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
		/* mark as completed */
		(*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
	}
	else 
	{
		pthread_mutex_lock( &ec_rx_mutex );
		/* non blocking call to retrieve frame from socket */
		if (ec_recvpkt( stacknumber)) 
		{
			rval = EC_OTHERFRAME;
			ehp =(ec_etherheadert*)(stack->tempbuf);
			/* check if it is an EtherCAT frame */
			if (ehp->etype == htons(ETH_P_ECAT)) 
			{
				ec_incnt++;
				ecp =(ec_comt*)(&(*stack->tempbuf)[ETH_HEADERSIZE]); 
				l = etohs(ecp->elength) & 0x0fff;
				idxf = ecp->index;
				/* found index equals reqested index ? */
				if (idxf == idx) 
				{
					/* yes, put it in the buffer array (strip ethernet header) */
					memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idx] - ETH_HEADERSIZE);
					/* return WKC */
					rval = ((*rxbuf)[l] + ((uint16)((*rxbuf)[l + 1]) << 8));
					/* mark as completed */
					(*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
					/* store MAC source word 1 for redundant routing info */
					(*stack->rxsa)[idx] = ntohs(ehp->sa1);
				}
				else 
				{
					/* check if index exist? */
					if (idxf < EC_MAXBUF) 
					{
						rxbuf = &(*stack->rxbuf)[idxf];
						/* put it in the buffer array (strip ethernet header) */
						memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idxf] - ETH_HEADERSIZE);
						/* mark as received */
						(*stack->rxbufstat)[idxf] = EC_BUF_RCVD;
						(*stack->rxsa)[idxf] = ntohs(ehp->sa1);
					}
					else 
					{
						/* strange things happend */
						ec_errcnt++;
					}
				}
			}
		}
		pthread_mutex_unlock( &ec_rx_mutex );
		
	}
	
	/* WKC if mathing frame found */
	return rval;
}

/** Blocking redundant receive frame function. If redundant mode is not active then
 * it skips the secondary stack and redundancy functions. In redundant mode it waits
 * for both (primary and secondary) frames to come in. The result goes in an decision
 * tree that decides, depending on the route of the packet and its possible missing arrival,
 * how to reroute the original packet to get the data in an other try. 
 *
 * @param[in] idx = requested index of frame
 * @param[in] tvs = timeout
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME.
 */
static int ec_waitinframe_red(uint8 idx, struct timeval tvs)
{
	struct timeval tv1,tv2;
	int wkc  = EC_NOFRAME;
	int wkc2 = EC_NOFRAME;
	int primrx, secrx;
	
	/* if not in redundat mode then always assume secondary is OK */
	if (ec_redstate == ECT_RED_NONE)
		wkc2 = 0;
	do 
	{
		/* only read frame if not already in */
		if (wkc <= EC_NOFRAME)
			wkc  = ec_inframe(idx, 0);
		/* only try secondary if in redundant mode */
		if (ec_redstate != ECT_RED_NONE)
		{	
			/* only read frame if not already in */
			if (wkc2 <= EC_NOFRAME)
				wkc2 = ec_inframe(idx, 1);
		}	
		gettimeofday(&tv1, 0);
	/* wait for both frames to arrive or timeout */	
	} while (((wkc <= EC_NOFRAME) || (wkc2 <= EC_NOFRAME)) && (timercmp(&tv1, &tvs, <)));
	/* only do redundant functions when in redundant mode */
	if (ec_redstate != ECT_RED_NONE)
	{
		/* primrx if the reveived MAC source on primary socket */
		primrx = 0;
		if (wkc > EC_NOFRAME) primrx = ec_rxsa[idx];
		/* secrx if the reveived MAC source on psecondary socket */
		secrx = 0;
		if (wkc2 > EC_NOFRAME) secrx = ec_rxsa2[idx];
		
		/* primary socket got secondary frame and secondary socket got primary frame */
		/* normal situation in redundant mode */
		if ( ((primrx == RX_SEC) && (secrx == RX_PRIM)) )
		{
			/* copy secondary buffer to primary */
			memcpy(&ec_rxbuf[idx], &ec_rxbuf2[idx], ec_txbuflength[idx] - ETH_HEADERSIZE);
			wkc = wkc2;
		}	
		/* primary socket got nothing or primary frame, and secondary socket got secondary frame */
		/* we need to resend TX packet */ 
		if ( ((primrx == 0) && (secrx == RX_SEC)) ||
			 ((primrx == RX_PRIM) && (secrx == RX_SEC)) )
		{
			/* If both primary and secondary have partial connection retransmit the primary received
			 * frame over the secondary socket. The result from the secondary received frame is a combined
			 * frame that traversed all slaves in standard order. */
			if ( (primrx == RX_PRIM) && (secrx == RX_SEC) )
			{	
				/* copy primary rx to tx buffer */
				memcpy(&ec_txbuf[idx][ETH_HEADERSIZE], &ec_rxbuf[idx], ec_txbuflength[idx] - ETH_HEADERSIZE);
			}
			gettimeofday(&tv1, 0);				
			tv2.tv_sec = 0;
			tv2.tv_usec = EC_TIMEOUTRET;
			timeradd(&tv1, &tv2, &tvs);
			/* resend secondary tx */
			ec_outframe(idx,1);
			do 
			{
				/* retrieve frame */
				wkc2 = ec_inframe(idx, 1);
				gettimeofday(&tv1, 0);
			} while ((wkc2 <= EC_NOFRAME) && (timercmp(&tv1, &tvs, <)));
			if (wkc2 > EC_NOFRAME)
			{	
				/* copy secondary result to primary rx buffer */
				memcpy(&ec_rxbuf[idx], &ec_rxbuf2[idx], ec_txbuflength[idx] - ETH_HEADERSIZE);
				wkc = wkc2;
			}	
		}		
	}
	
	/* return WKC or EC_NOFRAME */
	return wkc;
}	

/** Blocking receive frame function. Calls ec_waitinframe_red().
 * @param[in] idx = requested index of frame
 * @param[in] timeout = timeout in us
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME.
 */
int ec_waitinframe(uint8 idx, int timeout)
{
	int wkc;
	struct timeval tv1, tv2, tve;
	
	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
	wkc = ec_waitinframe_red(idx, tve);
	/* if nothing received, clear buffer index status so it can be used again */
	if (wkc <= EC_NOFRAME) 
	{
		ec_setbufstat(idx, EC_BUF_EMPTY);
	}
	
	return wkc;
}

/** Blocking send and recieve frame function. Used for non processdata frames.
 * A datagram is build into a frame and transmitted via this function. It waits
 * for an answer and returns the workcounter. The function retries if time is
 * left and the result is WKC=0 or no frame received.
 *
 * The function calls ec_outframe_red() and ec_waitinframe_red().
 *
 * @param[in] idx		= index of frame
 * @param[in] timeout  = timeout in us
 * @return Workcounter or EC_NOFRAME
 */
int ec_srconfirm(uint8 idx, int timeout)
{
	int wkc = EC_NOFRAME;
	struct timeval tv1, tv2, tv3, tve, tvs, tvh;

	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
	do 
	{
		/* tx frame on primary and if in redundant mode a dummy on secondary */
		ec_outframe_red(idx);
		gettimeofday(&tv2, 0);
		timersub(&tv2, &tv1, &tvh);
		hlp_txtime += (int)tvh.tv_usec;
		tv1.tv_sec = 0;
		if (timeout < EC_TIMEOUTRET) 
		{
			tv1.tv_usec = timeout; 
		}
		else 
		{
			/* normally use partial timout for rx */
			tv1.tv_usec = EC_TIMEOUTRET;
		}
		timeradd(&tv2, &tv1, &tvs);
		/* get frame from primary or if in redundant mode possibly from secondary */
		wkc = ec_waitinframe_red(idx, tvs);
		gettimeofday(&tv3, 0);
		timersub(&tv3, &tv2, &tvh);
		hlp_rxtime += (int)tvh.tv_usec;		
	/* wait for answer with WKC>0 or otherwise retry until timeout */	
	} while ((wkc <= EC_NOFRAME) && (timercmp(&tv3, &tve, <)));
	/* if nothing received, clear buffer index status so it can be used again */
	if (wkc <= EC_NOFRAME) 
	{
		ec_setbufstat(idx, EC_BUF_EMPTY);
	}
	
	return wkc;
}
