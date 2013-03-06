/*
 * Simple Open EtherCAT Master Library 
 *
 * File    : ethercatfoe.c
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
 *
 * 14-06-2010 : fixed bug in FOEread() by Torsten Bitterlich
 */

/** \file 
 * \brief
 * File over EtherCAT (FoE) module.
 *
 * SDO read / write and SDO service functions
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatfoe.h"

#define EC_MAXFOEDATA 512

/** FOE structure.
 * Used for Read, Write, Data, Ack and Error mailbox packets.
 */
typedef struct PACKED
{
    ec_mbxheadert   MbxHeader;
	uint8			OpCode;
	uint8			Reserved;
	union
	{	
		uint32			Password;
		uint32			PacketNumber;
		uint32			ErrorCode;
	};
	union
	{	
		char			FileName[EC_MAXFOEDATA];
		uint8			Data[EC_MAXFOEDATA];
		char			ErrorText[EC_MAXFOEDATA];
	};	
} ec_FOEt;

/** FoE read, blocking.
 * 
 * @param[in]  slave		= Slave number.
 * @param[in]  filename		= Filename of file to read.
 * @param[in]  password     = password.
 * @param[in,out] psize		= Size in bytes of file buffer, returns bytes read from file.
 * @param[out] p			= Pointer to file buffer
 * @param[in]  timeout		= Timeout in us, standard is EC_TIMEOUTRXM
 * @return Workcounter from last slave response
 */
int ec_FOEread(uint16 slave, char *filename, uint32 password, int *psize, void *p, int timeout)
{
    ec_FOEt *FOEp, *aFOEp;
	int wkc;
    int32 dataread = 0;
	int32 buffersize, packetnumber, prevpacket = 0;
	uint16 fnsize, maxdata, segmentdata;
    ec_mbxbuft MbxIn, MbxOut;
    uint8 cnt;
    boolean worktodo;

	buffersize = *psize;
    ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timout set to 0 */
    wkc = ec_mbxreceive(slave, (ec_mbxbuft *)&MbxIn, 0);
    ec_clearmbx(&MbxOut);
    aFOEp = (ec_FOEt *)&MbxIn;
    FOEp = (ec_FOEt *)&MbxOut;
	fnsize = strlen(filename);
	maxdata = ec_slave[slave].mbx_l - 12;
	if (fnsize > maxdata)
		fnsize = maxdata;
    FOEp->MbxHeader.length = htoes(0x0006 + fnsize);
    FOEp->MbxHeader.address = htoes(0x0000);
    FOEp->MbxHeader.priority = 0x00;
	/* get new mailbox count value, used as session handle */
    cnt = ec_nextmbxcnt(ec_slave[slave].mbx_cnt);
    ec_slave[slave].mbx_cnt = cnt;
    FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + (cnt << 4); /* FoE */
    FOEp->OpCode = ECT_FOE_READ;
    FOEp->Password = htoel(password);
	/* copy filename in mailbox */
	memcpy(&FOEp->FileName[0], filename, fnsize);
	/* send FoE request to slave */
    wkc = ec_mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
    if (wkc > 0) /* succeeded to place mailbox in slave ? */
    {
		do
		{	
			worktodo = FALSE;
			/* clean mailboxbuffer */
			ec_clearmbx(&MbxIn);
			/* read slave response */
			wkc = ec_mbxreceive(slave, (ec_mbxbuft *)&MbxIn, timeout);
			if (wkc > 0) /* succeeded to read slave response ? */
			{
				/* slave response should be FoE */
				if ((aFOEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_FOE)
				{
					if(aFOEp->OpCode == ECT_FOE_DATA)
					{
						segmentdata = etohs(aFOEp->MbxHeader.length) - 0x0006;
						packetnumber = etohl(aFOEp->PacketNumber);
						if ((packetnumber == ++prevpacket) && (dataread + segmentdata <= buffersize))
						{
							memcpy(p, &aFOEp->Data[0], segmentdata);
							dataread += segmentdata;
							p += segmentdata;
							if (segmentdata == maxdata)
								worktodo = TRUE; 
							FOEp->MbxHeader.length = htoes(0x0006);
							FOEp->MbxHeader.address = htoes(0x0000);
							FOEp->MbxHeader.priority = 0x00;
							/* get new mailbox count value */
							cnt = ec_nextmbxcnt(ec_slave[slave].mbx_cnt);
							ec_slave[slave].mbx_cnt = cnt;
							FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + (cnt << 4); /* FoE */
							FOEp->OpCode = ECT_FOE_ACK;
							FOEp->PacketNumber = htoel(packetnumber);
							/* send FoE ack to slave */
							wkc = ec_mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
							if (wkc <= 0)
								worktodo = FALSE;
						}
						else
						{
							/* FoE error */
							wkc = -EC_ERR_TYPE_FOE_BUF2SMALL;
						}
					}
					else
					if(aFOEp->OpCode == ECT_FOE_ERROR)
					{
						/* FoE error */
						wkc = -EC_ERR_TYPE_FOE_ERROR;
					}
					else
					{
						/* unexpected mailbox received */
						wkc = -EC_ERR_TYPE_PACKET_ERROR;
					}
				}
				else
				{
					/* unexpected mailbox received */
					wkc = -EC_ERR_TYPE_PACKET_ERROR;
				}
				*psize = dataread;
			}
		} while (worktodo);	
	}
	
	return wkc;
}	

/** FoE write, blocking.
 * 
 * @param[in]  slave		= Slave number.
 * @param[in]  filename		= Filename of file to write.
 * @param[in]  password     = password.
 * @param[in]  psize		= Size in bytes of file buffer.
 * @param[out] p			= Pointer to file buffer
 * @param[in]  timeout		= Timeout in us, standard is EC_TIMEOUTRXM
 * @return Workcounter from last slave response
 */
int ec_FOEwrite(uint16 slave, char *filename, uint32 password, int psize, void *p, int timeout)
{
    ec_FOEt *FOEp, *aFOEp;
	int wkc;
	int32 packetnumber, sendpacket = 0;
	uint16 fnsize, maxdata;
	int segmentdata;
    ec_mbxbuft MbxIn, MbxOut;
    uint8 cnt;
    boolean worktodo;
	int tsize;

    ec_clearmbx(&MbxIn);
	/* Empty slave out mailbox if something is in. Timout set to 0 */
    wkc = ec_mbxreceive(slave, (ec_mbxbuft *)&MbxIn, 0);
    ec_clearmbx(&MbxOut);
    aFOEp = (ec_FOEt *)&MbxIn;
    FOEp = (ec_FOEt *)&MbxOut;
	fnsize = strlen(filename);
	maxdata = ec_slave[slave].mbx_l - 12;
	if (fnsize > maxdata)
		fnsize = maxdata;
    FOEp->MbxHeader.length = htoes(0x0006 + fnsize);
    FOEp->MbxHeader.address = htoes(0x0000);
    FOEp->MbxHeader.priority = 0x00;
	/* get new mailbox count value, used as session handle */
    cnt = ec_nextmbxcnt(ec_slave[slave].mbx_cnt);
    ec_slave[slave].mbx_cnt = cnt;
    FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + (cnt << 4); /* FoE */
    FOEp->OpCode = ECT_FOE_WRITE;
    FOEp->Password = htoel(password);
	/* copy filename in mailbox */
	memcpy(&FOEp->FileName[0], filename, fnsize);
	/* send FoE request to slave */
    wkc = ec_mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
    if (wkc > 0) /* succeeded to place mailbox in slave ? */
    {
		do
		{	
			worktodo = FALSE;
			/* clean mailboxbuffer */
			ec_clearmbx(&MbxIn);
			/* read slave response */
			wkc = ec_mbxreceive(slave, (ec_mbxbuft *)&MbxIn, timeout);
			if (wkc > 0) /* succeeded to read slave response ? */
			{
				/* slave response should be FoE */
				if ((aFOEp->MbxHeader.mbxtype & 0x0f) == ECT_MBXT_FOE)
				{
					if(aFOEp->OpCode == ECT_FOE_ACK)
					{
						packetnumber = etohl(aFOEp->PacketNumber);
						if (packetnumber == sendpacket)
						{
							tsize = psize;
							if (tsize > maxdata)
							{
								worktodo = TRUE; 
								tsize = maxdata;
							}	
							segmentdata = tsize;
							psize -= segmentdata;
							FOEp->MbxHeader.length = htoes(0x0006 + segmentdata);
							FOEp->MbxHeader.address = htoes(0x0000);
							FOEp->MbxHeader.priority = 0x00;
							/* get new mailbox count value */
							cnt = ec_nextmbxcnt(ec_slave[slave].mbx_cnt);
							ec_slave[slave].mbx_cnt = cnt;
							FOEp->MbxHeader.mbxtype = ECT_MBXT_FOE + (cnt << 4); /* FoE */
							FOEp->OpCode = ECT_FOE_DATA;
							FOEp->PacketNumber = htoel(++sendpacket);
							memcpy(&FOEp->Data[0], p, segmentdata);
							p += segmentdata;
							/* send FoE data to slave */
							wkc = ec_mbxsend(slave, (ec_mbxbuft *)&MbxOut, EC_TIMEOUTTXM);
							if (wkc <= 0)
								worktodo = FALSE;
						}
						else
						{
							/* FoE error */
							wkc = -EC_ERR_TYPE_FOE_PACKETNUMBER;
						}
					}
					else
					if(aFOEp->OpCode == ECT_FOE_ERROR)
					{
						/* FoE error */
						wkc = -EC_ERR_TYPE_FOE_ERROR;
					}
					else
					{
						/* unexpected mailbox received */
						wkc = -EC_ERR_TYPE_PACKET_ERROR;
					}
				}
				else
				{
					/* unexpected mailbox received */
					wkc = -EC_ERR_TYPE_PACKET_ERROR;
				}
			}
		} while (worktodo);	
	}
	
	return wkc;
}	