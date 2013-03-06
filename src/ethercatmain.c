/*
 * Simple Open EtherCAT Master Library 
 *
 * File    : ethercatmain.c
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

/**
 * \file
 * \brief
 * Main EtherCAT functions.
 *
 * Initialisation, state set and read, mailbox primitives, EEPROM primitives,
 * SII reading and processdata exchange.
 *
 * Defines ec_slave[]. All slave information is put in this structure. 
 * Needed for most user interaction with slaves.
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"

/** delay in us for eeprom ready loop */
#define EC_LOCALDELAY   200

/** stack structure to store segmented LRD/LWR/LRW constructs */
typedef struct
{
	uint8   pushed;
	uint8   pulled;
	uint8   idx[EC_MAXBUF];
	void	*data[EC_MAXBUF];
	uint16  length[EC_MAXBUF];
} ec_idxstackT;

/** record for ethercat eeprom communications */
typedef struct PACKED
{
    uint16  comm;
    uint16  addr;
    uint16  d2;
} ec_eepromt;

/** ringbuf for error storage */
typedef struct 
{
    int16       head;
    int16       tail;
    ec_errort   Error[EC_MAXELIST + 1];
} ec_eringt;

/** emergency request structure */
typedef struct
{
    ec_mbxheadert   MbxHeader;
    uint16          CANOpen;
    uint16          ErrorCode;
    uint8           ErrorReg;
    uint8           bData;
    uint16          w1,w2;
} ec_emcyt;

/** Main slave data array.
 *  Each slave found on the network gets its own record.
 *  ec_slave[0] is reserved for the master. Structure gets filled
 *  in by the configuration function ec_config().
 */
ec_slavet	ec_slave[EC_MAXSLAVE];
/** number of slaves found on the network */ 
int			ec_slavecount;
/** slave group structure */
ec_groupt	ec_group[EC_MAXGROUP];

/** cache for EEPROM read functions */
static uint8		esibuf[EC_MAXEEPBUF];
/** bitmap for filled cache buffer bytes */
static uint32		esimap[EC_MAXEEPBITMAP];
/** current slave for EEPROM cache buffer */
static uint16		esislave=0;
static ec_eringt	ec_elist;
static ec_idxstackT ec_idxstack;

/** Global variable TRUE if error available in error stack */
boolean		EcatError = FALSE;

uint16 ec_DCtO;
static uint16 ec_DCl;
int64 ec_DCtime;

/** Pushes an error on the error list.
 *
 * @param[in] Ec	   Struct describing the error.
 */
void ec_pusherror(const ec_errort *Ec)
{
    ec_elist.Error[ec_elist.head] = *Ec;
    ec_elist.Error[ec_elist.head].Signal = TRUE;
    ec_elist.head++;
    if (ec_elist.head > EC_MAXELIST)
        ec_elist.head = 0;
    if (ec_elist.head == ec_elist.tail)
        ec_elist.tail++;
    if (ec_elist.tail > EC_MAXELIST)
        ec_elist.tail = 0;
	EcatError = TRUE;
}

/** Pops an error from the list.
 *
 * @param[out] Ec = Struct describing the error.
 * @return TRUE if an error was popped.
 */
boolean ec_poperror(ec_errort *Ec)
{
    boolean notEmpty = (ec_elist.head != ec_elist.tail);

    *Ec = ec_elist.Error[ec_elist.tail];
    ec_elist.Error[ec_elist.tail].Signal = FALSE;
    if (notEmpty)
    {
        ec_elist.tail++;
        if (ec_elist.tail > EC_MAXELIST)
            ec_elist.tail = 0;
    }
	else EcatError = FALSE;
    return notEmpty;
}

/** Check if error list has entries.
 *
 * @return TRUE if error list contains entries.
 */
boolean ec_iserror(void)
{
	return (ec_elist.head != ec_elist.tail);
}

/** Report packet error
 *
 * @param[in]  Slave		= Slave number
 * @param[in]  Index		= Index that generated error
 * @param[in]  SubIdx	    = Subindex that generated error
 * @param[in]  ErrorCode	= Error code
 */
void ec_packeterror(uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode)
{
    ec_errort Ec;

	gettimeofday(&Ec.Time, 0);
    Ec.Slave = Slave;
    Ec.Index = Index;
    Ec.SubIdx = SubIdx;
    EcatError = TRUE;
    Ec.Etype = EC_ERR_TYPE_PACKET_ERROR;
    Ec.ErrorCode = ErrorCode;
    ec_pusherror(&Ec);
}

/** Report Mailbox Emergency Error
 *
 * @param[in]  Slave		= Slave number
 * @param[in]  ErrorCode    = Following EtherCAT specification
 * @param[in]  ErrorReg
 * @param[in]  b1
 * @param[in]  w1
 * @param[in]  w2
 */
static void ec_mbxemergencyerror(uint16 Slave,uint16 ErrorCode,uint16 ErrorReg,
    uint8 b1, uint16 w1, uint16 w2)
{
    ec_errort Ec;

	gettimeofday(&Ec.Time, 0);
    Ec.Slave = Slave;
    Ec.Index = 0;
    Ec.SubIdx = 0;
    Ec.Etype = EC_ERR_TYPE_EMERGENCY;
    Ec.ErrorCode = ErrorCode;
    Ec.ErrorReg = (uint8)ErrorReg;
    Ec.b1 = b1;
    Ec.w1 = w1;
    Ec.w2 = w2;
    ec_pusherror(&Ec);
}

/** Initialise lib in single NIC mode
 * @param[in] ifname   = Dev name, f.e. "eth0"
 * @return >0 if OK
 */
int ec_init(const char * ifname)
{
	return ec_setupnic(ifname, FALSE);
}	

/** Initialise lib in redundant NIC mode
 * @param[in] ifname	= Primary Dev name, f.e. "eth0"
 * @param[in] if2name	= Secondary Dev name, f.e. "eth1"
 * @return >0 if OK
 */
int ec_init_redundant(const char *ifname, const char *if2name)
{
	int rval, zbuf;
	ec_etherheadert *ehp;
	
	ec_setupnic(ifname, FALSE);
	rval = ec_setupnic(if2name, TRUE);
	/* prepare "dummy" BRD tx frame for redundant operation */
	ehp = (ec_etherheadert *)&ec_txbuf2;
	ehp->sa1 = htons(secMAC[0]);
	zbuf = 0;
	ec_setupdatagram(&ec_txbuf2, EC_CMD_BRD, 0, 0x0000, 0x0000, 2, &zbuf);
	ec_txbuflength2 = ETH_HEADERSIZE + EC_HEADERSIZE + EC_WKCSIZE + 2;
	
	return rval;
}

/** Close lib.
 */
void ec_close(void)
{
	ec_closenic();
};	

/** Read one byte from slave EEPROM via cache.
 *  If the cache location is empty then a read request is made to the slave.
 *  Depending on the slave capabillities the request is 4 or 8 bytes.
 *  @param[in] slave   = slave number
 *  @param[in] address = eeprom address in bytes (slave uses words)
 *  @return requested byte, if not available then 0xff
 */
uint8 ec_siigetbyte(uint16 slave, uint16 address)
{
	uint16 configadr, eadr;
	uint64 edat;
	uint16 mapw, mapb;
	int lp,cnt;
	uint8 retval;
	
	retval = 0xff;
	if (slave != esislave) /* not the same slave? */
	{	
		memset(esimap,0x00,EC_MAXEEPBITMAP); /* clear esibuf cache map */
		esislave=slave;
	}	
	if (address < EC_MAXEEPBUF)
	{		
		mapw = address >> 5;
		mapb = address - (mapw << 5);
		if (esimap[mapw] & (uint32)(1 << mapb))
		{
			/* byte is already in buffer */
			retval = esibuf[address];
		}
		else
		{
			/* byte is not in buffer, put it there */
		    configadr = ec_slave[slave].configadr;
			ec_eeprom2master(slave); /* set eeprom control to master */
			eadr = address >> 1;
			edat = ec_readeepromFP (configadr, eadr, EC_TIMEOUTEEP);
			/* 8 byte response */
			if (ec_slave[slave].eep_8byte)
			{	
				put_unaligned64(edat, &esibuf[eadr << 1]);
				cnt = 8;
			}
			/* 4 byte response */
			else
			{
				put_unaligned32(edat, &esibuf[eadr << 1]);
				cnt = 4;
			}
			/* find bitmap location */
			mapw = eadr >> 4;
			mapb = (eadr << 1) - (mapw << 5);
			for(lp = 0 ; lp < cnt ; lp++)
			{
				/* set bitmap for each byte that is read */
				esimap[mapw] |= (1 << mapb);
				mapb++;
				if (mapb > 31)
				{
					mapb = 0;
					mapw++;
				}	
			}	
			retval = esibuf[address];
		}	
	}	
	
	return retval;
}	

/** Find SII section header in slave EEPROM.
 *  @param[in] slave   = slave number
 *  @param[in] cat	   = section category
 *  @return byte address of section at section length entry, if not available then 0
 */
int16 ec_siifind(uint16 slave, uint16 cat)
{
    int16 a;
	uint16 p;
	uint8 eectl = ec_slave[slave].eep_pdi;

    a = ECT_SII_START << 1;
	/* read first SII section category */
	p = ec_siigetbyte(slave, a++);
	p += (ec_siigetbyte(slave, a++) << 8);
	/* traverse SII while category is not found and not EOF */
    while ((p != cat) && (p != 0xffff))
    {
		/* read section length */
		p = ec_siigetbyte(slave, a++);
		p += (ec_siigetbyte(slave, a++) << 8);
		/* locate next section category */
        a += p << 1;
		/* read section category */
		p = ec_siigetbyte(slave, a++);
		p += (ec_siigetbyte(slave, a++) << 8);
    }
    if (p != cat)
        a = 0;

	if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */

	return a;
}

/** Get string from SII string section in slave EEPROM.
 *  @param[out] str	= requested string, 0x00 if not found
 *  @param[in] slave   = slave number
 *  @param[in] Sn	   = string number
 */
void ec_siistring(char *str, uint16 slave, uint16 Sn)
{
    uint16 a,i,j,l,n,ba,p;
    char *ptr;
	uint8 eectl = ec_slave[slave].eep_pdi;

    ptr = str;
    p = 0;
    a = ec_siifind (slave, ECT_SII_STRING); /* find string section */
    if (a > 0)
    {
        ba = a + 2; /* skip SII section header */
        n = ec_siigetbyte(slave, ba++); /* read number of strings in section */
        if (Sn <= n) /* is req string available? */
        {
            for (i = 1; i <= Sn; i++) /* walk through strings */
            {
                l = ec_siigetbyte(slave, ba++); /* length of this string */
                ptr = str;
                for (j = 1; j <= l; j++) /* copy one string */
                {
                    *ptr = (char)ec_siigetbyte(slave, ba++);
					ptr++;
                }
            }
            *ptr = 0; /* add zero terminator */
        }
		else
		{
			ptr = str;
			*ptr = 0; /* empty string */
		}	
    }
	if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
}

/** Get FMMU data from SII FMMU section in slave EEPROM.
 *  @param[in] slave   = slave number
 *  @param[out] FMMU   = FMMU struct from SII, max. 4 FMMU's
 *  @return number of FMMU's defined in section
 */
uint16 ec_siiFMMU(uint16 slave, ec_eepromFMMUt* FMMU)
{
	uint16  a;
	uint8 eectl = ec_slave[slave].eep_pdi;
	
    FMMU->nFMMU = 0;
    FMMU->FMMU0 = 0;
    FMMU->FMMU1 = 0;
    FMMU->FMMU2 = 0;
    FMMU->FMMU3 = 0;
    FMMU->Startpos = ec_siifind(slave, ECT_SII_FMMU);

    if (FMMU->Startpos > 0)
    {
		a = FMMU->Startpos;
        FMMU->nFMMU = ec_siigetbyte(slave, a++);
		FMMU->nFMMU += (ec_siigetbyte(slave, a++) << 8);
		FMMU->nFMMU *= 2;
        FMMU->FMMU0 = ec_siigetbyte(slave, a++);
        FMMU->FMMU1 = ec_siigetbyte(slave, a++);
        if (FMMU->nFMMU > 2)
        {
            FMMU->FMMU2 = ec_siigetbyte(slave, a++);
            FMMU->FMMU3 = ec_siigetbyte(slave, a++);
        }
    }
	if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return FMMU->nFMMU;
}

/** Get SM data from SII SM section in slave EEPROM.
 *  @param[in] slave   = slave number
 *  @param[out] SM	   = first SM struct from SII
 *  @return number of SM's defined in section
 */
uint16 ec_siiSM(uint16 slave, ec_eepromSMt* SM)
{
    uint16 a,w,l;
	uint8 eectl = ec_slave[slave].eep_pdi;

    SM->nSM = 0;
    l = 0;
    SM->Startpos = ec_siifind(slave, ECT_SII_SM);
    if (SM->Startpos > 0)
    {
		a = SM->Startpos;		
		w = ec_siigetbyte(slave, a++);
		w += (ec_siigetbyte(slave, a++) << 8);
        SM->nSM = (w / 4);
        SM->PhStart = ec_siigetbyte(slave, a++);
		SM->PhStart += (ec_siigetbyte(slave, a++) << 8);
        SM->Plength = ec_siigetbyte(slave, a++);
		SM->Plength += (ec_siigetbyte(slave, a++) << 8);
        SM->Creg = ec_siigetbyte(slave, a++);
        SM->Sreg = ec_siigetbyte(slave, a++);
        SM->Activate = ec_siigetbyte(slave, a++);
        SM->PDIctrl = ec_siigetbyte(slave, a++);
    }
	if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return SM->nSM;
}

/** Get next SM data from SII SM section in slave EEPROM.
 *  @param[in] slave   = slave number
 *  @param[out] SM	   = first SM struct from SII
 *  @param[in] n	   = SM number
 *  @return >0 if OK
 */
uint16 ec_siiSMnext(uint16 slave, ec_eepromSMt* SM, uint16 n)
{
    uint16 a;
    uint16 retVal = 0;
	uint8 eectl = ec_slave[slave].eep_pdi;

    if (n < SM->nSM)
    {
        a = SM->Startpos + 2 + (n * 8);
        SM->PhStart = ec_siigetbyte(slave, a++);
		SM->PhStart += (ec_siigetbyte(slave, a++) << 8);
        SM->Plength = ec_siigetbyte(slave, a++);
		SM->Plength += (ec_siigetbyte(slave, a++) << 8);
        SM->Creg = ec_siigetbyte(slave, a++);
        SM->Sreg = ec_siigetbyte(slave, a++);
        SM->Activate = ec_siigetbyte(slave, a++);
        SM->PDIctrl = ec_siigetbyte(slave, a++);
        retVal = 1;
    }
	if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return retVal;
}

/** Get PDO data from SII PDO section in slave EEPROM.
 *  @param[in] slave	= slave number
 *  @param[out] PDO		= PDO struct from SII
 *  @param[in] t		= 0=RXPDO 1=TXPDO
 *  @return mapping size in bits of PDO
 */
int ec_siiPDO(uint16 slave, ec_eepromPDOt* PDO, uint8 t)
{
    uint16 a , w, c, e, er, Size;
	uint8 eectl = ec_slave[slave].eep_pdi;

	Size = 0;
    PDO->nPDO = 0;
    PDO->Length = 0;
    PDO->Index[1] = 0;
	for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;
    if (t > 1)
        t = 1;
    PDO->Startpos = ec_siifind(slave, ECT_SII_PDO + t);
    if (PDO->Startpos > 0)
    {
		a = PDO->Startpos;
		w = ec_siigetbyte(slave, a++);
		w += (ec_siigetbyte(slave, a++) << 8);
        PDO->Length = w;
        c = 1;
		/* traverse through all PDOs */
        do
        {
            PDO->nPDO++;
            PDO->Index[PDO->nPDO] = ec_siigetbyte(slave, a++);
			PDO->Index[PDO->nPDO] += (ec_siigetbyte(slave, a++) << 8);
            PDO->BitSize[PDO->nPDO] = 0;
            c++;
            e = ec_siigetbyte(slave, a++);
            PDO->SyncM[PDO->nPDO] = ec_siigetbyte(slave, a++);
			a += 4;
            c += 2;
			if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
			{	
				/* read all entries defined in PDO */
	            for (er = 1; er <= e; er++)
		        {
    		        c += 4;
					a += 5;
	                PDO->BitSize[PDO->nPDO] += ec_siigetbyte(slave, a++);
					a += 2;
		        }
				PDO->SMbitsize[ PDO->SyncM[PDO->nPDO] ] += PDO->BitSize[PDO->nPDO];
				Size += PDO->BitSize[PDO->nPDO];
    		    c++;
			}
			else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
			{
				c += 4 * e;
				a += 8 * e;
				c++;
			}	
			if (PDO->nPDO >= (EC_MAXEEPDO - 1)) c = PDO->Length; /* limit number of PDO entries in buffer */
        }
        while (c < PDO->Length);
    }
	if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */

    return (Size);
}
/** Read all slave states in ec_slave. 
 * @return lowest state found
 */
int ec_readstate(void)
{
    uint16 slave, configadr, lowest, rval;
	ec_alstatust slstat;

	lowest = 0xff;
	ec_slave[0].ALstatuscode = 0;
    for (slave = 1; slave <= ec_slavecount; slave++)
    {
        configadr = ec_slave[slave].configadr;
		slstat.alstatus = 0;
		slstat.alstatuscode = 0;
        ec_FPRD(configadr, ECT_REG_ALSTAT, sizeof(slstat), &slstat, EC_TIMEOUTRET);
		rval = etohs(slstat.alstatus);
		ec_slave[slave].ALstatuscode = etohs(slstat.alstatuscode);
		if (rval < lowest)
			lowest = rval;
		ec_slave[slave].state = rval;
		ec_slave[0].ALstatuscode |= ec_slave[slave].ALstatuscode;
    }
	ec_slave[0].state = lowest;

    return lowest;
}

/** Write slave state, if slave = 0 then write to all slaves.
 * The function does not check if the actual state is changed.
 * @param[in] slave	= Slave number, 0 = master
 * @return 0
 */
int ec_writestate(uint16 slave)
{
    uint16 configadr, slstate;

	if (slave == 0)
	{
		slstate = htoes(ec_slave[slave].state);
		ec_BWR(0, ECT_REG_ALCTL, sizeof(slstate), &slstate, EC_TIMEOUTRET); /* write slave status */
	}
	else
	{
		configadr = ec_slave[slave].configadr;
		ec_FPWRw(configadr, ECT_REG_ALCTL, htoes(ec_slave[slave].state), EC_TIMEOUTRET); /* write slave status */
	}
	return 0;
}

/** Check actual slave state.
 * This is a blocking function.
 * @param[in] slave		= Slave number, 0 = all slaves
 * @param[in] reqstate	= Requested state
 * @param[in] timeout	= Timout value in us
 * @return Requested state, or found state after timeout.
 */
uint16 ec_statecheck(uint16 slave, uint16 reqstate, int timeout)
{
    uint16 configadr, state, rval;
	struct timeval tv1, tv2, tve;
	ec_alstatust slstat;
	
	if ( slave > ec_slavecount ) return 0;
	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
    configadr = ec_slave[slave].configadr;
    do
    {
		if (slave < 1)
		{
			rval = 0;
	        ec_BRD(0, ECT_REG_ALSTAT, sizeof(rval), &rval , EC_TIMEOUTRET);
			rval = etohs(rval);
		}
		else
		{	
			slstat.alstatus = 0;
			slstat.alstatuscode = 0;
	        ec_FPRD(configadr, ECT_REG_ALSTAT, sizeof(slstat), &slstat, EC_TIMEOUTRET);
			rval = etohs(slstat.alstatus);
			ec_slave[slave].ALstatuscode = etohs(slstat.alstatuscode);
		}
		state = rval & 0x000f; /* read slave status */
        if (state != reqstate) usleep(1000);
		gettimeofday(&tv2, 0);
    }
	while ((state != reqstate) && timercmp(&tv2, &tve, <));
    ec_slave[slave].state = rval;

	return state;
}

/** Get index of next mailbox counter value.
 * Used for Mailbox Link Layer.
 * @param[in] cnt	  = Mailbox counter value [0..7]
 * @return next mailbox counter value
 */
uint8 ec_nextmbxcnt(uint8 cnt)
{
    cnt++;
    if (cnt > 7)
        cnt = 1; /* wrap around to 1, not 0 */
    return cnt;
}

/** Clear mailbox buffer.
 * @param[out] Mbx	  = Mailbox buffer to clear
 */
void ec_clearmbx(ec_mbxbuft *Mbx)
{
    memset(*Mbx, 0x00, EC_MAXMBX);
}

/** Check if IN mailbox of slave is empty.
 * @param[in] slave		= Slave number
 * @param[in] timeout	= Timeout in us
 * @return >0 is success
 */
int ec_mbxempty(uint16 slave, int timeout)
{
    uint16 configadr;
    uint8 SMstat;
	int wkc;
	struct timeval tv1, tv2, tve;
	
	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
    configadr = ec_slave[slave].configadr;
		do
		{			
		    wkc = ec_FPRD(configadr, ECT_REG_SM0STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
			SMstat = etohs(SMstat);
			if (((SMstat & 0x08) != 0) && (timeout > EC_LOCALDELAY)) usleep(EC_LOCALDELAY);
			gettimeofday(&tv2, 0);
		}	
		while (((wkc <= 0) || ((SMstat & 0x08) != 0)) && timercmp(&tv2, &tve, <));
	if ((wkc > 0) && ((SMstat & 0x08) == 0)) return 1;
	return 0;
}
	
/** Write IN mailbox to slave.
 * @param[in] slave		= Slave number
 * @param[out] mbx      = Mailbox data
 * @param[in] timeout	= Timeout in us
 * @return Work counter (>0 is success)
 */
int ec_mbxsend(uint16 slave,ec_mbxbuft *mbx, int timeout)
{
	uint16 mbxwo,mbxl,configadr;
	int wkc;

	wkc = 0;
	configadr = ec_slave[slave].configadr;
	mbxl = ec_slave[slave].mbx_l;
	if (mbxl > 0)
	{
		if (ec_mbxempty(slave, timeout))
		{
			mbxwo = ec_slave[slave].mbx_wo;
			/* write slave in mailbox */
			wkc = ec_FPWR(configadr, mbxwo, mbxl, mbx, EC_TIMEOUTRET);  
		}
		else 
			wkc = 0;
	}

	return wkc;
}
/** Read OUT mailbox from slave.
 * Supports Mailbox Link Layer with repeat requests.
 * @param[in] slave		= Slave number
 * @param[out] mbx      = Mailbox data
 * @param[in] timeout	= Timeout in us
 * @return Work counter (>0 is success)
 */
int ec_mbxreceive(uint16 slave, ec_mbxbuft *mbx, int timeout)
{
    uint16 mbxro,mbxl,configadr;
	int wkc=0;
	int wkc2;
    uint16 SMstat;
	uint8 SMcontr;
	ec_mbxheadert *mbxh;
    ec_emcyt *EMp;
	struct timeval mtv1, mtv2, mtve;
	
	configadr = ec_slave[slave].configadr;
	mbxl = ec_slave[slave].mbx_rl;
    if (mbxl > 0)
    {
		gettimeofday(&mtv1, 0);
		mtv2.tv_sec = 0;
		mtv2.tv_usec = timeout;
		timeradd(&mtv1, &mtv2, &mtve);
	    wkc = 0;
	    do /* wait for read mailbox available */
		{
		    wkc = ec_FPRD(configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
			SMstat = etohs(SMstat);
			if (((SMstat & 0x08) == 0) && (timeout > EC_LOCALDELAY))
				usleep(EC_LOCALDELAY);
			gettimeofday(&mtv2, 0);
	    }
		while (((wkc <= 0) || ((SMstat & 0x08) == 0)) && timercmp(&mtv2, &mtve, <));

	    if ((wkc > 0) && ((SMstat & 0x08) > 0)) /* read mailbox available ? */
	    {
		    mbxro = ec_slave[slave].mbx_ro;
			mbxh = (ec_mbxheadert *)mbx;
			do
			{	
	            wkc = ec_FPRD(configadr, mbxro, mbxl, mbx, EC_TIMEOUTRET); /* get mailbox */
/* TODO : check for mailbox error response */				
		        if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == 0x03)) /* CoE response? */
   			    {
					EMp = (ec_emcyt *)mbx;
	                if ((etohs(EMp->CANOpen) >> 12) == 0x01) /* Emergency request? */
		            {
    		            ec_mbxemergencyerror(slave, etohs(EMp->ErrorCode), EMp->ErrorReg, 
        		            EMp->bData, etohs(EMp->w1), etohs(EMp->w2));
						wkc = 0; /* prevent emergency to cascade up, it is already handled. */
	                }
		        }
				else
				{  
					if (wkc <= 0) /* read mailbox lost */
					{
						SMstat ^= 0x0200; /* toggle repeat request */
						SMstat = htoes(SMstat);
						wkc2 = ec_FPWR(configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
						SMstat = etohs(SMstat);
					    do /* wait for toggle ack */
						{
						    wkc2 = ec_FPRD(configadr, ECT_REG_SM1CONTR, sizeof(SMcontr), &SMcontr, EC_TIMEOUTRET);
							gettimeofday(&mtv2, 0);
					    } while (((wkc2 <= 0) || ((SMcontr & 0x02) != (HI_BYTE(SMstat) & 0x02))) && timercmp(&mtv2, &mtve, <));
					    do /* wait for read mailbox available */
						{
						    wkc2 = ec_FPRD(configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
							SMstat = etohs(SMstat);
							if (((SMstat & 0x08) == 0) && (timeout > EC_LOCALDELAY)) 
							{
								usleep(EC_LOCALDELAY);
							}
							gettimeofday(&mtv2, 0);
					    } while (((wkc2 <= 0) || ((SMstat & 0x08) == 0)) && timercmp(&mtv2, &mtve, <));
					}	
				}
			} while ((wkc <= 0) && timercmp(&mtv2, &mtve, <)); /* if WKC<=0 repeat */
	    }
		else /* no read mailbox available */
    		wkc = 0;
	}
	
    return wkc;
}

/** Dump complete EEPROM data from slave in buffer.
 * @param[in] slave		= Slave number
 * @param[out] esibuf   = EEPROM data buffer, make sure it is big enough.
 */
void ec_esidump(uint16 slave, uint8 *esibuf, uint8 test)
{
	int address, incr;
	uint16 configadr;
	uint64 *p64;
	uint16 *p16;
	uint64 edat;
	uint8 eectl = ec_slave[slave].eep_pdi;

	ec_eeprom2master(slave); /* set eeprom control to master */
    configadr = ec_slave[slave].configadr;
	address = ECT_SII_START;
	p16=(uint16*)esibuf;
	if (ec_slave[slave].eep_8byte) 
		incr = 4; 
	else 
		incr = 2;
	do
	{
		edat = ec_readeepromFP(configadr, address, EC_TIMEOUTEEP);
		p64 = (uint64*)p16;
		*p64 = edat;
		p16 += incr;
		address += incr; 
	} while ((address <= (EC_MAXEEPBUF >> 1)) && ((uint32)edat != 0xffffffff));

	if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
}

/** Read EEPROM from slave bypassing cache.
 * @param[in] slave		= Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] timeout	= Timeout in us.
 * @return EEPROM data 32bit
 */
uint32 ec_readeeprom(uint16 slave, uint16 eeproma, int timeout)
{
    uint16 configadr;

	ec_eeprom2master(slave); /* set eeprom control to master */
    configadr = ec_slave[slave].configadr;
    return (ec_readeepromFP(configadr, eeproma, timeout));
}

/** Write EEPROM to slave bypassing cache.
 * @param[in] slave		= Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data		= 16bit data
 * @param[in] timeout	= Timeout in us.
 * @return >0 if OK
 */
int ec_writeeeprom(uint16 slave, uint16 eeproma, uint16 data, int timeout)
{
    uint16 configadr;

	ec_eeprom2master(slave); /* set eeprom control to master */
    configadr = ec_slave[slave].configadr;
    return (ec_writeeepromFP(configadr, eeproma, data, timeout));
}


/** Set eeprom control to master. Only if set to PDI.
 * @param[in] slave		= Slave number
 * @return >0 if OK
 */
int ec_eeprom2master(uint16 slave)
{
	int wkc = 1, cnt = 0;
    uint16 configadr;
	uint8 eepctl;

	if ( ec_slave[slave].eep_pdi )
	{
    	configadr = ec_slave[slave].configadr; 
		eepctl = 2;
		do
		{
			wkc = ec_FPWR(configadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* force Eeprom from PDI */
		}
		while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
		eepctl = 0;
		cnt = 0;
		do
		{
			wkc = ec_FPWR(configadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to master */
		}
		while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
		ec_slave[slave].eep_pdi = 0;
	}

	return wkc;
}	

/** Set eeprom control to PDI. Only if set to master.
 * @param[in] slave		= Slave number
 * @return >0 if OK
 */
int ec_eeprom2pdi(uint16 slave)
{
	int wkc = 1, cnt = 0;
    uint16 configadr;
	uint8 eepctl;

	if ( !ec_slave[slave].eep_pdi )
	{
    	configadr = ec_slave[slave].configadr; 
		eepctl = 1;
		do
		{
			wkc = ec_FPWR(configadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to PDI */
		}
		while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
		ec_slave[slave].eep_pdi = 1;
	}

	return wkc;
}	

uint16 ec_eeprom_waitnotbusyAP(uint16 aiadr,uint16 *estat, int timeout)
{
	int wkc, cnt = 0, retval = 0;
	struct timeval tv1, tv2, tve;
	
	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
	do
	{
		if (cnt++) usleep(EC_LOCALDELAY);
		wkc=ec_APRD(aiadr, ECT_REG_EEPSTAT, sizeof(*estat), estat, EC_TIMEOUTRET);
		*estat = etohs(*estat);
		gettimeofday(&tv2, 0);
	}
	while (((wkc <= 0) || ((*estat & EC_ESTAT_BUSY) > 0)) && (timercmp(&tv2, &tve, <))); /* wait for eeprom ready */
	if ((*estat & EC_ESTAT_BUSY) == 0) retval = 1;
	return retval;
}

/** Read EEPROM from slave bypassing cache. APRD method.
 * @param[in] aiadr		= auto increment address of slave
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] timeout	= Timeout in us.
 * @return EEPROM data 64bit or 32bit
 */
uint64 ec_readeepromAP(uint16 aiadr, uint16 eeproma, int timeout)
{
    uint16 estat;
    uint32 edat32;
	uint64 edat64;
    ec_eepromt ed;
	int wkc, cnt, nackcnt = 0;
	struct timeval tv1, tv2, tve;
	
	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
	edat64 = 0;
	edat32 = 0;
	if (ec_eeprom_waitnotbusyAP(aiadr, &estat, timeout))
	{
		if (estat & EC_ESTAT_EMASK) /* error bits are set */
		{
			estat = htoes(EC_ECMD_NOP); /* clear error bits */
			wkc=ec_APWR(aiadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET);
		}

		do
		{
			ed.comm = htoes(EC_ECMD_READ);
			ed.addr = htoes(eeproma);
			ed.d2   = 0x0000;
			cnt = 0;
			do
				wkc=ec_APWR(aiadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
			while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
			if (wkc)
			{
				usleep(EC_LOCALDELAY);
				estat = 0x0000;
				if (ec_eeprom_waitnotbusyAP(aiadr, &estat, timeout))
				{	
					if (estat & EC_ESTAT_NACK)
					{
						nackcnt++;
						usleep(EC_LOCALDELAY * 5);
					}
					else
					{
						nackcnt = 0;
						if (estat & EC_ESTAT_R64)
						{
							cnt = 0;
							do
								wkc=ec_APRD(aiadr, ECT_REG_EEPDAT, sizeof(edat64), &edat64, EC_TIMEOUTRET);
							while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
						}
						else
						{	
							cnt = 0;
							do
								wkc=ec_APRD(aiadr, ECT_REG_EEPDAT, sizeof(edat32), &edat32, EC_TIMEOUTRET);
							while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
							edat64=(uint64)edat32;
						}
					}
				}	
			}			
		}
		while ((nackcnt > 0) && (nackcnt < 3));
	}
    return edat64;
}

/** Write EEPROM to slave bypassing cache. APWR method.
 * @param[in] aiadr		= configured address of slave
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data		= 16bit data
 * @param[in] timeout	= Timeout in us.
 * @return >0 if OK
 */
int ec_writeeepromAP(uint16 aiadr, uint16 eeproma, uint16 data, int timeout)
{
    uint16 estat;
    ec_eepromt ed;
	int wkc, rval = 0, cnt = 0, nackcnt = 0;
	struct timeval tv1, tv2, tve;

	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
	if (ec_eeprom_waitnotbusyAP(aiadr, &estat, timeout))
	{
		if (estat & EC_ESTAT_EMASK) /* error bits are set */
		{
			estat = htoes(EC_ECMD_NOP); /* clear error bits */
			wkc=ec_APWR(aiadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET);
		}
		do
		{
			cnt = 0;
			do
				wkc=ec_APWR(aiadr, ECT_REG_EEPDAT, sizeof(data), &data, EC_TIMEOUTRET);
			while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));

			ed.comm = EC_ECMD_WRITE;
			ed.addr = eeproma;
			ed.d2   = 0x0000;
			cnt = 0;
			do
				wkc=ec_APWR(aiadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
			while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
			if (wkc)
			{
				usleep(EC_LOCALDELAY * 2);
				estat = 0x0000;
				if (ec_eeprom_waitnotbusyAP(aiadr, &estat, timeout))
				{	
					if (estat & EC_ESTAT_NACK)
					{
						nackcnt++;
						usleep(EC_LOCALDELAY * 5);
					}
					else
					{
						nackcnt = 0;
						rval = 1;
					}
				}
			}

		}
		while ((nackcnt > 0) && (nackcnt < 3));		
	}
    return rval;
}

uint16 ec_eeprom_waitnotbusyFP(uint16 configadr,uint16 *estat, int timeout)
{
	int wkc, cnt = 0, retval = 0;
	struct timeval tv1, tv2, tve;
	
	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
	do
	{
		if (cnt++) usleep(EC_LOCALDELAY);
		wkc=ec_FPRD(configadr, ECT_REG_EEPSTAT, sizeof(*estat), estat, EC_TIMEOUTRET);
		*estat = etohs(*estat);
		gettimeofday(&tv2, 0);
	}
	while (((wkc <= 0) || ((*estat & EC_ESTAT_BUSY) > 0)) && (timercmp(&tv2, &tve, <))); /* wait for eeprom ready */
	if ((*estat & EC_ESTAT_BUSY) == 0) retval = 1;
	return retval;
}

/** Read EEPROM from slave bypassing cache. FPRD method.
 * @param[in] configadr	= configured address of slave
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] timeout	= Timeout in us.
 * @return EEPROM data 64bit or 32bit
 */
uint64 ec_readeepromFP(uint16 configadr, uint16 eeproma, int timeout)
{
    uint16 estat;
    uint32 edat32;
	uint64 edat64;
    ec_eepromt ed;
	int wkc, cnt, nackcnt = 0;
	struct timeval tv1, tv2, tve;
	
	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
	edat64 = 0;
	edat32 = 0;
	if (ec_eeprom_waitnotbusyFP(configadr, &estat, timeout))
	{
		if (estat & EC_ESTAT_EMASK) /* error bits are set */
		{
			estat = htoes(EC_ECMD_NOP); /* clear error bits */
			wkc=ec_FPWR(configadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET);
		}

		do
		{
			ed.comm = htoes(EC_ECMD_READ);
			ed.addr = htoes(eeproma);
			ed.d2   = 0x0000;
			cnt = 0;
			do
				wkc=ec_FPWR(configadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
			while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
			if (wkc)
			{
				usleep(EC_LOCALDELAY);
				estat = 0x0000;
				if (ec_eeprom_waitnotbusyFP(configadr, &estat, timeout))
				{	
					if (estat & EC_ESTAT_NACK)
					{
						nackcnt++;
						usleep(EC_LOCALDELAY * 5);
					}
					else
					{
						nackcnt = 0;
						if (estat & EC_ESTAT_R64)
						{
							cnt = 0;
							do
								wkc=ec_FPRD(configadr, ECT_REG_EEPDAT, sizeof(edat64), &edat64, EC_TIMEOUTRET);
							while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
						}
						else
						{	
							cnt = 0;
							do
								wkc=ec_FPRD(configadr, ECT_REG_EEPDAT, sizeof(edat32), &edat32, EC_TIMEOUTRET);
							while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
							edat64=(uint64)edat32;
						}
					}
				}	
			}			
		}
		while ((nackcnt > 0) && (nackcnt < 3));
	}
    return edat64;
}

/** Write EEPROM to slave bypassing cache. FPWR method.
 * @param[in] configadr	= configured address of slave
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 * @param[in] data		= 16bit data
 * @param[in] timeout	= Timeout in us.
 * @return >0 if OK
 */
int ec_writeeepromFP(uint16 configadr, uint16 eeproma, uint16 data, int timeout)
{
    uint16 estat;
    ec_eepromt ed;
	int wkc, rval = 0, cnt = 0, nackcnt = 0;
	struct timeval tv1, tv2, tve;
	
	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
	if (ec_eeprom_waitnotbusyFP(configadr, &estat, timeout))
	{
		if (estat & EC_ESTAT_EMASK) /* error bits are set */
		{
			estat = htoes(EC_ECMD_NOP); /* clear error bits */
			wkc=ec_FPWR(configadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET);
		}
		do
		{
			cnt = 0;
			do
				wkc=ec_FPWR(configadr, ECT_REG_EEPDAT, sizeof(data), &data, EC_TIMEOUTRET);
			while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
			ed.comm = EC_ECMD_WRITE;
			ed.addr = eeproma;
			ed.d2   = 0x0000;
			cnt = 0;
			do
				wkc=ec_FPWR(configadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
			while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
			if (wkc)
			{
				usleep(EC_LOCALDELAY * 2);
				estat = 0x0000;
				if (ec_eeprom_waitnotbusyFP(configadr, &estat, timeout))
				{	
					if (estat & EC_ESTAT_NACK)
					{
						nackcnt++;
						usleep(EC_LOCALDELAY * 5);
					}
					else
					{
						nackcnt = 0;
						rval = 1;
					}
				}
			}
		}
		while ((nackcnt > 0) && (nackcnt < 3));		
	}
    return rval;
}

/** Read EEPROM from slave bypassing cache.
 * Parallel read step 1, make request to slave.
 * @param[in] slave		= Slave number
 * @param[in] eeproma   = (WORD) Address in the EEPROM
 */
void ec_readeeprom1(uint16 slave, uint16 eeproma)
{
    uint16 configadr, estat;
    ec_eepromt ed;
	int wkc, cnt = 0;

	ec_eeprom2master(slave); /* set eeprom control to master */
    configadr = ec_slave[slave].configadr;
	if (ec_eeprom_waitnotbusyFP(configadr, &estat, EC_TIMEOUTEEP))
	{
		if (estat & EC_ESTAT_EMASK) /* error bits are set */
		{
			estat = htoes(EC_ECMD_NOP); /* clear error bits */
			wkc=ec_FPWR(configadr, ECT_REG_EEPCTL, sizeof(estat), &estat, EC_TIMEOUTRET);
		}
		ed.comm = htoes(EC_ECMD_READ);
		ed.addr = htoes(eeproma);
		ed.d2   = 0x0000;
		do
			wkc = ec_FPWR(configadr, ECT_REG_EEPCTL, sizeof(ed), &ed, EC_TIMEOUTRET);
		while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
	}
}

/** Read EEPROM from slave bypassing cache.
 * Parallel read step 2, actual read from slave.
 * @param[in] slave		= Slave number
 * @param[in] timeout	= Timeout in us.
 * @return EEPROM data 32bit
 */
uint32 ec_readeeprom2(uint16 slave, int timeout)
{
    uint16 estat, configadr;
    uint32 edat;
	int wkc, cnt = 0;
	struct timeval tv1, tv2, tve;
	
	gettimeofday(&tv1, 0);
	tv2.tv_sec = 0;
	tv2.tv_usec = timeout;
	timeradd(&tv1, &tv2, &tve);
    configadr = ec_slave[slave].configadr;
	edat = 0;
    estat = 0x0000;
	if (ec_eeprom_waitnotbusyFP(configadr, &estat, timeout))
	{	
		do
		    wkc = ec_FPRD(configadr, ECT_REG_EEPDAT, sizeof(edat), &edat, EC_TIMEOUTRET);
		while ((wkc <= 0) && (cnt++ < EC_DEFAULTRETRIES));
	}

    return edat;
}

/** Push index of segmented LRD/LWR/LRW combination.
 * @param[in] idx			= Used datagram index.
 * @param[in] data			= Pointer to process data segment.
 * @param[in] length		= Length of data segment in bytes.
 */
static void ec_pushindex(uint8 idx, void *data, uint16 length)
{
	if(ec_idxstack.pushed < EC_MAXBUF)
	{
		ec_idxstack.idx[ec_idxstack.pushed] = idx;
		ec_idxstack.data[ec_idxstack.pushed] = data;
		ec_idxstack.length[ec_idxstack.pushed] = length;
		ec_idxstack.pushed++;
	}	
}	

/** Pull index of segmented LRD/LWR/LRW combination.
 * @return Stack location, -1 if stack is empty.
 */
static int ec_pullindex(void)
{
	int rval = -1;
	if(ec_idxstack.pulled < ec_idxstack.pushed)
	{
		rval = ec_idxstack.pulled;
		ec_idxstack.pulled++;
	}
	
	return rval;
}	

/** Transmit processdata to slaves.
 * Uses LRW, or LRD/LWR if LRW is not allowed (blockLRW).
 * Both the input and output processdata are transmitted.
 * The outputs with the actual data, the inputs have a placeholder.
 * The inputs are gathered with the receive processdata function.
 * In contrast to the base LRW function this function is non-blocking.
 * If the processdata does not fit in one datagram, multiple are used.
 * In order to recombine the slave response, a stack is used.
 * @return >0 if processdata is transmitted.
 */
int ec_send_processdata_group(uint8 group)
{
	uint32 LogAdr;
	uint16 w1, w2;
	int length, sublength;
    uint8 idx;
	int wkc;
	void* data;
	boolean first=FALSE;
	uint16 currentsegment = 0;

	wkc = 0;
	if(ec_group[group].hasdc)
		first = TRUE;
	length = ec_group[group].Obytes + ec_group[group].Ibytes;
	LogAdr = ec_group[group].logstartaddr;
	if (length)
	{	
		if(!group)
		{
			ec_idxstack.pushed = 0;
			ec_idxstack.pulled = 0;
		}
		wkc = 1;
		/* LRW blocked by one or more slaves ? */
		if (ec_group[group].blockLRW)
		{
			/* if inputs available generate LRD */
			if(ec_group[group].Ibytes)
			{
				currentsegment = ec_group[group].Isegment;
				data=ec_group[group].inputs;
				length = ec_group[group].Ibytes;
				LogAdr += ec_group[group].Obytes;
				/* segment transfer if needed */
				do
				{			
					if(currentsegment == ec_group[group].Isegment)
						sublength = ec_group[group].IOsegment[currentsegment++] - ec_group[group].Ioffset;
					else
						sublength = ec_group[group].IOsegment[currentsegment++];
					/* get new index */
				    idx = ec_getindex();
					w1 = LO_WORD(LogAdr);
					w2 = HI_WORD(LogAdr);
				    ec_setupdatagram(&ec_txbuf[idx], EC_CMD_LRD, idx, w1, w2, sublength, data);
					/* send frame */
					ec_outframe_red(idx);
					/* push index and data pointer on stack */
					ec_pushindex(idx, data, sublength);
					length -= sublength;
					LogAdr += sublength;
					data += sublength;
				} while (length && (currentsegment < ec_group[group].nsegments));	
			}	
			/* if outputs available generate LWR */
			if(ec_group[group].Obytes)
			{
				data=ec_group[group].outputs;
				length = ec_group[group].Obytes;
				LogAdr = ec_group[group].logstartaddr;
				currentsegment = 0;
				/* segment transfer if needed */
				do
				{			
					sublength = ec_group[group].IOsegment[currentsegment++];
					if((length - sublength) < 0)
						sublength = length;
					/* get new index */
				    idx = ec_getindex();
					w1 = LO_WORD(LogAdr);
					w2 = HI_WORD(LogAdr);
				    ec_setupdatagram(&ec_txbuf[idx], EC_CMD_LWR, idx, w1, w2, sublength, data);
					/* send frame */
					ec_outframe_red(idx);
					/* push index and data pointer on stack */
					ec_pushindex(idx, data, sublength);
					length -= sublength;
					LogAdr += sublength;
					data += sublength;
				} while (length && (currentsegment < ec_group[group].nsegments));	
			}
		}
		/* LRW can be used */
		else
		{	
			if (ec_group[group].Obytes)
				data=ec_group[group].outputs;
			else
				data=ec_group[group].inputs;
			/* segment transfer if needed */
			do
			{			
				sublength = ec_group[group].IOsegment[currentsegment++];
				/* get new index */
			    idx = ec_getindex();
			    w1 = LO_WORD(LogAdr);
				w2 = HI_WORD(LogAdr);
			    ec_setupdatagram(&ec_txbuf[idx], EC_CMD_LRW, idx, w1, w2, sublength, data);
				if(first)
				{
					ec_DCl = sublength;
					/* FPRMW in second datagram */
				    ec_DCtO = ec_adddatagram(&ec_txbuf[idx], EC_CMD_FRMW, idx, FALSE, 
											 ec_slave[ec_group[group].DCnext].configadr, 
											 ECT_REG_DCSYSTIME, sizeof(ec_DCtime), &ec_DCtime);
					first = FALSE;
				}					
				/* send frame */
				ec_outframe_red(idx);
				/* push index and data pointer on stack */
				ec_pushindex(idx, data, sublength);
				length -= sublength;
				LogAdr += sublength;
				data += sublength;
			} while (length && (currentsegment < ec_group[group].nsegments));	
		}	
	}	

	return wkc;
}

/** Receive processdata from slaves.
 * Second part from ec_send_processdata().
 * Received datagrams are recombined with the processdata with help from the stack.
 * If a datagram contains input processdata it copies it to the processdata structure.
 * @param[in] timeout	  = Timeout in us.
 * @return Work counter.
 */
int ec_receive_processdata_group(uint8 group, int timeout)
{
    int pos, idx;
	int wkc = 0, wkc2;
	boolean first = FALSE;

	if(ec_group[group].hasdc)
		first = TRUE;
	/* get first index */
	pos = ec_pullindex();
	/* read the same number of frames as send */
	while (pos >= 0)
	{	
		idx = ec_idxstack.idx[pos];
	    wkc2 = ec_waitinframe(ec_idxstack.idx[pos], timeout);
		/* check if there is input data in frame */
		if ((wkc2 > EC_NOFRAME) && ((ec_rxbuf[idx][EC_CMDOFFSET]==EC_CMD_LRD) || (ec_rxbuf[idx][EC_CMDOFFSET]==EC_CMD_LRW)))
	    {
			if(first)
			{	
		        memcpy(ec_idxstack.data[pos], &ec_rxbuf[idx][EC_HEADERSIZE], ec_DCl);
				memcpy(&wkc, &ec_rxbuf[idx][EC_HEADERSIZE + ec_DCl], EC_WKCSIZE);
				wkc = etohs(wkc);
				memcpy(&ec_DCtime, &ec_rxbuf[idx][ec_DCtO], sizeof(ec_DCtime));
				ec_DCtime = etohll(ec_DCtime);
				first = FALSE;			}
			else
			{	
				/* copy input data back to process data buffer */
			    memcpy(ec_idxstack.data[pos], &ec_rxbuf[idx][EC_HEADERSIZE], ec_idxstack.length[pos]);
				wkc += wkc2;
			}	
	    }
		/* release buffer */
	    ec_setbufstat(idx, EC_BUF_EMPTY);
		/* get next index */
		pos = ec_pullindex();
	}	

	return wkc;
}	

int ec_send_processdata(void)
{
	return ec_send_processdata_group(0);
}

int ec_receive_processdata(int timeout)
{
	return ec_receive_processdata_group(0, timeout);
}
