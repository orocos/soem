/*
 * Simple Open EtherCAT Master Library 
 *
 * File    : ethercatdc.c
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
 * Distributed Clock EtherCAT functions. 
 *
 */
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"

#define PORTM0 0x01
#define PORTM1 0x02
#define PORTM2 0x04
#define PORTM3 0x08

/** 1st sync pulse delay in ns here 100ms */
#define SyncDelay       ((int32)100000000)

/**
 * Set DC of slave to fire sync0 at CyclTime interval with CyclShift offset.
 *
 * @param [in] slave            Slave number.
 * @param [in] act              TRUE = active, FALSE = deactivated
 * @param [in] CyclTime         Cycltime in ns.
 * @param [in] CyclShift        CyclShift in ns.
 */
void ec_dcsync0(uint16 slave, boolean act, uint32 CyclTime, uint32 CyclShift)
{
    uint8 h, RA;
    uint16 wc, slaveh;
    int64 t, t1;
    int32 tc;

    slaveh = ec_slave[slave].configadr;
    RA = 0;

    /* stop cyclic operation, ready for next trigger */
    wc = ec_FPWR(slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); 
    if (act)
    {
        RA = 1 + 2;    /* act cyclic operation and sync0, sync1 deactivated */
    }
    h = 0;
    wc = ec_FPWR(slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET); /* write access to ethercat */
    wc = ec_FPRD(slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET); /* read local time of slave */
	t1 = etohll(t1);

    /* Calculate first trigger time, always a whole multiple of CyclTime rounded up
    plus the shifttime (can be negative)
    This insures best sychronisation between slaves, slaves with the same CyclTime
    will sync at the same moment (you can use CyclShift to shift the sync) */
    if (CyclTime > 0)
    {
        t = ((t1 + SyncDelay) / CyclTime) * CyclTime + CyclTime + CyclShift;
    }
    else
    {
        t = t1 + SyncDelay + CyclShift;
        /* first trigger at T1 + CyclTime + SyncDelay + CyclShift in ns */
    }
	t = htoell(t);
    wc = ec_FPWR(slaveh, ECT_REG_DCSTART0, sizeof(t), &t, EC_TIMEOUTRET); /* SYNC0 start time */
    tc = htoel(CyclTime);
    wc = ec_FPWR(slaveh, ECT_REG_DCCYCLE0, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC0 cycle time */
    wc = ec_FPWR(slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); /* activate cyclic operation */
}

/**
 * Set DC of slave to fire sync0 and sync1 at CyclTime interval with CyclShift offset.
 *
 * @param [in] slave            Slave number.
 * @param [in] act              TRUE = active, FALSE = deactivated
 * @param [in] CyclTime0        Cycltime SYNC0 in ns.
 * @param [in] CyclTime1        Cycltime SYNC1 in ns. This time is a delta time in relation to
 								the SYNC0 fire. If CylcTime1 = 0 then SYNC1 fires a the same time
 								as SYNC0.
 * @param [in] CyclShift        CyclShift in ns.
 */
void ec_dcsync01(uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, uint32 CyclShift)
{
    uint8 h, RA;
    uint16 wc, slaveh;
    int64 t, t1;
    int32 tc;

    slaveh = ec_slave[slave].configadr;
    RA = 0;

    /* stop cyclic operation, ready for next trigger */
    wc = ec_FPWR(slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); 
    if (act)
    {
        RA = 1 + 2 + 4;    /* act cyclic operation and sync0 + sync1 */
    }
    h = 0;
    wc = ec_FPWR(slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET); /* write access to ethercat */
    wc = ec_FPRD(slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET); /* read local time of slave */
	t1 = etohll(t1);

    /* Calculate first trigger time, always a whole multiple of CyclTime rounded up
    plus the shifttime (can be negative)
    This insures best sychronisation between slaves, slaves with the same CyclTime
    will sync at the same moment (you can use CyclShift to shift the sync) */
    if (CyclTime0 > 0)
    {
        t = ((t1 + SyncDelay) / CyclTime0) * CyclTime0 + CyclTime0 + CyclShift;
    }
    else
    {
        t = t1 + SyncDelay + CyclShift;
        /* first trigger at T1 + CyclTime + SyncDelay + CyclShift in ns */
    }
	t = htoell(t);
    wc = ec_FPWR(slaveh, ECT_REG_DCSTART0, sizeof(t), &t, EC_TIMEOUTRET); /* SYNC0 start time */
    tc = htoel(CyclTime0);
    wc = ec_FPWR(slaveh, ECT_REG_DCCYCLE0, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC0 cycle time */
    tc = htoel(CyclTime1);
    wc = ec_FPWR(slaveh, ECT_REG_DCCYCLE1, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC1 cycle time */
    wc = ec_FPWR(slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); /* activate cyclic operation */
}

/* latched port time of slave */
int32 ec_porttime(uint16 slave, uint8 port)
{
	int32 ts;
	switch (port)
	{
		case 0:
			ts = ec_slave[slave].DCrtA;
			break;
		case 1:
			ts = ec_slave[slave].DCrtB;
			break;
		case 2:
			ts = ec_slave[slave].DCrtC;
			break;
		case 3:
			ts = ec_slave[slave].DCrtD;
			break;
		default:
			ts = 0;
	}
	return ts;
}

/* calculate previous active port of a slave */
uint8 ec_prevport(uint16 slave, uint8 port)
{
	uint8 pport = port;
	uint8 aport = ec_slave[slave].activeports;
	switch(port)
	{
		case 0:
			if(aport & PORTM2)
				pport = 2;
			else if (aport & PORTM1)
				pport = 1;
			else if (aport & PORTM2)
				pport = 3;
			break;
		case 1:
			if(aport & PORTM3)
				pport = 3;
			else if (aport & PORTM0)
				pport = 0;
			else if (aport & PORTM2)
				pport = 2;
			break;
		case 2:
			if(aport & PORTM1)
				pport = 1;
			else if (aport & PORTM3)
				pport = 3;
			else if (aport & PORTM0)
				pport = 0;
			break;
		case 3:
			if(aport & PORTM0)
				pport = 0;
			else if (aport & PORTM2)
				pport = 2;
			else if (aport & PORTM1)
				pport = 1;
			break;
	}		
	return pport;
}

/* search unconsumed ports in parent, consume and return first open port */
uint8 ec_parentport(uint16 parent)
{
	uint8 parentport = 0;
	uint8 b;
	/* search order is important, here 3 - 1 - 2 - 0 */
	b = ec_slave[parent].consumedports;
	if (b & PORTM3)
	{
		parentport = 3;
		b &= (uint8)~PORTM3;
	}
	else if (b & PORTM1)
	{
		parentport = 1;
		b &= (uint8)~PORTM1;
	}
	else if (b & PORTM2)
	{
		parentport = 2;
		b &= (uint8)~PORTM2;
	}
	else if (b & PORTM0)
	{
		parentport = 0;
		b &= (uint8)~PORTM0;
	}
	ec_slave[parent].consumedports = b;
	return parentport;
}

/**
 * Locate DC slaves, measure propagation delays.
 *
 * return boolean if slaves are found with DC
 */
boolean ec_configdc(void)
{
    uint16 i, wc, slaveh, parent, child;
	uint16 parenthold = 0;
	uint16 prevDCslave = 0;
    int32 ht, dt1, dt2, dt3;
    int64 hrt;
	uint8 entryport;
	int8 nlist;
	int8 plist[4];
	int32 tlist[4];

	ec_slave[0].hasdc = FALSE;
	ec_group[0].hasdc = FALSE;
    ht = 0;
    ec_BWR(0, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);  /* latch DCrecvTimeA of all slaves */
    for (i = 1; i <= ec_slavecount; i++)
    {
		ec_slave[i].consumedports = ec_slave[i].activeports;
        if (ec_slave[i].hasdc)
        {
            if (!ec_slave[0].hasdc)
            {
				ec_slave[0].hasdc = TRUE;
				ec_slave[0].DCnext = i;
				ec_slave[i].DCprevious = 0;
				ec_group[0].hasdc = TRUE;
				ec_group[0].DCnext = i;
            }
			else
			{
				ec_slave[prevDCslave].DCnext = i;
				ec_slave[i].DCprevious = prevDCslave;
			}
			/* this branch has DC slave so remove parenthold */
			parenthold = 0;
			prevDCslave = i;
            slaveh = ec_slave[i].configadr;
            wc = ec_FPRD(slaveh, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);
            ec_slave[i].DCrtA = etohl(ht);
            /* 64bit latched DCrecvTimeA of each specific slave */
            wc = ec_FPRD(slaveh, ECT_REG_DCSOF, sizeof(hrt), &hrt, EC_TIMEOUTRET);
            /* use it as offset in order to set local time around 0 */
            hrt = htoell(-etohll(hrt));
            /* save it in the offset register */
            wc = ec_FPWR(slaveh, ECT_REG_DCSYSOFFSET, sizeof(hrt), &hrt, EC_TIMEOUTRET);
            wc = ec_FPRD(slaveh, ECT_REG_DCTIME1, sizeof(ht), &ht, EC_TIMEOUTRET);
            ec_slave[i].DCrtB = etohl(ht);
            wc = ec_FPRD(slaveh, ECT_REG_DCTIME2, sizeof(ht), &ht, EC_TIMEOUTRET);
            ec_slave[i].DCrtC = etohl(ht);
            wc = ec_FPRD(slaveh, ECT_REG_DCTIME3, sizeof(ht), &ht, EC_TIMEOUTRET);
            ec_slave[i].DCrtD = etohl(ht);

			/* make list of active ports and their time stamps */
			nlist = 0;
			if (ec_slave[i].activeports & PORTM0) 
			{
				plist[nlist] = 0;
				tlist[nlist] = ec_slave[i].DCrtA;
				nlist++;
			}
			if (ec_slave[i].activeports & PORTM3) 
			{
				plist[nlist] = 3;
				tlist[nlist] = ec_slave[i].DCrtD;
				nlist++;
			}
			if (ec_slave[i].activeports & PORTM1) 
			{
				plist[nlist] = 1;
				tlist[nlist] = ec_slave[i].DCrtB;
				nlist++;
			}
			if (ec_slave[i].activeports & PORTM2) 
			{
				plist[nlist] = 2;
				tlist[nlist] = ec_slave[i].DCrtC;
				nlist++;
			}
			/* entryport is port with the lowest timestamp */
			entryport = 0;
			if((nlist > 1) && (tlist[1] < tlist[entryport]))
			{
				entryport = 1;
			}			
			if((nlist > 2) && (tlist[2] < tlist[entryport]))
			{
				entryport = 2;
			}
			if((nlist > 3) && (tlist[3] < tlist[entryport]))
			{
				entryport = 3;
			}
			entryport = plist[entryport];
			ec_slave[i].entryport = entryport;
			/* consume entryport from activeports */
			ec_slave[i].consumedports &= (uint8)~(1 << entryport);

            /* finding DC parent of current */
            parent = i;
            do
            {
                child = parent;
                parent = ec_slave[parent].parent;
            }
            while (!((parent == 0) || (ec_slave[parent].hasdc)));
			/* only calculate propagation delay if slave is not the first */
            if (parent > 0)
            {
				/* find port on parent this slave is connected to */
				ec_slave[i].parentport = ec_parentport(parent);
				if (ec_slave[parent].topology == 1)
				{
					ec_slave[i].parentport = ec_slave[parent].entryport;
				}

				dt1 = 0;
				dt2 = 0;
				/* delta time of (parentport - 1) - parentport */
				/* note: order of ports is 0 - 3 - 1 -2 */
				/* non active ports are skipped */
				dt3 = ec_porttime(parent, ec_slave[i].parentport) -
					  ec_porttime(parent, ec_prevport(parent, ec_slave[i].parentport));
				/* current slave has children */
				/* those childrens delays need to be substacted */
				if (ec_slave[i].topology > 1)
				{
					dt1 = ec_porttime(i, ec_prevport(i, ec_slave[i].entryport)) -
					      ec_porttime(i, ec_slave[i].entryport);
				}
				/* we are only interrested in positive diference */
				if (dt1 > dt3) dt1 = -dt1;
				/* current slave is not the first child of parent */
				/* previous childs delays need to be added */
				if ((child - parent) > 1)
				{
					dt2 = ec_porttime(parent, ec_prevport(parent, ec_slave[i].parentport)) -
					      ec_porttime(parent, ec_slave[parent].entryport);
				}
				if (dt2 < 0) dt2 = -dt2;

				/* calculate current slave delay from delta times */
				/* assumption : forward delay equals return delay */
                ec_slave[i].pdelay = ((dt3 - dt1) / 2) + dt2 + ec_slave[parent].pdelay;
                ht = htoel(ec_slave[i].pdelay);
                /* write propagation delay*/
                wc = ec_FPWR(slaveh, ECT_REG_DCSYSDELAY, sizeof(ht), &ht, EC_TIMEOUTRET);
            }
        }
        else
        {
            ec_slave[i].DCrtA = 0;
            ec_slave[i].DCrtB = 0;
            ec_slave[i].DCrtC = 0;
            ec_slave[i].DCrtD = 0;
			parent = ec_slave[i].parent;
			/* if non DC slave found on first position on branch hold root parent */
			if ( (parent > 0) && (ec_slave[parent].topology > 2))
				parenthold = parent;
			/* if branch has no DC slaves consume port on root parent */
			if ( parenthold && (ec_slave[i].topology == 1))
			{
				ec_parentport(parenthold);
				parenthold = 0;
			}
        }
    }

    return ec_slave[0].hasdc;
}
