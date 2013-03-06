/*
 * Simple Open EtherCAT Master Library 
 *
 * File    : ethercatconfig.c
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
 * Configuration module for EtherCAT master.
 *
 * After successful initialisation with ec_init() or ec_init_redundant()
 * the slaves can be auto configured with this module.
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercatsoe.h"
#include "ethercatconfig.h"

// define if debug printf is needed
//#define EC_DEBUG

#ifdef EC_DEBUG
#define EC_PRINT printf
#else
#define EC_PRINT(...) do {} while (0)
#endif

/** Slave configuration structure */
typedef const struct
{
	/** Manufacturer code of slave */
	uint32				man;
	/** ID of slave */
	uint32				id;
	/** Readable name */
	char				name[EC_MAXNAME + 1];
	/** Data type */
	uint8				Dtype;
	/** Input bits */
	uint16				Ibits;
	/** Output bits */
	uint16				Obits;
	/** SyncManager 2 address */
	uint16				SM2a;
	/** SyncManager 2 flags */
	uint32				SM2f;
	/** SyncManager 3 address */
	uint16				SM3a;
	/** SyncManager 3 flags */
	uint32				SM3f;
	/** FMMU 0 activation */
	uint8				FM0ac;
	/** FMMU 1 activation */
	uint8				FM1ac;
} ec_configlist_t;

#include "ethercatconfiglist.h"

/** standard SM0 flags configuration for mailbox slaves */
#define EC_DEFAULTMBXSM0	0x00010026
/** standard SM1 flags configuration for mailbox slaves */
#define EC_DEFAULTMBXSM1	0x00010022
/** standard SM0 flags configuration for digital output slaves */
#define EC_DEFAULTDOSM0		0x00010044

/** buffer for EEPROM SM data */
static ec_eepromSMt ec_SM;
/** buffer for EEPROM FMMU data */
static ec_eepromFMMUt ec_FMMU;

/** Find slave in standard configuration list ec_configlist[]
 *
 * @param[in] man	   = manufacturer
 * @param[in] id	   = ID
 * @return index in ec_configlist[] when found, otherwise 0
 */
int ec_findconfig( uint32 man, uint32 id)
{
	int i = 0;

	do 
	{
		i++;
	} while ( (ec_configlist[i].man != EC_CONFIGEND) && 
			  ((ec_configlist[i].man != man) || (ec_configlist[i].id != id)) );
	if (ec_configlist[i].man == EC_CONFIGEND)
		i = 0;
	
	return i;
}

/** Enumerate and init all slaves.
 *
 * @param[in] usetable	  = TRUE when using configtable to init slaves, FALSE otherwise
 * @return Workcounter of slave discover datagram = number of slaves found
 */
int ec_config_init(uint8 usetable)
{
    uint16 w, slave, ADPh, configadr, mbx_wo, mbx_ro, mbx_l, mbx_rl, ssigen;
    uint16 topology, estat;
    int16 topoc, slavec;
    uint8 b,h;
	uint8 zbuf[64];
	uint8 SMc;
	uint32 eedat;
	int wkc, cindex, nSM, lp;

	EC_PRINT("ec_config_init %d\n",usetable);
    ec_slavecount = 0;
	/* clean ec_slave array */
	memset(&ec_slave, 0x00, sizeof(ec_slave));
	memset(&zbuf, 0x00, sizeof(zbuf));
	memset(&ec_group, 0x00, sizeof(ec_group));
	for(lp = 0; lp < EC_MAXGROUP; lp++)
	{
		ec_group[lp].logstartaddr = lp << 16; /* default start address per group entry */
	}
    w = 0x0000;
    wkc = ec_BRD(0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);		/* detect number of slaves */
    if (wkc > 0)
    {
        ec_slavecount = wkc;
        b = 0x00;
        ec_BWR(0x0000, ECT_REG_DLPORT, sizeof(b), &b, EC_TIMEOUTRET);		/* deact loop manual */
        w = htoes(0x0004);
        ec_BWR(0x0000, ECT_REG_IRQMASK, sizeof(w), &w, EC_TIMEOUTRET);		/* set IRQ mask */
        ec_BWR(0x0000, ECT_REG_RXERR, 8, &zbuf, EC_TIMEOUTRET);  			/* reset CRC counters */
        ec_BWR(0x0000, ECT_REG_FMMU0, 16 * 3, &zbuf, EC_TIMEOUTRET);		/* reset FMMU's */
        ec_BWR(0x0000, ECT_REG_SM0, 8 * 4, &zbuf, EC_TIMEOUTRET);			/* reset SyncM */
        ec_BWR(0x0000, ECT_REG_DCSYSTIME, 4, &zbuf, EC_TIMEOUTRET); 		/* reset system time+ofs */
        w = htoes(0x1000);
        ec_BWR(0x0000, ECT_REG_DCSPEEDCNT, sizeof(w), &w, EC_TIMEOUTRET);   /* DC speedstart */
        w = htoes(0x0c00);
        ec_BWR(0x0000, ECT_REG_DCTIMEFILT, sizeof(w), &w, EC_TIMEOUTRET);   /* DC filt expr */
        b = 0x00;
        ec_BWR(0x0000, ECT_REG_DLALIAS, sizeof(b), &b, EC_TIMEOUTRET);		/* Ignore Alias register */
        b = EC_STATE_INIT | EC_STATE_ACK;
        ec_BWR(0x0000, ECT_REG_ALCTL, sizeof(b), &b, EC_TIMEOUTRET);		/* Reset all slaves to Init */
		b = 2;
		ec_BWR(0x0000, ECT_REG_EEPCFG, sizeof(b), &b , EC_TIMEOUTRET); 		/* force Eeprom from PDI */
		b = 0;
		ec_BWR(0x0000, ECT_REG_EEPCFG, sizeof(b), &b , EC_TIMEOUTRET); 		/* set Eeprom to master */
		
        for (slave = 1; slave <= ec_slavecount; slave++)
        {
            ADPh = (uint16)(1 - slave);
            ec_slave[slave].Itype = etohs(ec_APRDw(ADPh, ECT_REG_PDICTL, EC_TIMEOUTRET)); /* read interface type of slave */
			/* a node offset is used to improve readibility of network frames */
			/* this has no impact on the number of addressable slaves (auto wrap around) */
            ec_APWRw(ADPh, ECT_REG_STADR, htoes(slave + EC_NODEOFFSET) , EC_TIMEOUTRET); /* set node address of slave */
            if (slave == 1) 
				b = 1; /* kill non ecat frames for first slave */
			else 
				b = 0; /* pass all frames for following slaves */
            ec_APWRw(ADPh, ECT_REG_DLCTL, htoes(b), EC_TIMEOUTRET); /* set non ecat frame behaviour */
            configadr = etohs(ec_APRDw(ADPh, ECT_REG_STADR, EC_TIMEOUTRET));
            ec_slave[slave].configadr = configadr;
			ec_FPRD(configadr, ECT_REG_ALIAS, sizeof(ec_slave[slave].aliasadr), &ec_slave[slave].aliasadr, EC_TIMEOUTRET);
			ec_FPRD(configadr, ECT_REG_EEPSTAT, sizeof(estat), &estat, EC_TIMEOUTRET);
			estat = etohs(estat);
			if (estat & EC_ESTAT_R64) /* check if slave can read 8 byte chunks */
				ec_slave[slave].eep_8byte = 1;
            ec_readeeprom1(slave, ECT_SII_MANUF); /* Manuf */
        }
		for (slave = 1; slave <= ec_slavecount; slave++)
        {
            ec_slave[slave].eep_man = etohl(ec_readeeprom2(slave, EC_TIMEOUTEEP)); /* Manuf */
            ec_readeeprom1(slave, ECT_SII_ID); /* ID */
        }
        for (slave = 1; slave <= ec_slavecount; slave++)
        {
            ec_slave[slave].eep_id = etohl(ec_readeeprom2(slave, EC_TIMEOUTEEP)); /* ID */
            ec_readeeprom1(slave, ECT_SII_REV); /* revision */
        }
        for (slave = 1; slave <= ec_slavecount; slave++)
        {
            ec_slave[slave].eep_rev = etohl(ec_readeeprom2(slave, EC_TIMEOUTEEP)); /* revision */
            ec_readeeprom1(slave, ECT_SII_RXMBXADR); /* write mailbox address + mailboxsize */
        }
        for (slave = 1; slave <= ec_slavecount; slave++)
        {
            eedat = etohl(ec_readeeprom2(slave, EC_TIMEOUTEEP)); /* write mailbox address and mailboxsize */
            ec_slave[slave].mbx_wo = (uint16)LO_WORD(eedat);
            ec_slave[slave].mbx_l = (uint16)HI_WORD(eedat);
			if (ec_slave[slave].mbx_l > 0) 
	            ec_readeeprom1(slave, ECT_SII_TXMBXADR); /* read mailbox offset */
        }
        for (slave = 1; slave <= ec_slavecount; slave++)
        {
			if (ec_slave[slave].mbx_l > 0) 
			{
	            eedat = etohl(ec_readeeprom2(slave, EC_TIMEOUTEEP)); /* read mailbox offset */
				ec_slave[slave].mbx_ro = (uint16)LO_WORD(eedat); /* read mailbox offset */
	            ec_slave[slave].mbx_rl = (uint16)HI_WORD(eedat); /*read mailbox length */
				if (ec_slave[slave].mbx_rl == 0)
	            	ec_slave[slave].mbx_rl = ec_slave[slave].mbx_l;
			}
            configadr = ec_slave[slave].configadr;
            mbx_ro = ec_slave[slave].mbx_ro;
            mbx_wo = ec_slave[slave].mbx_wo;
            mbx_l = ec_slave[slave].mbx_l;
            mbx_rl = ec_slave[slave].mbx_rl;
            if ((etohs(ec_FPRDw(configadr, ECT_REG_ESCSUP, EC_TIMEOUTRET)) & 0x04) > 0)  /* Support DC? */
                ec_slave[slave].hasdc = TRUE;
            else
                ec_slave[slave].hasdc = FALSE;
            topology = etohs(ec_FPRDw(configadr, ECT_REG_DLSTAT, EC_TIMEOUTRET)); /* extract topology from DL status */
			h = 0; 
			b = 0;
            if ((topology & 0x0300) == 0x0200) /* port0 open and communication established */
            {
                h++;
				b |= 0x01;
            }
            if ((topology & 0x0c00) == 0x0800) /* port1 open and communication established */
            {
                h++;
				b |= 0x02;
            }
            if ((topology & 0x3000) == 0x2000) /* port2 open and communication established */
            {
                h++;
				b |= 0x04;
            }
            if ((topology & 0xc000) == 0x8000) /* port3 open and communication established */
            {
                h++;
				b |= 0x08;
            }
            /* ptype = Physical type*/
            ec_slave[slave].ptype = LO_BYTE(etohs(ec_FPRDw(configadr, ECT_REG_PORTDES, EC_TIMEOUTRET)));
            ec_slave[slave].topology = h;
			ec_slave[slave].activeports = b;
			/* 0=no links, not possible             */
            /* 1=1 link  , end of line              */
            /* 2=2 links , one before and one after */
            /* 3=3 links , split point              */
            /* 4=4 links , cross point              */
            /* search for parent */
            ec_slave[slave].parent = 0; /* parent is master */
            if (slave > 1)
            {
                topoc = 0; 
                slavec = slave - 1;
                do
                {
		            topology = ec_slave[slavec].topology;
                    if (topology == 1)
                        topoc--; /* endpoint found */
                    if (topology == 3)
                        topoc++; /* split found */
                    if (topology == 4)
                        topoc+=2; /* cross found */
                    if (((topoc >= 0) && (topology > 1)) ||
					    (slavec == 1)) /* parent found */
                    {
                        ec_slave[slave].parent = slavec;
                        slavec = 1;
                    }
					slavec--;
                }
                while (slavec > 0);
            }

            w = ec_statecheck(slave, EC_STATE_INIT,  EC_TIMEOUTSTATE); //* check state change Init */
	
			/* set default mailbox configuration if slave has mailbox */
			if (ec_slave[slave].mbx_l>0)
			{	
				ec_slave[slave].SMtype[0] = 1;
				ec_slave[slave].SMtype[1] = 2;
				ec_slave[slave].SMtype[2] = 3;
				ec_slave[slave].SMtype[3] = 4;
				ec_slave[slave].SM[0].StartAddr = htoes(ec_slave[slave].mbx_wo);
				ec_slave[slave].SM[0].SMlength = htoes(ec_slave[slave].mbx_l);
				ec_slave[slave].SM[0].SMflags = htoel(EC_DEFAULTMBXSM0);
				ec_slave[slave].SM[1].StartAddr = htoes(ec_slave[slave].mbx_ro);
				ec_slave[slave].SM[1].SMlength = htoes(ec_slave[slave].mbx_rl);
				ec_slave[slave].SM[1].SMflags = htoel(EC_DEFAULTMBXSM1);
				ec_slave[slave].mbx_proto = ec_readeeprom (slave, ECT_SII_MBXPROTO, EC_TIMEOUTEEP);
			}	
			cindex = 0;
			/* use configuration table ? */
			if (usetable)
			{
				cindex = ec_findconfig( ec_slave[slave].eep_man, ec_slave[slave].eep_id );
				ec_slave[slave].configindex= cindex;
			}
			/* slave found in configuration table ? */
			if (cindex)
			{
				ec_slave[slave].Dtype = ec_configlist[cindex].Dtype;				
				strcpy(	ec_slave[slave].name ,ec_configlist[cindex].name);
				ec_slave[slave].Ibits = ec_configlist[cindex].Ibits;
				ec_slave[slave].Obits = ec_configlist[cindex].Obits;
				if (ec_slave[slave].Obits)
					ec_slave[slave].FMMU0func = 1;
				if (ec_slave[slave].Ibits)
					ec_slave[slave].FMMU1func = 2;
				ec_slave[slave].FMMU[0].FMMUactive = ec_configlist[cindex].FM0ac;
				ec_slave[slave].FMMU[1].FMMUactive = ec_configlist[cindex].FM1ac;
				ec_slave[slave].SM[2].StartAddr = htoes(ec_configlist[cindex].SM2a);
				ec_slave[slave].SM[2].SMflags = htoel(ec_configlist[cindex].SM2f);
				/* simple (no mailbox) output slave found ? */
				if (ec_slave[slave].Obits && !ec_slave[slave].SM[2].StartAddr)
				{
					ec_slave[slave].SM[0].StartAddr = htoes(0x0f00);
					ec_slave[slave].SM[0].SMlength = htoes((ec_slave[slave].Obits + 7) / 8);
					ec_slave[slave].SM[0].SMflags = htoel(EC_DEFAULTDOSM0);			
					ec_slave[slave].FMMU[0].FMMUactive = 1;
					ec_slave[slave].FMMU[0].FMMUtype = 2;
					ec_slave[slave].SMtype[0] = 3;
				}
				/* complex output slave */
				else
				{
					ec_slave[slave].SM[2].SMlength = htoes((ec_slave[slave].Obits + 7) / 8);
					ec_slave[slave].SMtype[2] = 3;
				}	
				ec_slave[slave].SM[3].StartAddr = htoes(ec_configlist[cindex].SM3a);
				ec_slave[slave].SM[3].SMflags = htoel(ec_configlist[cindex].SM3f);
				/* simple (no mailbox) input slave found ? */
				if (ec_slave[slave].Ibits && !ec_slave[slave].SM[3].StartAddr)
				{
					ec_slave[slave].SM[1].StartAddr = htoes(0x1000);
					ec_slave[slave].SM[1].SMlength = htoes((ec_slave[slave].Ibits + 7) / 8);
					ec_slave[slave].SM[1].SMflags = htoel(0x00000000);			
					ec_slave[slave].FMMU[1].FMMUactive = 1;
					ec_slave[slave].FMMU[1].FMMUtype = 1;
					ec_slave[slave].SMtype[1] = 4;
				}
				/* complex input slave */
				else
				{
					ec_slave[slave].SM[3].SMlength = htoes((ec_slave[slave].Ibits + 7) / 8);
					ec_slave[slave].SMtype[3] = 4;
				}	
			}
			/* slave not in configuration table, find out via SII */
			else
			{
				ssigen = ec_siifind(slave, ECT_SII_GENERAL);
				/* SII general section */
				if (ssigen)
                {
					ec_slave[slave].CoEdetails = ec_siigetbyte(slave, ssigen + 0x07);
					ec_slave[slave].FoEdetails = ec_siigetbyte(slave, ssigen + 0x08);
					ec_slave[slave].EoEdetails = ec_siigetbyte(slave, ssigen + 0x09);
					ec_slave[slave].SoEdetails = ec_siigetbyte(slave, ssigen + 0x0a);
					if((ec_siigetbyte(slave, ssigen + 0x0d) & 0x02) > 0)
					{
						ec_slave[slave].blockLRW = 1;
						ec_slave[0].blockLRW++;						
					}	
					ec_slave[slave].Ebuscurrent = ec_siigetbyte(slave, ssigen + 0x0e);
					ec_slave[slave].Ebuscurrent += ec_siigetbyte(slave, ssigen + 0x0f) << 8;
					ec_slave[0].Ebuscurrent += ec_slave[slave].Ebuscurrent;
                }
				/* SII strings section */
				if (ec_siifind(slave, ECT_SII_STRING) > 0)
                    ec_siistring(ec_slave[slave].name, slave, 1);
				/* no name for slave found, use constructed name */
                else
                {
                    sprintf(ec_slave[slave].name, "? M:%8.8x I:%8.8x",
                    (unsigned int)ec_slave[slave].eep_man, (unsigned int)ec_slave[slave].eep_id);
                }
				/* SII SM section */
				nSM = ec_siiSM (slave,&ec_SM);
				if (nSM>0)
				{	
					ec_slave[slave].SM[0].StartAddr = htoes(ec_SM.PhStart);
					ec_slave[slave].SM[0].SMlength = htoes(ec_SM.Plength);
					ec_slave[slave].SM[0].SMflags = htoel((ec_SM.Creg) + (ec_SM.Activate << 16));
					SMc = 1;
					while ((SMc < EC_MAXSM) &&  ec_siiSMnext(slave, &ec_SM, SMc))
					{
						ec_slave[slave].SM[SMc].StartAddr = htoes(ec_SM.PhStart);
						ec_slave[slave].SM[SMc].SMlength = htoes(ec_SM.Plength);
						ec_slave[slave].SM[SMc].SMflags = htoel((ec_SM.Creg) + (ec_SM.Activate << 16));
						SMc++;
					}	
				}	
				/* SII FMMU section */
                if (ec_siiFMMU(slave, &ec_FMMU))
				{
					if (ec_FMMU.FMMU0 !=0xff) 
						ec_slave[slave].FMMU0func = ec_FMMU.FMMU0;
					if (ec_FMMU.FMMU1 !=0xff) 
						ec_slave[slave].FMMU1func = ec_FMMU.FMMU1;
					if (ec_FMMU.FMMU2 !=0xff) 
						ec_slave[slave].FMMU2func = ec_FMMU.FMMU2;
					if (ec_FMMU.FMMU3 !=0xff) 
						ec_slave[slave].FMMU3func = ec_FMMU.FMMU3;
				}	
			}	

			if (ec_slave[slave].mbx_l > 0)
			{
				if (ec_slave[slave].SM[0].StartAddr == 0x0000) /* should never happen */
				{
					ec_slave[slave].SM[0].StartAddr = htoes(0x1000);
					ec_slave[slave].SM[0].SMlength = htoes(0x0080);
					ec_slave[slave].SM[0].SMflags = htoel(EC_DEFAULTMBXSM0);
					ec_slave[slave].SMtype[0] = 1;					
				}			
				if (ec_slave[slave].SM[1].StartAddr == 0x0000) /* should never happen */
				{
					ec_slave[slave].SM[1].StartAddr = htoes(0x1080);
					ec_slave[slave].SM[1].SMlength = htoes(0x0080);
					ec_slave[slave].SM[1].SMflags = htoel(EC_DEFAULTMBXSM1);
					ec_slave[slave].SMtype[1] = 2;
				}			
				/* program SM0 mailbox in for slave */
				ec_FPWR (configadr, ECT_REG_SM0, sizeof(ec_smt), &ec_slave[slave].SM[0], EC_TIMEOUTRET);
				/* program SM1 mailbox out for slave */
				// usleep(1000); // was needed for NETX (needs internal time after SM update)
				ec_FPWR (configadr, ECT_REG_SM1, sizeof(ec_smt), &ec_slave[slave].SM[1], EC_TIMEOUTRET);
			}	
			ec_FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_PRE_OP | EC_STATE_ACK) , EC_TIMEOUTRET); /* set preop status */
		}
	}	
    return wkc;
}

/** Map all PDOs in one group of slaves to IOmap.
 *
 * @param[out] pIOmap	  = pointer to IOmap	
 * @param[in]  group	  = group to map, 0 = all groups	
 * @return IOmap size
 */
int ec_config_map_group(void *pIOmap, uint8 group)
{
    uint16 slave, configadr;
	int Isize, Osize, BitCount, ByteCount, FMMUsize, FMMUdone;
	uint16 SMlength;
    uint8 BitPos, EndAddr;
	uint8 SMc, FMMUc;
	uint32 LogAddr = 0;
	uint32 oLogAddr = 0;
	uint32 diff;
	int nSM, rval;
	ec_eepromPDOt eepPDO;
	uint16 currentsegment = 0;
	uint32 segmentsize = 0;

	if ((ec_slavecount > 0) && (group < EC_MAXGROUP))
	{	
		EC_PRINT("ec_config_map_group IOmap:%p group:%d\n", pIOmap, group);
		LogAddr = ec_group[group].logstartaddr;
		oLogAddr = LogAddr;
		BitPos = 0;
		ec_group[group].nsegments = 0;
		ec_group[group].expectedWKC = 0;

		/* find output mapping of slave and program FMMU */
		for (slave = 1; slave <= ec_slavecount; slave++)
        {
            configadr = ec_slave[slave].configadr;

			ec_statecheck(slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */

			EC_PRINT(" >Slave %d, configadr %x, state %2.2x\n",
			    slave, ec_slave[slave].configadr, ec_slave[slave].state);

		  if (!group || (group == ec_slave[slave].group))
		  {	
			
			/* if slave not found in configlist find IO mapping in slave self */
			if (!ec_slave[slave].configindex)
			{
				Isize = 0;
				Osize = 0;
				if (ec_slave[slave].mbx_proto & ECT_MBXPROT_COE) /* has CoE */
				{
					if (ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA) /* has Complete Access */
						/* read PDO mapping via CoE and use Complete Access */
						rval = ec_readPDOmapCA(slave, &Osize, &Isize);
					else
						/* read PDO mapping via CoE */
						rval = ec_readPDOmap(slave, &Osize, &Isize);
					EC_PRINT("  CoE Osize:%d Isize:%d\n", Osize, Isize);
				}
				if ((!Isize && !Osize) && (ec_slave[slave].mbx_proto & ECT_MBXPROT_SOE)) /* has SoE */
				{
					/* read AT / MDT mapping via SoE */
					rval = ec_readIDNmap(slave, &Osize, &Isize);
					ec_slave[slave].SM[2].SMlength = htoes((Osize + 7) / 8);
					ec_slave[slave].SM[3].SMlength = htoes((Isize + 7) / 8);
					EC_PRINT("  SoE Osize:%d Isize:%d\n", Osize, Isize);
				}
				if (!Isize && !Osize) /* find PDO mapping by SII */
				{
					memset(&eepPDO, 0, sizeof(eepPDO));
					Isize = (int)ec_siiPDO (slave, &eepPDO, 0);
					EC_PRINT("  SII Isize:%d\n", Isize);					
					for( nSM=0 ; nSM < EC_MAXSM ; nSM++ )
					{	
						if (eepPDO.SMbitsize[nSM] > 0)
						{	
							ec_slave[slave].SM[nSM].SMlength =  htoes((eepPDO.SMbitsize[nSM] + 7) / 8);
							ec_slave[slave].SMtype[nSM] = 4;
							EC_PRINT("    SM%d length %d\n", nSM, eepPDO.SMbitsize[nSM]);
						}	
					}	
					Osize = (int)ec_siiPDO (slave, &eepPDO, 1);
					EC_PRINT("  SII Osize:%d\n", Osize);					
					for( nSM=0 ; nSM < EC_MAXSM ; nSM++ )
					{	
						if (eepPDO.SMbitsize[nSM] > 0)
						{	
							ec_slave[slave].SM[nSM].SMlength =  htoes((eepPDO.SMbitsize[nSM] + 7) / 8);
							ec_slave[slave].SMtype[nSM] = 3;
							EC_PRINT("    SM%d length %d\n", nSM, eepPDO.SMbitsize[nSM]);
						}	
					}	
				}
				ec_slave[slave].Obits = Osize;
				ec_slave[slave].Ibits = Isize;
				EC_PRINT("     ISIZE:%d %d OSIZE:%d\n", ec_slave[slave].Ibits, Isize,ec_slave[slave].Obits);
				
			}

			EC_PRINT("  SM programming\n");  
			if (!ec_slave[slave].mbx_l && ec_slave[slave].SM[0].StartAddr)
			  {
				ec_FPWR (configadr, ECT_REG_SM0, sizeof(ec_smt), &ec_slave[slave].SM[0], EC_TIMEOUTRET);
				EC_PRINT("    SM0 Type:%d StartAddr:%4.4x Flags:%8.8x\n", 
				    ec_slave[slave].SMtype[0], ec_slave[slave].SM[0].StartAddr, ec_slave[slave].SM[0].SMflags);   
			  }
			if (!ec_slave[slave].mbx_l && ec_slave[slave].SM[1].StartAddr)
			  {
				ec_FPWR (configadr, ECT_REG_SM1, sizeof(ec_smt), &ec_slave[slave].SM[1], EC_TIMEOUTRET);
				EC_PRINT("    SM1 Type:%d StartAddr:%4.4x Flags:%8.8x\n", 
				    ec_slave[slave].SMtype[1], ec_slave[slave].SM[1].StartAddr, ec_slave[slave].SM[1].SMflags);   
			  }
			/* program SM2 to SMx */
			for( nSM = 2 ; nSM < EC_MAXSM ; nSM++ )
			{	
				if (ec_slave[slave].SM[nSM].StartAddr)
				{
					/* check if SM length is zero -> clear enable flag */
					if( ec_slave[slave].SM[nSM].SMlength == 0) 
						ec_slave[slave].SM[nSM].SMflags = htoel( etohl(ec_slave[slave].SM[nSM].SMflags) & EC_SMENABLEMASK);
					ec_FPWR (configadr, ECT_REG_SM0 + (nSM * sizeof(ec_smt)), sizeof(ec_smt), &ec_slave[slave].SM[nSM], EC_TIMEOUTRET);
					EC_PRINT("    SM%d Type:%d StartAddr:%4.4x Flags:%8.8x\n", nSM,
					    ec_slave[slave].SMtype[nSM], ec_slave[slave].SM[nSM].StartAddr, ec_slave[slave].SM[nSM].SMflags);   
				}
			}
			if (ec_slave[slave].Ibits > 7)
				ec_slave[slave].Ibytes = (ec_slave[slave].Ibits + 7) / 8;
			if (ec_slave[slave].Obits > 7)
				ec_slave[slave].Obytes = (ec_slave[slave].Obits + 7) / 8;

			FMMUc = ec_slave[slave].FMMUunused;
			SMc = 0;
			BitCount = 0;
			ByteCount = 0;
			EndAddr = 0;
			FMMUsize = 0;
			FMMUdone = 0;
			/* create output mapping */
			if (ec_slave[slave].Obits)
			{
				EC_PRINT("  OUTPUT MAPPING\n");
				/* search for SM that contribute to the output mapping */
				while ( (SMc < (EC_MAXSM - 1)) && (FMMUdone < ((ec_slave[slave].Obits + 7) / 8)))
				{	
					EC_PRINT("    FMMU %d\n", FMMUc);
					while ( (SMc < (EC_MAXSM - 1)) && (ec_slave[slave].SMtype[SMc] != 3)) SMc++;
					EC_PRINT("      SM%d\n", SMc);
					ec_slave[slave].FMMU[FMMUc].PhysStart = ec_slave[slave].SM[SMc].StartAddr;
					SMlength = etohs(ec_slave[slave].SM[SMc].SMlength);
					ByteCount += SMlength;
					BitCount += SMlength * 8;
					EndAddr = etohs(ec_slave[slave].SM[SMc].StartAddr) + SMlength;
					while ( (BitCount < ec_slave[slave].Obits) && (SMc < (EC_MAXSM - 1)) ) /* more SM for output */
					{
						SMc++;
						while ( (SMc < (EC_MAXSM - 1)) && (ec_slave[slave].SMtype[SMc] != 3)) SMc++;
						/* if addresses from more SM connect use one FMMU otherwise break up in mutiple FMMU */
						if ( etohs(ec_slave[slave].SM[SMc].StartAddr) > EndAddr ) break;
						EC_PRINT("      SM%d\n", SMc);
						SMlength = etohs(ec_slave[slave].SM[SMc].SMlength);
						ByteCount += SMlength;
						BitCount += SMlength * 8;
						EndAddr = etohs(ec_slave[slave].SM[SMc].StartAddr) + SMlength;					
					}	

					/* bit oriented slave */
					if (!ec_slave[slave].Obytes)
					{	
						ec_slave[slave].FMMU[FMMUc].LogStart = htoel(LogAddr);
						ec_slave[slave].FMMU[FMMUc].LogStartbit = BitPos;
						BitPos += ec_slave[slave].Obits - 1;
						if (BitPos > 7)
						{
							LogAddr++;
							BitPos -= 8;
						}	
						FMMUsize = LogAddr - etohl(ec_slave[slave].FMMU[FMMUc].LogStart) + 1;
						ec_slave[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
						ec_slave[slave].FMMU[FMMUc].LogEndbit = BitPos;
						BitPos ++;
						if (BitPos > 7)
						{
							LogAddr++;
							BitPos -= 8;
						}	
					}
					/* byte oriented slave */
					else
					{
						if (BitPos)
						{
							LogAddr++;
							BitPos = 0;
						}	
						ec_slave[slave].FMMU[FMMUc].LogStart = htoel(LogAddr);
						ec_slave[slave].FMMU[FMMUc].LogStartbit = BitPos;
						BitPos = 7;
						FMMUsize = ByteCount;
						if ((FMMUsize + FMMUdone)> ec_slave[slave].Obytes)
							FMMUsize = ec_slave[slave].Obytes - FMMUdone;
						LogAddr += FMMUsize;
						ec_slave[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
						ec_slave[slave].FMMU[FMMUc].LogEndbit = BitPos;
						BitPos = 0;
					}
					FMMUdone += FMMUsize;
					ec_slave[slave].FMMU[FMMUc].PhysStartBit = 0;
					ec_slave[slave].FMMU[FMMUc].FMMUtype = 2;
					ec_slave[slave].FMMU[FMMUc].FMMUactive = 1;
					/* program FMMU for output */
					ec_FPWR (configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc), sizeof(ec_fmmut), 
					         &ec_slave[slave].FMMU[FMMUc], EC_TIMEOUTRET);
					/* add two for an output FMMU */
					ec_group[group].expectedWKC += 2;
					if (!ec_slave[slave].outputs)
					{	
						ec_slave[slave].outputs = pIOmap + etohl(ec_slave[slave].FMMU[FMMUc].LogStart);
						ec_slave[slave].Ostartbit = ec_slave[slave].FMMU[FMMUc].LogStartbit;
						EC_PRINT("    slave %d Outputs %p startbit %d\n", slave, ec_slave[slave].outputs, ec_slave[slave].Ostartbit);
					}
					FMMUc++;
				}	
				ec_slave[slave].FMMUunused = FMMUc;
				diff = LogAddr - oLogAddr;
				oLogAddr = LogAddr;
				if ((segmentsize + diff) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
				{
					ec_group[group].IOsegment[currentsegment] = segmentsize;
					if (currentsegment < (EC_MAXIOSEGMENTS - 1))
					{
						currentsegment++;
						segmentsize = diff;	
					}
				}
				else
					segmentsize += diff;
			}
		  }	
        }
		if (BitPos)
		{
			LogAddr++;
			oLogAddr = LogAddr;
			BitPos = 0;
			if ((segmentsize + 1) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
			{
				ec_group[group].IOsegment[currentsegment] = segmentsize;
				if (currentsegment < (EC_MAXIOSEGMENTS - 1))
				{
					currentsegment++;
					segmentsize = 1;	
				}
			}
			else
				segmentsize += 1;
		}	
		ec_group[group].outputs = pIOmap;
		ec_group[group].Obytes = LogAddr;
		ec_group[group].nsegments = currentsegment + 1;
		ec_group[group].Isegment = currentsegment;
		ec_group[group].Ioffset = segmentsize;
		if (!group)
		{	
			ec_slave[0].outputs = pIOmap;
			ec_slave[0].Obytes = LogAddr; /* store output bytes in master record */
		}	
		
		/* do input mapping of slave and program FMMUs */
		for (slave = 1; slave <= ec_slavecount; slave++)
        {
            configadr = ec_slave[slave].configadr;
			FMMUc = ec_slave[slave].FMMUunused;
			if (ec_slave[slave].Obits) /* find free FMMU */
				while ( ec_slave[slave].FMMU[FMMUc].LogStart ) FMMUc++;
			SMc = 0;
			BitCount = 0;
			ByteCount = 0;
			EndAddr = 0;
			FMMUsize = 0;
			FMMUdone = 0;
			/* create input mapping */
			if (ec_slave[slave].Ibits)
			{
				EC_PRINT(" =Slave %d, INPUT MAPPING\n", slave);
				/* search for SM that contribute to the input mapping */
				while ( (SMc < (EC_MAXSM - 1)) && (FMMUdone < ((ec_slave[slave].Ibits + 7) / 8)))
				{	
					EC_PRINT("    FMMU %d\n", FMMUc);
					while ( (SMc < (EC_MAXSM - 1)) && (ec_slave[slave].SMtype[SMc] != 4)) SMc++;
					EC_PRINT("      SM%d\n", SMc);
					ec_slave[slave].FMMU[FMMUc].PhysStart = ec_slave[slave].SM[SMc].StartAddr;
					SMlength = etohs(ec_slave[slave].SM[SMc].SMlength);
					ByteCount += SMlength;
					BitCount += SMlength * 8;
					EndAddr = etohs(ec_slave[slave].SM[SMc].StartAddr) + SMlength;
					while ( (BitCount < ec_slave[slave].Ibits) && (SMc < (EC_MAXSM - 1)) ) /* more SM for input */
					{
						SMc++;
						while ( (SMc < (EC_MAXSM - 1)) && (ec_slave[slave].SMtype[SMc] != 4)) SMc++;
						/* if addresses from more SM connect use one FMMU otherwise break up in mutiple FMMU */
						if ( etohs(ec_slave[slave].SM[SMc].StartAddr) > EndAddr ) break;
						EC_PRINT("      SM%d\n", SMc);
						SMlength = etohs(ec_slave[slave].SM[SMc].SMlength);
						ByteCount += SMlength;
						BitCount += SMlength * 8;
						EndAddr = etohs(ec_slave[slave].SM[SMc].StartAddr) + SMlength;					
					}	

					/* bit oriented slave */
					if (!ec_slave[slave].Ibytes)
					{	
						ec_slave[slave].FMMU[FMMUc].LogStart = htoel(LogAddr);
						ec_slave[slave].FMMU[FMMUc].LogStartbit = BitPos;
						BitPos += ec_slave[slave].Ibits - 1;
						if (BitPos > 7)
						{
							LogAddr++;
							BitPos -= 8;
						}	
						FMMUsize = LogAddr - etohl(ec_slave[slave].FMMU[FMMUc].LogStart) + 1;
						ec_slave[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
						ec_slave[slave].FMMU[FMMUc].LogEndbit = BitPos;
						BitPos ++;
						if (BitPos > 7)
						{
							LogAddr++;
							BitPos -= 8;
						}	
					}
					/* byte oriented slave */
					else
					{
						if (BitPos)
						{
							LogAddr++;
							BitPos = 0;
						}	
						ec_slave[slave].FMMU[FMMUc].LogStart = htoel(LogAddr);
						ec_slave[slave].FMMU[FMMUc].LogStartbit = BitPos;
						BitPos = 7;
						FMMUsize = ByteCount;
						if ((FMMUsize + FMMUdone)> ec_slave[slave].Ibytes)
							FMMUsize = ec_slave[slave].Ibytes - FMMUdone;
						LogAddr += FMMUsize;
						ec_slave[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
						ec_slave[slave].FMMU[FMMUc].LogEndbit = BitPos;
						BitPos = 0;
					}
					FMMUdone += FMMUsize;
					if (ec_slave[slave].FMMU[FMMUc].LogLength)
					{	
						ec_slave[slave].FMMU[FMMUc].PhysStartBit = 0;
						ec_slave[slave].FMMU[FMMUc].FMMUtype = 1;
						ec_slave[slave].FMMU[FMMUc].FMMUactive = 1;
						/* program FMMU for input */
						ec_FPWR (configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc), sizeof(ec_fmmut), 
						         &ec_slave[slave].FMMU[FMMUc], EC_TIMEOUTRET);
						/* add one for an input FMMU */
						ec_group[group].expectedWKC += 1;
					}	
					if (!ec_slave[slave].inputs)
					{	
						ec_slave[slave].inputs = pIOmap + etohl(ec_slave[slave].FMMU[FMMUc].LogStart);
						ec_slave[slave].Istartbit = ec_slave[slave].FMMU[FMMUc].LogStartbit;
						EC_PRINT("    Inputs %p startbit %d\n", ec_slave[slave].inputs, ec_slave[slave].Istartbit);
					}
					FMMUc++;
				}	
				ec_slave[slave].FMMUunused = FMMUc;
				diff = LogAddr - oLogAddr;
				oLogAddr = LogAddr;
				if ((segmentsize + diff) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
				{
					ec_group[group].IOsegment[currentsegment] = segmentsize;
					if (currentsegment < (EC_MAXIOSEGMENTS - 1))
					{
						currentsegment++;
						segmentsize = diff;	
					}
				}
				else
					segmentsize += diff;
			}

			ec_eeprom2pdi(slave); /* set Eeprom control to PDI */			
			ec_FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_SAFE_OP) , EC_TIMEOUTRET); /* set safeop status */
						
		}
		if (BitPos)
		{
			LogAddr++;
			oLogAddr = LogAddr;
			BitPos = 0;
			if ((segmentsize + 1) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
			{
				ec_group[group].IOsegment[currentsegment] = segmentsize;
				if (currentsegment < (EC_MAXIOSEGMENTS - 1))
				{
					currentsegment++;
					segmentsize = 1;	
				}
			}
			else
				segmentsize += 1;
		}	
		ec_group[group].IOsegment[currentsegment] = segmentsize;
		ec_group[group].nsegments = currentsegment + 1;
		ec_group[group].inputs = pIOmap + ec_group[group].Obytes;
		ec_group[group].Ibytes = LogAddr - ec_group[group].Obytes;
		if (ec_slave[slave].blockLRW)
			ec_group[group].blockLRW++;						
		ec_group[group].Ebuscurrent += ec_slave[slave].Ebuscurrent;
		if (!group)
		{	
			ec_slave[0].inputs = pIOmap + ec_slave[0].Obytes;
			ec_slave[0].Ibytes = LogAddr - ec_slave[0].Obytes; /* store input bytes in master record */
		}	

		EC_PRINT("IOmapSize %d\n", LogAddr - ec_group[group].logstartaddr);		
		return (LogAddr - ec_group[group].logstartaddr);
	}
	return 0;
}	

/** Map all PDOs from slaves to IOmap.
 *
 * @param[out] pIOmap	  = pointer to IOmap	
 * @return IOmap size
 */
int ec_config_map(void *pIOmap)
{
	return ec_config_map_group(pIOmap, 0);
}

/** Enumerate / map and init all slaves.
 *
 * @param[in] usetable	  = TRUE when using configtable to init slaves, FALSE otherwise
 * @param[out] pIOmap	  = pointer to IOmap	
 * @return Workcounter of slave discover datagram = number of slaves found
 */
int ec_config(uint8 usetable, void *pIOmap)
{
	int wkc;
	wkc = ec_config_init(usetable);
	if (wkc) 
		ec_config_map(pIOmap);
    return wkc;
}

/** Recover slave.
 *
 * @param[in] slave	  = slave to recover
 * @return >0 if successful
 */
int ec_recover_slave(uint16 slave)
{
	int rval;
	uint16 ADPh, configadr;

	rval = 0;
    configadr = ec_slave[slave].configadr;
	/* clear possible slaves at EC_TEMPNODE */
	ec_FPWRw(EC_TEMPNODE, ECT_REG_STADR, htoes(0) , 0);
    ADPh = (uint16)(1 - slave);
	/* set temporary node address of slave */
	if(ec_APWRw(ADPh, ECT_REG_STADR, htoes(EC_TEMPNODE) , EC_TIMEOUTRET) <= 0)
		return 0; /* slave fails to respond */
	
    ec_slave[slave].configadr = EC_TEMPNODE; /* temporary config address */	
	ec_eeprom2master(slave); /* set Eeprom control to master */			

	/* check if slave is the same as configured before */
	if ((ec_FPRDw(EC_TEMPNODE, ECT_REG_ALIAS, EC_TIMEOUTRET) == ec_slave[slave].aliasadr) &&
	    (ec_readeeprom(slave, ECT_SII_ID, EC_TIMEOUTEEP) == ec_slave[slave].eep_id) &&
	    (ec_readeeprom(slave, ECT_SII_MANUF, EC_TIMEOUTEEP) == ec_slave[slave].eep_man) &&
	    (ec_readeeprom(slave, ECT_SII_REV, EC_TIMEOUTEEP) == ec_slave[slave].eep_rev))
	{
		rval = ec_FPWRw(EC_TEMPNODE, ECT_REG_STADR, htoes(configadr) , EC_TIMEOUTRET);
	    ec_slave[slave].configadr = configadr;
	}
	else
	{
		/* slave is not the expected one, remove config address*/
		ec_FPWRw(EC_TEMPNODE, ECT_REG_STADR, htoes(0) , EC_TIMEOUTRET);
	    ec_slave[slave].configadr = configadr;
	}

	return rval;
}

/** Reconfigure slave.
 *
 * @param[in] slave	  = slave to reconfigure
 * @return Slave state
 */
int ec_reconfig_slave(uint16 slave)
{
	int state, nSM, FMMUc;
	uint16 configadr;
	
    configadr = ec_slave[slave].configadr;
	if (ec_FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_INIT) , EC_TIMEOUTRET) <= 0)
		return 0;
	state = 0;
	ec_eeprom2pdi(slave); /* set Eeprom control to PDI */			
	/* check state change init */
	state = ec_statecheck(slave, EC_STATE_INIT, EC_TIMEOUTSTATE);
	if(state == EC_STATE_INIT)
	{
		/* program all enabled SM */
		for( nSM = 0 ; nSM < EC_MAXSM ; nSM++ )
		{	
			if (ec_slave[slave].SM[nSM].StartAddr)
				ec_FPWR (configadr, ECT_REG_SM0 + (nSM * sizeof(ec_smt)), sizeof(ec_smt),
				    &ec_slave[slave].SM[nSM], EC_TIMEOUTRET);
		}
		/* program configured FMMU */
		for( FMMUc = 0 ; FMMUc < ec_slave[slave].FMMUunused ; FMMUc++ )
		{	
			ec_FPWR (configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc), sizeof(ec_fmmut), 
			    &ec_slave[slave].FMMU[FMMUc], EC_TIMEOUTRET);
		}
		ec_FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_PRE_OP) , EC_TIMEOUTRET);
		state = ec_statecheck(slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */
		if( state == EC_STATE_PRE_OP)
		{
			ec_FPWRw(configadr, ECT_REG_ALCTL, htoes(EC_STATE_SAFE_OP) , EC_TIMEOUTRET); /* set safeop status */
			state = ec_statecheck(slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE); /* check state change safe-op */
		}
	}
	return state;		
}
