/*
 * Simple Open EtherCAT Master Library 
 *
 * File    : nicdrv.h
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
 * Headerfile for nicdrv.c 
 */

#ifndef _nicdrvh_
#define _nicdrvh_

extern ec_bufT ec_rxbuf[EC_MAXBUF];
extern ec_bufT ec_txbuf[EC_MAXBUF];
extern ec_bufT ec_txbuf2;
extern int ec_txbuflength[EC_MAXBUF];
extern int ec_txbuflength2;
extern int ec_incnt;
extern int ec_errcnt;
extern int ec_redstate;

extern int hlp_txtime;
extern int hlp_rxtime;

extern int sockhandle, sockhandle2;

extern const uint16 priMAC[3];
extern const uint16 secMAC[3];

int ec_setupnic(const char * ifname, int secondary);
int ec_closenic(void);
void ec_setupheader(void *p);
void ec_setbufstat(uint8 idx, int bufstat);
uint8 ec_getindex(void);
int ec_outframe(uint8 idx, int sock);
int ec_outframe_red(uint8 idx);
int ec_waitinframe(uint8 idx, int timeout);
int ec_srconfirm(uint8 idx,int timeout);

#endif
