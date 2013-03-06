/*
 * Simple Open EtherCAT Master Library 
 *
 * File    : ethercatbase.h
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
 * Headerfile for ethercatbase.c 
 */

#ifndef _ethercatbase_
#define _ethercatbase_

int ec_setupdatagram(void *frame, uint8 com, uint8 idx, uint16 ADP, uint16 ADO, uint16 length, void *data);
int ec_adddatagram(void *frame, uint8 com, uint8 idx, boolean more, uint16 ADP, uint16 ADO, uint16 length, void *data);
int ec_BWR(uint16 ADP,uint16 ADO,uint16 length,void *data,int timeout);
int ec_BRD(uint16 ADP,uint16 ADO,uint16 length,void *data,int timeout);
int ec_APRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ec_ARMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ec_FRMW(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
uint16 ec_APRDw(uint16 ADP, uint16 ADO, int timeout);
int ec_FPRD(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
uint16 ec_FPRDw(uint16 ADP, uint16 ADO, int16 timeout);
int ec_APWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout);
int ec_APWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ec_FPWRw(uint16 ADP, uint16 ADO, uint16 data, int timeout);
int ec_FPWR(uint16 ADP, uint16 ADO, uint16 length, void *data, int timeout);
int ec_LRW(uint32 LogAdr, uint16 length, void *data, int timeout);
int ec_LRD(uint32 LogAdr, uint16 length, void *data, int timeout);
int ec_LWR(uint32 LogAdr, uint16 length, void *data, int timeout);
int ec_LRWDC(uint32 LogAdr, uint16 length, void *data, uint16 DCrs, int64 *DCtime, int timeout);

#endif
