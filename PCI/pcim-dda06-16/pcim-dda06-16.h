/***************************************************************************
 Copyright (C) 2020  Warren J. Jasper
 All rights reserved.

 This program, PCIM-DDA06-16, is free software; you can redistribute it
 and/or modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation; either version 2 of the
 License, or (at your option) any later version, provided that this
 copyright notice is preserved on all copies.

 ANY RIGHTS GRANTED HEREUNDER ARE GRANTED WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, AND FURTHER,
 THERE SHALL BE NO WARRANTY AS TO CONFORMITY WITH ANY USER MANUALS OR
 OTHER LITERATURE PROVIDED WITH SOFTWARE OR THAM MY BE ISSUED FROM TIME
 TO TIME. IT IS PROVIDED SOLELY "AS IS".

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
***************************************************************************/

#ifndef PCIM_DDA06_16_H
#define PCIM_DDA06_16_H

/*
 * pcim-dda06-16.h
 */

/*  ioctl() values */

#include <linux/ioctl.h>
#define IOCTL_MAGIC 'w'

#define DIO_SET_DIRECTION        _IO(IOCTL_MAGIC, 1)
#define DAC_SET_GAINS            _IO(IOCTL_MAGIC, 2)          // Jumper selectable
#define DAC_GET_GAINS            _IOR(IOCTL_MAGIC, 3, long)
#define DAC_SET_SIMULT           _IO(IOCTL_MAGIC, 4)          // Jumper selectable
#define DAC_GET_SIMULT           _IOR(IOCTL_MAGIC, 5, long)   

#define IOCTL_MAXNR 5         /* maxinum ordinal number */

/* digital output */
#define DIO_0 0x1
#define DIO_1 0x2
#define DIO_2 0x4
#define DIO_3 0x8
#define DIO_4 0x10
#define DIO_5 0x20
#define DIO_6 0x40
#define DIO_7 0x80

/* Digital I/O Modes */ 
#define MODE_IO            0
#define MODE_STROBE_IO     1
#define MODE_BIDIRECTIONAL 2

/* Digital I/O Direction Settings */
#define PORT_INPUT         1
#define PORT_OUTPUT        0
#define HIGH_PORT_INPUT    2
#define HIGH_PORT_OUTPUT   3
#define LOW_PORT_INPUT     4
#define LOW_PORT_OUTPUT    5

/* Analog Output  Ranges */
#define BP_5_0V   (0x2 << 6)  /* +/- 5.0V  */
#define BP_10_0V  (0x3 << 6)  /* +/- 10.0V */
#define UP_5_0V   (0x6 << 6)  /* 0 - 5.0V  */
#define UP_10_0V  (0x7 << 6)  /* 0 - 10.0V */


#endif
