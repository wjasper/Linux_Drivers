/***************************************************************************
 Copyright (C) 1999 - 2015  Warren J. Jasper  <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DIO48H, is free software; you can redistribute it
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

#ifndef DIO48H_H
#define DIO48H_H

/*
 * dio48H.h
 */

//#define DEBUG

#define ADAPTER_ID "PCI-DIO48H.v1.10"

/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x4) & 0x7)
#define PORT(x) (x & 0xf)

/* General defines */

#define TRUE  1
#define FALSE 0

#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_DIO48H
#define PCI_DEVICE_ID_CBOARDS_DIO48H 0x000b
#endif

#define MAX_BOARDS           2   /* Currently supports 1-4 boards. See Makefile */

/* Default device major number */
#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#define DIO_PORTS           6    /* 48 Digital I/O Ports (6 bytes)             */

/* I/O Register locations */

/* NOTE: base_reg to be found dynamically at boot time. Not hard coded into driver */

/* BADR1: These are 8 8-Bit Word Registers */
#define DIO_PORT0A         (base2)    /* Port A 8 bit I/O of 8255 */
#define DIO_PORT0B         (base2+1)  /* Port B 8 bit I/O of 8255 */
#define DIO_PORT0C         (base2+2)  /* Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_0    (base2+3)  /* Mode and Direction Control Register */
#define DIO_PORT1A         (base2+4)  /* Port A 8 bit I/O of 8255 */
#define DIO_PORT1B         (base2+5)  /* Port B 8 bit I/O of 8255 */
#define DIO_PORT1C         (base2+6)  /* Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_1    (base2+7)  /* Mode and Direction Control Register */

/* Interrupt context data. Controls interrupt service handler */
typedef struct BOARD_REC {
  u16 base2;          /* Base 2 address of DIO board  */
  u8 dio48H_reg_0;    /* status register Group 0      */
  u8 dio48H_reg_1;    /* status register Group 1      */
} BoardRec;

/* Values kept for each channel. */

typedef struct CHANNEL_REC {
  int open;               /* Is channel device open()    */
  u16 value;              /* value of DIO Channel        */
  u16 addr;               /* address of the channel      */
  unsigned short f_flags; /* flags sent by open()        */
} ChanRec;

#endif
