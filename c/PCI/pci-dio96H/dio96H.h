/***************************************************************************
 Copyright (C) 2005-2015  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DIO96H, is free software; you can redistribute it
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

#ifndef DIO96H_H
#define DIO96H_H

/*
 * dio96H.h
 */

#define ADAPTER_ID "PCI-DIO96H.v1.9"
//#define DEBUG


/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x4) & 0x7)
#define PORT(x) (x & 0xf)

/* General defines */

#define TRUE  1
#define FALSE 0

/* Vendor and Device ID */ 
#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_DIO96H
#define PCI_DEVICE_ID_CBOARDS_DIO96H 0x0017
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_DIO96H_PCIe
#define PCI_DEVICE_ID_CBOARDS_DIO96H_PCIe 0x00da
#endif

#define MAX_BOARDS           1   /* Currently supports 1-4 boards. See Makefile */

/* Default device major number */
#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#define DIO_PORTS          12    /* twelve 8-bit  Digital I/O Ports           */

/* I/O Register locations */
/* NOTE: base_reg to be found dynamically at boot time. Not hard coded into driver */

/* BADR: These are 22 8-Bit Word Registers */
#define DIO_PORT0A           (base2)      /* Group 0 Port A 8 bit I/O of 8255 */
#define DIO_PORT0B           (base2+1)    /* Group 0Port B 8 bit I/O of 8255 */
#define DIO_PORT0C           (base2+2)    /* Group 0 Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_0      (base2+3)    /* Group 0 Mode and Direction Control Register */
#define DIO_PORT1A           (base2+4)    /* Group 1 Port A 8 bit I/O of 8255 */
#define DIO_PORT1B           (base2+5)    /* Group 1 Port B 8 bit I/O of 8255 */
#define DIO_PORT1C           (base2+6)    /* Group 1 Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_1      (base2+7)    /* Group 1 Mode and Direction Control Register */
#define DIO_PORT2A           (base2+8)    /* Group 2 Port A 8 bit I/O of 8255 */
#define DIO_PORT2B           (base2+9)    /* Group 2 Port B 8 bit I/O of 8255 */
#define DIO_PORT2C           (base2+0xa)  /* Group 2 Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_2      (base2+0xb)  /* Group 2 Mode and Direction Control Register */
#define DIO_PORT3A           (base2+0xc)  /* Group 3 Port A 8 bit I/O of 8255 */
#define DIO_PORT3B           (base2+0xd)  /* Group 3 Port B 8 bit I/O of 8255 */
#define DIO_PORT3C           (base2+0xe)  /* Group 3 Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_3      (base2+0xf)  /* Group 3 Mode and Direction Control Register */


/* Interrupt context data. Controls interrupt service handler */
typedef struct BOARD_REC {
  u32 pci_control_reg; /* BADR1 + 0x4c                */
  u16 base2;           /* Base2 address of DIO board  */
  u8 dio96H_reg_0;     /* status register   Group 0   */
  u8 dio96H_reg_1;     /* status register   Group 1   */
  u8 dio96H_reg_2;     /* status register   Group 2   */
  u8 dio96H_reg_3;     /* status register   Group 3   */
} BoardRec;

/* Values kept for each channel. */

typedef struct CHANNEL_REC {
  int  open;              /* Is channel device open()    */
  u16 value;              /* value of DIO Channel        */
  u16 addr;               /* address of the channel      */
  unsigned short f_flags; /* flags sent by open()        */
} ChanRec;

#endif
