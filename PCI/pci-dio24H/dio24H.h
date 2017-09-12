/***************************************************************************
 Copyright (C) 1999 - 2015  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DIO24H, is free software; you can redistribute it
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

#ifndef DIO24H_H
#define DIO24H_H

/*
 * dio24H.h
 */

//#define DEBUG
#define ADAPTER_ID "PCI-DIO24H.v1.11"

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

#ifndef PCI_DEVICE_ID_CBOARDS_DIO24H
#define PCI_DEVICE_ID_CBOARDS_DIO24H 0x0014
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS      1   /* Currently supports 1-4 boards. See Makefile */
#endif

/* Default device major number */
#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#define DIO_PORTS           3    /* 24 Digital I/O Ports (3 bytes)             */

/* I/O Register locations */

/* NOTE: base_reg to be found dynamically at boot time. Not hard coded into driver */

/* BADR1: These are 2 32-bit Word Registers */
#define INTERRUPT_REG        (base1+0x4c)
#define USER_IO_STATUS_REG   (base1+0x50)

/* BADR2: These are 4 8-Bit Word Registers */
#define DIO_PORTA           (base2)    /* Port A 8 bit I/O of 8255 */
#define DIO_PORTB           (base2+1)  /* Port B 8 bit I/O of 8255 */
#define DIO_PORTC           (base2+2)  /* Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG       (base2+3)  /* Mode and Direction Control Register */


/* Interrupt context data. Controls interrupt service handler */
typedef struct BOARD_REC {
  u32 pci_control_reg; /* BADR1 + 0x4c                */
  int IRQComplete;     /* 1 = IRQ Complete            */
  int  irq;            /* IRQ inturrepts              */
  u16 base1;           /* Base1 address of DIO board  */
  u16 base2;           /* Base2 address of DIO board  */
  u8 dio24H_reg;       /* status register             */
} BoardRec;

/* Values kept for each channel. */

typedef struct CHANNEL_REC {
  int  open;              /* Is channel device open()    */
  u16 value;              /* value of DIO Channel        */
  u16 addr;               /* address of the channel      */
  int mode;               /* mode for 8255               */
  unsigned short f_flags; /* flags sent by open()        */
} ChanRec;


/*************************************************************************
 * PCI 9050 Interrupt Status/Control BADR1+0x4c
 **************************************************************************/

#define INTE     0x1       /* Interrupt Enable     */
#define INTPOL   0x2       /* Interrupt Polarity   */
#define INT      0x4       /* Interrupt Status     */
#define PCIINT   0x40      /* PCI Interrupt Enable */
#define INTSEL   0x100     /* Interrupt Edge/Level Select:
			      0 = level-sensitive (default), 1 = edge-sensitive               */
#define INTCLR   0x400     /* Clears the INT selected interrupt status
			      0 = no effect,  1 = clear the INT interupt                      */

/*************************************************************************
 * PCI 9050 Interrupt Status/Control BADR1+0x50
 **************************************************************************/
#define OUT0   0x4    /* 0=10 Mhz, 1=1Mhz */

#endif
