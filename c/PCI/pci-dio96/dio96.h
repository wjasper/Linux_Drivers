/***************************************************************************
 Copyright (C) 2005-2015  Warren J. Jasper
 All rights reserved.

 This program, PCI-DIO96, is free software; you can redistribute it
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

#ifndef DIO96_H
#define DIO96_H

/*
 * dio96.h
 */

#define ADAPTER_ID "PCI-DIO96.v1.7"
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

#ifndef PCI_DEVICE_ID_CBOARDS_DIO96
#define PCI_DEVICE_ID_CBOARDS_DIO96 0x0054
#endif

#define MAX_BOARDS           2   /* Currently supports 1-4 boards. See Makefile */

/* Default device major number */
#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#define DIO_PORTS          12    /* twelve 8-bit  Digital I/O Ports           */


/* I/O Register locations */

/* NOTE: base_reg to be found dynamically at boot time. Not hard coded into driver */

/* BADR1: This is 1 32-bit Word Registers */
#define PCI_INT_CSR        (base1+0x4c)

/* BADR3: These are 22 8-Bit Word Registers */
#define DIO_PORT0A           (base3)      /* Group 0 Port A 8 bit I/O of 8255 */
#define DIO_PORT0B           (base3+1)    /* Group 0Port B 8 bit I/O of 8255 */
#define DIO_PORT0C           (base3+2)    /* Group 0 Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_0      (base3+3)    /* Group 0 Mode and Direction Control Register */
#define DIO_PORT1A           (base3+4)    /* Group 1 Port A 8 bit I/O of 8255 */
#define DIO_PORT1B           (base3+5)    /* Group 1 Port B 8 bit I/O of 8255 */
#define DIO_PORT1C           (base3+6)    /* Group 1 Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_1      (base3+7)    /* Group 1 Mode and Direction Control Register */
#define DIO_PORT2A           (base3+8)    /* Group 2 Port A 8 bit I/O of 8255 */
#define DIO_PORT2B           (base3+9)    /* Group 2 Port B 8 bit I/O of 8255 */
#define DIO_PORT2C           (base3+0xa)  /* Group 2 Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_2      (base3+0xb)  /* Group 2 Mode and Direction Control Register */
#define DIO_PORT3A           (base3+0xc)  /* Group 3 Port A 8 bit I/O of 8255 */
#define DIO_PORT3B           (base3+0xd)  /* Group 3 Port B 8 bit I/O of 8255 */
#define DIO_PORT3C           (base3+0xe)  /* Group 3 Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_3      (base3+0xf)  /* Group 3 Mode and Direction Control Register */
#define COUNTER_1            (base3+0x10) /* 82C54 Counter Data 1 */
#define COUNTER_2            (base3+0x11) /* 82C54 Counter Data 2 */
#define COUNTER_CONFIG_REG   (base3+0x13) /* 82C54 Counter Configuration Register */
#define INT_CONTRL_1         (base3+0x14) /* Interrupt Sorce Configure */
#define INT_CONTRL_2         (base3+0x15) /* Interrupt Sorce Configure */


/* Interrupt context data. Controls interrupt service handler */
typedef struct BOARD_REC {
  u32 pci_control_reg; /* BADR1 + 0x4c                      */
  int IRQComplete;       /* 1 = IRQ Complete                  */
  int  irq;              /* IRQ inturrepts                    */
  u32 counter;           /* 32 bits of data for 82C54 Counter */
  u16 base1;             /* Base1 address of DIO board        */
  u16 base3;             /* Base3 address of DIO board        */
  u8  control_reg_1;     /* Interrupt Control 1               */
  u8  control_reg_2;     /* Interrupt Control 2               */
  u8 dio96_reg_0;        /* status register   Group 0         */
  u8 dio96_reg_1;        /* status register   Group 1         */
  u8 dio96_reg_2;        /* status register   Group 2         */
  u8 dio96_reg_3;        /* status register   Group 3         */
  u8 counter_config_reg; /* 82C54 counter configuration reg   */
} BoardRec;

/* Values kept for each channel. */

typedef struct CHANNEL_REC {
  int  open;              /* Is channel device open()    */
  int mode;               /* mode for 8255               */
  u16 value;              /* value of DIO Channel        */
  u16 addr;               /* address of the channel      */
  unsigned short f_flags; /* flags sent by open()        */
} ChanRec;


/*************************************************************************
 * PCI PLX-9052 Interrupt Status/Control BADR1+0x4c
 **************************************************************************/

#define INTE     0x1       /* Interrupt Enable     */
#define INTPOL   0x2       /* Interrupt Polarity   */
#define INT      0x4       /* Interrupt Status     */
#define PCIINT   0x40      /* PCI Interrupt Enable */
#define INTSEL   0x100     /* 0 = Level triggered, 1 = edge triggered mode */
#define INTCLR   0x400     /* Interrupt clear (edge triggered mode only)
			      0 = N/A, 1 = clear interrupt */
#define ISAMD    0x1000    /* 0 = ISA mode disabled, 1 = ISA mode enabled */


/*************************************************************************
 * 8255 Interrupt Source Configure Register 1 base3+0x14
 **************************************************************************/
#define DIRQ1 0x80   // When bit is set, 8255 in Group 3 will generate an interrupt on
                     // INTRB if INTEN is also set.
#define DIRQ0 0x40   // When bit is set, 8255 in Group 3 will generate an interrupt on
                     // INTRA if INTEN is also set.
#define CIRQ1 0x20   // When bit is set, 8255 in Group 2 will generate an interrupt on
                     // INTRB if INTEN is also set.
#define CIRQ0 0x10   // When bit is set, 8255 in Group 2 will generate an interrupt on
                     // INTRA if INTEN is also set.
#define BIRQ1 0x8    // When bit is set, 8255 in Group 1 will generate an interrupt on
                     // INTRB if INTEN is also set.
#define BIRQ0 0x4    // When bit is set, 8255 in Group 1 will generate an interrupt on
                     // INTRA if INTEN is also set.
#define AIRQ1 0x2    // When bit is set, 8255 in Group 0 will generate an interrupt on
                     // INTRB if INTEN is also set.
#define AIRQ0 0x1    // When bit is set, 8255 in Group 0 will generate an interrupt on
                     // INTRA if INTEN is also set.

/*************************************************************************
 * 8255 Interrupt Source Configure  Register 2 base3+0x14
 **************************************************************************/
#define INTEN   (0x4)  /* Enables or disables interrupts. 1 = enabled,  0 = disabled  */
#define CTRIR   (0x2)  /* Enables or disables coutners as interupt source. 
                          1 = counters may generate interrutps.
                          0 = counters can not generate interrutps.                   */
#define CTR1     (0x1) /* Controls whether counter 2 is the interrupt source,
		          or counter 1 is the interrupt source.  When CTR1 is
		          set to 1, the interrupt source is counter 2 and counter 1
		          is the prescaler for counter 2.  When CTR1 is set to 0, the
		          interrupt source is counter 1. (Counter 3 is not used).     */
#endif
