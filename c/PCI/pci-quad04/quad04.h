/***************************************************************************
 Copyrig (C) 2010-2015  Warren J. Jasper
 All rights reserved.

 This program, PCI-QUAD04, is free software; you can redistribute it
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

/*
 * quad04.h  Genearal header file for the PCI-QUAD04
 */

#ifndef QUAD04_H
#define QUAD04_H

#define DEBUG 1
#define  ADAPTER_ID "PCI-QUAD04.v1.4"

/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x3) & 0x7) /* 0-7 boards    */
#define CHAN(x) (x & 0x7)           /* 0-3 channels  */

/* Vendor and Device ID */
#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_QUAD04
#define PCI_DEVICE_ID_CBOARDS_QUAD04 0x004d
#endif

#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS    4   /* Suppport 4 boards for now */
#endif

#define NCOUNTERS     4    /* Number of counters per chip */

/* General Defines */
#define TRUE  1
#define FALSE 0

/* PLX-9052,  LS7266R1 and 8259 Programmable Interrupt Registers */
#define INTCSR_REG (base1+0x4c) // Interrupt Status/Control Reg for PLX-9052
#define DATA1_REG (base2)       // Read channel 1 OL byte segment addressed by BP
#define CMD1_REG (base2+1)      // Channel 1 FLAG, RLD, CMR, IOR, IDR registers
#define DATA2_REG (base2+2)     // Read channel 2 OL byte segment addressed by BP
#define CMD2_REG (base2+3)      // Channel 2 FLAG, RLD, CMR, IOR, IDR registers
#define DATA3_REG (base2+4)     // Read channel 3 OL byte segment addressed by BP
#define CMD3_REG (base2+5)      // Channel 3 FLAG, RLD, CMR, IOR, IDR registers
#define DATA4_REG (base2+6)     // Read channel 4 OL byte segment addressed by BP
#define CMD4_REG (base2+7)      // Channel 4 FLAG, RLD, CMR, IOR, IDR registers
#define IRCR_REG (base2+8)      // Index and Interrupt Routing Control Register
#define ISCR_REG (base2+9)      // Input Signal Control Register
#define PIC_A_REG (base2+10)    // 8259A Interrupt Controller Register Part A
#define PIC_B_REG (base2+11)    // 8259A Interrupt Controller Register Part B

/*
 * --------------------------------------------------------------------
 * BAR1 ( Local Space 1) registers.
 * --------------------------------------------------------------------
 */
// BAR1 + 0x4c Interrupt Status / Interrupt Control
#define INTE    0x001        // Interrupt Enable: 0 = disabled (default), 1 = enabled.
#define INTPOL  0x002        // Interrupt Polarity: 0 active low (default), 1 = active high.
#define INT     0x004        // Interrupt Status: 0 = interrupt not active, 1 = interrupt active.
#define PCINT   0x040        // PCI Interrupt Enable: 0 = disabled (default), 1 = enabled.
#define INTSEL  0x100        // Interrupt select bit: 0 = level sensitive (default), 1 = edge sensitive.
                             // Note that this bit only has an effect when in high polarity mode.  The
			     // interrupt is always level-sensitive when low polarity is selected.
#define INTCLR  0x400        // Clear the Interrupt when in edge-triggered triggered configuration.


/**************************************
 * Input Signal Control (SIGCTRL_REG) *
 **************************************/
#define CONFIG_0 0x00  // (4) 24-bit counters (1/2/3/4) (default)
#define CONFIG_1 0x53  // (2) 48-bit counters (1-2/3-4)
#define CONFIG_2 0x3c  // (1) 24-bit / (1) 72-bit counter (1/2-3-4)
#define CONFIG_3 0x3f  // (1) 96-bit counter (1-2-3-4)

typedef struct BOARD_REC {
  int busy;            /* busy = TRUE, free = FALSE              */
  int irq;             /* board irq                              */
  u16 base1;           /* Base address region 1                  */
  u16 base2;           /* Base address region 2                  */
  u8 mode;             /* Mode for cascaded counters             */
  u8 ircr;             /* Index and Interrupt Routing Control    */
  u8 iscr;             /* Input Signal Control Register          */
  u8 pic_A;            /* Programmable Interrupt Control Port A  */
  u8 pic_B;            /* Programmable Interrupt Control Port B  */
} BoardRec;

/* Values kept for each counter. */
typedef struct COUNTER_REC {
  int open;             /* Is channel device open()         */
  u32 OL;               /* 24 bit Output Latch data         */
  u32 PR;               /* 24 bit preset value              */
  u16 data_port;        /* address of data port             */
  u16 cmd_port;         /* address of command port          */
  u8 cmr;               /* Counter Mode Register            */
  u8 ior;               /* Input/Output Control Register    */
  u8 idr;               /* Index Contgrol Register          */
  u8 flag;              /* flag register                    */
  u8 counter;           /* counter number (0 base)          */
} CounterRec;

#endif
