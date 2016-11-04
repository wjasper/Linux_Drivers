/***************************************************************************
 Copyright (C) 2003-2015  Warren Jasper
 All rights reserved.

 This program, PCI-CTR20HD, is free software; you can redistribute it
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
 * c9513.h  Genearal header file for the PCI-CTR20HD
 */

#ifndef C9513_H
#define C9513_H

//#define DEBUG 1
#define  ADAPTER_ID "PCI-CTR20HD.v1.5"

/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x8) & 0x7) 
#define CHIP(x) ((x >> 0x3) & 0x3)  /* up to 4 chips per board */
#define CHAN(x) (x & 0x7)           /* 0-5 channels            */

/* Vendor and Device ID */
#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307 
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_CTR20HD
#define PCI_DEVICE_ID_CBOARDS_CTR20HD 0x0074
#endif

#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS       1       /* Suppport 1 board for now */
#endif

#define NCHIP            4       /* 4 CTS9513-2 chips           */
#define NCOUNTERS        5       /* Number of counters per chip */

/* General Defines */
#define TRUE  1
#define FALSE 0

/* 9513 counter command codes */
#define ARM           0x20
#define LOAD          0x40
#define LOAD_ARM      0x60
#define DISARM_LATCH  0x80
#define CTR_LATCH     0xa0
#define DISARM        0xc0
#define SET_OUTPUT    0xe8
#define CLEAR_OUTPUT  0xe0
#define MASTER_RESET  0xff

/* Select Registers */
#define MODE_REG      0x0
#define LOAD_REG      0x8
#define HOLD_REG      0x10

/* Counters */
#define CTR1 1
#define CTR2 2
#define CTR3 3
#define CTR4 4
#define CTR5 5

/* Counter Select bits */
#define CTR1_BIT    0x1
#define CTR2_BIT    0x2
#define CTR3_BIT    0x4
#define CTR4_BIT    0x8
#define CTR5_BIT    0x10

/* I/O Register locations */
#define INTERRUPT_REG (base1 + 0x4c) /* Interrupt Status and Control */
#define DATA_REG_A    (base2 + 0)    /* 9513_A Data Register        */
#define CMD_REG_A     (base2 + 1)    /* 9513_A Command Register     */
#define DATA_REG_B    (base2 + 2)    /* 9513_B Data Register        */
#define CMD_REG_B     (base2 + 3)    /* 9513_B Command Register     */
#define DATA_REG_C    (base2 + 4)    /* 9513_C Data Register        */
#define CMD_REG_C     (base2 + 5)    /* 9513_C Command Register     */
#define DATA_REG_D    (base2 + 6)    /* 9513_D Data Register        */
#define CMD_REG_D     (base2 + 7)    /* 9513_D Command Register     */
#define CNT_AB_INT    (base2 + 8)    /* Counter A&B Source Select INT Pol */
#define CNT_CD_INT    (base2 + 9)    /* Counter C&D Source Select INT Pol */

/*
 * --------------------------------------------------------------------
 * BAR1 (Local Space 1) registers.
 *
 * --------------------------------------------------------------------
 */

// BAR1 + 0x4c Interrupt Status / Interrupt Control
#define INTE_AB   0x001     // Interrupt_AB Enable: 0 = disabled (default), 1 = enabled.
#define INTPOL_AB 0x002     // Interrupt_A Polarity: 0 active low (default), 1 = active high.
                            // This bit should always be set to 1 for this product.
                            // Actual interrupt polarity is determined via the INTAB_POL bit in BADR2+8.
#define INT_AB    0x004     // Interrupt_AB Status: 0 = interrupt not active, 1 = interrupt active.
#define INTE_CD   0x008     // Interrupt_CD Enable: 0 = disabled (default), 1 = enabled.
#define INTPOL_CD 0x010     // Interrupt_CD Polarity: 0 active low (default), 1 = active high.
                            // This bit should always be set to 1 for this product.
                            // Actual interrupt polarity is determined via the INTCD_POL bit in BADR2+9.
#define INT_CD    0x020     // Interrupt_B Status: 0 = interrupt not active, 1 = interrupt active.
#define PCINT     0x040     // PCI Interrupt Enable: 0 = disabled (default), 1 = enabled.
#define SW_INT    0x080     // A value of 1 generates a sofware interrupt.
#define INTSEL_AB 0x100     // Interrupt_AB select bit: 0 = level sensitive (default), 1 = edge sensitive.
                            // Note that this bit only has an effect when in high polarity mode.  The
			    // interrupt is always level-sensitive when low polarity is selected.
#define INTSEL_CD 0x200     // Interrupt_B select bit: 0 = level sensitive (default), 1 = edge sensitive.
                            // Note that this bit only has an effect when in high polarity mode.  The
			    // interrupt is always level-sensitive when low polarity is selected.
#define INTCLR_AB 0x400     // Clear the Interrupt_AB when in edge-triggered triggered configuration.
#define INTCLR_CD 0x800     // Clear the Interrupt_B when in edge-triggered triggered configuration.

#define INTAB_POL 0x80
#define INTCD_POL 0x80

/* Data structures for the ctr10 board */

typedef struct BOARD_REC {
  struct pci_dev *pdev;      /* pointer to pci device struct                */
  int busy;                  /* busy = TRUE, free = FALSE                   */
  int irq;                   /* board irq                                   */
  int useCounter[NCHIP];     /* counts the number of freq counters          */
  u16 base1;                 /* Base address region 1                       */
  u16 base2;                 /* Base address region 2                       */
  u16 gateInterval[NCHIP];   /* time in ms that the counter will count      */
  u16 mm[NCHIP];             /* Master Mode Control                         */
  u8  clockSelectReg_AB;     /* Counters A & B Clock Source Select Register */
  u8  clockSelectReg_CD;     /* Counters C & D Clock Source Select Register */
} BoardRec;

/* Values kept for each channel. */

typedef struct COUNTER_REC {
  int open;             /* Is channel device open()         */
  u32 count;            /* Number of samples requested      */
  u16 cm;               /* Counter Mode Register            */
  u16 hr;               /* Counter Hold Register            */
  u16 lr;               /* Counter Load Register            */
  u8 mode;              /*                                  */
  u8 chan;              /* Counter number                   */
} CounterRec;

#endif
