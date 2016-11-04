/***************************************************************************
 Copyright (C) 2003-2015  Warren Jasper
 All rights reserved.

 This program, PCI-CTR10, is free software; you can redistribute it
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
 * c9513.h  Genearal header file for the PCI-CTR10
 */

#ifndef C9513_H
#define C9513_H

// #define DEBUG 1
#define  ADAPTER_ID "PCI-CTR10.v1.10"

/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x8) & 0x7) 
#define CHIP(x)  ((x >> 0x3) & 0x3)  /* up to 4 chips per board */
#define CHAN(x)  (x & 0x7)           /* 0-5 channels            */

/* Vendor and Device ID */
#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307 
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_CTR10
#define PCI_DEVICE_ID_CBOARDS_CTR10 0x006e
#endif

#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS       1       /* Suppport 1 board for now */
#endif

#define NCHIP            2       /* 2 CTS9513-2 chips           */
#define NCOUNTERS        5       /* Number of counters per chip */
#define NUM_DIO          2       /* 2 DIO ports on the board    */

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
#define USER_STATUS   (base1 + 0x50) /* User I/0 Status and Control */
#define DATA_REG_A    (base2)        /* 9513_A Data Register        */
#define CMD_REG_A     (base2 + 1)    /* 9513_A Command Register     */
#define DIO_IN_A      (base2 + 2)    /* Digital Input Port A        */
#define DIO_OUT_A     (base2 + 3)    /* Digital Output Port A       */
#define DATA_REG_B    (base2 + 4)    /* 9513_B Data Register        */
#define CMD_REG_B     (base2 + 5)    /* 9513_B Command Register     */
#define DIO_IN_B      (base2 + 6)    /* Digital Input Port B        */
#define DIO_OUT_B     (base2 + 7)    /* Digital Output Port B       */

/*
 * --------------------------------------------------------------------
 * BAR1 ( Local Space 1) registers.
 *
 * --------------------------------------------------------------------
 */

// BAR1 + 0x4c Interrupt Status / Interrupt Control
#define INTE_A   0x001     // Interrupt_A Enable: 0 = disabled (default), 1 = enabled.
#define INTPOL_A 0x002     // Interrupt_A Polarity: 0 active low (default), 1 = active high.
#define INT_A    0x004     // Interrupt_A Status: 0 = interrupt not active, 1 = interrupt active.
#define INTE_B   0x008     // Interrupt_B Enable: 0 = disabled (default), 1 = enabled.
#define INTPOL_B 0x010     // Interrupt_B Polarity: 0 active low (default), 1 = active high.
#define INT_B    0x020     // Interrupt_B Status: 0 = interrupt not active, 1 = interrupt active.

#define PCINT    0x040     // PCI Interrupt Enable: 0 = disabled (default), 1 = enabled.
#define SW_INT   0x080     // A value of 1 generates a sofware interrupt.
#define INTSEL_A 0x100     // Interrupt_A select bit: 0 = level sensitive (default), 1 = edge sensitive.
                           // Note that this bit only has an effect when in high polarity mode.  The
			   // interrupt is always level-sensitive when low polarity is selected.
#define INTSEL_B 0x200     // Interrupt_B select bit: 0 = level sensitive (default), 1 = edge sensitive.
                           // Note that this bit only has an effect when in high polarity mode.  The
			   // interrupt is always level-sensitive when low polarity is selected.
#define INTCLR_A 0x400     // Clear the Interrupt_A when in edge-triggered triggered configuration.
#define INTCLR_B 0x800     // Clear the Interrupt_B when in edge-triggered triggered configuration.

/* 
 BAR1 + 0x50 User I/O Status and Control    
  |-------------------------------------------------------------|
  |    OUT1  |   OUT0   |    Clock          |      Source       |
  |------------------------------------------------------------ |
  |     0    |    0     |  5 MHz (default)  |   10 MHz Xtal/2   |
  |     0    |    1     |   1 MHz           |   10 MHz Xtal/10  |
  |     1    |    0     |   3.33 MHz        |   PCI 33 MHz/10   |
  |     1    |    1     |   1.67 MHz        |   PCI 33 MHz/20   |
  |------------------------------------------------------------ |
*/

#define OUT0   0x04         
#define OUT1   0x02         

/* Data structures for the ctr10 board */

typedef struct BOARD_REC {
  struct pci_dev *pdev;    /* pointer to pci device struct           */
  int busy;                /* busy = TRUE, free = FALSE              */
  int irq;                 /* board irq                              */
  int useCounter[NCHIP];   /* counts the number of freq counters     */
  u16 base1;               /* Base address region 1                  */
  u16 base2;               /* Base address region 2                  */
  u16 gateInterval[NCHIP]; /* time in ms that the counter will count */
  u16 mm[NCHIP];           /* Master Mode Control                    */  
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

typedef struct DIO_CHANNEL_REC {
  int  open;              /* Is channel device open()    */
} DIO_ChanRec;


#endif
