/***************************************************************************
 Copyright (C) 2003-2015  Warren Jasper
 All rights reserved.

 This program, PCI-CTR05, is free software; you can redistribute it
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
 * c9513.h  Genearal header file for the PCI-CTR05
 */

#ifndef C9513_H
#define C9513_H

//#define DEBUG 1 
#define  ADAPTER_ID "PCI-CTR05.v1.10"

/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x4) & 0x7)
#define CHAN(x) (x & 0x7)           /* 0-5 channels            */

/* Vendor and Device ID */
#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307 
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_CTR05
#define PCI_DEVICE_ID_CBOARDS_CTR05 0x0018 
#endif

#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS    1   /* Suppport 1 board for now */
#endif

#define NUM_COUNTERS  5    /* 5 counters on the chip      */
#define NCOUNTERS     5    /* Number of counters per chip */
#define NUM_DIO       1    /* 1 DIO ports on the board    */

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
#define DATA_REG      (base2)        /* 9513 Data Register    */
#define CMD_REG       (base2 + 1)    /* 9513 Command Register */
#define DIO_IN        (base2 + 2)    /* Digital Input Port    */
#define DIO_OUT       (base2 + 3)    /* Digital Output Port   */

/*
 * --------------------------------------------------------------------
 * BAR1 ( Local Space 1) registers.
 *
 * --------------------------------------------------------------------
 */

// BAR1 + 0x4c Interrupt Status / Interrupt Control
#define INTE    0x001        // Interrupt Enable: 0 = disabled (default), 1 = enabled.
#define INTPOL  0x002        // Interrupt Polarity: 0 active low (default), 1 = active high.
#define INT     0x004        // Interrupt Status: 0 = interrupt not active, 1 = interrupt active.
#define PCINT   0x040        // PCI Interrupt Enable: 0 = disabled (default), 1 = enabled.
#define SW_INT  0x080        // A value of 1 generates a sofware interrupt.
#define INTSEL  0x100        // Interrupt select bit: 0 = level sensitive (default), 1 = edge sensitive.
                             // Note that this bit only has an effect when in high polarity mode.  The
			     // interrupt is always level-sensitive when low polarity is selected.
#define INTCLR  0x400        // Clear the Interrupt when in edge-triggered triggered configuration.

// BAR1 + 0x50 User I/O Status and Control
#define OUT0   0x4          // 0: input = 5 MHz (default), 1: input = 1 MHz.

/* Data structures for the ctr05 board */

typedef struct BOARD_REC {
  int busy;            /* busy = TRUE, free = FALSE              */
  int irq;             /* board irq                              */
  int useCounter;      /* counts the nubmer of freq counters     */
  u16 base1;           /* Base address region 1                  */
  u16 base2;           /* Base address region 2                  */
  u16 gateInterval;    /* time in ms that the counter will count */
  u16 mm;              /* Master Mode Control Register           */  
} BoardRec;

/* Values kept for each channel. */

typedef struct COUNTER_REC {
  int  open;            /* Is channel device open()         */
  int mode;             /* Counter of Frequency mode        */
  u32 count;            /* Number of samples requested      */
  u16 cm;               /* Counter Mode Register            */
  u16 hr;               /* Counter Hold Register            */
  u16 lr;               /* Counter Load Register            */

  u8 chan;              /* Counter number                   */
} CounterRec;

typedef struct DIO_CHANNEL_REC {
  int  open;              /* Is channel device open()    */
} DIO_ChanRec;


#endif
