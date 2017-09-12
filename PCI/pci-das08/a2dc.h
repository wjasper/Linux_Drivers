/***************************************************************************
 Copyright (C) 2007-2009  Warren Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DAS08, is free software; you can redistribute it
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
 * a2dc.h
 */

#ifndef A2DC_H
#define A2DC_H

#define ADAPTER_ID "PCI-DAS08.v2.11"
// #define DEBUG 

/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x4) & 0x7)
#define CHAN(x)  (x & 0xf)
#define ALIGN_ADDRESS(addr, alignment) \
        ((((u32)(addr)) + (((u32)(alignment))-1)) & ~(((u32)(alignment)) - 1))


/* General defines */
#define TRUE  1
#define FALSE 0
#define MAX_COUNT 1024

/*  change to get bigger buf size.  Max value is 0x100000  */
#define ADC_BUFF_PHY_SIZE   0x10000       /* Physical size of the ADCs buffer  */

/* Default device major number */
#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS    1           /* Suppport 1 board for now */
#endif

#define AD_CHANNELS         8    /* 8 Single-Ended A/D Chan                       */ 
#define BADR1_SIZE       0x80    /* region of the PCI 9052 Controller             */
#define BADR2_SIZE       0x8     /* 8 bytes of General Control/Status  registers  */

/* Vendor and Device ID */ 
#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_DAS08
#define PCI_DEVICE_ID_CBOARDS_DAS08 0x0029
#endif

/* I/O Register locations */

/* NOTE: base1 and base2 to be found dynamically at boot time. Not hard coded into driver */

/* BADR1: These are 32-bit Word Registers */
#define PCI_9052_INTERRUPT      (base1+0x4c) /* Interrupt status and control */
#define PCI_9052_SOURCE_SELECT  (base1+0x50) /* Interrupt source select      */

/* BADR2: These are 8 8-Bit Word Registers */
#define LSB_AND_CHNLS    (base2)    /* A/D Data & Channel Register      */
#define MSB_DATA_BYTE    (base2+1)    
#define STATUS_REG       (base2+2)  /* Channel Mux Scan Limits Register */
#define COUNTER_0_DATA   (base2+4)  /* Counter 0 Register               */
#define COUNTER_1_DATA   (base2+5)  /* Counter 1 Register               */
#define COUNTER_2_DATA   (base2+6)  /* Counter 2 Register               */
#define COUNTER_CONTROL  (base2+7)  /* Conter Control  Register         */

/* Interrupt context data. Controls interrupt service handler */
/* Values kept for each channel. */

typedef struct ADC_CHANNEL_REC {
  int  open;            /* Is channel device open()                 */
  long count;           /* Number of samples requested              */
  u8  mode;             /* 0 =  Soft Trigger                        */
                        /* 2 = External Trigger                     */
  u8 chan;              /* Mux number                               */
} ADC_ChanRec;

typedef struct BOARD_REC {
  struct pci_dev *pdev;          // pointer to pci device struct
  u16 base1;                     // Base address of ADC board
  u16 base2;                     // Base address of ADC board
  int  busy;                     // busy = TRUE, free = FALSE
  int  irq;                      // board irq
  u8 status;                     // Status BYTE
  int wordsToRead;               // Words left to read in the DAQ
  int IRQComplete;
  u32 buf_phy_size;              // size in bytes of the buffer for this board
  dma_addr_t buf_bus_addr;       // bus address of the DMA buffer for this board
  u16 *buf_virt_addr;            // virtual kernal (cpu) address of dma buffer
  u16 *ADC_KernBuffPtr;          // pointer into the data buffer array
  ADC_ChanRec *ADC_CurrChan;     // Pointer to current channel
  struct timer_list das08_timer; // kernel timer (in case we don't return from the ISR 
} BoardRec;


/*************************************************************************
*          base1+0x4c                PCI_9052_INTERRUPT                  *
**************************************************************************/
#define INTE    0x1  /* Interrrupt Enable:  disable=0, enable=1 */
#define INT     0x4  /* Interrupt Status:  not active = 0, interrupt active=1 */
#define PCIINT  0x40 /* PCI Interrupt Enable: disable=0, enable=1 */
#define SW_INT 0x80  /* 1 = generate a sofware interrupt */
#define INTCLR 0x400 /* Clear interrupt */

/*************************************************************************
*          base1+0x50                PCI_9052_SOURCE_SELECT              *
**************************************************************************/
#define OUT0   0x4  /* 0 = Interrupt input user connector, 1 = Counter 2 ouptput */

/*************************************************************************
* STATUS_REG         base2+0x2              Status Register              *
**************************************************************************/

/* Read */
#define MUX0  0x01      /* Current multiplexor channel */
#define MUX1  0x02      /* Current multiplexor channel */
#define MUX2  0x04      /* Current multiplexor channel */
#define IRQ   0x08      /* 1 = positive edge detected  */
#define IP1   0x10      /* digial input line IP1       */
#define IP2   0x20      /* digial input line IP2       */
#define IP3   0x40      /* digial input line IP3       */
#define EOC   0x80      /* 1=A/D busy, 0=not busy      */

/* Write */
#define INTE2 0x08     /* 1=enable interrupts, 0=disable */
#define OP1   0x10     /* digital output line OP1        */
#define OP2   0x20     /* digital output line OP2        */
#define OP3   0x40     /* digital output line OP3        */
#define OP4   0x80     /* digital output line OP4        */


/*************************************************************************
* Constants for dealing with the 8254 counters                           *
**************************************************************************/

#define MODE0 0x0
#define MODE1 0x2
#define MODE2 0x4
#define MODE3 0x6
#define MODE4 0x8
#define MODE5 0xa

#define C0 0x00
#define C1 0x40
#define C2 0x80

#define CNTLATCH 0x00
#define LSBONLY  0x10
#define MSBONLY  0x20
#define LSBFIRST 0x30

#define S0       0x00
#define S1       0x02

#endif
