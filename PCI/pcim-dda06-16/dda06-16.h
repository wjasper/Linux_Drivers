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

#ifndef DDA06_16_H
#define DDA06_16_H

/*
 * dda06-16.h
 */

//#define DEBUG
#define ADAPTER_ID "PCI-DDA06-16.v0.1"

/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x4) & 0x7)
#define PORT(x)  (x & 0xf)

/* General defines */

#define TRUE  1
#define FALSE 0

#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_DDA06_16
#define PCI_DEVICE_ID_CBOARDS_DDA06_16 0x0053
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS      1   /* Currently supports 1-4 boards. See Makefile */
#endif

/* Default device major number */
#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif

#define DIO_PORTS           3    /* 24 bits Digital I/O Ports (6 bytes) */
#define DAC_PORTS           6    /* Maximum number of DAC Channels */

/* I/O Register locations */

/* NOTE: base_reg to be found dynamically at boot time. Not hard coded into driver */

/* BADR3: 16 8-Bit Word Registers */
#define DAC0_LSB_REG      (base3+0)    /* D/A 0 Data LSB */
#define DAC0_MSB_REG      (base3+1)    /* D/A 0 Data MSB */
#define DAC1_LSB_REG      (base3+2)    /* D/A 1 Data LSB */
#define DAC1_MSB_REG      (base3+3)    /* D/A 1 Data MSB */
#define DAC2_LSB_REG      (base3+4)    /* D/A 2 Data LSB */
#define DAC2_MSB_REG      (base3+5)    /* D/A 2 Data MSB */
#define DAC3_LSB_REG      (base3+6)    /* D/A 3 Data LSB */
#define DAC3_MSB_REG      (base3+7)    /* D/A 3 Data MSB */
#define DAC4_LSB_REG      (base3+8)    /* D/A 4 Data LSB */
#define DAC4_MSB_REG      (base3+9)    /* D/A 4 Data MSB */
#define DAC5_LSB_REG      (base3+10)   /* D/A 5 Data LSB */
#define DAC5_MSB_REG      (base3+11)   /* D/A 5 Data MSB */
#define DIO_PORTA         (base3+12)   /* Port A 8 bit I/O of 8255 */
#define DIO_PORTB         (base3+13)   /* Port B 8 bit I/O of 8255 */
#define DIO_PORTC         (base3+14)   /* Port C 4+4 bit of 8255 */
#define DIO_CNTRL_REG     (base3+15)   /* Mode and Direction Control Register 8255 */

typedef struct BOARD_REC {
  u32 device;             /* device ID                        */
  u16 base3;              /* Base3 address of DDA06-16 board  */
  u8 dio_reg;             /* Status register Group 0          */
  u8 simultUpdate;        /* Simultanous update               */
} BoardRec;

/* Values kept for each channel. */

typedef struct t_ChanRec_DIO {
  int open;               /* Is channel device open()    */
  u16 value;              /* value of DIO Channel        */
  u16 addr;               /* address of the channel      */
  unsigned short f_flags; /* flags sent by open()        */
} ChanRec_DIO;

typedef struct t_ChanRec_DAC {
  int open;               /* Is channel device open()     */
  u16 value;              /* value of DAC Channel         */
  u16 addr;               /* address of the channel       */
  u8 gain;                /* gain of channel (Set onboard */
} ChanRec_DAC;

#endif
