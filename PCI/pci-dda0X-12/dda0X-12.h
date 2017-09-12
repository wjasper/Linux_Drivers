/***************************************************************************
 Copyright (C) 2007-2015  Warren J. Jasper    <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DDA0X-12, is free software; you can redistribute it
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

#ifndef DDA0X_12_H
#define DDA0X_12_H

/*
 * dda0X-12.h
 */

//#define DEBUG
#define ADAPTER_ID "PCI-DDA0X-12.v0.7"

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

#ifndef PCI_DEVICE_ID_CBOARDS_DDA02_12
#define PCI_DEVICE_ID_CBOARDS_DDA02_12 0x0020
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_DDA04_12
#define PCI_DEVICE_ID_CBOARDS_DDA04_12 0x0021
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_DDA08_12
#define PCI_DEVICE_ID_CBOARDS_DDA08_12 0x0022
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS      1   /* Currently supports 1-4 boards. See Makefile */
#endif

/* Default device major number */
#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0    /* Default Major Device Number */
#endif


#define DIO_PORTS           6    /* 48 bits Digital I/O Ports (6 bytes) */
#define DAC_PORTS           8    /* Maximum number of DAC Channels */

/* I/O Register locations */

/* NOTE: base_reg to be found dynamically at boot time. Not hard coded into driver */


/* BADR2: These are 8 8-Bit Word Registers */
/* Note: The Digitial I/O ports emulates 82C55 Mode 0 operation only. */

#define DIO_PORT0A         (base2)    /* Port 0A 8 bit I/O of 8255 */
#define DIO_PORT0B         (base2+1)  /* Port 0B 8 bit I/O of 8255 */
#define DIO_PORT0C         (base2+2)  /* Port 0C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_0    (base2+3)  /* Mode and Direction Control Register 0*/
#define DIO_PORT1A         (base2+4)  /* Port 1A 8 bit I/O of 8255 */
#define DIO_PORT1B         (base2+5)  /* Port 1B 8 bit I/O of 8255 */
#define DIO_PORT1C         (base2+6)  /* Port 1C 4+4 bit of 8255 */
#define DIO_CNTRL_REG_1    (base2+7)  /* Mode and Direction Control Register 1*/

/* BADR3: 12 16-Bit Word Registers */
#define DAC_CNTRL_REG      (base3)      /* D/A Control Register */
#define DAC_CAL_REG_1      (base3+4)    /* D/A Calibration Register 1 */
#define DAC_CAL_REG_2      (base3+6)    /* D/A Calibration Register 2 */
#define DAC0_REG           (base3+8)    /* D/A 0 Data */
#define DAC1_REG           (base3+10)   /* D/A 1 Data */
#define DAC2_REG           (base3+12)   /* D/A 2 Data */
#define DAC3_REG           (base3+14)   /* D/A 3 Data */
#define DAC4_REG           (base3+16)   /* D/A 4 Data */
#define DAC5_REG           (base3+18)   /* D/A 5 Data */
#define DAC6_REG           (base3+20)   /* D/A 6 Data */
#define DAC7_REG           (base3+22)   /* D/A 7 Data */

/* D/A Control Register */
#define SU                 (0x1)       /* This bit enables simultaneous update for the DAC pair specified by D2 and D1.
					  Setting the simultaneous bit inhibits updating the DAC output until a simultaneous
					  update is initiated.  The DAC's are paired as follows: DACs 0 and 1, DACs 2 and 3,
					  DACs 4 and 5, and DACs 6 and 7. Setting simultaneous update for either DAC in the
					  pair will set it for both.
					  0 = Simultaneous update disabled  1 = Simultaneous update enabled
				       */
#define EN                 (0x2)       /* This bit enables DAC specified by D2, D1, D0
					  0 = DAC disabled   1 = DAC enabled
				       */
#define D0                 (0x4)
#define D1                 (0x8)
#define D2                 (0x10)
#define R0                 (0x40)     /* These bit select gain/range for the DAC specified by D2, D1, and D0 */
#define R1                 (0x80)
#define R2                 (0x100)
#define RANGE              (0x1c0)					  

typedef struct BOARD_REC {
  u32 device;         /* device ID                     */
  u16 base2;          /* Base1 address of DDA0X board  */
  u16 base3;          /* Base2 address of DDA0X board  */
  u8 dio_reg_0;       /* status register Group 0       */
  u8 dio_reg_1;       /* status register Group 1       */
} BoardRec;

/* Values kept for each channel. */

typedef struct t_ChanRec_DIO {
  int  open;              /* Is channel device open()    */
  u16 value;              /* value of DIO Channel        */
  u16 addr;               /* address of the channel      */
  unsigned short f_flags; /* flags sent by open()        */
} ChanRec_DIO;

typedef struct t_ChanRec_DAC {
  int  open;              /* Is channel device open()    */
  u16 value;              /* value of DAC Channel        */
  u16 addr;               /* address of the channel      */
  u16 gain;               /* gain/range value            */
  u16 cntrl_reg;          /* value of control reg        */
} ChanRec_DAC;

#endif
