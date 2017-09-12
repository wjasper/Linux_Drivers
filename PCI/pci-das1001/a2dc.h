/***************************************************************************
 Copyright (C) 2003-2015  Warren J. Jasper    <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DAS1001, is free software; you can redistribute it
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

// #define DEBUG
#define ADAPTER_ID "PCI-DAS1001.v1.10"

/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x5) & 0x7)
#define CHAN(x)  (x & 0x1f)
#define ALIGN_ADDRESS(addr, alignment) \
        ((((u32)(addr)) + (((u32)(alignment))-1)) & ~(((u32)(alignment)) - 1))


/* Default device major number */
#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV  0   /* Default Major Device Number  */
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS           1   /* Number of Boards */
#endif

#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_DAS1001
#define PCI_DEVICE_ID_CBOARDS_DAS1001 0x1A
#endif

/*  change to get bigger buf size.  Max value is 0x100000  */
#define ADC_BUFF_PHY_SIZE   0x10000       /* Physical size of the ADCs buffer  */

#define AD_CHANNELS          16  /* 16 Single Ended              */
#define DA_CHANNELS           2  /* 2 Digital to Analog Channels */
#define DIO_PORTS             3  /* dio ports                    */

#define DEFAULT_FREQ      1000   /* Default frequency  1KHz      */

/* I/O Register locations */
/* NOTE: BADRn to be found dynamically at boot time. Not hard coded into driver */

/* BADR0 */
#define INTCSR_ADDR      (base0+0x38) 
#define BMCSR_ADDR       (base0+0x3c)
#define NVRAM_DATA_REG   (base0+0x3e) /* NVRAM Address/Data Register        */
#define NVRAM_CTRL_REG   (base0+0x3f) /* NVRAM Access Control Register      */


/* BADR1: These are 5 16-Bit Word Registers */
#define IRQ_REG          (base1)     /* Interrupt/ADC FIFO Register          */
#define MUX_REG          (base1+0x2) /* ADC Channel MUX and Control Register */
#define TRIG_REG         (base1+0x4) /* Trigger Control/Status Register      */
#define CALIBRATE_REG    (base1+0x6) /* Calibration Register                 */
#define DAC_REG          (base1+0x8) /* DAC Control/Status Register          */

/* BADR2: These are 2 16-Bit Word Registers */
#define ADC_DATA_REG     (base2)     /* ADC Data Register                    */
#define ADC_FIFO_CLR     (base2+0x2) /* ADC FIFO Clear Register              */

/* BADR3: These are 12 8-Bit Byte Registers */
#define COUNTERA_0_DATA  (base3)     /* ADC Residual Sample Counter         */
#define COUNTERA_1_DATA  (base3+0x1) /* ADC Pacer Divider Lower             */
#define COUNTERA_2_DATA  (base3+0x2) /* ADC Pacer Divider Upper             */
#define COUNTERA_CONTROL (base3+0x3) /* ADC 8254-A Control Register         */
#define DIO_PORTA        (base3+0x4) /* Port A 8 bit I/O of 8255            */
#define DIO_PORTB        (base3+0x5) /* Port B 8 bit I/O of 8255            */
#define DIO_PORTC        (base3+0x6) /* Port C 4+4 bit of 8255              */
#define DIO_CNTRL_REG    (base3+0x7) /* Mode and Direction Control Register */
#define COUNTERB_0_DATA  (base3+0x8) /* ADC Pre-Trigger Index Counter       */
#define COUNTERB_1_DATA  (base3+0x9) /* DAC Pacer Divder Lower              */
#define COUNTERB_2_DATA  (base3+0xa) /* DAC Pacer Divider Upper             */
#define COUNTERB_CONTROL (base3+0xb) /* DAC 8254-B Control Register         */

/* BADR4: These are 2 16-Bit Word Registers */
#define DAC0_DATA_REG    (base4)     /* DAC0 Data Register                  */
#define DAC1_DATA_REG    (base4+0x2) /* DAC0 Data Register                  */

/* Errors */
#define BAD_CHANNEL        1
#define BAD_COUNT          2
#define BAD_GAIN           3
#define NOT USED           4
#define BAD_SPEED          5
#define ADC_BUSY           6
#define INT_ALREADY_ACTIVE 7

/* General defines */

#define TRUE  1
#define FALSE 0


/* Values kept for each channel. */

typedef struct ADC_CHANNEL_REC {
  int open;           // Is channel device open()
  int nonBlockFlag;   // FALSE = block, TRUE = noblocking
  int mmap_flag;      // Channel has been mmaped for dma
  u32 count;          // Number of samples requested
  u16 pretrigCount;   // Number of samples before trigger
  u16 gain;           // Voltage range for input gain
  u16 nSpare2;        // BADR1+4  ADC trigger
  u8 lowChan;         // Starting Channel of MUX scan limit
  u8 hiChan;          // Ending Channel of MUX scan limit
  u8 pacerSource;     // 0 = Software Convert
                      // 6 = Pacer Clock  (Internal)
} ADC_ChanRec;

typedef struct DAC_CHANNEL_REC {
  int open;              /* Is channel device open()         */
  u16 value;             /* value of D/A Channel             */
  u16 gain;              /* Voltage range for output gain    */
} DAC_ChanRec;

typedef struct DIO_CHANNEL_REC {
  int  open;             /* Is channel device open()    */
  int mode;              /* mode for 8255               */
  u16 value;             /* value of DIO Channel        */
  u16 addr;              /* address of the channel      */
  u16 f_flags;           /* flags sent by open()        */
} DIO_ChanRec;


/* Data that is board realted */
typedef struct BOARD_REC {
  struct pci_dev *pdev;      // pointer to pci device struct
  u32 base0;                 // Base0 address of ADC board
  u32 base1;                 // Base1 address of ADC board
  u32 base2;                 // Base2 address of ADC board
  u32 base3;                 // Base3 address of ADC board
  u32 base4;                 // Base4 address of ADC board
  u16 nSpare0;               // BADR1+0  IRQ/ADC Fifo
  u16 nSpare1;               // BADR1+2  ADC Mux/Control
  u16 nSpare2;               // BADR1+4  ADC trigger
  u16 nSpare3;               // BADR1+8  DAC control
  int  irq;                  // IRQ inturrepts
  int  busyRead;             // busy = TRUE, free = FALSE
  int  WordsToRead;          // Words left to read in the DAQ
  struct file *nonBlockFile; //  when we do non-blocking input,
                             //  between the first (initiation) read
                             // and final (termination) read, nonblockFile
                             // stores which open file requested theread. 
  u8 dio_reg;                // DIO control register
  u32 ADC_freq;              // Complete ADC pacer frequency
  u16 ADC_ctr1;              // MSB of estimated ADC "freq"
  u16 ADC_ctr2;              // LSB of estimated ADC "freq"
  dma_addr_t buf_bus_addr;   // bus address of the DMA buffer for this board
  u16 *buf_virt_addr;        // virtual kernal (cpu) address of dma buffer
  u16 *ADC_KernBuffPtr;      // pointer into the data buffer array
  u32 buf_phy_size;          // size in bytes of the buffer for this board
  int PreTrigMode;           // True = 1, False = 0
  int PreTrigIndexCounter; 
  int PreTrigBufSize;        // number of bytes in circular buffer
  int PreTrigBufCnt;         // offset into circular buffer
  ADC_ChanRec *ADC_CurrChan; // Pointer to current channel
} BoardRec;

/*************************************************************************
* AMC9533 PCI Controller base 0                                          *
**************************************************************************/
#define INTCSR_DWORD    0x00FF1F00   /* dword written to INTCSR_ADDR */
#define BMCSR_DWORD     0x08000000   /* dword written to BMCSR_ADDR  */


/*************************************************************************
* IRQ_REG             base1      Interrupt / ADC FIFO Register           *
**************************************************************************/
/* Write */
#define INT0   0x1    /* General Interrupt Source selection bits.           */
#define INT1   0x2
#define INTE   0x4    /* 1=Interrupt Enabled. 0=Interrupt Disabled          */
#define EOAIE  0x10   /* 1=Enable EOA interrupt. 0=Disable EOA interrupt    */
#define EOACL  0x40   /* 1=Clear EOA interrupt. 0=No effect.                */
#define INTCL  0x80   /* 1=Clear INT[1:0] interrupt. 0=No effect            */
#define ADFLCL 0x2000 /* 1=Clear ADC FIFO Full latch. 0=No effect.          */
/* Note: It is not necessary to reset any write-clear bits after they are set */

/* Read */
#define EOAI   0x40   /* 1=Indicates an EOA interrupt has been latched.     */
#define INT    0x80   /* 1=General Interrupt has been latched.              */
#define EOBI   0x200  /* 1=Indicates an EOB interrupt has been latched.     */
#define ADHFI  0x400  /* 1=Indicates an ADC Half-Full interrupt latched.    */
#define ADNEI  0x800  /* 1=ADC FIFO Not-Empty interrupt occurred.           */
#define ADNE   0x1000 /* 1=ADC FIFO not empty. 0=ADC FIFO empty.            */
#define LADFUL 0x2000 /* 1=ADC FIFO has exceeded full state. Data lost.     */

/*************************************************************************
* MUX_REG      base1 + 0x2       ADC Channel MUX and Control Register    *
**************************************************************************/
/* Write */
#define GS0    0x100  /* ADC Gain ranges                                  */
#define GS1    0x200  /* ADC Gain ranges                                  */
#define SEDIFF 0x400  /* 1=Single Ended (16 Chan) 0=Differential (8 Chan) */
#define UNIBIP 0x800  /* 1=ADC Unipolar  0=ADC Bipolar                    */
#define ADPS0  0x1000 /* Select the ADC Pacer Source                      */
#define ADPS1  0x2000 

/* Read */
#define EOC    0x4000 /* 1=ADC Done  0=ADC Busy                           */

/*************************************************************************
* TRIG_REG     base1 + 0x4       Trigger Control / Status Register      *
**************************************************************************/
/* Write */
#define TS0    0x1    /* ADC Trigger Source bits                          */
#define TS1    0x2    /* ADC Trigger Source bits                          */
#define TGEN   0x10   /* 1=Selected Trigger Source Enabled.               */
#define BURSTE 0x20   /* 1=Burst Mode Enabled 0=Burst Mode Disabled       */
#define PRTRG  0x40   /* 1=Enable Pre-Trigger Mode 0=Disable Pre-Trigger  */
#define XTRCL  0x80   /* 1=Clear XTRIG status                             */
#define ARM    0x800  /* ARM the Residual Counter                         */
#define FFM0   0x1000 /* 1=Start Residual Counter Now, 0=ARM on ADHF      */
#define C0SRC  0x2000 /* 1=Internal 10 MHz oscillator 0=Extern clock src  */

/* Read  */
#define XTRIG   0x80   /* 1=Extern Trigger set. 0=No trigger received     */
#define INDX_GT 0x1000 /* 1=PreTrig index counter complete.               */

/*************************************************************************
* CALIBRATE_REG  base1+0x6   Calibration Register                        *
**************************************************************************/
/* Write */
#define SEL8800 0x100
#define SEL7376 0x200
#define CSR0    0x800
#define CSR1    0x1000
#define CSR2    0x2000
#define CALEN   0x4000
#define SDI     0x8000

/*************************************************************************
* DAC_REG       base1 + 0x8   DAC Control / Status Register              *
**************************************************************************/
/* Write */
#define DACEN   0x2   /* 1=DAC0/1 enabled  0=DAC0/1 disabled              */
#define MODE    0x80  /* 1=Both DAC0 & DAC1 updated when written to DAC0  */
#define DAC0R0  0x100 /* Bits select the independent DAC gains/ranges    */
#define DAC0R1  0x200 
#define DAC1R0  0x400
#define DAC1R1  0x800

/* Read  */
#define LDAEM   0x1   /* 1=DAC FIFO was emptied  0=Status good           */

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

#define PACKETSIZE  256
#define FIFO_SIZE   512


/***************************************************************
 PCI-DAS1000 non-volatile RAM organization
 This table stores the address offsets for the various 
 calibration coeffecients
***************************************************************/
#define ADC_BIP10V_COEFF      0x0bc
#define ADC_BIP1V_COEFF       0x0bf
#define ADC_BIP0PT1V_COEFF    0x0c2
#define ADC_BIP0PT01V_COEFF   0x0c5
#define ADC_UNI10V_COEFF      0x0c8
#define ADC_UNI1V_COEFF       0x0cb
#define ADC_UNI0PT1V_COEFF    0x0ce
#define ADC_UNI0PT01V_COEFF   0x0d1
#define DAC_BIP5V_COEFF       0x0dc
#define DAC_BIP10V_COEFF      0x0e0
#define DAC_UNI5V_COEFF       0x0e4
#define DAC_UNI10V_COEFF      0x0e8



/***************************************************************
PCI-DAS1000 DAC8800 channels
***************************************************************/

#define DAC0_FINE_GAIN        0
#define DAC0_COARSE_GAIN      1
#define DAC0_OFFSET           2
#define DAC1_OFFSET           3
#define DAC1_FINE_GAIN        4
#define DAC1_COARSE_GAIN      5
#define ADC_COARSE_OFFSET     6
#define ADC_FINE_OFFSET       7

#endif
