/***************************************************************************
 Copyright (C) 2011-2015  Warren J. Jasper  <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCIM-DAS16JR/16, is free software; you can redistribute it
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

//#define DEBUG 1
#define ADAPTER_ID "PCIM-DAS16JR.v1.3"

/*****************************
 *     Useful Macros         *
 *****************************/

#define CHAN(x)  (x & 0x1f)             // 5 bits 0-4 for channels
#define BOARD(x) ((x >> 0x5) & 0x7)     // 3 bits 5-7 for board number
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

#ifndef PCI_DEVICE_ID_CBOARDS_PCIM_DAS16JR
#define PCI_DEVICE_ID_CBOARDS_PCIM_DAS16JR 0x7b
#endif

/*  change to get bigger buf size.  Max value is 0x100000  */
#define ADC_BUFF_PHY_SIZE   0x10000       /* Physical size of the ADCs buffer  */


#define AD_CHANNELS          16  // 16 Single Ended / 8 Differential
#define DIO_PORTS             3  // 3 dio ports
#define COUNTERS              3  // 3 counters on the 82C54

#define DEFAULT_FREQ      1000   // Default ADC frequency  1KHz 


/* I/O Register locations */
/* NOTE: BADRn to be found dynamically at boot time. Not hard coded into driver */

/* BADR1: 32-bit Word */
#define INTCSR_ADDR      (base1+0x4c) // PLX PCI 9052 Interrupt Stats and Control Register

/* BADR2: These are 3 16-Bit Word Registers */
#define ADC_DATA_REG     (base2+0x0)  // ADC Data/Contert

/* BADR3: These are 15 8-Bit Byte Registers */
#define MUX_REG          (base3+0x0) // ADC Channel MUX scan limits
#define DIO_REG          (base3+0x1) // Main Connector Digital Input/Output Registers
#define ADC_CHAN_REG     (base3+0x2) // ADC Channel Status and Switch Settings Register
#define ADC_CONV_REG     (base3+0x3) // ADC Conversion Status Register
#define INTCSR_REG       (base3+0x4) // ADC Interrupt Status and Control Register
#define PACER_CLK_REG    (base3+0x5) // A/D Pacer Clock Status and Control Register
#define BURST_CNTL_REG   (base3+0x6) // A/D Burst mode and Converter Control Register
#define GAIN_REG         (base3+0x7) // A/D Programmable Gain Control Register
#define COUNTER_0_DATA   (base3+0x8) // Counter 1 Data - User Counter
#define COUNTER_1_DATA   (base3+0x9) // Counter 2 Data - ADC Pacer Lower Counter
#define COUNTER_2_DATA   (base3+0xa) // Counter 3 Data - ADC Pacer Upper Counter
#define COUNTER_CONTROL  (base3+0xb) // Counter Control Register
#define COUNTER_SELECT   (base3+0xc) // User Counter Clock Select Register
#define RESIDUAL_COUNT_0 (base3+0xd) // Residual Sample Counter Registers
#define RESIDUAL_COUNT_1 (base3+0xe) //

/* General defines */
#define TRUE  1
#define FALSE 0

/* Values kept for each channel. */
typedef struct ADC_CHANNEL_REC {
  int open;           // Is channel device open()
  int nonBlockFlag;   // FALSE = block, TRUE = noblocking
  int mmap_flag;      // Channel has been mmaped for dma
  u32 count;          // Number of samples requested
  u8 lowChan;         // Starting Channel of MUX scan limit
  u8 hiChan;          // Ending Channel of MUX scan limit
  u8 gain;            // gain value.  Polarity (unipolar/bipolar set by jumpers)
  u8 pacerSource;     // 0 = Software Convert
                      // 6 = Pacer Clock  (Internal)
  u8 nSpare1;         // A/D Pacer Clock Status and Control Register
  u8 burstMode;       // 1 = True, 0 = False
} ADC_ChanRec;

typedef struct BOARD_REC {
  struct pci_dev *pdev;      // pointer to pci device struct
  dma_addr_t buf_bus_addr;   // bus address of the DMA buffer for this board
  u32 base0;                 // Base0 address of ADC board
  u32 base1;                 // Base1 address of ADC board
  u32 base2;                 // Base2 address of ADC board
  u32 base3;                 // Base3 address of ADC board
  int irq;                   // IRQ inturrepts
  int busyRead;              // busy = TRUE, free = FALSE
  int WordsToRead;           // Words left to read in the DAQ
  int nChannels;             // Number of Channels (8 or 16).
  ADC_ChanRec *ADC_CurrChan; // Pointer to current channel
  struct file *nonBlockFile; // when we do non-blocking input,
                             // between the first (initiation) read
                             // and final (termination) read, nonblockFile
                             // stores which open file requested theread.
  u32 PacerClock;            // Onboard Pacer Clock either 10 MHz or 1 MHz.
  u32 ADC_freq;              // Complete ADC pacer frequency
  u16 ADC_ctr1;              // ADC Pacer Lower Counter
  u16 ADC_ctr2;              // ADC Pacer Upper Counter
  u32 buf_phy_size;          // size in bytes of the buffer for this board
  u16 *buf_virt_addr;        // virtual kernal (cpu) address of dma buffer
  u16 *ADC_KernBuffPtr;      // pointer into the data buffer array
  u8 nSpare0;                // INTCSR_REG     (base3+0x4)
  u8 nSpare1;                // PACER_CLK_REG  (base3+0x5)
  u8 nSpare2;                // BURST_CNTL_REG (base3+0x6)
  u8 nSpare3;                // GAIN_REG       (base3+0x7)
  u8 nSpare4;                // DIO_REG        (base3+0x1)
  u8 nSpare5;                // ADC_CHAN_REG   (base3+0x2)
} BoardRec;
  
/*************************************************************************
* PLX PCI 9052        base1      Interrupt Status & Control Register     *
**************************************************************************/
#define PLX_INTE   0x1    /* 1=Interrupt Enabled. 0=Interrupt Disabled          */
#define PLX_INTPOL 0x2    /* 0=Active Low. 1=Active High                        */
#define PLX_INT    0x4    /* 0=Interrupt not active, 1=Interupt Active          */
#define PLX_PCIINT 0x40   /* 0=disabled, 1=enabled                              */
#define PLX_INTSEL 0x100  /* 0=level-sensitive, 1=edge sensitive                */
#define PLX_INTCLR 0x400  /* 0=no effect, 1=clear the PCI INT interrupt         */

/***********************************************************************************
* ADC_CHAN__REG      base3 + 0x2    ADC Channel Status and Switc Settings Register *
************************************************************************************/
/* Read Only */
#define MUX_SEL 0x0f   /* MA0 - MA3 indicate MUX channel currently selected       */
#define CLK     0x10   /* 1=Pacer Clock set to 10 MHz, 0=Pacer Clock set to 1 MHz */
#define MUX     0x20   /* 1=ADC Single-Ended, 0=ADC Differential                  */
#define U_B     0x40   /* 1=ADC set to Unipolar, 0=ADC set to Bipolar             */    
#define EOC     0x80   /* 1=ADC busy,  0=ADC free                                 */

/***********************************************************************************
* ADC_CONV__REG      base3 + 0x3    ADC Conversion Status Register                 *
************************************************************************************/
/* Read Only */
#define OVERRUN 0x4   /* 1=FIFO memory has overrun, 0=FIFO memory has not overrun  */
#define FHF     0x8   /* 1=FIFO memory has at least 512 samples,
			 0=FIFO memory has less than 512 sample                    */
#define FNE     0x10  /* 1=FIFO memory contains at least one sample
			 0=FIFO memory contains no samples (empty)                 */
#define EOA    0x20  /* 1=the residual # of samples have been written to the FIFO
			 0=the residual # of samples have not been written to FIFO */
#define EOB     0x40  /* 1 = an ADC Burst has been completed
			 0 = an ADC Burst is in progress or has not started.       */
// #define EOC     0x80  /* 1=ADC busy,  0=ADC free                                   */

/***********************************************************************************
* INTCSR_REG      base3 + 0x4    ADC Interrupt Status and Control Register         *
************************************************************************************/
#define INTSEL0     0x1  /* Used to select the source of the interupt.             */
#define INTSEL1     0x2
#define EOA_INT_SEL 0x4  /* 1=Interupt at end of acquisition, 0=no interrupt       */
#define EOA2        0x8  /* 1=the residual # of samples have been written to the FIFO
			    0=the residual # of samples have not been written to FIFO */
#define OVERRUN2    0x10 /* 1=FIFO memory has overrun, 0=FIFO memory has not overrun  */

#define INT         0x40 /* 1=Interrupt generated,  0=No interrupt generated, clear INT */
#define INTE        0x80 /* 1=Interrupts enabled, 0=Interrutps disabled                 */

/***********************************************************************************
* PACER_CLK_REG      base3 + 0x5    A/D Pacer Clock Status and Control Register    *
************************************************************************************/
#define PS0           0x1  /* Control source of the A/D Pacing.  See Register Map documentation. */
#define PS1           0x2
#define EXT_PACER_POL 0x4  /* 1=external pacer polarity set to negative edge
			      0=external pacer polarity set to positive edge */
#define GATE_EN       0x8  /* 1=gate internal pacer always on, ignore pin 25
			      0=the gate to the internal pacer is controlled by signal on pin 25 */
#define GATE_LATCH    0x10 /* 1=signal on pin 25 will act as edge trigger to internal pacer. It is latched
			      in hardware.  Software must clear latch by writing 0 to GATE_STATUS bit.
			      0 = the signal on pin 25 will act as a level gate to the internal pacer. */
#define GATE_POL      0x20 /* 1=the trigger/gate polarity is set to a negative edge / low level for non
			      burst mode, and a positive edge / high level for burst mode.
			      0=the trigger/gate polarity is set to a positive edge / low level for non
			      burst mode, and a negative edge / high level for burst mode. */
#define GATE_STATUS   0x40 /* (Read) 1=the gate to the internal pacer is on, 0=the gate to the internal pacer is off */
                           /* (Write) 0=clears the hardware latch when GATE_LATCH=1 */

/***********************************************************************************
* BURST_CNTL_REG      base3 + 0x6    A/D Burst mode and Converter Control Register *
************************************************************************************/
#define CONV_EN       0x1  /* 1=conversion are enabled, 0=conversions are disabled */
#define BME           0x2  /* 1=bursting enabled. When burst mode is enabled, the mux
			      channel select bits in MUX_REG are used to specify the
			      channels in the burst.
			      0=bursting is disabled */

/***********************************************************************************
* GAIN_REG            base3 + 0x7    Programmable Gain Control Register            *
************************************************************************************/
#define G0     0x1
#define G1     0x2
#define UNIBP  0x4         // 0 = Unipolar   1 = Bipolar


/***********************************************************************************
* COUNTER_SELECT     base3 + 0xc    User Counter Clock Select Register             *
************************************************************************************/
#define CTR1_CLK_SEL  0x1 /* 1=the onboard 100kHz clock signal is ANDed with the COUNTER 1
			     CLOCK INPUT (pin 21).  A high on pin 21 will allow pulses from
			     the onboard source into the 8254 Counter 1 input.  (This input
			     has a pull-up resistor on it, so no connection is necessary to
			     use the onboard 100 kHz clock.)
			     0=the output to the 82C54 Counter 1 is entirely dependent on pulses at
			     pin 21, COUNTER 1 CLOCK INPUT. */



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

#define PACKETSIZE 512      // half FIFO size
#define FIFO_SIZE  1024


#endif			     
