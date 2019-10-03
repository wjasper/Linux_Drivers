/***************************************************************************
 Copyright (C) 2002-2015  Warren Jasper
 All rights reserved.

 This program, PCI-DAS4020-12, is free software; you can redistribute it
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

#define ADAPTER_ID "PCI-DAS4020-12.v1.26"

// #define DEBUG

/*****************************
 *     Useful Macros         *
 *****************************/

#define BOARD(x) ((x >> 0x4) & 0x7)
#define CHAN(x)  (x & 0xf)
#define ALIGN_ADDRESS(addr, alignment) \
        ((((u32)(addr)) + (((u32)(alignment))-1)) & ~(((u32)(alignment)) - 1))

/* Note:  Edit these values for your particular system.  See README  */
#define ADC_BUFF_PHY_SIZE   0x0200000  // Physical size of the ADCs buffer
#define ADC_BUFF_PAGE_SIZE  0x0010000  /* must be a multiple of PAGE_SIZE and HALF_FIFO_SIZE and 
                                          greater than or equal to PAGE_SIZE.

                                           HALF_FIFO_SIZE = 0x8000     32k
                                                            0x4000     16k
                                                            0x2000      8k 
                                                            0x1000      4k 
                                                            0x800       2k
                                                            0x400       1k
                                                            0x200       512
                                                            0x100       256
                                           PAGE_SIZE =      0x1000      4k   on i386 machines 
				       */
/* Vendor and Device ID */ 
#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS 0x1307 
#endif

#ifndef PCI_DEVICE_ID_CBOARDS_DAS4020_12
#define PCI_DEVICE_ID_CBOARDS_DAS4020_12 0x0052 
#endif

/* Default device major number */
#ifndef DEFAULT_MAJOR_DEV
#define DEFAULT_MAJOR_DEV 	0
#endif

#ifndef MAX_BOARDS
#define MAX_BOARDS    1   /* Suppport 1 board for now */
#endif

#define DIO_PORTS    3	  /* DIO ports 			    */ 	
#define AD_CHANNELS  4
#define DA_CHANNELS  2

/*
 * --------------------------------------------------------------------
 * 			mc4020 kernel structures
 * --------------------------------------------------------------------
 */

#define	BASE_CLK_FREQ	 40000000	// internal clock ref (40 MHz)

#define	MIN_SCAN_RATE	     1000	//  1 KSPS
#define	MAX_SCAN_RATE_1	 20000000	// 20 MSPS
#define	MAX_SCAN_RATE_2	 20000000	// 20 MSPS
#define	MAX_SCAN_RATE_4	 10000000	// 10 MSPS

/* General Definitions */
#define TRUE 		1
#define FALSE		0

/* Structure for the Board */
typedef struct BOARD_REC {
  struct pci_dev *pdev;        // pointer to pci device struct
  u32 phys_addr0;              // physical address of PLX regs
  u32 phys_addr2;              // physical address of PCI-DAS4020/12 reg
  u32 phys_addr3;              // physical address of FIFO
  u32 half_FIFO_size;          // size of 1/2 FIFO in bytes
  u8 *base0;                   // virtural address of PLX regs
  u8 *base2;                   // virtural address of PCI-DAS4020/12 reg
  u8 *base3;                   // virtural address of FIFO
  u8 dio_reg;                  // DIO control register
  int  irq;                    // IRQ inturrepts 
  int  busyRead;               // busy = TRUE, free = FALSE
  int  busyWrite;              // busy = TRUE, free = FALSE
  int  nchannels;              // Number of active channels 1, 2 or 4
  int  WordsToRead;            // Words left to read in the DAQ
  struct file *nonBlockFile;   //  when we do non-blocking input,
                               //  between the first (initiation) read
                               // and final (termination) read, nonblockFile
                               // stores which open file requested theread. 
  struct dma_desc *chain_desc; // dma chaining descriptor
  void **dma_virt_tbl;         // virtual kernal (cpu) address of dma buffer table
  dma_addr_t *dma_bus_addr_tbl; // table of bus addresses of the DMA buffer for this board
  u32 dma_tbl_size;            // number of valid buffer table entries
  u32 dma_tbl_page_size;       // size of each buffer table entry
  u32 dma_phy_size;            // overall size in bytes of DMA buffer for this board
  dma_addr_t chain_desc_bus_addr;   // bus address of the dma chain descriptors (for scatter gather dma)
  u32 chain_desc_size;         // size in bytes of the chain descriptors (for scatter gather dma)
  u32 *ADC_KernBuffPtr;        // pointer to kernbuf (one for each board)
   
  /* Shadow Registers which are write only  */
  u16 InterruptEnableReg;
  u16 HardwareConfigReg;
  u16 MemorySizeReg;
  u16 AtrigLowReg;
  u16 AtrigHighReg;
  u16 DAQ_ControlReg0;
  u16 DAQ_ControlReg1;
  u16 SampleIntervalLowReg;
  u16 SampleIntervalHighReg;
  u16 DAQSoftConversionReg;
  u16 SampleScanHighReg;
  u16 SampleScanLowReg;
  
  u16 cal_data[16];           // 8 pairs of gain/offset for +/- 5 and +/- 1 V
  u16 cal_coef[8];            // Current set of calibration coef (up to 4 channels)
  u16 i2c_reg;
  u16 DAC_ControlReg;         // DAC Control Register
  u32 bitmask;		
  u32 scan_rate;
  u32 throughput;
  u32 DMAComplete;
} BoardRec;

// bitmask values for bitmask
#define	CH0_EN		 0x00000001	// enable channel 0 for input
#define	CH1_EN		 0x00000002	// enable channel 1 for input
#define	CH2_EN		 0x00000004	// enable channel 2 for input
#define	CH3_EN		 0x00000008	// enable channel 3 for input
#define	CH0_5V		 0x00000010	// ch0 range: +/- 5V
#define	CH0_1V		 0x00000000	// ch0 range: +/- 1V
#define	CH1_5V		 0x00000020	// ch1 range: +/- 5V
#define	CH1_1V		 0x00000000	// ch1 range: +/- 1V
#define	CH2_5V		 0x00000040	// ch2 range: +/- 5V
#define	CH2_1V		 0x00000000	// ch2 range: +/- 1V
#define	CH3_5V		 0x00000080	// ch3 range: +/- 5V
#define	CH3_1V		 0x00000000	// ch3 range: +/- 1V

// select input source
#define	ASRC_MASK	 0x00000700
#define	ASRC_BNC	 0x00000000	// BNC connector (normal)
#define	ASRC_CAL_AGND	 0x00000100	// AGND
#define	ASRC_CAL_0_625	 0x00000200	// 0.625 V ref
#define	ASRC_CAL_4_375	 0x00000300	// 4.375 V ref
#define	ASRC_CAL_HDR	 0x00000400	// calibration header (P2)

// Trig/Ext Clk BNC threshold select
#define	EXT_BNC_THRESH_ZERO	 0x00000800
#define	EXT_BNC_THRESH_2_5V	 0x00000000

//          data acquistion base clock source
#define	CLK_MASK	     0x00003000

#define ALL_EN   (CH0_EN | CH1_EN | CH2_EN | CH3_EN)
#define	ALL_1V   (CH0_1V | CH1_1V | CH2_1V | CH3_1V)
#define	ALL_5V	 (CH0_5V | CH1_5V | CH2_5V | CH3_5V)
#define	ALL_CHAN (0xf0)


/* Structures for each channel */

typedef struct ADC_CHANNEL_REC {
  int open;             // Is channel device open()
  int mmap_flag;        // Channel has been mmaped for dma
  int nonBlockFlag;     // FALSE = block, TRUE = noblocking
  u32 count;            // Number of samples requested
  u32 frequency;        // Sampling Frequency
  u32 bitmask;		// 
  u16 pretrigCount;     // Number of samples before trigger
  u16 analog_trig_low;  // DAQ Analog Trigger LOW register
  u16 analog_trig_high; // DAQ Analog Trigger HIGH register
  u16 controlReg0;      // shadow of control register 0
  u16 controlReg1;      // shadow of control register 1
  u8 lowChan;           // Low Channel in two channel mode
  u8 hiChan;            // High Channel of two channel mode
  u8 adcMode;           // 0 = Software Convert
                        // 2 = DMA transfer
  u8 atrigMode;         // Analog Trigger Modes
  u8 trigger;           // Type of trigger
  u8 gate;              // Type of gating
} ADC_ChanRec;

typedef struct DAC_CHANNEL_REC {
  int open;            // Is channel device open()
  u16 value;           // value of D/A Channel
  u16 gain;            // Voltage range for output gain
} DAC_ChanRec;

typedef struct DIO_CHANNEL_REC {
  int open;		// Is channel device open()
  int mode;		// mode for 8255
  u8 *addr;		// address of the channel
  u8 value;		// Value of DIO Channel
  u32 f_flags;	        // flags sent by open()
} DIO_ChanRec;

/* I/O Register Locations */

/* BADR0 */
#define DAQ_CALIB_COEFF		0x4E	     /* Starting address of DAQ calib coeffs */	
/* All PLX Register Locations are defined in plx9080.h */

/* BADR2 */
/*Configuration Group*/
#define INTRPT_EN_REG	      (base2+0x00)
#define HW_CONFIG_REG	      (base2+0x02)
#define MEM_SIZE_REG	      (base2+0x04)

/*ADC Regster Group*/
#define ATRIG_LOW_REG	      (base2+0x0c) // DAQ ATRIG LOW  Register
#define ATRIG_HIGH_REG	      (base2+0x0e) // DAQ ATRIG HIGH Register
#define CTRL_REG_0	      (base2+0x10) // DAQ Control Register 0
#define CTRL_REG_1	      (base2+0x12) // DAQ Control Register 1
#define SAMPLE_INT_LOW	      (base2+0x16) // Sample Interval Register (Lower)
#define SAMPLE_INT_HIGH	      (base2+0x18) // Sample Interval Register (Upper)
#define SAMPLE_SCAN_REG_LOW   (base2+0x1e) // Sample/Scan Count Register (Lower)
#define SAMPLE_SCAN_REG_HIGH  (base2+0x20) // Sample/Scan Count Register (Upper)
#define DAQ_START_REG	      (base2+0x22) // DAQ Soft Start Register
#define DAQ_SINGLE_CONV	      (base2+0x24) // DAQ Single Conversion Register
#define ADC_FIFO_CLR          (base2+0x2A) // ADC Buffer Pointer Clear Register

/* DAC Register Group */
#define DAC_CTRL_REG          (base2+0x52) // DAC Control Register
#define DAC0_CONV_LSB         (base2+0x70) // DAC0 Single Conversion Register(LOWER)
#define DAC0_CONV_MSB         (base2+0x72) // DAC0 Single Conversion Register(UPPER)
#define DAC1_CONV_LSB         (base2+0x74) // DAC1 Single Conversion Register(LOWER)
#define DAC1_CONV_MSB         (base2+0x76) // DAC1 Single Conversion Register(LOWER)

/* 16-bit Read Only */
#define HW_STAT_REG  	      (base2+0x00)
#define ADC_READ_REG 	      (base2+0x08)
#define ADC_WRITE_REG 	      (base2+0x0c)
#define USER_XFER_REG	      (base2+0x10)
#define PRE_POST_REG	      (base2+0x14)

/* FIX ME..I am not sure what this is, Page 13 in pci-das4020/12 manual */
#define USER_CHIP_SEL0	      (base2+0x48)
#define USER_CHIP_SEL1	      (base2+0x49)
#define USER_CHIP_SEL2	      (base2+0x4a)
#define USER_CHIP_SEL3	      (base2+0x4b)
#define USER_CHIP_SEL4	      (base2+0x4c)
#define USER_CHIP_SEL5	      (base2+0x4d)
#define USER_CHIP_SEL6	      (base2+0x4e)

/* 16-bit Read/Write */
#define DIO_PORTA	      (base2+0x48)
#define DIO_PORTB	      (base2+0x4A)
#define DIO_PORTC	      (base2+0x4C)
#define DIO_CNTRL_REG	      (base2+0x4E)


/* I2C Registers */
/* Analog Front end cicuitry is controlled by I2C-Interface 
 * The addresses below are the I2C addresses of the three I2C devices 
 * on the board
 * */

#define I2C_REG                   0x40
#define I2C_CAL_DAC0              0x18
#define I2C_CAL_DAC1              0x1a

/*
 * ======================================================================
 *			  I2C & eeprom access
 *
 * The I2C bus is a two wire bus consisting of clock and data.  The data
 * is generally bidirectional, but in this implementation is output only.
 * The clock line is called SCL and the data line is SDA.
 *
 * SDA and SCLK are controlled with the PLX 9080 CNTRL register.
 * SCL is the USEROUT pin, SDA is the EEPROM write bit (shared with the
 * configuration EEPROM).
 *
 * On this card, the I2C bus consists of 3 devices, an 8 bit register 
 * (haven't figured out the part number) and two Analog Devices quad 10
 * bit dacs (AD5315).
 *
 * The factory calibration data is stored in the unused top of the
 * PCI configuration EEPROM starting at offset 0x30.  The EEPROM is
 * compatible with the Fairchild FM93CS56 part and uses the
 * "Microwire" 4-wire bus.  This bus is also controlled via the PLX 9080
 * CNTRL register.
 * ======================================================================
 */

/* bit defs in the CNTRL register */

#define	I2CBUS_SCL		PLX_USEROUT
#define	I2CBUS_SDA		PLX_EEWD

/* I2C devices */

#define	I2C_REGISTER_ADDR			0x20

#define	  IREG_THRESH_ZERO	(1 << 7)	// trig/ext clk BNC threshold
#define	  IREG_THRESH_TTL	(0 << 7)
						// selects analog source
#define	  IREG_ASRC_CAL_HDR	(0x0 << 4)	// Calibration header (P2)
#define	  IREG_ASRC_BNC	    	(0x4 << 4)	// BNC connector
#define	  IREG_ASRC_CAL_4_375   (0x5 << 4)	// 4.375 V ref
#define	  IREG_ASRC_CAL_0_625	(0x6 << 4)	// 0.625 V ref
#define   IREG_ASRC_CAL_AGND	(0x7 << 4)	// 0.000 V ref

#define	  IREG_ATTEN_CH3	(1 <<  3)	// attenuate input by 5:1
#define	  IREG_ATTEN_CH2	(1 <<  2)
#define	  IREG_ATTEN_CH1	(1 <<  1)
#define	  IREG_ATTEN_CH0	(1 <<  0)

#define	I2C_CAL_DAC0_ADDR			0x18
#define	I2C_CAL_DAC1_ADDR			0x1a

/*
 * --------------------------------------------------------------------
 * BAR3 ( Local Space 1) registers.
 *
 * The FIFO is mapped into this area...
 * --------------------------------------------------------------------
 */

#define FIFO                  (base3+0x200)           // 32-bit register (2 samples wide)
        
/* Interrupt Enable Register BADR1/LocalSpace0 */           
/* Write Only*/

#define DAQ_ISRC0     0x1     /* ISRC1 ISRC0  (Selects additional DAQ interrupt source)*/
#define DAQ_ISRC1     0x2     /*  0    0     => DAQ FIFO 1/2 Full.
                                  0    1     => DAQ Single Conversion. 
                                  1    0     => Paced Conversion.
                               */
#define DAQ_IENB      0x4     // Set => DAQ_ISRC conditions generates an interrupt.
#define DAQDONE       0x8     // Set => An interrupt is generated when DAQ sequence completes.
#define XINT          0x100   // Set => The external Interrupt signal can generate an interrupt.
#define ADC_ACTIVE    0x200   // Set => An interrupt is generated when a DAQ sequence is active.
#define DAQ_STOP      0x400   // Set => An interrupt is generated when the stop trigger(TRIG2) is detected.
#define OVERRUN       0x8000  // When set it enables detection of DAQ overrun condition.

/* Hardware Configuration Register */
#define WCLK_SRC0     0x1         /* Selects the data acquisition boards base clock source  */
#define WCLK_SRC1     0x2         /* WCLK_SRC1  WCLK_SRC0
                                                   0        0       Inactive
                                                   0        1       On Card 40Mhz oscillator
                                                   1        0       Trig/Ext Clk BNC input
                                                   1        1       A/D Start Trigger In
						                                                */
#define WCLK_MASK                   0x3
#define	WCLK_INACTIVE		    0x0
#define	WCLK_INTERNAL_40MHZ	    0x1
#define	WCLK_EXT_CLK_BNC	    0x2
#define	WCLK_AD_START_TRIG_IN	    0x3


#define XINT_POL      0x100       /* External Interrupt polarity selector
					       0 => rising edge, 1=> falling edge                       */


/*Memory Size register */       
#define ASEG0         0x1         // Selects ADC buffer segment size (FIFO size).
				  //	       Two samples stored per FIFO location
#define ASEG1         0x2         // ASEG(6:0)    FIFO-size
#define ASEG2         0x4         //    0x00          32K
#define ASEG3         0x8         //    0x40          16K
#define ASEG4         0x10        //    0x60           8K
#define ASEG5         0x20        //    0x70           4K
#define ASEG6         0x40        //    0x78           2K
                                  //    0x7C           1K
                                  //    0x7E          512
                                  //    0x7F          256
                                            
/* DAQ ATRIG LOW Register */  
#define THRESH_L0     0x1    // THRESH_L(11:0) load the low threshold value for analog trigerring
#define THRESH_L1     0x2         
#define THRESH_L2     0x4         
#define THRESH_L3     0x8         
#define THRESH_L4     0x10        
#define THRESH_L5     0x20         
#define THRESH_L6     0x40         
#define THRESH_L7     0x80         
#define THRESH_L8     0x100         
#define THRESH_L9     0x200         
#define THRESH_L10    0x400         
#define THRESH_L11    0x800         
#define XTRIG1_SRC    0X2000    // External TRIG1 Source Select 0 => Start Trig pin 1 => Trig/Ext Clk BNC source.
#define XTRIG2_SRC    0X4000    // External TRIG2 Source Select 0 => Stop  Trig pin 1 => Trig/Ext Clk BNC source.
#define XAGATE_SRC    0X8000    // External AGATE Source Select 0 =>A/D Pacer Gate Pin 1=> Trig/Ext Clk BNC source.

/* DAQ ATRIG HIGH Register */
#define THRESH_H0     0x1       // THRESH_H(11:0) To load the high threshold value for analog trigerring.
#define THRESH_H1     0x2 
#define THRESH_H2     0x4 
#define THRESH_H3     0x8 
#define THRESH_H4     0x10 
#define THRESH_H5     0x20 
#define THRESH_H6     0x40 
#define THRESH_H7     0x80 
#define THRESH_H8     0x100 
#define THRESH_H9     0x200 
#define THRESH_H10    0x400 
#define THRESH_H11    0x800 

/* DAQ Control Register 0 */
#define AGATE_SRC0   0x1    // AGATE_SRC(0:1). Select AGATE source
#define AGATE_SRC1   0X2         
#define AGATE_LVL    0X4    // AGATE Level select    0=> edge sensitive gate; 1=> level sensitive gate
#define AGATE_POL    0x8    // AGATE polarity select 0=> active high gate, 1=> active low gate
#define TRIG1_SRC0   0x10   // TRIG1_SRC1: TRIG1_SRC0  (Select TRIG1 Source)
#define TRIG1_SRC1   0x20   //      0       0           Disabled
                            //      0       1           Software trigger trigger 
                            //      1       0           external TRIG1 source
                            //      1       1           Analog Trigger Source as start trigger
#define TRIG1_POL    0x40   // Polarity Select: 0=> Rising edge 1=> Falling edge
#define TRIG2_SRC    0x80   // Source select: 0=> External A/D Stop Trig pin; 1 => Analog Trigger
#define TRIG2_POL    0x100  // Polarity Select: 0=> rising edge 1=> falling edge
#define TRIG2_ENB    0x200  // Enables Pre-trigger mode
#define SAMPCNTENB   0x1000 // Set => DAQ Sample counter is enabled
#define DMA_DSBL     0x4000 // DMA disable
#define DAQ_ENB      0x8000 // Master Enable for Data Acquisition operations.

/* DAQ Control Register 1 */
#define	  CR1_PSC_ENB			(1 << 15)	// divide by 400 prescaler enable
#define	  CR1_reserved_14		(1 << 14)

#define	  CR1_CHANMODE_MASK		(0x3 << 12)
#define	  CR1_CHANMODE_1		(0x0 << 12)	// 20MHz max sample rate on 1 channel
#define	  CR1_CHANMODE_2		(0x1 << 12)	// 20MHz max sample rate on 2 channels
#define	  CR1_CHANMODE_4		(0x2 << 12)	// 10MHz max sample rate on 4 channels
#define	  CR1_CHANMODE_4_BURST		(0x3 << 12)	// 20MHz max sample rate; 32K samples / channel max
#define	  CR1_UCHAN_MASK		(0x3 << 10)	// upper channel select in 2 channel mode
#define	  CR1_UCHAN(x)			(((x) & 0x3) << 10)
#define	  CR1_UCHAN_0			(0x0 << 10)	// channel 0	
#define	  CR1_UCHAN_1			(0x1 << 10)	// channel 1
#define	  CR1_UCHAN_2			(0x2 << 10)	// channel 2
#define	  CR1_UCHAN_3			(0x3 << 10)	// channel 3

#define	  CR1_LCHAN_MASK		(0x3 <<  8)	// lower channel select in 1 and 2 channel mode
#define	  CR1_LCHAN(x)			(((x) & 0x3) << 8)
#define	  CR1_LCHAN_0			(0x0 <<  8)	// channel 0	
#define	  CR1_LCHAN_1			(0x1 <<  8)	// channel 1
#define	  CR1_LCHAN_2			(0x2 <<  8)	// channel 2
#define	  CR1_LCHAN_3			(0x3 <<  8)	// channel 3
#define	  CR1_reserved_7		(1 <<  7)
#define	  CR1_SFT_GATE			(1 <<  6)	// set enables ADC conversions, see CR0_AGATE_SRC_SOFTWARE

#define	  CR1_ATRIG_SRC_MASK		(0x3 << 4)	// analog trigger source channel...
#define	  CR1_ATRIG_SRC_0		(0x0 << 4)	// channel 0
#define	  CR1_ATRIG_SRC_1		(0x1 << 4)
#define	  CR1_ATRIG_SRC_2		(0x2 << 4)
#define	  CR1_ATRIG_SRC_3		(0x3 << 4)

#define	  CR1_ATRIG_MD_MASK		(0x7 << 1)	// Analog trigger/gate modes
#define	  CR1_ATRIG_MD_INACTIVE		(0x0 << 1)
#define	  CR1_ATRIG_MD_POS_HYST		(0x2 << 1)     
#define	  CR1_ATRIG_MD_NEG_HYST		(0x3 << 1)     
#define	  CR1_ATRIG_MD_NEG_SLOPE	(0x4 << 1)
#define	  CR1_ATRIG_MD_POS_SLOPE	(0x5 << 1)
#define	  CR1_ATRIG_MD_WINDOW		(0x6 << 1)

#define	  CR1_reserved_0		(1 <<  0)


/* DAQ Soft Start Register */
/* Accesing the DAQ Soft Start Register initiates a multiple A/D conversion */
/* BITs 0 - 15 ALL ARE DONT CARES */ 

/*DAQ Single Conversion Register */         /*Access to this reg. starts a single conversion*/
#define CHANSEL0  0x100    // CHANSEL1 CHANSEL0 : Select channel to be acquired in single conversion
#define CHANSEL1  0x200    //   0      0       => Channel 0
                           //   0      1       => Channel 1
                           //   1      0       => Channel 2
                           //   1      1       => Channel 3

/* ADC Buffer Pointer Clear Register */
/* Accessing the ADC buffer pointer clear register restes the pointer to home state */
/* BITs 0 - 15 ALL ARE DONT CARES  Values kept for each channel.                    */
       
/* DAC Control Register1 */                  
#define DAC0R0   0x1      // Configure range of DAC0 0 = bipolar +/-10V;1= +/-5V
#define DAC1R0   0x4      // Configure range of DAC1 0 = bipolar +/-10V;1=+/-5V
#define DAC_OE   0x80     // DAC output enable. When this bit is set DAC outputs are enabled.

/* Hardware Status Register */
#define REV_MASK     (0x3 << 13)
#define NEXT_CHAIN   (0x1 << 12)
#define PIPEFULL1    (0x1 << 11)
#define PIPEFULL0    (0x1 << 10)
#define PIPEFULL     (PIPEFULL1 | PIPEFULL0)
#define TRIG2_FLG    (0x1 <<  9)
#define XINT_FLG     (0x1 <<  8)
#define DAQ_DONE     (0x1 <<  7)
#define ASRC_FLG     (0x1 <<  5)
#define DAQ_ACTIVE   (0x1 <<  3)
#define DAQ_OVERRUN  (0x1 <<  1)

#endif
