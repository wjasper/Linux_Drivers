/***************************************************************************
 Copyright (C) 1997 - 2003  Warren J. Jasper
 All rights reserved.

 This program, PCI-DAS1602-16, is free software; you can redistribute it
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
 *   Note: Interrupts, when enabled, are generated differently depending
 *   upon the mode of the board:
 *
 *   Soft Convert:   poll IRQDATA for A/D done.
 *   Pacer clock:    interrupts generated with FIFO half full
 *                   or when Total_Count = 0.
 *
 *   Documentation:
 *       PCI-DAS1602/16 Manual Revision 1, June 1997, ComputerBoards Inc.
 *       125 High Street, #6, Mansfield, MA 02048
 *       (508) 261-1123 www.computerboards.com
 *
 *       Calibrating the PCI-DAS1602/16 Rev. E  Feb.18, 1997
 *       Internal ComputerBoards Memo
 *
 *       Reading and Writing to a serial NVRAM S5933, Applied Micro Circuits
 *       Corp. Design Note:  http://www.amcc.com/Products/PCI/S5933.htm
 *       Programming the S5933 NVRAM
 *
 *       PCI Products Data S5933, Applied Micro Circuits
 *       Corp.  http://www.amcc.com/Products/PCI/S5933.htm
 *
 *       
 */

/***************************************************************************
 *
 * a2dc.c
 *
 ***************************************************************************/

#ifndef __KERNEL__
#define __KERNEL__
#endif
#ifndef MODULE
#define MODULE
#endif

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/config.h>       /* for CONFIG_PCI */
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include "pci-das1602-16.h"
#include "a2dc.h"

#ifdef CONFIG_PCI
#include <linux/pci.h>
#endif


/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int  SoftRead(ADC_ChanRec *chan);
static int  PacerRead(ADC_ChanRec *chan);
static void SoftWrite(int channel, int gain, u16 value);
static void PacerWrite(int channel);
static int  SetADCPacerFreq(BoardRec *board);
static int  SetDACPacerFreq(BoardRec *board);
static int  SetADCChannelMux(u8 lowChan, u8 highChan);
static int  SetADCGain(u16 gain);
static u16 SetDACGain(int channel, u16 gain);
static void StartADCPacer(void);
static void StopADCPacer(void);
static void StopDACPacer(void);
static void LoadADCPacer(BoardRec *board);
static void LoadDACPacer(BoardRec *board);
static void LoadCounter0(u32 arg);
static void LoadResidualCount(u16 resCount);
static void SetTrigger(u32 arg, int minor );

static void pci_das1602_AD_HalfFullInt(void);
static void pci_das1602_AD_PretrigInt(void);
static void pci_das1602_AD_NotEmptyInt(void);
static void pci_das1602_DA_FIFOEmptyInt(void);
static void pci_das1602_DA_HalfFullInt(void);

static int __init das1602_init(void);
static void __exit das1602_exit (void);
static ssize_t das1602_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t das1602_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int das1602_open(struct inode *iNode, struct file *filePtr);
static int das1602_close(struct inode *iNode, struct file *filePtr);
static int das1602_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg);
static void das1602_Interrupt(int irq, void *dev_id, struct pt_regs *regs);

static void calibrate_pci_das1602_aout( int channel, int gain );
static void calibrate_pci_das1602_ain( int gain );
static u8 get_pci_das1602_nVRam( u16 addr );
static void write_pci_das1602_8800 ( u8 addr, u8 value );
static void write_pci_das1602_8402 ( u8 addr, u8 value );
static void write_pci_das1602_dac08 ( u8 value );
static int config_region( unsigned int pci_ioaddr, int size, u16 *base );

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI DAS 1602/16 module");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

module_init(das1602_init);
module_exit(das1602_exit);


/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/

static u16 base0;                          /* base register 0 address */
static u16 base1;                          /* base register 1 address */
static u16 base2;                          /* base register 2 address */
static u16 base3;                          /* base register 3 address */
static u16 base4;                          /* base register 4 address */
static int WordsToRead = 0;                 /* number of A/D conversions until done  */
static int WordsToWrite = 0;                /* number of D/A conversions until done  */
static u8 dio_reg = 0x80;                 /* DIO control register                  */
static ADC_ChanRec *ADC_CurrChan = NULL;    /* pointer to ChanADC[channel]           */
static DAC_ChanRec *DAC_CurrChan = NULL;    /* pointer to ChanDAC[channel            */
static int MajorNumber = DEFAULT_MAJOR_DEV; /* Major number compiled in              */
static ADC_ChanRec ChanADC[AD_CHANNELS];    /* ADC Channel specific information      */
static DAC_ChanRec ChanDAC[DA_CHANNELS];    /* DAC Channel specific information      */
static DIO_ChanRec ChanDIO[DIO_PORTS];      /* DIO Channel specific information      */
static BoardRec BoardData;                  /* Board specific information            */
static u16 ADC_KernBuff[MAX_COUNT];        /* Kernel buffer where samples are       */
                                            /* saved before write to user space      */
static u16 *ADC_KernBuffPtr;               /* pointer to kernbuf                    */
static u16 DAC_KernBuff[MAX_COUNT];        /* Kernel buffer for DAC                 */
static u16 *DAC_KernBuffPtr;               /* pointer to DAC_KernBuff               */
static int PreTrigMode = FALSE;             /* True = 1, False = 0                   */
static int PreTrigIndexCounter;
static int PreTrigBufSize;                  /* number of bytes in circular buffer    */
static int PreTrigBufCnt;                   /* offset into circular buffer           */

static DECLARE_WAIT_QUEUE_HEAD(das1602_16_wait);

/***************************************************************************
 *
 *
 ***************************************************************************/

static struct file_operations das1602_fops = {
  owner:          THIS_MODULE,
  read:           das1602_read,
  write:          das1602_write,
  ioctl:          das1602_ioctl,
  open:           das1602_open,
  release:        das1602_close,
};

#ifdef __cplusplus
extern "C" {
#endif

/* 
   Toggles CHI_EN and CLO_EN bits high in the Trigger control register.
   This is part of hysteresis mode initialization
*/

inline void toggleHys(void)
{
  outw(CHI_EN | CLO_EN, TRIG_REG);
  outw(0x0, TRIG_REG);
}

/***************************************************************************
 *
 * Loads driver. Called when "insmod pci-das1602-16.o" is invoked on the 
 *               command line. The board is set to IRQ.
 *
 ***************************************************************************/

static int __init das1602_init(void) 
{
  struct pci_dev *dev = NULL;
  int pci_index;
  unsigned char pci_bus, pci_device_fn;
  unsigned int pci_ioaddr;
  int i, err;
  u16 wReg;

#ifndef CONFIG_PCI
    /* this should not happen. */
    printk("%s pci not configured with kernel!\n", ADAPTER_ID);
    return -ENODEV;
#endif

  /* Register as a device with kernel.  */

  if ((err = register_chrdev(MajorNumber, "pci-das1602/16", &das1602_fops))) {
      printk("%s: Failure to load module. error %d\n", ADAPTER_ID, -err);
      return err;
  } 

  /* Register the region of IO space with kernel and get base address */

  if(pci_present()) {
    for ( pci_index = 0; pci_index < MAX_BOARDS; pci_index++ ) {
      if( (dev = pci_find_device( PCI_VENDOR_ID_CBOARDS, 
				  PCI_DEVICE_ID_CBOARDS_DAS1602_16, 
				  dev)) == 0 ) break;
      pci_bus = dev->bus->number;
      pci_device_fn = dev->devfn;

      /* Get PCI_BASE_ADDRESS_0 */
      /* Strip the I/O address out of the returned value */
      pci_ioaddr = pci_resource_start(dev, 0);
      if (config_region( pci_ioaddr, BADR0_SIZE, &base0 ) != 0 ) {
        printk("%s: Can't allocate region port address %#x\n", 
                ADAPTER_ID, pci_ioaddr );
        if (unregister_chrdev(MajorNumber, "pci-das1602/16") != 0) {
          printk("%s: unregister_chrdev() failed.\n", ADAPTER_ID);
        }
        return -ENODEV;
      }
      if ( pci_ioaddr != base0 ) {
          pci_ioaddr = base0 | 0x1;
          pcibios_write_config_dword( pci_bus, pci_device_fn,
                        PCI_BASE_ADDRESS_0, pci_ioaddr );
      }

      /* Get PCI_BASE_ADDRESS_1 */
      /* Strip the I/O address out of the returned value */
      pci_ioaddr = pci_resource_start(dev, 1);
      if (config_region( pci_ioaddr, BADRn_SIZE, &base1 ) != 0 ) {
        printk("%s: Can't allocate region port address %#x\n", 
                ADAPTER_ID, pci_ioaddr );
        if (unregister_chrdev(MajorNumber, "pci-das1602/16") != 0) {
          printk("%s: unregister_chrdev() failed.\n", ADAPTER_ID);
        }
        return -ENODEV;
      }
      if ( pci_ioaddr != base1 ) {
          pci_ioaddr = base1 | 0x1;
          pcibios_write_config_dword( pci_bus, pci_device_fn,
                        PCI_BASE_ADDRESS_1, pci_ioaddr );
      }

      /* Get PCI_BASE_ADDRESS_2 */
      /* Strip the I/O address out of the returned value */
      pci_ioaddr = pci_resource_start(dev, 2);
      if (config_region( pci_ioaddr, BADRn_SIZE, &base2 ) != 0 ) {
        printk("%s: Can't allocate region port address %#x\n", 
                ADAPTER_ID, pci_ioaddr );
        if (unregister_chrdev(MajorNumber, "pci-das1602/16") != 0) {
          printk("%s: unregister_chrdev() failed.\n", ADAPTER_ID);
        }
        return -ENODEV;
      }
      if ( pci_ioaddr != base2 ) {
          pci_ioaddr = base2 | 0x1;
          pcibios_write_config_dword( pci_bus, pci_device_fn,
                       PCI_BASE_ADDRESS_2, pci_ioaddr );
      }

      /* Get PCI_BASE_ADDRESS_3 */
      /* Strip the I/O address out of the returned value */
      pci_ioaddr = pci_resource_start(dev, 3);
      if (config_region( pci_ioaddr, BADRn_SIZE, &base3 ) != 0 ) {
        printk("%s: Can't allocate region port address %#x\n", 
                ADAPTER_ID, pci_ioaddr );
        if (unregister_chrdev(MajorNumber, "pci-das1602/16") != 0) {
          printk("%s: unregister_chrdev() failed.\n", ADAPTER_ID);
        }
        return -ENODEV;
      }
      if ( pci_ioaddr != base3 ) {
          pci_ioaddr = base3 | 0x1;
          pcibios_write_config_dword( pci_bus, pci_device_fn,
                        PCI_BASE_ADDRESS_3, pci_ioaddr );
      }

      /* Get PCI_BASE_ADDRESS_4 */
      /* Strip the I/O address out of the returned value */
      pci_ioaddr = pci_resource_start(dev, 4);
      if (config_region( pci_ioaddr, BADRn_SIZE, &base4 ) != 0 ) {
        printk("%s: Can't allocate region port address %#x\n", 
                ADAPTER_ID, pci_ioaddr );
        if (unregister_chrdev(MajorNumber, "pci-das1602/16") != 0) {
          printk("%s: unregister_chrdev() failed.\n", ADAPTER_ID);
        }
        return -ENODEV;
      }
      if ( pci_ioaddr != base4 ) {
          pci_ioaddr = base4 | 0x1;
          pcibios_write_config_dword( pci_bus, pci_device_fn,
                        PCI_BASE_ADDRESS_4, pci_ioaddr );
      }

      /* Register interrupt handler. */

      BoardData.irq = dev->irq;
      if (request_irq(dev->irq, das1602_Interrupt, (SA_INTERRUPT | SA_SHIRQ), 
          "pci-das1602/16", (void *) PCI_VENDOR_ID_CBOARDS)) {
        /* No free irq found! cleanup and exit */
        printk("%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData.irq);
        if (unregister_chrdev(MajorNumber, "pci-das1602/16") != 0) { 
          printk("%s: unregister_chrdev() failed.\n", ADAPTER_ID);
	}
        release_region( base0,  BADR0_SIZE );       /* release region */
        release_region( base1,  BADRn_SIZE );       /* release region */
        release_region( base2,  BADRn_SIZE );       /* release region */
        release_region( base3,  BADRn_SIZE );       /* release region */
        release_region( base4,  BADRn_SIZE );       /* release region */
        return -EADDRNOTAVAIL;
        break;
      }
    }

    if ( pci_index == 0 ) {
      printk("%s No device present.\n", ADAPTER_ID);
      return -ENODEV;
    }
  } else {
    printk("%s No pci bios present.\n", ADAPTER_ID);
    return -ENODEV;
  }
  BoardData.base0 = base0;
  BoardData.base1 = base1;
  BoardData.base2 = base2;
  BoardData.base3 = base3;
  BoardData.base4 = base4;

  /* Set all channel structures to show nothing active/open */
  for (i = 0; i < AD_CHANNELS; i++) {
      ChanADC[i].open = FALSE;
      ChanADC[i].lowChan = i;
      ChanADC[i].hiChan = i;
      ChanADC[i].gain = UP_10_00V;   /* set range to 0-10V */
                                     /* Differential Mode  */
      ChanADC[i].nSpare2 = 0;
      ChanADC[i].pretrigCount = 0;
  }

  for ( i = 0; i < DA_CHANNELS; i++ ) {
      ChanDAC[i].open = FALSE;
      ChanDAC[i].gain = UP_10_00V;   /* set range to 0-10V */
  }

  ChanDAC[0].threshold = 0x0;      /* CLO Threshold                */
  ChanDAC[1].threshold = 0xffff;   /* CHI Threshold                */

  ChanDIO[0].addr = DIO_PORTA;
  ChanDIO[1].addr = DIO_PORTB;
  ChanDIO[2].addr = DIO_PORTC;

  for ( i = 0; i < DIO_PORTS; i++ ) {
    ChanDIO[i].open = FALSE; 
    ChanDIO[i].mode = 0;
    outb(0x0, ChanDIO[i].addr);
  }

  init_waitqueue_head(&das1602_16_wait);

  /* Reset FIFO interrupts and disable all interupts */
  wReg = (ADFLCL | INTCL | EOACL | DAHFCL | DAEMCL );
  BoardData.nSpare0 = 0x0;
  outw(wReg, IRQ_REG);

  /* Clear TRIG_REG register */
  outw(0x0, TRIG_REG);
  BoardData.nSpare2 = 0x0;

  /* Clear DAC empty interrupt.  Select unipolar 10V
     range for both DACS and zero them out by calling
     pci1602_AOut.  Set BoardData.nSpare3 to zero to
     force calibration.
  */
  BoardData.nSpare3 = 0x0;
  SoftWrite( 0, UP_10_00V, 0x0 );
  SoftWrite( 1, UP_10_00V, 0x0 );

  BoardData.DAC_freq = DEFAULT_FREQ;   /* Set default pacer clock frequency */
  SetDACPacerFreq(&BoardData);

  /* Clear DAC FIFO */
  outw(0x1, ADC_FIFO_CLR);

  /* Load the ADC calibration coeffs for unipolar 10V */
  calibrate_pci_das1602_ain( UP_10_00V );

  BoardData.ADC_freq = DEFAULT_FREQ;   /* Set default pacer clock frequency */
  SetADCPacerFreq(&BoardData);

  /* Reset Interrupt on AMCC5933 PCI controller */
  outl(INTCSR_DWORD, INTCSR_ADDR);
  outl(BMCSR_DWORD, BMCSR_ADDR);

  BoardData.busyRead = FALSE;          /* board ready */
  BoardData.busyWrite = FALSE;          /* board ready */

  printk("%s: BADR0=%#x BADR1=%#x BADR2=%#x BADR3=%#x BADR4=%#x\n",
          ADAPTER_ID, base0, base1, base2, base3, base4);
  printk("%s: IRQ=%d 10/3/97 wjasper@ncsu.edu\n", ADAPTER_ID, BoardData.irq);

  return 0;
}

/***************************************************************************
 *
 * Test memory locations for exsistance of pci-das1602/16 board.
 *
 ***************************************************************************/
static int config_region( unsigned int pci_ioaddr, int size, u16 *base )
{
  /* check to see if the allocated region is ok */
  if ( check_region(pci_ioaddr, size) == 0 ) {
    request_region( pci_ioaddr, size, "pci-das1602/16" );
    *base = pci_ioaddr;
    return 0;
  } else {
    /* Look to see if there is another place in the io space */
    for ( pci_ioaddr = PORT_MIN; pci_ioaddr < PORT_MAX; pci_ioaddr += PORT_STEP ) {
      if ( check_region(pci_ioaddr,  size ) == 0 ) {
        request_region( pci_ioaddr, size, "pci-das1602/16" );
        *base = pci_ioaddr;
        return 0;
      }
    }
    
    if ( pci_ioaddr > PORT_MAX ) {
      printk("%s: No free address from %#x to %#x.\n", 
             ADAPTER_ID, PORT_MIN, PORT_MAX);
      if (unregister_chrdev(MajorNumber, "pci-das1602/16") != 0) {
          printk("%s: unregister_chrdev() failed.\n", ADAPTER_ID);
      }
      return -ENODEV;
    }
  }
  return 1;
}

/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-das1602/16" is run on the command line.
 *
 ***************************************************************************/

void __exit das1602_exit(void) 
{
  if (MOD_IN_USE) {
    printk("%s: device busy, remove delayed.\n", ADAPTER_ID);
    return;
  }

  StopADCPacer();
  StopDACPacer();
  release_region( base0,  BADR0_SIZE );       /* release region */
  release_region( base1,  BADRn_SIZE );       /* release region */
  release_region( base2,  BADRn_SIZE );       /* release region */
  release_region( base3,  BADRn_SIZE );       /* release region */
  release_region( base4,  BADRn_SIZE );       /* release region */
  
  free_irq(BoardData.irq, (void *) PCI_VENDOR_ID_CBOARDS);

  if (unregister_chrdev(MajorNumber, "pci-das1602/16") != 0) {
    printk("%s: cleanup_module failed.\n", ADAPTER_ID);
  } else {

  #ifdef DEBUG
      printk("%s: module removed.\n", ADAPTER_ID);
  #endif

  }    
}

#ifdef __cplusplus
}
#endif

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int das1602_open(struct inode *iNode, struct file *filePtr)
{
  int minor = MINOR(iNode->i_rdev);
  int port;
  unsigned long flags;
  u16 wReg;

  /* 
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes 
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */

  if ( minor >= 0 && minor < AD_CHANNELS ) {
    if ( ChanADC[minor].open == TRUE ) {
      return -EBUSY;
    }

    MOD_INC_USE_COUNT;
    ChanADC[minor].open = TRUE;                     /* The device is open          */
    ChanADC[minor].gain = BP_10_00V;                /* +/- 10V , Differential Mode */
    ChanADC[minor].pacerSource = filePtr->f_flags;  /* set acquisition mode        */
    ChanADC[minor].nSpare2 = 0x0;
    ChanADC[minor].lowChan = minor;
    ChanADC[minor].hiChan = minor;
    ChanADC[minor].pretrigCount = 0;                /* turn pretrigger off         */

    /* Reset FIFO interrupts and disable all A/D interupts */
    wReg = (ADFLCL | INTCL | EOACL);
    save_flags(flags);
    cli();
    BoardData.nSpare0 &= ~(EOAIE | INTE);
    restore_flags(flags);
    outw(BoardData.nSpare0 | wReg, IRQ_REG);

    /* Clear TRIG_REG register */
    outw(0x0, TRIG_REG);
    BoardData.nSpare2 = 0x0;

    #ifdef DEBUG
        printk("%s: open(): minor %d mode %d.\n", ADAPTER_ID, minor, ChanADC[minor].pacerSource);
    #endif
    return 0;   
  }

  if ( (minor >= AD_CHANNELS) && (minor < AD_CHANNELS+DIO_PORTS) ) {
    port = minor - AD_CHANNELS;
    if ( ChanDIO[port].open == TRUE ) {
      return -EBUSY;
    }

    MOD_INC_USE_COUNT;
    ChanDIO[port].open = TRUE;                 /* The device is open */
    ChanDIO[port].f_flags = filePtr->f_flags;

    #ifdef DEBUG
      printk("%s: open(): minor %d mode %d.\n", ADAPTER_ID, minor, ChanDIO[port].mode);
    #endif
    return 0;
  }

  if ( (minor >= AD_CHANNELS+DIO_PORTS) && (minor < AD_CHANNELS+DIO_PORTS+DA_CHANNELS) ) {
    port = minor - AD_CHANNELS - DIO_PORTS;
    if ( ChanDAC[port].open == TRUE ) {
      return -EBUSY;
    }

    MOD_INC_USE_COUNT;
    ChanDAC[port].open = TRUE;                 /* The device is open */
    ChanDAC[port].pacerSource = filePtr->f_flags;
    ChanDAC[port].recycle = FALSE;

    if (port == 0) {
      ChanDAC[port].threshold = 0x0;
    } else {
      ChanDAC[port].threshold = 0xffff;
    }

    /* Reset FIFO interrupts and disable all D/A interupts */
    wReg = (DAHFCL | DAEMCL);
    save_flags(flags);
    cli();
    BoardData.nSpare0 &= ~(DAHFIE | DAEMIE);
    restore_flags(flags);
    outw(BoardData.nSpare0 | wReg, IRQ_REG);

    #ifdef DEBUG
      printk("%s: open(): minor %d mode %d.\n", ADAPTER_ID, minor, ChanDAC[port].pacerSource);
    #endif

  }
  return 0;
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int das1602_close(struct inode *iNode, struct file *filePtr)
{
  int minor = MINOR(iNode->i_rdev);
  int port;

  MOD_DEC_USE_COUNT;

  if ( minor >= 0 && minor < AD_CHANNELS ) {
    /* ADC */
    ChanADC[minor].open = FALSE;
  } else if ( minor >= AD_CHANNELS && minor < AD_CHANNELS + DIO_PORTS ) {
    /* DIO */
    port = minor - AD_CHANNELS;
    ChanDIO[port].open = FALSE;
  } else if ( minor >= AD_CHANNELS+DIO_PORTS && minor < AD_CHANNELS+DIO_PORTS+DA_CHANNELS ) {
    /* DAC */
    port = minor - AD_CHANNELS - DIO_PORTS;
    ChanDAC[port].recycle = FALSE;
    StopDACPacer();
    BoardData.nSpare3 &= ~(HS1 | HS0 | DAPS1 | DAPS0 | START);
    outw(BoardData.nSpare3, DAC_REG);
    BoardData.nSpare0 &= ~(DAEMIE | DAHFIE);
    outw(BoardData.nSpare0 | DAEMCL | DAHFCL, IRQ_REG);
    BoardData.busyWrite = FALSE;

    ChanDAC[port].gain = UP_10_00V;       /* set range to 0-10V */
    SoftWrite( port, UP_10_00V, 0x0 );    /* zero out the port */
    ChanDAC[port].open = FALSE;
  } else {
    printk("das1602_close: Incorrect minor number (%d).\n", minor);
  }  

  #ifdef DEBUG
      printk("%s: close() of minor number %d.\n", ADAPTER_ID, minor);
  #endif

  return 0;
}

/***************************************************************************
 *
 * read() service function
 *
 ***************************************************************************/

static ssize_t das1602_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  u8  bReg;
  int   minor;
  int   port;
  int   start, stop;
  register int i;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = MINOR(iNode->i_rdev);

  if ( BoardData.busyRead == TRUE ) {   /* if board is in use, return busy */
    return (-EBUSY);
  } else {
    BoardData.busyRead = TRUE;
  }

  if ( count < 1 ) {
    printk("das1602_read(): count must be greater than 0.\n");
    BoardData.busyRead = FALSE;
    return (-1);
  }

  if ( minor >= 0 && minor < AD_CHANNELS ) {
    if ( (ChanADC[minor].count = count) > MAX_COUNT ) {
        printk("das1602_read(): requesting too large a count size.\n");
        BoardData.busyRead = FALSE;
        return (-1);
    }

    /* Read */

    switch (ChanADC[minor].pacerSource) {

      case ADC_SOFT_CONVERT:
          #ifdef DEBUG
              printk("das1602_read(): Entering ADC_SOFT_CONVERT mode.  count = %d\n",
                      count);
          #endif

          if (SoftRead(&ChanADC[minor])) {
              printk("das1602_read: SoftTrigRead() failed.\n");
              BoardData.busyRead = FALSE;
              return(-1);
          } 
          break;

      case ADC_EXTERNAL_PACER_FALLING:
        #ifdef DEBUG
            printk("das1602_read(): Entering ADC_EXTERNAL_PACER_FALLING mode.\n");
        #endif

        /* PreTrigger */
         if ( ChanADC[minor].pretrigCount ) {
           if ( count < ChanADC[minor].pretrigCount + PACKETSIZE ) {
             printk("das1602_read(): TotalCount (%d) must be greater than PretrigCount (%d) + %d.\n",
                     count, ChanADC[minor].pretrigCount, PACKETSIZE);
             return (-1);
	   }
	 }

         BoardData.nSpare1 |= ADPS1;           /* Select Pacer Source */
         BoardData.nSpare1 &= ~ADPS0;
         if (PacerRead(&ChanADC[minor])) {
            printk("das1602_read: PacerRead() failed.\n");
            BoardData.busyRead = FALSE;
            return(-1);
	 }
         break;

      case ADC_EXTERNAL_PACER_RISING:
        #ifdef DEBUG
            printk("das1602_read(): Entering ADC_EXTERNAL_PACER_RISING mode.\n");
        #endif

        /* PreTrigger */
         if ( ChanADC[minor].pretrigCount ) {
           if ( count < ChanADC[minor].pretrigCount + PACKETSIZE ) {
             printk("das1602_read(): TotalCount (%d) must be greater than PretrigCount (%d) + %d.\n",
                     count, ChanADC[minor].pretrigCount, PACKETSIZE);
             return (-1);
	   }
	 }

         BoardData.nSpare1 |= ADPS1;           /* Select Pacer Source */
         BoardData.nSpare1 |= ADPS0;

         if (PacerRead(&ChanADC[minor])) {
            printk("das1602_read: PacerRead() failed.\n");
            BoardData.busyRead = FALSE;
            return(-1);
	 }
         break;

      case ADC_PACER_CLOCK:
          #ifdef DEBUG
              printk("das1602_read(): Entering ADC_PACER_CLOCK mode.\n");
          #endif

          if ( ChanADC[minor].count == 1 ) {            /* use SoftRead */
              if (SoftRead(&ChanADC[minor])) {
                  printk("das1602_read: SoftRead() failed with pacer.\n");
                  BoardData.busyRead = FALSE;
                  return(-1);
              }
          } else {
              StopADCPacer();                        /* disable pacer if in pacer mode */
              BoardData.nSpare1 &= ~ADPS1;           /* Select Pacer Source */
              BoardData.nSpare1 |=  ADPS0;
              if (PacerRead(&ChanADC[minor])) {
                  printk("das1602_read: PacerRead() failed.\n");
                  BoardData.busyRead = FALSE;
                  return(-1);
              } 
          }
          break;
    }                  /* end switch */

    /* Check for overflows */

    /* Write data to user space */
    if (ChanADC[minor].count == 1) {
      put_user(*ADC_KernBuff, (u16*) buf);
    } else  {
      if ( ChanADC[minor].pretrigCount && (ChanADC[minor].pretrigCount > PreTrigIndexCounter) ) {
        /* Some pretrig values in the circular buffer.  Reorder and write to buf */
        stop = PreTrigBufCnt;             /* last offset in circular buffer */
        if ( stop == 0 ) stop += PreTrigBufSize;
         start =  stop - (ChanADC[minor].pretrigCount - PreTrigIndexCounter);
         start += PreTrigBufSize;           /* make sure offset is positive number */
         start %= PreTrigBufSize;           /* and located in the ring buffer */
         if ( stop > start ) {
           i = (stop - start)*sizeof(u16);
	   if ( copy_to_user(buf, &ADC_KernBuff[start], i) ) return -EFAULT;
           buf += i;
	 } else {
           i = (PreTrigBufSize - start)*sizeof(u16);
	   if ( copy_to_user(buf, &ADC_KernBuff[start], i) ) return -EFAULT;
           buf += i;
           i = stop*sizeof(u16);
	   if ( copy_to_user(buf, ADC_KernBuff, i) ) return -EFAULT;
           buf += i;
	 }
         i = ChanADC[minor].count - ChanADC[minor].pretrigCount + PreTrigIndexCounter;
	 if ( copy_to_user(buf, &ADC_KernBuff[PreTrigBufSize], i*sizeof(u16)) ) return -EFAULT;
      } else {
        if ( copy_to_user(buf, ADC_KernBuff, ChanADC[minor].count*sizeof(u16)) ) return -EFAULT;
      }
    }

    BoardData.busyRead = FALSE;
    return(ChanADC[minor].count);     /* return number of samples read */
  }

  /* check to see if reading a value from the DIO */

  if ( minor >= AD_CHANNELS && minor < AD_CHANNELS + DIO_PORTS ) {
    port = minor - AD_CHANNELS;
    bReg = inb(ChanDIO[port].addr);
    put_user(bReg, (u8*) buf);

    #ifdef DEBUG
       printk("das1602_read: %s DIO Port %#x addr %#x set to %#x\n", 
                             ADAPTER_ID, port, ChanDIO[port].addr, bReg);
    #endif
    BoardData.busyRead = FALSE;
    return 1;
  }

  printk("das1602_read: Incorrect minor number (%d).\n", minor);
  BoardData.busyRead = FALSE;
  return -1;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t das1602_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  int  minor;
  int  port;
  u8 bReg;
  register int i;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = MINOR(iNode->i_rdev); 

  if ( BoardData.busyWrite == TRUE ) {   /* if board is in use, return busy */
      return (-EBUSY);
  } else {
    BoardData.busyWrite = TRUE;
  }

  #ifdef DEBUG
    printk("das1602_write(): Minor = %d, Count = %d\n", minor, count);
  #endif

  if ( count < 1 ) {
    printk("das1602_write(): count must be greater than 0.\n");
    BoardData.busyWrite = FALSE;
    return (-1);
  }

  if ( count > MAX_COUNT ) {
    printk("das1602_write(): count must be less than %d.\n", MAX_COUNT);
    BoardData.busyWrite = FALSE;
    return (-1);
  }

  /* check to see if writing a value to the DIO */
  if ( (minor >= AD_CHANNELS) && (minor < AD_CHANNELS+DIO_PORTS) ) {
    port = minor - AD_CHANNELS;
    get_user(bReg, (u8*)buf);
    ChanDIO[port].value = bReg;
    outb( bReg, ChanDIO[port].addr);

    #ifdef DEBUG
    printk("das1602_write %s: DIO Port %#x set to %#x\n", ADAPTER_ID, port, bReg);
    #endif

    BoardData.busyWrite = FALSE;
    return 1;
  } 

  if ( (minor >= AD_CHANNELS+DIO_PORTS) && (minor < AD_CHANNELS+DIO_PORTS+DA_CHANNELS) ) {
    port = minor - AD_CHANNELS - DIO_PORTS;
    ChanDAC[port].count = count;

    /* Read data from user space */
    if (ChanDAC[port].count == 1) {
      get_user(DAC_KernBuff[0], (u16*)buf);
    } else {
      if ( copy_from_user(DAC_KernBuff, buf, count*sizeof(u16)) ) return -EFAULT;
    }

    switch (ChanDAC[port].pacerSource) {

      case DAC_SOFT_CONVERT:
        #ifdef DEBUG
            printk("das1602_write(): Entering DAC_SOFT_CONVERT mode. port = %d, gain = %d\n",
                    port, ChanDAC[port].gain );
        #endif
        for ( i = 0; i < count; i++ ) {
            SoftWrite( port, ChanDAC[port].gain, DAC_KernBuff[i] );
	}
        BoardData.busyWrite = FALSE;
        return count;
        break;

      case DAC_PACER_CLOCK:
        #ifdef DEBUG
            printk("das1602_write(): Entering DAC_PACER_CLOCK mode.\n");
        #endif
        PacerWrite( port );
        break;

      case DAC_FALLING_EDGE:
        #ifdef DEBUG
            printk("das1602_write(): Entering DAC_FALLING_EDGE mode.\n");
        #endif
        PacerWrite( port );
        break;

      case DAC_RISING_EDGE:
        #ifdef DEBUG
            printk("das1602_write(): Entering DAC_RISING_EDGE mode.\n");
        #endif
        PacerWrite( port );
        break;
    }
    return(ChanDAC[port].count);     /* return number of samples written */
  }
  return 1;
}

/***************************************************************************
 *
 * iotctl() service handler
 *
 ***************************************************************************/

/* Note:
    Remember that FIOCLEX, FIONCLEX, FIONBIO, and FIOASYN are reserved ioctl cmd numbers
*/

static int das1602_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  int minor = MINOR(iNode->i_rdev);
  int port;
  int channel;
  int err = 0;
  int size = _IOC_SIZE(cmd);       /* the size bitfield in cmd */

  /* 
   * extract the type and number bitfields, and don't decode
   * wrong cmds;  return EINVAL before verify_area()
   */

  if (_IOC_TYPE(cmd) != IOCTL_MAGIC) return -EINVAL;
  if (_IOC_NR(cmd) > IOCTL_MAXNR)  return -EINVAL;

  if (_IOC_DIR(cmd) & _IOC_READ) {
    err = verify_area(VERIFY_WRITE, (void *)arg, size);
  } else if (_IOC_DIR(cmd) & _IOC_WRITE) {
     err = verify_area(VERIFY_READ, (void *)arg, size);
  }
  if (err) return err;

  /* global ioctl calls, not specific to A/D, D/A, or DIO */
  switch (cmd) {
    case GET_BUF_SIZE:
      put_user( (long) MAX_COUNT, (long*) arg );
      return 0;
      break;
  }

  if ( minor >= 0 && minor < AD_CHANNELS ) {
    switch (cmd) {
      case ADC_SET_GAINS:
        #ifdef DEBUG
          printk("ioctl ADC_SET_GAINS: Channel = %d, Gain = %ld\n", minor, arg);
        #endif
        ChanADC[minor].gain &= ~(UNIBIP | GS1 | GS0);
        ChanADC[minor].gain |=  (u16) arg & (UNIBIP | GS1 | GS0);
        break;
      case ADC_GET_GAINS:
	put_user( (long)ChanADC[minor].gain, (long*) arg);
        break;
      case ADC_SET_PACER_FREQ:
        if ( arg > MAX_AD_FREQ ) {
            printk("ioctl:  Can not set frequency %lu greater than %d.\n", arg, MAX_AD_FREQ );
            return -1;
        } else {
            BoardData.ADC_freq = (u32)arg;
            SetADCPacerFreq(&BoardData);
            LoadADCPacer(&BoardData);               /* load the board frequency now */
        }
        break;
      case ADC_GET_PACER_FREQ:
	put_user(BoardData.ADC_freq, (long*) arg);
        break;
      case ADC_START_PACER:
        StartADCPacer();
        break;
      case ADC_STOP_PACER:
        StopADCPacer();
        break;
      case ADC_COUNTER0:
        LoadCounter0( (u32) arg );
        break;
      case ADC_SET_MUX_LOW:
        ChanADC[minor].lowChan = (u8) arg;
        break;
      case ADC_SET_MUX_HIGH:
        ChanADC[minor].hiChan = (u8) arg;
        break;
      case ADC_GET_CHAN_MUX_REG:
        put_user( BoardData.nSpare1, (long*) arg);
        break;
      case ADC_SET_FRONT_END:
        if ( arg ) {
          BoardData.nSpare1 |= SEDIFF;     /* Single Ended Mode */
        } else {
          BoardData.nSpare1 &= ~(SEDIFF);  /* Differential Mode */
	}
        break;
      case ADC_BURST_MODE:
        if (arg == 0) {
          ChanADC[minor].nSpare2 &= ~BURSTE;  /* Burst Mode disabled */
	} else {
          ChanADC[minor].nSpare2 |= BURSTE;   /* Burst Mode enabled */
	}
        break;
      case ADC_SET_TRIGGER:
        SetTrigger(arg, minor);
        break;
      case ADC_PRETRIG:
        /* sets up the borad for pre-trigger.  The PretrigCount value must
           be less than 32768 and also less than TotalCount - 512.
        */
        if ( arg >= 32768 ) {
          printk("ioctl:  Can not set PretrigCount greater than 32768.\n");
          return -EINVAL;
	} else {
          ChanADC[minor].pretrigCount = (u16) arg;
	}
        break;
      default:
        return(-EINVAL);
        break;
    } /* end switch */
  }  /* end if */

  if ( minor >= AD_CHANNELS && minor < AD_CHANNELS+DIO_PORTS ) {
    port = minor - AD_CHANNELS;
    switch (cmd) {
      case DIO_SET_MODE:
        arg &= 0x3;
        switch (port) {
	    case 0:		       /* Port A */
              #ifdef DEBUG
              printk("DIO_SET_MODE for Port A\n");
              #endif
              dio_reg &= 0x9f;
              dio_reg |= (arg << 5);
              outb(dio_reg, DIO_CNTRL_REG);
              ChanDIO[port].mode = arg;
              break;

            case 1:			/* Port 1B */
              #ifdef DEBUG
              printk("DIO_SET_MODE for Port 1B\n");
              #endif
              if (arg & 0x2)
	        return(-EINVAL);	       /* Port 1B only has Modes 0 & 1 */
              dio_reg &=  0xfb;
              dio_reg |= (arg << 2);
              outb(dio_reg, DIO_CNTRL_REG);
              ChanDIO[port].mode = arg;
              break;

            case 2:			/* Port 1C */
              #ifdef DEBUG
              printk("DIO_SET_MODE for Port 1C\n");
              #endif
              if (arg)
	        return(-EINVAL);	/* Port 1C only has Mode 0 I/O */
              ChanDIO[port].mode = arg;
              break;

            default:
              #ifdef DEBUG
              printk("DIO_SET_MODE for Invalid Port\n");
              #endif
              return(-EINVAL);	/* Wrong Port Number */
              break;
        }
        break;
    
      case DIO_SET_DIRECTION:

        #ifdef DEBUG
        printk("DIO_SET_DIRECTION: minor = %d, arg = %d\n", (int) minor, (int) arg );
        #endif

        switch (port) {
          case 0:			/* Port A */
            #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port A\n");
            #endif
            arg &= 0x1;
            dio_reg &=  0xef;
            dio_reg |= (arg << 4);
            outb(dio_reg, DIO_CNTRL_REG);
            break;

          case 1:			/* Port B */
            #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port B\n");
            #endif
            arg &= 0x1;
            dio_reg &=  0xfd;
            dio_reg |= (arg << 1);
            outb(dio_reg, DIO_CNTRL_REG);
            break;

          case 2:			/* Port C */
            #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port C\n");
            #endif
            switch ( arg ) {
              case PORT_INPUT:         /* CU = IN  CL = IN */
                  dio_reg |= 0x9;
                  break;
              case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                  dio_reg &= ~(0x9);
                  break;
              case LOW_PORT_INPUT:     /* CL = IN */
                  dio_reg |= 0x1;
                  break;
              case LOW_PORT_OUTPUT:    /* CL = OUT */
                  dio_reg &= ~(0x1);
                  break;
              case HIGH_PORT_INPUT:    /* CU = IN */
                  dio_reg |= 0x8;
                  break;
              case HIGH_PORT_OUTPUT:   /* CU = OUT */
                  dio_reg &= ~(0x8);
                  break;
              default:
                  return(-EINVAL);
                  break;
            }
            outb(dio_reg, DIO_CNTRL_REG);
            break;

          default:
            #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Invalid Port\n");
            #endif
            return(-EINVAL);
            break;
        }
        break;
    
      default:
        return(-EINVAL);
        break;
    }  /* end switch */
  }    /* end if */

  if ( minor >= AD_CHANNELS+DIO_PORTS && minor < AD_CHANNELS+DIO_PORTS+DA_CHANNELS ) {
    channel = minor - AD_CHANNELS - DIO_PORTS;
    switch (cmd) {
      case DAC_SET_GAINS:
        #ifdef DEBUG
          printk("ioctl DAC_SET_GAINS: Channel = %d, Gain = %ld\n", channel, arg);
        #endif
        ChanDAC[channel].gain = (u16) arg;
        break;
      case DAC_GET_GAINS:
        put_user((long)ChanDAC[channel].gain, (long*) arg);
        break;
      case DAC_SET_PACER_FREQ:
        if ( arg > MAX_DA_FREQ ) {
          printk("ioctl:  Can not set frequency %ld greater than %d.\n", arg, MAX_DA_FREQ );
          return -1;
	} else {
          BoardData.DAC_freq = (u32) arg;
          SetDACPacerFreq(&BoardData);
          LoadDACPacer(&BoardData);
	}
        break;
      case DAC_GET_PACER_FREQ:
        put_user(BoardData.DAC_freq, (long*) arg);
        break;
      case DAC_STOP_PACER:
        StopDACPacer();
        break;
      case DAC_RECYCLE:
        ChanDAC[channel].recycle = arg;
        break;
      case DAC_SET_CLO:
        ChanDAC[0].threshold = (u16) arg;
        break;
      case DAC_SET_CHI:
        ChanDAC[1].threshold = (u16) arg;
        break;
      case DAC_SET_SIMULTANEOUS:
        if ( arg == TRUE ) {  /* set simultaneous */
	  BoardData.nSpare3 |= (HS1 | HS0);
	} else {  /* reset to one channel only */
	  if ( channel == 0 ) {
	    /* HS0 = 1, HS1 = 0 */
	    BoardData.nSpare3 |=  HS0;
            BoardData.nSpare3 &=  ~HS0;
	  } else {
            /* HS0 = 0, HS1 = 1 */
	    BoardData.nSpare3 |= HS1;
            BoardData.nSpare3 &= ~HS1;
	  }
	}
        outw(BoardData.nSpare3, DAC_REG);
        break;
    }
  }
  return 0;
}

/***************************************************************************
 *
 * Interrupt handler used to service interrupt read().
 *
 ***************************************************************************/

static void das1602_Interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
  unsigned long flags;
  u16 IntSource;
 
  /* Check that interrupt came from the pci-das1602/16 board (needed for sharded interrupts) */
  if (inl(INTCSR_ADDR) & 0x00820000) {
    save_flags(flags);
    cli();

    /* Reset Interrupt on AMCC5933 PCI controller  */
    /* Right now, to avoid race condition with simultaneous A/D and D/A */
    outl_p(BMCSR_DWORD, BMCSR_ADDR);
    outl_p(INTCSR_DWORD, INTCSR_ADDR);

    IntSource = inw(IRQ_REG) & 0x6fe0;  /* mask out extraneous garbage */

    #ifdef DEBUG
        printk("Entering das1602_Interrupt(). irq = %d  WordsToRead = %d  WordsToWrite = %d  IntSource = %#hx\n", 
              irq, WordsToRead, WordsToWrite, IntSource );
    #endif

    while (IntSource) {
      if (IntSource & EOAI) {         /* End of acquisition          */
        IntSource &= ~EOAI;
        pci_das1602_AD_HalfFullInt();
      } else if (IntSource & ADHFI) { /* AD-FIFO-half-full interrupt */
        IntSource &= ~ADHFI;
        if ( PreTrigMode ) { 
          pci_das1602_AD_PretrigInt();
        } else {
         pci_das1602_AD_HalfFullInt();
        }
      } else if (IntSource & ADNEI) { /* AD-not-empty                */
        IntSource &= ~ADNEI;
        pci_das1602_AD_NotEmptyInt();
      } else if (IntSource & EOBI) {  /* end-of-burst interrrupt     */
        IntSource &= ~EOBI;
        pci_das1602_AD_HalfFullInt();
      } else if (IntSource & DAEMI) { /* DA-FIFO-empty               */
        IntSource &= ~DAEMI;
        pci_das1602_DA_FIFOEmptyInt();
      } else if (IntSource & DAHFI) { /* DA-FIFO-half-full interrupt */
        IntSource &= ~DAHFI;
        pci_das1602_DA_HalfFullInt();
      } else {
        break;
      }
    }

    if (WordsToRead == 0) { 
      BoardData.nSpare0 &= ~(EOAIE | INTE);
      outw(BoardData.nSpare0, IRQ_REG);
      wake_up_interruptible(&das1602_16_wait);  
    }
    restore_flags(flags);
  }
  return;
}

/***********************************************************************
 *  This routine gets called from das1602_Interrupt() when the         *
 *  interrupt was caused by the FIFO going half full.  It reads half   *
 *  a FIFO's worth of data and stores it in the kernel's arrary.       *
 ***********************************************************************/

static void pci_das1602_AD_HalfFullInt(void)
{
  #ifdef DEBUG
      printk("Entering pci_das1602_AD_HalfFullInt()\n");
  #endif

  /* check for FIFO Overflow errors */
  if (inw(IRQ_REG) & LADFUL) {
    outw(0x1, ADC_FIFO_CLR);
    outw(BoardData.nSpare0 | ADFLCL | INTCL, IRQ_REG);  /* Clear Interrupts */
    printk("pci_das1602_AD_HalfFullInt: LADFUL error\n");
    return;
  }

  /* Caclulate remaining count.  If it's less than PACKET_SIZE then use
     remaining count, otherwise use PACKET_SIZE
  */

  if (WordsToRead  >= FIFO_SIZE) {
    insw(ADC_DATA_REG, ADC_KernBuffPtr, PACKETSIZE);      /* uses rep insw */
    ADC_KernBuffPtr += PACKETSIZE;
    WordsToRead -= PACKETSIZE;
    if (WordsToRead < 2*PACKETSIZE && WordsToRead > PACKETSIZE) {
      BoardData.nSpare2 |= ARM;             /* Arm the resisual at next ADHF */
      outw(BoardData.nSpare2,  TRIG_REG);
    }
  } else if (WordsToRead < 2*PACKETSIZE && WordsToRead > PACKETSIZE) {
    if ( !(BoardData.nSpare2 & ARM)) {
      BoardData.nSpare2 |=  ARM | FFM0;
      outw(BoardData.nSpare2, TRIG_REG);    /* Arm the resisual counter now */
    }
    insw(ADC_DATA_REG, ADC_KernBuffPtr, PACKETSIZE);        /* uses rep insw */
    ADC_KernBuffPtr += PACKETSIZE;
    WordsToRead -= PACKETSIZE;
    /* check for a race condition before leaving */
    if (inw(IRQ_REG) & EOAI) {
      insw(ADC_DATA_REG, ADC_KernBuffPtr, WordsToRead);       /* uses rep insw */
      WordsToRead =0;
    }
  } else {                                              /* we are done */
    insw(ADC_DATA_REG, ADC_KernBuffPtr, WordsToRead);       /* uses rep insw */
    WordsToRead =0;
  }
  outw(BoardData.nSpare0 | INTCL, IRQ_REG);  /* Clear Interrupts */
}

static void pci_das1602_AD_PretrigInt(void)
{
  register int i;

  #ifdef DEBUG
      printk("Entering pci_das1602_AD_PretrigInt()\n");
  #endif

  if ( inw(TRIG_REG) & INDX_GT ) {  
    /* Pre Trigger index counter has completed its count */
    PreTrigMode = FALSE;
    outb(C2+CNTLATCH, COUNTERB_CONTROL);                /* latch the pretrig index */
    PreTrigIndexCounter = inb(COUNTERB_0_DATA);         /* Read LSB */
    PreTrigIndexCounter |= inb(COUNTERB_0_DATA) << 8;   /* Read MSB */
    i = ADC_CurrChan->pretrigCount - PreTrigIndexCounter;
    if ( i > 0 ) {
      ADC_KernBuffPtr = ADC_KernBuff + PreTrigBufSize;
      insw(ADC_DATA_REG, ADC_KernBuffPtr, PACKETSIZE);        /* uses rep insw */
      WordsToRead -= PACKETSIZE - PreTrigIndexCounter;
      ADC_KernBuffPtr += PACKETSIZE;
    } else if ( i < 0 ) {
      i = -i;
      ADC_KernBuffPtr = ADC_KernBuff;
      insw(ADC_DATA_REG, ADC_KernBuffPtr, i);   
      insw(ADC_DATA_REG, ADC_KernBuffPtr, PACKETSIZE-i);
      WordsToRead -= PACKETSIZE - ADC_CurrChan->pretrigCount;
      ADC_KernBuffPtr += PACKETSIZE - i;
    } else { /* i = 0 */
      ADC_KernBuffPtr = ADC_KernBuff;
      insw(ADC_DATA_REG, ADC_KernBuffPtr, PACKETSIZE);
      WordsToRead -= PACKETSIZE - PreTrigIndexCounter;
      ADC_KernBuffPtr += PACKETSIZE;
    }
  } else {
    /* read PACKETSIZE words */
    insw(ADC_DATA_REG, ADC_KernBuffPtr, PACKETSIZE);
    PreTrigBufCnt += PACKETSIZE;           /* increment to next PACKETSIZE chunk */
    PreTrigBufCnt %= PreTrigBufSize;       /* wrap around if at the end          */
    ADC_KernBuffPtr = ADC_KernBuff + PreTrigBufCnt;
  }
}

/***********************************************************************
 *  This routine gets called from das1602_Interrupt() when the         *
 *  interrupt was caused by any data going into the FIFO.  Usually     *
 *  gets called when sampling in SINGLEIO mode with interrupts.  For   *
 *  preformance reasons, I am polling in SoftConvert Mode              *
 ***********************************************************************/

static void pci_das1602_AD_NotEmptyInt(void)
{
  #ifdef DEBUG
      printk("Entering pci_das1602_AD_NotEmptyInt()\n");
  #endif

  /* check for FIFO Overflow errors */
  if (inw(IRQ_REG) & LADFUL) {
    printk("pci_das1602_AD_NotEmptyInt: ADC FIFO exceeded full state. Data may be lost\n");
    return;
  }

  do {
    /* Read a sample from FIFO */
    *ADC_KernBuffPtr++ = inw(ADC_DATA_REG);
    WordsToRead--;
  } while ( WordsToRead && (inw(IRQ_REG) & ADNE) );

  outw(BoardData.nSpare0 | INTCL, IRQ_REG);  /* Clear Interrupts */
}

static void pci_das1602_DA_FIFOEmptyInt(void)
{
  #ifdef DEBUG
      printk("Entering pci_das1602_DA_FIFOEmptyInt()\n");
  #endif

  if ( DAC_CurrChan->recycle ) {
    WordsToWrite = DAC_CurrChan->count;
    DAC_KernBuffPtr = DAC_KernBuff;
    BoardData.nSpare0 &= ~(DAHFIE | DAEMIE);
    if ( WordsToWrite <= FIFO_SIZE ) {
      BoardData.nSpare0 |= DAEMIE;
      /* Load the DAC FIFO */
      outsw(DAC_DATA_REG, DAC_KernBuff, WordsToWrite ); 
      DAC_KernBuffPtr += WordsToWrite;
      WordsToWrite = 0;
    } else {
      BoardData.nSpare0 |= DAHFIE;
      /* Load the DAC FIFO */
      outsw(DAC_DATA_REG, DAC_KernBuff, FIFO_SIZE ); 
      DAC_KernBuffPtr += FIFO_SIZE;
      WordsToWrite -= FIFO_SIZE;
    }
    outw(BoardData.nSpare0 | DAHFCL | DAEMCL, IRQ_REG);
  } else {
    WordsToWrite = 0;

    /* When stopping the aout_scan, be careful NOT to reset the
       gain setting and DACEN. */

    /* stop DAC pacers */
    StopDACPacer();

    /* disable interrupts */
    /* outw(0x0, IRQ_REG); */

    /* disconnect DAC's from pacer */
    BoardData.nSpare3 &= ~(HS1 | HS0 | DAPS1 | DAPS0 | START);
    outw(BoardData.nSpare3, DAC_REG);

    /* Disable and Clear Interrupts */
    BoardData.nSpare0 &= ~(DAEMIE | DAHFIE);
    outw(BoardData.nSpare0 | DAEMCL | DAHFCL, IRQ_REG);
    BoardData.busyWrite = FALSE;
  }
}

static void pci_das1602_DA_HalfFullInt(void)
{
  int recycle_words;

  #ifdef DEBUG
      printk("Entering pci_das1602_DA_HalfFullInt()\n");
  #endif

  if ( WordsToWrite >= PACKETSIZE ) {
    outsw(DAC_DATA_REG, DAC_KernBuffPtr, PACKETSIZE ); 
    DAC_KernBuffPtr += PACKETSIZE;
    WordsToWrite -= PACKETSIZE;
    outw(BoardData.nSpare0 | DAHFCL | DAHFIE, IRQ_REG);
  } else {
    if ( DAC_CurrChan->recycle ) {
      outsw(DAC_DATA_REG, DAC_KernBuffPtr, WordsToWrite );       /* write out remaining words */
      DAC_KernBuffPtr = DAC_KernBuff;                            /* reinitialize the pointer  */
      recycle_words = PACKETSIZE - WordsToWrite;                 /* extra words to wrap       */
      outsw(DAC_DATA_REG, DAC_KernBuff, recycle_words);
      WordsToWrite = DAC_CurrChan->count - recycle_words;        /* number of words remaining */
      DAC_KernBuffPtr += recycle_words;
      outw(BoardData.nSpare0 | DAHFCL | DAHFIE, IRQ_REG);
     } else {
      outsw(DAC_DATA_REG, DAC_KernBuffPtr, WordsToWrite ); 
      DAC_KernBuffPtr += WordsToWrite;
      WordsToWrite = 0;
      BoardData.nSpare0 &= ~DAHFIE;
      BoardData.nSpare0 |=  DAEMIE;
      outw(BoardData.nSpare0 | DAHFCL | DAEMCL, IRQ_REG);
    }
  }
}

/***************************************************************************
 *
 * Handles software/pacer triggered read().
 *
 * Bang! Bang! Method: 
 *    
 *    o Force interrupt by forcing conversion.
 *    o Get one sample at a time and put it in the kernel buffer(kernBuff).
 *    o Get chan->count samples
 *
 ***************************************************************************/

static int SoftRead(ADC_ChanRec *chanRec)
{
  u16 wReg;

  #ifdef DEBUG
      printk("Entering SoftRead().\n");
  #endif

  /* Disable everything first */
  /* outw(0x0, IRQ_REG); */
  outw(0x0, TRIG_REG);

  /* Clear the AD FIFO */
  outw(0x1, ADC_FIFO_CLR);

  /* Set up the ADC Mux and Control register */
  SetADCChannelMux(chanRec->lowChan, chanRec->hiChan);  /* Read only this channel */
  SetADCGain(chanRec->gain);
  BoardData.nSpare1 &= ~(ADPS1 | ADPS0);
  outw(BoardData.nSpare1, MUX_REG);
  udelay(10);    /* wait 10 usecs for things to settle down */

  WordsToRead = chanRec->count;
  ADC_KernBuffPtr = ADC_KernBuff;

  while ( WordsToRead ) {
    outw(0x0, ADC_DATA_REG);  /* Force first conversion */

    /*
       Poll EOC for conversion.  Normally, I would use a interrupt
       handler, but the DAS1602 board is too fast, and we get
       a race condition.  Much better to poll for a single conversion
       here.
    */

    do {
      wReg = inw_p(MUX_REG) & EOC;
    } while (!wReg);

    *ADC_KernBuffPtr++ = (u16) inw_p(ADC_DATA_REG);  /* Load into buffer */
    WordsToRead--;
  }

  #ifdef DEBUG
      printk("SoftRead(): value = %#x.\n", ADC_KernBuff[0]);
  #endif

  return 0;
}

/****************************************************************************
 *   Set ADC Pacer Clock Frequency
 * 
 *   Description: 
 *       Set the counters so that the ADC pacer clock runs at the
 *       desired frequency.  The frequency is generated by dividing
 *       down a 10 MHz clock, so all frequencies can not be generated.
 *       This routine calculated the divisor to generate a frequency
 *       as near as possible to the requested one.  It then calculates
 *       the real frequency and returns it .
 ****************************************************************************/

static int SetADCPacerFreq(BoardRec *board)
{
  u16 ctr1, ctr2;
  u32 product, error, error2;

  if ( board->ADC_freq == 0 ) {
    return -1;
  }

  /* divide 10MHz by frequency */
  product =  (long) 10000000 / board->ADC_freq;

  /* check for rounding error */
  error = abs(10000000 - product*board->ADC_freq);
  error2 = abs(10000000 - (product+1)*board->ADC_freq);
  if ( error2 < error ) product++;

  /* Now the job is to find two 16 bit numbers, that when multiplied
     together are approximately equal to product.  Start by setting
     one of them, ctr1 to 2 (minimum settable value) and increment until
     the error is minimized and ctr2 is less than 32768.

     NOTE: In Mode 2, a value of 1 is illegal! Therefore, crt1 and crt2
     can never be 1.

  */

  ctr1 = product / 32768;
  if ( ctr1 < 2 ) ctr1 = 2;
  ctr2 = product / ctr1;
  error = abs(product - (long) ctr2 * (long) ctr1);

  while ( error && ctr1 < 32768  && ctr2 > 1 ) {
       ctr1++;
       ctr2 = product / ctr1;
       error = abs(product - (long) ctr2 * (long) ctr1);
  }

  /* the frequency is prime, add 1 to it */
  if ( error ) {
      product++;
      ctr1 = product / 32768;
      if ( ctr1 < 2 ) ctr1 = 2;
      ctr2 = product / ctr1;
      error = abs(product - (long) ctr2 * (long) ctr1);

      while ( error && ctr1 < 32768 && ctr2 > 1) {
          ctr1++;
          ctr2 = product / ctr1;
          error = abs(product - (long) ctr2 * (long) ctr1);
      }
  }

  /* we can't have ctr2 equal to 1, or system hangs */
  if ( ctr2 == 1 ) {
      ctr2++;
      ctr1 /= 2;
  }

  board->ADC_ctr1 = ctr1;
  board->ADC_ctr2 = ctr2;
  board->ADC_freq = 10000000/((long)ctr1*(long)ctr2);

  #ifdef DEBUG
      printk("SetADCPacerFreq: Pacer Frequecy set to %ld\n", BoardData.ADC_freq);
  #endif

  return 0;
}

static int SetDACPacerFreq(BoardRec *board)
{
  u16 ctr1, ctr2;
  u32 product, error, error2;

  if ( board->DAC_freq == 0 ) {
    return -1;
  }

  /* divide 10MHz by frequency */
  product =  (long) 10000000 / board->DAC_freq;

  /* check for rounding error */
  error = abs(10000000 - product*board->DAC_freq);
  error2 = abs(10000000 - (product+1)*board->DAC_freq);
  if ( error2 < error ) product++;

  /* 
     Now the job is to find two 16 bit numbers, that when multiplied
     together are approximately equal to product.  Start by setting
     one of them, ctr1 to 2 (minimum settable value) and increment until
     the error is minimized and ctr2 is less than 32768.

     NOTE: In Mode 2, a value of 1 is illegal! Therefore, crt1 and crt2
     can never be 1.
  */

  ctr1 = product / 32768;
  if ( ctr1 < 2 ) ctr1 = 2;
  ctr2 = product / ctr1;
  error = abs(product - (long) ctr2 * (long) ctr1);

  while ( error && ctr1 < 32768  && ctr2 > 1 ) {
       ctr1++;
       ctr2 = product / ctr1;
       error = abs(product - (long) ctr2 * (long) ctr1);
  }

  /* the frequency is prime, add 1 to it */
  if ( error ) {
      product++;
      ctr1 = product / 32768;
      if ( ctr1 < 2 ) ctr1 = 2;
      ctr2 = product / ctr1;
      error = abs(product - (long) ctr2 * (long) ctr1);

      while ( error && ctr1 < 32768 && ctr2 > 1) {
          ctr1++;
          ctr2 = product / ctr1;
          error = abs(product - (long) ctr2 * (long) ctr1);
      }
  }

  /* we can't have ctr2 equal to 1, or system hangs */
  if ( ctr2 == 1 ) {
      ctr2++;
      ctr1 /= 2;
  }

  board->DAC_ctr1 = ctr1;
  board->DAC_ctr2 = ctr2;
  board->DAC_freq = 10000000/((long)ctr1*(long)ctr2);

  #ifdef DEBUG
      printk("SetDACPacerFreq: Pacer Frequecy set to %ld\n", BoardData.DAC_freq);
  #endif

  return 0;
}

/***************************************************************************
 *
 * Load two part frequency to pacer counter chip.
 *
 ***************************************************************************/

static void LoadADCPacer(BoardRec *board)
{
  u8 mask;
  u8 bData;

  /* Write the values of ctr1 and ctr2 into counter A1 and A2 */

  #ifdef DEBUG
      printk("LoadADCPacer: freq = %ld   load values: ctr1 %#x ctr2 %#x\n", 
              board->ADC_freq, board->ADC_ctr1, board->ADC_ctr2);
  #endif

  mask = C2+MODE2+LSBFIRST;
  outb_p(mask, COUNTERA_CONTROL); 

  bData = (u8) (board->ADC_ctr2 & 0xff);
  outb_p(bData, COUNTERA_2_DATA); 

  bData = (u8) (board->ADC_ctr2 >> 8);
  outb_p(bData, COUNTERA_2_DATA); 

  mask = C1+MODE2+LSBFIRST;
  outb_p(mask, COUNTERA_CONTROL); 

  bData = (board->ADC_ctr1 & 0xff);
  outb_p(bData, COUNTERA_1_DATA); 

  bData = (board->ADC_ctr1 >> 8) & 0xff;
  outb_p(bData, COUNTERA_1_DATA); 
}

static void LoadDACPacer(BoardRec *board)
{
  u8 mask;
  u8 bData;

  /* Write the values of ctr1 and ctr2 into counter BADR3 + 9 */

  #ifdef DEBUG
      printk("LoadDACPacer: load values: ctr1 %#x ctr2 %#x\n", board->DAC_ctr1, board->DAC_ctr2);
  #endif

  mask = C2+MODE2+LSBFIRST;
  outb_p(mask, COUNTERB_CONTROL); 

  bData = (u8) (board->DAC_ctr2 & 0xff);
  outb_p(bData, COUNTERB_2_DATA); 

  bData = (u8) (board->DAC_ctr2 >> 8);
  outb_p(bData, COUNTERB_2_DATA); 

  mask = C1+MODE2+LSBFIRST;
  outb_p(mask, COUNTERB_CONTROL); 

  bData = (board->DAC_ctr1 & 0xff);
  outb_p(bData, COUNTERB_1_DATA); 

  bData = (board->DAC_ctr1 >> 8) & 0xff;
  outb_p(bData, COUNTERB_1_DATA); 
}

/***************************************************************************
 *
 * Load value into Counter 0  XXXX    Mode    MSB     LSB
 *                            Byte 3  Byte 2  Byte 1  Byte 0
 *
 ***************************************************************************/

static void LoadCounter0( u32 value )
{
  u8 mask;
  u8 bData;

  /* Write the value into Counter 0 Mode 2 */

  #ifdef DEBUG
      printk("LoadCounter0: load value %#lx into Counter 0.\n", value);
  #endif

  /* the mode is in the thrid byte */
  mask = 0xff & (value >> 16);
  mask += C0+LSBFIRST;
  outb_p(mask, COUNTERB_CONTROL); 

  if ( value & 0xff000000 ) {       /* load control word only */
      return;
  } else {
      /* LSB in the first byte of value */
      bData = (u8) (value & 0xff);
      outb_p(bData, COUNTERB_0_DATA); 

      /* MSB in the second byte of value */
      bData = (u8) ((value >> 8) & 0xff);
      outb_p(bData, COUNTERB_0_DATA); 
  }
}

/***************************************************************************
 *
 *  Load the residual sample count.  
 *
 *  Counter 0 is used to stop the acquisition when the desired number of samples 
 *  have been gathered.  It is gated on when a "residual" number of conversions
 *  remain.  Counter 0 will be enabled by use of the ARM bit.  Counter 0 is
 *  to be operated in Mode 0.
 *
 ***************************************************************************/

static void LoadResidualCount( u16 resCount )
{
  u8 bReg;

  #ifdef DEBUG
    printk("LoadResidualCount: resCount = %d\n", resCount);
  #endif  

  bReg = C0+MODE0+LSBFIRST;         /* 8254A Counter 0 -> mode 0 */
  outb_p(bReg, COUNTERA_CONTROL);

  /* Set word of total count */
  bReg = (u8) (resCount & 0xff);
  outb_p(bReg, COUNTERA_0_DATA);         
  bReg = (u8) (resCount >> 8);
  outb_p(bReg, COUNTERA_0_DATA);
  
  return;
}

/***************************************************************************
 *
 * Set which channels read() is interested in.
 *
 ***************************************************************************/

static int SetADCChannelMux( u8 lowChan, u8 highChan )
{
  u8 channelMask;

  #ifdef DEBUG
    printk("SetADCChannelMux: lowChan = %d      highChan = %d\n", lowChan, highChan);
  #endif

  channelMask = (highChan << 4) | lowChan;
  BoardData.nSpare1 &= ~(0xff);
  BoardData.nSpare1 |= channelMask;
  return 0;
}

/***************************************************************************
 *
 * Set ADC gain
 *
 ***************************************************************************/

static int SetADCGain( u16 gain )
{
  #ifdef DEBUG
    printk("SetADCGain: gain = %#x\n", gain);
  #endif

  gain &= ~SEDIFF;  /* mask out measurement configuration */

 /* if the gain is different from the one already loaded,
    then calibration coeffs have to be loaded */

  if ( gain != (BoardData.nSpare1 & (UNIBIP | GS1 | GS0))) {
    calibrate_pci_das1602_ain(gain);
    BoardData.nSpare1 &= ~(UNIBIP | GS1 | GS0);
    BoardData.nSpare1 |= gain;
  }
  return 0;
}

/***************************************************************************
 *
 * Set DAC gain
 *
 ***************************************************************************/

static u16 SetDACGain( int channel, u16 gain )
{
  u16 wReg = 0x0;

  #ifdef DEBUG
    printk("SetDACGain: channel = %d   gain = %#x\n", channel, gain);
  #endif

  if ( channel ) {
    /* select channel 1 */
    wReg |= HS1;
    switch( gain ) {
      case BP_5_00V:
        break;
      case BP_10_00V:
        wReg |=  DAC1R0;
        break;
      case UP_5_00V:
        wReg |=  DAC1R1;
        break;
      case UP_10_00V:
        wReg |=  (DAC1R0 | DAC1R1);
        break;
      default:
        printk("SetDACGain: unknown gain %d.\n", gain);
        return -1;
    }
  } else {
    /* select channel 0 */
    wReg |= HS0;
    switch( gain ) {
      case BP_5_00V:
        break;
      case BP_10_00V:
        wReg |=  DAC0R0;
        break;
      case UP_5_00V:
        wReg |=  DAC0R1;
        break;
      case UP_10_00V:
        wReg |= (DAC0R0 | DAC0R1);
        break;
      default:
        printk("SetDACGain: unknown gain %d.\n", gain);
        return -1;
    }
  }

  return wReg;
}

/***************************************************************************
 *
 * Turn on ADC pacer timer chip.
 *
 ***************************************************************************/

static void StartADCPacer(void)
{
  BoardData.nSpare1 &= ~ADPS1;
  BoardData.nSpare1 |=  ADPS0;
  outw(BoardData.nSpare1, MUX_REG);
  udelay(10);              /* wait 10 usecs for things to settle down */
  outw(BoardData.nSpare2 & ~TS0, TRIG_REG);
  outw(BoardData.nSpare2 | TS0 | XTRCL, TRIG_REG);

  #ifdef DEBUG
      printk("StartADCPacer: MUX_REG = %#x\n", BoardData.nSpare1);
      printk("StartADCPacer: TRIG_REG = %#x\n", BoardData.nSpare2);
  #endif
}

/***************************************************************************
 *
 * Turn off ADC pacer timer chip.
 *
 ***************************************************************************/

static void StopADCPacer(void)
{
  u8 mask;

  BoardData.nSpare1 &= ~(ADPS1 | ADPS0);
  outw(BoardData.nSpare1, MUX_REG);

  mask = C2+MODE2+LSBFIRST;
  outb_p(mask, COUNTERA_CONTROL);
  mask = C1+MODE2+LSBFIRST;
  outb(mask, COUNTERA_CONTROL);
}

/***************************************************************************
 *
 * Turn off DAC pacer timer chip.
 *
 ***************************************************************************/

static void StopDACPacer(void)
{
  u8 mask;

  /*
  BoardData.nSpare3 &= ~(DAPS1 | DAPS0);
  outw(BoardData.nSpare3, DAC_REG);
  */

  mask = C2+MODE2+LSBFIRST;
  outb_p(mask, COUNTERB_CONTROL);
  mask = C1+MODE2+LSBFIRST;
  outb(mask, COUNTERB_CONTROL);
}

/***************************************************************************
 *
 * Handles 
 *    o pacer/counter triggered read() with software start.
 *
 * Pacer Method: 
 *
 *    o Pacer counter controls frequency of sample conversions
 *    o Total counter controls number of samples taken
 *    o Interrupts occur every 512(PACKETSIZE) samples and at last sample
 *    o Get chan->count samples
 *    o Samples are saved in kernel buffer(kernBuff) in ISR
 *
 ***************************************************************************/

static int PacerRead(ADC_ChanRec *chanRec)
{
  u16 residualCnt, sampleCnt;
  unsigned long flags;

  #ifdef DEBUG
      printk("Entering PacerRead().\n");
  #endif

  /* Prepare global data for parameterless interrupt handler */

  ADC_CurrChan = chanRec;                        /* pass chanel number to global */
  ADC_KernBuffPtr = ADC_KernBuff;                /* same with ADC_KernBuff       */

  /* Disable everything first */
  /* outw(0x0, IRQ_REG); */
  outw(0x0, TRIG_REG);

  /* Clear the AD FIFO */
  outw(0x1, ADC_FIFO_CLR);

  if ( chanRec->pretrigCount ) {
    PreTrigMode = TRUE;
    if ( (chanRec->pretrigCount%PACKETSIZE) == 0 ) {
      PreTrigBufSize = chanRec->pretrigCount;
    } else {
      PreTrigBufSize = ((chanRec->pretrigCount / PACKETSIZE) + 1)*PACKETSIZE;
    }
    PreTrigBufCnt = 0;
    PreTrigIndexCounter = 0;
    WordsToRead = chanRec->count - chanRec->pretrigCount;
  } else {
    WordsToRead = chanRec->count;
  }

  /* Load the residual count 

     We can get into a race condition here.  After an interrupt at FIFO
     half full, we need to make sure we have enough time to read the buffer
     (PACKETSIZE) before the next interrupt.  I am assuming a time of 640 us.
     If there is not enough time, it is better to sample another group and
     only read the number we need.
  */

  residualCnt = chanRec->count % PACKETSIZE;
  sampleCnt = BoardData.ADC_freq / 1562;
  if ( sampleCnt > PACKETSIZE ) sampleCnt = PACKETSIZE;

  if ( sampleCnt > residualCnt ) {
    LoadResidualCount(sampleCnt);      
  } else {
    LoadResidualCount(residualCnt);      
  }

  /* Set up the ADC Mux and Control register */
  SetADCChannelMux(chanRec->lowChan, chanRec->hiChan);  /* Read only this channel */
  SetADCGain(chanRec->gain);

  /* Load the Trigger Control Register */
  BoardData.nSpare2 = chanRec->nSpare2;

  /* Load the Trigger Control/Status Register */
  if ( chanRec->count < 2*PACKETSIZE && chanRec->count >= PACKETSIZE) {
    BoardData.nSpare2 |= ARM;
  } else if ( chanRec->count < PACKETSIZE ) {
    BoardData.nSpare2 |= ARM | FFM0;
  }

  if ( (chanRec->nSpare2 & (TS1 | TS0)) == 0 ) {  /* No trigger enabled */
    BoardData.nSpare2 |= (TS0 | XTRCL);   /* Enable SW Trigger and clear XTRIG */
  } else {
    BoardData.nSpare2 |=  XTRCL;          /*  clear XTRIG */
  }

  if ( chanRec->pretrigCount ) {
      BoardData.nSpare2 |= PRTRG;
  } else {
      BoardData.nSpare2 &= ~PRTRG;
  }

  /* Enable interrupts */
  save_flags(flags);
  cli();
  BoardData.nSpare0 |= (INTE | INT1 | EOAIE);
  BoardData.nSpare0 &= ~(INT0);
  restore_flags(flags);
  outw(BoardData.nSpare0 | ADFLCL | INTCL | EOACL, IRQ_REG);

  if ( chanRec->pacerSource == ADC_PACER_CLOCK ) {
    LoadADCPacer(&BoardData);                   /* Establish sample frequency */
  }
  outw(BoardData.nSpare1, MUX_REG);
  udelay(10);    /* wait 10 usecs for things to settle down */

  #ifdef DEBUG
    printk("PacerRead: Enter interrupt.  nSpare0 = %#x.\n", BoardData.nSpare0);
    printk("PacerRead: Enter interrupt.  nSpare1 = %#x.\n", BoardData.nSpare1);
    printk("PacerRead: Enter interrupt.  nSpare2 = %#x.\n", BoardData.nSpare2);
  #endif

  outw(BoardData.nSpare2, TRIG_REG);        /* let's go ... */
  interruptible_sleep_on(&das1602_16_wait);         /* Block in wait state */

  if ( WordsToRead != 0 ) {
      printk("Timing error in PacerRead: WordsToRead = %d\n", WordsToRead);
      return -1;
  }

  StopADCPacer();
  #ifdef DEBUG
    printk("Leaving PacerRead.\n");
  #endif
  return 0;
}

static void SoftWrite( int channel, int gain, u16 value )
{
  u16 wReg;

  /* Clear the DAC FIFO PTR4 BASE+2 */
  outw(0x0, DAC_FIFO_CLR);

  /* get current gain info */
  wReg = BoardData.nSpare3 & (DAC1R1 | DAC1R0 | DAC0R1 | DAC0R0);

  if ( channel ) {
    wReg &= ~(DAC1R1 | DAC1R0);
    wReg |= SetDACGain(channel, gain);
  } else {
    wReg &= ~(DAC0R1 | DAC0R0);
    wReg |= SetDACGain(channel, gain);
  }

  /* Set up SW Convert  PTR1 BASE+8 */
  wReg &= ~(DAPS1 | DAPS0);
  wReg |= (LDAEMCL | DACEN);

  #ifdef DEBUG
    printk("SoftWrite: channel = %d  DAC_REG = %#x, value = %#x\n", 
            channel, wReg, value);
  #endif

  outw(wReg, DAC_REG);

  /* 
     If the range has changed between the last output and the
     current one, then reload the calibration coefficients.
  */

  if ( channel ) {  /* channel 1 */
    if ( (wReg & (DAC1R1 | DAC1R0)) != (BoardData.nSpare3 & (DAC1R1 | DAC1R0))) {
      calibrate_pci_das1602_aout( channel, gain );
      BoardData.nSpare3 &= ~(DAC1R1 | DAC1R0);                  /* clear the bits */
      BoardData.nSpare3 |= (wReg & (DAC1R1 | DAC1R0));          /* set the bits   */
    }
  } else {    /* channel 0 */
    if ( (wReg & (DAC0R1 | DAC0R0)) != (BoardData.nSpare3 & (DAC0R1 | DAC0R0))) {
      calibrate_pci_das1602_aout( channel, gain );
      BoardData.nSpare3 &= ~(DAC0R1 | DAC0R0);                  /* clear the bits */
      BoardData.nSpare3 |= (wReg & (DAC0R1 | DAC0R0));          /* set the bits   */
    }
  }

  /* Output the value */
  outw(value, DAC_DATA_REG);
}

static void PacerWrite( int channel )
{
  u16 wReg;
  int gain = ChanDAC[channel].gain;
  int count = ChanDAC[channel].count;      /* Number of words to output             */
  unsigned long flags;

  WordsToWrite = count;
  DAC_KernBuffPtr = DAC_KernBuff;
  DAC_CurrChan = &ChanDAC[channel];

  #ifdef DEBUG
    printk("Entering PacerWrite().\n");
  #endif

  /* Disable everything first */
  save_flags(flags);    
  cli();
  BoardData.nSpare0 &= ~(DAHFIE | DAEMIE);  
  restore_flags(flags);
  outw(BoardData.nSpare0, IRQ_REG);


  /* Setup DAC Control/Status Register and mask with Range bits  DACnR[1:0] */
  wReg = BoardData.nSpare3 & (DAC1R1 | DAC1R0 | DAC0R1 | DAC0R0);
  wReg |= DACEN;                    /* else DAC set to ground */

  /* disable DAC interrupts/pacing without changing gain */
  outw(wReg, DAC_REG);

  /* Clear the DAC FIFO */
  outw(0x0, DAC_FIFO_CLR);

  if ( ChanDAC[channel].pacerSource == DAC_PACER_CLOCK ) {
    LoadDACPacer(&BoardData);
  }

  /* contiue with DAC Control Register */
  wReg |= START | LDAEMCL;

  /* update correct gains, and HS bits ... */

  if ( channel ) {
    wReg &= ~(DAC1R1 | DAC1R0);
    wReg |= SetDACGain(channel, gain);
  } else {
    wReg &= ~(DAC0R1 | DAC0R0);
    wReg |= SetDACGain(channel, gain);
  }

  /* 
     If conversions are not done by external clock, set up
     the pacer clock.
  */

  /* Set up DA pacer/mode/range */
  switch( ChanDAC[channel].pacerSource ) {
    case DAC_PACER_CLOCK:
      wReg |= DAPS0;
      break;
    case DAC_FALLING_EDGE:
      wReg |= DAPS1;
      break;
    case DAC_RISING_EDGE:
      wReg |= DAPS1;
      wReg |= DAPS0;
      break;
  }

  if ( count <= FIFO_SIZE ) {
    save_flags(flags);
    cli();
    BoardData.nSpare0 |= DAEMIE;
    restore_flags(flags);
    /* Load the DAC FIFO */
    outsw(DAC_DATA_REG, DAC_KernBuff, count ); 
    DAC_KernBuffPtr += count;
    WordsToWrite -= count;
  } else {
    save_flags(flags);    
    cli();
    BoardData.nSpare0 |= DAHFIE;
    restore_flags(flags);
    /* Load the DAC FIFO */
    outsw(DAC_DATA_REG, DAC_KernBuff, FIFO_SIZE ); 
    DAC_KernBuffPtr += FIFO_SIZE;
    WordsToWrite -= FIFO_SIZE;
  }

  /* 
     If the range has changed between the last output
     and the current one, then reload the calibration
     coefficients.
   */

  if ( channel ) {  /* channel 1 */
    if ( (wReg & (DAC1R1 | DAC1R0)) != (BoardData.nSpare3 & (DAC1R1 | DAC1R0))) {
      calibrate_pci_das1602_aout( channel, gain );
    }
  } else {    /* channel 0 */
    if ( (wReg & (DAC0R1 | DAC0R0)) != (BoardData.nSpare3 & (DAC0R1 | DAC0R0))) {
      calibrate_pci_das1602_aout( channel, gain );
    }
  }

  /* check to see if in simulaneous mode */
  if (  (BoardData.nSpare3 & (HS0 | HS1)) == (HS0 | HS1) ) {    
    wReg |= (HS0 | HS1);
  }

  BoardData.nSpare3 = wReg;

  /* clear pending interrupts if any */
  /* Enable interrupts. */
  outw(BoardData.nSpare0 | DAHFCL | DAEMCL, IRQ_REG); 

  /* set DAC_REG without START */
  outw(BoardData.nSpare3 & ~START, DAC_REG);
  
  /* delay */
  udelay(10);

  /* set DAC_REG with START */
  outw(BoardData.nSpare3, DAC_REG);
}

/*************************************************************************** 
 * Reads the calibration coeffs from non-volatile RAM in a PCI-DAS1602     *
 * and writes them to the appropriate trim-DACs associated with the analog * 
 * output sections.                                                        *
 * *************************************************************************/

void calibrate_pci_das1602_aout( int channel, int gain )
{
  u16 addr;
  u8 value;

  #ifdef DEBUG
    printk("calibrate_pci_das1602_aout: channel %d set to gain %#x.\n",
            channel, gain);
  #endif

  /*
     Get the starting offset in nvRAM at which the 
     calibration coeffs for the given range is stored
  */

  switch( gain ) {
    case BP_5_00V:
      addr = DAC_BIP5V_COEFF;
      break;
    case BP_10_00V:
      addr = DAC_BIP10V_COEFF;
      break;
    case UP_5_00V:
      addr = DAC_UNI5V_COEFF;
      break;
    case UP_10_00V:
      addr = DAC_UNI10V_COEFF;
      break;
    default:
      printk("pci1602_AOut: unknown gain %d.\n", gain);
      return;
  }

  if ( channel ) {
    /* DAC1 */
    addr += 0x10;  /* add 16 bytes if DAC1 */
    value = get_pci_das1602_nVRam( addr++ );
    write_pci_das1602_8800 ( DAC1_COARSE_GAIN, value );
    value = get_pci_das1602_nVRam( addr++ );
    write_pci_das1602_8800 ( DAC1_COARSE_OFFSET, value );
    value = get_pci_das1602_nVRam( addr++ );
    write_pci_das1602_8800 ( DAC1_FINE_GAIN, value );
    value = get_pci_das1602_nVRam( addr );
    write_pci_das1602_8800 ( DAC1_FINE_OFFSET, value );
  } else {
    /* DAC0 */
    value = get_pci_das1602_nVRam( addr++ );
    write_pci_das1602_8800 ( DAC0_COARSE_GAIN, value );
    value = get_pci_das1602_nVRam( addr++ );
    write_pci_das1602_8800 ( DAC0_COARSE_OFFSET, value );
    value = get_pci_das1602_nVRam( addr++ );
    write_pci_das1602_8800 ( DAC0_FINE_GAIN, value );
    value = get_pci_das1602_nVRam( addr );
    write_pci_das1602_8800 ( DAC0_FINE_OFFSET, value );
  }
}

/*************************************************************************** 
 * Reads the calibration coeffs from non-volatile RAM in a PCI-DAS1602     *
 * and writes them to the appropriate trim-DACs associated with the analog * 
 * input sections.                                                         *
 ***************************************************************************/

void calibrate_pci_das1602_ain( int gain )
{
  u16 addr;
  u8 value;

  /*
     Get the starting offset in nvRAM at which the 
     calibration coeffs for the given range is stored.
  */

  #ifdef DEBUG
      printk("calibrate_pci_das1602_ain: resetting calibration coefficients. gain = %d\n",
              gain);
  #endif

  switch( gain ) {
    case BP_10_00V:
      addr = ADC_BIP10V_COEFF;
      break;
    case BP_5_00V:
      addr = ADC_BIP5V_COEFF;
      break;
    case BP_2_50V:
      addr = ADC_BIP2PT5V_COEFF;
      break;
    case BP_1_25V:
      addr = ADC_BIP1PT25V_COEFF;
      break;
    case UP_10_00V:
      addr = ADC_UNI10V_COEFF;
      break;
    case UP_5_00V:
      addr = ADC_UNI5V_COEFF;
      break;
    case UP_2_50V:
      addr = ADC_UNI2PT5V_COEFF;
      break;
    case UP_1_25V:
      addr = ADC_UNI1PT25V_COEFF;
      break;
    default:
      printk("pci1602_ain: unknown gain %d.\n", gain);
      return;
  }

  value = get_pci_das1602_nVRam(addr++);  /* get front-end offset */
  write_pci_das1602_dac08(value);
  value = get_pci_das1602_nVRam(addr++);  /* get ADC offset */
  write_pci_das1602_8402(1, value);       /* pot #1 is for offset */
  value = get_pci_das1602_nVRam(addr);    /* get ADC gain */
  write_pci_das1602_8402(0, value);       /* pot #0 is for gain */

  udelay(1000);          /* waint 1 milliseconds when changing gains */
}
/*************************************************************************** 
 * Reads a byte from the specified address of the                          *
 * non-volatile RAM on a PCI-DAS1602.                                      *
 ***************************************************************************/

u8 get_pci_das1602_nVRam( u16 addr )
{
  /* See AMCC S5933 PCI Contrller Data Book, Spring 1996 */

  u8 lo_addr;
  u8 hi_addr;
  u8 value;

  lo_addr = addr & 0xff;                /* low byte of address */
  hi_addr = (addr >> 0x8) & 0xff;       /* high byte of address */

  /* make sure device is not busy */
  while (inb(NVRAM_CTRL_REG) & 0x80);   /* D7=0 (not busy) */
  
  outb(0x80, NVRAM_CTRL_REG);           /* CMD to load low byte of address */
  outb(lo_addr, NVRAM_DATA_REG);        /* load low byte */

  outb(0xa0, NVRAM_CTRL_REG);           /* CMD to load high byte of address */
  outb(hi_addr, NVRAM_DATA_REG);        /* load low byte */

  outb(0xe0, NVRAM_CTRL_REG);           /* CMD to read NVRAM data */
  
  while (inb(NVRAM_CTRL_REG) & 0x80);   /* D7=0 (not busy) */
  value = inb(NVRAM_DATA_REG);
 
  return (value);
}


void write_pci_das1602_8800( u8 addr, u8 value )
{
  int i;

  /* write 3 bits MSB first of address, keep SEL8800 low */
  for ( i = 0; i < 3; i++) {
    if ( addr & 0x4 ) {
        outw_p(SDI, CALIBRATE_REG );
    } else {
        outw_p(0x0, CALIBRATE_REG );
    }
    addr <<= 1;
  }

  /* write 8 bits MSB first of data, keep SEL8800 low */
  for ( i = 0; i < 8; i++) {
    if ( value & 0x80 ) {
        outw_p(SDI, CALIBRATE_REG );
    } else {
        outw_p(0x0, CALIBRATE_REG );
    }
    value <<= 1;
  }

  /* SEL8800 to set load clock */
  outw_p( SEL8800, CALIBRATE_REG );

  /* SEL8800 to reset load clock and update output */
  outw_p( 0x0, CALIBRATE_REG );
}


void write_pci_das1602_8402( u8 addr, u8 value )
{
  int i;

  /* Select the device */
  outw_p( SEL8402, CALIBRATE_REG );

  /* Select one of the two potentiometers:
     0 = ADC gain pot
     1 = ADC offset pot
  */
  if ( addr ) {
    outw_p( SDI| SEL8402, CALIBRATE_REG );
  } else {
    outw_p( SEL8402, CALIBRATE_REG );
  }

  /* Write out each bit of the byte starting with the msb */
  for ( i = 0; i < 8; i++) {
    if ( value & 0x80 ) {
        outw_p(SDI | SEL8402, CALIBRATE_REG );
    } else {
        outw_p(SEL8402, CALIBRATE_REG );
    }
    value <<= 1;
  }

  /* Deselect the device */
  outw_p(0x0, CALIBRATE_REG );
}

void write_pci_das1602_dac08( u8 value )
{
  /* Select front end offset value and load it */
  outw_p( SEL08, CALIBRATE_REG);
  outw_p( (u16) value , CALIBRATE_REG);
}

void SetTrigger( u32 value, int minor )
{
  switch(value) {
    case GATE_NEG_HYS:
      toggleHys();
      ChanADC[minor].nSpare2 = TS0 | TS1 | TGEN;
      SoftWrite( 0, BP_10_00V, ChanDAC[0].threshold );
      SoftWrite( 1, BP_10_00V, ChanDAC[1].threshold );
      break;
    case GATE_POS_HYS:
      toggleHys();
      ChanADC[minor].nSpare2 = TS0 | TS1 | HMODE | TGEN;
      SoftWrite( 0, BP_10_00V, ChanDAC[0].threshold );
      SoftWrite( 1, BP_10_00V, ChanDAC[1].threshold );
      break;
    case GATE_ABOVE:
      ChanADC[minor].nSpare2 = TS0 | TS1 | CHI_EN | TGEN;
      SoftWrite( 1, BP_10_00V, ChanDAC[1].threshold );
      break;
    case GATE_BELOW:
      ChanADC[minor].nSpare2 = TS0 | TS1 | TGPOL | CHI_EN | TGEN;
      SoftWrite( 1, BP_10_00V, ChanDAC[1].threshold );
      break;
    case TRIG_ABOVE:
      ChanADC[minor].nSpare2 = TS0 | TS1 | TGSEL | CHI_EN | TGEN;
      SoftWrite( 1, BP_10_00V, ChanDAC[1].threshold );
      break;
    case TRIG_BELOW:
      ChanADC[minor].nSpare2 = TS0 | TS1 | TGSEL | TGPOL | CHI_EN | TGEN;
      SoftWrite( 1, BP_10_00V, ChanDAC[1].threshold );
      break;
    case GATE_OUT_WINDOW:
      ChanADC[minor].nSpare2 = TS0 | TS1 | CHI_EN | CLO_EN | TGEN;
      SoftWrite( 0, BP_10_00V, ChanDAC[0].threshold );
      SoftWrite( 1, BP_10_00V, ChanDAC[1].threshold );
      break;
    case GATE_IN_WINDOW:
      ChanADC[minor].nSpare2 = TS0 | TS1 | CHI_EN | CLO_EN | TGPOL | TGEN;
      SoftWrite( 0, BP_10_00V, ChanDAC[0].threshold );
      SoftWrite( 1, BP_10_00V, ChanDAC[1].threshold );
      break;
    case GATE_HIGH:
      ChanADC[minor].nSpare2 =  TS1 | TGEN;
      break;
    case GATE_LOW:
      ChanADC[minor].nSpare2 =  TS1 | TGEN | TGPOL;
      break;
    case TRIG_POS_EDGE:
      ChanADC[minor].nSpare2 =  TS1 | TGEN | TGSEL;
      break;
    case TRIG_NEG_EDGE:
      ChanADC[minor].nSpare2 =  TS1 | TGEN | TGSEL | TGPOL;
      break;
    default:
      /* Trigger type has not been selected: default TRIG_POS_EDGE */
      ChanADC[minor].nSpare2 =  TS1 | TGEN | TGSEL;
      break;
  }

  /* if DACs were loaded in analog trigger mode, wait until they settle */
  if ( ChanADC[minor].nSpare2 & TS0 ){
    udelay(200);  /* delay 200 microseconds */
  }
}

