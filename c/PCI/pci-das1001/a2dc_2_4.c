/***************************************************************************
 Copyright (C) 2003  Warren J. Jasper
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
 *   Note: Interrupts, when enabled, are generated differently depending
 *   upon the mode of the board:
 *
 *   Soft Convert:   poll IRQDATA for A/D done.
 *   Pacer clock:    interrupts generated with FIFO half full
 *                   or when Total_Count = 0.
 *
 *   Documentation:
 *       PCI-DAS1000/1001/1002  User's Manual Revision 2, March 2002,
 *       Measurement Computing
 *       16 Commerce Boulevard
 *       Middleboro, MA  02346
 *       (508) 946-5100 www.measurementcomputing.com
 *
 *       Register Map for the PCI-DAS1000 Series
 *       Measurement Computing
 *       16 Commerce Boulevard
 *       Middleboro, MA  02346
 *       (508) 946-5100 www.measurementcomputing.com
 *
 *       Calibrating the PCI-DAS1001/1002 Rev. 3  August 27, 1998
 *       Internal ComputerBoards Memo
 *
 *       Reading and Writing to a serial NVRAM S5933, Applied Micro Circuits
 *       Corp. Design Note:  http://www.amcc.com/Products/PCI/S5933.htm
 *       Programming the S5933 NVRAM
 *
 *       PCI Products Data S5933, Applied Micro Circuits
 *       Corp.  http://www.amcc.com/Products/PCI/S5933.htm
 *       
 */

/***************************************************************************
 *
 * a2dc_2_4.c for PCI-DAS1001
 *
 ***************************************************************************/

#ifndef __KERNEL__
#define __KERNEL__
#endif
#ifndef MODULE
#define MODULE
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include "pci-das1001.h"
#include "a2dc.h"

#ifdef CONFIG_PCI
#include <linux/pci.h>
#endif


/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int  SoftRead(ADC_ChanRec *chan, BoardRec *boardRecPtr);
static int  PacerRead(ADC_ChanRec *chan, BoardRec *boardRecPtr, struct file *readfile);
static void SoftWrite(int channel, int gain, u16 value, BoardRec *boardRecPtr);
static int  SetADCPacerFreq(BoardRec *board);
static int  SetADCChannelMux(u8 lowChan, u8 highChan, BoardRec *boardRecPtr);
static int  SetADCGain(u16 gain, BoardRec *boardRecPtr);
static u16  SetDACGain(int channel, u16 gain, BoardRec *boardRecPtr);
static void StartADCPacer(BoardRec *boardRecPtr);
static void StopADCPacer(BoardRec *boardRecPtr);
static void LoadADCPacer(BoardRec *boardRecPtr);
static void LoadCounter0(u32 value, BoardRec *boardRecPtr);
static void LoadResidualCount(u16 resCount, BoardRec *boardRecPtr);
static void SetTrigger(u32 arg, ADC_ChanRec *chanRecPtr );

static void pci_das1001_AD_HalfFullInt(BoardRec *boardRecPtr);
static void pci_das1001_AD_PretrigInt(BoardRec *boardRecPtr);
static void pci_das1001_AD_NotEmptyInt(BoardRec *boardRecPtr);

static int __init das1001_init(void);
static void __exit das1001_exit (void);
static ssize_t das1001_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t das1001_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int das1001_open(struct inode *iNode, struct file *filePtr);
static int das1001_close(struct inode *iNode, struct file *filePtr);
static int das1001_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg);
static int das1001_mmap(struct file *filePtr, struct vm_area_struct * vma);
static unsigned int das1001_poll(struct file *filePtr, poll_table *wait);
static struct page *das1001_nopage(struct vm_area_struct *vma, unsigned long address, int write_access);
static void das1001_Interrupt(int irq, void *dev_id, struct pt_regs *regs);

static void calibrate_pci_das1001_aout( int channel, int gain, BoardRec *boardRecPtr);
static void calibrate_pci_das1001_ain( int gain, BoardRec *boardRecPtr);
static u8 get_pci_das1001_nVRam( u16 addr, u32 base0 );
static void write_pci_das1001_8800( u8 addr, u8 value, u32 base1 );
static void write_pci_das1001_7376( u8 value, u32 base1 );

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-DAS1001 module");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

module_init(das1001_init);
module_exit(das1001_exit);


/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/
static spinlock_t das1001_lock;
static int MajorNumber = DEFAULT_MAJOR_DEV;          /* Major number compiled in    */
static BoardRec BoardData[MAX_BOARDS];               /* Board specific information  */
static ADC_ChanRec ChanADC[MAX_BOARDS][AD_CHANNELS]; /* ADC Channel specific info   */
static DAC_ChanRec ChanDAC[MAX_BOARDS][DA_CHANNELS]; /* DAC Channel specific info   */
static DIO_ChanRec ChanDIO[MAX_BOARDS][DIO_PORTS];   /* DIO Channel specific info   */
static DECLARE_WAIT_QUEUE_HEAD(das1001_wait);
static int NumBoards = 0;                            /* number of boards found      */

static struct file_operations das1001_fops = {
  owner:    THIS_MODULE,
  read:     das1001_read,
  write:    das1001_write,
  ioctl:    das1001_ioctl,
  mmap:     das1001_mmap,
  open:     das1001_open,
  release:  das1001_close,
  poll:     das1001_poll,
};

static struct vm_operations_struct das1001_vops = {
  nopage: das1001_nopage
};


  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int das1001_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void das1001_remove_one(struct pci_dev *pdev);

static struct pci_device_id das1001_id_tbl[] __devinitdata = {
  {
  vendor:                  PCI_VENDOR_ID_CBOARDS,
  device:                  PCI_DEVICE_ID_CBOARDS_DAS1001,
  subvendor:               PCI_ANY_ID,
  subdevice:               PCI_ANY_ID,
  class:                   0,
  class_mask:              0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, das1001_id_tbl);

static struct pci_driver das1001_driver = {
  name:         "pci-das1001",
  id_table:     das1001_id_tbl,
  probe:        das1001_init_one,
  remove:       das1001_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/
static int __init das1001_init(void) 
{
  int err;

  /* Register as a device with kernel.  */

  if ((err = register_chrdev(MajorNumber, "pci-das1001", &das1001_fops))) {
    printk(KERN_WARNING"%s: Failure to load module. Major Number = %d error %d\n",
	   ADAPTER_ID, MajorNumber, -err);
    return err; 
  }

  if ((err = pci_module_init(&das1001_driver))) return err;
  return  err;
}

static int __devinit das1001_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  u32 base0;
  u32 base1;
  u32 base2;
  u32 base3;
  u32 base4;
  u16 wReg;
  struct page *page;
  
  if (NumBoards >= MAX_BOARDS) {
    printk(KERN_WARNING"das1001_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit a2dc.h.\n",
	   NumBoards);
    goto err_out_0;
  }

  BoardData[NumBoards].pdev = pdev;

  /* Get PCI_BASE_ADDRESS_0 */
  base0 = BoardData[NumBoards].base0 = pci_resource_start(pdev, 0);

  /* Get PCI_BASE_ADDRESS_1 */
  base1 = BoardData[NumBoards].base1 = pci_resource_start(pdev, 1);

  /* Get PCI_BASE_ADDRESS_2 */
  base2 = BoardData[NumBoards].base2 = pci_resource_start(pdev, 2);
      
  /* Get PCI_BASE_ADDRESS_3 */
  base3 = BoardData[NumBoards].base3 = pci_resource_start(pdev, 3);

  /* Get PCI_BASE_ADDRESS_4 */
  base4 = BoardData[NumBoards].base4 = pci_resource_start(pdev, 4);

  /* Register interrupt handler. */
  BoardData[NumBoards].irq = pdev->irq;
  if (request_irq(pdev->irq, das1001_Interrupt, (SA_INTERRUPT | SA_SHIRQ), 
      "pci-das1001", (void *) &BoardData[NumBoards])) {
    /* No free irq found! cleanup and exit */
    printk(KERN_WARNING"%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData[NumBoards].irq);
    goto err_out_1;
  }

  /* Set all channel structures to show nothing active/open */
  for (i = 0; i < AD_CHANNELS; i++) {
    ChanADC[NumBoards][i].open = FALSE;
    ChanADC[NumBoards][i].nonBlockFlag = FALSE;
    ChanADC[NumBoards][i].lowChan = i;
    ChanADC[NumBoards][i].hiChan = i;
    ChanADC[NumBoards][i].gain = UP_10_00V;   /* set range to 0-10V */
    ChanADC[NumBoards][i].nSpare2 = 0;        /* Differential Mode  */
    ChanADC[NumBoards][i].pretrigCount = 0;
  }

  for ( i = 0; i < DA_CHANNELS; i++ ) {
      ChanDAC[NumBoards][i].open = FALSE;
      ChanDAC[NumBoards][i].gain = UP_10_00V;   /* set range to 0-10V */
  }

  ChanDIO[NumBoards][0].addr = DIO_PORTA;
  ChanDIO[NumBoards][1].addr = DIO_PORTB;
  ChanDIO[NumBoards][2].addr = DIO_PORTC;
  BoardData[NumBoards].dio_reg = 0x9b;           /* set Mode 0 all Input */

  for ( i = 0; i < DIO_PORTS; i++ ) {
    ChanDIO[NumBoards][i].open = FALSE; 
    ChanDIO[NumBoards][i].mode = 0;
    outb(0x0, ChanDIO[NumBoards][i].addr);
  }

  init_waitqueue_head(&das1001_wait);
  BoardData[NumBoards].nonBlockFile = NULL;

  /* Reset FIFO interrupts and disable all interupts */
  wReg = (ADFLCL | INTCL | EOACL );
  BoardData[NumBoards].nSpare0 = 0x0;
  outw(wReg, IRQ_REG);

  /* Clear TRIG_REG register */
  outw(0x0, TRIG_REG);
  BoardData[NumBoards].nSpare2 = 0x0;

  /* Select unipolar 10V range for both DACS
     and zero them out by calling
     pci1001_aout.  Set BoardData.nSpare3 to zero to
     force calibration.
  */

  BoardData[NumBoards].nSpare3 = 0x0;
  SoftWrite( 0, UP_10_00V, 0x0, &BoardData[NumBoards] );
  SoftWrite( 1, UP_10_00V, 0x0, &BoardData[NumBoards] );

  /* Load the ADC calibration coeffs for unipolar 10V */
  calibrate_pci_das1001_ain( UP_10_00V, &BoardData[NumBoards] );

  BoardData[NumBoards].ADC_freq = DEFAULT_FREQ;   /* Set default pacer clock frequency */
  SetADCPacerFreq(&BoardData[NumBoards]);

  /* Reset Interrupt on AMCC5933 PCI controller */
  outl(INTCSR_DWORD, INTCSR_ADDR);
  outl(BMCSR_DWORD, BMCSR_ADDR);

  BoardData[NumBoards].busyRead = FALSE;          /* board ready */

  init_waitqueue_head(&das1001_wait);
  spin_lock_init(&das1001_lock);
  BoardData[NumBoards].buf_phy_size = ALIGN_ADDRESS(ADC_BUFF_PHY_SIZE, PAGE_SIZE);
  BoardData[NumBoards].buf_virt_addr = (u16 *) pci_alloc_consistent(0,
					    BoardData[NumBoards].buf_phy_size,
					    &BoardData[NumBoards].buf_bus_addr);
  if (BoardData[NumBoards].buf_virt_addr == 0x0) return -ENOMEM;

  /* set PG_reserved flag on DMA memory pages. 
     This protects them from the VM system after they're mmap()'d  
  */
  page = virt_to_page(BoardData[NumBoards].buf_virt_addr);
  for (i = 0; i < BoardData[NumBoards].buf_phy_size/PAGE_SIZE; i++) {
    SetPageReserved(&page[i]);
  }

  #ifdef DEBUG
    printk("das1001_init_one: Board %d: buf_phy_size = %#x\n",
	   NumBoards, BoardData[NumBoards].buf_phy_size);
    printk("das1001_init_one: Board %d: buf_virt_addr = %p  buf_bus_addr = %#x\n",
	   NumBoards, BoardData[NumBoards].buf_virt_addr, BoardData[NumBoards].buf_bus_addr);
  #endif
  
  printk("%s: BADR0=%#x BADR1=%#x BADR2=%#x BADR3=%#x BADR4=%#x\n",
          ADAPTER_ID, base0, base1, base2, base3, base4);
  printk("%s: IRQ=%d 9/24/2003 wjasper@ncsu.edu\n", ADAPTER_ID, BoardData[NumBoards].irq);

  NumBoards++;
  return 0;

err_out_1:
  free_irq(pdev->irq, (void *) &BoardData[NumBoards]);

err_out_0:
  return -ENODEV;
}

/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-das1001" is run on the command line.
 *
 ***************************************************************************/

void __exit das1001_exit(void) 
{
  pci_unregister_driver(&das1001_driver);

  if (unregister_chrdev(MajorNumber, "pci-das1001") != 0) {
    printk(KERN_WARNING"%s: cleanup module failed.\n", ADAPTER_ID);
  } else {
    printk("%s: module removed.\n", ADAPTER_ID);
  }
}

static void das1001_remove_one(struct pci_dev *pdev)
{
  struct page *page;
  int i;  

  if (MOD_IN_USE) {
    printk(KERN_WARNING"%s: device busy, remove delayed.\n", ADAPTER_ID);
    return;
  }

  NumBoards--;
  StopADCPacer(&BoardData[NumBoards]);
  
  free_irq(BoardData[NumBoards].irq, (void *) &BoardData[NumBoards]);

  /* clear reserved flag on DMA pages */
  page = virt_to_page(BoardData[NumBoards].buf_virt_addr);
  for (i = 0; i < BoardData[NumBoards].buf_phy_size/PAGE_SIZE; i++) {
    ClearPageReserved(&page[i]);
  }

  pci_free_consistent(0,
		      BoardData[NumBoards].buf_phy_size,
		      BoardData[NumBoards].buf_virt_addr,
		      BoardData[NumBoards].buf_bus_addr);

  #ifdef DEBUG
      printk("%s: module removed.\n", ADAPTER_ID);
  #endif
  
}     /* das1001_remove_one() */

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int das1001_open(struct inode *iNode, struct file *filePtr)
{
  u32 base1;

  unsigned long flags;
  u16 wReg;

  int port  = 0;
  int board = 0;
  int minor = MINOR(iNode->i_rdev);

  board = BOARD(minor);    /* get which board   */
  minor &= 0x1f;          /* get which channel */

  base1 = BoardData[board].base1;

  /* 
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes 
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */

  if ( minor >= 0 && minor < AD_CHANNELS ) {
    if ( ChanADC[board][minor].open == TRUE ) {
      return -EBUSY;
    }

    MOD_INC_USE_COUNT;
    ChanADC[board][minor].open = TRUE;                           /* The device is open          */
    ChanADC[board][minor].gain = BP_10_00V;                      /* +/- 10V , Differential Mode */
    ChanADC[board][minor].pacerSource = filePtr->f_flags & 0x7;  /* set acquisition mode        */
    ChanADC[board][minor].nSpare2 = 0x0;
    ChanADC[board][minor].lowChan = minor;
    ChanADC[board][minor].hiChan = minor;
    ChanADC[board][minor].pretrigCount = 0;                      /* turn pretrigger off         */

    /* Reset FIFO interrupts and disable all A/D interupts */
    wReg = (ADFLCL | INTCL | EOACL);
    save_flags(flags);
    cli();
    BoardData[board].nSpare0 &= ~(EOAIE | INTE);
    restore_flags(flags);
    outw(BoardData[board].nSpare0 | wReg, IRQ_REG);

    /* Clear TRIG_REG register */
    outw(0x0, TRIG_REG);
    BoardData[board].nSpare2 = 0x0;

    BoardData[board].nonBlockFile = NULL;

    #ifdef DEBUG
    printk("%s: open(): minor %d mode %d.\n",
	   ADAPTER_ID, minor, ChanADC[board][minor].pacerSource);
    #endif
    return 0;   
  }

  if ( (minor >= AD_CHANNELS) && (minor < AD_CHANNELS+DIO_PORTS) ) {
    port = minor - AD_CHANNELS;
    if ( ChanDIO[board][port].open == TRUE ) {
      return -EBUSY;
    }

    MOD_INC_USE_COUNT;
    ChanDIO[board][port].open = TRUE;                 /* The device is open */
    ChanDIO[board][port].f_flags = filePtr->f_flags;
   
    #ifdef DEBUG
      printk("%s: open(): minor %d mode %d.\n", ADAPTER_ID, minor, ChanDIO[board][port].mode);
    #endif
    return 0;
  }

  if ( (minor >= AD_CHANNELS+DIO_PORTS) && (minor < AD_CHANNELS+DIO_PORTS+DA_CHANNELS) ) {
    port = minor - AD_CHANNELS - DIO_PORTS;
    if ( ChanDAC[board][port].open == TRUE ) {
      return -EBUSY;
    }
  
    MOD_INC_USE_COUNT;
    ChanDAC[board][port].open = TRUE;                 /* The device is open */

    #ifdef DEBUG
    printk("%s: open(): minor %d .\n", ADAPTER_ID, minor);
    #endif
  }
  return 0;
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int das1001_close(struct inode *iNode, struct file *filePtr)
{
  u32 base2;
  int port  = 0;
  int board = 0;
  int minor = MINOR(iNode->i_rdev);

  board = BOARD(minor);    /* get which board   */
  minor &=  0x1f;          /* get which channel */
  base2 = BoardData[board].base2;

  if (filePtr && BoardData[board].nonBlockFile && filePtr == BoardData[board].nonBlockFile) {
  /* cancel pending non-blocking i/o */
  /* disable acquisition */
  #ifdef DEBUG
    printk("close()  clearing ADC FIFO and nonblock flag.\n");
  #endif
    StopADCPacer(&BoardData[board]);  // disable pacer if in pacer mode 
    
  /* Clear ADC FIFO Pointer */
    outw(0x1, ADC_FIFO_CLR);
    udelay(1000);
    BoardData[board].nonBlockFile = NULL; // clear non-block flag
  }
 
  MOD_DEC_USE_COUNT;

  if ( minor >= 0 && minor < AD_CHANNELS ) {
    /* ADC */
    ChanADC[board][minor].open = FALSE;
    ChanADC[board][minor].nonBlockFlag = FALSE;
  } else if ( minor >= AD_CHANNELS && minor < AD_CHANNELS + DIO_PORTS ) {
    /* DIO */
    port = minor - AD_CHANNELS;
    ChanDIO[board][port].open = FALSE;
  } else if ( minor >= AD_CHANNELS+DIO_PORTS && minor < AD_CHANNELS+DIO_PORTS+DA_CHANNELS ) {
    /* DAC */
    port = minor - AD_CHANNELS - DIO_PORTS;
    ChanDAC[board][port].gain = UP_10_00V;     /* set range to 0-10V */
    ChanDAC[board][port].open = FALSE;
    SoftWrite(port, UP_10_00V, 0x0, &BoardData[board]);
  } else {
    printk("das1001_close: Incorrect minor number (%d).\n", minor);
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

static ssize_t das1001_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  u32 base1;

  u8  bReg;
  int minor;
  int chan;
  int board;
  int port;
  int start, stop;
  register int i;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = MINOR(iNode->i_rdev);
  board = BOARD(minor);            /* get which board   */
  chan =  (minor & 0x1f);          /* get which channel */
  base1 = BoardData[board].base1;

  /* Initialize the pointer into the data buffer array */
  BoardData[board].ADC_KernBuffPtr = BoardData[board].buf_virt_addr;
 
  ChanADC[board][chan].nonBlockFlag = (filePtr->f_flags & O_NONBLOCK);
  
  if ( count < 1 ) {
    printk("das1001_read(): count must be greater than 0.\n");
    BoardData[board].busyRead = FALSE;
    return (-1);
  }

  if ( chan >= 0 && chan < AD_CHANNELS ) {
    if (BoardData[board].nonBlockFile && BoardData[board].nonBlockFile != filePtr) {
      /* somebody else has a non-blocking request going */
      return -EBUSY;
    } else if (BoardData[board].busyRead == TRUE  &&
	       !ChanADC[board][chan].nonBlockFlag) { /* if board is in use, return busy */
      return (-EBUSY);
    } else {
      BoardData[board].busyRead = TRUE;
    }

    /* Set up configuration and load calibration coefficeints */

    if ( (ChanADC[board][chan].count = count) > BoardData[board].buf_phy_size ) {
        printk("das1001_read(): requesting too large a count size.\n");
        BoardData[board].busyRead = FALSE;
        return (-1);
    }

    /* Read */
    switch (ChanADC[board][chan].pacerSource) {

      case ADC_SOFT_CONVERT:
          #ifdef DEBUG
              printk("das1001_read(): Entering ADC_SOFT_CONVERT mode.  count = %d\n",
                      count);
          #endif

          if (SoftRead(&ChanADC[board][chan], &BoardData[board])) {
              printk("das1001_read: SoftRead() failed.\n");
              BoardData[board].busyRead = FALSE;
              return(-1);
          } 
          break;

      case ADC_EXTERNAL_PACER_FALLING:
        #ifdef DEBUG
            printk("das1001_read(): Entering ADC_EXTERNAL_PACER_FALLING mode.\n");
        #endif

        /* PreTrigger */
         if ( ChanADC[board][chan].pretrigCount ) {
           if ( count < ChanADC[board][chan].pretrigCount + PACKETSIZE ) {
             printk("das1001_read(): TotalCount (%d) must be greater than PretrigCount (%d) + %d.\n",
                     count, ChanADC[board][chan].pretrigCount, PACKETSIZE);
             return (-1);
	   }
	 }

         BoardData[board].nSpare1 |= ADPS1;           /* Select Pacer Source */
         BoardData[board].nSpare1 &= ~ADPS0;
         if (PacerRead(&ChanADC[board][chan], &BoardData[board], filePtr)) {
            printk("das1001_read: PacerRead() failed.\n");
            BoardData[board].busyRead = FALSE;
            return(-1);
	 }
         break;

      case ADC_EXTERNAL_PACER_RISING:
        #ifdef DEBUG
            printk("das1001_read(): Entering ADC_EXTERNAL_PACER_RISING mode.\n");
        #endif

        /* PreTrigger */
         if ( ChanADC[board][chan].pretrigCount ) {
           if ( count < ChanADC[board][chan].pretrigCount + PACKETSIZE ) {
             printk("das1001_read(): TotalCount (%d) must be greater than PretrigCount (%d) + %d.\n",
                     count, ChanADC[board][chan].pretrigCount, PACKETSIZE);
             return (-1);
	   }
	 }

         BoardData[board].nSpare1 |= ADPS1;           /* Select Pacer Source */
         BoardData[board].nSpare1 |= ADPS0;
         if (PacerRead(&ChanADC[board][chan], &BoardData[board], filePtr)) {
            printk("das1001_read: PacerRead() failed.\n");
            BoardData[board].busyRead = FALSE;
            return(-1);
	 }
         break;

      case ADC_PACER_CLOCK:
          #ifdef DEBUG
              printk("das1001_read(): Entering ADC_PACER_CLOCK mode.\n");
          #endif

          if ( ChanADC[board][chan].count == 1 ) {            /* use SoftRead */
              if (SoftRead(&ChanADC[board][chan], &BoardData[board])) {
                  printk("das1001_read: SoftRead() failed with pacer.\n");
                  BoardData[board].busyRead = FALSE;
                  return(-1);
              }
          } else {
              StopADCPacer(&BoardData[board]);        /* disable pacer if in pacer mode */
              BoardData[board].nSpare1 &= ~ADPS1;     /* Select Pacer Source */
              BoardData[board].nSpare1 |=  ADPS0;
              switch (PacerRead(&ChanADC[board][chan], &BoardData[board], filePtr)) {
  	        case 0:
                  break;  /* success */
                case -EAGAIN:
		  return -EAGAIN; /* non-blocking, and not complete */
		  break;
	        default:
                  printk("das1001_read: PacerRead() failed.\n");
                  BoardData[board].busyRead = FALSE;
                  return(-1);
              } 
          }
          break;
    }                  /* end switch */

    /* Check for overflows */


    /* Write data to user space */
    if (ChanADC[board][chan].count == 1) {
      put_user(*BoardData[board].buf_virt_addr, (u16*) buf);
    } else  {
      if ( ChanADC[board][chan].pretrigCount &&
	   (ChanADC[board][chan].pretrigCount > BoardData[board].PreTrigIndexCounter) ) {
        /* Some pretrig values in the circular buffer.  Reorder and write to buf */
        stop = BoardData[board].PreTrigBufCnt;          /* last offset in circular buffer */
        if ( stop == 0 ) stop += BoardData[board].PreTrigBufSize;
         start =  stop - (ChanADC[board][chan].pretrigCount - BoardData[board].PreTrigIndexCounter);
         start += BoardData[board].PreTrigBufSize;           /* make sure offset is positive number */
         start %= BoardData[board].PreTrigBufSize;           /* and located in the ring buffer */
         if ( stop > start ) {
           i = (stop - start)*sizeof(u16);
	   if (copy_to_user(buf, &BoardData[board].buf_virt_addr[start], i) ) return -EFAULT;
           buf += i;
	 } else {
           i = (BoardData[board].PreTrigBufSize - start)*sizeof(u16);
	   if ( copy_to_user(buf, &BoardData[board].buf_virt_addr[start], i) ) return -EFAULT;
           buf += i;
           i = stop*sizeof(u16);
	   if ( copy_to_user(buf, BoardData[board].buf_virt_addr, i) ) return -EFAULT;
           buf += i;
	 }
         i = ChanADC[board][chan].count - ChanADC[board][chan].pretrigCount
	                                + BoardData[board].PreTrigIndexCounter;
	 if (copy_to_user(buf, (BoardData[board].ADC_KernBuffPtr + BoardData[board].PreTrigBufSize),
			   i*sizeof(u16)) ) return -EFAULT;
      } else {
        if (copy_to_user(buf, BoardData[board].buf_virt_addr,
			 ChanADC[board][chan].count*sizeof(u16)) ) {
	  return -EFAULT;
	}
      }
    }

    BoardData[board].busyRead = FALSE;
    return(ChanADC[board][chan].count);     /* return number of samples read */
  }

  /* check to see if reading a value from the DIO */

  if ( chan >= AD_CHANNELS && chan < AD_CHANNELS + DIO_PORTS ) {
    port = chan - AD_CHANNELS;
    bReg = inb(ChanDIO[board][port].addr);
    put_user(bReg, (u8*) buf);

    #ifdef DEBUG
       printk("das1001_read: %s DIO Port %#x addr %#x set to %#x\n", 
                             ADAPTER_ID, port, ChanDIO[board][port].addr, bReg);
    #endif
    BoardData[board].busyRead = FALSE;
    return 1;
  }

  printk("das1001_read: Incorrect minor number (%d).\n", minor);
  BoardData[board].busyRead = FALSE;
  return -1;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t das1001_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  u32 base1;
  u32 base4;

  int minor;
  int port;
  int board;
  int chan;
  u8 bReg;
  struct inode *iNode = filePtr->f_dentry->d_inode;

  minor = MINOR(iNode->i_rdev);
  board = BOARD(minor);            /* get which board   */
  chan =  (minor & 0x1f);          /* get which channel */

  base1 = BoardData[board].base1;
  base4 = BoardData[board].base4;

  #ifdef DEBUG
    printk("das1001_write(): Minor = %d, Count = %d\n", minor, count);
  #endif

  if ( count < 1 ) {
    printk("das1001_write(): count must be greater than 0.\n");
    return (-1);
  }

  /* check to see if writing a value to the DIO */
  if ( (chan >= AD_CHANNELS) && (chan < AD_CHANNELS+DIO_PORTS) ) {
    port = chan - AD_CHANNELS;
    get_user(bReg, (u8*)buf);
    ChanDIO[board][port].value = bReg;
    outb( bReg, ChanDIO[board][port].addr);

    #ifdef DEBUG
    printk("das1001_write %s: DIO Port %#x set to %#x\n", ADAPTER_ID, port, bReg);
    #endif

    return 1;
  }

  /* writing to the DAC */
  if ( (chan >= AD_CHANNELS+DIO_PORTS) && (chan < AD_CHANNELS+DIO_PORTS+DA_CHANNELS) ) {
    port = chan - AD_CHANNELS - DIO_PORTS;

    /* Read data from user space */
    get_user(ChanDAC[board][port].value, (u16*)buf);
    SoftWrite(port, ChanDAC[board][port].gain, ChanDAC[board][port].value, &BoardData[board]);

    #ifdef DEBUG
        printk("das1001_write():  port = %d, gain = %d\n  value = %#x",
                port, ChanDAC[board][port].gain, ChanDAC[board][port].value );
    #endif
    return count;
    return 1;

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

static int das1001_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  int minor = MINOR(iNode->i_rdev);
  int port;
  int channel;
  int err = 0;
  int size = _IOC_SIZE(cmd);       /* the size bitfield in cmd */
  int board;
  u32 base1;
  u32 base2;
  u32 base3;

  board = BOARD(minor);            /* get which board   */
  minor &= 0x1f;                   /* mask off board information */

  base1 = BoardData[board].base1;
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;

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
      put_user( (long) BoardData[board].buf_phy_size, (long*) arg );
      return 0;
      break;
  }

  if ( minor >= 0 && minor < AD_CHANNELS ) {
    switch (cmd) {
      case ADC_SET_GAINS:
        #ifdef DEBUG
          printk("ioctl ADC_SET_GAINS: Channel = %d, Gain = %ld\n", minor, arg);
        #endif
        ChanADC[board][minor].gain &= ~(UNIBIP | GS1 | GS0);
        ChanADC[board][minor].gain |=  (u16) arg & (UNIBIP | GS1 | GS0);
        break;
      case ADC_GET_GAINS:
	put_user( (long)ChanADC[board][minor].gain, (long*) arg);
        break;
      case ADC_SET_PACER_FREQ:
        if ( arg > MAX_AD_FREQ ) {
            printk("ioctl:  Can not set frequency %lu greater than %d.\n", arg, MAX_AD_FREQ );
            return -1;
        } else {
            BoardData[board].ADC_freq = (u32)arg;
            SetADCPacerFreq(&BoardData[board]);
            LoadADCPacer(&BoardData[board]);               /* load the board frequency now */
        }
        break;
      case ADC_GET_PACER_FREQ:
	put_user(BoardData[board].ADC_freq, (long*) arg);
        break;
      case ADC_START_PACER:
        StartADCPacer(&BoardData[board]);
        break;
      case ADC_STOP_PACER:
        StopADCPacer(&BoardData[board]);
        break;
      case ADC_COUNTER0:
        LoadCounter0( (u32) arg, &BoardData[board]);
        break;
      case ADC_SET_MUX_LOW:
        ChanADC[board][minor].lowChan = (u8) arg;
        break;
      case ADC_SET_MUX_HIGH:
        ChanADC[board][minor].hiChan = (u8) arg;
        break;
      case ADC_GET_CHAN_MUX_REG:
        put_user( BoardData[board].nSpare1, (long*) arg);
        break;
      case ADC_SET_FRONT_END:
        if ( arg ) {
          BoardData[board].nSpare1 |= SEDIFF;     /* Single Ended Mode */
        } else {
          BoardData[board].nSpare1 &= ~(SEDIFF);  /* Differential Mode */
	}
        break;
      case ADC_BURST_MODE:
        if (arg == 0) {
          ChanADC[board][minor].nSpare2 &= ~BURSTE;  /* Burst Mode disabled */
	} else {
          ChanADC[board][minor].nSpare2 |= BURSTE;   /* Burst Mode enabled */
	}
        break;
      case ADC_SET_TRIGGER:
        SetTrigger(arg, &ChanADC[board][minor]);
        break;
      case ADC_PRETRIG:
        /* sets up the borad for pre-trigger.  The PretrigCount value must
           be less than 32768 and also less than TotalCount - 512.
        */
        if ( arg >= 32768 ) {
          printk("ioctl:  Can not set PretrigCount greater than 32768.\n");
          return -EINVAL;
	} else {
          ChanADC[board][minor].pretrigCount = (u16) arg;
	}
        break;
      case ADC_NBIO_CANCEL:
        if (BoardData[board].nonBlockFile && (filePtr == BoardData[board].nonBlockFile || arg )) {
	  // disable acquisition
          StopADCPacer(&BoardData[board]);  // disable pacer if in pacer mode 
	  outw(0x1, ADC_FIFO_CLR);
	  udelay(1000);
	  BoardData[board].nonBlockFile = NULL;         /* clear non-block flag */
	}
	break;
      case ADC_NBIO_PENDING:
        if (BoardData[board].nonBlockFile && filePtr == BoardData[board].nonBlockFile) {
          put_user(1, (long *) arg);
        } else {
          put_user(0, (long *) arg);
        }
        break;
    case ADC_NBIO_COMPLETE:
        #ifdef DEBUG
          printk("ioctl: ADC_NBIO_COMPLETE: WordsToRead = %d\n", BoardData[board].WordsToRead);
        #endif
        if (BoardData[board].nonBlockFile && filePtr == BoardData[board].nonBlockFile &&
	    BoardData[board].WordsToRead == 0) {
          put_user(1, (long *) arg);
	} else {
          put_user(0, (long *) arg);
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
              BoardData[board].dio_reg &= 0x9f;
              BoardData[board].dio_reg |= (arg << 5);
              outb(BoardData[board].dio_reg, DIO_CNTRL_REG);
              ChanDIO[board][port].mode = arg;
              break;

            case 1:			/* Port 1B */
              #ifdef DEBUG
              printk("DIO_SET_MODE for Port 1B\n");
              #endif
              if (arg & 0x2)
	        return(-EINVAL);	       /* Port 1B only has Modes 0 & 1 */
              BoardData[board].dio_reg &=  0xfb;
              BoardData[board].dio_reg |= (arg << 2);
              outb(BoardData[board].dio_reg, DIO_CNTRL_REG);
              ChanDIO[board][port].mode = arg;
              break;

            case 2:			/* Port 1C */
              #ifdef DEBUG
              printk("DIO_SET_MODE for Port 1C\n");
              #endif
              if (arg)
	        return(-EINVAL);	/* Port 1C only has Mode 0 I/O */
              ChanDIO[board][port].mode = arg;
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
            BoardData[board].dio_reg &=  0xef;
            BoardData[board].dio_reg |= (arg << 4);
            outb(BoardData[board].dio_reg, DIO_CNTRL_REG);
            break;

          case 1:			/* Port B */
            #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port B\n");
            #endif
            arg &= 0x1;
            BoardData[board].dio_reg &=  0xfd;
            BoardData[board].dio_reg |= (arg << 1);
            outb(BoardData[board].dio_reg, DIO_CNTRL_REG);
            break;

          case 2:			/* Port C */
            #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port C\n");
            #endif
            switch ( arg ) {
              case PORT_INPUT:         /* CU = IN  CL = IN */
                  BoardData[board].dio_reg |= 0x9;
                  break;
              case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                  BoardData[board].dio_reg &= ~(0x9);
                  break;
              case LOW_PORT_INPUT:     /* CL = IN */
                  BoardData[board].dio_reg |= 0x1;
                  break;
              case LOW_PORT_OUTPUT:    /* CL = OUT */
                  BoardData[board].dio_reg &= ~(0x1);
                  break;
              case HIGH_PORT_INPUT:    /* CU = IN */
                  BoardData[board].dio_reg |= 0x8;
                  break;
              case HIGH_PORT_OUTPUT:   /* CU = OUT */
                  BoardData[board].dio_reg &= ~(0x8);
                  break;
              default:
                  return(-EINVAL);
                  break;
            }
            outb(BoardData[board].dio_reg, DIO_CNTRL_REG);
            break;

          default:
            #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Invalid Port\n");
            #endif
            return(-EINVAL);
            break;
        }
	
        #ifdef DEBUG
        printk("DIO_SET_DIRECTION: Control Reg = %#x\n", BoardData[board].dio_reg);
        #endif

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
	ChanDAC[board][channel].gain = (u16) arg;
        break;
      case DAC_GET_GAINS:
        put_user((long)ChanDAC[board][channel].gain, (long*) arg);
        break;
      case DAC_MODE:
	if ( arg == 0 ) {
	  BoardData[board].nSpare3 &= (MODE);
	} else {
	  BoardData[board].nSpare3 |= (MODE);
	}
        break;
    }
  }
  return 0;
}

/*****************************************************
 *                                                   *
 * mmap() service handler                            *
 *                                                   *
 *****************************************************/

static int das1001_mmap( struct file *filePtr, struct vm_area_struct *vma ) 
{
  int board = 0;
  int chan  = 0;
  int size  = vma->vm_end - vma->vm_start;
  unsigned int minor = MINOR(filePtr->f_dentry->d_inode->i_rdev);

  board = BOARD(minor);            /* get which board   */
  chan =  (minor & 0xf);           /* get which channel */

  if (chan >= 0 && chan < AD_CHANNELS) {
    /* Mmap requested for ADC. Note: It does not matter for which ADC, all ADC's share the buffer.*/

    #ifdef DEBUG
    printk("%s: das1001_mmap(): board %d chan %d.\n", ADAPTER_ID, board, chan);
    printk("das1001_mmap(): vma->vm_start = %#lx  vma->vm_end= %#lx size = %#x offset = %#lx\n",
	   vma->vm_start, vma->vm_end, size, vma->vm_pgoff);
    #endif
    
    if(vma->vm_pgoff != 0) {     // You have to map the entire buffer.
      #ifdef DEBUG
        printk("das1001_mmap(): The page offset has to be zero \n");
      #endif
      return(-EINVAL);
    }

    if (size > BoardData[board].buf_phy_size) {     // You cannot request more than the max buffer
      printk("das1001_mmap(): Size = %d is too large.\n", size);
      return(-EINVAL);
    }

    vma->vm_ops = &das1001_vops;
    vma->vm_flags |= VM_RESERVED;
    vma->vm_private_data = (void *) minor;
  }
  return 0;
}

/*****************************************************
 *                                                   *
 * nopage() service handler                          *
 *                                                   *
 *****************************************************/

static struct page *das1001_nopage(struct vm_area_struct *vma, unsigned long address, int write_access)
{
  struct page  *page = NOPAGE_SIGBUS;      // page to be mapped
  unsigned char *v_addr = NULL;            // kernel virtual address to be mapped to user space
  unsigned int minor = (unsigned int) vma->vm_private_data;
  int board = 0;
  int chan = 0;
  unsigned long offset;   // offset to test for valid address

  board = BOARD(minor);
  chan = (minor & 0xf);

  offset = (address - vma->vm_start) + vma->vm_pgoff * PAGE_SIZE;
  if (offset >=  BoardData[board].buf_phy_size) {
    printk("das1001_nopage: offset = %#lx  buf_phys_size = %#x\n", offset, BoardData[board].buf_phy_size);
    return NOPAGE_SIGBUS;
  }
  
  v_addr =  (u8*) BoardData[board].buf_virt_addr + (address - vma->vm_start);
  page = virt_to_page(v_addr);

  #ifdef DEBUG
    printk ("das1001_nopage mapped address %08lx to %p\n", address, v_addr);
  #endif
  return (page);
}

/***************************************************************************
 *
 * poll() service handler
 *
 ***************************************************************************/

static unsigned int das1001_poll(struct file *filePtr, poll_table *wait)
{
  unsigned int minor = MINOR(filePtr->f_dentry->d_inode->i_rdev);
  unsigned int mask = 0;
  int board = 0;

  board = BOARD(minor);            /* get which board   */
  
  /* tell select()/poll() to wake up and give us a call if
     das1001_wait is awakened
  */
  poll_wait(filePtr, &das1001_wait, wait);
 
  /* can we currently do a non-blocking read? */
  if (BoardData[board].nonBlockFile && BoardData[board].WordsToRead == 0) {
    mask |= POLLIN|POLLRDNORM;
  }
  return mask;
}

/***************************************************************************
 *
 * Interrupt handler used to service interrupt read().
 *
 ***************************************************************************/

static void das1001_Interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
  unsigned long flags;
  BoardRec *boardRecPtr = dev_id;
  u16 IntSource;
  u32 base0 = boardRecPtr->base0;
  u32 base1 = boardRecPtr->base1;

  spin_lock_irq(&das1001_lock);
 
  /* Check that interrupt came from the pci-das1001 board (needed for sharded interrupts) */
  if (inl(INTCSR_ADDR) & 0x00820000) {
    save_flags(flags);
    cli();

    /* Reset Interrupt on AMCC5933 PCI controller  */
    /* Right now, to avoid race condition with simultaneous A/D and D/A */
    outl_p(BMCSR_DWORD, BMCSR_ADDR);
    outl_p(INTCSR_DWORD, INTCSR_ADDR);

    IntSource = inw(IRQ_REG) & 0x6fe0;  /* mask out extraneous garbage */

    #ifdef DEBUG
        printk("Entering das1001_Interrupt(). irq = %d  WordsToRead = %d  IntSource = %#hx\n", 
              irq, boardRecPtr->WordsToRead, IntSource );
    #endif

    while (IntSource) {
      if (IntSource & EOAI) {         /* End of acquisition          */
        IntSource &= ~EOAI;
        pci_das1001_AD_HalfFullInt(boardRecPtr);
      } else if (IntSource & ADHFI) { /* AD-FIFO-half-full interrupt */
        IntSource &= ~ADHFI;
        if ( boardRecPtr->PreTrigMode ) { 
          pci_das1001_AD_PretrigInt(boardRecPtr);
        } else {
         pci_das1001_AD_HalfFullInt(boardRecPtr);
        }
      } else if (IntSource & ADNEI) { /* AD-not-empty                */
        IntSource &= ~ADNEI;
        pci_das1001_AD_NotEmptyInt(boardRecPtr);
      } else if (IntSource & EOBI) {  /* end-of-burst interrrupt     */
        IntSource &= ~EOBI;
        pci_das1001_AD_HalfFullInt(boardRecPtr);
      } else {
        break;
      }
    }

    if (boardRecPtr->WordsToRead == 0) { 
      boardRecPtr->nSpare0 &= ~(EOAIE | INTE);
      outw( boardRecPtr->nSpare0, IRQ_REG);
      wake_up_interruptible(&das1001_wait);  
    }
    restore_flags(flags);
  }
  spin_unlock_irq(&das1001_lock);
  return;
}

/***********************************************************************
 *  This routine gets called from das1001_Interrupt() when the         *
 *  interrupt was caused by the FIFO going half full.  It reads half   *
 *  a FIFO's worth of data and stores it in the kernel's arrary.       *
 ***********************************************************************/

static void pci_das1001_AD_HalfFullInt(BoardRec *boardRecPtr)
{
  u32 base1 = boardRecPtr->base1;
  u32 base2 = boardRecPtr->base2;

#ifdef DEBUG
      printk("Entering pci_das1001_AD_HalfFullInt()\n");
  #endif

  /* check for FIFO Overflow errors */
  if (inw(IRQ_REG) & LADFUL) {
    outw(0x1, ADC_FIFO_CLR);
    outw(boardRecPtr->nSpare0 | ADFLCL | INTCL, IRQ_REG);  /* Clear Interrupts */
    printk("pci_das1001_AD_HalfFullInt: LADFUL error\n");
    return;
  }

  /* Caclulate remaining count.  If it's less than PACKET_SIZE then use
     remaining count, otherwise use PACKET_SIZE
  */

  if (boardRecPtr->WordsToRead  >= FIFO_SIZE) {
    insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, PACKETSIZE);      /* uses rep insw */
    boardRecPtr->ADC_KernBuffPtr += PACKETSIZE;
    boardRecPtr->WordsToRead -= PACKETSIZE;
    if (boardRecPtr->WordsToRead < 2*PACKETSIZE && boardRecPtr->WordsToRead > PACKETSIZE) {
      boardRecPtr->nSpare2 |= ARM;             /* Arm the resisual at next ADHF */
      outw(boardRecPtr->nSpare2,  TRIG_REG);
    }
  } else if (boardRecPtr->WordsToRead < 2*PACKETSIZE && boardRecPtr->WordsToRead > PACKETSIZE) {
    if ( !(boardRecPtr->nSpare2 & ARM)) {
      boardRecPtr->nSpare2 |=  ARM | FFM0;
      outw(boardRecPtr->nSpare2, TRIG_REG);    /* Arm the resisual counter now */
    }
    insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, PACKETSIZE);        /* uses rep insw */
    boardRecPtr->ADC_KernBuffPtr += PACKETSIZE;
    boardRecPtr->WordsToRead -= PACKETSIZE;
    /* check for a race condition before leaving */
    if (inw(IRQ_REG) & EOAI) {
      insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, boardRecPtr->WordsToRead);  /* uses rep insw */
      boardRecPtr->WordsToRead =0;
    }
  } else {                                              /* we are done */
    insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, boardRecPtr->WordsToRead);    /* uses rep insw */
    boardRecPtr->WordsToRead =0;
  }
  outw(boardRecPtr->nSpare0 | INTCL, IRQ_REG);  /* Clear Interrupts */
}

static void pci_das1001_AD_PretrigInt(BoardRec *boardRecPtr)
{
  register int i;
  u32 base1 = boardRecPtr->base1;
  u32 base2 = boardRecPtr->base2;
  u32 base3 = boardRecPtr->base3;
  ADC_ChanRec *ADC_CurrChan = boardRecPtr->ADC_CurrChan;

  #ifdef DEBUG
      printk("Entering pci_das1001_AD_PretrigInt()\n");
  #endif

  if ( inw(TRIG_REG) & INDX_GT ) {  
    /* Pre Trigger index counter has completed its count */
    boardRecPtr->PreTrigMode = FALSE;
    outb(C2+CNTLATCH, COUNTERB_CONTROL);                /* latch the pretrig index */
    boardRecPtr->PreTrigIndexCounter = inb(COUNTERB_0_DATA);         /* Read LSB */
    boardRecPtr->PreTrigIndexCounter |= inb(COUNTERB_0_DATA) << 8;   /* Read MSB */
    i = ADC_CurrChan->pretrigCount - boardRecPtr->PreTrigIndexCounter;
    if ( i > 0 ) {
      boardRecPtr->ADC_KernBuffPtr = boardRecPtr->buf_virt_addr + boardRecPtr->PreTrigBufSize;
      insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, PACKETSIZE);        /* uses rep insw */
      boardRecPtr->WordsToRead -= PACKETSIZE - boardRecPtr->PreTrigIndexCounter;
      boardRecPtr->ADC_KernBuffPtr += PACKETSIZE;
    } else if ( i < 0 ) {
      i = -i;
      boardRecPtr->ADC_KernBuffPtr = boardRecPtr->buf_virt_addr;
      insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, i);   
      insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, PACKETSIZE-i);
      boardRecPtr->WordsToRead -= PACKETSIZE - ADC_CurrChan->pretrigCount;
      boardRecPtr->ADC_KernBuffPtr += PACKETSIZE - i;
    } else { /* i = 0 */
      boardRecPtr->ADC_KernBuffPtr = boardRecPtr->buf_virt_addr;
      insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, PACKETSIZE);
      boardRecPtr->WordsToRead -= PACKETSIZE - boardRecPtr->PreTrigIndexCounter;
      boardRecPtr->ADC_KernBuffPtr += PACKETSIZE;
    }
  } else {
    /* read PACKETSIZE words */
    insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, PACKETSIZE);
    boardRecPtr->PreTrigBufCnt += PACKETSIZE;                  /* increment to next PACKETSIZE chunk */
    boardRecPtr->PreTrigBufCnt %= boardRecPtr->PreTrigBufSize; /* wrap around if at the end          */
    boardRecPtr->ADC_KernBuffPtr = boardRecPtr->buf_virt_addr + boardRecPtr->PreTrigBufCnt;
  }
}

/***********************************************************************
 *  This routine gets called from das1001_Interrupt() when the         *
 *  interrupt was caused by any data going into the FIFO.  Usually     *
 *  gets called when sampling in SINGLEIO mode with interrupts.  For   *
 *  preformance reasons, I am polling in SoftConvert Mode              *
 ***********************************************************************/

static void pci_das1001_AD_NotEmptyInt(BoardRec *boardRecPtr)
{
  u32 base1 = boardRecPtr->base1;
  u32 base2 = boardRecPtr->base2;

#ifdef DEBUG
      printk("Entering pci_das1001_AD_NotEmptyInt()\n");
  #endif

  /* check for FIFO Overflow errors */
  if (inw(IRQ_REG) & LADFUL) {
    printk("pci_das1001_AD_NotEmptyInt: ADC FIFO exceeded full state. Data may be lost\n");
    return;
  }

  do {
    /* Read a sample from FIFO */
    *(boardRecPtr->ADC_KernBuffPtr)++ = inw(ADC_DATA_REG);
    boardRecPtr->WordsToRead--;
  } while ( boardRecPtr->WordsToRead && (inw(IRQ_REG) & ADNE) );

  outw(boardRecPtr->nSpare0 | INTCL, IRQ_REG);  /* Clear Interrupts */
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

static int SoftRead(ADC_ChanRec *chanRec, BoardRec *boardRecPtr)
{
  u16 wReg;
  u32 base1 =  boardRecPtr->base1;
  u32 base2 =  boardRecPtr->base2;
  
  #ifdef DEBUG
      printk("Entering SoftRead().\n");
  #endif

  /* Disable everything first */
  outw(0x0, IRQ_REG);
  outw(0x0, TRIG_REG);

  /* Clear the AD FIFO */
  outw(0x1, ADC_FIFO_CLR);

  /* Set up the ADC Mux and Control register */
  SetADCChannelMux(chanRec->lowChan, chanRec->hiChan, boardRecPtr);  /* Read only this channel */
  SetADCGain(chanRec->gain, boardRecPtr);
  boardRecPtr->nSpare1 &= ~(ADPS1 | ADPS0);
  outw(boardRecPtr->nSpare1, MUX_REG);
  udelay(100);    /* wait 10 usecs for things to settle down */

  boardRecPtr->WordsToRead = chanRec->count;
  boardRecPtr->ADC_KernBuffPtr = boardRecPtr->buf_virt_addr;

  while ( boardRecPtr->WordsToRead ) {
    outw(0x0, ADC_DATA_REG);  /* Force first conversion */

    /*
       Poll EOC for conversion.  Normally, I would use a interrupt
       handler, but the DAS1001 board is too fast, and we get
       a race condition.  Much better to poll for a single conversion
       here.
    */

    do {
      wReg = inw_p(MUX_REG) & EOC;
    } while (!wReg);
    *(boardRecPtr->ADC_KernBuffPtr++) = inw_p(ADC_DATA_REG);  /* Load into buffer */
    udelay(30);    /* wait 30 usecs for things to settle down! */
    boardRecPtr->WordsToRead--;

  #ifdef DEBUG
      printk("SoftRead(): value = %#x.\n", *(boardRecPtr->ADC_KernBuffPtr - 1));
   #endif
  }
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

static int SetADCPacerFreq(BoardRec *boardRecPtr)
{
  u16 ctr1, ctr2;
  u32 product, error, error2;

  if ( boardRecPtr->ADC_freq == 0 ) {
    return -1;
  }

  /* divide 10MHz by frequency */
  product =  (long) 10000000 / boardRecPtr->ADC_freq;

  /* check for rounding error */
  error = abs(10000000 - product*boardRecPtr->ADC_freq);
  error2 = abs(10000000 - (product+1)*boardRecPtr->ADC_freq);
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

  boardRecPtr->ADC_ctr1 = ctr1;
  boardRecPtr->ADC_ctr2 = ctr2;
  boardRecPtr->ADC_freq = 10000000/((long)ctr1*(long)ctr2);

  #ifdef DEBUG
      printk("SetADCPacerFreq: Pacer Frequecy set to %d\n", boardRecPtr->ADC_freq);
  #endif

  return 0;
}


/***************************************************************************
 *
 * Load two part frequency to pacer counter chip.
 *
 ***************************************************************************/

static void LoadADCPacer(BoardRec *boardRecPtr)
{
  u8 mask;
  u8 bData;
  u32 base3 = boardRecPtr->base3;

  /* Write the values of ctr1 and ctr2 into counter A1 and A2 */

  #ifdef DEBUG
      printk("LoadADCPacer: freq = %d   load values: ctr1 %#x ctr2 %#x\n", 
              boardRecPtr->ADC_freq, boardRecPtr->ADC_ctr1, boardRecPtr->ADC_ctr2);
  #endif

  mask = C2+MODE2+LSBFIRST;
  outb_p(mask, COUNTERA_CONTROL); 

  bData = (u8) (boardRecPtr->ADC_ctr2 & 0xff);
  outb_p(bData, COUNTERA_2_DATA); 

  bData = (u8) (boardRecPtr->ADC_ctr2 >> 8);
  outb_p(bData, COUNTERA_2_DATA); 

  mask = C1+MODE2+LSBFIRST;
  outb_p(mask, COUNTERA_CONTROL); 

  bData = (boardRecPtr->ADC_ctr1 & 0xff);
  outb_p(bData, COUNTERA_1_DATA); 

  bData = (boardRecPtr->ADC_ctr1 >> 8) & 0xff;
  outb_p(bData, COUNTERA_1_DATA); 
}


/***************************************************************************
 *
 * Load value into Counter 0  XXXX    Mode    MSB     LSB
 *                            Byte 3  Byte 2  Byte 1  Byte 0
 *
 ***************************************************************************/

static void LoadCounter0(u32 value, BoardRec *boardRecPtr)
{
  u32 base3 = boardRecPtr->base3;
  u8 mask;
  u8 bData;
  
  /* Write the value into Counter 0 Mode 2 */

  #ifdef DEBUG
      printk("LoadCounter0: load value %#x into Counter 0.\n", value);
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

static void LoadResidualCount( u16 resCount, BoardRec *boardRecPtr )
{
  u32 base3 = boardRecPtr->base3;
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

static int  SetADCChannelMux(u8 lowChan, u8 highChan, BoardRec *boardRecPtr)
{
  u8 channelMask;

  #ifdef DEBUG
    printk("SetADCChannelMux: lowChan = %d      highChan = %d\n", lowChan, highChan);
  #endif

  channelMask = (highChan << 4) | lowChan;
  boardRecPtr->nSpare1 &= ~(0xff);
  boardRecPtr->nSpare1 |= channelMask;
  return 0;
}

/***************************************************************************
 *
 * Set ADC gain
 *
 ***************************************************************************/

static int SetADCGain( u16 gain, BoardRec *boardRecPtr )
{
  #ifdef DEBUG
    printk("SetADCGain: gain = %#x\n", gain);
  #endif

  gain &= ~SEDIFF;  /* mask out measurement configuration */

 /* if the gain is different from the one already loaded,
    then calibration coeffs have to be loaded */

  if ( gain != (boardRecPtr->nSpare1 & (UNIBIP | GS1 | GS0))) {
    calibrate_pci_das1001_ain(gain, boardRecPtr);
    boardRecPtr->nSpare1 &= ~(UNIBIP | GS1 | GS0);
    boardRecPtr->nSpare1 |= gain;
  }
  return 0;
}

/***************************************************************************
 *
 * Set DAC gain
 *
 ***************************************************************************/

static u16  SetDACGain(int channel, u16 gain, BoardRec *boardRecPtr)
{
  u16 wReg = 0x0;

  #ifdef DEBUG
    printk("SetDACGain: channel = %d   gain = %#x\n", channel, gain);
  #endif

  if ( channel ) {
    /* select channel 1 */
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

static void StartADCPacer(BoardRec *boardRecPtr)
{
  u32 base1 = boardRecPtr->base1;
  
  boardRecPtr->nSpare1 &= ~ADPS1;
  boardRecPtr->nSpare1 |=  ADPS0;
  outw(boardRecPtr->nSpare1, MUX_REG);
  udelay(10);              /* wait 10 usecs for things to settle down */
  outw(boardRecPtr->nSpare2 & ~TS0, TRIG_REG);
  outw(boardRecPtr->nSpare2 | TS0 | XTRCL, TRIG_REG);

  #ifdef DEBUG
      printk("StartADCPacer: MUX_REG = %#x\n",  boardRecPtr->nSpare1);
      printk("StartADCPacer: TRIG_REG = %#x\n", boardRecPtr->nSpare2);
  #endif
}

/***************************************************************************
 *
 * Turn off ADC pacer timer chip.
 *
 ***************************************************************************/

void StopADCPacer(BoardRec *boardRecPtr)
{
  u8 mask;
  u32 base1 = boardRecPtr->base1;
  u32 base3 = boardRecPtr->base3;

  boardRecPtr->nSpare1 &= ~(ADPS1 | ADPS0);
  outw(boardRecPtr->nSpare1, MUX_REG);

  mask = C2+MODE2+LSBFIRST;
  outb_p(mask, COUNTERA_CONTROL);
  mask = C1+MODE2+LSBFIRST;
  outb(mask, COUNTERA_CONTROL);
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

static int PacerRead(ADC_ChanRec *chanRec, BoardRec* boardRecPtr, struct file *readfile)
{
  u32 base1 =  boardRecPtr->base1;
  u32 base2 =  boardRecPtr->base2;
  u16 residualCnt, sampleCnt;
  unsigned long flags;

  #ifdef DEBUG
      printk("Entering PacerRead().\n");
#endif

 /*
    For Non Blocking Read:  check to see if all the words
    have been read.  
 */

  if (chanRec->nonBlockFlag && boardRecPtr->nonBlockFile) {
    if (boardRecPtr->WordsToRead == 0) {  // check to see if we are done
      boardRecPtr->nonBlockFile = NULL;
      return 0;
    } else {
      return -EAGAIN;
    }
  }

  /* Prepare global data for parameterless interrupt handler */
  boardRecPtr->ADC_CurrChan = chanRec;                         /* pass chanel number to global */
  boardRecPtr->ADC_KernBuffPtr = boardRecPtr->buf_virt_addr;   /* same with ADC_KernBuff       */

  /* Disable everything first */
  /* outw(0x0, IRQ_REG); */
  outw(0x0, TRIG_REG);

  
  /* Clear the AD FIFO */
  outw(0x1, ADC_FIFO_CLR);

  if ( chanRec->pretrigCount ) {
    boardRecPtr->PreTrigMode = TRUE;
    if ( (chanRec->pretrigCount%PACKETSIZE) == 0 ) {
      boardRecPtr->PreTrigBufSize = chanRec->pretrigCount;
    } else {
      boardRecPtr->PreTrigBufSize = ((chanRec->pretrigCount / PACKETSIZE) + 1)*PACKETSIZE;
    }
    boardRecPtr->PreTrigBufCnt = 0;
    boardRecPtr->PreTrigIndexCounter = 0;
    boardRecPtr->WordsToRead = chanRec->count - chanRec->pretrigCount;
  } else {
    boardRecPtr->WordsToRead = chanRec->count;
  }

  /* Load the residual count 

     We can get into a race condition here.  After an interrupt at FIFO
     half full, we need to make sure we have enough time to read the buffer
     (PACKETSIZE) before the next interrupt.  I am assuming a time of 640 us.
     If there is not enough time, it is better to sample another group and
     only read the number we need.
  */

  residualCnt = chanRec->count % PACKETSIZE;
  sampleCnt = boardRecPtr->ADC_freq / 1562;
  if ( sampleCnt > PACKETSIZE ) sampleCnt = PACKETSIZE;

  if ( sampleCnt > residualCnt ) {
    LoadResidualCount(sampleCnt, boardRecPtr);      
  } else {
    LoadResidualCount(residualCnt, boardRecPtr);      
  }

  /* Set up the ADC Mux and Control register */
  SetADCChannelMux(chanRec->lowChan, chanRec->hiChan, boardRecPtr);
  SetADCGain(chanRec->gain, boardRecPtr);

  /* Load the Trigger Control Register */
  boardRecPtr->nSpare2 = chanRec->nSpare2;

  /* Load the Trigger Control/Status Register */
  if ( chanRec->count < 2*PACKETSIZE && chanRec->count >= PACKETSIZE) {
    boardRecPtr->nSpare2 |= ARM;
  } else if ( chanRec->count < PACKETSIZE ) {
    boardRecPtr->nSpare2 |= ARM | FFM0;
  }

  if ( (chanRec->nSpare2 & (TS1 | TS0)) == 0 ) {  /* No trigger enabled */
    boardRecPtr->nSpare2 |= (TS0 | XTRCL);   /* Enable SW Trigger and clear XTRIG */
  } else {
    boardRecPtr->nSpare2 |=  XTRCL;          /*  clear XTRIG */
  }

  if ( chanRec->pretrigCount ) {
      boardRecPtr->nSpare2 |= PRTRG;
  } else {
      boardRecPtr->nSpare2 &= ~PRTRG;
  }

  /* Enable interrupts */
  save_flags(flags);
  cli();
  boardRecPtr->nSpare0 |= (INTE | INT1 | EOAIE);
  boardRecPtr->nSpare0 &= ~(INT0);
  restore_flags(flags);
  outw(boardRecPtr->nSpare0 | ADFLCL | INTCL | EOACL, IRQ_REG);

  if ( chanRec->pacerSource == ADC_PACER_CLOCK ) {
    LoadADCPacer(boardRecPtr);                /* Establish sample frequency */
  }
  outw(boardRecPtr->nSpare1, MUX_REG);
  udelay(10);    /* wait 10 usecs for things to settle down */

  #ifdef DEBUG
    printk("PacerRead: Enter interrupt.  nSpare0 = %#x.\n", boardRecPtr->nSpare0);
    printk("PacerRead: Enter interrupt.  nSpare1 = %#x.\n", boardRecPtr->nSpare1);
    printk("PacerRead: Enter interrupt.  nSpare2 = %#x.\n", boardRecPtr->nSpare2);
#endif

  outw(boardRecPtr->nSpare2, TRIG_REG);          /* let's go ... */

  if (chanRec->nonBlockFlag) {  /* do this one time only */
    boardRecPtr->nonBlockFile = readfile;
    return -EAGAIN;
  } else {
    wait_event_interruptible(das1001_wait, (boardRecPtr->WordsToRead==0) != 0);  // Block in wait state 
  }

  if ( boardRecPtr->WordsToRead != 0 ) {
      printk("Timing error in PacerRead: WordsToRead = %d\n", boardRecPtr->WordsToRead);
      return -1;
  }

  StopADCPacer(boardRecPtr);
  #ifdef DEBUG
    printk("Leaving PacerRead.\n");
  #endif
  return 0;
}

static void SoftWrite( int channel, int gain, u16 value, BoardRec *boardRecPtr )
{
  u32 base1 = boardRecPtr->base1;
  u32 base4 = boardRecPtr->base4;
  u16 wReg;

  /* get current gain info */
  wReg = boardRecPtr->nSpare3 & (DAC1R1 | DAC1R0 | DAC0R1 | DAC0R0);

  if ( channel ) {
    wReg &= ~(DAC1R1 | DAC1R0);
    wReg |= SetDACGain(channel, gain, boardRecPtr);
  } else {
    wReg &= ~(DAC0R1 | DAC0R0);
    wReg |= SetDACGain(channel, gain, boardRecPtr);
  }

  /* Enable DAC */
  wReg |= (DACEN);

  #ifdef DEBUG
    printk("SoftWrite: channel = %d  DAC_REG = %#x, value = %#x\n", 
            channel, wReg, value);
  #endif

  outw(wReg, DAC_REG);

  /* 
     If the range has changed between the last output and the
     current one, then reload the calibration coefficients.
  */

  if ( channel ) {  /* DAC channel 1 */
    if ( (wReg & (DAC1R1 | DAC1R0)) != (boardRecPtr->nSpare3 & (DAC1R1 | DAC1R0))) {
      calibrate_pci_das1001_aout( channel, gain, boardRecPtr );
      boardRecPtr->nSpare3 &= ~(DAC1R1 | DAC1R0);                  /* clear the bits */
      boardRecPtr->nSpare3 |= (wReg & (DAC1R1 | DAC1R0));          /* set the bits   */
    }
    outw(value, DAC1_DATA_REG);                                    /* output data value */
  } else {    /* DAC channel 0 */
    if ( (wReg & (DAC0R1 | DAC0R0)) != (boardRecPtr->nSpare3 & (DAC0R1 | DAC0R0))) {
      calibrate_pci_das1001_aout( channel, gain, boardRecPtr );
      boardRecPtr->nSpare3 &= ~(DAC0R1 | DAC0R0);                  /* clear the bits */
      boardRecPtr->nSpare3 |= (wReg & (DAC0R1 | DAC0R0));          /* set the bits   */
    }
    outw(value, DAC0_DATA_REG);                                    /* output data value */
  }
}

/*************************************************************************** 
 * Reads the calibration coeffs from non-volatile RAM in a PCI-DAS1001     *
 * and writes them to the appropriate trim-DACs associated with the analog * 
 * output sections.                                                        *
 * *************************************************************************/

void calibrate_pci_das1001_aout( int channel, int gain, BoardRec *boardRecPtr )
{
  u32 base0 = boardRecPtr->base0;
  u32 base1 = boardRecPtr->base1;
  u16 addr;
  u8 value;

  #ifdef DEBUG
    printk("calibrate_pci_das1001_aout: channel %d set to gain %#x.\n",
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
      printk("pci1001_AOut: unknown gain %d.\n", gain);
      return;
  }

  if ( channel ) {
    /* DAC1 */
    addr += 0x10;  /* add 16 bytes if DAC1 */
    value = get_pci_das1001_nVRam( addr++, base0 );
    write_pci_das1001_8800( DAC1_COARSE_GAIN, value, base1 );
    value = get_pci_das1001_nVRam( addr++, base0 );
    write_pci_das1001_8800( DAC1_OFFSET, value, base1 );
    value = get_pci_das1001_nVRam( addr++, base0 );
    write_pci_das1001_8800( DAC1_FINE_GAIN, value, base1 );
  } else {
    /* DAC0 */
    value = get_pci_das1001_nVRam( addr++, base0 );
    write_pci_das1001_8800( DAC0_COARSE_GAIN, value, base1 );
    value = get_pci_das1001_nVRam( addr++, base0 );
    write_pci_das1001_8800( DAC0_OFFSET, value, base1 );
    value = get_pci_das1001_nVRam( addr++, base0 );
    write_pci_das1001_8800( DAC0_FINE_GAIN, value, base1 );
  }
}

/*************************************************************************** 
 * Reads the calibration coeffs from non-volatile RAM in a PCI-DAS1001     *
 * and writes them to the appropriate trim-DACs associated with the analog * 
 * input sections.                                                         *
 ***************************************************************************/

void calibrate_pci_das1001_ain( int gain, BoardRec *boardRecPtr )
{
  u32 base0 = boardRecPtr->base0;
  u32 base1 = boardRecPtr->base1;
  u16 addr;
  u8 value;

  /*
     Get the starting offset in nvRAM at which the 
     calibration coeffs for the given range is stored.
  */

  #ifdef DEBUG
      printk("calibrate_pci_das1001_ain: resetting calibration coefficients. gain = %d\n",
              gain);
  #endif

  switch( gain ) {
    case BP_10_00V:
      addr = ADC_BIP10V_COEFF;
      break;
    case BP_1_00V:
      addr = ADC_BIP1V_COEFF;
      break;
    case BP_0_10V:
      addr = ADC_BIP0PT1V_COEFF;
      break;
    case BP_0_01V:
      addr = ADC_BIP0PT01V_COEFF;
      break;
    case UP_10_00V:
      addr = ADC_UNI10V_COEFF;
      break;
    case UP_1_00V:
      addr = ADC_UNI1V_COEFF;
      break;
    case UP_0_10V:
      addr = ADC_UNI0PT1V_COEFF;
      break;
    case UP_0_01V:
      addr = ADC_UNI0PT01V_COEFF;
      break;
    default:
      printk("pci1001_ain: unknown gain %d.\n", gain);
      return;
  }

  value = get_pci_das1001_nVRam(addr++, base0);             /* get ADC Coarse offset */
  #ifdef DEBUG
  printk("ADC coarse offset = %#x  ", value);
  #endif
  write_pci_das1001_8800(ADC_COARSE_OFFSET, value, base1);  /* write DAC Trim coarse offset */

  value = get_pci_das1001_nVRam(addr++, base0);             /* get ADC Fine offset */
  #ifdef DEBUG
    printk("ADC fine offset = %#x  ", value);
  #endif
  write_pci_das1001_8800(ADC_FINE_OFFSET, value, base1);    /* write DAC Trim fine offset */

  value = get_pci_das1001_nVRam(addr, base0);               /* get ADC gain */
  #ifdef DEBUG
    printk("ADC gain = %#x\n", value);
  #endif
  write_pci_das1001_7376(value, base1);                     /* set ADC gain */ 

  udelay(1000);          /* waint 1 milliseconds when changing gains */
}
/*************************************************************************** 
 * Reads a byte from the specified address of the                          *
 * non-volatile RAM on a PCI-DAS1001.                                      *
 ***************************************************************************/

u8 get_pci_das1001_nVRam( u16 addr, u32 base0 )
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


void write_pci_das1001_8800( u8 addr, u8 value, u32 base1 )
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


void write_pci_das1001_7376(u8 value, u32 base1 )
{
  int i;

  /* select device  */
  outw_p(SEL7376, CALIBRATE_REG );

  /* Write out each bit of the byte starting with the msb */
  for ( i = 0; i < 7; i++) {
    value <<= 1;               /* use only 7 bits of data */
    if ( value & 0x80 ) {
        outw_p(SDI | SEL7376, CALIBRATE_REG );
    } else {
        outw_p(SEL7376, CALIBRATE_REG );
    }
  }

  /* Deselect the device */
  outw_p(0x0, CALIBRATE_REG );
}

void SetTrigger( u32 value, ADC_ChanRec *chanRecPtr )
{
  switch(value) {
    case DISABLE_TRIGGER:
      chanRecPtr->nSpare2 &= ~(TS1 | TS0 | TGEN);
      break;
    case SW_TRIGGER:
      chanRecPtr->nSpare2 &= ~(TS1 | TS0 | TGEN);
      chanRecPtr->nSpare2 |= TS0;
      break;
    case EXTERNAL_TRIGGER:
      chanRecPtr->nSpare2 &=  ~(TS1 | TS0 | TGEN);
      chanRecPtr->nSpare2 |= (TS1 | TGEN);
      break;
    default:
      /* Trigger type has not been selected: default disable */
      chanRecPtr->nSpare2 &=  ~(TS1 | TS0);
      break;
  }
}

