/***************************************************************************
 Copyright (C) 2013-2015  Warren J. Jasper    <wjasper@ncsu.edu>
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
 *   Note: Interrupts, when enabled, are generated differently depending
 *   upon the mode of the board:
 *
 *   Soft Convert:   poll IRQDATA for A/D done.
 *   Pacer clock:    interrupts generated with FIFO half full
 *                   or when Total_Count = 0.
 *
 *   Documentation:
 *       PCIM-DAS16jrJR/16  User's Manual Revision 5, June 2006
 *       Register Map for the PCIM-DAS16JR/16
 *
 *       Measurement Computing
 *       10 Commerce Way
 *       Suite 1008
 *       Norton, MA  02766
 *       (508) 946-5100 www.mccdaq.com
 *
 *
 *       
 */

/***************************************************************************
 *                                                                         *
 * a2dc_2_6_29.c for PCIM-DAS16JR/16                                       *
 *                                                                         *
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
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include "pcim-das16jr.h"
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
static int  SetADCPacerFreq(BoardRec *board);
static int  SetADCChannelMux(u8 lowChan, u8 highChan, BoardRec *boardRecPtr);
static void StartADCPacer(BoardRec *boardRecPtr);
static void StopADCPacer(BoardRec *boardRecPtr);
static void LoadADCPacer(BoardRec *boardRecPtr);
static void LoadCounter0(u32 value, BoardRec *boardRecPtr);
static void LoadResidualCount(u16 value, BoardRec *boardRecPtr);

static void AD_HalfFullInt(BoardRec *boardRecPtr);
static void AD_NotEmptyInt(BoardRec *boardRecPtr);
static void AD_EndOfAcquisitionInt(BoardRec *boardRecPtr);

static int __init das16jr_init(void);
static void __exit das16jr_exit (void);
static ssize_t das16jr_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t das16jr_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int das16jr_open(struct inode *iNode, struct file *filePtr);
static int das16jr_close(struct inode *iNode, struct file *filePtr);
static long das16jr_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg);
static int das16jr_mmap(struct file *filePtr, struct vm_area_struct * vma);
static unsigned int das16jr_poll(struct file *filePtr, poll_table *wait);
static int das16jr_fault(struct vm_area_struct *vma, struct vm_fault *vmf);
static irqreturn_t das16jr_Interrupt(int irq, void *dev_id);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCIM-DAS16JR/16 module");
MODULE_LICENSE("GPL")
;
module_init(das16jr_init);
module_exit(das16jr_exit);

static int MajorNumber = DEFAULT_MAJOR_DEV;          /* Major number compiled in    */
static BoardRec BoardData[MAX_BOARDS];               /* Board specific information  */
static ADC_ChanRec ChanADC[MAX_BOARDS][AD_CHANNELS]; /* ADC Channel specific info   */
static DECLARE_WAIT_QUEUE_HEAD(das16jr_wait);
static int NumBoards = 0;                            /* number of boards found      */
static spinlock_t das16jr_lock;

static struct cdev das16jr_cdev;
static struct class *das16jr_class;

static struct file_operations das16jr_fops = {
  .owner          = THIS_MODULE,
  .read           = das16jr_read,
  .write          = das16jr_write,
  .unlocked_ioctl = das16jr_ioctl,
  .mmap           = das16jr_mmap,
  .open           = das16jr_open,
  .release        = das16jr_close,
  .poll           = das16jr_poll,
};

static struct vm_operations_struct das16jr_vops = {
  .fault = das16jr_fault,
};


  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int das16jr_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void das16jr_remove_one(struct pci_dev *pdev);

static struct pci_device_id das16jr_id_tbl[] = {
  {
  .vendor     =   PCI_VENDOR_ID_CBOARDS,
  .device     =   PCI_DEVICE_ID_CBOARDS_PCIM_DAS16JR,
  .subvendor  =   PCI_ANY_ID,
  .subdevice  =   PCI_ANY_ID,
  .class      =   0,
  .class_mask =   0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, das16jr_id_tbl);

static struct pci_driver pcim_das16jr_driver = {
  .name     =   "pcim_das16jr",
  .id_table =   das16jr_id_tbl,
  .probe    =   das16jr_init_one,
  .remove   =   das16jr_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/
static int __init das16jr_init(void) 
{
  int err;
  dev_t dev;

 /* Register as a device with kernel.  */
  if (MajorNumber) {
    dev = MKDEV(MajorNumber, 0);
    err = register_chrdev_region(dev, 0xff, "pcim_das16jr");
  } else {
    err = alloc_chrdev_region(&dev, 0, 0xff, "pcim_das16jr");
    MajorNumber = MAJOR(dev);
  }
  if (err < 0) {
    printk("%s: Failure to load module. Major Number = %d  error = %d\n",
	   ADAPTER_ID, MAJOR(dev), err);
    return err;
  }

  cdev_init(&das16jr_cdev, &das16jr_fops);
  das16jr_cdev.owner = THIS_MODULE;
  das16jr_cdev.ops = &das16jr_fops;
  err = cdev_add(&das16jr_cdev, dev, 0xff);
  if (err) {
    printk("%s: Error %d in registering file operations", ADAPTER_ID, err);
    return err;
  }

  /* create the class for the pcim-das16jr/16 */
  das16jr_class = class_create(THIS_MODULE, "pcim-das16jr");
  if (IS_ERR(das16jr_class)) {
    return PTR_ERR(das16jr_class);
  }

  err = pci_register_driver(&pcim_das16jr_driver);
  return  err;	
}

static int das16jr_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  int minor;
  u32 base0;
  u32 base1;
  u32 base2;
  u32 base3;
  u32 lReg;
  struct page *page;
  char name[64];
  
  if (NumBoards >= MAX_BOARDS) {
    printk("das16jr_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit a2dc.h.\n", NumBoards);
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

  /* pci_enable_device handles IRQ routing, so it must be before request_irq */
  if (pci_enable_device(pdev)) {
    goto err_out_1;
  }

  /* Reset FIFO interrupts and disable all interupts */
  lReg = inl(INTCSR_ADDR);
  lReg &= ~(PLX_INTE | PLX_PCIINT);
  outl(lReg, INTCSR_ADDR);
  
  /* Register interrupt handler. */
  BoardData[NumBoards].irq = pdev->irq;
  if (request_irq(pdev->irq, das16jr_Interrupt, (IRQF_SHARED), "pcim_das16jr", (void *) &BoardData[NumBoards])) {
    // No free irq found! cleanup and exit 
    printk(KERN_WARNING"%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData[NumBoards].irq);
    goto err_out_1;
  }

  // Determine the jumper setting of the Pacer clock */
  BoardData[NumBoards].nSpare5 = inb_p(ADC_CHAN_REG);
  if (BoardData[NumBoards].nSpare5 & CLK) {
    BoardData[NumBoards].PacerClock = 10000000;
    printk("das16jr_init_one: Pacer clock set to 10 MHz\n");
  } else {
    BoardData[NumBoards].PacerClock = 1000000;
    printk("das16jr_init_one: Pacer clock set to 1 MHz\n");
  }

  if (BoardData[NumBoards].nSpare5 & MUX) {
    BoardData[NumBoards].nChannels = 16;
    printk("das16jr_init_one: Analog input modes set to 16 single-ended.\n");
  } else {
    BoardData[NumBoards].nChannels = 8;
    printk("das16jr_init_one: Analog input modes set to 8 differential.\n");
  }
  
  BoardData[NumBoards].ADC_freq = DEFAULT_FREQ;   // Set default acquisition frequency 
  SetADCPacerFreq(&BoardData[NumBoards]);

  
  /* Set all channel structures to show nothing active/open */
  for (i = 0; i < AD_CHANNELS; i++) {
    ChanADC[NumBoards][i].open = FALSE;
    ChanADC[NumBoards][i].nonBlockFlag = FALSE;
    ChanADC[NumBoards][i].lowChan = i;
    ChanADC[NumBoards][i].hiChan = i;
    ChanADC[NumBoards][i].gain = UP_10_00V;   // set range to 0-10V 
    ChanADC[NumBoards][i].burstMode = FALSE;
  }

  // create a device in /sys/class/das16jr/
  for (i = 0; i < BoardData[NumBoards].nChannels; i++) {
    minor = (NumBoards<<0x5) + i;
    sprintf(name, "das16jr/ad%d_%d", NumBoards, i);
    device_create(das16jr_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  }

  BoardData[NumBoards].nonBlockFile = NULL;

  BoardData[NumBoards].nSpare1 = (GATE_EN);
  outb(0x0, MUX_REG);  // reset FIFO
  udelay(10);

  // Reset Interrupt on PLX PCI9052 PCI controller 
  lReg = inl(INTCSR_ADDR);
  lReg |= (PLX_INTE | PLX_PCIINT | PLX_INTCLR);
  outl(lReg, INTCSR_ADDR);

  BoardData[NumBoards].busyRead = FALSE;          // board ready

  init_waitqueue_head(&das16jr_wait);
  spin_lock_init(&das16jr_lock);

  BoardData[NumBoards].buf_phy_size = ALIGN_ADDRESS(ADC_BUFF_PHY_SIZE, PAGE_SIZE);
  BoardData[NumBoards].buf_virt_addr = (u16 *) pci_alloc_consistent( 0,
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
    printk("das16jr_init_one: Board %d: buf_phy_size = %#x\n",
	   NumBoards, BoardData[NumBoards].buf_phy_size);
    printk("das16jr_init_one: Board %d: buf_virt_addr = %p  buf_bus_addr = %#x\n",
	   NumBoards, BoardData[NumBoards].buf_virt_addr, (int) BoardData[NumBoards].buf_bus_addr);
  #endif
  
  printk("%s: BADR0=%#x BADR1=%#x BADR2=%#x BADR3=%#x \n",
          ADAPTER_ID, base0, base1, base2, base3);
  printk("%s: IRQ=%d 8/8/2015 wjasper@ncsu.edu\n", ADAPTER_ID, BoardData[NumBoards].irq);

  NumBoards++;
  return 0;

err_out_1:
  free_irq(pdev->irq, (void *) &BoardData[NumBoards]);

err_out_0:
  return -ENODEV;
}


/**********************************************************************************
 *                                                                                *
 * Remove driver. Called when "rmmod pcim_das16jr" is run on the command line. *
 *                                                                                *
 **********************************************************************************/

void __exit das16jr_exit(void) 
{
  dev_t dev;

  dev = MKDEV(MajorNumber, 0);
  pci_unregister_driver(&pcim_das16jr_driver);
  class_destroy(das16jr_class);
  cdev_del(&das16jr_cdev);
  unregister_chrdev_region(dev, 0xff);
  printk("%s: module removed from das16jr_exit.\n", ADAPTER_ID);
}

static void das16jr_remove_one(struct pci_dev *pdev)
{
  struct page *page;
  int i;
  int minor;

  NumBoards--;

  StopADCPacer(&BoardData[NumBoards]);
  if (BoardData[NumBoards].irq >= 0 ) {
    free_irq(BoardData[NumBoards].irq, (void *) &BoardData[NumBoards]);
  }

  for (i = 0; i < BoardData[NumBoards].nChannels; i++) {
    minor = (NumBoards<<0x5) + i;
    device_destroy(das16jr_class, MKDEV(MajorNumber, minor));
  }

  // clear reserved flag on DMA pages
  page = virt_to_page(BoardData[NumBoards].buf_virt_addr);
  for (i = 0; i < BoardData[NumBoards].buf_phy_size/PAGE_SIZE; i++) {
    ClearPageReserved(&page[i]);
  }

  pci_free_consistent(0,
		      BoardData[NumBoards].buf_phy_size,
		      BoardData[NumBoards].buf_virt_addr,
		      BoardData[NumBoards].buf_bus_addr);

  #ifdef DEBUG
    printk("%s: Board %d removed.\n", ADAPTER_ID, NumBoards);
  #endif
}   /* das16jr_remove_one() */

/*************************************
 *                                   *
 * open() service handler            *
 *                                   *
 *************************************/

static int das16jr_open(struct inode *iNode, struct file *filePtr)
{
  u32 base3;
  unsigned long flags;

  int board = 0;
  int chan  = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);    /* get which board   */
  chan  = CHAN(minor);     /* get which channel */
  base3 = BoardData[board].base3;

  /* 
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes 
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */

  if ( chan >= 0 && chan < AD_CHANNELS ) {
    if ( ChanADC[board][chan].open == TRUE ) {
      return -EBUSY;
    }

    ChanADC[board][chan].open = TRUE;                           /* The device is open          */
    ChanADC[board][chan].gain = BP_10_00V;                      /* +/- 10V , Differential Mode */
    ChanADC[board][chan].pacerSource = ADC_SOFT_CONVERT;        /* set acquisition mode        */
    ChanADC[board][chan].lowChan = chan;
    ChanADC[board][chan].hiChan = chan;
    ChanADC[board][chan].burstMode = FALSE;

    /* Reset FIFO interrupts and disable all A/D interupts */
    spin_lock_irqsave(&das16jr_lock, flags);
    BoardData[board].nSpare0 = 0x0;
    spin_unlock_irqrestore(&das16jr_lock, flags);
    outb(BoardData[board].nSpare0, INTCSR_REG);

    BoardData[board].nonBlockFile = NULL;

    #ifdef DEBUG
    printk("%s: open(): ADC minor %d mode %d.\n",
	   ADAPTER_ID, minor, ChanADC[board][chan].pacerSource);
    #endif
    return 0;   
  }

  return 0;
}

/*****************************
 *                           *
 * close() service handler   *
 *                           *
 *****************************/

static int das16jr_close(struct inode *iNode, struct file *filePtr)
{
  u32 base3;
  int board = 0;
  int chan = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);    /* get which board   */
  chan = CHAN(minor);      /* get which channel */
  base3 = BoardData[board].base3;

  if (filePtr && BoardData[board].nonBlockFile && filePtr == BoardData[board].nonBlockFile) {
    // cancel pending non-blocking i/o 
    // disable acquisition 
    #ifdef DEBUG
      printk("close()  clearing ADC FIFO and nonblock flag.\n");
    #endif
    StopADCPacer(&BoardData[board]);  // disable pacer if in pacer mode 
    
  /* Clear ADC FIFO Pointer */
    BoardData[board].nonBlockFile = NULL; // clear non-block flag
  }
 
  if (chan >= 0 && chan < AD_CHANNELS) {
    /* ADC */
    ChanADC[board][chan].open = FALSE;
    ChanADC[board][chan].nonBlockFlag = FALSE;
  } else {
    printk("das16jr_close: Incorrect minor number (%d).\n", minor);
  }  

  #ifdef DEBUG
      printk("%s: close() of minor number %d.\n", ADAPTER_ID, minor);
  #endif

  return 0;
}


/****************************
 *                          *
 * read() service function  *
 *                          *
 ****************************/

static ssize_t das16jr_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  u32 base3;
  int minor;
  int chan;
  int board;

  struct inode *iNode = filePtr->f_path.dentry->d_inode;
  minor = iminor(iNode);
  board = BOARD(minor);         /* get which board   */
  chan =  CHAN(minor);          /* get which channel */
  base3 = BoardData[board].base3;

  /* Initialize the pointer into the data buffer array */
  BoardData[board].ADC_KernBuffPtr = BoardData[board].buf_virt_addr;
 
  ChanADC[board][chan].nonBlockFlag = (filePtr->f_flags & O_NONBLOCK);
  
  if (count < 1) {
    printk("das16jr_read(): count must be greater than 0.\n");
    BoardData[board].busyRead = FALSE;
    return (-1);
  }

  if (chan >= 0 && chan < AD_CHANNELS) {
    if (BoardData[board].nonBlockFile && BoardData[board].nonBlockFile != filePtr) {
      /* somebody else has a non-blocking request going */
      return -EBUSY;
    } else if (BoardData[board].busyRead == TRUE  &&
	       !ChanADC[board][chan].nonBlockFlag) { /* if board is in use, return busy */
      return (-EBUSY);
    } else {
      BoardData[board].busyRead = TRUE;
    }

    /* Set up configuration */

    ChanADC[board][chan].count = count;
    BoardData[board].WordsToRead = count;

    if (count > 2*BoardData[board].buf_phy_size) {
        printk("das16jr_read(): requesting too large a count size.\n");
        BoardData[board].busyRead = FALSE;
        return (-1);
    }

    /* Set the gain level */
    outb(ChanADC[board][chan].gain, GAIN_REG);

    /* Set up the MUX ranges  and clear FIFO */
    SetADCChannelMux(ChanADC[board][chan].lowChan, ChanADC[board][chan].hiChan, &BoardData[board]);

    /* Read */
    switch (ChanADC[board][chan].pacerSource) {

      case ADC_SOFT_CONVERT:
          #ifdef DEBUG
              printk("das16jr_read(): Entering ADC_SOFT_CONVERT mode.  count = %d\n",
		     (int) count);
          #endif
      
          if (SoftRead(&ChanADC[board][chan], &BoardData[board])) {
              printk("das16jr_read: SoftRead() failed.\n");
              BoardData[board].busyRead = FALSE;
              return(-1);
          } 
          break;

      case ADC_EXTERNAL_PACER_FALLING:
        #ifdef DEBUG
            printk("das16jr_read(): Entering ADC_EXTERNAL_PACER_FALLING mode.\n");
        #endif

         BoardData[board].nSpare1 |= EXT_PACER_POL | PS1;
         BoardData[board].nSpare1 &= ~PS0;
         if (PacerRead(&ChanADC[board][chan], &BoardData[board], filePtr)) {
            printk("das16jr_read: PacerRead() failed.\n");
            BoardData[board].busyRead = FALSE;
            return(-1);
	 }
         break;

      case ADC_EXTERNAL_PACER_RISING:
        #ifdef DEBUG
            printk("das16jr_read(): Entering ADC_EXTERNAL_PACER_RISING mode.\n");
        #endif

         BoardData[board].nSpare1 |= PS1;           /* Select Pacer Source */
         BoardData[board].nSpare1 &= ~(EXT_PACER_POL | PS0);
         if (PacerRead(&ChanADC[board][chan], &BoardData[board], filePtr)) {
            printk("das16jr_read: PacerRead() failed.\n");
            BoardData[board].busyRead = FALSE;
            return(-1);
	 }
         break;

      case ADC_PACER_CLOCK:
          #ifdef DEBUG
              printk("das16jr_read(): Entering ADC_PACER_CLOCK mode.\n");
          #endif

          if ( ChanADC[board][chan].count == 1 ) {            /* use SoftRead */
              if (SoftRead(&ChanADC[board][chan], &BoardData[board])) {
                  printk("das16jr_read: SoftRead() failed with pacer.\n");
                  BoardData[board].busyRead = FALSE;
                  return(-1);
              }
          } else {
	    StopADCPacer(&BoardData[board]);                    // disable pacer if in pacer mode 
	    BoardData[board].nSpare1 &= ~EXT_PACER_POL;         // external pacer polarity set to positive edge
	    BoardData[board].nSpare1 |=  (GATE_EN | PS1 | PS0); // Internal Pacer Clock CTR 2 OUT
              switch (PacerRead(&ChanADC[board][chan], &BoardData[board], filePtr)) {
  	        case 0:
                  break;  /* success */
                case -EAGAIN:
		  return -EAGAIN; /* non-blocking, and not complete */
		  break;
	        default:
                  printk("das16jr_read: PacerRead() failed.\n");
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
      if (copy_to_user(buf, BoardData[board].buf_virt_addr,
		       ChanADC[board][chan].count*sizeof(u16)) ) {
	return -EFAULT;
      }
    }

    BoardData[board].busyRead = FALSE;
    return(ChanADC[board][chan].count);     /* return number of samples read */
  }
  return -1;
}

/*****************************
 *                           *
 * write() service function  *
 *                           *
 *****************************/

static ssize_t das16jr_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  u32 base2;
  u32 base3;

  int minor;
  int board;
  int chan;
  struct inode *iNode = filePtr->f_path.dentry->d_inode;

  minor = iminor(iNode);
  board = BOARD(minor);         /* get which board   */
  chan =  CHAN(minor);          /* get which channel */
  
  #ifdef DEBUG
  printk("das16jr_write(): Minor = %d, Board = %d  Channel = %d, Count = %d\n",
	 minor, board, chan, (int) count);
  #endif

  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;

  if ( count < 1 ) {
    printk("das16jr_write(): count must be greater than 0.\n");
    return (-1);
  }

  return 1;
}


/*****************************
 *                           *
 * iotctl() service handler  *
 *                           *
 *****************************/

static long das16jr_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  struct inode *iNode = filePtr->f_path.dentry->d_inode;
  int minor = iminor(iNode);
  int err = 0;
  int size = _IOC_SIZE(cmd);       /* the size bitfield in cmd */
  int board;
  int chan;
  u32 base2;
  u32 base3;

  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);              /* get which channel */

  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;

  /* 
   * extract the type and number bitfields, and don't decode
   * wrong cmds;  return EINVAL before access_ok()
   */

  if (_IOC_TYPE(cmd) != IOCTL_MAGIC) return -EINVAL;
  if (_IOC_NR(cmd) > IOCTL_MAXNR)  return -EINVAL;

  if (_IOC_DIR(cmd) & _IOC_READ) {
    err = !access_ok(VERIFY_WRITE, (void *)arg, size);
  } else if (_IOC_DIR(cmd) & _IOC_WRITE) {
     err = !access_ok(VERIFY_READ, (void *)arg, size);
  }
  if (err) return -EFAULT;

  /* global ioctl calls, not specific to A/D, D/A, or DIO */
  switch (cmd) {
    case GET_BUF_SIZE:
      put_user( (long) BoardData[board].buf_phy_size, (long*) arg );
      return 0;
      break;
  }

  if ( chan >= 0 && chan < AD_CHANNELS ) {
    switch (cmd) {
      case ADC_SET_GAINS:
        #ifdef DEBUG
          printk("ioctl ADC_SET_GAINS: Channel = %d, Gain = %ld\n", chan, arg);
        #endif
        ChanADC[board][chan].gain &= ~(UNIBP | G1 | G0);
        ChanADC[board][chan].gain |=  (u16) arg & (UNIBP | G1 | G0);
        break;
      case ADC_GET_GAINS:
	put_user( (long)ChanADC[board][chan].gain, (long*) arg);
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
      case ADC_SET_MODE:
        ChanADC[board][chan].pacerSource = arg;
        #ifdef DEBUG
  	  printk("ioctl: ADC_SET_MODE: mode = %d\n", (int) arg);
	#endif
	break;
      case ADC_COUNTER0:
        LoadCounter0( (u32) arg, &BoardData[board]);
        break;
      case ADC_SET_MUX_LOW:
        ChanADC[board][chan].lowChan = (u8) arg;
        break;
      case ADC_SET_MUX_HIGH:
        ChanADC[board][chan].hiChan = (u8) arg;
        break;
      case ADC_GET_CHAN_MUX_REG:
        put_user(inb(MUX_REG), (long*) arg);
        break;
      case ADC_BURST_MODE:
	ChanADC[board][chan].burstMode = TRUE;
        break;
      case ADC_SET_TRIGGER:
        ChanADC[board][chan].pacerSource = arg;
        break;
      case ADC_NBIO_CANCEL:
        if (BoardData[board].nonBlockFile && (filePtr == BoardData[board].nonBlockFile || arg )) {
	  // disable acquisition
          StopADCPacer(&BoardData[board]);  // disable pacer if in pacer mode 
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
      case ADC_GET_CLK_FREQ:
        if (BoardData[NumBoards].nSpare5 & CLK) {
          put_user(10000000, (long *) arg);
        } else {
          put_user(1000000, (long *) arg);
        }
	break;
      case ADC_GET_FRONT_END:
        if (BoardData[NumBoards].nSpare5 & MUX) {
          put_user(16, (long *) arg);
        } else {
          put_user(8, (long *) arg);
        }
	break;
      case ADC_GET_POLARITY:
        if (BoardData[NumBoards].nSpare5 & U_B) {
          put_user('U', (long *) arg);
        } else {
          put_user('B', (long *) arg);
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

  if ( chan >= AD_CHANNELS && chan < AD_CHANNELS+DIO_PORTS ) {
    switch (cmd) {
      case DIO_WRITE:
        break;
      case DIO_READ:
       break;
      default:
        return(-EINVAL);
        break;
    }  /* end switch */
  }    /* end if */
  return 0;
}

/*****************************************************
 *                                                   *
 * mmap() service handler                            *
 *                                                   *
 *****************************************************/

static int das16jr_mmap( struct file *filePtr, struct vm_area_struct *vma ) 
{
  int board = 0;
  int chan  = 0;
  int size  = vma->vm_end - vma->vm_start;
  unsigned int minor = iminor(filePtr->f_path.dentry->d_inode);

  board = BOARD(minor);          /* get which board   */
  chan =  CHAN(minor);           /* get which channel */

  if (chan >= 0 && chan < AD_CHANNELS) {
    /* Mmap requested for ADC. Note: It does not matter for which ADC, all ADC's share the buffer.*/

    #ifdef DEBUG
    printk("%s: pcim-das16jr_mmap(): board %d chan %d.\n", ADAPTER_ID, board, chan);
    printk("pcim-das16jr_mmap(): vma->vm_start = %#lx  vma->vm_end= %#lx size = %#x offset = %#lx\n",
	   vma->vm_start, vma->vm_end, size, vma->vm_pgoff);
    #endif
    
    if(vma->vm_pgoff != 0) {     // You have to map the entire buffer.
      #ifdef DEBUG
        printk("pcim-das16jr_mmap(): The page offset has to be zero \n");
      #endif
      return(-EINVAL);
    }

    if (size > BoardData[board].buf_phy_size) {     // You cannot request more than the max buffer
      printk("pcim-das16jr_mmap(): Size = %d is too large.\n", size);
      return(-EINVAL);
    }

    vma->vm_ops = &das16jr_vops;
    vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
    vma->vm_private_data = (void *) minor;
  }
  return 0;
}

/*****************************************************
 *                                                   *
 * fault() service handler                           *
 *                                                   *
 *****************************************************/

static int das16jr_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
  unsigned char *v_addr = NULL;            // kernel virtual address to be mapped to user space
  unsigned int minor = (unsigned int) vma->vm_private_data;
  int board = 0;
  int chan = 0;
  unsigned long offset;   // offset to test for valid address

  board = BOARD(minor);
  chan = CHAN(minor);

  offset = ((unsigned long) vmf->virtual_address  - vma->vm_start) + vma->vm_pgoff * PAGE_SIZE;
  if (offset >=  BoardData[board].buf_phy_size) {
    printk("pcim-das16jr_fault: offset = %#lx  buf_phys_size = %#x\n", offset, BoardData[board].buf_phy_size);
    return VM_FAULT_SIGBUS;
  }
  
  v_addr =  (u8*) BoardData[board].buf_virt_addr + ((unsigned long) vmf->virtual_address - vma->vm_start);
  vmf->page = virt_to_page(v_addr);

  #ifdef DEBUG
    printk ("pcim-das16jr_fault mapped address %p to %p\n", vmf->virtual_address, v_addr);
  #endif
  return 0;
}

/***************************************************************************
 *
 * poll() service handler
 *
 ***************************************************************************/

static unsigned int das16jr_poll(struct file *filePtr, poll_table *wait)
{
  unsigned int minor = iminor(filePtr->f_path.dentry->d_inode);
  unsigned int mask = 0;
  int board = 0;

  board = BOARD(minor);            /* get which board   */
  
  /* tell select()/poll() to wake up and give us a call if
     das16jr_wait is awakened
  */
  poll_wait(filePtr, &das16jr_wait, wait);
 
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

static irqreturn_t das16jr_Interrupt(int irq, void *dev_id)
{
  unsigned long flags;
  BoardRec *boardRecPtr = dev_id;
  u8 intSource;
  u32 base1 = boardRecPtr->base1;
  u32 base3 = boardRecPtr->base3;
  u32 intcsr;

  spin_lock_irqsave(&das16jr_lock, flags);
 
  /* Check that interrupt came from the pci-das16jr board (needed for sharded interrupts) */
  if ((intcsr = inl(INTCSR_ADDR)) & PLX_INT) {

    /* Reset Interrupt on  PLX PCI 9052 controller  */
    intcsr |= (PLX_INTCLR | PLX_PCIINT | PLX_INTE);
    outl_p(intcsr, INTCSR_ADDR);

    intSource = inb(ADC_CONV_REG); 

    #ifdef DEBUG
        printk("Entering das16jr_Interrupt(). irq = %d  WordsToRead = %d  intSource = %#hx\n", 
              irq, boardRecPtr->WordsToRead, intSource );
    #endif
    if (intSource & EOA) {                      // End of Acaquisition
      AD_EndOfAcquisitionInt(boardRecPtr);
    } else if (intSource & FHF) {               // FIFO half full
      AD_HalfFullInt(boardRecPtr);
    } else if (intSource & EOB) {               // End of Burst
      AD_HalfFullInt(boardRecPtr);
    } else if (intSource & FNE) {               // FIFO Not Empty
      AD_NotEmptyInt(boardRecPtr);
    }

    if (boardRecPtr->WordsToRead == 0) {
      boardRecPtr->nSpare0 &= ~(INTE);
      outb(boardRecPtr->nSpare0, INTCSR_REG);
      wake_up_interruptible(&das16jr_wait);  
    }
  }

  spin_unlock_irqrestore(&das16jr_lock, flags);
  return IRQ_HANDLED;
}

static void AD_NotEmptyInt(BoardRec *boardRecPtr)
{
  u32 base2 = boardRecPtr->base2;
  u32 base3 = boardRecPtr->base3;

  #ifdef DEBUG
    printk("Entering pcim_das16jr_AD_NotEmptyInt()\n");
  #endif

  /* check for FIFO Overflow errors */
  if (inb(ADC_CONV_REG) & OVERRUN) {
    printk("pcim_das16jr_AD_NotEmptyInt: OVERRUN error\n");
    return;
  }

  do {
    *boardRecPtr->ADC_KernBuffPtr = inw(ADC_DATA_REG);
    boardRecPtr->ADC_KernBuffPtr++;
    boardRecPtr->WordsToRead--;
    if (boardRecPtr->WordsToRead == 0) break;
  } while (inb(INTCSR_REG) & FNE);
}

/***********************************************************************
 *  This routine gets called from das16jr_Interrupt() when the         *
 *  interrupt was caused by the EOA, or when the residual count has    *
 *  been read.                                                         *
 ***********************************************************************/

static void AD_EndOfAcquisitionInt(BoardRec *boardRecPtr)
{
  u32 base2 = boardRecPtr->base2;
  u32 base3 = boardRecPtr->base3;

  #ifdef DEBUG
    printk("Entering AD_EndOfAcquisitionInt(). WordsToRead = %d\n",
	   boardRecPtr->WordsToRead);
  #endif
    
  insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, boardRecPtr->WordsToRead); // uses rep insw
  boardRecPtr->ADC_KernBuffPtr += boardRecPtr->WordsToRead;
  boardRecPtr->WordsToRead = 0;
  outb(0x3, INTCSR_REG);
}

/***********************************************************************
 *  This routine gets called from das16jr_Interrupt() when the         *
 *  interrupt was caused by the FIFO going half full.  It reads half   *
 *  a FIFO's worth of data and stores it in the kernel's arrary.       *
 ***********************************************************************/

static void AD_HalfFullInt(BoardRec *boardRecPtr)
{
  u32 base2 = boardRecPtr->base2;
  u32 base3 = boardRecPtr->base3;
  u8 intSource;
  
  #ifdef DEBUG
    printk("Entering AD_HalfFullInt()\n");
  #endif

  /* check for FIFO Overflow errors */
    if (inb(ADC_CONV_REG) & OVERRUN) {
      printk("AD_HalfFullInt: OVERRUN error\n");
      goto EndOfHalfFullInt;
    }
 
    if (boardRecPtr->WordsToRead >= 2*PACKETSIZE) {
      insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, PACKETSIZE); // uses rep insw
      boardRecPtr->ADC_KernBuffPtr += PACKETSIZE;
      boardRecPtr->WordsToRead -= PACKETSIZE;
      outb((INTE | INTSEL1 | INTSEL0), INTCSR_REG);
      printk("Read only 512 samples.\n");
    } else if ((boardRecPtr->WordsToRead > PACKETSIZE) && (boardRecPtr->WordsToRead < 2*PACKETSIZE)) {
      insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, PACKETSIZE); // uses rep insw
      boardRecPtr->ADC_KernBuffPtr += PACKETSIZE;
      boardRecPtr->WordsToRead -= PACKETSIZE;
      outb((INTE | EOA_INT_SEL  |INTSEL1 | INTSEL0), INTCSR_REG);
    } else {
      insw(ADC_DATA_REG, boardRecPtr->ADC_KernBuffPtr, boardRecPtr->WordsToRead); // uses rep insw
      boardRecPtr->ADC_KernBuffPtr += boardRecPtr->WordsToRead;
      boardRecPtr->WordsToRead = 0;
      printk("Last read.\n");
    }
    
EndOfHalfFullInt:
  if (boardRecPtr->nSpare2 | BME) {   // Burst Mode
    intSource = inb(INTCSR_REG);
    intSource &= ~INT;               // clear the interrupt
    outb(intSource, INTCSR_REG);
  }
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
  product =  (long) boardRecPtr->PacerClock / boardRecPtr->ADC_freq;

  /* check for rounding error */
  error = abs(boardRecPtr->PacerClock - product*boardRecPtr->ADC_freq);
  error2 = abs(boardRecPtr->PacerClock - (product+1)*boardRecPtr->ADC_freq);
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
  boardRecPtr->ADC_freq = boardRecPtr->PacerClock / ((long)ctr1*(long)ctr2);

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
  outb_p(mask, COUNTER_CONTROL); 

  bData = (u8) (boardRecPtr->ADC_ctr2 & 0xff);
  outb_p(bData, COUNTER_2_DATA); 

  bData = (u8) (boardRecPtr->ADC_ctr2 >> 8);
  outb_p(bData, COUNTER_2_DATA); 

  mask = C1+MODE2+LSBFIRST;
  outb_p(mask, COUNTER_CONTROL); 

  bData = (boardRecPtr->ADC_ctr1 & 0xff);
  outb_p(bData, COUNTER_1_DATA); 

  bData = (boardRecPtr->ADC_ctr1 >> 8) & 0xff;
  outb_p(bData, COUNTER_1_DATA); 
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
  outb_p(mask, COUNTER_CONTROL); 

  if ( value & 0xff000000 ) {       /* load control word only */
      return;
  } else {
      /* LSB in the first byte of value */
      bData = (u8) (value & 0xff);
      outb_p(bData, COUNTER_0_DATA); 

      /* MSB in the second byte of value */
      bData = (u8) ((value >> 8) & 0xff);
      outb_p(bData, COUNTER_0_DATA); 
  }
}

static void LoadResidualCount(u16 value, BoardRec *boardRecPtr)
{
  u32 base3 = boardRecPtr->base3;
  u8 bReg;

  bReg = (u8) ((value >> 8) & 0x3);
  outb(bReg, RESIDUAL_COUNT_1);
  bReg = (u8) (value & 0xff);
  outb(bReg, RESIDUAL_COUNT_0);

  #ifdef DEBUG
    printk("LoadResidualCount: resCount = %d\n", value);
  #endif  

}

/***************************************************************************
 *
 * Turn on ADC pacer timer chip.
 *
 ***************************************************************************/

static void StartADCPacer(BoardRec *boardRecPtr)
{
  u32 base3 = boardRecPtr->base3;
  boardRecPtr->nSpare1 |=  (PS1 | PS0);   // Internal Pacer Clock
  outb(boardRecPtr->nSpare1, PACER_CLK_REG);
  udelay(10);                /* wait 10 usecs for things to settle down */

  #ifdef DEBUG
      printk("StartADCPacer: PACER_CLK_REG = %#x\n",  boardRecPtr->nSpare1);
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
  u32 base3 = boardRecPtr->base3;

  boardRecPtr->nSpare1 = inb(PACER_CLK_REG);
  boardRecPtr->nSpare1 &= ~(PS1 | PS0);
  outb(boardRecPtr->nSpare1, PACER_CLK_REG);

  mask = C2+MODE2+LSBFIRST;
  outb_p(mask, COUNTER_CONTROL);
  mask = C1+MODE2+LSBFIRST;
  outb(mask, COUNTER_CONTROL);
}


/***************************************************************************
 *
 * Set which channels read() is interested in.
 *
 ***************************************************************************/

static int SetADCChannelMux(u8 lowChan, u8 highChan, BoardRec *boardRecPtr)
{
  u32 base3 = boardRecPtr->base3;
  u8 channelMask;

  #ifdef DEBUG
    printk("SetADCChannelMux: lowChan = %d      highChan = %d\n", lowChan, highChan);
  #endif

  channelMask = (highChan << 4) | lowChan;
  outb(channelMask, MUX_REG);
  udelay(10);                          /* wait 10 usecs for things to settle down */
  return 0;
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
  u32 base2 =  boardRecPtr->base2;
  u32 base3 =  boardRecPtr->base3;
  u8  bReg;
  
  #ifdef DEBUG
      printk("Entering SoftRead().\n");
  #endif

 /* Disable everything first */
  boardRecPtr->nSpare0 = inb(INTCSR_REG);
  boardRecPtr->nSpare0 &= ~(INTE);
  outb(boardRecPtr->nSpare0, INTCSR_REG);
  outb(0x0, PACER_CLK_REG);

  while (boardRecPtr->WordsToRead) {
    outw(0x0, ADC_DATA_REG);  /* Force first conversion */

    /*
       Poll EOC for conversion.  Normally, I would use an interrupt
       handler, but the DAS16jr board is too fast, and we get
       a race condition.  Much better to poll for a single conversion
       here.
    */

    do {
      bReg = inb_p(ADC_CHAN_REG) & EOC;
    } while (bReg);
    *(boardRecPtr->ADC_KernBuffPtr++) = inw_p(ADC_DATA_REG);  /* Load into buffer */
    udelay(10);                                             /* wait 10 usecs for things to settle down! */
    boardRecPtr->WordsToRead--;

  #ifdef DEBUG
    printk("SoftRead(): value = %#x.\n", *(boardRecPtr->ADC_KernBuffPtr - 1));
  #endif
  }
  return 0;
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
  u32 base3 =  boardRecPtr->base3;
  u16 residualCnt;
  unsigned long flags;

  #ifdef DEBUG
    printk("Entering PacerRead()  count = %d.\n", chanRec->count);
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

  /* Disable everything first */
  boardRecPtr->nSpare0 = inb(INTCSR_REG);
  boardRecPtr->nSpare0 &= ~(INTE);
  outb(boardRecPtr->nSpare0, INTCSR_REG);

  // Load the residual count 

  if (chanRec->count > 2*PACKETSIZE) {
    residualCnt = chanRec->count % PACKETSIZE;
    LoadResidualCount(residualCnt, boardRecPtr);
    boardRecPtr->nSpare0 = (INTE | INTSEL1 | INTSEL0);
  } else if ((chanRec->count < (2*PACKETSIZE))  && (chanRec->count > PACKETSIZE)) {
    residualCnt = chanRec->count % (2*PACKETSIZE);
    LoadResidualCount(residualCnt, boardRecPtr);
    boardRecPtr->nSpare0 = (INTE | EOA_INT_SEL | INTSEL1 | INTSEL0);
  } else {
    residualCnt = chanRec->count % PACKETSIZE;
    LoadResidualCount(residualCnt, boardRecPtr);
    boardRecPtr->nSpare0 = (INTE | EOA_INT_SEL | INTSEL1 | INTSEL0);
  }

  if (chanRec->burstMode == TRUE) {
    boardRecPtr->nSpare2 |= BME;
  } else {
    boardRecPtr->nSpare2 &= ~BME;
  }
  
  /* Enable interrupts */
  spin_lock_irqsave(&das16jr_lock, flags);
  outb(boardRecPtr->nSpare0 , INTCSR_REG);

  if (chanRec->pacerSource == ADC_PACER_CLOCK) {
    LoadADCPacer(boardRecPtr);                /* Establish sample frequency */
  }

  boardRecPtr->nSpare2 |= CONV_EN;
  udelay(10);                         // wait 10 usecs for things to settle down

  #ifdef DEBUG
    printk("PacerRead: Enter interrupt.  nSpare0 = %#x.\n", boardRecPtr->nSpare0);
    printk("PacerRead: Enter interrupt.  nSpare1 = %#x.\n", boardRecPtr->nSpare1);
    printk("PacerRead: Enter interrupt.  nSpare2 = %#x.\n", boardRecPtr->nSpare2);
  #endif

  StartADCPacer(boardRecPtr);
  spin_unlock_irqrestore(&das16jr_lock, flags);
  outb(boardRecPtr->nSpare2, BURST_CNTL_REG);  // Begin sampling now ...

  if (chanRec->nonBlockFlag) {  /* do this one time only */
    boardRecPtr->nonBlockFile = readfile;
    return -EAGAIN;
  } else {
    wait_event_interruptible(das16jr_wait, (boardRecPtr->WordsToRead==0) != 0);  // Block in wait state 
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
