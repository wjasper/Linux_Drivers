/***************************************************************************
 Copyright (C) 2007  Warren Jasper
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


 ***************************************************************************
 *
 *   Documentation:
 *       PCI-DAS08 Manual Revision 1, January 1999, ComputerBoards Inc.
 *       125 High Street, #6, Mansfield, MA 02048
 *       (508) 261-1123 www.computerboards.com
 *       
 *       PLX Technology
 *       www.plxtech.com  PCI-9052-1 Data Book
 *
 *       Intel: 8255A-5 Programmable Peripheral Interface
 *       Order Number 231308-004
 *       www.intel.com
 *
 *
 ***************************************************************************/


/***************************************************************************
 *
 * pci-a2dc.c
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
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include "pci-das08.h"
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
static int  TrigRead(ADC_ChanRec *chan, BoardRec *boardRecPtr, struct file *readfile);
static void LoadCounter0(u32 value, u16 base2);
static void LoadCounter1(u32 value, u16 base2);
static void LoadCounter2(u32 value, u16 base2);

static int __init das08_init(void);
static void __exit das08_exit (void);
static ssize_t das08_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t das08_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int das08_open(struct inode *iNode, struct file *filePtr);
static int das08_close(struct inode *iNode, struct file *filePtr);
static int das08_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg);
static irqreturn_t das08_ReadInterrupt(int irq, void *dev_id);
static void das08_TimerHandler(unsigned long dev_id);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-DAS08  module");
MODULE_LICENSE("GPL");

module_init(das08_init);
module_exit(das08_exit);


/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/
static int MajorNumber = DEFAULT_MAJOR_DEV;          /* Major number compiled in          */
static BoardRec BoardData[MAX_BOARDS];               /* Board specific information        */
static ADC_ChanRec ChanADC[MAX_BOARDS][AD_CHANNELS]; /* ADC Channel specific information  */
static DECLARE_WAIT_QUEUE_HEAD(das08_wait);
static int NumBoards = 0;                             /* number of boards found          */
static spinlock_t das08_lock;

/***************************************************************************
 *
 *
 ***************************************************************************/
static struct cdev das08_cdev;
static struct class *das08_class;

static struct file_operations das08_fops = {
  .owner   =     THIS_MODULE,
  .read    =     das08_read,
  .write   =     das08_write,
  .ioctl   =     das08_ioctl,
  .open    =     das08_open,
  .release =     das08_close,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int das08_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void das08_remove_one(struct pci_dev *pdev);

static struct pci_device_id das08_id_tbl[] __devinitdata = {
  {
  .vendor    =         PCI_VENDOR_ID_CBOARDS,
  .device    =         PCI_DEVICE_ID_CBOARDS_DAS08,
  .subvendor =         PCI_ANY_ID,
  .subdevice =         PCI_ANY_ID,
  .class     =         0,
  .class_mask =        0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, das08_id_tbl);

static struct pci_driver das08_driver = {
  .name     =     "das08",
  .id_table =     das08_id_tbl,
  .probe    =     das08_init_one,
  .remove   =     das08_remove_one,
};


/*********************************************************************************
 *
 * Loads driver. Called when "insmod pci-das08.o" is invoked on the command line.
 *
 *********************************************************************************/

static int __init das08_init(void)
{
    int err;
  dev_t dev;

 /* Register as a device with kernel.  */
  if (MajorNumber) {
    dev = MKDEV(MajorNumber, 0);
    err = register_chrdev_region(dev, 0xff, "das08");
  } else {
    err = alloc_chrdev_region(&dev, 0, 0xff, "das08");
    MajorNumber = MAJOR(dev);
  }
  if (err < 0) {
    printk("%s: Failure to load module. Major Number = %d  error = %d\n",
	   ADAPTER_ID, MAJOR(dev), err);
    return err;
  }

  cdev_init(&das08_cdev, &das08_fops);
  das08_cdev.owner = THIS_MODULE;
  das08_cdev.ops = &das08_fops;
  err = cdev_add(&das08_cdev, dev, 0xff);
  if (err) {
    printk("%s: Error %d in registering file operations", ADAPTER_ID, err);
    return err;
  }

  /* create the class for the das08 */
  das08_class = class_create(THIS_MODULE, "das08");
  if (IS_ERR(das08_class)) {
    return PTR_ERR(das08_class);
  }

  init_waitqueue_head(&das08_wait);
  spin_lock_init(&das08_lock);

  err = pci_register_driver(&das08_driver);
  return  err;	
}


static int __devinit das08_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  int minor;
  u32 pci_interrupt;
  u16 base1;
  u16 base2;
  struct page *page;
  char name[64];

  if (NumBoards >= MAX_BOARDS) {
    printk("das08_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit a2dc.h.\n", 
	   NumBoards);
    return -ENODEV;
  }

  BoardData[NumBoards].pdev = pdev;

  /* Get PCI_BASE_ADDRESS_1 */
  base1 = BoardData[NumBoards].base1 =  (u16) pci_resource_start(pdev, 1);

  /* Get PCI_BASE_ADDRESS_2 */
  base2 = BoardData[NumBoards].base2 = (u16) pci_resource_start(pdev, 2);

  /* pci_enable_device handles IRQ routing, so it must be before request_irq */
  if (pci_enable_device(pdev))          
    goto err_out_0;

  
  /* Register interrupt handler. */

  BoardData[NumBoards].irq = pdev->irq;
  printk("das08: irq = %d\n", BoardData[NumBoards].irq);

  if (request_irq(pdev->irq, das08_ReadInterrupt, (IRQF_DISABLED | IRQF_SHARED),
		  "das08", (void *) &BoardData[NumBoards])) {
    // No free irq found! cleanup and exit
    printk("%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData[NumBoards].irq);
    goto err_out_0;
  }

  /* disable interrupts on boot up */
  pci_interrupt = inl(PCI_9052_INTERRUPT);
  pci_interrupt &= ~(INTE | PCIINT);
  outl(pci_interrupt, PCI_9052_INTERRUPT);

  /* Set all channel structures to show nothing active/open */
  for (i = 0; i < AD_CHANNELS; i++) {
    ChanADC[NumBoards][i].open = FALSE;
    ChanADC[NumBoards][i].chan = i;           /* set channel number */
  }

  /* create a device in /sys/class/das08/ */
  for (i = 0; i < AD_CHANNELS; i++) {
    minor = (NumBoards<<0x4) + i;
    sprintf(name, "ad%d_%d", NumBoards, i);
    device_create(das08_class, NULL, MKDEV(MajorNumber, minor), name);
  }

  BoardData[NumBoards].busy = FALSE;          /* board ready */
  BoardData[NumBoards].status = 0;

  /* Allocate the kernel buffers */
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

  /* Initialize the timer */
  init_timer(&(BoardData[NumBoards].das08_timer));
  BoardData[NumBoards].das08_timer.function = das08_TimerHandler;
  BoardData[NumBoards].das08_timer.data = (unsigned long) &BoardData[NumBoards];

  printk("%s: BASR0=%#x BADR1=%#x IRQ=%d.",ADAPTER_ID, base1, base2, BoardData[NumBoards].irq);
  printk(" 9/11/2007 wjasper@ncsu.edu\n");

  NumBoards++;
  return 0;

err_out_0:
  return -ENODEV;
}


/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-das08" is run on the command line.
 *
 ***************************************************************************/

void __exit das08_exit(void) 
{
  dev_t dev;

  dev = MKDEV(MajorNumber, 0);
  pci_unregister_driver(&das08_driver);
  class_destroy(das08_class);
  cdev_del(&das08_cdev);
  unregister_chrdev_region(dev, 0xff);
  printk("%s: module removed from das08_exit.\n", ADAPTER_ID);

  #ifdef DEBUG
      printk("%s: module removed.\n", ADAPTER_ID);
  #endif
}

static void das08_remove_one(struct pci_dev *pdev)
{
  struct page *page;
  int i;
  int minor;

  NumBoards--;
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

  for (i = 0; i < AD_CHANNELS; i++) {
    minor = (NumBoards<<0x4) + i;
    device_destroy(das08_class, MKDEV(MajorNumber, minor));
  }

  #ifdef DEBUG
    printk("das08_remove_one: Board #%d removed.\n", NumBoards);
  #endif
}


/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int das08_open(struct inode *iNode, struct file *filePtr)
{
  int minor = iminor(iNode);
  int board = 0;
  int chan  = 0;
  u16 base2;

  board = BOARD(minor);            /* get which board   */
  chan =  CHAN(minor);             /* get which channel */
  base2 = BoardData[board].base2;

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

    ChanADC[board][chan].open = TRUE;              /* The device is open */
    ChanADC[board][chan].mode = filePtr->f_flags;  /* set acquisition mode */

    outb_p(0x0, STATUS_REG);

    #ifdef DEBUG
        printk("%s: open(): minor %d mode %d.\n", ADAPTER_ID, minor, ChanADC[board][minor].mode);
    #endif
  }
    return 0;   
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int das08_close(struct inode *iNode, struct file *filePtr)
{
  int minor = iminor(iNode);
  int board = 0;
  int chan  = 0;

  board = BOARD(minor);            /* get which board   */
  chan =  CHAN(minor);             /* get which channel */

  if ( chan >= 0 && chan < AD_CHANNELS ) {
    ChanADC[board][chan].open = FALSE;
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

static ssize_t das08_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  int   minor;
  int   board;
  int   chan;
  u16   base2;
  u16* kernBuff;

  register int i;
  
  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = iminor(iNode);
  board = BOARD(minor);          /* get which board   */
  chan =  CHAN(minor);           /* get which channel */
  base2 = BoardData[board].base2;

  if ( BoardData[board].busy == TRUE ) {   /* if board is in use, return busy */
      return (-EBUSY);
  } else {
      BoardData[board].busy = TRUE;
  }

    #ifdef DEBUG
       printk("das1602_read: Entering function\n");
    #endif


  if ( chan >= 0 && chan < AD_CHANNELS ) {

    if ( (ChanADC[board][chan].count = count) > BoardData[board].buf_phy_size ) {
        printk("das08_read(): requesting too large a count size.\n");
        BoardData[board].busy = FALSE;
        return (-1);
    }

    /* Read */
    switch (ChanADC[board][chan].mode) {

      case ADC_SOFT_TRIGGER:
          #ifdef DEBUG
              printk("adc_read(): Entering ADC_SOFT_TRIGGER mode.\n");
          #endif
          if (SoftRead(&ChanADC[board][chan], &BoardData[board])) {
              printk("das08_read: SoftRead() failed.\n");
              BoardData[board].busy = FALSE;
              return(-1);
          } 
          break;
     
      case ADC_EXTERNAL_TRIGGER:
          #ifdef DEBUG
              printk("adc_read(): Entering ADC_EXTERNAL_TRIGGER mode.\n");
          #endif
          if (TrigRead(&ChanADC[board][chan], &BoardData[board], filePtr)) {
              printk("das08_read: TrigRead() failed.\n");
              BoardData[board].busy = FALSE;
              return(-1);
          } 
          break;
    }

    /* Check that data can be written to file. */

    if (!access_ok(VERIFY_WRITE, buf, sizeof(u16)*ChanADC[board][chan].count)) {
      printk("das08_read: Failed VERIFY_WRITE.\n");
      BoardData[board].busy = FALSE;
      return -1;
    }

    /* Write data to user space */
    kernBuff = BoardData[board].buf_virt_addr;
    for (i = 0; i < ChanADC[board][chan].count; i++) {
      *kernBuff++  >>= 4;
    }

    if (ChanADC[board][chan].count == 1) {
      put_user(*BoardData[board].buf_virt_addr, (u16*) buf);
    } else if (copy_to_user(buf, BoardData[board].buf_virt_addr, ChanADC[board][chan].count*sizeof(u16))) {
	return -EFAULT;
    }

    BoardData[board].busy = FALSE;
    return(ChanADC[board][chan].count);     /* return number of samples read */
  } else {
    printk("das08_read: Incorrect minor number (%d).\n", minor);
    BoardData[board].busy = FALSE;
    return -1;
  }
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t das08_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
    return 0; 
}

/***************************************************************************
 *
 * iotctl() service handler
 *
 ***************************************************************************/

/* Note:
    Remember that FIOCLEX, FIONCLEX, FIONBIO, and FIOASYN are reserved ioctl cmd numbers
*/

static int das08_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  int minor = iminor(iNode);
  int board = 0;
  int chan  = 0;
  int err = 0;
  int size = _IOC_SIZE(cmd);       /* the size bitfield in cmd */
  u32 pci9052_interrupt;
  u32 pci9052_select;
  u16 base1;
  u16 base2;
  u8 bReg;

  board = BOARD(minor);          /* get which board   */
  chan =  CHAN(minor);           /* get which channel */
  base1 = BoardData[board].base1;
  base2 = BoardData[board].base2;

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
  
  /* global IOCTL CALLS */

  /* ADC Specific  IOCTL CALLS */
  if ( chan >= 0 && chan < AD_CHANNELS ) {
    switch (cmd) {
      case ADC_COUNTER0:
        LoadCounter0((u32) arg, base2);
        break;
      case ADC_COUNTER1:
        LoadCounter1((u32) arg, base2);
        break;
      case ADC_COUNTER2:
        LoadCounter2((u32) arg, base2);
        break;
      case ADC_GET_STATUS:
        /* return status register */
        put_user( (long) inb_p(STATUS_REG), (long*) arg);
        break;
      case ADC_GET_DIO:
        bReg = inb_p(STATUS_REG) >> 4;
        put_user( (long) bReg, (long*) arg);
        break;
      case ADC_SET_DIO:
        bReg = (u8) (arg & 0xf);
        bReg <<= 4;
        BoardData[board].status &= 0xf;
        BoardData[board].status |= bReg;
        outb_p(BoardData[board].status, STATUS_REG);
        break;
      case INT_SELECT:
        pci9052_select = inl(PCI_9052_SOURCE_SELECT);
	if (arg == 0) {
	  pci9052_select &= ~(OUT0);
	} else {
	  pci9052_select |= (OUT0);
	}
	outl(pci9052_select,PCI_9052_SOURCE_SELECT);
	break;
      case INT_ENABLE:
        pci9052_interrupt = inl(PCI_9052_INTERRUPT);
	if (arg == 0) {
	  pci9052_interrupt &= ~(INTE | PCIINT);
	  BoardData[board].status &= ~(INTE2);
	} else {
  	  pci9052_interrupt |= (INTE | PCIINT);
	  BoardData[board].status |= (INTE2);
	}
	outl(pci9052_interrupt, PCI_9052_INTERRUPT);
	outb(BoardData[board].status, STATUS_REG);
	break;
      case SW_INTERRUPT:
	pci9052_interrupt = inl(PCI_9052_INTERRUPT);
        pci9052_interrupt |= (SW_INT);
	break;
      default:
        return(-EINVAL);
        break;
    }  /* end switch */
  }  /* end if */

  return 0;
}

/***************************************************************************
 *
 *  Alarm handler to handle situations where an interrupt is missed.
 *
 ***************************************************************************/

static void das08_TimerHandler(unsigned long dev_id)
{
  BoardRec *boardRecPtr = (BoardRec *) dev_id;
  /* 
     we should only be here if we blocked forever 
  */

  #ifdef DEBUG
      printk("TimerHandler: WordsToRead = %d\n", boardRecPtr->wordsToRead);
  #endif

  boardRecPtr->IRQComplete = 1;
  wake_up_interruptible(&das08_wait);  
}

/***************************************************************************
 *
 * Interrupt handler used to service enhanced mode(interrupt) read().
 *
 ***************************************************************************/

static irqreturn_t das08_ReadInterrupt(int irq, void *dev_id)
{
  u32 pci9052_interrupt;
  BoardRec *boardRecPtr = dev_id;
  u16 base1 = boardRecPtr->base1;
  u16 base2 = boardRecPtr->base2;

  spin_lock_irq(&das08_lock);

  pci9052_interrupt = inl(PCI_9052_INTERRUPT);
  if (pci9052_interrupt & SW_INT) {
    printk("das08_ReadInterrupt: software interrupt\n");
    pci9052_interrupt &= ~(SW_INT);
    outl(pci9052_interrupt, PCI_9052_INTERRUPT);
  }    
  
  /* Check that interrupt came from the pci-das08 board (needed for sharded interrupts) */
  if (pci9052_interrupt & INT)  {
    pci9052_interrupt |= INTCLR;
    outl(pci9052_interrupt, PCI_9052_INTERRUPT);

    #ifdef DEBUG
      printk("Entering das08_ReadInterrupt(). irq = %d  count = %d\n", 
	     irq, boardRecPtr->wordsToRead );
    #endif
  }
  spin_unlock_irq(&das08_lock);
  return IRQ_HANDLED;
}

/***************************************************************************
 *
 * Handles software read().
 *
 * Bang! Bang! Method: 
 *    
 *    o Force interrupt by forcing conversion.
 *    o Get one sample at a time and put it in the kernel buffer(kernBuff).
 *    o Get chan->count samples
 *
 ***************************************************************************/

static int  SoftRead(ADC_ChanRec *chan, BoardRec *boardRecPtr)
{
  u16 msb, lsb;
  u16 base2 = boardRecPtr->base2;

  #ifdef DEBUG
      printk("Entering SoftRead().\n");
  #endif

  /* Prepare global data for parameterless interrupt handler */

  boardRecPtr->ADC_KernBuffPtr = boardRecPtr->buf_virt_addr;
  boardRecPtr->wordsToRead = chan->count;

  outb_p(chan->chan | boardRecPtr->status, STATUS_REG);  /* Set MUX channel */

  while ( boardRecPtr->wordsToRead ) {
    outb_p(0x1, MSB_DATA_BYTE);                /* Force conversion */

    /* poll until done (Yech!) */
    while (inb_p(STATUS_REG) & EOC);

    lsb = (u16) inb(LSB_AND_CHNLS);           /* Sample: ADC -> kernel buffer */
    msb = (u16) inb(MSB_DATA_BYTE);
    *boardRecPtr->ADC_KernBuffPtr++ = (msb << 8) | lsb;
    boardRecPtr->wordsToRead--;               /* One sample at a time */
  }
  return 0;
}

static int TrigRead(ADC_ChanRec *chanRec, BoardRec* boardRecPtr, struct file *readfile)
{
  u16 base2 = boardRecPtr->base2;

  #ifdef DEBUG
      printk("Entering TrigRead().\n");
  #endif

  /* Prepare global data for parameterless interrupt handler */
  boardRecPtr->ADC_CurrChan = chanRec;                             /* pass chanel number to global */
  boardRecPtr->ADC_KernBuffPtr = boardRecPtr->buf_virt_addr;       /* same with KernBuff           */
  boardRecPtr->wordsToRead = chanRec->count;

  outb_p(chanRec->chan | boardRecPtr->status, STATUS_REG);         /* Set MUX               */
  boardRecPtr->das08_timer.expires =  jiffies + HZ;                /* just in case ...      */
  add_timer(&(boardRecPtr->das08_timer));

  boardRecPtr->IRQComplete = 0;
  wait_event_interruptible(das08_wait,(boardRecPtr->IRQComplete=1) != 0);

  if ( boardRecPtr->wordsToRead != 0 ) {
      printk("Timing error in TrigRead: WordsToRead = %d\n", boardRecPtr->wordsToRead);
      return -1;
  }
  #ifdef DEBUG
    printk("Leaving TrigRead.\n");
  #endif

  return 0;
}


/***************************************************************************
 *
 * Load value into Counter 0  XXXX    Mode    MSB     LSB
 *                            Byte 3  Byte 2  Byte 1  Byte 0
 *
 ***************************************************************************/

static void LoadCounter0( u32 value, u16 base2 )
{
  u8 mask;
  u8 bData;

  /* Write the value into Counter 0 Mode 2 */

  #ifdef DEBUG
      printk("LoadCounter0: load value %#lx into Counter 0.\n", (unsigned long) value);
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

static void LoadCounter1( u32 value, u16 base2 )
{
  u8 mask;
  u8 bData;

  /* Write the value into Counter 1 Mode 2 */

  #ifdef DEBUG
      printk("LoadCounter0: load value %#lx into Counter 0.\n", (unsigned long) value);
  #endif

  /* the mode is in the thrid byte */
  mask = 0xff & (value >> 16);
  mask += C1+LSBFIRST;
  outb_p(mask, COUNTER_CONTROL); 

  if ( value & 0xff000000 ) {       /* load control word only */
      return;
  } else {
      /* LSB in the first byte of value */
      bData = (u8) (value & 0xff);
      outb_p(bData, COUNTER_1_DATA); 

      /* MSB in the second byte of value */
      bData = (u8) ((value >> 8) & 0xff);
      outb_p(bData, COUNTER_1_DATA); 
  }
}

static void LoadCounter2( u32 value, u16 base2 )
{
  u8 mask;
  u8 bData;

  /* Write the value into Counter 2 Mode 2 */

  #ifdef DEBUG
      printk("LoadCounter0: load value %#lx into Counter 0.\n", (unsigned long) value);
  #endif

  /* the mode is in the thrid byte */
  mask = 0xff & (value >> 16);
  mask += C2+LSBFIRST;
  outb_p(mask, COUNTER_CONTROL); 

  if ( value & 0xff000000 ) {       /* load control word only */
      return;
  } else {
      /* LSB in the first byte of value */
      bData = (u8) (value & 0xff);
      outb_p(bData, COUNTER_2_DATA); 

      /* MSB in the second byte of value */
      bData = (u8) ((value >> 8) & 0xff);
      outb_p(bData, COUNTER_2_DATA); 
  }
}

