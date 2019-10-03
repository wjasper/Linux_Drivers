/***************************************************************************
 Copyright (C) 2003-2012  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-CTR05, is free software; you can redistribute it
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
    c9513.c for PCI-CTR05
*/

#ifndef __KERNEL__       
#define __KERNEL__
#endif

#ifndef MODULE	
#define MODULE
#endif

#define HIGH(x) (*((unsigned char*)&(x)+1))
#define LOW(x)  (*(unsigned char*)&(x))
#define GATEINTERVAL   5000

/* INCLUDES */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "pci-ctr05.h"
#include "c9513.h"


/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init ctr05_init(void);
static void __exit ctr05_exit (void);
static ssize_t ctr05_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t ctr05_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int ctr05_open(struct inode *iNode, struct file *filePtr);
static int ctr05_close(struct inode *iNode, struct file *filePtr);
static long ctr05_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg);
static void ctr05_TimerHandler(unsigned long unused);
static irqreturn_t ctr05_Interrupt(int irq, void *dev_id);

module_init(ctr05_init);
module_exit(ctr05_exit);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-CTR05 module");
MODULE_LICENSE("GPL");

/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/

static spinlock_t ctr05_lock;
static BoardRec BoardData[MAX_BOARDS];                /* Board specific information      */
static CounterRec  Counter[MAX_BOARDS][NCOUNTERS+1];  /* Counter specific information    */
static DIO_ChanRec ChanDIO[MAX_BOARDS];               /* DIO Channel specific information*/
static int MajorNumber = DEFAULT_MAJOR_DEV;           /* Major number compiled in        */
static DECLARE_WAIT_QUEUE_HEAD(ctr05_wait);           /* wait semaphore                  */
//static struct timer_list TimerList = {function: ctr05_TimerHandler};
static int NumBoards = 0;                             /* number of boards found          */

/***************************************************************************
 *
 *
 ***************************************************************************/
static struct cdev ctr05_cdev;
static struct class *ctr05_class;

static struct file_operations ctr05_fops = {
  .owner            =  THIS_MODULE,
  .read             =  ctr05_read,
  .write            =  ctr05_write,
  .unlocked_ioctl   =  ctr05_ioctl,
  .open             =  ctr05_open,
  .release          =  ctr05_close,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int ctr05_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void ctr05_remove_one(struct pci_dev *pdev);

static struct pci_device_id ctr05_id_tbl[] __devinitdata = {
  {
  .vendor      =  PCI_VENDOR_ID_CBOARDS,
  .device      =  PCI_DEVICE_ID_CBOARDS_CTR05,
  .subvendor   =  PCI_ANY_ID,
  .subdevice   =  PCI_ANY_ID,
  .class       =  0,
  .class_mask  =  0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, ctr05_id_tbl);

static struct pci_driver ctr05_driver = {
  .name     =  "ctr05",
  .id_table =  ctr05_id_tbl,
  .probe    =  ctr05_init_one,
  .remove   =  ctr05_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/
static int __init ctr05_init(void) 
{
  int err;
  dev_t dev;

  /* Register as a device with kernel.  */
  if (MajorNumber) {
    dev = MKDEV(MajorNumber, 0);
    err = register_chrdev_region(dev, 0xff, "ctr05");
  } else {
    err = alloc_chrdev_region(&dev, 0, 0xff, "ctr05");
    MajorNumber = MAJOR(dev);
  }
  if (err < 0) {
    printk("%s: Failure to load module. Major Number = %d  error = %d\n",
	   ADAPTER_ID, MAJOR(dev), err);
    return err;
  }

  cdev_init(&ctr05_cdev, &ctr05_fops);
  ctr05_cdev.owner = THIS_MODULE;
  ctr05_cdev.ops = &ctr05_fops;
  err = cdev_add(&ctr05_cdev, dev, 0xff);
  if (err) {
    printk("%s: Error %d in registering file operations", ADAPTER_ID, err);
    return err;
  }

  /* create the class for the ctr05 */
  ctr05_class = class_create(THIS_MODULE, "ctr05");
  if (IS_ERR(ctr05_class)) {
    return PTR_ERR(ctr05_class);
  }

  spin_lock_init(&ctr05_lock);
  err = pci_register_driver(&ctr05_driver);	

  return  err;	
}

static int __devinit ctr05_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  int minor;
  u16 base1;
  u16 base2;
  u8 bReg;
  char name[64];

  if (NumBoards >= MAX_BOARDS) {
    printk("ctr05_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit c9513.h.\n", NumBoards);
    return -ENODEV;
  }

  /*GETTING BASE ADDRESS 1 */
  base1 = BoardData[NumBoards].base1 =  (u16) pci_resource_start(pdev, 1);

  /*GETTING BASE ADDRESS 2 */
  base2 = BoardData[NumBoards].base2 = (u16) pci_resource_start(pdev, 2);

  /* Register interrupt handler. */
  BoardData[NumBoards].irq = pdev->irq;
  if (request_irq(pdev->irq, ctr05_Interrupt, (IRQF_DISABLED | IRQF_SHARED),
		  "ctr05", (void *) &BoardData[NumBoards])) {
    /* No free irq found! cleanup and exit */
    printk("%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData[NumBoards].irq);
    goto err_out_0;
  }

  if (pci_enable_device(pdev))          
    goto err_out_1;

  /* Initialize the 9513 chip */
  outb_p(MASTER_RESET, CMD_REG);      /* MASTER RESET */
  outb_p(MASTER_MODE_REG, CMD_REG);   /* Select Master Mode Register */
  bReg = TOD_DISABLED | MM_SRC1;
  outb_p(bReg, DATA_REG);
  bReg = 0xc0;        /* BCD Division, Disable increment, 8-Bit bus, FOUT On */
  outb_p(bReg, DATA_REG);

  BoardData[NumBoards].mm = BCD_DIV | DISABLE_INCREMENT | TOD_DISABLED | MM_SRC1;
  BoardData[NumBoards].gateInterval = GATEINTERVAL;
  BoardData[NumBoards].useCounter = 0;

  for ( i = 1; i <= NCOUNTERS; i++ ) {
    Counter[NumBoards][i].open = FALSE;
    Counter[NumBoards][i].count = 0;
    Counter[NumBoards][i].chan = i;
    Counter[NumBoards][i].cm = 0;
    Counter[NumBoards][i].mode = 0;
  }

    for ( i = 1; i <= NCOUNTERS; i++ ) {
    minor = (NumBoards<<0x4) + i;
    sprintf(name, "ctr05/ctr%d_0%d", NumBoards, i);
    device_create(ctr05_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  }


  ChanDIO[NumBoards].open = FALSE;
  minor = (NumBoards<<0x4) + 0;
  sprintf(name, "ctr05/dio%d_0A", NumBoards);
  device_create(ctr05_class, NULL, MKDEV(MajorNumber, minor), NULL, name);

  printk("%s: address=%#x IRQ=%d.",ADAPTER_ID, BoardData[NumBoards].base2, BoardData[NumBoards].irq);
  printk(" 8/3/2012 wjasper@ncsu.edu\n");

  NumBoards++;
  return 0;

err_out_1:
  free_irq(pdev->irq, (void *) &BoardData[NumBoards]);
err_out_0:
  return -ENODEV;
}

/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-ctr05" is run on the command line.
 *
 ***************************************************************************/
void __exit ctr05_exit(void)
{
  dev_t dev;

  dev = MKDEV(MajorNumber, 0);
  pci_unregister_driver(&ctr05_driver);
  class_destroy(ctr05_class);
  cdev_del(&ctr05_cdev);
  unregister_chrdev_region(dev, 0xff);
  printk("%s: module removed from ctr05_exit.\n", ADAPTER_ID);
}

static void ctr05_remove_one(struct pci_dev *pdev)
{
  int i;
  int minor;

  NumBoards--;
  free_irq(BoardData[NumBoards].irq, (void *) &BoardData[NumBoards]);

  for (i = 0; i <= NCOUNTERS ; i++) {
    minor = (NumBoards<<0x4) + i;
    device_destroy(ctr05_class, MKDEV(MajorNumber, minor));
  }

#ifdef DEBUG
    printk("ctr05_remove_one: Board #%d removed.\n", NumBoards);
  #endif
}     /* ctr05_remove_one() */

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int ctr05_open(struct inode *iNode, struct file *filePtr)
{
  u16 base2;
  int board = 0;
  int chan = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);
  base2 = BoardData[board].base2;

  /*
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */

  if ( (minor & 0x7) == 0 ) {                   /* DIO port */
    ChanDIO[board].open = TRUE;                 /* The device is open */
  } else {
    if ( Counter[board][chan].open == TRUE ) {
      return -EBUSY;
    }
    Counter[board][chan].open = TRUE;              /* The device is open */
    Counter[board][chan].mode = filePtr->f_flags;  /* set acquisition mode */

    /* if we are in frequency counting mode, set counter 5 busy */
    if ( Counter[board][chan].mode == CTR05_FREQUENCY ) {
      BoardData[board].useCounter++;
      Counter[board][CTR5].open = TRUE;
    }

    #ifdef DEBUG
    printk("%s: open(): minor %d mode %d\n", ADAPTER_ID, minor,
	   Counter[board][chan].mode);
    #endif
  }
  return 0;
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int ctr05_close(struct inode *iNode, struct file *filePtr)
{
  u8 bReg;
  u16 base2;
  int board = 0;
  int chan = 0;                    /* counter number 0 - 5 */
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);
  base2 = BoardData[board].base2;

  if ( (minor & 0x7) == 0 ) {              /* DIO port */
    ChanDIO[board].open = FALSE;
  } else {
    Counter[board][chan].open = FALSE;

    /* if we are in frequency counting mode, disarm counter */
    if ( Counter[board][chan].mode == CTR05_FREQUENCY ) {
      bReg = 1 << (chan-1);
      outb_p(DISARM | bReg, CMD_REG);         /* disarm  CTR minor */
      BoardData[board].useCounter--;
      if ( BoardData[board].useCounter == 0 ) {
        Counter[board][CTR5].open = FALSE;
        outb_p(DISARM | CTR5_BIT, CMD_REG);     /* disarm CTR5  */
      }
    }
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

static ssize_t ctr05_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  int minor;
  int board = 0;
  int chan  = 0;
  u16 data;
  u16 base2;
  u8 bReg;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = iminor(iNode);
  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);
  base2 = BoardData[board].base2;

  if ( (minor & 0x7) == 0 ) {              /* DIO port */
    bReg = inb(DIO_IN);
    put_user(bReg, (u8*) buf);
    return 1;
  }

  if ( chan > 0 && chan <= NCOUNTERS ) {
    switch (Counter[board][chan].mode) {
      case CTR05_COUNTER:    /* read the current count from the counter */
        /* Disarm and Save counters   */
	outb((DISARM_LATCH | 1<<(chan-1)), CMD_REG);

	/* Read the Hold Register */
        outb((chan | HOLD_REG), CMD_REG);       /* Fetch Hold Reg */
	data = inb_p(DATA_REG) & 0xff;           /* Low Byte */
        data |= (inb_p(DATA_REG) << 8) & 0xff00; /* High Byte */
	
        /* Write data to user space */
        put_user(data, (u16*) buf);
        #ifdef DEBUG
          printk("ctr05_read Counter : minor = %d data = %d\n", minor, data);
        #endif

        /* BoardData.busy = FALSE; */
        break;
       
    case CTR05_FREQUENCY:
        /* Read the Hold Register */
        outb(HOLD_REG | chan, CMD_REG );
        data  = inb_p(DATA_REG) & 0xff;
        data |= (inb_p(DATA_REG) << 8) & 0xff00;

	#ifdef DEBUG
	   printk("read-Frequency read is %x.\n",data);
	#endif

	/* Write data to user space */
        put_user(data, (u16*) buf);
        break;

      default:
        break;
    }
    return 1;     /* return 1 16 bit counter read */
  }
  return 0;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t ctr05_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  int minor;
  int board = 0;
  int chan  = 0;
  u16 base2;
  u16 wReg;
  u8 bReg;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = iminor(iNode); 
  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);
  base2 = BoardData[board].base2;

#ifdef DEBUG
  printk("ctr05_write: minor = %d\n",minor);
#endif

  if ( (minor & 0x7) == 0 ) {              /* DIO port */
    get_user(bReg, (u8*)buf);
    outb( bReg, DIO_OUT);
    return 1;
  }

  /* write to LOAD register */
  if ( chan > 0 && chan <= NCOUNTERS ) {  
    switch (Counter[board][chan].mode) {
      case CTR05_COUNTER:    // set mode D for Count #chan 
        outb_p(MODE_REG | chan, CMD_REG);
        outb_p(COUNTUP | RECYCLE, DATA_REG);
        get_user(wReg, (u16*) buf);
        bReg = wReg >> 8;
        outb_p(bReg, DATA_REG);       /* SRC #chan */

        /* Select the desired LOAD Register */
        bReg = chan | LOAD_REG;
        outb_p (bReg, CMD_REG);
      
        /* Load the desired data into the register */
        bReg = 0;
        outb_p (bReg, DATA_REG);    /* Low Byte */
        outb_p (bReg, DATA_REG);    /* High Byte */

        /* LOAD and ARM the counter now */
        bReg = LOAD_ARM | (1 << (chan - 1));
        outb_p (bReg, CMD_REG);
        break;
      
      case CTR05_FREQUENCY:
        bReg = 1 << (chan-1);
        outb_p(DISARM | CTR5_BIT | bReg, CMD_REG);     /* disarm CTR5 & CTR minor */

        /* below sets mode J for CTR5 */
        outb_p(MODE_REG | CTR5, CMD_REG); /* Select counter 5                              */
        outb_p( 0x62, DATA_REG);          /* Disable spec. gate, Reload from load or hold, */
                                          /* Count repeat, binary, Count down, TC toggled  */
        outb_p(0xe, DATA_REG);            /* No gating, Count on rising edge, Freq4 input  */

        outb_p(LOAD_REG | CTR5, CMD_REG); /* Setting up a variable duty pulse on CTR5      */
        outb_p(5, DATA_REG);              /* the first "half" is 5 counts (from LOAD reg)  */
        outb_p(0, DATA_REG);              /* the second "half" is "GateInterval" counts    */
                                          /* load a 5 count delay into CTR5 load reg       */

        /* load gate interval into CTR5 hold reg */
        outb_p(HOLD_REG | CTR5, CMD_REG); 
        outb_p(BoardData[board].gateInterval, DATA_REG);
        outb_p(BoardData[board].gateInterval >> 8, DATA_REG);

        /* set Mode Q for Counter #chan */
        outb_p(MODE_REG | chan, CMD_REG);
        outb_p(COUNTUP | SPECIALGATE |RECYCLE, DATA_REG);
        get_user(wReg, (u16*)buf);
        bReg = (wReg | AHLGATE) >> 8;
        outb_p(bReg , DATA_REG); 
      
        /* Load 0 into CTR minor's Load Reg. */
        outb_p(LOAD_REG | chan, CMD_REG);
        outb_p(0, DATA_REG);
        outb_p(0, DATA_REG);
      
        /*
  	NOTE: To setup a 32 bit counter, use CTR3 as the gate.
	Setup CTR3 as CTR4 as setup above.  Then setup CTR4 to use Mode E with
	it's input from SigSource (as for CTR5 above) and CTR3 as the gate.
	Finally setup CTR5 using Mode B, but with its source as TCN-1 (the TC
	output from CTR4 which will pulse each time CTR4 wraps) and also gated
	by CTR3.  When done, CTR4 holds the low 16 bits and CTR5 holds the
	high 16 bits.
      */
      
        bReg = 1 << (chan -1);
        outb_p(LOAD | CTR5_BIT | bReg, CMD_REG);      /* Load CTR5 & CTR minor */
        outb_p(CLEAR_OUTPUT | CTR5, CMD_REG);         /* set TC Low            */
        outb_p (ARM | CTR5_BIT | bReg, CMD_REG);      /* Go ...                */
      
        /* Load 1 into CTR minor's Load Reg. */
        outb_p(LOAD_REG | chan, CMD_REG);
        outb_p(1, DATA_REG);
        outb_p(0, DATA_REG);
        break;
      default:
        break;
    }
    return 1;
  }
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

static long ctr05_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  struct inode *iNode = filePtr->f_dentry->d_inode;
  int minor = iminor(iNode);
  int board = 0;
  int chan  = 0;
  u32 pci9052_status;
  u16 base2;
  u8 *ptr = (u8*) &arg;
  u8 bReg;
  int size = _IOC_SIZE(cmd);       /* the size bitfield in cmd */
  int err = 0;

  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);
  base2 = BoardData[board].base2;
  
  #ifdef DEBUG
     printk("ctr05_ioctl: minor = %d\n", minor);
  #endif

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

  if ( chan > 0 && chan <= NCOUNTERS ) {
    switch (cmd) {
      case LOAD_MASTER_MODE_REG:
        outb_p(MASTER_MODE_REG, CMD_REG);        /* Select Master Mode Register */
        outb_p(*(ptr++), DATA_REG);
        outb_p(*ptr, DATA_REG);
        BoardData[board].mm = (u16) arg;
        break;
      case LOAD_ALARM_REG1:
        outb_p(ALARM_REG1, CMD_REG);
        outb_p(*(ptr++), DATA_REG);
        outb_p(*ptr, DATA_REG);
        break;
      case LOAD_ALARM_REG2:
        outb_p(ALARM_REG2, CMD_REG);
        outb_p(*(ptr++), DATA_REG);
        outb_p(*ptr, DATA_REG);
        break;
      case LOAD_MODE_REG:
        outb_p(chan|MODE_REG, CMD_REG);         /* Select Counter Mode Register */
        outb_p(*(ptr++), DATA_REG);
        outb_p(*ptr, DATA_REG);
        Counter[board][chan].cm = (u16) arg;
        break;
      case LOAD_LOAD_REG:
	outb_p(chan|LOAD_REG, CMD_REG);
        outb_p(*(ptr++), DATA_REG);
        outb_p(*ptr, DATA_REG);
        break;
      case LOAD_HOLD_REG:
	outb_p(chan|HOLD_REG, CMD_REG);
        outb_p(*(ptr++), DATA_REG);
        outb_p(*ptr, DATA_REG);
        break;
      case LOAD_CMD_REG:
        outb_p( (u8) arg, CMD_REG);
        break;
      case GET_MASTER_MODE_REG:
        put_user((u16) BoardData[board].mm, (u16*) arg);
        break;
      case GET_COUNTER_MODE_REG:
        put_user((u16) Counter[board][chan].cm, (u16*) arg);
        break;
      case GET_STATUS_REG:
        bReg = inb(CMD_REG);
        put_user((u8) bReg, (u8*) arg);
        break;
      case SET_GATE_INTERVAL:
        BoardData[board].gateInterval = (u16) arg;
        break;
      case SET_CLOCK_INPUT:
        pci9052_status = inl(STATUS_REG);
        pci9052_status &= ~(OUT0);
        pci9052_status |= arg & OUT0;
	outl(pci9052_status, STATUS_REG);
        break;
      case SET_SQUARE_FREQ:
        bReg = 1<<(chan-1);
        outb_p(DISARM | bReg, CMD_REG);            // disarm counter         
        outb_p(MODE_REG | chan, CMD_REG);          // select mode register
	if (arg  > 100) {
          outb_p(0x62, DATA_REG);                  // Disable spec. gate, Reload from load or hold, 
                                                   // Count repeat, binary, Count down, TC toggled
          outb_p(0x0b, DATA_REG);                  // No gating, Count on rising edge, Freq1 input
          arg = 2500000 / arg;                     // convert Hz to time
        } else {
          outb_p(0x62, DATA_REG);                  // Disable spec. gate, Reload from load or hold, 
                                                   // Count repeat, binary, Count down, TC toggled
          outb_p(0x0e, DATA_REG);                  // No gating, Count on rising edge, Freq4 input
          arg = 2500 / arg;                        // convert Hz to time
        }
        outb_p(LOAD_REG | chan, CMD_REG);          // Setting up a variable duty pulse on CTR5
        outb_p(*(ptr), DATA_REG);                  // first half of duty cycle
	outb_p(*(ptr+1), DATA_REG);             
        outb_p(HOLD_REG | chan, CMD_REG);          // Setting up a variable duty pulse on CTR5
        outb_p(*(ptr), DATA_REG);                  // second half of duty cycle
	outb_p(*(ptr+1), DATA_REG);             
        outb_p(LOAD | bReg, CMD_REG);           
        outb_p(CLEAR_OUTPUT | chan, CMD_REG);      // set TC Low
        outb_p (ARM | bReg, CMD_REG);              // Go ...    
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

static void ctr05_TimerHandler( unsigned long unused)
{
}

/***************************************************************************
 *
 * Interrupt handler.
 *
 ***************************************************************************/

static irqreturn_t ctr05_Interrupt(int irq, void *dev_id)
{
  BoardRec *boardData = dev_id;
  u32 pci9052_intreg;
  u16 base1 = boardData->base1;

  spin_lock_irq(&ctr05_lock);

  #ifdef DEBUG
      printk("Entering ctr05_ReadInterrupt(). irq = %d\n", irq );
  #endif

  pci9052_intreg = inl(INTERRUPT_REG);
  if ( pci9052_intreg & INT ) {    // we got interrupted
    outl( pci9052_intreg | INTCLR, INTERRUPT_REG);
  }

  spin_unlock_irq(&ctr05_lock);
  return IRQ_HANDLED;
}

