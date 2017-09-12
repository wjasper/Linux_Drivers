/***************************************************************************
 Copyright (C) 2013-2015  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-QUAD04, is free software; you can redistribute it
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
 *  Documentation:
 *
 *  Register map for the PCI-QUAD04 Revision 1.0 June 2005
 *  Measurement Computing  www.measurementcomputing.com
 *
 *  LSI/CSI LS7266R1 24-Bit Dual-Axis Quadrature Counter, LSI
 * Comptuer Systems, Inc.  1235 Walt Whitman Road, Melville, NC 11747
 *
 *  82C59A CMOS Priority Interrupt Controller, Harris Semiconductor,
 *  March 1997, File Number 2784.2
 *
 */

/***************************************************************************
 *
 * quad04.c for PCI-QUAD04
 *
 ***************************************************************************/


#ifndef __KERNEL__       
#define __KERNEL__
#endif

#ifndef MODULE	
#define MODULE
#endif

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
#include <linux/pci.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "pci-quad04.h"
#include "quad04.h"

/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init quad04_init(void);
static void __exit quad04_exit (void);
static ssize_t quad04_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t quad04_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int quad04_open(struct inode *iNode, struct file *filePtr);
static int quad04_close(struct inode *iNode, struct file *filePtr);
static long quad04_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg);

module_init(quad04_init);
module_exit(quad04_exit);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-QUAD04 module");
MODULE_LICENSE("GPL");

/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/
static spinlock_t quad04_lock;
static BoardRec BoardData[MAX_BOARDS];                /* Board specific information      */
static CounterRec  Counter[MAX_BOARDS][NCOUNTERS];    /* Counter specific information    */
static int MajorNumber = DEFAULT_MAJOR_DEV;           /* Major number compiled in        */
static int NumBoards = 0;                             /* number of boards found          */

/***************************************************************************
 *
 *
 ***************************************************************************/
static struct cdev quad04_cdev;
static struct class *quad04_class;

static struct file_operations quad04_fops = {
  .owner          = THIS_MODULE,
  .read           = quad04_read,
  .write          = quad04_write,
  .unlocked_ioctl = quad04_ioctl,
  .open           = quad04_open,
  .release        = quad04_close,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int quad04_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void quad04_remove_one(struct pci_dev *pdev);

static struct pci_device_id quad04_id_tbl[] = {
  {
  .vendor      =  PCI_VENDOR_ID_CBOARDS,
  .device      =  PCI_DEVICE_ID_CBOARDS_QUAD04,
  .subvendor   =  PCI_ANY_ID,
  .subdevice   =  PCI_ANY_ID,
  .class       =  0,
  .class_mask  =  0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, quad04_id_tbl);

static struct pci_driver quad04_driver = {
  .name     =  "quad04",
  .id_table =  quad04_id_tbl,
  .probe    =  quad04_init_one,
  .remove   =  quad04_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/

static int __init quad04_init(void) 
{
  int err;
  dev_t dev;

  /* Register as a device with kernel.  */
  if (MajorNumber) {
    dev = MKDEV(MajorNumber, 0);
    err = register_chrdev_region(dev, 0xff, "quad04");
  } else {
    err = alloc_chrdev_region(&dev, 0, 0xff, "quad04");
    MajorNumber = MAJOR(dev);
  }
  if (err < 0) {
    printk("%s: Failure to load module. Major Number = %d  error = %d\n",
	   ADAPTER_ID, MAJOR(dev), err);
    return err;
  }

  cdev_init(&quad04_cdev, &quad04_fops);
  quad04_cdev.owner = THIS_MODULE;
  quad04_cdev.ops = &quad04_fops;
  err = cdev_add(&quad04_cdev, dev, 0xff);
  if (err) {
    printk("%s: Error %d in registering file operations", ADAPTER_ID, err);
    return err;
  }

  /* create the class for the quad04 */
  quad04_class = class_create(THIS_MODULE, "quad04");
  if (IS_ERR(quad04_class)) {
    return PTR_ERR(quad04_class);
  }

  spin_lock_init(&quad04_lock);
  err = pci_register_driver(&quad04_driver);	

  return  err;	
}
static int quad04_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  int minor;
  u16 base1;
  u16 base2;
  u8 data[3];
  char name[64];
  u32 pci9052_intreg;

  if (NumBoards >= MAX_BOARDS) {
    printk("quad04_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit quad04.h.\n", NumBoards);
    return -ENODEV;
  }

  /*GETTING BASE ADDRESS 1 */
  base1 = BoardData[NumBoards].base1 =  (u16) pci_resource_start(pdev, 1);

  /*GETTING BASE ADDRESS 2 */
  base2 = BoardData[NumBoards].base2 = (u16) pci_resource_start(pdev, 2);

  /* Register interrupt handler. */
  BoardData[NumBoards].irq = pdev->irq;

  if (pci_enable_device(pdev))          
    goto err_out_0;

  /* Turn off interrupts */
  pci9052_intreg = inl(INTCSR_REG);
  pci9052_intreg &= ~(INTE | PCINT);
  outl(pci9052_intreg, INTCSR_REG);
  pci9052_intreg = inl(INTCSR_REG);
  #ifdef DEBUG
  printk("quad04_init_one(): pci9052_intreg = %#x\n", pci9052_intreg);
  #endif
  outl(pci9052_intreg | INTCLR, INTCSR_REG);
  
  /* Initialize the LS7266R1 chip */
  BoardData[NumBoards].mode = CONFIG_0;  // (4) 24-bit counters (1/2/3/4) (default)
  BoardData[NumBoards].pic_A = 0x0;
  BoardData[NumBoards].pic_B = 0x0;
  outb(BoardData[NumBoards].pic_A, PIC_A_REG);
  outb(BoardData[NumBoards].pic_B, PIC_B_REG);
  
  outb_p(CONFIG_0, ISCR_REG);          // default to 4 24 bit counters

  for (i = 0; i < NCOUNTERS; i++) {
    minor = (NumBoards << 0x3) + i;
    sprintf(name, "quad04/channel%d_%d", NumBoards, i+1);
    device_create(quad04_class, NULL, MKDEV(MajorNumber, minor), NULL, name);

    Counter[NumBoards][i].open = FALSE;
    Counter[NumBoards][i].data_port = DATA1_REG + 2*i;
    Counter[NumBoards][i].cmd_port =  CMD1_REG + 2*i;
    Counter[NumBoards][i].counter = i;

    /* Reset/Load Register */
    outb(RESET_CNTR, Counter[NumBoards][i].cmd_port);
    outb(RESET_FLAGS, Counter[NumBoards][i].cmd_port);
    outb(RESET_E, Counter[NumBoards][i].cmd_port);
    outb(RESET_BP, Counter[NumBoards][i].cmd_port);
    
    /* Be sure the prescaler is setup */
    outb(1, Counter[NumBoards][i].data_port);
    outb(TRAN_PR_PSC, Counter[NumBoards][i].cmd_port);
    outb(RESET_BP, Counter[NumBoards][i].cmd_port);

    /* Set Counter Mode to Normal, Binary, QuadX1 */
    Counter[NumBoards][i].cmr = (BIN | X1);
    outb(Counter[NumBoards][i].cmr, Counter[NumBoards][i].cmd_port);

    /* Enable Counter Inputs */
    Counter[NumBoards][i].ior = (ENABLE_AB | LOL | RCNTR);
    outb(Counter[NumBoards][i].ior, Counter[NumBoards][i].cmd_port);

    /* Enable Index */
    Counter[NumBoards][i].idr = (DISABLE_INDEX | LCNTRL_INDEX);
    outb(Counter[NumBoards][i].idr, Counter[NumBoards][i].cmd_port);

    /* Load Counter Preset */
    outb(RESET_BP, Counter[NumBoards][i].cmd_port);
    outb(0x12, Counter[NumBoards][i].data_port);
    outb(0x34, Counter[NumBoards][i].data_port);
    outb(0x56, Counter[NumBoards][i].data_port);

    /* Transfer the loaded value to the counter */
    outb(TRAN_PR_CNTR, Counter[NumBoards][i].cmd_port);

    /* Transfer counter to latch */
    outb(TRAN_CNTR_OL, Counter[NumBoards][i].cmd_port);
    outb(RESET_BP, Counter[NumBoards][i].cmd_port);

    /* Now read back the data from the output latch */
    data[0] = inb_p(Counter[NumBoards][i].data_port);
    data[1] = inb_p(Counter[NumBoards][i].data_port);
    data[2] = inb_p(Counter[NumBoards][i].data_port);

    #ifdef DEBUG
    printk("%s: Counter %d: test %#2x %#2x %#2x\n", ADAPTER_ID, i+1, data[2], data[1], data[0]);
    #endif

    /* Clear the preset */
    outb(RESET_BP, Counter[NumBoards][i].cmd_port);
    outb(0x0, Counter[NumBoards][i].data_port);
    outb(0x0, Counter[NumBoards][i].data_port);
    outb(0x0, Counter[NumBoards][i].data_port);

    /* Reset all fags */
    outb(RESET_CNTR, Counter[NumBoards][i].cmd_port);
    outb(RESET_FLAGS, Counter[NumBoards][i].cmd_port);
    outb(RESET_E, Counter[NumBoards][i].cmd_port);
    outb(RESET_BP, Counter[NumBoards][i].cmd_port);
  }

  printk("%s: address=%#x IRQ=%d.",ADAPTER_ID, BoardData[NumBoards].base2, BoardData[NumBoards].irq);
  printk(" 8/8/2015 wjasper@ncsu.edu\n");

  NumBoards++;
  return 0;

err_out_0:
  return -ENODEV;
}

/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-quad04" is run on the command line.
 *
 ***************************************************************************/
void __exit quad04_exit(void)
{
  dev_t dev;

  dev = MKDEV(MajorNumber, 0);
  pci_unregister_driver(&quad04_driver);
  class_destroy(quad04_class);
  cdev_del(&quad04_cdev);
  unregister_chrdev_region(dev, 0xff);
  printk("%s: module removed from quad04_exit.\n", ADAPTER_ID);
}

static void quad04_remove_one(struct pci_dev *pdev)
{
  int i;
  int minor;

  NumBoards--;
  //  free_irq(BoardData[NumBoards].irq, (void *) &BoardData[NumBoards]);

  for (i = 0; i < NCOUNTERS ; i++) {
    minor = (NumBoards<<0x3) + i;
    device_destroy(quad04_class, MKDEV(MajorNumber, minor));
  }

#ifdef DEBUG
    printk("quad04_remove_one: Board #%d removed.\n", NumBoards);
#endif
}     /* quad04_remove_one() */

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int quad04_open(struct inode *iNode, struct file *filePtr)
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
     channel at a time.  There is still the possibility of two processes
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */
  
  if (Counter[board][chan].open == TRUE) {
      return -EBUSY;
  }

  Counter[board][chan].open = TRUE;              /* The device is open */

  /* Clear the preset */
  outb(RESET_BP, Counter[board][chan].cmd_port);
  outb(0x0, Counter[board][chan].data_port);
  outb(0x0, Counter[board][chan].data_port);
  outb(0x0, Counter[board][chan].data_port);

  /* Reset all fags */
  outb(RESET_CNTR, Counter[board][chan].cmd_port);
  outb(RESET_FLAGS, Counter[board][chan].cmd_port);
  outb(RESET_E, Counter[board][chan].cmd_port);
  outb(RESET_BP, Counter[board][chan].cmd_port);
  
  #ifdef DEBUG
  printk("%s: open(): minor %d \n", ADAPTER_ID, minor);
  #endif

  return 0;
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int quad04_close(struct inode *iNode, struct file *filePtr)
{
  u16 base2;
  int board = 0;
  int chan = 0;                    /* counter number 0 - 3 */
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);
  base2 = BoardData[board].base2;

  Counter[board][chan].open = FALSE;

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

static ssize_t quad04_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  int minor;
  int board = 0;
  int chan  = 0;
  int i;
  u16 base2;
  u8 data[12];    // counter could be 24 bit to 96 bits depending upon mode
    
  struct inode *iNode = filePtr->f_path.dentry->d_inode;
  minor = iminor(iNode);
  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);              /* which channel     */
  base2 = BoardData[board].base2;

#ifdef DEBUG
  printk("quad04_read(): mode = %#x  channel = %d  board = %d\n", BoardData[board].mode, chan, board);
#endif

  switch(BoardData[board].mode) {

    case CONFIG_0:   // (4) 24-Bit counters  1/2/3/4
      outb_p((TRAN_CNTR_OL | RESET_BP), Counter[board][chan].cmd_port);  // transfer counter to output latch
      for (i = 0; i < 3; i++) {
	data[i] = inb_p(Counter[board][chan].data_port);
      }
      #ifdef DEBUG
        printk("quad04_read(): data = %#x %#x %#x\n", data[2], data[1], data[0]);
      #endif
      if (copy_to_user(buf, data, 3)) return -EFAULT;
      return 3;
      break;  

    case CONFIG_1:  // (2) 48-Bit counters  1-2/3-4
      if (chan == 0 || chan == 1) {
	outb((TRAN_CNTR_OL | RESET_BP), CMD1_REG);
	for (i = 0; i < 3; i++) {
	  data[i] = inb_p(DATA1_REG);
	}
	outb((TRAN_CNTR_OL | RESET_BP), CMD2_REG);
	for (i = 3; i < 6; i++) {
	  data[i] = inb_p(DATA2_REG);
	}
      } else {
	outb((TRAN_CNTR_OL | RESET_BP), CMD3_REG);
	for (i = 0; i < 3; i++) {
	  data[i] = inb_p(DATA3_REG);
	}
	outb((TRAN_CNTR_OL | RESET_BP), CMD4_REG);
	for (i = 3; i < 6; i++) {
	  data[i] = inb_p(DATA4_REG);
	}
      }
      if (copy_to_user(buf, data, 6)) return -EFAULT;
      return 6;
      break;  

    case CONFIG_2: // (1) 24-bit counter,  (1) 72-bit counter  1/2-3-4
      if (chan == 0) {
	outb((TRAN_CNTR_OL | RESET_BP), CMD1_REG);
	for (i = 0; i < 3; i++) {
	  data[i] = inb(DATA1_REG);
	}
	if (copy_to_user(buf, data, 3)) return -EFAULT;
	return 3;
      } else {
	outb((TRAN_CNTR_OL | RESET_BP), CMD2_REG);
	for (i = 0; i < 3; i++) {
	  data[i] = inb(DATA2_REG);
	}
	outb(RESET_BP, CMD3_REG);
	for (i = 3; i < 6; i++) {
	  data[i] = inb(DATA3_REG);
	}
	outb(RESET_BP, CMD4_REG);
	for (i = 6; i < 8; i++) {
	  data[i] = inb(DATA4_REG);
	}
	if (copy_to_user(buf, data, 9)) return -EFAULT;
	return 9;	
      }
      break;  

    case CONFIG_3: // (1) 96-bit counter   1-2-3-4
      outb((TRAN_CNTR_OL | RESET_BP), CMD1_REG);
      for (i = 0; i < 3; i++) {
	data[i] = inb(DATA1_REG);
      }
      outb((TRAN_CNTR_OL | RESET_BP), CMD2_REG);
      for (i = 3; i < 6; i++) {
	data[i] = inb(DATA2_REG);
      }
      outb((TRAN_CNTR_OL | RESET_BP), CMD3_REG);
      for (i = 6; i < 9; i++) {
	data[i] = inb(DATA3_REG);
      }
      outb((TRAN_CNTR_OL | RESET_BP), CMD4_REG);
      for (i = 9; i < 12; i++) {
	data[i] = inb(DATA4_REG);
      }
      if (copy_to_user(buf, data, 12)) return -EFAULT;
      return 12;
      break;  
  }
  return 0;
}


/***************************************************************************
 *
 * write() service function (loads the preset register)
 *
 ***************************************************************************/

static ssize_t quad04_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  int minor;
  int board = 0;
  int chan  = 0;
  int i;
  u16 base2;
  u8 data[12];    // preset values could be 24 bit to 96 bits depending upon mode

  struct inode *iNode = filePtr->f_path.dentry->d_inode;
  minor = iminor(iNode); 
  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);
  base2 = BoardData[board].base2;

  #ifdef DEBUG
    printk("quad04_write: minor = %d\n",minor);
  #endif


  switch(BoardData[board].mode) {
    case CONFIG_0:
      if (copy_from_user(data, buf, 3)) return -EFAULT;
      outb_p(RESET_BP, Counter[board][chan].cmd_port);  // reset the base pointer
      outb_p(data[0], Counter[board][chan].data_port);    // least significant byte first
      outb_p(data[1], Counter[board][chan].data_port);    // 
      outb_p(data[2], Counter[board][chan].data_port);    // most significant byte
      return 3;
      break;

    case CONFIG_1:
      if (copy_from_user(data, buf, 6)) return -EFAULT;
      if (chan == 0 || chan == 1) {
	outb(RESET_BP, CMD1_REG);
	for (i = 0; i < 3; i++) {
	  outb_p(data[i], DATA1_REG);
	}
	outb(RESET_BP, CMD2_REG);
	for (i = 3; i < 6; i++) {
	  outb_p(data[i], DATA2_REG);
	}
	return 6;
      } else {
	outb(RESET_BP, CMD3_REG);
	for (i = 0; i < 3; i++) {
	  outb_p(data[i], DATA3_REG);
	}
	outb(RESET_BP, CMD4_REG);
	for (i = 3; i < 6; i++) {
	  outb_p(data[i], DATA4_REG);
	}
	return 6;
      }
      break;  

    case CONFIG_2:
      if (chan == 0) {
	if (copy_from_user(data, buf, 3)) return -EFAULT;
	outb(RESET_BP, CMD1_REG);
	for (i = 0; i < 3; i++) {
	  outb_p(data[i], DATA1_REG);
	}
	return 3;
      } else {
	if (copy_from_user(data, buf, 9)) return -EFAULT;
	outb(RESET_BP, CMD2_REG);
	for (i = 0; i < 3; i++) {
	  outb_p(data[i], DATA2_REG);
	}
	outb(RESET_BP, CMD3_REG);
	for (i = 3; i < 6; i++) {
	  outb_p(data[i], DATA3_REG);
	}
	outb(RESET_BP, CMD4_REG);
	for (i = 6; i < 9; i++) {
	  outb_p(data[i], DATA4_REG);
	}
	return 9;	
      }
      break;  

    case CONFIG_3:
      if (copy_from_user(data, buf, 12)) return -EFAULT;
      outb(RESET_BP, CMD1_REG);
      for (i = 0; i < 3; i++) {
	outb_p(data[i], DATA1_REG);
      }
      outb(RESET_BP, CMD2_REG);
      for (i = 3; i < 6; i++) {
	outb_p(data[i], DATA2_REG);
      }
      outb(RESET_BP, CMD3_REG);
      for (i = 6; i < 9; i++) {
	outb_p(data[i], DATA3_REG);
      }
      outb(RESET_BP, CMD4_REG);
      for (i = 9; i < 12; i++) {
	outb_p(data[i], DATA4_REG);
      }
      return 12;
      break;  
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

static long quad04_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  struct inode *iNode = filePtr->f_path.dentry->d_inode;
  int minor = iminor(iNode);
  int board = 0;
  int chan  = 0;
  u16 base2;
  u8 bReg;
  int size = _IOC_SIZE(cmd);       /* the size bitfield in cmd */
  int err = 0;

  board = BOARD(minor);            /* get which board   */
  chan = CHAN(minor);              /* get which channel */
  base2 = BoardData[board].base2;
  
  #ifdef DEBUG
     printk("quad04_ioctl: minor = %d\n", minor);
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

  if (chan >= 0 && chan < NCOUNTERS) {
    switch (cmd) {
      case LOAD_CMD_REG:
        outb((u8) arg, Counter[board][chan].cmd_port);
        #ifdef DEBUG
	printk("quad04_ioctl(): LOAD_CHAN_REG chan = %d  value = %#x\n", chan, (unsigned int) arg);
	#endif
        break;
      case GET_CMD_REG:
        bReg = inb_p(Counter[board][chan].cmd_port);
	put_user((u8) bReg, (u8*) arg);
	break;
      case LOAD_IRCR_REG:
        outb_p((u8) arg, IRCR_REG);
        break;
      case GET_IRCR_REG:
        bReg = inb_p(IRCR_REG);
	put_user((u8) bReg, (u8*) arg);
        break;
      case LOAD_ISCR_REG:
	outb_p((u8) arg, ISCR_REG);
        break;
      case GET_ISCR_REG:
        bReg = inb_p(ISCR_REG);
	put_user((u8) bReg, (u8*) arg);
        break;
      case LOAD_PIC_A_REG:
	outb_p((u8) arg, PIC_A_REG);
        break;
      case LOAD_PIC_B_REG:
	outb_p((u8) arg, PIC_B_REG);
        break;
    }  /* end switch */
  }  /* end if */
  return 0;
}
