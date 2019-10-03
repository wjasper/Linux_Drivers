/***************************************************************************
 Copyright (C) 2012  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DDA0X-16, is free software; you can redistribute it
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
 *
 *   Documentation:
 *       Register Map for the PCI-DDA0x/16 Series
 *       Document revison 1, June 2005
 *       (508) 946-5100  www.mccdaq.com
 *       
 *       PLX Technology
 *       www.plxtech.com  PCI-9052-1 Data Book
 *
 *       Intel: 8255A-5 Programmable Peripheral Interface
 *       Order Number 231308-004
 *       www.intel.com
 *
 */

/***************************************************************************
 *
 * dda0X-16_2_6.c
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
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/sysfs.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "dda0X-16.h"
#include "pci-dda0X-16.h"

/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init dda0X_16_init(void);
static void __exit dda0X_16_exit (void);
static ssize_t dda0X_16_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t dda0X_16_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int dda0X_16_open(struct inode *iNode, struct file *filePtr);
static int dda0X_16_close(struct inode *iNode, struct file *filePtr);
static long dda0X_16_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-DDA0X_16  module");
MODULE_LICENSE("GPL");

module_init(dda0X_16_init);
module_exit(dda0X_16_exit);

/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/

static int MajorNumber = DEFAULT_MAJOR_DEV;         /* Major number compiled in     */
static BoardRec BoardData[MAX_BOARDS];              /* Board specific information   */
static ChanRec_DIO Chan_DIO[MAX_BOARDS][DIO_PORTS]; /* Channel specific information */
static ChanRec_DAC Chan_DAC[MAX_BOARDS][DAC_PORTS]; /* Channel specific information */
static int NumBoards = 0;                           /* number of boards found       */

/***************************************************************************
 *
 *
 ***************************************************************************/
static struct cdev dda0X_16_cdev;
static struct class *dda0X_16_class;

static struct file_operations dda0X_16_fops = {
  .owner          = THIS_MODULE,
  .read           = dda0X_16_read,
  .write          = dda0X_16_write,
  .unlocked_ioctl = dda0X_16_ioctl,
  .open           = dda0X_16_open,
  .release        = dda0X_16_close,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int dda0X_16_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void dda0X_16_remove_one(struct pci_dev *pdev);

static struct pci_device_id dda0X_16_id_tbl[] = {
  {
  .vendor      =       PCI_VENDOR_ID_CBOARDS,
  .device      =       PCI_DEVICE_ID_CBOARDS_DDA02_16,
  .subvendor   =       PCI_ANY_ID,
  .subdevice   =       PCI_ANY_ID,
  .class       =       0,
  .class_mask  =       0,
  },         
  {
  .vendor      =       PCI_VENDOR_ID_CBOARDS,
  .device      =       PCI_DEVICE_ID_CBOARDS_DDA04_16,
  .subvendor   =       PCI_ANY_ID,
  .subdevice   =       PCI_ANY_ID,
  .class       =       0,
  .class_mask  =       0,
  },         
  {
  .vendor      =       PCI_VENDOR_ID_CBOARDS,
  .device      =       PCI_DEVICE_ID_CBOARDS_DDA08_16,
  .subvendor   =       PCI_ANY_ID,
  .subdevice   =       PCI_ANY_ID,
  .class       =       0,
  .class_mask  =       0,
  },  { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, dda0X_16_id_tbl);

static struct pci_driver dda0X_16_driver = {
  .name     =  "dda0X_16",
  .id_table =  dda0X_16_id_tbl,
  .probe    =  dda0X_16_init_one,
  .remove   =  dda0X_16_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/

static int __init dda0X_16_init(void) 
{
  int err;
  dev_t dev;

  /* Register as a device with kernel.  */
  if (MajorNumber) {
    dev = MKDEV(MajorNumber, 0);
    err = register_chrdev_region(dev, 0xff, "dda0X-16");
  } else {
    err = alloc_chrdev_region(&dev, 0, 0xff, "dda0X-16");
    MajorNumber = MAJOR(dev);
  }
  if (err < 0) {
    printk("%s: Failure to load module. Major Number = %d  error = %d\n",
	   ADAPTER_ID, MAJOR(dev), err);
    return err;
  }

  cdev_init(&dda0X_16_cdev, &dda0X_16_fops);
  dda0X_16_cdev.owner = THIS_MODULE;
  dda0X_16_cdev.ops = &dda0X_16_fops;
  err = cdev_add(&dda0X_16_cdev, dev, 0xff);
  if (err) {
    printk("%s: Error %d in registering file operations", ADAPTER_ID, err);
    return err;
  }

  /* create the class for the dda0X-16 */
  dda0X_16_class = class_create(THIS_MODULE, "dda0x-16");
  if (IS_ERR(dda0X_16_class)) {
    return PTR_ERR(dda0X_16_class);
  }

  err = pci_register_driver(&dda0X_16_driver);
  return  err;	
}

static int dda0X_16_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  int minor;
  u16 base2;
  u16 base3;
  char name[64];

  if (NumBoards >= MAX_BOARDS) {
    printk("dda0X_16_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit dda0X_16.h.\n", NumBoards);
    return -ENODEV;
  }

  BoardData[NumBoards].device = ent->device;

  /* GETTING BASE ADDRESS 2 */
  base2 = BoardData[NumBoards].base2 =  (u16) pci_resource_start(pdev, 2);

  /* GETTING BASE ADDRESS 3 */
  base3 = BoardData[NumBoards].base3 = (u16) pci_resource_start(pdev, 3);

  if (pci_enable_device(pdev))          
    goto err_out_0;

  BoardData[NumBoards].dio_reg_0 = 0x00;
  BoardData[NumBoards].dio_reg_1 = 0x00;

  printk("%s: VendorID = %#x  BADR2=%#x  BADR3=%#x ",ADAPTER_ID,
	 BoardData[NumBoards].device, BoardData[NumBoards].base2, BoardData[NumBoards].base3);
  printk(" 8/3/2012 wjasper@ncsu.edu\n");

  /* Set all channel structures to show nothing active/open */

  Chan_DIO[NumBoards][0].addr = DIO_PORT0A;
  Chan_DIO[NumBoards][1].addr = DIO_PORT0B;
  Chan_DIO[NumBoards][2].addr = DIO_PORT0C;
  Chan_DIO[NumBoards][3].addr = DIO_PORT1A;
  Chan_DIO[NumBoards][4].addr = DIO_PORT1B;
  Chan_DIO[NumBoards][5].addr = DIO_PORT1C;

  for (i = 0; i < DIO_PORTS; i++) {
      Chan_DIO[NumBoards][i].open = FALSE;
      outb(0x0, Chan_DIO[NumBoards][i].addr);
  }

  minor = (NumBoards<<0x4) + DAC_PORTS;
  sprintf(name, "dda0x-16/dio%d_0A", NumBoards);
  device_create(dda0X_16_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dda0x-16/dio%d_0B", NumBoards);
  device_create(dda0X_16_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dda0x-16/dio%d_0C", NumBoards);
  device_create(dda0X_16_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dda0x-16/dio%d_1A", NumBoards);
  device_create(dda0X_16_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dda0x-16/dio%d_1B", NumBoards);
  device_create(dda0X_16_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dda0x-16/dio%d_1C", NumBoards);
  device_create(dda0X_16_class, NULL, MKDEV(MajorNumber, minor), NULL, name);

  /* Set all DAC channels to zero also */
  Chan_DAC[NumBoards][0].addr = DAC0_REG;
  Chan_DAC[NumBoards][1].addr = DAC1_REG;
  Chan_DAC[NumBoards][2].addr = DAC2_REG;
  Chan_DAC[NumBoards][3].addr = DAC3_REG;
  Chan_DAC[NumBoards][4].addr = DAC4_REG;
  Chan_DAC[NumBoards][5].addr = DAC5_REG;
  Chan_DAC[NumBoards][6].addr = DAC6_REG;
  Chan_DAC[NumBoards][7].addr = DAC7_REG;
  
  for (i = 0; i < DAC_PORTS; i++) {
    Chan_DAC[NumBoards][i].open = FALSE;
    Chan_DAC[NumBoards][i].cntrl_reg  = (i << 2);
    outb(0x0, Chan_DAC[NumBoards][i].addr);
  }

  /* create a device in /sys/class/dda0X-16/ */
  for (i = 0; i < DAC_PORTS; i++) {
    minor = (NumBoards<<0x4) + i;
    sprintf(name, "dda0x-16/da%d_%d", NumBoards, i);
    device_create(dda0X_16_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  }

  NumBoards++;
  return 0;
  
err_out_0:
  return -ENODEV;
}


/***************************************************************************
 *
 * Remove driver. Called when "rmmod dda0X_16" is run on the command line.
 *
 ***************************************************************************/

void __exit dda0X_16_exit(void)
{
  dev_t dev;

  dev = MKDEV(MajorNumber, 0);
  pci_unregister_driver(&dda0X_16_driver);
  class_destroy(dda0X_16_class);
  cdev_del(&dda0X_16_cdev);
  unregister_chrdev_region(dev, 0xff);
  printk("%s: module removed from dda0X_16_exit.\n", ADAPTER_ID);
}

static void dda0X_16_remove_one(struct pci_dev *pdev)
{
  int i;
  int minor;
  
  NumBoards--;
  for (i = 0; i < DAC_PORTS + DIO_PORTS; i++) {
    minor = (NumBoards<<0x4) + i;
    device_destroy(dda0X_16_class, MKDEV(MajorNumber, minor));
  }

  #ifdef DEBUG
    printk("%s: module removed.\n", ADAPTER_ID);
  #endif
}

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int dda0X_16_open(struct inode *iNode, struct file *filePtr)
{
  u16 base2;
  u16 base3;
  u32 device;
  int board = 0;
  int port = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  port =  PORT(minor);             /* get which port */
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;
  device = BoardData[board].device;

  /* 
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes 
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */
  if (port >= 0 && port < 2) {
    if (Chan_DAC[board][port].open == TRUE) {
        return -EBUSY;
    }
    Chan_DAC[board][port].open = TRUE;                 /* The device is open */
    Chan_DAC[board][port].cntrl_reg |= EN;
    Chan_DAC[board][port].cntrl_reg &= ~(SU | RANGE);
    outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
    Chan_DAC[board][port].value = 0x0;
    outw(Chan_DAC[board][port].value, Chan_DAC[board][port].addr);
    #ifdef DEBUG
      printk("%s: open(): minor %d DAC Channel %d.\n", ADAPTER_ID, minor, port);
    #endif
    return 0;   
  } else if (port >= 2 && port < 4 && ((device == PCI_DEVICE_ID_CBOARDS_DDA04_16) || (device == PCI_DEVICE_ID_CBOARDS_DDA08_16))) {
    if (Chan_DAC[board][port].open == TRUE) {
        return -EBUSY;
    }
    Chan_DAC[board][port].open = TRUE;                 /* The device is open */
    Chan_DAC[board][port].cntrl_reg |= EN;
    Chan_DAC[board][port].cntrl_reg &= ~(SU | RANGE);
    outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
    Chan_DAC[board][port].value = 0x0;
    outw(Chan_DAC[board][port].value, Chan_DAC[board][port].addr);
    Chan_DAC[board][port].cntrl_reg |= EN;
    outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
    #ifdef DEBUG
      printk("%s: open(): minor %d DAC Channel %d.\n", ADAPTER_ID, minor, port);
    #endif
    return 0;   
  } else if (port >= 4 && port < 8 && (device == PCI_DEVICE_ID_CBOARDS_DDA08_16)) {
    if (Chan_DAC[board][port].open == TRUE) {
        return -EBUSY;
    }
    Chan_DAC[board][port].open = TRUE;                 /* The device is open */
    Chan_DAC[board][port].cntrl_reg |= EN;
    Chan_DAC[board][port].cntrl_reg &= ~(SU | RANGE);
    outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
    Chan_DAC[board][port].value = 0x0;
    outw(Chan_DAC[board][port].value, Chan_DAC[board][port].addr);
    Chan_DAC[board][port].cntrl_reg |= EN;
    outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
    #ifdef DEBUG
      printk("%s: open(): minor %d DAC Channel %d.\n", ADAPTER_ID, minor, port);
    #endif
    return 0;   
  } else {   /* DIO Port */
    port -= DAC_PORTS;
    if (Chan_DIO[board][port].open == TRUE) {
        return -EBUSY;
    }
    Chan_DIO[board][port].open = TRUE;                 /* The device is open */
    Chan_DIO[board][port].f_flags = filePtr->f_flags;

    #ifdef DEBUG
      printk("%s: open(): minor %d  port %d \n", ADAPTER_ID, minor, port);
    #endif
    return 0;   
  }
  return -ENODEV;
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int dda0X_16_close(struct inode *iNode, struct file *filePtr)
{
  u16 base2, base3;
  int board = 0;
  int port  = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  port =  PORT(minor);             /* get which port */
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;

  #ifdef DEBUG
      printk("%s: close() of minor number %d.\n", ADAPTER_ID, minor);
  #endif
  if (port < DAC_PORTS) {
    Chan_DAC[board][port].open = FALSE;
    Chan_DAC[board][port].cntrl_reg &= ~(EN | SU);
    outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
  } else {
    port -= DAC_PORTS;
    Chan_DIO[board][port].open = FALSE;
  }
  return 0;
}

/***************************************************************************
 *
 * read() service function
 *
 ***************************************************************************/

static ssize_t dda0X_16_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  u8  value;
  int minor;
  int base2;
  int board = 0;
  int port = 0;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = iminor(iNode);
  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port */
  base2 = BoardData[board].base2;

  /* check to see if reading a value from the DIO */

  if (port < DAC_PORTS) {
    put_user(Chan_DAC[board][port].value, (u16*) buf );
    #ifdef DEBUG
    printk("dda0X_16_read: %s DIO Port %#x  address = %#x  value = %#x\n", 
	   ADAPTER_ID, port, Chan_DAC[board][port].addr, Chan_DAC[board][port].value);
     #endif
  } else if (port >= DAC_PORTS  && port < (DAC_PORTS + DIO_PORTS)) {
    port -= DAC_PORTS;
    value = inb(Chan_DIO[board][port].addr);
    put_user(value, (u8*) buf);
    #ifdef DEBUG
      printk("dda0X_16_read: %s DIO Port %#x  address = %#x  value = %#x\n", 
	     ADAPTER_ID, port, Chan_DIO[board][port].addr, value);
     #endif
  } else {
    printk("dda0X_16_read: Incorrect minor number (%d).\n", minor);
    return -1;
  }
  return 1;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t dda0X_16_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  int  minor;
  int board = 0;
  int port = 0;
  int base2;
  int base3;
  u16 dac_value;
  u8 dio_value;
  struct inode *iNode = filePtr->f_dentry->d_inode;

  minor = iminor(iNode);
  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port    */
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;

  if (port < DAC_PORTS) {
    get_user(dac_value, (u16*)buf);
    Chan_DAC[board][port].value = dac_value;
    outw(dac_value, Chan_DAC[board][port].addr);
    #ifdef DEBUG
      printk("dda0X_16_write %s DAC Channel %#x  address = %#x  value = %#x cntrl_reg = %#x\n", 
	     ADAPTER_ID, port, Chan_DAC[board][port].addr, dac_value, Chan_DAC[board][port].cntrl_reg);
    #endif
  } else if (port >= DAC_PORTS && port < (DAC_PORTS + DIO_PORTS)) {   /* check to see if writing a value to the DIO */
    port -= DAC_PORTS;
    get_user(dio_value, (u8*)buf);
    Chan_DIO[board][port].value = dio_value;
    outb(dio_value, Chan_DIO[board][port].addr);
    #ifdef DEBUG
      printk("dda0X_16_write %s DIO Port %#x  address = %#x  value = %#x\n", 
	     ADAPTER_ID, port, Chan_DIO[board][port].addr, dio_value);
    #endif
    return 1;
  } else {
    printk("dda0X_16_write: Incorrect minor number (%d).\n", minor);
    return -1;
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

static long dda0X_16_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  struct inode *iNode = filePtr->f_dentry->d_inode;
  int minor = iminor(iNode);
  int board = 0;
  int port = 0;
  u16 base2, base3;
  u16 wreg = 0x0;

  board = BOARD(minor);    /* get which board   */
  port = PORT(minor);      /* get which port */
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;
  
  #ifdef DEBUG
     printk("dda0X_16_ioctl: minor = %d\n", minor);
  #endif
 
  if (_IOC_TYPE(cmd) != IOCTL_MAGIC) return -EINVAL;
  if (_IOC_NR(cmd) > IOCTL_MAXNR)  return -EINVAL;

  if (port < 0 || port >= (DIO_PORTS + DAC_PORTS)) {
        return(-EINVAL);
  }

  switch (cmd) {
    case DIO_SET_DIRECTION:
      #ifdef DEBUG
        printk("DIO_SET_DIRECTION: minor = %d, arg = %d\n", (int) minor, (int) arg );
      #endif
      port -= DAC_PORTS;
      switch (port) {
        case 0:			/* Port 0A */
          arg &= 0x1;
          BoardData[board].dio_reg_0 &=  0xef;
          BoardData[board].dio_reg_0 |= (arg << 4);
          outb(BoardData[board].dio_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 0A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_0, BoardData[board].dio_reg_0);
          #endif
          break;

        case 1:			/* Port 0B */
          arg &= 0x1;
          BoardData[board].dio_reg_0 &=  0xfd;
          BoardData[board].dio_reg_0 |= (arg << 1);
          outb(BoardData[board].dio_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 0B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_0, BoardData[board].dio_reg_0);
          #endif
          break;

        case 2:			/* Port 0C */
          switch ( arg ) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio_reg_0 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio_reg_0 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio_reg_0 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio_reg_0 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio_reg_0 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio_reg_0 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 0C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_0, BoardData[board].dio_reg_0);
          #endif
          break;

        case 3:			/* Port 1A */
          arg &= 0x1;
          BoardData[board].dio_reg_1 &=  0xef;
          BoardData[board].dio_reg_1 |= (arg << 4);
          outb(BoardData[board].dio_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_1, BoardData[board].dio_reg_1);
          #endif
          break;

        case 4:			/* Port 1B */
          arg &= 0x1;
          BoardData[board].dio_reg_1 &=  0xfd;
          BoardData[board].dio_reg_1 |= (arg << 1);
          outb(BoardData[board].dio_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_1, BoardData[board].dio_reg_1);
          #endif
          break;

        case 5:			/* Port 1C */
          switch ( arg ) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio_reg_1 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio_reg_1 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio_reg_1 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio_reg_1 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio_reg_1 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio_reg_1 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_1, BoardData[board].dio_reg_1);
          #endif
          break;

        default:
          #ifdef DEBUG
          printk("DIO_SET_DIRECTION for Invalid Port\n");
          #endif
          return(-EINVAL);
          break;
      }
      break;
    case DAC_SET_GAINS:
      #ifdef DEBUG
        printk("ioctl DAC_SET_GAINS: Channel = %d, Gain = %#lx\n", minor, arg);
      #endif
      Chan_DAC[board][port].cntrl_reg &= ~RANGE;
      Chan_DAC[board][port].cntrl_reg |= (u16) (arg & RANGE);
      outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
      break;
    case DAC_GET_GAINS:
      put_user( (long) (Chan_DAC[board][port].cntrl_reg & RANGE), (long*) arg );
      break;
    case DAC_ENABLE:
      Chan_DAC[board][port].cntrl_reg |= EN;
      outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
      break;
    case DAC_DISABLE:
      Chan_DAC[board][port].cntrl_reg &= ~EN;
      outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
      break;
    case DAC_SIMULT_UPDATE:
      // initiate a simultaneous update
      wreg = inw(DAC_CNTRL_REG);  
      break;
    case DAC_SET_SIMULT:
      Chan_DAC[board][port].cntrl_reg &= ~SU;
      Chan_DAC[board][port].cntrl_reg |= (arg & SU);
      outw(Chan_DAC[board][port].cntrl_reg, DAC_CNTRL_REG);
      port ^= 0x1;    // get the other pair also
      Chan_DAC[board][port].cntrl_reg &= ~SU;
      Chan_DAC[board][port].cntrl_reg |= (arg & SU);
      break;
    case DAC_GET_SIMULT:
      put_user((long) (Chan_DAC[board][port].cntrl_reg & SU), (long*) arg );
      break;
    default:
      return(-EINVAL);
      break;
  }

  return 0;
}


