/***************************************************************************
 Copyright (C) 2007-2012  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DIO96H, is free software; you can redistribute it
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
 *       PCI-DIO96H Manual Revision 2, November 2000, Measurement Computing
 *       16 Commerce Blvd.
 *       Middleboro, MA  02346
 *       (508) 946-5100  www.measurementcomputing.com
 *       
 *       Intel: 8255A-5 Programmable Peripheral Interface
 *       Order Number 231308-004
 *       www.intel.com
 *
 */

/***************************************************************************
 *
 * pci-dio96H.c
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
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include "dio96H.h"
#include "pci-dio96H.h"

/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init dio96H_init(void);
static void __exit dio96H_exit (void);
static ssize_t dio96H_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t dio96H_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int dio96H_open(struct inode *iNode, struct file *filePtr);
static int dio96H_close(struct inode *iNode, struct file *filePtr);
static long dio96H_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-DIO96H  module");
MODULE_LICENSE("GPL");

module_init(dio96H_init);
module_exit(dio96H_exit);


/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/

static int MajorNumber = DEFAULT_MAJOR_DEV; /* Major number compiled in     */
static BoardRec BoardData[MAX_BOARDS];      /* Board specific information   */
static ChanRec Chan[MAX_BOARDS][DIO_PORTS]; /* Channel specific information */
static int NumBoards = 0;                   /* number of boards found       */
static spinlock_t dio96H_lock;

/***************************************************************************
 *
 *
 ***************************************************************************/
static struct cdev dio96H_cdev;
static struct class *dio96H_class;

static struct file_operations dio96H_fops = {
  .owner            =   THIS_MODULE,
  .read             =   dio96H_read,
  .write            =   dio96H_write,
  .unlocked_ioctl   =   dio96H_ioctl,
  .open             =   dio96H_open,
  .release          =   dio96H_close,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int dio96H_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void dio96H_remove_one(struct pci_dev *pdev);

static struct pci_device_id dio96H_id_tbl[] __devinitdata = {
  {
  .vendor      =       PCI_VENDOR_ID_CBOARDS,
  .device      =       PCI_DEVICE_ID_CBOARDS_DIO96H,
  .subvendor   =       PCI_ANY_ID,
  .subdevice   =       PCI_ANY_ID,
  .class       =       0,
  .class_mask  =       0,
  },
  {
  .vendor      =       PCI_VENDOR_ID_CBOARDS,
  .device      =       PCI_DEVICE_ID_CBOARDS_DIO96H_PCIe,
  .subvendor   =       PCI_ANY_ID,
  .subdevice   =       PCI_ANY_ID,
  .class       =       0,
  .class_mask  =       0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, dio96H_id_tbl);

static struct pci_driver dio96H_driver = {
  .name     =  "dio96H",
  .id_table =  dio96H_id_tbl,
  .probe    =  dio96H_init_one,
  .remove   =  dio96H_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/

static int __init dio96H_init(void) 
{
  int err;
  dev_t dev;

  /* Register as a device with kernel.  */
  if (MajorNumber) {
    dev = MKDEV(MajorNumber, 0);
    err = register_chrdev_region(dev, 0xff, "dio96H");
  } else {
    err = alloc_chrdev_region(&dev, 0, 0xff, "dio96H");
    MajorNumber = MAJOR(dev);
  }
  if (err < 0) {
    printk("%s: Failure to load module. Major Number = %d  error = %d\n",
	   ADAPTER_ID, MAJOR(dev), err);
    return err;
  }

  cdev_init(&dio96H_cdev, &dio96H_fops);
  dio96H_cdev.owner = THIS_MODULE;
  dio96H_cdev.ops = &dio96H_fops;
  err = cdev_add(&dio96H_cdev, dev, 0xff);
  if (err) {
    printk("%s: Error %d in registering file operations", ADAPTER_ID, err);
    return err;
  }

  /* create the class for the dio96H */
  dio96H_class = class_create(THIS_MODULE, "dio96H");
  if (IS_ERR(dio96H_class)) {
    return PTR_ERR(dio96H_class);
  }

  spin_lock_init(&dio96H_lock);
  err = pci_register_driver(&dio96H_driver);	

  return  err;	
}

static int __devinit dio96H_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  int minor;
  u16 base2;
  char name[64];

  if (NumBoards >= MAX_BOARDS) {
    printk("dio96H_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit dio96H.h.\n", NumBoards);
    return -ENODEV;
  }

  /*GETTING BASE ADDRESS 2 */
  base2 = BoardData[NumBoards].base2 = (u16) pci_resource_start(pdev, 2);

  if (pci_enable_device(pdev))          
    goto err_out_0;

  BoardData[NumBoards].dio96H_reg_0 = 0x80;
  BoardData[NumBoards].dio96H_reg_1 = 0x80;
  BoardData[NumBoards].dio96H_reg_2 = 0x80;
  BoardData[NumBoards].dio96H_reg_3 = 0x80;

  printk("%s: BADR2=%#x ",ADAPTER_ID, BoardData[NumBoards].base2);
  printk(" 8/3/2012 wjasper@ncsu.edu\n");

  /* Set all channel structures to show nothing active/open */

  Chan[NumBoards][0].addr = DIO_PORT0A;
  Chan[NumBoards][1].addr = DIO_PORT0B;
  Chan[NumBoards][2].addr = DIO_PORT0C;
  Chan[NumBoards][3].addr = DIO_PORT1A;
  Chan[NumBoards][4].addr = DIO_PORT1B;
  Chan[NumBoards][5].addr = DIO_PORT1C;
  Chan[NumBoards][6].addr = DIO_PORT2A;
  Chan[NumBoards][7].addr = DIO_PORT2B;
  Chan[NumBoards][8].addr = DIO_PORT2C;
  Chan[NumBoards][9].addr = DIO_PORT3A;
  Chan[NumBoards][10].addr = DIO_PORT3B;
  Chan[NumBoards][11].addr = DIO_PORT3C;

  for (i = 0; i < DIO_PORTS; i++) {
      Chan[NumBoards][i].open = FALSE;
      outb(0x0, Chan[NumBoards][i].addr);
  }

  minor = (NumBoards<<0x4);
  sprintf(name, "dio96H/dio%d_0A", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_0B", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_0C", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_1A", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_1B", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_1C", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_2A", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_2B", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_2C", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_3A", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_3B", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio96H/dio%d_3C", NumBoards);
  device_create(dio96H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;

  NumBoards++;
  return 0;
  
err_out_0:
  return -ENODEV;
}


/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-dio96H" is run on the command line.
 *
 ***************************************************************************/

void __exit dio96H_exit(void)
{
  dev_t dev;

  dev = MKDEV(MajorNumber, 0);
  pci_unregister_driver(&dio96H_driver);
  class_destroy(dio96H_class);
  cdev_del(&dio96H_cdev);
  unregister_chrdev_region(dev, 0xff);
  printk("%s: module removed from dio96H_exit.\n", ADAPTER_ID);
}

static void dio96H_remove_one(struct pci_dev *pdev)
{
  int i;
  int minor;

  NumBoards--;

  for (i = 0; i < DIO_PORTS; i++) {
    minor = (NumBoards<<0x4) + i;
    device_destroy(dio96H_class, MKDEV(MajorNumber, minor));
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

static int dio96H_open(struct inode *iNode, struct file *filePtr)
{
  u16 base2;
  int board = 0;
  int port = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  port =  PORT(minor);             /* get which port */
  base2 = BoardData[board].base2;

  /* 
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes 
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */

  if ( Chan[board][port].open == TRUE ) {
      return -EBUSY;
  }

  Chan[board][port].open = TRUE;                 /* The device is open */
  Chan[board][port].f_flags = filePtr->f_flags;

  #ifdef DEBUG
      printk("%s: open(): minor %d mode 0.\n", ADAPTER_ID, minor);
  #endif
  return 0;   
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int dio96H_close(struct inode *iNode, struct file *filePtr)
{
  u16 base2;
  int board = 0;
  int port  = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  port =  PORT(minor);             /* get which port */
  base2 = BoardData[board].base2;

  #ifdef DEBUG
      printk("%s: close() of minor number %d.\n", ADAPTER_ID, minor);
  #endif

  Chan[board][port].open = FALSE;

  return 0;
}


/***************************************************************************
 *
 * read() service function
 *
 ***************************************************************************/

static ssize_t dio96H_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  u8  value;
  int minor;
  int port;
  int base2;
  int board = 0;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = iminor(iNode);
  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port */
  base2 = BoardData[board].base2;


  #ifdef DEBUG
  printk("dio96H_read: %s DIO Port %#x  address = %#x  value = %#x\n", 
          ADAPTER_ID, port, Chan[board][port].addr, value);
  #endif

  /* check to see if reading a value from the DIO */
  if ( port >= 0 && port < DIO_PORTS ) {
    value = inb(Chan[board][port].addr);
    put_user(value, (u8*) buf);
  } else {
    printk("dio96H_read: Incorrect minor number (%d).\n", minor);
    return -1;
  }

  return 1;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t dio96H_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  int  minor;
  int board = 0;
  int port = 0;
  int base2;
  u8 value;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = iminor(iNode);
  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port    */
  base2 = BoardData[board].base2;

  /* check to see if writing a value to the DIO */
  if ( port >= 0 && port <  DIO_PORTS ) {
    get_user(value, (u8*)buf);
    Chan[board][port].value = value;
    outb(value, Chan[board][port].addr);
  } else {
    printk("dio96H_write: Incorrect minor number (%d).\n", minor);
    return -1;
  }
  #ifdef DEBUG
  printk("dio96H_write %s DIO Port %#x  address = %#x  value = %#x\n", 
          ADAPTER_ID, port, Chan[board][port].addr, value);
  #endif
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

static long dio96H_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  struct inode *iNode = filePtr->f_dentry->d_inode;
  int minor = iminor(iNode);
  int board = 0;
  int port = 0;
  u16 base2;

  board = BOARD(minor);    /* get which board   */
  port = PORT(minor);      /* get which port */
  base2 = BoardData[board].base2;
  
  #ifdef DEBUG
     printk("dio96H_ioctl: minor = %d\n", minor);
  #endif
 
  if (_IOC_TYPE(cmd) != IOCTL_MAGIC) return -EINVAL;
  if (_IOC_NR(cmd) > IOCTL_MAXNR)  return -EINVAL;

  if (port < 0 || port >= DIO_PORTS) {
        return(-EINVAL);
  }

  switch (cmd) {
    case DIO_SET_DIRECTION:
      #ifdef DEBUG
        printk("DIO_SET_DIRECTION: minor = %d, arg = %d\n", (int) minor, (int) arg );
      #endif

      switch (port) {
        case 0:			/* Port 0A */
          arg &= 0x1;
          BoardData[board].dio96H_reg_0 &=  0xef;
          BoardData[board].dio96H_reg_0 |= (arg << 4);
          outb(BoardData[board].dio96H_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 0A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_0, BoardData[board].dio96H_reg_0);
          #endif
          break;

        case 1:			/* Port 0B */
          arg &= 0x1;
          BoardData[board].dio96H_reg_0 &=  0xfd;
          BoardData[board].dio96H_reg_0 |= (arg << 1);
          outb(BoardData[board].dio96H_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 0B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_0, BoardData[board].dio96H_reg_0);
          #endif
          break;

        case 2:			/* Port 0C */
          switch (arg) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio96H_reg_0 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio96H_reg_0 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio96H_reg_0 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio96H_reg_0 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio96H_reg_0 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio96H_reg_0 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio96H_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 0C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_0, BoardData[board].dio96H_reg_0);
          #endif
          break;

        case 3:			/* Port 1A */
          arg &= 0x1;
          BoardData[board].dio96H_reg_1 &=  0xef;
          BoardData[board].dio96H_reg_1 |= (arg << 4);
          outb(BoardData[board].dio96H_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 1A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_1, BoardData[board].dio96H_reg_1);
          #endif
          break;

        case 4:			/* Port 1B */
          arg &= 0x1;
          BoardData[board].dio96H_reg_1 &=  0xfd;
          BoardData[board].dio96H_reg_1 |= (arg << 1);
          outb(BoardData[board].dio96H_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 1B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_1, BoardData[board].dio96H_reg_1);
          #endif
          break;

        case 5:			/* Port 1C */
          switch (arg) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio96H_reg_1 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio96H_reg_1 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio96H_reg_1 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio96H_reg_1 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio96H_reg_1 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio96H_reg_1 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio96H_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 1C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_1, BoardData[board].dio96H_reg_1);
          #endif
          break;

        case 6:			/* Port 2A */
          arg &= 0x1;
          BoardData[board].dio96H_reg_2 &=  0xef;
          BoardData[board].dio96H_reg_2 |= (arg << 4);
          outb(BoardData[board].dio96H_reg_2, DIO_CNTRL_REG_2);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_2, BoardData[board].dio96H_reg_2);
          #endif
          break;

        case 7:			/* Port 2B */
          arg &= 0x1;
          BoardData[board].dio96H_reg_2 &=  0xfd;
          BoardData[board].dio96H_reg_2 |= (arg << 1);
          outb(BoardData[board].dio96H_reg_2, DIO_CNTRL_REG_2);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_2, BoardData[board].dio96H_reg_2);
          #endif
          break;

        case 8:			/* Port 2C */
          switch (arg) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio96H_reg_2 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio96H_reg_2 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio96H_reg_2 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio96H_reg_2 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio96H_reg_2 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio96H_reg_2 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio96H_reg_2, DIO_CNTRL_REG_2);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_2, BoardData[board].dio96H_reg_2);
          #endif
          break;

        case 9:			/* Port 3A */
          arg &= 0x1;
          BoardData[board].dio96H_reg_3 &=  0xef;
          BoardData[board].dio96H_reg_3 |= (arg << 4);
          outb(BoardData[board].dio96H_reg_3, DIO_CNTRL_REG_3);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 3A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_3, BoardData[board].dio96H_reg_3);
          #endif
          break;

        case 10:			/* Port 3B */
          arg &= 0x1;
          BoardData[board].dio96H_reg_3 &=  0xfd;
          BoardData[board].dio96H_reg_3 |= (arg << 1);
          outb(BoardData[board].dio96H_reg_3, DIO_CNTRL_REG_3);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 3B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_3, BoardData[board].dio96H_reg_3);
          #endif
          break;

        case 11:			/* Port 3C */
          switch (arg) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio96H_reg_3 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio96H_reg_3 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio96H_reg_3 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio96H_reg_3 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio96H_reg_3 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio96H_reg_3 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio96H_reg_3, DIO_CNTRL_REG_3);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 3C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_3, BoardData[board].dio96H_reg_3);
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

    default:
      return(-EINVAL);
      break;
  }
  return 0;
}





