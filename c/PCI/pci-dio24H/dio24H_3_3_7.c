/***************************************************************************
 Copyright (C) 2007-2012  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DIO24H, is free software; you can redistribute it
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
 *       PCI-DIO24 Manual Revision 1, September 1998, ComputerBoards Inc.
 *       125 High Street, #6, Mansfield, MA 02048
 *       (508) 261-1123 www.computerboards.com
 *       
 *       PLX Technology
 *       www.plxtech.com  PCI-9050-1 Data Boot
 *
 *       Intel: 8255A-5 Programmable Peripheral Interface
 *       Order Number 231308-004
 *       www.intel.com
 *
 */

/***************************************************************************
 *
 * pci-dio24H.c
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
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include "dio24H.h"
#include "pci-dio24H.h"

/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init dio24H_init(void);
static void __exit dio24H_exit (void);
static ssize_t dio24H_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t dio24H_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int dio24H_open(struct inode *iNode, struct file *filePtr);
static int dio24H_close(struct inode *iNode, struct file *filePtr);
static long dio24H_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg);
static irqreturn_t dio24H_Interrupt(int irq, void *dev_id);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-DIO24H  module");
MODULE_LICENSE("GPL");

module_init(dio24H_init);
module_exit(dio24H_exit);


/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/

static int MajorNumber = DEFAULT_MAJOR_DEV; /* Major number compiled in     */
static BoardRec BoardData[MAX_BOARDS];      /* Board specific information   */
static ChanRec Chan[MAX_BOARDS][DIO_PORTS]; /* Channel specific information */
static spinlock_t dio24H_lock;
static DECLARE_WAIT_QUEUE_HEAD(dio24H_wait); /* wait semaphore               */
static int NumBoards = 0;                   /* number of boards found       */

/***************************************************************************
 *
 *
 ***************************************************************************/
static struct cdev dio24H_cdev;
static struct class *dio24H_class;

static struct file_operations dio24H_fops = {
  .owner          = THIS_MODULE,
  .read           = dio24H_read,
  .write          = dio24H_write,
  .unlocked_ioctl = dio24H_ioctl,
  .open           = dio24H_open,
  .release        = dio24H_close,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int dio24H_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void dio24H_remove_one(struct pci_dev *pdev);

static struct pci_device_id dio24H_id_tbl[] __devinitdata = {
  {
  .vendor     =   PCI_VENDOR_ID_CBOARDS,
  .device     =   PCI_DEVICE_ID_CBOARDS_DIO24H,
  .subvendor  =   PCI_ANY_ID,
  .subdevice  =   PCI_ANY_ID,
  .class      =   0,
  .class_mask =   0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, dio24H_id_tbl);

static struct pci_driver dio24H_driver = {
  .name     =  "dio24H",
  .id_table =  dio24H_id_tbl,
  .probe    =  dio24H_init_one,
  .remove   =  dio24H_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/

static int __init dio24H_init(void) 
{
  int err;
  dev_t dev;

  /* Register as a device with kernel.  */
  if (MajorNumber) {
    dev = MKDEV(MajorNumber, 0);
    err = register_chrdev_region(dev, 0xff, "dio24H");
  } else {
    err = alloc_chrdev_region(&dev, 0, 0xff, "dio24H");
    MajorNumber = MAJOR(dev);
  }
  if (err < 0) {
    printk("%s: Failure to load module. Major Number = %d  error = %d\n",
	   ADAPTER_ID, MAJOR(dev), err);
    return err;
  }

  cdev_init(&dio24H_cdev, &dio24H_fops);
  dio24H_cdev.owner = THIS_MODULE;
  dio24H_cdev.ops = &dio24H_fops;
  err = cdev_add(&dio24H_cdev, dev, 0xff);
  if (err) {
    printk("%s: Error %d in registering file operations", ADAPTER_ID, err);
    return err;
  }

  /* create the class for the dio24H */
  dio24H_class = class_create(THIS_MODULE, "dio24H");
  if (IS_ERR(dio24H_class)) {
    return PTR_ERR(dio24H_class);
  }

  err = pci_register_driver(&dio24H_driver);
  return  err;	

  spin_lock_init(&dio24H_lock);
  err = pci_register_driver(&dio24H_driver);	

  return  err;	
}

static int __devinit dio24H_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  int minor;
  u16 base1;
  u16 base2;
  char name[64];

  if (NumBoards >= MAX_BOARDS) {
    printk("dio24H_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit dio24H.h.\n", NumBoards);
    return -ENODEV;
  }

  /*GETTING BASE ADDRESS 1 */
  base1 = BoardData[NumBoards].base1 =  (u16) pci_resource_start(pdev, 1);

  /*GETTING BASE ADDRESS 2 */
  base2 = BoardData[NumBoards].base2 = (u16) pci_resource_start(pdev, 2);

  /* Register interrupt handler. */
  BoardData[NumBoards].irq = pdev->irq;
  BoardData[NumBoards].IRQComplete = 0;
  if (request_irq(pdev->irq, dio24H_Interrupt, (IRQF_DISABLED | IRQF_SHARED),
		  "dio24H", (void *) &BoardData[NumBoards])) {
    /* No free irq found! cleanup and exit */
    printk("%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData[NumBoards].irq);
    goto err_out_0;
  }

  if (pci_enable_device(pdev))          
    goto err_out_1;

  init_waitqueue_head(&dio24H_wait);
  BoardData[NumBoards].pci_control_reg = inl(INTERRUPT_REG);
  BoardData[NumBoards].pci_control_reg &= ~(INTE | PCIINT);
  outl(BoardData[NumBoards].pci_control_reg, INTERRUPT_REG);
  BoardData[NumBoards].dio24H_reg = 0x80;

  printk("%s: BADR1=%#x  BADR2=%#x  IRQ=%d.",ADAPTER_ID,
	 BoardData[NumBoards].base1, BoardData[NumBoards].base2,  BoardData[NumBoards].irq);
  printk(" 8/3/2012 wjasper@ncsu.edu\n");

  /* Set all channel structures to show nothing active/open */

  Chan[NumBoards][0].addr = DIO_PORTA;
  Chan[NumBoards][1].addr = DIO_PORTB;
  Chan[NumBoards][2].addr = DIO_PORTC;

  for (i = 0; i < DIO_PORTS; i++) {
      Chan[NumBoards][i].open = FALSE;
      Chan[NumBoards][i].mode = 0;
      outb(0x0, Chan[NumBoards][i].addr);
  }

  minor = (NumBoards<<0x4);
  sprintf(name, "dio24H/dio%d_0A", NumBoards);
  device_create(dio24H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio24H/dio%d_0B", NumBoards);
  device_create(dio24H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "dio24H/dio%d_0C", NumBoards);
  device_create(dio24H_class, NULL, MKDEV(MajorNumber, minor), NULL, name);

  NumBoards++;
  return 0;
  
err_out_1:
  free_irq(pdev->irq, (void *) &BoardData[NumBoards]);
err_out_0:
  return -ENODEV;
}


/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-dio24H" is run on the command line.
 *
 ***************************************************************************/

void __exit dio24H_exit(void)
{
  dev_t dev;

  dev = MKDEV(MajorNumber, 0);
  pci_unregister_driver(&dio24H_driver);
  class_destroy(dio24H_class);
  cdev_del(&dio24H_cdev);
  unregister_chrdev_region(dev, 0xff);
  printk("%s: module removed from dio24H_exit.\n", ADAPTER_ID);
}

static void dio24H_remove_one(struct pci_dev *pdev)
{
  int i;
  int minor;

  NumBoards--;
  free_irq(BoardData[NumBoards].irq, (void *) &BoardData[NumBoards]);

  for (i = 0; i < DIO_PORTS; i++) {
    minor = (NumBoards<<0x4) + i;
    device_destroy(dio24H_class, MKDEV(MajorNumber, minor));
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

static int dio24H_open(struct inode *iNode, struct file *filePtr)
{
  u16 base2;
  int board = 0;
  int port = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port    */
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
      printk("%s: open(): minor %d mode %d.\n", ADAPTER_ID, minor, Chan[board][minor].mode);
  #endif
  return 0;   
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int dio24H_close(struct inode *iNode, struct file *filePtr)
{
  u16 base2;
  int board = 0;
  int port = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port    */
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

static ssize_t dio24H_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  u8  value;
  int minor;
  int base2;
  int board = 0;
  int port = 0;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = iminor(iNode);
  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port    */
  base2 = BoardData[board].base2;

  /* check to see if reading a value from the DIO */

  if ( port >= 0 && port < DIO_PORTS ) {
    if ((BoardData[board].pci_control_reg & INTE) && !(Chan[board][port].f_flags & O_NONBLOCK) ) {
      BoardData->IRQComplete = 0;
      wait_event_interruptible(dio24H_wait,(BoardData->IRQComplete==1) != 0);
    }
    value = inb(Chan[board][port].addr);
    put_user(value, (u8*) buf);
  } else {
    printk("dio24H_read: Incorrect minor number (%d).\n", minor);
    return -1;
  }

  #ifdef DEBUG
  printk("dio24H_read: %s DIO Port %#x  address = %#x  value = %#x\n", 
          ADAPTER_ID, port, Chan[board][port].addr, value);
  #endif

  return 1;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t dio24H_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
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
    printk("dio24H_write: Incorrect minor number (%d).\n", minor);
    return -1;
  }
  #ifdef DEBUG
  printk("dio24H_write %s DIO Port %#x  address = %#x  value = %#x\n", 
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

static long dio24H_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  struct inode *iNode = filePtr->f_dentry->d_inode;
  int minor = iminor(iNode);
  int board = 0;
  int port = 0;
  u32 pci9052_intreg;
  u16 base1;
  u16 base2;

  board = BOARD(minor);    /* get which board   */
  port = PORT(minor);              /* get which port    */
  base1 = BoardData[board].base1;
  base2 = BoardData[board].base2;
  
  #ifdef DEBUG
     printk("dio24H_ioctl: minor = %d\n", minor);
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
        case 0:			/* Port A */
          arg &= 0x1;
          BoardData[board].dio24H_reg &=  0xef;
          BoardData[board].dio24H_reg |= (arg << 4);
          outb(BoardData[board].dio24H_reg, DIO_CNTRL_REG);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG, BoardData[board].dio24H_reg);
          #endif
          break;

        case 1:			/* Port B */
          arg &= 0x1;
          BoardData[board].dio24H_reg &=  0xfd;
          BoardData[board].dio24H_reg |= (arg << 1);
          outb(BoardData[board].dio24H_reg, DIO_CNTRL_REG);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG, BoardData[board].dio24H_reg);
          #endif
          break;

        case 2:			/* Port C */
          switch ( arg ) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio24H_reg |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio24H_reg &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio24H_reg |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio24H_reg &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio24H_reg |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio24H_reg &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio24H_reg, DIO_CNTRL_REG);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG, BoardData[board].dio24H_reg);
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
      
    case INTERRUPT_ENABLE:
      pci9052_intreg = inl(INTERRUPT_REG);
      pci9052_intreg |= (INTE | PCIINT);
      BoardData[board].pci_control_reg = pci9052_intreg;
      outl(pci9052_intreg, INTERRUPT_REG);
      break;
      
    case INTERRUPT_DISABLE:
      pci9052_intreg = inl(INTERRUPT_REG);
      pci9052_intreg &= ~(INTE | PCIINT);
      BoardData[board].pci_control_reg = pci9052_intreg;
      outl(pci9052_intreg, INTERRUPT_REG);
      break;
      
    case INTERRUPT_POLARITY:
      pci9052_intreg = inl(INTERRUPT_REG);
      pci9052_intreg &= ~(INTPOL);
      pci9052_intreg |= arg;
      BoardData[board].pci_control_reg = pci9052_intreg;
      outl(pci9052_intreg, INTERRUPT_REG);
      break;

    case INTERRUPT_SELECT:
      pci9052_intreg = inl(INTERRUPT_REG);
      pci9052_intreg &= ~(INTSEL);
      pci9052_intreg |= (arg & INTSEL);
      if ( arg ) {  /* Edge trigger requires active high polarity */
        pci9052_intreg |= INTPOL;
      }
      BoardData[board].pci_control_reg = pci9052_intreg;
      outl(pci9052_intreg, INTERRUPT_REG);
      break;
      
    default:
      return(-EINVAL);
      break;
  }

  return 0;
}


/******************************
 *                            *
 *     Interrupt handler      *
 *                            *
 ******************************/

static irqreturn_t dio24H_Interrupt(int irq, void *dev_id)
{
  BoardRec *boardData = dev_id;
  u16 base1 = boardData->base1;

  spin_lock_irq(&dio24H_lock);

  /* check to see if the PCI-DIO24H did in fact interrupt */
  if ( inl(INTERRUPT_REG) & INT ) {
    #ifdef DEBUG
      printk("Entering dio24H_Interrupt()\n");
    #endif

    BoardData->pci_control_reg = inl(INTERRUPT_REG);
    outl(BoardData->pci_control_reg & ~INT, INTERRUPT_REG);
    outl(BoardData->pci_control_reg | INTE | PCIINT | INTCLR, INTERRUPT_REG);
    BoardData->IRQComplete = 1;
    wake_up_interruptible(&dio24H_wait);
  }
  spin_unlock_irq(&dio24H_lock);  
  return IRQ_HANDLED;
}







