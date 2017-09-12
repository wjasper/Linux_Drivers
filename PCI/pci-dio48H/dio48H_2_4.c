/***************************************************************************
 Copyright (C) 2003  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DIO48H, is free software; you can redistribute it
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
 *       PCI-DIO48H Manual Revision 1, February 2001
 *       Measurement Computing Corportation
 *       16 Commerce Blvd.
 *       Middleboro, MA  02346
 *       
 *       (508) 946-5100 www.measurementcomputing.com
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
 * pci-dio48H.c
 *
 ***************************************************************************/

#ifndef __KERNEL__
#define __KERNEL__
#endif
#ifndef MODULE
#define MODULE
#endif

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include "dio48H.h"
#include "pci-dio48H.h"

/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init dio48H_init(void);
static void __exit dio48H_exit (void);
static ssize_t dio48H_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t dio48H_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int dio48H_open(struct inode *iNode, struct file *filePtr);
static int dio48H_close(struct inode *iNode, struct file *filePtr);
static int dio48H_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-DIO48H  module");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

module_init(dio48H_init);
module_exit(dio48H_exit);


/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/

static int MajorNumber = DEFAULT_MAJOR_DEV; /* Major number compiled in     */
static BoardRec BoardData[MAX_BOARDS];      /* Board specific information   */
static ChanRec Chan[MAX_BOARDS][DIO_PORTS]; /* Channel specific information */
static int NumBoards = 0;                   /* number of boards found       */

/***************************************************************************
 *
 *
 ***************************************************************************/
static struct file_operations dio48H_fops = {
  owner:          THIS_MODULE,
  read:           dio48H_read,
  write:          dio48H_write,
  ioctl:          dio48H_ioctl,
  open:           dio48H_open,
  release:        dio48H_close,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int dio48H_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void dio48H_remove_one(struct pci_dev *pdev);

static struct pci_device_id dio48H_id_tbl[] __devinitdata = {
  {
  vendor:                  PCI_VENDOR_ID_CBOARDS,
  device:                  PCI_DEVICE_ID_CBOARDS_DIO48H,
  subvendor:               PCI_ANY_ID,
  subdevice:               PCI_ANY_ID,
  class:                   0,
  class_mask:              0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, dio48H_id_tbl);

static struct pci_driver dio48H_driver = {
  name:         "pci-dio48H",
  id_table:     dio48H_id_tbl,
  probe:        dio48H_init_one,
  remove:       dio48H_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/

static int __init dio48H_init(void) 
{
  int err;

  /* Register as a device with kernel.  */

  if ((err = register_chrdev(MajorNumber, "pci-dio48H", &dio48H_fops))) {
    printk("%s: Failure to load module. Major Number = %d error %d\n",
	   ADAPTER_ID, MajorNumber, -err);
    return err; 
  }

  err = pci_module_init(&dio48H_driver);	
  return  err;	
}

static int __devinit dio48H_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  u16 base1;

  if (NumBoards >= MAX_BOARDS) {
    printk("dio48H_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit dio48H.h.\n", NumBoards);
    return -ENODEV;
  }

  /*GETTING BASE ADDRESS 1 */
  base1 = BoardData[NumBoards].base1 = (u16) pci_resource_start(pdev, 1);

  if (pci_enable_device(pdev))          
    goto err_out_0;

  printk("%s: BADR1=%#x ",ADAPTER_ID, BoardData[NumBoards].base1);
  printk(" 04/12/05 wjasper@ncsu.edu\n");

  BoardData[NumBoards].dio48H_reg_0 = 0x80;
  BoardData[NumBoards].dio48H_reg_1 = 0x80;

  /* Set all channel structures to show nothing active/open */

  Chan[NumBoards][0].addr = DIO_PORT0A;
  Chan[NumBoards][1].addr = DIO_PORT0B;
  Chan[NumBoards][2].addr = DIO_PORT0C;
  Chan[NumBoards][3].addr = DIO_PORT1A;
  Chan[NumBoards][4].addr = DIO_PORT1B;
  Chan[NumBoards][5].addr = DIO_PORT1C;

  for (i = 0; i < DIO_PORTS; i++) {
      Chan[NumBoards][i].open = FALSE;
      outb(0x0, Chan[NumBoards][i].addr);
  }

  NumBoards++;
  return 0;
  
err_out_0:
  return -ENODEV;
}


/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-dio48H" is run on the command line.
 *
 ***************************************************************************/

void __exit dio48H_exit(void)
{
  pci_unregister_driver(&dio48H_driver);

  if (unregister_chrdev(MajorNumber, "pci-dio48H") != 0) {
    printk("%s: cleanup_module failed.\n", ADAPTER_ID);
  } else {
    printk("%s: module removed.\n", ADAPTER_ID);
  }
}

static void dio48H_remove_one(struct pci_dev *pdev)
{
  if (MOD_IN_USE) {
    printk("%s: device busy, remove delayed\n", ADAPTER_ID);
    return;
  }

  NumBoards--;

  #ifdef DEBUG
    printk("%s: module removed.\n", ADAPTER_ID);
  #endif
}

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int dio48H_open(struct inode *iNode, struct file *filePtr)
{
  u16 base1;
  int board = 0;
  int port = 0;
  int minor = MINOR(iNode->i_rdev);

  board = BOARD(minor);            /* get which board   */
  port =  PORT(minor);             /* get which port */
  base1 = BoardData[board].base1;


  /* 
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes 
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */

  if ( Chan[board][port].open == TRUE ) {
      return -EBUSY;
  }

  MOD_INC_USE_COUNT;
  Chan[board][port].open = TRUE;                 /* The device is open */
  Chan[board][port].f_flags = filePtr->f_flags;

  #ifdef DEBUG
      printk("%s: open(): minor %d .\n", ADAPTER_ID, minor);
  #endif
  return 0;   
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int dio48H_close(struct inode *iNode, struct file *filePtr)
{
  u16 base1;
  int board = 0;
  int port  = 0;
  int minor = MINOR(iNode->i_rdev);

  board = BOARD(minor);            /* get which board   */
  port =  PORT(minor);             /* get which port */
  base1 = BoardData[board].base1;

  #ifdef DEBUG
      printk("%s: close() of minor number %d.\n", ADAPTER_ID, minor);
  #endif

  MOD_DEC_USE_COUNT;
  Chan[board][port].open = FALSE;

  return 0;
}


/***************************************************************************
 *
 * read() service function
 *
 ***************************************************************************/

static ssize_t dio48H_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  u8  value;
  int minor;
  int base1;
  int board = 0;
  int port = 0;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = MINOR(iNode->i_rdev);
  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port */
  base1 = BoardData[board].base1;

  /* check to see if reading a value from the DIO */

  if ( port >= 0 && port < DIO_PORTS ) {
    value = inb(Chan[board][port].addr);
    put_user(value, (u8*) buf);
  } else {
    printk("dio48H_read: Incorrect minor number (%d).\n", minor);
    return -1;
  }

  #ifdef DEBUG
  printk("dio48H_read: %s DIO Port %#x  address = %#x  value = %#x\n", 
          ADAPTER_ID, port, Chan[board][port].addr, value);
  #endif

  return 1;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t dio48H_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  int  minor;
  int board = 0;
  int port = 0;
  int base1;
  u8 value;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = MINOR(iNode->i_rdev);
  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port    */
  base1 = BoardData[board].base1;


  /* check to see if writing a value to the DIO */
  if ( port >= 0 && port <  DIO_PORTS ) {
    get_user(value, (u8*)buf);
    Chan[board][port].value = value;
    outb(value, Chan[board][port].addr);
  } else {
    printk("dio48H_write: Incorrect minor number (%d).\n", minor);
    return -1;
  }
  #ifdef DEBUG
  printk("dio48H_write %s DIO Port %#x  address = %#x  value = %#x\n", 
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

static int dio48H_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  int minor = MINOR(iNode->i_rdev);
  int board = 0;
  int port = 0;
  u16 base1;

  board = BOARD(minor);    /* get which board   */
  port = PORT(minor);      /* get which port */
  base1 = BoardData[board].base1;
  
  #ifdef DEBUG
     printk("dio48H_ioctl: minor = %d\n", minor);
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
          BoardData[board].dio48H_reg_0 &=  0xef;
          BoardData[board].dio48H_reg_0 |= (arg << 4);
          outb(BoardData[board].dio48H_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 1A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_0, BoardData[board].dio48H_reg_0);
          #endif
          break;

        case 1:			/* Port 0B */
          arg &= 0x1;
          BoardData[board].dio48H_reg_0 &=  0xfd;
          BoardData[board].dio48H_reg_0 |= (arg << 1);
          outb(BoardData[board].dio48H_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 1B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_0, BoardData[board].dio48H_reg_0);
          #endif
          break;

        case 2:			/* Port 0C */
          switch ( arg ) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio48H_reg_0 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio48H_reg_0 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio48H_reg_0 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio48H_reg_0 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio48H_reg_0 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio48H_reg_0 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio48H_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 1C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_0, BoardData[board].dio48H_reg_0);
          #endif
          break;

        case 3:			/* Port 1A */
          arg &= 0x1;
          BoardData[board].dio48H_reg_1 &=  0xef;
          BoardData[board].dio48H_reg_1 |= (arg << 4);
          outb(BoardData[board].dio48H_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_1, BoardData[board].dio48H_reg_1);
          #endif
          break;

        case 4:			/* Port 1B */
          arg &= 0x1;
          BoardData[board].dio48H_reg_1 &=  0xfd;
          BoardData[board].dio48H_reg_1 |= (arg << 1);
          outb(BoardData[board].dio48H_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_1, BoardData[board].dio48H_reg_1);
          #endif
          break;

        case 5:			/* Port 1C */
          switch ( arg ) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio48H_reg_1 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio48H_reg_1 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio48H_reg_1 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio48H_reg_1 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio48H_reg_1 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio48H_reg_1 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio48H_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_1, BoardData[board].dio48H_reg_1);
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








