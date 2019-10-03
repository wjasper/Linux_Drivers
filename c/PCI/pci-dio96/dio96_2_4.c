/***************************************************************************
 Copyright (C) 2005  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DIO96, is free software; you can redistribute it
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
 *       PCI-DIO96 Manual Revision 2, November 2000, Measurement Computing
 *       16 Commerce Blvd.
 *       Middleboro, MA  02346
 *       (508) 946-5100  www.measurementcomputing.com
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
 * pci-dio96.c
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
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include "dio96.h"
#include "pci-dio96.h"

/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init dio96_init(void);
static void __exit dio96_exit (void);
static ssize_t dio96_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t dio96_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int dio96_open(struct inode *iNode, struct file *filePtr);
static int dio96_close(struct inode *iNode, struct file *filePtr);
static int dio96_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg);
static void dio96_Interrupt(int irq, void *dev_id, struct pt_regs *fp);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-DIO96  module");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

module_init(dio96_init);
module_exit(dio96_exit);

/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/

static int MajorNumber = DEFAULT_MAJOR_DEV; /* Major number compiled in     */
static BoardRec BoardData[MAX_BOARDS];      /* Board specific information   */
static ChanRec Chan[MAX_BOARDS][DIO_PORTS]; /* Channel specific information */
static int NumBoards = 0;                   /* number of boards found       */
static spinlock_t dio96_lock;
static DECLARE_WAIT_QUEUE_HEAD(dio96_wait); /* wait semaphore           */


/***************************************************************************
 *
 *
 ***************************************************************************/
static struct file_operations dio96_fops = {
  owner:          THIS_MODULE,
  read:           dio96_read,
  write:          dio96_write,
  ioctl:          dio96_ioctl,
  open:           dio96_open,
  release:        dio96_close,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int dio96_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void dio96_remove_one(struct pci_dev *pdev);

static struct pci_device_id dio96_id_tbl[] __devinitdata = {
  {
  vendor:                  PCI_VENDOR_ID_CBOARDS,
  device:                  PCI_DEVICE_ID_CBOARDS_DIO96,
  subvendor:               PCI_ANY_ID,
  subdevice:               PCI_ANY_ID,
  class:                   0,
  class_mask:              0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, dio96_id_tbl);

static struct pci_driver dio96_driver = {
  name:         "pci-dio96",
  id_table:     dio96_id_tbl,
  probe:        dio96_init_one,
  remove:       dio96_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/

static int __init dio96_init(void) 
{
  int err;

  /* Register as a device with kernel.  */

  if ((err = register_chrdev(MajorNumber, "pci-dio96", &dio96_fops))) {
    printk("%s: Failure to load module. Major Number = %d error %d\n",
	   ADAPTER_ID, MajorNumber, -err);
    return err; 
  }

  spin_lock_init(&dio96_lock);
  err = pci_module_init(&dio96_driver);	

  return  err;	
}

static int __devinit dio96_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i;
  u16 base1;
  u16 base3;

  if (NumBoards >= MAX_BOARDS) {
    printk("dio96_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit dio96.h.\n", NumBoards);
    return -ENODEV;
  }

  /*GETTING BASE ADDRESS 1 */
  base1 = BoardData[NumBoards].base1 =  (u16) pci_resource_start(pdev, 1);

  /*GETTING BASE ADDRESS 2 */
  base3 = BoardData[NumBoards].base3 = (u16) pci_resource_start(pdev, 3);

  BoardData[NumBoards].pci_control_reg = inl(PCI_INT_CSR);
  
  if (ENABLE_INTERRUPTS) {
    /* Register interrupt handler. */
    BoardData[NumBoards].irq = pdev->irq;
    if (request_irq(pdev->irq, dio96_Interrupt, (SA_INTERRUPT | SA_SHIRQ), 
		    "pci-dio96", (void *) &BoardData[NumBoards])) {
      /* No free irq found! cleanup and exit */
      printk("%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData[NumBoards].irq);
      goto err_out_0;
    }
  }

  BoardData[NumBoards].pci_control_reg &= ~(INTE | PCIINT);
  BoardData[NumBoards].control_reg_1 = 0x0;
  BoardData[NumBoards].control_reg_2 |= 0x0;

  outl(BoardData[NumBoards].pci_control_reg, PCI_INT_CSR);
  outb(BoardData[NumBoards].control_reg_1, INT_CONTRL_1);
  outb(BoardData[NumBoards].control_reg_2, INT_CONTRL_2);  
  
  if (pci_enable_device(pdev))          
    goto err_out_1;
  
  BoardData[NumBoards].dio96_reg_0 = 0x80;
  BoardData[NumBoards].dio96_reg_1 = 0x80;
  BoardData[NumBoards].dio96_reg_2 = 0x80;
  BoardData[NumBoards].dio96_reg_3 = 0x80;

  printk("%s: BADR1=%#x  BADR2=%#x  IRQ=%d.",ADAPTER_ID,
	 BoardData[NumBoards].base1, BoardData[NumBoards].base3,  BoardData[NumBoards].irq);
  printk(" 03/20/05 wjasper@ncsu.edu\n");

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
      Chan[NumBoards][i].mode = 0;
      outb(0x0, Chan[NumBoards][i].addr);
  }

  NumBoards++;
  return 0;
  
err_out_1:
  free_irq(pdev->irq, (void *) &BoardData[NumBoards]);
err_out_0:
  return -ENODEV;
}


/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-dio96" is run on the command line.
 *
 ***************************************************************************/

void __exit dio96_exit(void)
{
  pci_unregister_driver(&dio96_driver);

  if (unregister_chrdev(MajorNumber, "pci-dio96") != 0) {
    printk("%s: cleanup_module failed.\n", ADAPTER_ID);
  } else {
    printk("%s: module removed.\n", ADAPTER_ID);
  }
}

static void dio96_remove_one(struct pci_dev *pdev)
{
  if (MOD_IN_USE) {
    printk("%s: device busy, remove delayed\n", ADAPTER_ID);
    return;
  }

  NumBoards--;
  free_irq(BoardData[NumBoards].irq, (void *) &BoardData[NumBoards]);

  #ifdef DEBUG
    printk("%s: module removed.\n", ADAPTER_ID);
  #endif
}

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int dio96_open(struct inode *iNode, struct file *filePtr)
{
  u16 base3;
  int board = 0;
  int port  = 0;
  int minor = MINOR(iNode->i_rdev);

  board = BOARD(minor);            /* get which board   */
  port =  PORT(minor);             /* get which port */
  base3 = BoardData[board].base3;

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
      printk("%s: open(): minor %d mode %d.\n", ADAPTER_ID, minor, Chan[board][port].mode);
  #endif
  return 0;   
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int dio96_close(struct inode *iNode, struct file *filePtr)
{
  u16 base3;
  int board = 0;
  int port  = 0;
  int minor = MINOR(iNode->i_rdev);

  board = BOARD(minor);            /* get which board   */
  port =  PORT(minor);             /* get which port */
  base3 = BoardData[board].base3;

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

static ssize_t dio96_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  int minor;
  int base3;
  int board = 0;
  int port;
  u8  value = 0;
  struct inode *iNode = filePtr->f_dentry->d_inode;

  minor = MINOR(iNode->i_rdev);
  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port */
  base3 = BoardData[board].base3;
 
  #ifdef DEBUG
    printk("dio96_read: %s DIO Port %#x  address = %#x  value = %#x\n", 
	   ADAPTER_ID, port, Chan[board][port].addr, value);
  #endif

  /* check to see if reading a value from the DIO */
  if ( port >= 0 && port < DIO_PORTS ) {
    if ( ENABLE_INTERRUPTS && !(Chan[board][port].f_flags & O_NONBLOCK) ) {
      BoardData->IRQComplete = 0;
      wait_event_interruptible(dio96_wait,(BoardData->IRQComplete==1) != 0);
    }
    value = inb(Chan[board][port].addr);
    put_user(value, (u8*) buf);
  } else {
    printk("dio96_read: Incorrect minor number (%d).\n", minor);
    return -1;
  }

  return 1;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t dio96_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  int  minor;
  int board = 0;
  int port = 0;
  int base3;
  u8 value;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = MINOR(iNode->i_rdev);
  board = BOARD(minor);            /* get which board   */
  port = PORT(minor);              /* get which port    */
  base3 = BoardData[board].base3;

  /* check to see if writing a value to the DIO */
  if (port >= 0 && port <  DIO_PORTS) {
    get_user(value, (u8*)buf);
    Chan[board][port].value = value;
    outb(value, Chan[board][port].addr);
  } else {
    printk("dio96_write: Incorrect minor number (%d).\n", minor);
    return -1;
  }
  #ifdef DEBUG
  printk("dio96_write %s DIO Port %#x  address = %#x  value = %#x\n", 
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

static int dio96_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  int minor = MINOR(iNode->i_rdev);
  int board = 0;
  int port = 0;
  u32 pci9052_intreg;
  u16 base1;
  u16 base3;

  board = BOARD(minor);    /* get which board   */
  port = PORT(minor);      /* get which port */
  base1 = BoardData[board].base1;
  base3 = BoardData[board].base3;
  
  #ifdef DEBUG
     printk("dio96_ioctl: minor = %d\n", minor);
  #endif
 
  if (_IOC_TYPE(cmd) != IOCTL_MAGIC) return -EINVAL;
  if (_IOC_NR(cmd) > IOCTL_MAXNR)  return -EINVAL;

  if (port < 0 || port >= DIO_PORTS) {
        return(-EINVAL);
  }

  switch (cmd) {
    case DIO_SET_MODE:
      arg &= 0x3;
      switch (port) {
	  case 0:		       /* Port 0A */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 0A\n");
            #endif
             BoardData[board].dio96_reg_0 &= 0x9f;
             BoardData[board].dio96_reg_0 |= (arg << 5);
             outb(BoardData[board].dio96_reg_0, DIO_CNTRL_REG_0);
             Chan[board][port].mode = arg;
             break;

          case 1:			/* Port 0B */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 0B\n");
            #endif
            if (arg & 0x2)
	       return(-EINVAL);	       /* Port 0B only has Modes 0 & 1 */
            BoardData[board].dio96_reg_0 &=  0xfb;
            BoardData[board].dio96_reg_0 |= (arg << 2);
            outb(BoardData[board].dio96_reg_0, DIO_CNTRL_REG_0);
            Chan[board][port].mode = arg;
            break;

          case 2:			/* Port 0C */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 0C\n");
            #endif
            if (arg)
	      return(-EINVAL);	/* Port 0C only has Mode 0 I/O */
            Chan[board][port].mode = arg;
            break;

	  case 3:		       /* Port 1A */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 1A\n");
            #endif
             BoardData[board].dio96_reg_1 &= 0x9f;
             BoardData[board].dio96_reg_1 |= (arg << 5);
             outb(BoardData[board].dio96_reg_1, DIO_CNTRL_REG_1);
             Chan[board][port].mode = arg;
             break;

          case 4:			/* Port 1B */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 1B\n");
            #endif
            if (arg & 0x2)
	       return(-EINVAL);	       /* Port 1B only has Modes 0 & 1 */
            BoardData[board].dio96_reg_1 &=  0xfb;
            BoardData[board].dio96_reg_1 |= (arg << 2);
            outb(BoardData[board].dio96_reg_1, DIO_CNTRL_REG_1);
            Chan[board][port].mode = arg;
            break;

          case 5:			/* Port 1C */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 1C\n");
            #endif
            if (arg)
	      return(-EINVAL);	/* Port 1C only has Mode 0 I/O */
            Chan[board][port].mode = arg;
            break;

	  case 6:		       /* Port 2A */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 2A\n");
            #endif
             BoardData[board].dio96_reg_2 &= 0x9f;
             BoardData[board].dio96_reg_2 |= (arg << 5);
             outb(BoardData[board].dio96_reg_2, DIO_CNTRL_REG_2);
             Chan[board][port].mode = arg;
             break;

          case 7:			/* Port 2B */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 2B\n");
            #endif
            if (arg & 0x2)
	       return(-EINVAL);	       /* Port 2B only has Modes 0 & 1 */
            BoardData[board].dio96_reg_2 &=  0xfb;
            BoardData[board].dio96_reg_2 |= (arg << 2);
            outb(BoardData[board].dio96_reg_2, DIO_CNTRL_REG_2);
            Chan[board][port].mode = arg;
            break;

          case 8:			/* Port 2C */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 2C\n");
            #endif
            if (arg)
	      return(-EINVAL);	/* Port 2C only has Mode 0 I/O */
            Chan[board][port].mode = arg;
            break;

	  case 9:		       /* Port 3A */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port A\n");
            #endif
             BoardData[board].dio96_reg_3 &= 0x9f;
             BoardData[board].dio96_reg_3 |= (arg << 5);
             outb(BoardData[board].dio96_reg_3, DIO_CNTRL_REG_3);
             Chan[board][port].mode = arg;
             break;

          case 10:			/* Port 3B */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 3B\n");
            #endif
            if (arg & 0x2)
	       return(-EINVAL);	       /* Port 3B only has Modes 0 & 1 */
            BoardData[board].dio96_reg_3 &=  0xfb;
            BoardData[board].dio96_reg_3 |= (arg << 2);
            outb(BoardData[board].dio96_reg_3, DIO_CNTRL_REG_3);
            Chan[board][port].mode = arg;
            break;

          case 11:			/* Port 3C */
            #ifdef DEBUG
              printk("DIO_SET_MODE for Port 3C\n");
            #endif
            if (arg)
	      return(-EINVAL);	/* Port 3C only has Mode 0 I/O */
            Chan[board][port].mode = arg;
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
        case 0:			/* Port 0A */
          arg &= 0x1;
          BoardData[board].dio96_reg_0 &=  0xef;
          BoardData[board].dio96_reg_0 |= (arg << 4);
          outb(BoardData[board].dio96_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 0A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_0, BoardData[board].dio96_reg_0);
          #endif
          break;

        case 1:			/* Port 0B */
          arg &= 0x1;
          BoardData[board].dio96_reg_0 &=  0xfd;
          BoardData[board].dio96_reg_0 |= (arg << 1);
          outb(BoardData[board].dio96_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 0B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_0, BoardData[board].dio96_reg_0);
          #endif
          break;

        case 2:			/* Port 0C */
          switch ( arg ) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio96_reg_0 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio96_reg_0 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio96_reg_0 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio96_reg_0 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio96_reg_0 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio96_reg_0 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio96_reg_0, DIO_CNTRL_REG_0);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 0C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_0, BoardData[board].dio96_reg_0);
          #endif
          break;

        case 3:			/* Port 1A */
          arg &= 0x1;
          BoardData[board].dio96_reg_1 &=  0xef;
          BoardData[board].dio96_reg_1 |= (arg << 4);
          outb(BoardData[board].dio96_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 1A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_1, BoardData[board].dio96_reg_1);
          #endif
          break;

        case 4:			/* Port 1B */
          arg &= 0x1;
          BoardData[board].dio96_reg_1 &=  0xfd;
          BoardData[board].dio96_reg_1 |= (arg << 1);
          outb(BoardData[board].dio96_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 1B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_1, BoardData[board].dio96_reg_1);
          #endif
          break;

        case 5:			/* Port 1C */
          switch ( arg ) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio96_reg_1 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio96_reg_1 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio96_reg_1 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio96_reg_1 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio96_reg_1 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio96_reg_1 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio96_reg_1, DIO_CNTRL_REG_1);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 1C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_1, BoardData[board].dio96_reg_1);
          #endif
          break;

        case 6:			/* Port 2A */
          arg &= 0x1;
          BoardData[board].dio96_reg_2 &=  0xef;
          BoardData[board].dio96_reg_2 |= (arg << 4);
          outb(BoardData[board].dio96_reg_2, DIO_CNTRL_REG_2);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_2, BoardData[board].dio96_reg_2);
          #endif
          break;

        case 7:			/* Port 2B */
          arg &= 0x1;
          BoardData[board].dio96_reg_2 &=  0xfd;
          BoardData[board].dio96_reg_2 |= (arg << 1);
          outb(BoardData[board].dio96_reg_2, DIO_CNTRL_REG_2);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_2, BoardData[board].dio96_reg_2);
          #endif
          break;

        case 8:			/* Port 2C */
          switch ( arg ) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio96_reg_2 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio96_reg_2 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio96_reg_2 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio96_reg_2 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio96_reg_2 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio96_reg_2 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio96_reg_2, DIO_CNTRL_REG_2);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 2C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_2, BoardData[board].dio96_reg_2);
          #endif
          break;

        case 9:			/* Port 3A */
          arg &= 0x1;
          BoardData[board].dio96_reg_3 &=  0xef;
          BoardData[board].dio96_reg_3 |= (arg << 4);
          outb(BoardData[board].dio96_reg_3, DIO_CNTRL_REG_3);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 3A address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_3, BoardData[board].dio96_reg_3);
          #endif
          break;

        case 10:			/* Port 3B */
          arg &= 0x1;
          BoardData[board].dio96_reg_3 &=  0xfd;
          BoardData[board].dio96_reg_3 |= (arg << 1);
          outb(BoardData[board].dio96_reg_3, DIO_CNTRL_REG_3);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 3B address = %#x value = %#x\n", 
                    DIO_CNTRL_REG_3, BoardData[board].dio96_reg_3);
          #endif
          break;

        case 11:			/* Port 3C */
          switch ( arg ) {
            case PORT_INPUT:         /* CU = IN  CL = IN */
                BoardData[board].dio96_reg_3 |= 0x9;
                break;
            case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
                BoardData[board].dio96_reg_3 &= ~(0x9);
                break;
            case LOW_PORT_INPUT:     /* CL = IN */
                BoardData[board].dio96_reg_3 |= 0x1;
                break;
            case LOW_PORT_OUTPUT:    /* CL = OUT */
                BoardData[board].dio96_reg_3 &= ~(0x1);
                break;
            case HIGH_PORT_INPUT:    /* CU = IN */
                BoardData[board].dio96_reg_3 |= 0x8;
                break;
            case HIGH_PORT_OUTPUT:   /* CU = OUT */
                BoardData[board].dio96_reg_3 &= ~(0x8);
                break;
            default:
                return(-EINVAL);
                break;
          }
          outb(BoardData[board].dio96_reg_3, DIO_CNTRL_REG_3);
          #ifdef DEBUG
            printk("DIO_SET_DIRECTION for Port 3C address = %#x value = %#x\n", 
                  DIO_CNTRL_REG_3, BoardData[board].dio96_reg_3);
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
      pci9052_intreg = inl(PCI_INT_CSR);
      pci9052_intreg |= (INTE | PCIINT);
      outl(pci9052_intreg, PCI_INT_CSR);
      break;
      
    case INTERRUPT_DISABLE:
      pci9052_intreg = inl(PCI_INT_CSR);
      pci9052_intreg &= ~(INTE | PCIINT);
      outl(pci9052_intreg, PCI_INT_CSR);
      break;
      
    case INTERRUPT_POLARITY:
      pci9052_intreg = inl(PCI_INT_CSR);
      pci9052_intreg &= ~(INTPOL);
      pci9052_intreg |= arg;
      outl(pci9052_intreg, PCI_INT_CSR);
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

static void dio96_Interrupt(int irq, void *dev_id, struct pt_regs *fp)
{
  BoardRec *boardData = dev_id;
  u16 base1 = boardData->base1;

  spin_lock_irq(&dio96_lock);

  /* check to see if the PCI-DIO96 did in fact interrupt */
  if ( inl(PCI_INT_CSR) & INT ) {
    #ifdef DEBUG
      printk("Entering dio96_Interrupt()\n");
    #endif

    BoardData->pci_control_reg = inl(PCI_INT_CSR);
    if (BoardData->pci_control_reg & LEVEL) BoardData->pci_control_reg |= INTCLR;
    BoardData->pci_control_reg &= ~INT;
    BoardData->pci_control_reg |= (INTE | PCIINT);
    outl(BoardData->pci_control_reg, PCI_INT_CSR);
    BoardData->IRQComplete = 1;
    wake_up_interruptible(&dio96_wait);
 
  }
  spin_unlock_irq(&dio96_lock);  
}







