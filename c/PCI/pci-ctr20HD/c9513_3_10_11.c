/***************************************************************************
 Copyright (C) 2003-2013  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-CTR20HD, is free software; you can redistribute it
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
    c9513.c for PCI-CTR20HD
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
#include <linux/pci.h>    /* defines pci_present() */
#include <asm/io.h>
#include <asm/uaccess.h>
#include "pci-ctr20HD.h"
#include "c9513.h"

/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init ctr20HD_init(void);
static void __exit ctr20HD_exit (void);
static ssize_t ctr20HD_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t ctr20HD_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int ctr20HD_open(struct inode *iNode, struct file *filePtr);
static int ctr20HD_close(struct inode *iNode, struct file *filePtr);
static long ctr20HD_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg);
static void ctr20HD_TimerHandler(unsigned long unused);
static irqreturn_t ctr20HD_Interrupt(int irq, void *dev_id);

module_init(ctr20HD_init);
module_exit(ctr20HD_exit);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-CTR20HD module");
MODULE_LICENSE("GPL");

/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/

static spinlock_t ctr20HD_lock;
static BoardRec BoardData[MAX_BOARDS];                      /* Board specific information      */
static CounterRec  Counter[MAX_BOARDS][NCHIP][NCOUNTERS+1]; /* Counter specific information    */
static int MajorNumber = DEFAULT_MAJOR_DEV;                 /* Major number compiled in        */
static DECLARE_WAIT_QUEUE_HEAD(ctr20HD_wait);               /* wait semaphore                  */
//static struct timer_list TimerList = {function: ctr20HD_TimerHandler};
static int NumBoards = 0;                                   /* number of boards found          */

/***************************************************************************
 *
 *
 ***************************************************************************/
static struct cdev ctr20HD_cdev;
static struct class *ctr20HD_class;

static struct file_operations ctr20HD_fops = {
  .owner            =   THIS_MODULE,
  .read             =   ctr20HD_read,
  .write            =   ctr20HD_write,
  .open             =   ctr20HD_open,
  .release          =   ctr20HD_close,
  .unlocked_ioctl   =   ctr20HD_ioctl,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int ctr20HD_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void ctr20HD_remove_one(struct pci_dev *pdev);

static struct pci_device_id ctr20HD_id_tbl[] = {
  {
  .vendor     =   PCI_VENDOR_ID_CBOARDS,
  .device     =   PCI_DEVICE_ID_CBOARDS_CTR20HD,
  .subvendor  =   PCI_ANY_ID,
  .subdevice  =   PCI_ANY_ID,
  .class      =   0,
  .class_mask =   0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, ctr20HD_id_tbl);

static struct pci_driver ctr20HD_driver = {
  .name     =    "ctr20HD",
  .id_table =    ctr20HD_id_tbl,
  .probe    =    ctr20HD_init_one,
  .remove   =    ctr20HD_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/
static int __init ctr20HD_init(void) 
{
  int err;
  dev_t dev;

  spin_lock_init(&ctr20HD_lock);

  /* Register as a device with kernel.  */
  if (MajorNumber) {
    dev = MKDEV(MajorNumber, 0);
    err = register_chrdev_region(dev, 0xff, "ctr20HD");
  } else {
    err = alloc_chrdev_region(&dev, 0, 0xff, "ctr20HD");
    MajorNumber = MAJOR(dev);
  }
  if (err < 0) {
    printk("%s: Failure to load module. Major Number = %d  error = %d\n",
	   ADAPTER_ID, MAJOR(dev), err);
    return err;
  }

  cdev_init(&ctr20HD_cdev, &ctr20HD_fops);
  ctr20HD_cdev.owner = THIS_MODULE;
  ctr20HD_cdev.ops = &ctr20HD_fops;
  err = cdev_add(&ctr20HD_cdev, dev, 0xff);
  if (err) {
    printk("%s: Error %d in registering file operations", ADAPTER_ID, err);
    return err;
  }

  /* create the class for the ctr20HD */
  ctr20HD_class = class_create(THIS_MODULE, "ctr20HD");
  if (IS_ERR(ctr20HD_class)) {
    return PTR_ERR(ctr20HD_class);
  }

  err = pci_register_driver(&ctr20HD_driver);
  return  err;	
}

static int ctr20HD_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i,j;
  int minor;
  u16 base1;
  u16 base2;
  u8 bReg;
  u32 wReg;
  char name[64];

  if (NumBoards >= MAX_BOARDS) {
    printk("ctr20HD_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit c9513.h.\n", NumBoards);
    return -ENODEV;
  }

  BoardData[NumBoards].pdev = pdev;
  BoardData[NumBoards].irq = 0;
  
  /*GETTING BASE ADDRESS 1 */
  base1 = BoardData[NumBoards].base1 =  (u16) pci_resource_start(pdev, 1);

  /*GETTING BASE ADDRESS 2 */
  base2 = BoardData[NumBoards].base2 = (u16) pci_resource_start(pdev, 2);

  /* pci_enable_device handles IRQ routing, so it must be before request_irq */
  if (pci_enable_device(pdev))          
   goto err_out_0;

  /* Register interrupt handler. */
  wReg = inl_p(INTERRUPT_REG);
  wReg &= ~(PCINT | INTE_AB | INT_CD);
  outl_p(wReg, INTERRUPT_REG);

  BoardData[NumBoards].irq = pdev->irq;
  if (request_irq(pdev->irq, ctr20HD_Interrupt, (IRQF_DISABLED | IRQF_SHARED), 
		  "ctr20HD", (void *) &BoardData[NumBoards])) {
    printk("%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData[NumBoards].irq);
    goto err_out_0;
  }
  
  /* Initialize the 9513 chip */
  outb_p(MASTER_RESET, CMD_REG_A);      /* MASTER RESET */
  outb_p(MASTER_MODE_REG, CMD_REG_A);   /* Select Master Mode Register */
  outb_p(MASTER_RESET, CMD_REG_B);      /* MASTER RESET */
  outb_p(MASTER_MODE_REG, CMD_REG_B);   /* Select Master Mode Register */
  outb_p(MASTER_RESET, CMD_REG_C);      /* MASTER RESET */
  outb_p(MASTER_MODE_REG, CMD_REG_C);   /* Select Master Mode Register */
  outb_p(MASTER_RESET, CMD_REG_D);      /* MASTER RESET */
  outb_p(MASTER_MODE_REG, CMD_REG_D);   /* Select Master Mode Register */

  bReg = TOD_DISABLED | MM_SRC1;
  outb_p(bReg, DATA_REG_A);
  outb_p(bReg, DATA_REG_B);
  outb_p(bReg, DATA_REG_C);
  outb_p(bReg, DATA_REG_D);
  bReg = 0xc0;        /* BCD Division, Disable increment, 8-Bit bus, FOUT On */
  outb_p(bReg, DATA_REG_A);
  outb_p(bReg, DATA_REG_B);
  outb_p(bReg, DATA_REG_C);
  outb_p(bReg, DATA_REG_D);

  BoardData[NumBoards].clockSelectReg_AB = 0x0;  // default to 5 MHz
  BoardData[NumBoards].clockSelectReg_CD = 0x0;  // default to 5 MHz

  outb_p(BoardData[NumBoards].clockSelectReg_AB, CNT_AB_INT);
  outb_p(BoardData[NumBoards].clockSelectReg_CD, CNT_CD_INT);
  
  for ( j = 0; j < NCHIP; j++ ) {
    BoardData[NumBoards].mm[j] = BCD_DIV | DISABLE_INCREMENT | TOD_DISABLED | MM_SRC1;
    BoardData[NumBoards].gateInterval[j] = GATEINTERVAL;
    BoardData[NumBoards].useCounter[j] = 0;
  }

  for ( i = 1; i <= NCOUNTERS; i++ ) {
    for ( j = 0; j < NCHIP; j++ ) {
      Counter[NumBoards][j][i].open = FALSE;
      Counter[NumBoards][j][i].count = 0;
      Counter[NumBoards][j][i].chan = i;
      Counter[NumBoards][j][i].cm = 0;
      Counter[NumBoards][j][i].mode = 0;
    }
  }

  for (i = 1; i <= NCOUNTERS; i++) {
    for (j = 0; j < NCHIP; j++) {
      minor = (NumBoards << 0x8) | (j << 0x3) | i;
      sprintf(name, "ctr20HD/ctr%d_%d%d", NumBoards, j, i);
      device_create(ctr20HD_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
    }
  }
  
  printk("%s: address=%#x IRQ=%d.", ADAPTER_ID, BoardData[NumBoards].base2, pdev->irq);
  printk(" 8/3/2012 wjasper@ncsu.edu\n");

  NumBoards++;
  return 0;

err_out_0:
  return -ENODEV;
}

/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-ctr20HD" is run on the command line.
 *
 ***************************************************************************/
void __exit ctr20HD_exit(void)
{
  dev_t dev;

  dev = MKDEV(MajorNumber, 0);
  pci_unregister_driver(&ctr20HD_driver);
  class_destroy(ctr20HD_class);
  cdev_del(&ctr20HD_cdev);
  unregister_chrdev_region(dev, 0xff);
  printk("%s: module removed from ctr20HD_exit.\n", ADAPTER_ID);
}

static void ctr20HD_remove_one(struct pci_dev *pdev)
{
  int i, j;
  int minor;

  NumBoards--;
  if (BoardData[NumBoards].irq != 0) {
    free_irq(BoardData[NumBoards].irq, (void *) &BoardData[NumBoards]);
  }

  for (i = 1; i <= NCOUNTERS ; i++) {
    for (j = 0; j < NCHIP; j++) {
      minor = (NumBoards << 0x8) | (j << 0x3) | i;
      device_destroy(ctr20HD_class, MKDEV(MajorNumber, minor));
    }
  }

  #ifdef DEBUG
    printk("ctr20HD_remove_one: Board #%d removed.\n", NumBoards);
  #endif
}     /* ctr20HD_remove_one() */

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int ctr20HD_open(struct inode *iNode, struct file *filePtr)
{
  u16 base2;
  int board = 0;
  int chip = 0;   /* 0 = A chip,   1 = B chip  2 = C chip  3 = D chip */
  int counter = 0;   /* counter number 0 - 5 */
  int minor = iminor(iNode);

  board = BOARD(minor);                           /* get which board   */
  chip = CHIP(minor);
  counter = CHAN(minor);
  base2 = BoardData[board].base2;

  /*
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */

  if ( Counter[board][chip][counter].open == TRUE ) {
      return -EBUSY;
  }
  Counter[board][chip][counter].open = TRUE;              /* The device is open */
  Counter[board][chip][counter].mode = filePtr->f_flags;  /* set acquisition mode */

  /* if we are in frequency counting mode, set counter 5 busy */
  if (Counter[board][chip][counter].mode == CTR20HD_FREQUENCY) {
    BoardData[board].useCounter[chip]++;
    Counter[board][chip][CTR5].open = TRUE;
  }

  #ifdef DEBUG
    printk("%s: open(): minor %d mode %d\n", ADAPTER_ID, minor,
	   Counter[board][chip][counter].mode);
   #endif

  return 0;
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int ctr20HD_close(struct inode *iNode, struct file *filePtr)
{
  u8 bReg;
  u16 base2;
  int board = 0;
  int counter = 0;   /* counter number 0 - 5 */
  int chip = 0;   /* 0 = A chip,   1 = B chip  2 = C chip  3 = D chip */
  int minor = iminor(iNode);
  u16 cmd_reg = 0;
  u16 data_reg= 0;

  board = BOARD(minor);                           /* get which board   */
  chip = CHIP(minor);
  counter = CHAN(minor);
  base2 = BoardData[board].base2;

  switch (chip) {
    case 0x0:
      cmd_reg = CMD_REG_A;
      data_reg = DATA_REG_A;
      break;
    case 0x1:
      cmd_reg = CMD_REG_B;
      data_reg = DATA_REG_B;
      break;
    case 0x2:
      cmd_reg = CMD_REG_C;
      data_reg = DATA_REG_C;
      break;
    case 0x3:
      cmd_reg = CMD_REG_D;
      data_reg = DATA_REG_D;
      break;
  }

  Counter[board][chip][counter].open = FALSE;

  /* if we are in frequency counting mode, disarm counter */
  if (Counter[board][chip][counter].mode == CTR20HD_FREQUENCY) {
    bReg = 1 << (counter-1);
    outb_p(DISARM | bReg, cmd_reg);         /* disarm  CTR minor */
    BoardData[board].useCounter[chip]--;
    if (BoardData[board].useCounter[chip] == 0 ) {
      Counter[board][chip][CTR5].open = FALSE;
      outb_p(DISARM | CTR5_BIT, cmd_reg);     /* disarm CTR5  */
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

static ssize_t ctr20HD_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  int minor;
  int board = 0;
  int counter = 0;   /* counter number 0 - 5 */
  int chip = 0;   /* 0 = A chip,   1 = B chip  2 = C chip  3 = D chip */
  u16 data;
  u16 base2;
  u16 cmd_reg = 0;
  u16 data_reg= 0;
  struct inode *iNode = filePtr->f_dentry->d_inode;

  minor = iminor(iNode);
  board = BOARD(minor);                           /* get which board   */
  chip = CHIP(minor);
  counter = CHAN(minor);
  base2 = BoardData[board].base2;

  switch (chip) {
    case 0x0:  /* chip A */
      #ifdef DEBUG
      printk("ctr20HD_read Counter A: minor = %d counter = %d\n", minor, counter);
      #endif
      cmd_reg = CMD_REG_A;
      data_reg = DATA_REG_A;
      break;      
    case 0x1:  /* chip B */
      #ifdef DEBUG
      printk("ctr20HD_read Counter B: minor = %d counter = %d\n", minor, counter);
      #endif
      cmd_reg = CMD_REG_B;
      data_reg = DATA_REG_B;
      break;      
    case 0x2:  /* chip C */
      #ifdef DEBUG
      printk("ctr20HD_read Counter C: minor = %d counter = %d\n", minor, counter);
      #endif
      cmd_reg = CMD_REG_C;
      data_reg = DATA_REG_C;
      break;      
    case 0x3:  /* chip D */
      #ifdef DEBUG
      printk("ctr20HD_read Counter D: minor = %d counter = %d\n", minor, counter);
      #endif
      cmd_reg = CMD_REG_D;
      data_reg = DATA_REG_D;
      break;      
  }

  switch (Counter[board][chip][counter].mode) {
    case CTR20HD_COUNTER:            // read the current count from the counter 
      /* Disarm and Save counters   */
      outb((DISARM_LATCH | 1<<(counter-1)), cmd_reg);

      /* Read the Hold Register */
      outb((counter | HOLD_REG), cmd_reg);        /* Fetch Hold Reg */
      data = inb_p(data_reg) & 0xff;           /* Low Byte */
      data |= (inb_p(data_reg) << 8) & 0xff00; /* High Byte */
      
      /* Write data to user space */
      put_user(data, (u16*) buf);
    #ifdef DEBUG
      printk("ctr20HD_read Counter : minor = %d data = %d\n", minor, data);
    #endif
      
      /* BoardData.busy = FALSE; */
      break;
      
  case CTR20HD_FREQUENCY:
    /* Read the Hold Register */
    outb(HOLD_REG | counter, cmd_reg);
    data  = inb_p(data_reg) & 0xff;
    data |= (inb_p(data_reg) << 8) & 0xff00;
    
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

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t ctr20HD_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  int minor;
  int board = 0;
  int chip = 0;   /* 0 = A chip,   1 = B chip  2 = C chip  3 = D chip */
  int counter = 0;   /* counter number 0 - 5 */
  u16 base2;
  u16 wReg;
  u16 cmd_reg = 0;
  u16 data_reg= 0;
  u8 bReg;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = iminor(iNode); 
  board = BOARD(minor);                           /* get which board   */
  chip = CHIP(minor);
  counter = CHAN(minor);
  base2 = BoardData[board].base2;

  #ifdef DEBUG
    printk("ctr20HD_write: minor = %d  counter = %d\n", minor, counter);
  #endif


  /* write to LOAD register */
  switch (chip) {
    case 0x0:  /* chip A */
      #ifdef DEBUG
      printk("ctr20HD_write Counter A: minor = %d counter = %d\n", minor, counter);
      #endif
      cmd_reg = CMD_REG_A;
      data_reg = DATA_REG_A;
      break;      
    case 0x1:  /* chip B */
      #ifdef DEBUG
      printk("ctr20HD_write Counter B: minor = %d counter = %d\n", minor, counter);
      #endif
      cmd_reg = CMD_REG_B;
      data_reg = DATA_REG_B;
      break;      
    case 0x2:  /* chip C */
      #ifdef DEBUG
      printk("ctr20HD_write Counter C: minor = %d counter = %d\n", minor, counter);
      #endif
      cmd_reg = CMD_REG_C;
      data_reg = DATA_REG_C;
      break;      
    case 0x3:  /* chip D */
      #ifdef DEBUG
      printk("ctr20HD_write Counter D: minor = %d counter = %d\n", minor, counter);
      #endif
      cmd_reg = CMD_REG_D;
      data_reg = DATA_REG_D;
      break;      
  }
    
  switch (Counter[board][chip][counter].mode) {
    case CTR20HD_COUNTER:       // set mode D for Count #counter 
      outb_p(MODE_REG | counter, cmd_reg);
      outb_p(COUNTUP | RECYCLE, data_reg);
      get_user(wReg, (u16*) buf);
      bReg = wReg >> 8;
      outb_p(bReg, data_reg);       /* SRC #counter */

      /* Select the desired LOAD Register */
      bReg = counter | LOAD_REG;
      outb_p (bReg, cmd_reg);
      
      /* Load the desired data into the register */
      bReg = 0;
      outb_p (bReg, data_reg);    /* Low Byte */
      outb_p (bReg, data_reg);    /* High Byte */
      
      /* LOAD and ARM the counter now */
      bReg = LOAD_ARM | (1 << (counter-1));
      outb_p (bReg, cmd_reg);
      break;
      
    case CTR20HD_FREQUENCY:
      bReg = 0x1 << (counter-1);
      #ifdef DEBUG
        printk("ctr20HD_write: CTR20HD_FREQUENCY counter = %d\n", counter);
      #endif
      outb_p(DISARM | CTR5_BIT | bReg, cmd_reg);     /* disarm CTR5 & CTR minor */
    
      /* below sets mode J for CTR5 */
      outb_p(MODE_REG | CTR5, cmd_reg);   /* Select counter 5                              */
      outb_p(0x62, data_reg);             /* Disable spec. gate, Reload from load or hold, */
      /* Count repeat, binary, Count down, TC toggled  */
      outb_p(0xe, data_reg);              /* No gating, Count on rising edge, Freq4 input  */
    
      outb_p(LOAD_REG | CTR5, cmd_reg);   /* Setting up a variable duty pulse on CTR5      */
      outb_p(5, data_reg);                /* the first "half" is 5 counts (from LOAD reg)  */
      outb_p(0, data_reg);                /* the second "half" is "GateInterval" counts    */
      /* load a 5 count delay into CTR5 load reg       */
    
      /* load gate interval into CTR5 hold reg */
      outb_p(HOLD_REG | CTR5, cmd_reg); 
      outb_p(BoardData[board].gateInterval[chip], data_reg);
      outb_p(BoardData[board].gateInterval[chip] >> 8, data_reg);
	
      /* set Mode Q for Counter #counter */
      outb_p(MODE_REG | counter, cmd_reg);
      outb_p(COUNTUP | SPECIALGATE | RECYCLE, data_reg);
      get_user(wReg, (u16*)buf);
      bReg = (wReg | AHLGATE) >> 8;
      outb_p(bReg , data_reg); 
    
      /* Load 0 into CTR minor's Load Reg. */
      outb_p(LOAD_REG | counter, cmd_reg);
      outb_p(0, data_reg);
      outb_p(0, data_reg);
      
      /*
        NOTE: To setup a 32 bit counter, use CTR3 as the gate.
        Setup CTR3 as CTR4 as setup above.  Then setup CTR4 to use Mode E with
        it's input from SigSource (as for CTR5 above) and CTR3 as the gate.
        Finally setup CTR5 using Mode B, but with its source as TCN-1 (the TC
        output from CTR4 which will pulse each time CTR4 wraps) and also gated
        by CTR3.  When done, CTR4 holds the low 16 bits and CTR5 holds the
        high 16 bits.
      */
    
      bReg = 1 << (counter-1);
      outb_p(LOAD | CTR5_BIT | bReg, cmd_reg);      /* Load CTR5 & CTR minor */
      outb_p(CLEAR_OUTPUT | CTR5, cmd_reg);         /* set TC Low            */
      outb_p (ARM | CTR5_BIT | bReg, cmd_reg);      /* Go ...                */
    
      /* Load 1 into CTR minor's Load Reg. */
      outb_p(LOAD_REG | counter, cmd_reg);
      outb_p(1, data_reg);
      outb_p(0, data_reg);
      break;
    default:
      break;
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

static long ctr20HD_ioctl(struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  struct inode *iNode = filePtr->f_dentry->d_inode;
  int minor = iminor(iNode);
  int board = 0;
  int chip = 0;   /* 0 = A chip,   1 = B chip  2 = C chip  3 = D chip */
  int counter = 0;   /* counter number 0 - 5 */
  u16 base2;
  u16 cmd_reg = 0;
  u16 data_reg= 0;
  u8 *ptr = (u8*) &arg;
  u8 bReg;
  int size = _IOC_SIZE(cmd);       /* the size bitfield in cmd */
  int err = 0;

  board = BOARD(minor);            /* get which board   */
  chip = CHIP(minor);              /* select which c9513 chip */
  counter = CHAN(minor);           /* select which couter */
  base2 = BoardData[board].base2;
  
  #ifdef DEBUG
     printk("ctr20HD_ioctl: minor = %d\n", minor);
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

  switch (chip) {
    case 0x0:
      cmd_reg = CMD_REG_A;
      data_reg = DATA_REG_A;
      break;
    case 0x1:
      cmd_reg = CMD_REG_B;
      data_reg = DATA_REG_B;
      break;
    case 0x2:
      cmd_reg = CMD_REG_C;
      data_reg = DATA_REG_C;
      break;
    case 0x3:
      cmd_reg = CMD_REG_D;
      data_reg = DATA_REG_D;
      break;
  }

  switch (cmd) {
    case LOAD_MASTER_MODE_REG:
      outb_p(MASTER_MODE_REG, cmd_reg);        /* Select Master Mode Register */
      outb_p(*(ptr++), data_reg);
      outb_p(*ptr, data_reg);
      BoardData[board].mm[chip] = (u16) arg;
      break;
    case LOAD_ALARM_REG1:
      outb_p(ALARM_REG1, cmd_reg);
      outb_p(*(ptr++), data_reg);
      outb_p(*ptr, data_reg);
      break;
    case LOAD_ALARM_REG2:
      outb_p(ALARM_REG2, cmd_reg);
      outb_p(*(ptr++), data_reg);
      outb_p(*ptr, data_reg);
      break;
    case LOAD_MODE_REG:
      outb_p(counter|MODE_REG, cmd_reg);         /* Select Counter Mode Register */
      outb_p(*(ptr++), data_reg);
      outb_p(*ptr, data_reg);
      Counter[board][chip][counter].cm = (u16) arg;
      break;
    case LOAD_HOLD_REG:
      outb_p(counter|HOLD_REG, cmd_reg);         /* Select Counter Hold Register */
      outb_p(*(ptr++), data_reg);
      outb_p(*ptr, data_reg);
      Counter[board][chip][counter].hr = (u16) arg;
      break;
    case LOAD_LOAD_REG:
      outb_p(counter|LOAD_REG, cmd_reg);
      outb_p(*(ptr++), data_reg);
      outb_p(*ptr, data_reg);
      break;
    case LOAD_CMD_REG:
      outb_p( (u8) arg, cmd_reg);
      break;
    case GET_MASTER_MODE_REG:
      put_user((long) BoardData[board].mm[chip], (long*) arg);
      break;
    case GET_COUNTER_MODE_REG:
      put_user((long) Counter[board][chip][counter].cm, (long*) arg);
      break;
    case GET_STATUS_REG:
      bReg = inb(cmd_reg);
      put_user((long) bReg, (long*) arg);
      break;
    case SET_GATE_INTERVAL:
      BoardData[board].gateInterval[chip] = (u16) arg;
      break;
    case SET_CLOCK_INPUT:
      switch(chip) {
        case 0x0:  // Chip A
	  BoardData[board].clockSelectReg_AB &= ~CLK_A(7);
  	  BoardData[board].clockSelectReg_AB |= CLK_A(arg);
   	  outb_p(BoardData[board].clockSelectReg_AB, CNT_AB_INT);
	  break;
        case 0x1:  // Chip B
	  BoardData[board].clockSelectReg_AB &= ~CLK_B(7);
  	  BoardData[board].clockSelectReg_AB |= CLK_B(arg);
     	  outb_p(BoardData[board].clockSelectReg_AB, CNT_AB_INT);
	  break;
        case 0x2:  // Chip C
	  BoardData[board].clockSelectReg_CD &= ~CLK_C(7);
  	  BoardData[board].clockSelectReg_CD |= CLK_C(arg);
      	  outb_p(BoardData[board].clockSelectReg_CD, CNT_AB_INT);
	  break;
        case 0x3:  // Chip D
	  BoardData[board].clockSelectReg_CD &= ~CLK_D(7);
  	  BoardData[board].clockSelectReg_CD |= CLK_D(arg);
	  outb_p(BoardData[board].clockSelectReg_CD, CNT_AB_INT);
	  break;
      }
      break;
    case INT_POLARITY:
      switch(chip) {
        case 0x0:  // Chip A
        case 0x1:  // Chip B
	  BoardData[board].clockSelectReg_AB &= ~INTAB_POL;
	  BoardData[board].clockSelectReg_AB |= (arg & 0x1) << 7;
 	  outb_p(BoardData[board].clockSelectReg_AB, CNT_AB_INT);
	  break;
        case 0x2:  // Chip C
        case 0x3:  // Chip D
	  BoardData[board].clockSelectReg_CD &= ~INTCD_POL;
	  BoardData[board].clockSelectReg_CD |= (arg & 0x1) << 7;
	  outb_p(BoardData[board].clockSelectReg_CD, CNT_CD_INT);
	  break;
      }
      break;
  case SET_SQUARE_FREQ:
      /* assume base frequency is 5 MHz default */
      bReg = 1<<(counter-1);
      outb_p(DISARM | bReg, cmd_reg);            // disarm counter         
      outb_p(MODE_REG | counter, cmd_reg);       // select mode register
      if (arg  > 100) {
        outb_p(0x62, data_reg);                  // Disable spec. gate, Reload from load or hold, 
                                                 // Count repeat, binary, Count down, TC toggled
        outb_p(0x0b, data_reg);                  // No gating, Count on rising edge, Freq1 input
        arg = 2500000 / arg;                     // convert Hz to time
      } else {
        outb_p(0x62, data_reg);                  // Disable spec. gate, Reload from load or hold, 
                                                 // Count repeat, binary, Count down, TC toggled
        outb_p(0x0e, data_reg);                  // No gating, Count on rising edge, Freq4 input
        arg = 2500 / arg;                        // convert Hz to time
      }
      outb_p(LOAD_REG | counter, cmd_reg);       // Setting up a variable duty pulse on CTR5
      outb_p(*(ptr), data_reg);                  // first half of duty cycle
      outb_p(*(ptr+1), data_reg);             
      outb_p(HOLD_REG | counter, cmd_reg);       // Setting up a variable duty pulse on CTR5
      outb_p(*(ptr), data_reg);                  // second half of duty cycle
      outb_p(*(ptr+1), data_reg);             
      outb_p(LOAD | bReg, cmd_reg);           
      outb_p(CLEAR_OUTPUT | counter, cmd_reg);   // set TC Low
      outb_p (ARM | bReg, cmd_reg);              // Go ...    
      break;
  }  /* end switch */
  return 0;
}

/***************************************************************************
 *
 *  Alarm handler to handle situations where an interrupt is missed.
 *
 ***************************************************************************/

static void ctr20HD_TimerHandler( unsigned long unused)
{
}

/***************************************************************************
 *
 * Interrupt handler.
 *
 ***************************************************************************/

static irqreturn_t ctr20HD_Interrupt(int irq, void *dev_id)
{
  BoardRec *boardData = dev_id;
  u32 pci9052_intreg;
  u16 base1 = boardData->base1;

  spin_lock_irq(&ctr20HD_lock);

  pci9052_intreg = inl(INTERRUPT_REG);
  if (pci9052_intreg & INT_AB) {    // we got interrupted from CHIP A or B
    outl(pci9052_intreg | INTCLR_AB, INTERRUPT_REG);

  #ifdef DEBUG
      printk("Entering ctr20HD_AB_ReadInterrupt(). irq = %d\n", irq );
  #endif
  }

  if (pci9052_intreg & INT_CD) {    // we got interrupted chip C or D
    outl(pci9052_intreg | INTCLR_CD, INTERRUPT_REG);

  #ifdef DEBUG
      printk("Entering ctr20HD_B_ReadInterrupt(). irq = %d\n", irq );
  #endif
  }

  spin_unlock_irq(&ctr20HD_lock);
  return IRQ_HANDLED;
}
