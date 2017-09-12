/***************************************************************************
 Copyright (C) 2003  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-CTR10, is free software; you can redistribute it
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
    c9513.c for PCI-CTR10
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
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "pci-ctr10.h"
#include "c9513.h"

#ifdef CONFIG_PCI
    #include <linux/pci.h>    /* defines pci_present() */
#endif

/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init ctr10_init(void);
static void __exit ctr10_exit (void);
static ssize_t ctr10_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t ctr10_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int ctr10_open(struct inode *iNode, struct file *filePtr);
static int ctr10_close(struct inode *iNode, struct file *filePtr);
static int ctr10_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg);
static void ctr10_TimerHandler(unsigned long unused);
static void ctr10_Interrupt(int irq, void *dev_id, struct pt_regs *regs);

module_init(ctr10_init);
module_exit(ctr10_exit);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-CTR10 module");
MODULE_LICENSE("GPL");

/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/

static spinlock_t ctr10_lock;
static BoardRec BoardData[MAX_BOARDS][NCHIP];               /* Board specific information      */
static CounterRec  Counter[MAX_BOARDS][NCHIP][NCOUNTERS+1]; /* Counter specific information    */
static DIO_ChanRec ChanDIO[MAX_BOARDS][NUM_DIO];            /* DIO Channel specific information*/
static int MajorNumber = DEFAULT_MAJOR_DEV;                 /* Major number compiled in        */
static DECLARE_WAIT_QUEUE_HEAD(ctr10_wait);                 /* wait semaphore                  */
static struct timer_list TimerList = {function: ctr10_TimerHandler};
static int NumBoards = 0;                                   /* number of boards found          */

/***************************************************************************
 *
 *
 ***************************************************************************/

static struct file_operations ctr10_fops = {
  owner:          THIS_MODULE,
  read:           ctr10_read,
  write:          ctr10_write,
  ioctl:          ctr10_ioctl,
  open:           ctr10_open,
  release:        ctr10_close,
};

  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int ctr10_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void ctr10_remove_one(struct pci_dev *pdev);

static struct pci_device_id ctr10_id_tbl[] __devinitdata = {
  {
  vendor:                  PCI_VENDOR_ID_CBOARDS,
  device:                  PCI_DEVICE_ID_CBOARDS_CTR10,
  subvendor:               PCI_ANY_ID,
  subdevice:               PCI_ANY_ID,
  class:                   0,
  class_mask:              0,
  },         { /* all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, ctr10_id_tbl);

static struct pci_driver ctr10_driver = {
  name:         "pci-ctr10",
  id_table:     ctr10_id_tbl,
  probe:        ctr10_init_one,
  remove:       ctr10_remove_one,
};

/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/
static int __init ctr10_init(void) 
{
  int err;

  /* Register as a device with kernel.  */

  if ((err = register_chrdev(MajorNumber, "pci-ctr10", &ctr10_fops))) {
    printk("%s: Failure to load module. Major Number = %d error %d\n",
	   ADAPTER_ID, MajorNumber, -err);
    return err; 
  }

  spin_lock_init(&ctr10_lock);
  err = pci_module_init(&ctr10_driver);	

  return  err;	
}

static int __devinit ctr10_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
  int i,j;
  u16 base1;
  u16 base2;
  u8 bReg;

  if (NumBoards >= MAX_BOARDS) {
    printk("ctr10_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit c9513.h.\n", NumBoards);
    return -ENODEV;
  }

  /*GETTING BASE ADDRESS 1 */
  base1 = BoardData[NumBoards][0].base1 =  (u16) pci_resource_start(pdev, 1);
  base1 = BoardData[NumBoards][1].base1 =  (u16) pci_resource_start(pdev, 1);

  /*GETTING BASE ADDRESS 2 */
  base2 = BoardData[NumBoards][0].base2 = (u16) pci_resource_start(pdev, 2);
  base2 = BoardData[NumBoards][1].base2 = (u16) pci_resource_start(pdev, 2);

  /* Register interrupt handler. */
  BoardData[NumBoards][0].irq = pdev->irq;
  BoardData[NumBoards][1].irq = pdev->irq;
  if (request_irq(pdev->irq, ctr10_Interrupt, (SA_INTERRUPT | SA_SHIRQ), 
		  "pci-ctr10", (void *) &BoardData[NumBoards])) {
    /* No free irq found! cleanup and exit */
    printk("%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData[NumBoards][0].irq);
    goto err_out_0;
  }

  if (pci_enable_device(pdev))          
    goto err_out_1;

  /* Initialize the 9513 chip */
  outb_p(MASTER_RESET, CMD_REG_A);      /* MASTER RESET */
  outb_p(MASTER_MODE_REG, CMD_REG_A);   /* Select Master Mode Register */
  outb_p(MASTER_RESET, CMD_REG_B);      /* MASTER RESET */
  outb_p(MASTER_MODE_REG, CMD_REG_B);   /* Select Master Mode Register */
  bReg = TOD_DISABLED | MM_SRC1;
  outb_p(bReg, DATA_REG_A);
  outb_p(bReg, DATA_REG_B);
  bReg = 0xc0;        /* BCD Division, Disable increment, 8-Bit bus, FOUT On */
  outb_p(bReg, DATA_REG_A);
  outb_p(bReg, DATA_REG_B);

  for ( j = 0; j < NCHIP; j++ ) {
    BoardData[NumBoards][j].mm = BCD_DIV | DISABLE_INCREMENT | TOD_DISABLED | MM_SRC1;
    BoardData[NumBoards][j].gateInterval = GATEINTERVAL;
    BoardData[NumBoards][j].useCounter = 0;
  }

  for ( i = 1; i < NCOUNTERS; i++ ) {
    for ( j = 0; j < NCHIP; j++ ) {
      Counter[NumBoards][j][i].open = FALSE;
      Counter[NumBoards][j][i].count = 0;
      Counter[NumBoards][j][i].chan = i+1;
      Counter[NumBoards][j][i].cm = 0;
      Counter[NumBoards][j][i].mode = 0;
    }
  }

  for ( i = 0; i < NUM_DIO; i++ ) {
    ChanDIO[NumBoards][i].open = FALSE;
  }

  printk("%s: address=%#x IRQ=%d.", ADAPTER_ID, BoardData[NumBoards][0].base2, BoardData[NumBoards][0].irq);
  printk(" 4/30/04 wjasper@ncsu.edu\n");

  NumBoards++;
  return 0;

err_out_1:
  free_irq(pdev->irq, (void *) &BoardData[NumBoards]);
err_out_0:
  return -ENODEV;
}

/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-ctr10" is run on the command line.
 *
 ***************************************************************************/
void __exit ctr10_exit(void)
{
  pci_unregister_driver(&ctr10_driver);

  if (unregister_chrdev(MajorNumber, "pci-ctr10") != 0) {
    printk("%s: cleanup_module failed.\n", ADAPTER_ID);
  } else {
    printk("%s: module removed.\n", ADAPTER_ID);
  }
}

static void ctr10_remove_one(struct pci_dev *pdev)
{
  if (MOD_IN_USE) {
    printk("%s: device busy, remove delayed.\n", ADAPTER_ID);
    return;
  }

  NumBoards--;
  free_irq(BoardData[NumBoards][0].irq, (void *) &BoardData[NumBoards][0]);

  #ifdef DEBUG
    printk("ctr10_remove_one: Board #%d removed.\n", NumBoards);
  #endif
}     /* ctr10_remove_one() */

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int ctr10_open(struct inode *iNode, struct file *filePtr)
{
  u16 base2;
  int board = 0;
  int chip = 0;   /* 0 = A chip,   1 = B chip */
  int chan = 0;   /* counter number 0 - 5 */
  int port = 0;   /* DIO port number 0 or 1 */
  int minor = MINOR(iNode->i_rdev);

  board = BOARD(minor);                           /* get which board   */
  chip = CHIP(minor);
  chan = CHAN(minor);
  base2 = BoardData[board][0].base2;

  /*
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */

  if ( (minor & 0x7) == 0 ) {              /* DIO port */
    port = (minor >> 3) & 0x3;
    MOD_INC_USE_COUNT;
    ChanDIO[board][port].open = TRUE;     /* The device is open */
  } else {
    if ( Counter[board][chip][chan].open == TRUE ) {
      return -EBUSY;
    }
    MOD_INC_USE_COUNT;
    Counter[board][chip][chan].open = TRUE;              /* The device is open */
    Counter[board][chip][chan].mode = filePtr->f_flags;  /* set acquisition mode */

    /* if we are in frequency counting mode, set counter 5 busy */
    if ( Counter[board][chip][chan].mode == CTR10_FREQUENCY ) {
      BoardData[board][chip].useCounter++;
      Counter[board][chip][CTR5].open = TRUE;
    }
  }

  #ifdef DEBUG
    printk("%s: open(): minor %d mode %d\n", ADAPTER_ID, minor,
	   Counter[board][chip][chan].mode);
   #endif

  return 0;
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int ctr10_close(struct inode *iNode, struct file *filePtr)
{
  u8 bReg;
  u16 base2;
  int board = 0;
  int chan = 0;   /* counter number 0 - 5 */
  int chip = 0;   /* 0 = A chip,   1 = B chip */
  int port = 0;   /* DIO port number 0 or 1 */
  int minor = MINOR(iNode->i_rdev);
  u16 cmd_reg = 0;
  u16 data_reg= 0;

  board = BOARD(minor);                           /* get which board   */
  chip = CHIP(minor);
  chan = CHAN(minor);
  base2 = BoardData[board][0].base2;

  MOD_DEC_USE_COUNT;

  if ( (minor & 0x7) == 0 ) {              /* DIO port */
    port = (minor >> 3) & 0x3;
    ChanDIO[board][port].open = FALSE;
  } else {
    if (chip)  {                 /* chip B */
      cmd_reg = CMD_REG_B;
      data_reg = DATA_REG_B;
    } else {                     /* chip A */
      cmd_reg = CMD_REG_A;
      data_reg = DATA_REG_A;
    }

    Counter[board][chip][chan].open = FALSE;

    /* if we are in frequency counting mode, disarm counter */
    if ( Counter[board][chip][chan].mode == CTR10_FREQUENCY ) {
      bReg = 1 << (chan-1);
      outb_p(DISARM | bReg, cmd_reg);         /* disarm  CTR minor */
      BoardData[board][chip].useCounter--;
      if (BoardData[board][chip].useCounter == 0 ) {
        Counter[board][chip][CTR5].open = FALSE;
        outb_p(DISARM | CTR5_BIT, cmd_reg);     /* disarm CTR5  */
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

static ssize_t ctr10_read(struct file *filePtr, char *buf, size_t count, loff_t *off)
{
  int minor;
  int port = 0;
  int board = 0;
  int chan = 0;   /* counter number 0 - 5 */
  int chip = 0;   /* 0 = A chip,   1 = B chip */
  u16 data;
  u16 base2;
  u8 bReg;
  u16 cmd_reg = 0;
  u16 data_reg= 0;
  struct inode *iNode = filePtr->f_dentry->d_inode;

  minor = MINOR(iNode->i_rdev);
  board = BOARD(minor);                           /* get which board   */
  chip = CHIP(minor);
  chan = CHAN(minor);
  base2 = BoardData[board][0].base2;

  if ( (minor & 0x7) == 0 ) {              /* DIO port */
    port = (minor >> 3) & 0x3;
    if ( port ) {
      bReg = inb(DIO_IN_B);
      #ifdef DEBUG
      printk("ctr10_read: DIOB, minor = %d  bReg = %#x\n", minor, bReg);
      #endif
    } else {
      bReg = inb(DIO_IN_A);
      #ifdef DEBUG
      printk("ctr10_read: DIOA, minor = %d  bReg = %#x\n", minor, bReg);
      #endif
    }
    put_user(bReg, (u8*) buf);
    return 1;
  }

  if ( chan > 0 && chan <= NCOUNTERS ) {  
    if (chip) {   /* chip B */
      #ifdef DEBUG
      printk("ctr10_read Counter B: minor = %d chan = %d\n", minor, chan);
      #endif
      cmd_reg = CMD_REG_B;
      data_reg = DATA_REG_B;
    } else {                     /* chip A */
      #ifdef DEBUG
      printk("ctr10_read Counter A: minor = %d chan = %d\n", minor, chan);
      #endif
      cmd_reg = CMD_REG_A;
      data_reg = DATA_REG_A;
    }

    switch (Counter[board][chip][chan].mode) {
      case CTR10_COUNTER:            // read the current count from the counter 
        /* Disarm and Save counters   */
	outb((DISARM_LATCH | 1<<(chan-1)), cmd_reg);

	/* Read the Hold Register */
        outb((chan | HOLD_REG), cmd_reg);        /* Fetch Hold Reg */
	data = inb_p(data_reg) & 0xff;           /* Low Byte */
        data |= (inb_p(data_reg) << 8) & 0xff00; /* High Byte */

        /* Write data to user space */
        put_user(data, (u16*) buf);
        #ifdef DEBUG
          printk("ctr10_read Counter : minor = %d data = %d\n", minor, data);
        #endif

        /* BoardData.busy = FALSE; */
        break;
       
      case CTR10_FREQUENCY:
        /* Read the Hold Register */
        outb(HOLD_REG | chan, cmd_reg);
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
  
  return 0;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t ctr10_write(struct file *filePtr, const char *buf, size_t count, loff_t *off)
{
  int minor;
  int port = 0;
  int board = 0;
  int chip = 0;   /* 0 = A chip,   1 = B chip */
  int chan = 0;   /* counter number 0 - 5 */
  u16 base2;
  u16 wReg;
  u16 cmd_reg = 0;
  u16 data_reg= 0;
  u8 bReg;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = MINOR(iNode->i_rdev); 
  board = BOARD(minor);                           /* get which board   */
  chip = CHIP(minor);
  chan = CHAN(minor);
  base2 = BoardData[board][0].base2;

#ifdef DEBUG
  printk("ctr10_write: minor = %d  chan = %d\n", minor, chan);
#endif

  if ( (minor & 0x7) == 0 ) {              /* DIO port */
    port = (minor >> 3) & 0x3;
    get_user(bReg, (u8*)buf);
    if ( port ) {
      outb(bReg, DIO_OUT_B);
      #ifdef DEBUG
      printk("ctr10_write: DIOB, minor = %d  bReg = %#x\n", minor, bReg);
      #endif
    } else {
      outb(bReg, DIO_OUT_A);
      #ifdef DEBUG
      printk("ctr10_write: DIOA, minor = %d  bReg = %#x\n", minor, bReg);
      #endif
    }
    return 1;
  }

  /* write to LOAD register */
  if ( chan > 0 && chan <= NCOUNTERS ) {  
    if (chip) {                 /* chip B */
      cmd_reg = CMD_REG_B;
      data_reg = DATA_REG_B;
    } else {                    /* chip A */
      cmd_reg = CMD_REG_A;
      data_reg = DATA_REG_A;
    }
    switch (Counter[board][chip][chan].mode) {
      case CTR10_COUNTER:       // set mode D for Count #chan 
        outb_p(MODE_REG | chan, cmd_reg);
        outb_p(COUNTUP | RECYCLE, data_reg);
        get_user(wReg, (u16*) buf);
        bReg = wReg >> 8;
        outb_p(bReg, data_reg);       /* SRC #chan */

        /* Select the desired LOAD Register */
        bReg = chan | LOAD_REG;
        outb_p (bReg, cmd_reg);
      
        /* Load the desired data into the register */
        bReg = 0;
        outb_p (bReg, data_reg);    /* Low Byte */
        outb_p (bReg, data_reg);    /* High Byte */
 
        /* LOAD and ARM the counter now */
        bReg = LOAD_ARM | (1 << (chan-1));
        outb_p (bReg, cmd_reg);
        break;
      
      case CTR10_FREQUENCY:
        bReg = 0x1 << (chan-1);
        #ifdef DEBUG
  	  printk("ctr10_write: CTR10_FREQUENCY chan = %d\n", chan);
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
        outb_p(BoardData[board][chip].gateInterval, data_reg);
        outb_p(BoardData[board][chip].gateInterval >> 8, data_reg);

        /* set Mode Q for Counter #chan */
        outb_p(MODE_REG | chan, cmd_reg);
        outb_p(COUNTUP | SPECIALGATE | RECYCLE, data_reg);
        get_user(wReg, (u16*)buf);
        bReg = (wReg | AHLGATE) >> 8;
        outb_p(bReg , data_reg); 
      
        /* Load 0 into CTR minor's Load Reg. */
        outb_p(LOAD_REG | chan, cmd_reg);
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
      
        bReg = 1 << (chan-1);
        outb_p(LOAD | CTR5_BIT | bReg, cmd_reg);      /* Load CTR5 & CTR minor */
        outb_p(CLEAR_OUTPUT | CTR5, cmd_reg);         /* set TC Low            */
        outb_p (ARM | CTR5_BIT | bReg, cmd_reg);      /* Go ...                */
      
        /* Load 1 into CTR minor's Load Reg. */
        outb_p(LOAD_REG | chan, cmd_reg);
        outb_p(1, data_reg);
        outb_p(0, data_reg);
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

static int ctr10_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, unsigned long arg)
{
  int minor = MINOR(iNode->i_rdev);
  int board = 0;
  int chip = 0;   /* 0 = A chip,   1 = B chip */
  int chan = 0;   /* counter number 0 - 5 */
  u32 pci9052_status;
  u16 base2;
  u16 cmd_reg = 0;
  u16 data_reg= 0;
  u8 *ptr = (u8*) &arg;
  u8 bReg;
  int size = _IOC_SIZE(cmd);       /* the size bitfield in cmd */
  int err = 0;

  board = BOARD(minor);                           /* get which board   */
  chip = CHIP(minor);
  chan = CHAN(minor);
  base2 = BoardData[board][0].base2;
  
  #ifdef DEBUG
     printk("ctr10_ioctl: minor = %d\n", minor);
  #endif

  /*
   * extract the type and number bitfields, and don't decode
   * wrong cmds;  return EINVAL before verify_area()
   */
  
  if (_IOC_TYPE(cmd) != IOCTL_MAGIC) return -EINVAL;
  if (_IOC_NR(cmd) > IOCTL_MAXNR)  return -EINVAL;
  
  if (_IOC_DIR(cmd) & _IOC_READ) {
    err = verify_area(VERIFY_WRITE, (void *)arg, size);
  } else if (_IOC_DIR(cmd) & _IOC_WRITE) {
    err = verify_area(VERIFY_READ, (void *)arg, size);
  }
  if (err) return err;

  if ( chan > 0 && chan <= NCOUNTERS ) {
    if (chip) {   /* chip B */
      cmd_reg = CMD_REG_B;
      data_reg = DATA_REG_B;
    } else {                     /* chip A */
      cmd_reg = CMD_REG_A;
      data_reg = DATA_REG_A;
    }
    switch (cmd) {
      case LOAD_MASTER_MODE_REG:
        outb_p(MASTER_MODE_REG, cmd_reg);        /* Select Master Mode Register */
        outb_p(*(ptr++), data_reg);
        outb_p(*ptr, data_reg);
        BoardData[board][chip].mm = (u16) arg;
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
        outb_p(chan|MODE_REG, cmd_reg);         /* Select Counter Mode Register */
        outb_p(*(ptr++), data_reg);
        outb_p(*ptr, data_reg);
        Counter[board][chip][chan].cm = (u16) arg;
        break;
      case LOAD_HOLD_REG:
        outb_p(chan|HOLD_REG, cmd_reg);         /* Select Counter Hold Register */
        outb_p(*(ptr++), data_reg);
        outb_p(*ptr, data_reg);
        Counter[board][chip][chan].hr = (u16) arg;
        break;
      case LOAD_LOAD_REG:
	outb_p(chan|LOAD_REG, cmd_reg);
        outb_p(*(ptr++), data_reg);
        outb_p(*ptr, data_reg);
        break;
      case LOAD_CMD_REG:
        outb_p( (u8) arg, cmd_reg);
        break;
      case GET_MASTER_MODE_REG:
        put_user((long) BoardData[board][chip].mm, (long*) arg);
        break;
      case GET_COUNTER_MODE_REG:
        put_user((long) Counter[board][chip][chan].cm, (long*) arg);
        break;
      case GET_STATUS_REG:
        bReg = inb(cmd_reg);
        put_user((long) bReg, (long*) arg);
        break;
      case SET_GATE_INTERVAL:
        BoardData[board][chip].gateInterval = (u16) arg;
        break;
      case SET_CLOCK_INPUT:
        pci9052_status = inl(STATUS_REG);
        pci9052_status &= ~(OUT0 | OUT1);
        pci9052_status |= arg & (OUT0 | OUT1);
	outl(pci9052_status, STATUS_REG);
        break;
      case SET_SQUARE_FREQ:
	arg =  2500/arg;                            // convert Hz to time.
        bReg = 1<<(chan-1);
        outb_p(DISARM | bReg, cmd_reg);            // disarm counter         
        outb_p(MODE_REG | chan, cmd_reg);          // select mode register   
        outb_p(0x62, data_reg);                    // Disable spec. gate, Reload from load or hold, 
	                                           // Count repeat, binary, Count down, TC toggled
	outb_p(0x0e, data_reg);                    // No gating, Count on rising edge, Freq4 input
        outb_p(LOAD_REG | chan, cmd_reg);          // Setting up a variable duty pulse on CTR5
        outb_p(*(ptr), data_reg);                  // first half of duty cycle
	outb_p(*(ptr+1), data_reg);             
        outb_p(HOLD_REG | chan, cmd_reg);          // Setting up a variable duty pulse on CTR5
        outb_p(*(ptr), data_reg);                  // second half of duty cycle
	outb_p(*(ptr+1), data_reg);             
        outb_p(LOAD | bReg, cmd_reg);           
        outb_p(CLEAR_OUTPUT | chan, cmd_reg);      // set TC Low
        outb_p (ARM | bReg, cmd_reg);              // Go ...    
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

static void ctr10_TimerHandler( unsigned long unused)
{
}

/***************************************************************************
 *
 * Interrupt handler.
 *
 ***************************************************************************/

static void ctr10_Interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
  BoardRec *boardData = dev_id;
  u32 pci9052_intreg;
  u16 base1 = boardData->base1;

  spin_lock_irq(&ctr10_lock);

  pci9052_intreg = inl(INTERRUPT_REG);
  if ( pci9052_intreg & INT_A ) {    // we got interrupted chip A
    outl( pci9052_intreg | INTCLR_A, INTERRUPT_REG);

  #ifdef DEBUG
      printk("Entering ctr10_A_ReadInterrupt(). irq = %d\n", irq );
  #endif
  }

  if ( pci9052_intreg & INT_B ) {    // we got interrupted chip B
    outl( pci9052_intreg | INTCLR_B, INTERRUPT_REG);

  #ifdef DEBUG
      printk("Entering ctr10_B_ReadInterrupt(). irq = %d\n", irq );
  #endif
  }

  spin_unlock_irq(&ctr10_lock);

  return;
}
