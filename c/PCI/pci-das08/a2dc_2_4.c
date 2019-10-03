/*
 * Copyright (C) 2001  Warren J. Jasper
 * All rights reserved.
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
 */

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
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/config.h>       /* for CONFIG_PCI */
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

static int  SoftRead(WORD *outBuff, ADC_ChanRec *chan);
static int  TrigRead(WORD *outBuff, ADC_ChanRec *chan);
static void LoadCounter0( LONG value );
static void LoadCounter1( LONG value );
static void LoadCounter2( LONG value );

static int __init das08_init(void);
static void __exit das08_exit (void);
static ssize_t das08_read(struct file *filePtr, char *buf, size_t count, loff_t *off);
static ssize_t das08_write(struct file *filePtr, const char *buf, size_t count, loff_t *off);
static int das08_open(struct inode *iNode, struct file *filePtr);
static int das08_close(struct inode *iNode, struct file *filePtr);
static int das08_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, LONG arg);
static void das08_ReadInterrupt(int irq, void *dev_id, struct pt_regs *fp);
static void das08_TimerHandler(LONG);

MODULE_AUTHOR("Warren J. Jasper  <wjasper@ncsu.edu>");
MODULE_DESCRIPTION("Driver for the PCI-DAS08  module");
#ifdef MODULE_LICENSE
  MODULE_LICENSE("GPL");
#endif

module_init(das08_init);
module_exit(das08_exit);


/***************************************************************************
 *
 * Global data.
 *
 ***************************************************************************/
static WORD base1;                          /* base register 0 address           */
static WORD base2;                          /* base register 1 address           */
static int MajorNumber = DEFAULT_MAJOR_DEV; /* Major number compiled in          */
static int WordsToRead;                     /* number of conversions until done  */
static ADC_ChanRec *CurrChan = NULL;        /* pointer to ChanADC[minor]         */
static ADC_ChanRec ChanADC[AD_CHANNELS];    /* ADC Channel specific information  */
static BoardRec BoardData;                  /* Board specific information        */
static WORD KernBuff[MAX_COUNT];            /* Kernel buffer where samples are   */
                                            /* saved before write to user space  */
static WORD* KernBuffPtr;                   /* pointer to kernbuf                */
static DECLARE_WAIT_QUEUE_HEAD(das08_wait);
static struct timer_list TimerList = {function: das08_TimerHandler};


/***************************************************************************
 *
 *
 ***************************************************************************/

static struct file_operations das08_fops = {
  owner:          THIS_MODULE,
  read:           das08_read,
  write:          das08_write,
  ioctl:          das08_ioctl,
  open:           das08_open,
  release:        das08_close,
};


#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************************
 *
 * Loads driver. Called when "insmod pci-das08.o" is invoked on the command line.
 *
 *********************************************************************************/

static int __init das08_init(void)
{
  register int i;  
  int pci_index;
  unsigned char pci_bus, pci_device_fn;
  unsigned int pci_ioaddr;
  struct pci_dev *dev = NULL;
  int err;

#ifndef CONFIG_PCI
    /* this should not happen. */
    printk("%s pci not configured with kernel!\n", ADAPTER_ID);
    return -ENODEV;
#endif

  /* Register as a device with kernel.  */

  if ((err = register_chrdev(MajorNumber, "pci-das08", &das08_fops))) {
      printk("%s: Failure to load module. error %d\n", ADAPTER_ID, -err);
      return err;
  } 

  /* Register the region of iospace with kernel and get base address */
  if(pcibios_present()) {
    for ( pci_index = 0; pci_index < MAX_BOARDS; pci_index++ ) {
      if( (dev = pci_find_device( PCI_VENDOR_ID_CBOARDS, 
				  PCI_DEVICE_ID_CBOARDS_DAS08, 
				  dev)) == 0 ) break;
      pci_bus = dev->bus->number;
      pci_device_fn = dev->devfn;

      /* Get PCI_BASE_ADDRESS_1 */
      pci_ioaddr = pci_resource_start(dev, 1);
      if ( pci_ioaddr != base1 ) {
          pci_ioaddr = base1 | 0x1;
          pcibios_write_config_dword( pci_bus, pci_device_fn,
                       PCI_BASE_ADDRESS_1, pci_ioaddr );
      }

      /* Get PCI_BASE_ADDRESS_2 */
      pci_ioaddr = pci_resource_start(dev, 2) ;
      if ( pci_ioaddr != base2 ) {
          pci_ioaddr = base2 | 0x1;
          pcibios_write_config_dword( pci_bus, pci_device_fn,
                        PCI_BASE_ADDRESS_2, pci_ioaddr );
      }

      /* Register interrupt handler. */

/*****************************************
      BoardData.irq = dev->irq;
      if (request_irq(dev->irq, das08_ReadInterrupt, (SA_INTERRUPT | SA_SHIRQ), 
		      "pci-das08", (void *) PCI_VENDOR_ID_CBOARDS)) {
        // No free irq found! cleanup and exit 
        printk("%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData.irq);
        if (unregister_chrdev(MajorNumber, "pci-das08") != 0) { 
          printk("%s: unregister_chrdev() failed.\n", ADAPTER_ID);
        }
        release_region( base1,  BADR1_SIZE );       /* release region */
        release_region( base2,  BADR2_SIZE );       /* release region */
        return -EADDRNOTAVAIL;
        break;
      }
    }
*************************************/
    if ( pci_index == 0 ) {
      printk("%s No device present.\n", ADAPTER_ID);
      return -ENODEV;
    }
  } else {
    printk("%s No pci bios present.\n", ADAPTER_ID);
    return -ENODEV;
  }

  BoardData.base1 = base1;
  BoardData.base2 = base2;

  /* Set all channel structures to show nothing active/open */

  for (i = 0; i < AD_CHANNELS; i++) {
      ChanADC[i].open = FALSE;
      ChanADC[i].chan = i;           /* set channel number */
  }

  init_waitqueue_head(&das08_wait);

  BoardData.busy = FALSE;          /* board ready */
  BoardData.status = 0;

  printk("%s: BASR0=%#x BADR1=%#x IRQ=%d.",ADAPTER_ID, base1, base2, BoardData.irq);
  printk(" 5/1/01 wjasper@ncsu.edu\n");

  return 0;
}

/***************************************************************************
 *
 * Remove driver. Called when "rmmod pci-das08" is run on the command line.
 *
 ***************************************************************************/

void __exit das08_exit(void) 
{
  if (MOD_IN_USE) {
    printk("%s: device busy, remove delayed.\n", ADAPTER_ID);
    return;
  }

  release_region( base1,  BADR1_SIZE );       /* release region */
  release_region( base2,  BADR2_SIZE );       /* release region */

  free_irq(BoardData.irq, (void *) PCI_VENDOR_ID_CBOARDS);
  if (unregister_chrdev(MajorNumber, "pci-das08") != 0) {
    printk("%s: cleanup_module failed.\n", ADAPTER_ID);
  } else {

  #ifdef DEBUG
      printk("%s: module removed.\n", ADAPTER_ID);
  #endif

  }    
}

#ifdef __cplusplus
}
#endif

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int das08_open(struct inode *iNode, struct file *filePtr)
{
  int minor = MINOR(iNode->i_rdev);

  /* 
     check if device is already open: only one process may read from a
     port at a time.  There is still the possibility of two processes 
     reading from two different channels messing things up. However,
     the overhead to check for this may not be worth it.
  */

  if ( minor >= 0 && minor < AD_CHANNELS ) {
    if ( ChanADC[minor].open == TRUE ) {
      return -EBUSY;
    }

    MOD_INC_USE_COUNT;

    ChanADC[minor].open = TRUE;              /* The device is open */
    ChanADC[minor].mode = filePtr->f_flags;  /* set acquisition mode */

    outb_p(0x0, STATUS_REG);

    #ifdef DEBUG
        printk("%s: open(): minor %d mode %d.\n", ADAPTER_ID, minor, ChanADC[minor].mode);
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
  int minor = MINOR(iNode->i_rdev);

  MOD_DEC_USE_COUNT;

  if ( minor >= 0 && minor < AD_CHANNELS ) {
    ChanADC[minor].open = FALSE;
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
  int   stat;
  int   minor;
  register int i;
  
  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = MINOR(iNode->i_rdev);

  if ( BoardData.busy == TRUE ) {   /* if board is in use, return busy */
      return (-EBUSY);
  } else {
      BoardData.busy = TRUE;
  }

    #ifdef DEBUG
       printk("das1602_read: Entering function\n");
    #endif


  if ( minor >= 0 && minor < AD_CHANNELS ) {

    if ( (ChanADC[minor].count = count) > MAX_COUNT ) {
        printk("das1602_read(): requesting too large a count size.\n");
        BoardData.busy = FALSE;
        return (-1);
    }

    /* Read */

    switch (ChanADC[minor].mode) {

      case ADC_SOFT_TRIGGER:
          #ifdef DEBUG
              printk("adc_read(): Entering ADC_SOFT_TRIGGER mode.\n");
          #endif
          if (SoftRead(KernBuff, &ChanADC[minor])) {
              printk("das08_read: SoftRead() failed.\n");
              BoardData.busy = FALSE;
              return(-1);
          } 
          break;
     
      case ADC_EXTERNAL_TRIGGER:
          #ifdef DEBUG
              printk("adc_read(): Entering ADC_EXTERNAL_TRIGGER mode.\n");
          #endif
          if (TrigRead(KernBuff, &ChanADC[minor])) {
              printk("das08_read: TrigRead() failed.\n");
              BoardData.busy = FALSE;
              return(-1);
          } 
          break;
    }

    /* Check that data can be written to file. */

    if ((stat = verify_area(VERIFY_WRITE, buf, sizeof(WORD)*ChanADC[minor].count)) != 0) {
      printk("das08_read: Failed VERIFY_WRITE.\n");
      BoardData.busy = FALSE;
      return -1;
    }

    /* Write data to user space */

    for (i = 0; i < ChanADC[minor].count; i++) {
      KernBuff[i] >>= 4;
    }

    if (ChanADC[minor].count == 1) {
      put_user(*KernBuff, (WORD*) buf);
    } else  {
      copy_to_user(buf, KernBuff, ChanADC[minor].count*sizeof(WORD));
    }

    BoardData.busy = FALSE;
    return(ChanADC[minor].count);     /* return number of samples read */
  } else {
    printk("das08_read: Incorrect minor number (%d).\n", minor);
    BoardData.busy = FALSE;
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

static int das08_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, LONG arg)
{
  int minor = MINOR(iNode->i_rdev);
  BYTE bReg;

  int err = 0;
  int size = _IOC_SIZE(cmd);       /* the size bitfield in cmd */

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
  
  /* global IOCTL CALLS */

  /* ADC Specific  IOCTL CALLS */
  if ( minor >= 0 && minor < AD_CHANNELS ) {
    switch (cmd) {
      case ADC_COUNTER0:
        LoadCounter0( (LONG) arg );
        break;
      case ADC_COUNTER1:
        LoadCounter1( (LONG) arg );
        break;
      case ADC_COUNTER2:
        LoadCounter2( (LONG) arg );
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
        bReg = (BYTE) (arg & 0xf);
        bReg <<= 4;
        BoardData.status &= 0xf;
        BoardData.status |= bReg;
        outb_p(BoardData.status, STATUS_REG);
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

static void das08_TimerHandler(LONG unused)
{
  /* 
     we should only be here if an interrupt occured while 
     we were reading half the FIFO 
  */

  #ifdef DEBUG
      printk("TimerHandler: WordsToRead = %d\n", WordsToRead);
  #endif

  wake_up_interruptible(&das08_wait);  
}

/***************************************************************************
 *
 * Interrupt handler used to service enhanced mode(interrupt) read().
 *
 ***************************************************************************/

static void das08_ReadInterrupt(int irq, void *dev_id, struct pt_regs *fp)
{
  WORD msb, lsb;

  #ifdef DEBUG
      printk("Entering das08_ReadInterrupt(). irq = %d mode = %d  count = %d\n", 
              irq, CurrChan->mode, WordsToRead );
  #endif

  cli();
  if ( WordsToRead == 0 ) return;

  outb_p(0x1, MSB_DATA_BYTE);                 /* Force conversion */
  del_timer(&TimerList);

  /* this board is so slow that you need to set the MUX before the conversion is done! */
  outb_p(CurrChan->chan |  BoardData.status, STATUS_REG);  /* Set MUX and Interrupt */

  /* poll until done (Yech!) */
  while (inb_p(STATUS_REG) & EOC);

  lsb = (WORD) inb(LSB_AND_CHNLS);            /* Sample: ADC -> kernel buffer */
  msb = (WORD) inb(MSB_DATA_BYTE);

  *KernBuffPtr++ = (msb << 8) | lsb;
  WordsToRead--;                              /* One sample at a time */

  if (!WordsToRead) {
    wake_up_interruptible(&das08_wait);
  } else {
     TimerList.expires =  jiffies + 600;
     add_timer(&TimerList);
  }
  sti();
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

static int SoftRead(WORD *kernBuff, ADC_ChanRec *chan)
{
  WORD msb, lsb;

  #ifdef DEBUG
      printk("Entering SoftRead().\n");
  #endif

  /* Prepare global data for parameterless interrupt handler */

  KernBuffPtr = KernBuff;                      /* same with KernBuff           */
  WordsToRead = chan->count;

  outb_p(chan->chan | BoardData.status, STATUS_REG);  /* Set MUX channel */

  while ( WordsToRead ) {
    outb_p(0x1, MSB_DATA_BYTE);                /* Force conversion */

    /* poll until done (Yech!) */
    while (inb_p(STATUS_REG) & EOC);

    lsb = (WORD) inb(LSB_AND_CHNLS);           /* Sample: ADC -> kernel buffer */
    msb = (WORD) inb(MSB_DATA_BYTE);
    *KernBuffPtr++ = (msb << 8) | lsb;
    WordsToRead--;                              /* One sample at a time */
  }

  return 0;
}

static int TrigRead(WORD *kernBuff, ADC_ChanRec *chan)
{

  #ifdef DEBUG
      printk("Entering TrigRead().\n");
  #endif

  /* Prepare global data for parameterless interrupt handler */
  CurrChan = chan;                         /* pass chanel number to global */
  KernBuffPtr = KernBuff;                  /* same with KernBuff           */
  WordsToRead = chan->count;

  outb_p(chan->chan | INTE | BoardData.status, STATUS_REG);    /* Set MUX and Interrupt */
  TimerList.expires =  jiffies + 600;                   /* just in case ...      */
  add_timer(&TimerList);

  interruptible_sleep_on(&das08_wait);             /* Block in wait state */

  if ( WordsToRead != 0 ) {
      printk("Timing error in TrigRead: WordsToRead = %d\n", WordsToRead);
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

static void LoadCounter0( LONG value )
{
  BYTE mask;
  BYTE bData;

  /* Write the value into Counter 0 Mode 2 */

  #ifdef DEBUG
      printk("LoadCounter0: load value %#lx into Counter 0.\n", value);
  #endif

  /* the mode is in the thrid byte */
  mask = 0xff & (value >> 16);
  mask += C0+LSBFIRST;
  outb_p(mask, COUNTER_CONTROL); 

  if ( value & 0xff000000 ) {       /* load control word only */
      return;
  } else {
      /* LSB in the first byte of value */
      bData = (BYTE) (value & 0xff);
      outb_p(bData, COUNTER_0_DATA); 

      /* MSB in the second byte of value */
      bData = (BYTE) ((value >> 8) & 0xff);
      outb_p(bData, COUNTER_0_DATA); 
  }
}

static void LoadCounter1( LONG value )
{
  BYTE mask;
  BYTE bData;

  /* Write the value into Counter 1 Mode 2 */

  #ifdef DEBUG
      printk("LoadCounter0: load value %#lx into Counter 0.\n", value);
  #endif

  /* the mode is in the thrid byte */
  mask = 0xff & (value >> 16);
  mask += C1+LSBFIRST;
  outb_p(mask, COUNTER_CONTROL); 

  if ( value & 0xff000000 ) {       /* load control word only */
      return;
  } else {
      /* LSB in the first byte of value */
      bData = (BYTE) (value & 0xff);
      outb_p(bData, COUNTER_1_DATA); 

      /* MSB in the second byte of value */
      bData = (BYTE) ((value >> 8) & 0xff);
      outb_p(bData, COUNTER_1_DATA); 
  }
}

static void LoadCounter2( LONG value )
{
  BYTE mask;
  BYTE bData;

  /* Write the value into Counter 2 Mode 2 */

  #ifdef DEBUG
      printk("LoadCounter0: load value %#lx into Counter 0.\n", value);
  #endif

  /* the mode is in the thrid byte */
  mask = 0xff & (value >> 16);
  mask += C2+LSBFIRST;
  outb_p(mask, COUNTER_CONTROL); 

  if ( value & 0xff000000 ) {       /* load control word only */
      return;
  } else {
      /* LSB in the first byte of value */
      bData = (BYTE) (value & 0xff);
      outb_p(bData, COUNTER_2_DATA); 

      /* MSB in the second byte of value */
      bData = (BYTE) ((value >> 8) & 0xff);
      outb_p(bData, COUNTER_2_DATA); 
  }
}


