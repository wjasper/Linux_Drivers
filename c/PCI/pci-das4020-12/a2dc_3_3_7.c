/***************************************************************************
 Copyright (C) 2012  Warren Jasper
 All rights reserved.

 This program, PCI-DAS4020-12, is free software; you can redistribute it
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
    a2dc_3_3_7.c for PCI-DAS4020/12 
*/

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
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/major.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "pci-das4020.h"
#include "a2dc.h"
#include "plx9080.h"

#ifdef CONFIG_PCI
    #include <linux/pci.h>    /* defines pci_resent() */
#endif

/***************************************************************************
 *
 * Prototype of public and private functions.
 *
 ***************************************************************************/

static int __init das4020_init( void );
static void __exit das4020_exit( void );
static ssize_t das4020_read( struct file *filePtr, char *buf, size_t count, loff_t *off );
static ssize_t das4020_write( struct file *filePtr, const char *buf, size_t count, loff_t *off );
static int das4020_open( struct inode *iNode, struct file *filePtr );
static int das4020_close( struct inode *iNode, struct file *filePtr );
static long das4020_ioctl( struct file *filePtr, unsigned int cmd, unsigned long arg );
static int das4020_mmap( struct file *filePtr, struct vm_area_struct * vma );
static unsigned int das4020_poll( struct file *filePtr, poll_table *wait );
static irqreturn_t das4020_Interrupt( int irq, void *dev_id );
static u16 read_pci_das4020_EEPROM( u8 addr, BoardRec *boardData );
static void load_calibration_coef( int channel, int range, BoardRec *boardData );
static void store_calibration_coef( BoardRec *boardData );
static void writeReadCmd( u8 addr, BoardRec *boardData );
static void clk_EEPROM( BoardRec *boardData );
static void write_data_AD5315( u8 addr, u8 dac_select, u16 value, BoardRec *boardData );
static void write_byte_AD5315( u8 byte, BoardRec *boardData );
static void clk_ad5315( BoardRec *boardData );
static inline void *get_dmabuf_vaddr( BoardRec *boardData, long offset );
static inline dma_addr_t get_dmabuf_pciaddr( BoardRec *boardData, long offset );
static int set_config( BoardRec *boardData );
static int free_chain( BoardRec *boardData );
static void set_pacer( u32 freq, BoardRec *boardData );
inline static void initialise_i2c( BoardRec *boardData );    // one time init...
static void initialise_plx( BoardRec *boardData );
static int adc_soft_conversion( ADC_ChanRec *chanRec, BoardRec *boardData );
static int dma_read( ADC_ChanRec *chanRec, struct file *readfile, BoardRec *boardData );

module_init( das4020_init );
module_exit( das4020_exit );

MODULE_AUTHOR("Warren J. Jasper <wjasper@ncsu.edu>, Vimal Reddy <vimalreddy@yahoo.com>, Shobhit Kanaujia <sokanauj@yahoo.com>");
MODULE_DESCRIPTION("Driver for the PCI-DAS4020/12 module");
MODULE_LICENSE("GPL");

/* GLOBALS */

static spinlock_t das4020_lock;
static int MajorNumber = DEFAULT_MAJOR_DEV;
static BoardRec  BoardData[MAX_BOARDS];              /* Board Specific information       */
static DIO_ChanRec ChanDIO[MAX_BOARDS][DIO_PORTS];   /* DIO Channel specific info        */
static DAC_ChanRec ChanDAC[MAX_BOARDS][DA_CHANNELS]; /* DAC Channel specific info        */
static ADC_ChanRec ChanADC[MAX_BOARDS][AD_CHANNELS]; /* ADC Channel specific info        */

static int NumBoards = 0;                            /* number of boards found           */
static DECLARE_WAIT_QUEUE_HEAD(das4020_12_wait);     /* wait semaphore                   */

static struct cdev das4020_cdev;
static struct class *das4020_class;

static struct file_operations das4020_fops = {
    .owner          = THIS_MODULE,
    .read           = das4020_read,
    .write          = das4020_write,
    .unlocked_ioctl = das4020_ioctl,
    .mmap           = das4020_mmap,
    .open           = das4020_open,
    .release        = das4020_close,
    .poll           = das4020_poll
};


  /*
   * --------------------------------------------------------------------
   *           PCI  initialization and finalization code
   * --------------------------------------------------------------------
   */

static int das4020_init_one(struct pci_dev *pdev, const struct pci_device_id *ent);
static void das4020_remove_one(struct pci_dev *pdev);

static struct pci_device_id das4020_id_tbl[] = {
  {
  .vendor    =         PCI_VENDOR_ID_CBOARDS,
  .device    =         PCI_DEVICE_ID_CBOARDS_DAS4020_12,
  .subvendor =         PCI_ANY_ID,
  .subdevice =         PCI_ANY_ID,
  .class     =         0,
  .class_mask =        0,
  }, 
  { /* end: all zeros */}
};

MODULE_DEVICE_TABLE(pci, das4020_id_tbl);

static struct pci_driver das4020_driver = {
  .name     =     "das4020-12",
  .id_table =     das4020_id_tbl,
  .probe    =     das4020_init_one,
  .remove   =     das4020_remove_one,
};


/********************************************************************* 
*                                                                    *
* Entry point. Gets called when the driver is loaded using insmod    *
*                                                                    *
**********************************************************************/
static int __init das4020_init( void ) 
{
  int err;
  dev_t dev;

 /* Register as a device with kernel.  */
  if (MajorNumber) {
    dev = MKDEV(MajorNumber, 0);
    err = register_chrdev_region(dev, 0xff, "das4020-12");
  } else {
    err = alloc_chrdev_region(&dev, 0, 0xff, "das4020-12");
    MajorNumber = MAJOR(dev);
  }
  if (err < 0) {
    printk("%s: Failure to load module. Major Number = %d  error = %d\n",
	   ADAPTER_ID, MAJOR(dev), err);
    return err;
  }

  cdev_init(&das4020_cdev, &das4020_fops);
  das4020_cdev.owner = THIS_MODULE;
  das4020_cdev.ops = &das4020_fops;
  err = cdev_add(&das4020_cdev, dev, 0xff);
  if (err) {
    printk("%s: Error %d in registering file operations", ADAPTER_ID, err);
    return err;
  }

  /* create the class for the das4020 */
  das4020_class = class_create(THIS_MODULE, "das4020-12");
  if (IS_ERR(das4020_class)) {
    return PTR_ERR(das4020_class);
  }

  spin_lock_init(&das4020_lock);

  err = pci_register_driver(&das4020_driver);
  return  err;	
}

static int das4020_init_one( struct pci_dev *pdev, const struct pci_device_id *ent )
{
  int i, j;
  int minor;
  u8 *base0;
  u8 *base2;
  u8 *base3;
  struct page *page;
  int retval = -ENODEV;
  int dma_tbl_factor;
  char name[64];

  if (NumBoards >= MAX_BOARDS) {
    printk("das4020_init_one: NumBoards = %d. Can't exceed MAX_BOARDS.  edit a2dc.h.\n", 
	   NumBoards);
    return -ENODEV;
  }

  BoardData[NumBoards].pdev = pdev;
  BoardData[NumBoards].dma_virt_tbl = NULL;
  BoardData[NumBoards].dma_bus_addr_tbl = NULL;

  /* set DMA capabilities (this is a 32 bit PCI card) */
  if (pci_set_dma_mask(pdev, 0xffffffff)) {
    printk(KERN_WARNING "das4020: No suitable DMA available.\n");
    goto err_out_0;
  }

  // reserve the memory regions...
  if (!request_mem_region(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0),
			  "das4020/12")) {
    printk(KERN_WARNING "das4020/12: cannot reserve BAR 0.\n");
    goto err_out_0;
  }
  if (!request_mem_region(pci_resource_start(pdev, 2), pci_resource_len (pdev, 2),
			  "das4020/12")) {
    printk (KERN_WARNING"%s: cannot reserve BAR 2.\n", ADAPTER_ID);
    goto err_out_1;
  }
  if (!request_mem_region (pci_resource_start(pdev, 3), pci_resource_len(pdev, 3),
			   "das4020/12")) {
    printk ("KERN_WARNING%s: cannot reserve BAR 3.\n", ADAPTER_ID);
    goto err_out_2;
  }

  /* GETTING BASE ADDRES 0 */
  BoardData[NumBoards].phys_addr0 = pci_resource_start(pdev, 0);
  /* remap the address to lower 64 MB of memory */
  base0  = ioremap_nocache(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
  if (!base0) {
    printk("KERN_WARNING%s: cannot remap BAR0: addr = %#lx  length = %#lx\n",
	   ADAPTER_ID, (unsigned long)pci_resource_start(pdev, 0), (unsigned long)pci_resource_len(pdev, 0));
    goto err_out_3;
  } else {
    BoardData[NumBoards].base0 = base0;
  }

  /*GETTING BASE ADDRESS 2 */
  BoardData[NumBoards].phys_addr2 = pci_resource_start(pdev, 2);
  /* remap the address to lower 64 MB of memory */
  base2  = ioremap_nocache(pci_resource_start(pdev, 2), pci_resource_len(pdev, 2));
  if (!base2) {
    printk("KERN_WARNING%s: cannot remap BAR2: addr = %#lx  length = %#lx\n",
	   ADAPTER_ID, (unsigned long)pci_resource_start(pdev, 2), (unsigned long)pci_resource_len(pdev, 2));
    goto err_out_4;
  } else {
    BoardData[NumBoards].base2 = base2;
  }

  /*GETTING BASE ADDRESS 3 */
  BoardData[NumBoards].phys_addr3 = pci_resource_start(pdev, 3);
  /* remap the address to lower 64 MB of memory */
  base3  = ioremap_nocache(pci_resource_start(pdev, 3), pci_resource_len(pdev, 3));
  if (!base3) {
    printk("KERN_WARNING%s: cannot remap BAR3: addr = %#lx  length = %#lx\n",
	   ADAPTER_ID, (unsigned long)pci_resource_start(pdev, 3), (unsigned long)pci_resource_len(pdev, 3));
    goto err_out_5;
  } else {
    BoardData[NumBoards].base3 = base3;
  }

  /* pci_enable_device handles IRQ routing, so it must be before request_irq */
  if (pci_enable_device(pdev))          
    goto err_out_9;
  
  /* Register interrupt handler. */
  BoardData[NumBoards].irq = pdev->irq;
  if (request_irq(pdev->irq, das4020_Interrupt, (IRQF_DISABLED | IRQF_SHARED), 
		  "das4020/12", (void *) &BoardData[NumBoards])) {
    /* No free irq found! cleanup and exit */
    printk("KERN_WARNING%s: Can't request IRQ %d\n", ADAPTER_ID, BoardData[NumBoards].irq);
    goto err_out_6;
  }

  /* Initialising D/A */
  ChanDAC[NumBoards][0].value = 0;
  ChanDAC[NumBoards][1].value = 0;
  
  for (i = 0; i < DA_CHANNELS; i++) {
    ChanDAC[NumBoards][i].value = 0;
    ChanDAC[NumBoards][i].gain = BP_10_00V;
    ChanDAC[NumBoards][i].open = FALSE;
  }

  writew(0x0, DAC0_CONV_LSB);  /* load lower nibble first */
  writew(0x8, DAC0_CONV_MSB);
  writew(0x0, DAC1_CONV_LSB);  /* load lower nibble first */
  writew(0x8, DAC1_CONV_MSB);
  writew(0x0, DAC_CTRL_REG);
  BoardData[NumBoards].DAC_ControlReg = 0x0;  /* Disable DACs */

  /* create a device in /sys/class/das4020-12/ */
  for (i = 0; i < DA_CHANNELS; i++) {
    minor = (NumBoards<<0x4) + i + DIO_PORTS + AD_CHANNELS;
    sprintf(name, "das4020-12/da%d_%d", NumBoards, i);
    device_create(das4020_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  }

  /* Initializing DIO */
  ChanDIO[NumBoards][0].addr = DIO_PORTA;
  ChanDIO[NumBoards][1].addr = DIO_PORTB;
  ChanDIO[NumBoards][2].addr = DIO_PORTC;
  BoardData[NumBoards].dio_reg = 0x9b;        /* DIO control register */
 
  for (i = 0; i < DIO_PORTS; i++) {
    ChanDIO[NumBoards][i].open = FALSE;        
    ChanDIO[NumBoards][i].mode = 0;            
  }

  minor = (NumBoards<<0x4) + AD_CHANNELS;
  sprintf(name, "das4020-12/dio%d_0A", NumBoards);
  device_create(das4020_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "das4020-12/dio%d_0B", NumBoards);
  device_create(das4020_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;
  sprintf(name, "das4020-12/dio%d_0C", NumBoards);
  device_create(das4020_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  minor++;

  /* Initialize the registers on the board */
  BoardData[NumBoards].InterruptEnableReg = 0x0;
  writew(BoardData[NumBoards].InterruptEnableReg, INTRPT_EN_REG);
  
  BoardData[NumBoards].HardwareConfigReg = WCLK_SRC0;
  writew(BoardData[NumBoards].HardwareConfigReg, HW_CONFIG_REG);

  BoardData[NumBoards].MemorySizeReg = 0x0;   // set to 32k samples (64k bytes)
  writew(BoardData[NumBoards].MemorySizeReg, MEM_SIZE_REG);
  BoardData[NumBoards].half_FIFO_size = 32768; // 32k bytes 

  BoardData[NumBoards].DAQ_ControlReg0 = 0x0;
  writew(BoardData[NumBoards].DAQ_ControlReg0, CTRL_REG_0);

  BoardData[NumBoards].DAQ_ControlReg1 = 0x0;
  writew(BoardData[NumBoards].DAQ_ControlReg1, CTRL_REG_1);

  /* Initlialize the DAQ (A/D) */
  
  /* Bit Mask for the I2C Register */
  BoardData[NumBoards].bitmask = (ASRC_BNC);
  BoardData[NumBoards].scan_rate = 1000000;  /* 1 MHz */
  store_calibration_coef(&BoardData[NumBoards]);
  initialise_i2c(&BoardData[NumBoards]);
  initialise_plx(&BoardData[NumBoards]);

  /* Initializing each ADC for +/-5 Volts */
  ChanADC[NumBoards][0].bitmask = (CH0_EN | CH0_5V | ASRC_BNC);
  ChanADC[NumBoards][1].bitmask = (CH1_EN | CH1_5V | ASRC_BNC);
  ChanADC[NumBoards][2].bitmask = (CH2_EN | CH2_5V | ASRC_BNC);
  ChanADC[NumBoards][3].bitmask = (CH3_EN | CH3_5V | ASRC_BNC);

  /* Initializing each ADC for +/-5 Volts */
  for (i = 0; i< AD_CHANNELS; i++) {
    ChanADC[NumBoards][i].open = FALSE;
    ChanADC[NumBoards][i].nonBlockFlag = FALSE;
    ChanADC[NumBoards][i].lowChan = i;
    ChanADC[NumBoards][i].hiChan = i;
    ChanADC[NumBoards][i].pretrigCount = 0;     /* Software Convert */
    ChanADC[NumBoards][i].frequency = 1000000;  /* Default to 1 MHz */
    load_calibration_coef(i, BP_5_00V, &BoardData[NumBoards]);
  }

  for (i = 0; i < AD_CHANNELS; i++) {
    minor = (NumBoards<<0x4) + i;
    sprintf(name, "das4020-12/ad%d_%d", NumBoards, i);
    device_create(das4020_class, NULL, MKDEV(MajorNumber, minor), NULL, name);
  }

  init_waitqueue_head(&das4020_12_wait);
  BoardData[NumBoards].nonBlockFile = NULL;
  BoardData[NumBoards].DMAComplete = 0;

  /* Enable bus mastering, in case the BIOS didn't */
  pci_set_master(pdev);
  

  /* determine size of pages we will use for DMA 
      The page size must be:
        a) greater or equal to half FIFO size
        b) greater or equal to PAGE_SIZE
        c) a multiple of PAGE_SIZE
        d) a multiple of half FIFO size
  */
  dma_tbl_factor = BoardData[NumBoards].half_FIFO_size;
  if (PAGE_SIZE > dma_tbl_factor) dma_tbl_factor = PAGE_SIZE;
  BoardData[NumBoards].dma_tbl_page_size =
  ((ADC_BUFF_PAGE_SIZE+dma_tbl_factor-1)/dma_tbl_factor)*dma_tbl_factor;

  if (((BoardData[NumBoards].dma_tbl_page_size % PAGE_SIZE) != 0) ||
      ((BoardData[NumBoards].dma_tbl_page_size % BoardData[NumBoards].half_FIFO_size) != 0)) {
    printk("DAS4020: Internal buffer sizing error (this is not possible)\n");
    retval=-ENOMEM;
    goto err_out_10;
  }

  /* Just in case ... */
  BoardData[NumBoards].dma_tbl_page_size = ALIGN_ADDRESS(BoardData[NumBoards].dma_tbl_page_size, PAGE_SIZE);

  /* This is the total size of the dma buffer */
  BoardData[NumBoards].dma_phy_size = ALIGN_ADDRESS(ADC_BUFF_PHY_SIZE, BoardData[NumBoards].dma_tbl_page_size);
  BoardData[NumBoards].dma_tbl_size = BoardData[NumBoards].dma_phy_size/BoardData[NumBoards].dma_tbl_page_size;
  
  /* allocate virtual memory address table */
  BoardData[NumBoards].dma_virt_tbl = kmalloc(sizeof(void *)*BoardData[NumBoards].dma_tbl_size, GFP_KERNEL);
  if (!BoardData[NumBoards].dma_virt_tbl) {
    retval = -ENOMEM;
    goto err_out_10;
  }
  memset(BoardData[NumBoards].dma_virt_tbl, 0, sizeof(void *)*BoardData[NumBoards].dma_tbl_size);
  
  /* allocate bus address table */
  BoardData[NumBoards].dma_bus_addr_tbl = kmalloc(sizeof(dma_addr_t)*BoardData[NumBoards].dma_tbl_size, GFP_KERNEL);
  if (!BoardData[NumBoards].dma_bus_addr_tbl) {
    retval = -ENOMEM;
    goto err_out_10;
  }
  memset(BoardData[NumBoards].dma_bus_addr_tbl, 0, sizeof(dma_addr_t)*BoardData[NumBoards].dma_tbl_size);
    
  /* allocate table entries */
  for (j = 0; j < BoardData[NumBoards].dma_tbl_size; j++) {
    BoardData[NumBoards].dma_virt_tbl[j] = pci_alloc_consistent(pdev,
							      BoardData[NumBoards].dma_tbl_page_size,
							      &BoardData[NumBoards].dma_bus_addr_tbl[j]);
    if (BoardData[NumBoards].dma_virt_tbl[j] == NULL) {
      printk("%s: Error allocating DMA buffer space. Got %d of %d bytes\n",
	     ADAPTER_ID, j*BoardData[NumBoards].dma_tbl_page_size,
	     BoardData[NumBoards].dma_phy_size);
      retval = -ENOMEM;
      goto err_out_10;
    }
    /* set PG_reserved flag on DMA memory pages. 
       This protects them from the VM system after they're mmap()'d  
    */
    page = virt_to_page(BoardData[NumBoards].dma_virt_tbl[j]);
    for (i = 0; i < BoardData[NumBoards].dma_tbl_page_size/PAGE_SIZE; i++) {
      SetPageReserved(&page[i]);
    }
  }

  printk("%s: Board #%d BADR0 = 0x%p, BADR2 = 0x%p, BADR3 = 0x%p \n", ADAPTER_ID, NumBoards,
	 BoardData[NumBoards].base0, BoardData[NumBoards].base2, BoardData[NumBoards].base3);
  printk("%s: IRQ=%d 8/3/2012 Warren J. Jasper <wjasper@ncsu.edu>\n",
	 ADAPTER_ID, BoardData[NumBoards].irq);

  NumBoards++;
  return 0;

err_out_10:
  if (BoardData[NumBoards].dma_virt_tbl) {
    for (i = 0; i < BoardData[NumBoards].dma_tbl_size;i++) {
      if (BoardData[NumBoards].dma_virt_tbl[i]) {
	/* unmark pages as reserved */
	page = virt_to_page(BoardData[NumBoards].dma_virt_tbl[i]);
	for (j = 0; j < BoardData[NumBoards].dma_tbl_page_size/PAGE_SIZE; j++) {
	  ClearPageReserved(&page[j]);
	}
	/* release memory */
	pci_free_consistent(pdev,
			    BoardData[NumBoards].dma_tbl_page_size,
			    BoardData[NumBoards].dma_virt_tbl[i],
			    BoardData[NumBoards].dma_bus_addr_tbl[i]);
	BoardData[NumBoards].dma_virt_tbl[i]=NULL;
      }
    }
    kfree(BoardData[NumBoards].dma_virt_tbl);
    BoardData[NumBoards].dma_virt_tbl = NULL;
  }
  if (BoardData[NumBoards].dma_bus_addr_tbl) {
    kfree(BoardData[NumBoards].dma_bus_addr_tbl);
    BoardData[NumBoards].dma_bus_addr_tbl = NULL;    
  }
err_out_9:
  free_irq(pdev->irq, (void *) &BoardData[NumBoards]);
err_out_6:
  iounmap ((void *) base3);
err_out_5:
  iounmap ((void *) base2);
err_out_4:
  iounmap ((void *) base0);
err_out_3:
  release_mem_region(pci_resource_start(pdev, 3), pci_resource_len(pdev, 3));
err_out_2:
  release_mem_region(pci_resource_start(pdev, 2), pci_resource_len(pdev, 2));
err_out_1:
  release_mem_region(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
err_out_0:
  return retval;
}

/***************************************************************************
 *
 * Remove driver. Called when "rmmod das4020-12" is run on the command line.
 *
 ***************************************************************************/
void __exit das4020_exit( void )
{
  dev_t dev;

  dev = MKDEV(MajorNumber, 0);
  pci_unregister_driver(&das4020_driver);
  class_destroy(das4020_class);
  cdev_del(&das4020_cdev);
  unregister_chrdev_region(dev, 0xff);
  printk("%s: module removed from das4020_exit.\n", ADAPTER_ID);
}

static void das4020_remove_one( struct pci_dev *pdev )
{
  struct page *page;
  int i, j;
  int minor;

  NumBoards--;
  free_irq(BoardData[NumBoards].irq, (void *) &BoardData[NumBoards]);
  iounmap(BoardData[NumBoards].base0);
  iounmap(BoardData[NumBoards].base2);
  iounmap(BoardData[NumBoards].base3);
  release_mem_region(pci_resource_start(pdev, 3), pci_resource_len (pdev, 3));
  release_mem_region(pci_resource_start(pdev, 2), pci_resource_len (pdev, 2));
  release_mem_region(pci_resource_start(pdev, 0), pci_resource_len (pdev, 0));

  /* free up DMA buffer */
  for (i = 0; i < BoardData[NumBoards].dma_tbl_size; i++) {
    /* unmark pages as reserved */
    page = virt_to_page(BoardData[NumBoards].dma_virt_tbl[i]);
    for (j = 0; j < BoardData[NumBoards].dma_tbl_page_size/PAGE_SIZE; j++) {
      ClearPageReserved(&page[j]);
    }
    /* release memory */
    pci_free_consistent(pdev,
			BoardData[NumBoards].dma_tbl_page_size,
			BoardData[NumBoards].dma_virt_tbl[i],
			BoardData[NumBoards].dma_bus_addr_tbl[i]);
    BoardData[NumBoards].dma_virt_tbl[i]=NULL;
  }

  /* free address table memory */
  kfree(BoardData[NumBoards].dma_virt_tbl);
  BoardData[NumBoards].dma_virt_tbl = NULL;
  kfree(BoardData[NumBoards].dma_bus_addr_tbl);
  BoardData[NumBoards].dma_bus_addr_tbl = NULL;

  for (i = 0; i < AD_CHANNELS + DA_CHANNELS + DIO_PORTS; i++) {
    minor = (NumBoards<<0x4) + i;
    device_destroy(das4020_class, MKDEV(MajorNumber, minor));
  }

  #ifdef DEBUG
    printk("das4020_remove_one: Board #%d removed.\n", NumBoards);
  #endif
}     /* das4020_remove_one() */


/***************************************************************************
 *
 * read() service function
 *
 ***************************************************************************/
static ssize_t das4020_read( struct file *filePtr, char *buf, size_t count, loff_t *off ) 
{
  u8  bReg;
  u8 *base0;
  u8 *base2;
  u8 *base3;

  int   minor;
  int   port;
  int   board;
  int   chan;
  int   i;

  u16* wValue;

  struct inode *iNode = filePtr->f_dentry->d_inode;
  minor = iminor(iNode);
  board = BOARD(minor);          /* get which board   */
  chan =  CHAN(minor);           /* get which channel */
  base0 = BoardData[board].base0;
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;

  if (count < 1) {
    printk("das4020_read(): count must be greater than 0.\n");
    BoardData[board].busyRead = FALSE;
    return(-1);
  }

  if (chan >= 0 && chan < AD_CHANNELS) {
    ChanADC[board][chan].nonBlockFlag = (filePtr->f_flags & O_NONBLOCK);
    if (BoardData[board].nonBlockFile && BoardData[board].nonBlockFile != filePtr) {
      /* somebody else has a non-blocking request going */
      return -EBUSY;
    } else if (BoardData[board].busyRead == TRUE  &&
	       !ChanADC[board][chan].nonBlockFlag) { /* if board is in use, return busy */
      return (-EBUSY);
    } else {
      BoardData[board].busyRead = TRUE;
    }

    /* Set up configuration and load calibration coefficeints */
   #ifdef DEBUG
      printk("read:  BoardData[board].bitmask = %#x   ChanADC[board][chan].bitmask = %#x\n",
	     BoardData[board].bitmask, ChanADC[board][chan].bitmask);
    #endif
    if (BoardData[board].bitmask != ChanADC[board][chan].bitmask) {
      BoardData[board].bitmask = ChanADC[board][chan].bitmask;
      set_config(&BoardData[board]);
      for (i = ChanADC[board][chan].lowChan; i <= ChanADC[board][chan].hiChan; i++) {
	load_calibration_coef(i, ChanADC[board][chan].bitmask, &BoardData[board]);
      }
    }
    
    /* Read */
    switch (ChanADC[board][chan].adcMode) {
      case ADC_SOFT_CONVERSION:
        #ifdef DEBUG
	  printk("das4020_read(): Entering ADC_SOFT_CONVERSION mode.  count = %d\n",
		 (int) count);
        #endif

	if (((ChanADC[board][chan].count = count) > ADC_BUFF_PHY_SIZE/2) || 
	    (count > BoardData[board].dma_tbl_page_size/2)) {
	  printk("das4020_read(): requesting too large a count size (may need to increase ADC_BUFF_PAGE_SIZE).\n");
          BoardData[board].busyRead = FALSE;
          return(-1);
        }

	if (adc_soft_conversion(&ChanADC[board][chan], &BoardData[board])) {
	  printk("das4020_read: adc_soft_conversion() failed.\n");
	  BoardData[board].busyRead = FALSE;
	  return(-1);
	}

	/* Write data to user space */
	if (ChanADC[board][chan].count == 1) {
          wValue = BoardData[board].dma_virt_tbl[0];
	  put_user(*wValue, (u16*) buf);
	} else {
	  for (i = 0; i < ChanADC[board][chan].count*sizeof(u16)/BoardData[board].dma_tbl_page_size; i++) {
	    if (copy_to_user(buf + i*BoardData[board].dma_tbl_page_size,
			     BoardData[board].dma_virt_tbl[i],
			     BoardData[board].dma_tbl_page_size)
		) return -EFAULT;
	  }
          if (ChanADC[board][chan].count*sizeof(u16) % BoardData[board].dma_tbl_page_size) {
	    if (copy_to_user(buf + i*BoardData[board].dma_tbl_page_size,
			     BoardData[board].dma_virt_tbl[i],
			     ChanADC[board][chan].count*sizeof(u16) % BoardData[board].dma_tbl_page_size)
		) return -EFAULT;
	  }
	}
	break;

      case ADC_DMA_CONVERSION :
	if ((ChanADC[board][chan].count = count) > BoardData[board].dma_phy_size/2) {
	  printk("das4020_read(): requesting too large a count size. Max size is %d\n",
		 BoardData[board].dma_phy_size/2);
          BoardData[board].busyRead = FALSE;
          return(-1);
        }

       switch (dma_read(&ChanADC[board][chan], filePtr, &BoardData[board])) {
         case 0:
           break; /* success */
         case -EAGAIN:
	   return -EAGAIN; /* non-blocking, and not complete */
	   break;
         default:
	   printk("das4020_read: dma_read() failed.\n");
	   BoardData[board].busyRead = FALSE;
	   return(-1);
       }
    }
    BoardData[board].busyRead = FALSE;
    return(ChanADC[board][chan].count);        /* return number of samples read */
  }

  /* check to see if reading a value from the DIO */
  if (chan >= AD_CHANNELS && chan < AD_CHANNELS + DIO_PORTS) {
    port = chan - AD_CHANNELS;
    bReg = readb(ChanDIO[board][port].addr);
    put_user(bReg, (u8*) buf);

    #ifdef DEBUG
      printk("das4020_read: %s DIO Port %#x addr 0x%p set to %#x\n", 
               ADAPTER_ID, port, ChanDIO[board][port].addr, bReg);
    #endif
    return 1;
  }
  return 0;
}

/***************************************************************************
 *
 * write() service function
 *
 ***************************************************************************/

static ssize_t das4020_write( struct file *filePtr, const char *buf, size_t count, loff_t *off ) 
{
  u8 bReg;
  u8 *base0;
  u8 *base2;
  u8 *base3;

  int  minor;
  int  port  = 0;
  int  board = 0;
  int  chan  = 0;
  struct inode *iNode = filePtr->f_dentry->d_inode;

  minor = iminor(iNode);
  board = BOARD(minor);            /* get which board   */
  chan =  CHAN(minor);            /* get which channel */
  base0 = BoardData[board].base0;
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;
  
  #ifdef DEBUG
  printk("das4020_write(): Minor = %d, Count = %d\n", minor, (int) count);
  #endif

  if (count < 1) {
    printk("das4020_write(): count must be greater than 0.\n");
    BoardData[board].busyWrite = FALSE;
    return(-1);
  }

  /* check to see if writing a value to the DIO */
  if ((chan >= AD_CHANNELS) && (chan < AD_CHANNELS+DIO_PORTS)) {
    port = chan - AD_CHANNELS;
    get_user(bReg, (u8*)buf);
    ChanDIO[board][port].value = bReg;
    writeb(bReg, ChanDIO[board][port].addr);

#ifdef DEBUG
    printk("das4020_write %s: DIO Port %#x set to %#x\n", ADAPTER_ID, port, bReg);
#endif

    BoardData[board].busyWrite = FALSE;
    return 1;
  }

  if ((chan >= AD_CHANNELS+DIO_PORTS) && (chan < AD_CHANNELS+DIO_PORTS+DA_CHANNELS)) {
    port = chan - AD_CHANNELS - DIO_PORTS;

    /* Read data from user space */
    get_user(ChanDAC[board][port].value, (u16*)buf);
    if (port == 0) {
      writew(ChanDAC[board][port].value & 0xff,       DAC0_CONV_LSB);
      writew((ChanDAC[board][port].value >> 8) & 0xf, DAC0_CONV_MSB);
    } else {
      writew(ChanDAC[board][port].value & 0xff,       DAC1_CONV_LSB);
      writew((ChanDAC[board][port].value >> 8) & 0xf, DAC1_CONV_MSB);
    }
    return 1;
  }
  return 0;
}

/***************************************************************************
 *
 * open() service handler
 *
 ***************************************************************************/

static int das4020_open( struct inode *iNode, struct file *filePtr ) 
{
  u8 *base0;
  u8 *base2;
  u8 *base3;

  int port  = 0;
  int board = 0;
  int chan  = 0;
  int minor = iminor(iNode);

  board = BOARD(minor);            /* get which board   */
  chan =  CHAN(minor);             /* get which channel */
  base0 = BoardData[board].base0;
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;
  
  if ((chan >= 0 && chan < AD_CHANNELS)) {
    if (ChanADC[board][chan].open == TRUE) {
      return -EBUSY;
    }

    ChanADC[board][chan].bitmask = (0x1 << (chan) |  ASRC_BNC);  // Enable Channel 
    ChanADC[board][chan].bitmask |= (0x1 << (chan+4));           // +/- 5 V 

    ChanADC[board][chan].open = TRUE;
    ChanADC[board][chan].pretrigCount = 0;
    ChanADC[board][chan].adcMode = filePtr->f_flags;  /* set acquisition mode */
    ChanADC[board][chan].lowChan = chan;
    ChanADC[board][chan].hiChan = chan;
    ChanADC[board][chan].atrigMode = INACTIVE;
    ChanADC[board][chan].analog_trig_low = 0;
    ChanADC[board][chan].analog_trig_high = 0;
    ChanADC[board][chan].trigger = SOFT_TRIGGER;
    ChanADC[board][chan].gate = SOFT_GATE;
    ChanADC[board][chan].controlReg0 = 0;
    ChanADC[board][chan].controlReg1 = 0;

    #ifdef DEBUG
      printk("%s: open(): (ADC) board %d chan %d.\n", ADAPTER_ID, board, chan);
    #endif
    return 0;
  }

  if ((chan >= AD_CHANNELS) && (chan < AD_CHANNELS+DIO_PORTS)) {
    port = chan - AD_CHANNELS;
    if (ChanDIO[board][port].open == TRUE) {
    #ifdef DEBUG
      printk("%s: open() failed: board %d chan %d mode %d.\n",
	     ADAPTER_ID, board, chan, ChanDIO[board][port].mode);
    #endif
      return -EBUSY;
    }

    ChanDIO[board][port].open = TRUE;                 /* The device is open */
    ChanDIO[board][port].f_flags = filePtr->f_flags;

    #ifdef DEBUG
    printk("%s: open(): (DIO) board %d chan %d mode %d.\n",
	   ADAPTER_ID, board, chan, ChanDIO[board][port].mode);
    #endif
    return 0;
  }

  if ((chan >= AD_CHANNELS+DIO_PORTS) && (chan < AD_CHANNELS+DIO_PORTS+DA_CHANNELS)) {
    port = chan - AD_CHANNELS - DIO_PORTS;
    if (ChanDAC[board][port].open == TRUE) {
    #ifdef DEBUG
      printk("%s: open() failed: board %d chan %d \n", ADAPTER_ID, board, chan);
    #endif
      return -EBUSY;
    }
    
    ChanDAC[board][port].open = TRUE;            /* The device is open */
    BoardData[board].DAC_ControlReg |= DAC_OE;   /* Enable DACs */
    writew(BoardData[board].DAC_ControlReg, DAC_CTRL_REG);
    
    #ifdef DEBUG
      printk("%s: open(): (DAC) board %d chan %d.\n", ADAPTER_ID, board, chan);
    #endif
  }
  return 0;
}

/***************************************************************************
 *
 * close() service handler
 *
 ***************************************************************************/

static int das4020_close( struct inode *iNode, struct file *filePtr ) 
{
  u8 *base0;
  u8 *base2;
  u8 *base3;
  int port  = 0;
  int board = 0;
  int chan  = 0;
  int minor = iminor(iNode);
  int i;

  board = BOARD(minor);            /* get which board   */
  chan =  CHAN(minor);             /* get which channel */
  base0 = BoardData[board].base0;
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;

  if (filePtr && BoardData[board].nonBlockFile && filePtr == BoardData[board].nonBlockFile) {
    /* cancel pending non-blocking i/o */
    /* disable acquisition */
    BoardData[board].DAQ_ControlReg1 &= ~CR1_SFT_GATE;
    writew(BoardData[board].DAQ_ControlReg1, CTRL_REG_1);

    /* cancel DMA request to PLX here */
    // bugfix: disabling the dma channel locks up the PLX on newer 4020's
    // unless the READY bit is turned off.
    // Follow mfg's example, by leaving the enable bit set,
    // because turning off READY doesn't seem to be reliable 

    // writel(readl(PLX_DMA1_MODE) & ~(PLX_DMA_MODE_READY), PLX_DMA1_MODE); // disable DMA ready flag 
    /* PLX manual says to enable then start then wait for done */
    /* but doing this causes a lockup */
    // writeb(PLX_DMA_EN, PLX_DMA1_CSR);
    // writeb(PLX_DMA_EN | PLX_DMA_START, PLX_DMA1_CSR);
    // while(readb(PLX_DMA1_CSR) & PLX_DMA_DONE) udelay(1);
    // writeb(0, PLX_DMA1_CSR);
    
    writeb(PLX_DMA_EN | PLX_DMA_ABORT, PLX_DMA1_CSR);
    for(i = 0; i < 100000 && !(readb(PLX_DMA1_CSR) & PLX_DMA_DONE); i++) udelay(10);   // wait for DMA to stop
    if (i == 100000) printk("das4020_close: DMA cancel timed out.\n");
    
    /* Clear ADC FIFO Pointer */
    writew(0x0, ADC_FIFO_CLR);
    udelay(1000);
    BoardData[board].DMAComplete = 0;
    /* clear non-block flag */
    BoardData[board].nonBlockFile = NULL;
  }
  
  if (chan >= 0 && chan < AD_CHANNELS) {
    /* ADC */
    ChanADC[board][chan].open = FALSE;
  } else if (chan >= AD_CHANNELS && chan < AD_CHANNELS + DIO_PORTS) {
    /* DIO */
    port = chan - AD_CHANNELS;
    ChanDIO[board][port].open = FALSE;
  } else if (chan >= AD_CHANNELS+DIO_PORTS && chan < AD_CHANNELS+DIO_PORTS+DA_CHANNELS) {
    /* DAC */
    port = chan - AD_CHANNELS - DIO_PORTS;
    ChanDAC[board][port].open = FALSE;
    if (ChanDAC[board][0].open == FALSE && ChanDAC[board][1].open == FALSE) {
      BoardData[board].DAC_ControlReg &= ~DAC_OE;             /* Disable DACs */
      writew(BoardData[board].DAC_ControlReg, DAC_CTRL_REG);
    }
  } else {
    printk("das4020_close: board %d: Incorrect chan number (%d).\n", board, chan);
  }  

#ifdef DEBUG
  printk("%s: close() successful: board %d   chan %d.\n", ADAPTER_ID, board, chan);
#endif

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

static long das4020_ioctl( struct file *filePtr, unsigned int cmd, unsigned long arg ) 
{
  u8 *base0;
  u8 *base2;
  u8 *base3;

  int i;
  int port  = 0;
  int board = 0;
  int chan  = 0;
  int minor = iminor(filePtr->f_dentry->d_inode);
  
  int channel;
  int err = 0;
  int size = _IOC_SIZE(cmd); /* the size bitfield in cmd */

  board = BOARD(minor);          /* get which board   */
  chan =  CHAN(minor);           /* get which channel */
  base0 = BoardData[board].base0;
  base2 = BoardData[board].base2;
  base3 = BoardData[board].base3;
  
  /* 
   * extract the type and number bitfields, and don't decode
   * wrong cmds;  return EINVAL before access_ok()
   */
  
  if (_IOC_TYPE(cmd) != IOCTL_MAGIC) return -ENOTTY;
  if (_IOC_NR(cmd) > IOCTL_MAXNR)  return -ENOTTY;

  if (_IOC_DIR(cmd) & _IOC_READ) {
    err = !access_ok(VERIFY_WRITE, (void *) arg, size);
  } else if (_IOC_DIR(cmd) & _IOC_WRITE) {
    err = !access_ok(VERIFY_READ, (void *) arg, size);
  }
  if (err) return -EFAULT;
  
  if (chan >= 0 && chan < AD_CHANNELS) {
    switch (cmd) {
      case ADC_SET_GAINS:
        #ifdef DEBUG
          printk("ioctl ADC_SET_GAINS: Channel = %d, Range = %#lx\n", chan, arg);
        #endif
  	ChanADC[board][chan].bitmask &= ~(ALL_CHAN);             // Mask off the channel gain bits
  	ChanADC[board][chan].bitmask |= ((u16)(arg & ALL_CHAN)); // set the channel gain bits
	break;

      case ADC_GET_GAINS:
	put_user( (long)ChanADC[board][chan].bitmask, (long*) arg);
	break;

      /* (value will be written to the hardware on the next call to read() by set_config() ) */
      case ADC_SET_CLK_SRC:
	switch (arg) {
	  case CLK_EXT_BNC:
	    ChanADC[board][chan].bitmask = (ChanADC[board][chan].bitmask & ~CLK_MASK) | CLK_EXT_BNC;
	    break;
	  case CLK_AD_START_TRIG_IN:
	    ChanADC[board][chan].bitmask = (ChanADC[board][chan].bitmask & ~CLK_MASK) | CLK_AD_START_TRIG_IN;
	    break;
	  case CLK_INTERNAL:
	    ChanADC[board][chan].bitmask = (ChanADC[board][chan].bitmask & ~CLK_MASK) | CLK_INTERNAL;
	    break;
	  default:
	    return -EINVAL;
            break;
	}
	break;
	
      case ADC_GET_CLK_SRC:
	switch(ChanADC[board][chan].bitmask & CLK_MASK) {
	  case CLK_EXT_BNC:
	    put_user( (long) CLK_EXT_BNC, (long*) arg);	
	    break;
	  case CLK_AD_START_TRIG_IN:
	    put_user( (long) CLK_AD_START_TRIG_IN, (long*) arg);	
	    break;
	  case CLK_INTERNAL:
	  default:
	    put_user( (long) CLK_INTERNAL, (long*) arg);
            break;
	}      
	break;

      case ADC_SET_PACER_FREQ:
        if ((ChanADC[board][chan].hiChan - ChanADC[board][chan].lowChan ) == 3) {
	  if (arg > MAX_SCAN_RATE_4) {
	    printk("ioctl ADC_SET_PACER_FREQ:  Can not set frequency %ld greater than %d.\n", arg, MAX_SCAN_RATE_4);
	    return -EINVAL;
	  } else if (arg > MAX_SCAN_RATE_1) {
	    printk("ioctl ADC_SET_PACER_FREQ:  Can not set frequency %ld greater than %d.\n", arg, MAX_SCAN_RATE_1);
	    return -EINVAL;
	  }
	}
        if (arg < MIN_SCAN_RATE) {
	  printk("ioctl ADC_SET_PACER_FREQ:  Can not set frequency %ld less than %d.\n", arg, MIN_SCAN_RATE);
	  return -EINVAL;
	}
	ChanADC[board][chan].frequency = arg;
	set_pacer(arg, &BoardData[board]);
	break;

      case ADC_GET_PACER_FREQ:
	put_user(ChanADC[board][chan].frequency, (long *) arg);
	break;

      case ADC_SET_TRIGGER:
	ChanADC[board][chan].trigger = arg & 0x3f;
        break;

      case ADC_SET_GATE:
	ChanADC[board][chan].gate = arg & 0xf;
        break;

      case ADC_SET_CHAN_LOW:
        if (arg < 0 || arg > AD_CHANNELS) return -EINVAL;
	if (arg > ChanADC[board][chan].hiChan) return -EINVAL;
	ChanADC[board][chan].lowChan = (u16) arg;
	ChanADC[board][chan].bitmask &= ~(ALL_EN);
	for (i = ChanADC[board][chan].lowChan; i <= ChanADC[board][chan].hiChan; i++) {
	  ChanADC[board][chan].bitmask |= (0x1 << i);
	}

        ChanADC[board][chan].controlReg1 &= ~CR1_CHANMODE_MASK;
        switch (ChanADC[board][chan].hiChan - ChanADC[board][chan].lowChan) {
          case 0:
	    ChanADC[board][chan].controlReg1 |= CR1_CHANMODE_1; // Single Channel 20 Hz max. sample rate.
	    break;
	  case 1:
	    ChanADC[board][chan].controlReg1 |= CR1_CHANMODE_2; // Two Channels 20 Hz max. sample rate.
	    break;
	  case 3:
	    ChanADC[board][chan].controlReg1 |= CR1_CHANMODE_4; // Four Channels 10 Hz max. sample rate.
	    break;
	  default:
	    printk("ADC_SET_CHAN_LOW: Number of selected channels must be 1, 2 or 4.\n");
	    return (-EINVAL);
	}

        #ifdef DEBUG
	  printk("ADC_SET_CHAN_LOW: arg = %#lx  bitmask = %#x  lowChan = %d  hiChan = %d\n",
		 arg, ChanADC[board][chan].bitmask, 
                 ChanADC[board][chan].lowChan, ChanADC[board][chan].hiChan);
	#endif
	break;

      case ADC_SET_CHAN_HIGH:
        if (arg < 0 || arg > AD_CHANNELS) return -EINVAL;
	if (arg < ChanADC[board][chan].lowChan) return -EINVAL;
	ChanADC[board][chan].hiChan = (u16) arg;
	for (i = ChanADC[board][chan].lowChan; i <= ChanADC[board][chan].hiChan; i++) {
	  ChanADC[board][chan].bitmask |= (0x1 << i);
	}

        ChanADC[board][chan].controlReg1 &= ~CR1_CHANMODE_MASK;
        switch (ChanADC[board][chan].hiChan - ChanADC[board][chan].lowChan) {
          case 0:
	    ChanADC[board][chan].controlReg1 |= CR1_CHANMODE_1; // Single Channel 20 Hz max. sample rate.
	    break;
	  case 1:
	    ChanADC[board][chan].controlReg1 |= CR1_CHANMODE_2; // Two Channels 20 Hz max. sample rate.
	    break;
	  case 3:
	    ChanADC[board][chan].controlReg1 |= CR1_CHANMODE_4; // Four Channels 10 Hz max. sample rate.
	    break;
	  default:
	    printk("ADC_SET_CHAN_HIGH: Number of selected channels must be 1, 2 or 4.\n");
	    return (-EINVAL);
	}

        #ifdef DEBUG
  	  printk("ADC_SET_CHAN_HIGH: arg = %#lx  bitmask = %#x  lowChan = %d  hiChan = %d\n",
		 arg, ChanADC[board][chan].bitmask,
                 ChanADC[board][chan].lowChan, ChanADC[board][chan].hiChan);
	#endif
	break;

      case ADC_SET_ATRIG_LOW:
	ChanADC[board][chan].analog_trig_low = ((u16) arg);
	break;

      case ADC_SET_ATRIG_HIGH:
	ChanADC[board][chan].analog_trig_high = ((u16) arg);
	break;

      case ADC_SET_ATRIG_MODE:
        switch( arg & CR1_ATRIG_MD_MASK ) {
  	  case INACTIVE:
	  case POSITIVE_HYSTERESIS:
	  case NEGATIVE_HYSTERESIS:
	  case NEGATIVE_SLOPE:
	  case POSITIVE_SLOPE:
	  case WINDOW :
	    ChanADC[board][chan].atrigMode = ((u8) arg);
	    break;
	  default:
            return -EINVAL;
            break;
	}
	break;

      case ADC_GET_HALF_FIFO_SIZE:
	put_user(BoardData[board].half_FIFO_size, (long *) arg);
	break;

      case ADC_SET_FIFO_SIZE:
        switch (arg) {
  	  case 256:   BoardData[board].MemorySizeReg = 0x7f; break;  
          case 512:   BoardData[board].MemorySizeReg = 0x7e; break;
          case 1024:  BoardData[board].MemorySizeReg = 0x7c; break;
          case 2048:  BoardData[board].MemorySizeReg = 0x78; break;
          case 4096:  BoardData[board].MemorySizeReg = 0x70; break;
          case 8192:  BoardData[board].MemorySizeReg = 0x60; break;
          case 16384: BoardData[board].MemorySizeReg = 0x40; break;
          case 32768: BoardData[board].MemorySizeReg = 0x00; break;
	  default:    return -EINVAL;                 break;
        }
        BoardData[board].half_FIFO_size = arg;  /* value in byes */
        writew(BoardData[board].MemorySizeReg, MEM_SIZE_REG);
        break;

      case ADC_SET_HW_CONFIG_REG:
        BoardData[board].HardwareConfigReg = (u16) arg;
        writew(BoardData[board].HardwareConfigReg, HW_CONFIG_REG);
        break;

      case ADC_SET_CONTROL_REG0:
        ChanADC[board][chan].controlReg0 = (u16) arg;
        writew(BoardData[board].DAQ_ControlReg0, CTRL_REG_0);
        break;

      case ADC_SET_CONTROL_REG1:
        ChanADC[board][chan].controlReg1 = (u16) arg;
        writew(BoardData[board].DAQ_ControlReg1, CTRL_REG_1);
        break;

      case ADC_GET_DMA_BUF_SIZE:
	put_user(BoardData[board].dma_phy_size, (long *) arg);
	break;

      case ADC_NBIO_CANCEL:
        if (BoardData[board].nonBlockFile && (filePtr == BoardData[board].nonBlockFile || arg )) {
          /* disable acquisition */
          BoardData[board].DAQ_ControlReg1 &= ~CR1_SFT_GATE;
          writew(BoardData[board].DAQ_ControlReg1, CTRL_REG_1);

	  /* cancel DMA request to PLX here */
	  // bugfix: disabling the dma channel locks up the PLX on newer 4020's
	  // unless the READY bit is turned off.
	  // Follow mfg's example, by leaving the enable bit set,
	  // because turning off READY doesn't seem to be reliable 
	    
	  writeb(PLX_DMA_EN | PLX_DMA_ABORT, PLX_DMA1_CSR);
	  for(i = 0; i < 100000 && !(readb(PLX_DMA1_CSR) & PLX_DMA_DONE); i++) udelay(10);   // wait for DMA to stop
	  if (i == 100000) {
	    printk("ioctl ADC_NBIO_CANCEL: DMA cancel timed out\n");
	  }
	  
	  writew(0x0, ADC_FIFO_CLR);                    /* Clear ADC FIFO Pointer */
	  udelay(1000);
	  BoardData[board].DMAComplete = 0;
	  BoardData[board].nonBlockFile = NULL;         /* clear non-block flag */
        }
        break;

      case ADC_NBIO_PENDING:
        if (BoardData[board].nonBlockFile && filePtr == BoardData[board].nonBlockFile) {
          put_user(1, (long *) arg);
        } else {
          put_user(0, (long *) arg);
        }
        break;

      case ADC_NBIO_COMPLETE:
        if (BoardData[board].nonBlockFile && filePtr == BoardData[board].nonBlockFile &&
	    BoardData[board].DMAComplete) {
          put_user(1, (long *) arg);
	} else {
          put_user(0, (long *) arg);
	}
        break;

      case ADC_PSC_ENB:
        if (arg == 1) {
          ChanADC[board][chan].controlReg1  |= CR1_PSC_ENB;
        } else {
          ChanADC[board][chan].controlReg1  &= ~CR1_PSC_ENB;
        }
        #ifdef DEBUG
  	  printk(" ADC_PSC_ENB: ChanADC[%d][%d].controlReg1 = %#x\n", 
		 board, chan, ChanADC[board][chan].controlReg1);
         #endif
        break;
    }  /* switch */
  }

  if (chan >= AD_CHANNELS && chan < AD_CHANNELS+DIO_PORTS) {
    port = chan - AD_CHANNELS;
    switch (cmd) {
    case DIO_SET_MODE:
      arg &= 0x3;
      switch (port) {
      case 0:            /* Port A */
#ifdef DEBUG
	printk("DIO_SET_MODE for Port A\n");
#endif
	BoardData[board].dio_reg &= 0x9f;
	BoardData[board].dio_reg |= (arg << 5);
	writeb(BoardData[board].dio_reg, DIO_CNTRL_REG);
	ChanDIO[board][port].mode = arg;
	break;

      case 1:         /* Port 1B */
#ifdef DEBUG
	printk("DIO_SET_MODE for Port 1B\n");
#endif
	if (arg & 0x2)
	  return(-EINVAL);           /* Port 1B only has Modes 0 & 1 */
	BoardData[board].dio_reg &=  0xfb;
	BoardData[board].dio_reg |= (arg << 2);
	writeb(BoardData[board].dio_reg, DIO_CNTRL_REG);
	ChanDIO[board][port].mode = arg;
	break;

      case 2:         /* Port 1C */
#ifdef DEBUG
	printk("DIO_SET_MODE for Port 1C\n");
#endif
	if (arg)
	  return(-EINVAL);    /* Port 1C only has Mode 0 I/O */
	ChanDIO[board][port].mode = arg;
	break;

      default:
#ifdef DEBUG
	printk("DIO_SET_MODE for Invalid Port\n");
#endif
	return(-EINVAL);  /* Wrong Port Number */
	break;
      }
      break;

    case DIO_SET_DIRECTION:

#ifdef DEBUG
      printk("DIO_SET_DIRECTION: chan = %d, arg = %d\n", (int) chan, (int) arg );
#endif

      switch (port) {
      case 0:           /* Port A */
#ifdef DEBUG
	printk("DIO_SET_DIRECTION for Port A\n");
#endif
	arg &= 0x1;
	BoardData[board].dio_reg &=  0xef;
	BoardData[board].dio_reg |= (arg << 4);
	writeb(BoardData[board].dio_reg, DIO_CNTRL_REG);
	break;

      case 1:           /* Port B */
#ifdef DEBUG
	printk("DIO_SET_DIRECTION for Port B\n");
#endif
	arg &= 0x1;
	BoardData[board].dio_reg &=  0xfd;
	BoardData[board].dio_reg |= (arg << 1);
	writeb(BoardData[board].dio_reg, DIO_CNTRL_REG);
	break;

      case 2:           /* Port C */
#ifdef DEBUG
	printk("DIO_SET_DIRECTION for Port C\n");
#endif
	switch ( arg ) {
	case PORT_INPUT:         /* CU = IN  CL = IN */
	  BoardData[board].dio_reg |= 0x9;
	  break;
	case PORT_OUTPUT:        /* CU = OUT  CL = OUT */
	  BoardData[board].dio_reg &= ~(0x9);
	  break;
	case LOW_PORT_INPUT:     /* CL = IN */
	  BoardData[board].dio_reg |= 0x1;
	  break;
	case LOW_PORT_OUTPUT:    /* CL = OUT */
	  BoardData[board].dio_reg &= ~(0x1);
	  break;
	case HIGH_PORT_INPUT:    /* CU = IN */
	  BoardData[board].dio_reg |= 0x8;
	  break;
	case HIGH_PORT_OUTPUT:   /* CU = OUT */
	  BoardData[board].dio_reg &= ~(0x8);
	  break;
	default:
	  return(-EINVAL);
	  break;
	}
	writeb(BoardData[board].dio_reg, DIO_CNTRL_REG);
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
    }  /* end switch */
  }    /* end if */

  if (chan >= AD_CHANNELS+DIO_PORTS && chan < AD_CHANNELS+DIO_PORTS+DA_CHANNELS) {
    channel = chan - AD_CHANNELS - DIO_PORTS;
    switch (cmd) {
    case DAC_SET_GAINS:
      #ifdef DEBUG
        printk("ioctl DAC_SET_GAINS: Channel = %d, Gain = %ld\n", channel, arg);
      #endif
      ChanDAC[board][channel].gain = (u16) arg;
      switch (channel) {
      case 0:
	if (arg == BP_10_00V) {
	  BoardData[board].DAC_ControlReg &= ~DAC0R0;
	} else {
	  BoardData[board].DAC_ControlReg |= DAC0R0;
	}
	break;
      case 1:
	if (arg == BP_10_00V) {
	  BoardData[board].DAC_ControlReg &= ~DAC1R0;
	} else {
	  BoardData[board].DAC_ControlReg |= DAC1R0;
	}
	break;
      }
      writew(BoardData[board].DAC_ControlReg, DAC_CTRL_REG);
      break;
    case DAC_GET_GAINS:
      put_user((long)ChanDAC[board][channel].gain, (long*) arg);
      break;
    }
  }
  return 0;
}

static inline void *get_dmabuf_vaddr(BoardRec *boardData, long offset)
{
  return ((char *)boardData->dma_virt_tbl[offset / boardData->dma_tbl_page_size]) + 
    (offset % boardData->dma_tbl_page_size);
}

static inline dma_addr_t get_dmabuf_pciaddr(BoardRec *boardData, long offset)
{
  return boardData->dma_bus_addr_tbl[offset / boardData->dma_tbl_page_size] + 
    (offset % boardData->dma_tbl_page_size);
}

/*****************************************************
 *                                                   *
 * mmap() service handler                            *
 *                                                   *
 *****************************************************/

static int das4020_mmap( struct file *filePtr, struct vm_area_struct *vma ) 
{
  int board = 0;
  int chan  = 0;
  int minor;
  int size  = vma->vm_end - vma->vm_start;
  struct inode *iNode = filePtr->f_dentry->d_inode;
  unsigned j;
  unsigned long pos;

  minor = iminor(iNode);
  board = BOARD(minor);          /* get which board   */
  chan =  CHAN(minor);           /* get which channel */

  if (chan >= 0 && chan < AD_CHANNELS) {
    /* Mmap requested for ADC. Note: It does not matter for which ADC, all ADC's share the buffer.*/

    #ifdef DEBUG
    printk("%s: das4020_mmap(): board %d chan %d.\n", ADAPTER_ID, board, chan);
    printk("das4020_mmap(): vma->vm_start = %#lx  vma->vm_end= %#lx size = %#x offset = %#lx\n",
	   vma->vm_start, vma->vm_end, size, vma->vm_pgoff);
    #endif
    
    if(vma->vm_pgoff != 0) {     // You have to map the entire buffer.
      #ifdef DEBUG
        printk("das4020_mmap(): The page offset has to be zero \n");
      #endif
      return(-EINVAL);
    }

    if (size > BoardData[board].dma_phy_size) {     // You cannot request more than the max buffer
      printk("das4020_mmap(): Size = %d is too large.\n", size);
      return(-EINVAL);
    }

    /*
    vma->vm_ops = &das4020_vops;
    vma->vm_flags |= VM_RESERVED;
    vma->vm_private_data = (void *) minor;
    */

    
    for (j = 0, pos = vma->vm_start;
	 j < BoardData[board].dma_tbl_size && pos < vma->vm_end;
	 j++, pos += BoardData[board].dma_tbl_page_size) {
      if (remap_pfn_range(vma, vma->vm_start+j*BoardData[board].dma_tbl_page_size,
	  virt_to_phys((void *)BoardData[board].dma_virt_tbl[j]) >> PAGE_SHIFT,
	  (vma->vm_end-pos >= BoardData[board].dma_tbl_page_size) ? BoardData[board].dma_tbl_page_size : (vma->vm_end-pos),
			  vma->vm_page_prot) < 0) {
	printk("remap_pfn_range failed\n");
	return -EIO;
      }
    }
  }
  return 0;
}

/***************************************************************************
 *
 * poll() service handler
 *
 ***************************************************************************/

static unsigned int das4020_poll( struct file *filePtr, poll_table *wait )
{
  struct inode *iNode = filePtr->f_dentry->d_inode;
  unsigned int mask = 0;
  int board = 0;
  int minor = 0;

  minor = iminor(iNode);
  board = BOARD(minor);            /* get which board   */
  
  /* tell select()/poll() to wake up and give us a call if
     das4020_12_wait is awakened
  */
  poll_wait(filePtr, &das4020_12_wait, wait);
 
  /* can we currently do a non-blocking read? */
  if (BoardData[board].nonBlockFile && BoardData[board].DMAComplete) {
    mask |= POLLIN|POLLRDNORM;
  }
  return mask;
}

/***************************************************************************
 *
 * Interrupt handler used to service interrupt read().
 *
 ***************************************************************************/

static irqreturn_t das4020_Interrupt( int irq, void *dev_id ) 
{
  u32 plx_intcsr;
  u8  plx_dma1_csr;
  BoardRec *boardData = dev_id;
  u8 *base0 = boardData->base0;

  spin_lock_irq(&das4020_lock);
   
#ifdef DEBUG
  printk("a2dc_2_4.c(): This is from the Interrupt Service Routine\n");
#endif

  plx_intcsr = readl(PLX_INTCSR);

  if (plx_intcsr & PLX_LCL_DMA1_INT) {
    plx_dma1_csr = readb(PLX_DMA1_CSR);
    #ifdef DEBUG
      printk("das4020_Interrupt: DMA Chan 1 Command/Status Register = %#x\n",plx_dma1_csr);
    #endif
    plx_dma1_csr |= PLX_DMA_CLR;
    writeb(plx_dma1_csr, PLX_DMA1_CSR);

    #ifdef DEBUG
      printk("das4020_Interrupt: DMA related interrupt has occured\n");
      //    printk("das4020_Interrupt: Hardware Status Register = %#x\n", readw(HW_STAT_REG));
    #endif

    boardData->DMAComplete = 1;
    boardData->WordsToRead = 0;
    wake_up_interruptible(&das4020_12_wait);
  } else if ((plx_intcsr & (PLX_PCI_DOORBELL_INT | PLX_PCI_ABORT_INT | PLX_PCI_LOCAL_INT)) != 0) {
    #ifdef DEBUG
      printk("das4020_Interrupt: pxl_intcsr = %#x\n", plx_intcsr);
      printk("Non-DMA related interrupt.\n");
    //  printk("Hardware Status Register = %#x\n", readw(HW_STAT_REG));
    #endif
    wake_up_interruptible(&das4020_12_wait);
  } 
  spin_unlock_irq(&das4020_lock);

  return IRQ_HANDLED;
}


/***************************************************************************
 * Reading the values from EEPROM involves issuing the EEPROM_READ(0x06XX) *
 * command to the appropriate address and clocking back the 16-bit data    *
 * value. Access to the EEPROM is via BADR0 + 0x006C. Note that EEPROM     *
 * addresses are 7 bits and data values are 16 bits.                       *
 ***************************************************************************/
static void clk_EEPROM( BoardRec *boardData ) 
{
  /*
    Send clocking sequence to National Semiconductor
    or Fairchild  NMC93S56/FM93CS56  EEPROM
  */
  u32 regval;
  u8 *base0 = boardData->base0;
  
  regval = readl(PLX_CNTRL) | PLX_USEROUT;
  writel(regval | PLX_EECK, PLX_CNTRL);
  writel(regval & ~PLX_EECK, PLX_CNTRL);
}

/* Writes the read command of the EEPROM */
static void writeReadCmd( u8 addr, BoardRec *boardData ) 
{
  /****************************************************************************************
   *  Instruction |   Start Bit  |   Opcode Field  |             Address                  *
   *  ----------  |   --------   |   ------------- |            ---------                 *
   *   READ       |      1       |      10         |     X A6 A5 A4 A3 A2 A1 A0           *
   *                                                                                      *
   ****************************************************************************************/

  const int MAX_DATA_BIT = 10;    /* Position of leading 1 in opcode                */
  u32 regval;                     /* value of PLX_CNTRL                             */
  int i; 
  u16 opcode = 0x600 | addr;      /* read opcode and address                        */
  u8 *base0 = boardData->base0;
  
  regval = readl(PLX_CNTRL);
  regval |= (PLX_EECS | PLX_USEROUT);      /* Select the EEPROM Chip                             */
  regval &= ~(PLX_EECK | PLX_EEWD);        /* Clear the EEPROM Clock and instruction(write) bits */
  writel(regval & ~PLX_EECS, PLX_CNTRL);   
  writel(regval, PLX_CNTRL);

  for (i = MAX_DATA_BIT; i >= 0; i--) { /* Write in the Opcode for Read Command               */
    regval &= ~PLX_EEWD;
    if (opcode & (0x1 << i)) {
      regval |= PLX_EEWD;
    }
    writel(regval, PLX_CNTRL); 
    clk_EEPROM(boardData);
  } 
}

/* Reads from the EEPROM location @ addr */
static u16 read_pci_das4020_EEPROM( u8 addr, BoardRec *boardData ) 
{
  const int MAX_DATA_BIT = 15;
  int i;
  u16 coeff = 0x0;
  u8 *base0 = boardData->base0;
  
  writeReadCmd(addr, boardData);       /* Issue the Read Command              */
  
  for (i = MAX_DATA_BIT; i >= 0; i--) { /* Read the value by clocking 16 times */
    clk_EEPROM(boardData);     
    if (readl(PLX_CNTRL) & PLX_EERD) {
      coeff |= 0x1 << i;
    }
  }
  return coeff;
}

/* Reads the calibration coeffs (gain and offset) from the EEPROM  and stores it in cal_data array */
static void store_calibration_coef( BoardRec *boardData ) 
{
  int i;
  u8 addr = DAQ_CALIB_COEFF;
  
  /* Fetching the coefficients from the address DAQ_CALIB_COEFF onwards on the EEPROM */
  for (i = 0; i < 16; i++) {
    boardData->cal_data[i] = read_pci_das4020_EEPROM(addr++, boardData);
#ifdef DEBUG
    printk("store_calibration_coef: boardData->cal_data[%d] = %#x , %d\n", i,
	   boardData->cal_data[i], boardData->cal_data[i]);
#endif
  }
}

/*
 * --------------------------------------------------------------------
 *			AD5315 Calibration Routines
 * --------------------------------------------------------------------
 */

#define AD5315_UDELAY 2000

static void clk_ad5315( BoardRec *boardData )
{
  u8 *base0 = boardData->base0;
  u32 regval = readl(PLX_CNTRL);
  
  udelay (AD5315_UDELAY);
  
  writel(regval & ~PLX_USEROUT, PLX_CNTRL);
  udelay (AD5315_UDELAY);

  writel(regval | PLX_USEROUT, PLX_CNTRL);
  udelay (AD5315_UDELAY);
}

/**************************************************************************************
  Note on the AD5315
  SDA = Serial Data Line.  Bit number 26 of PLX_CNTRLN (Write Bit to serial of EEPROM)
  SCL = Serial Clock Line. Bit number 16 of PLX_CNTRL  (General Purpose Output)
***************************************************************************************/

static void write_byte_AD5315( u8 byte, BoardRec *boardData )
{
    u32 regval = 0;
    int bitNo;
    u8 *base0 = boardData->base0;

    for (bitNo = 7; bitNo > -1; bitNo--) {
      regval = readl(PLX_CNTRL) & ~PLX_EEWD; 
      regval |= (byte & (0x1 << bitNo) ? 0 : PLX_EEWD);
      writel(regval, PLX_CNTRL);
      clk_ad5315(boardData);
    }

    /* disable writes */
    regval = readl(PLX_CNTRL);
    writel(regval & ~PLX_EEWD,  PLX_CNTRL);
    clk_ad5315(boardData);
}

static void write_data_AD5315( u8 addr, u8 dac_select, u16 value, BoardRec *boardData )
{
  u8 data[3];
  u8 *base0 = boardData->base0;
  u32 regval = readl(PLX_CNTRL);

  /******************************************************************************************
   *           Most Significant Data Byte                    Least Significant Data Byte    *
   *             --    ---                                                                  *
   * PD1 | PD0 | CLR | LDAC | D9 | D8 | D7 | D6       D5 | D4 | D3 | D2 | D1 | D0 | X | X   *
   *                                                                                        *
   ******************************************************************************************/
 
  data[0] = dac_select;                      // pointer byte
  data[1] = 0x20 | ((value >> 6) & 0xf);     // MSB data 
  data[2] = (value << 2 ) & 0xfc;            // LSB data 

  regval &= ~(PLX_EEWD | PLX_USEROUT);
  writel(regval, PLX_CNTRL);
  udelay (AD5315_UDELAY);
  regval |= PLX_EEWD;
  writel(regval, PLX_CNTRL);
  udelay (AD5315_UDELAY);
 
  regval |= PLX_USEROUT;
  writel(regval, PLX_CNTRL);

  write_byte_AD5315(addr, boardData);
  udelay (AD5315_UDELAY);

  write_byte_AD5315(data[0], boardData);
  udelay (AD5315_UDELAY);
  write_byte_AD5315(data[1], boardData);
  udelay (AD5315_UDELAY);
  write_byte_AD5315(data[2], boardData);
  udelay (AD5315_UDELAY);

  regval = readl(PLX_CNTRL);
  regval |= PLX_EEWD;
  writel(regval, PLX_CNTRL);
  udelay (AD5315_UDELAY);

  regval &= ~(PLX_USEROUT);
  writel(regval, PLX_CNTRL);
  udelay (AD5315_UDELAY);

  regval &= ~(PLX_EEWD);
  writel(regval, PLX_CNTRL);
  udelay (AD5315_UDELAY);
}

static void writeCalFactors( u8  chan, u16 offset, u16 gain, BoardRec *boardData )
{
  u8 addr = ( chan < 2 ? I2C_CAL_DAC0_ADDR : I2C_CAL_DAC1_ADDR );
  u8 trimDACSel = 0;

  // updating the offset ...
  trimDACSel = (chan & 1 ? 0x4 : 0x1);
  write_data_AD5315( addr, trimDACSel, offset, boardData );

  // updating the gain ...
  trimDACSel = (chan & 1 ? 0x8 : 0x2);
  write_data_AD5315( addr, trimDACSel, gain, boardData );
}

/*
 * --------------------------------------------------------------------
 *			I2C i/o routines
 * --------------------------------------------------------------------
 */
#define	I2C_UDELAY	   2000

static void clk_i2c( BoardRec *boardData )
{
  u8 *base0 = boardData->base0;
  u32 regval = readl(PLX_CNTRL);
  
  udelay (I2C_UDELAY);
  
  writel(regval & ~PLX_USEROUT, PLX_CNTRL);
  udelay (I2C_UDELAY);

  writel(regval | PLX_USEROUT, PLX_CNTRL);
  udelay (I2C_UDELAY);
}

static void write_byte_i2c( u8 byte, BoardRec *boardData )
{
    u32 regval = 0;
    int bitNo;
    u8 *base0 = boardData->base0;
    
    for (bitNo = 7; bitNo > -1; bitNo--) {
      regval = readl(PLX_CNTRL) & ~PLX_EEWD; 
      regval |= (byte & (0x1 << bitNo) ? 0 : PLX_EEWD);
      writel(regval, PLX_CNTRL);
      clk_i2c(boardData);
    }

    /* disable writes */
    regval = readl(PLX_CNTRL);
    writel(regval & ~PLX_EEWD,  PLX_CNTRL);
    clk_i2c(boardData);
}

static void write_data_i2c( u16 value, BoardRec *boardData )
{
  u8 data[2];
  u8 *base0 = boardData->base0;
  u32 regval = readl(PLX_CNTRL);

  data[0] = (value >> 8);  // MSB
  data[1] = value & 0xff;  // LSB

  regval &= ~(PLX_EEWD | PLX_USEROUT);
  writel(regval, PLX_CNTRL);
  udelay (I2C_UDELAY);
  regval |= PLX_EEWD;
  writel(regval, PLX_CNTRL);
  udelay (I2C_UDELAY);

  regval |= PLX_USEROUT;
  writel(regval, PLX_CNTRL);

  write_byte_i2c(I2C_REG, boardData);
  udelay (I2C_UDELAY);

  write_byte_i2c(data[0], boardData);
  udelay (I2C_UDELAY);
  write_byte_i2c(data[1], boardData);
  udelay (I2C_UDELAY);

  regval = readl(PLX_CNTRL);
  regval |= PLX_EEWD;
  writel(regval, PLX_CNTRL);
  udelay (I2C_UDELAY);

  regval &= ~(PLX_USEROUT);
  writel(regval, PLX_CNTRL);
  udelay (I2C_UDELAY);

  regval &= ~(PLX_EEWD);
  writel(regval, PLX_CNTRL);
  udelay (I2C_UDELAY);
}

/* fill shadows with invalid values */
inline static void initialise_i2c( BoardRec *boardData ) 
{
  int i;

  boardData->i2c_reg = ~0;
  for (i = 0; i < 8; i++) {
    boardData->cal_coef[i] = ~0;
  }
}

/* Enables Interrupt in the PLX control register */
static void initialise_plx( BoardRec *boardData ) 
{
  u32 plx_cntrl;
  u8 *base0 = boardData->base0;
  
  plx_cntrl = readl(PLX_CNTRL);
  plx_cntrl |= PLX_PCI_IE;
  writel(plx_cntrl, PLX_CNTRL);
}

/* Gets the calibration coefs for the specified channel and range */
static void load_calibration_coef( int channel, int bitmask, BoardRec *boardData ) 
{
  u16 gain;
  u16 offset;
  u8 addr = 0;

  addr = 4*channel;                                    /* offset of 4 for each channel            */
  addr += (((bitmask >> (channel+4)) & 0x1) ? 0 : 2);  /* First +/- 5V, Second +/- 1V             */

  gain = boardData->cal_data[addr++];
  offset = boardData->cal_data[addr];

#ifdef DEBUG
  printk("load_calibration_coef():\n");
  printk("Channel = %d, Range =  %s \n", channel,
       ((bitmask >> (channel+4)) & 0x1) ? "BP_5_00V" : "BP_1_00V");
  printk("Gain Read = %d, Offset Read  %d\n",gain, offset);
#endif

  writeCalFactors(channel, offset, gain, boardData);
}

/*******************************************************************************/

static void set_pacer( u32 freq, BoardRec *boardData ) {
  
  u32 divisor;
  u8 *base2 = boardData->base2;

  divisor =  (BASE_CLK_FREQ / freq) - 2;
  boardData->scan_rate = (u32) freq;
  boardData->SampleIntervalLowReg = divisor & 0xffff;
  boardData->SampleIntervalHighReg = (divisor >> 16) & 0xff;
  writew(boardData->SampleIntervalLowReg, SAMPLE_INT_LOW);
  udelay(100);
  writew(boardData->SampleIntervalHighReg, SAMPLE_INT_HIGH);
  udelay(100);

  #ifdef DEBUG
  printk("set_pacer: freq = %d  divisor = %d  LowReg = %#x  HiReg = %#x\n",
	 freq, divisor,   boardData->SampleIntervalLowReg, boardData->SampleIntervalHighReg);
  #endif
}

/*******************************************************************************/

const static unsigned long chan_enables[4] = {
  CH0_EN,
  CH1_EN,
  CH2_EN,
  CH3_EN,
};

static int find_first_chan( unsigned long bitmask ) 
{
  int   i;

  for (i = 0; i < 4; i++) {
    if (bitmask & chan_enables[i]) {
      return i;
    }
  }
  return -1;
}

static int find_second_chan( unsigned long bitmask ) 
{
  int   i;

  for (i = find_first_chan(bitmask) + 1; i < 4; i++) {
    if (bitmask & chan_enables[i]) {
      return i;
    }
  }
  return -1;
}

/* Sets up the following:
 * I2C register
 * H/W config register
 * Sample Interval Registers
 * DAQ Control Reg 1
 */
static int set_config( BoardRec *boardData ) 
{
  int       nchan;
  int       ireg;
  int       ii;
  int       hw_conf;
  unsigned long bitmask;
  unsigned long scan_rate;
  unsigned long max_scan_rate = 0;
  unsigned long r;
  u8 *base2 = boardData->base2;

  bitmask = boardData->bitmask;
  scan_rate = boardData->scan_rate;

  if (scan_rate < MIN_SCAN_RATE) {
#ifdef DEBUG
    printk("set_config: scan rate too low (%ld)\n", scan_rate);
#endif
    return 0;
  }

  /* count # of channels selected */
  nchan = 0;
  if (bitmask & CH0_EN) nchan++;
  if (bitmask & CH1_EN) nchan++;
  if (bitmask & CH2_EN) nchan++;
  if (bitmask & CH3_EN) nchan++;

  switch (nchan) {       /* legal values are 1, 2 and 4 */
    case 1:    max_scan_rate = MAX_SCAN_RATE_1;    break;
    case 2:    max_scan_rate = MAX_SCAN_RATE_2;    break;
    case 4:    max_scan_rate = MAX_SCAN_RATE_4;    break;
    default:
    #ifdef DEBUG
      printk("set_config: invalid # of channels (%d)\n", nchan);
    #endif
    return 0;
  }

  //if ((boardData->bitmask & CLK_MASK) != CLK_INTERNAL) max_scan_rate = MAX_SCAN_RATE_4;
  if (scan_rate > max_scan_rate) {
#ifdef DEBUG
    printk("set_config: scan rate too high (%ld)\n", scan_rate);
#endif
    return 0;
  }

  /* check clock source */
  switch (bitmask & CLK_MASK) {
    case CLK_INTERNAL:          hw_conf = WCLK_INTERNAL_40MHZ;    break;
    case CLK_EXT_BNC:           hw_conf = WCLK_EXT_CLK_BNC;       break;
    case CLK_AD_START_TRIG_IN:  hw_conf = WCLK_AD_START_TRIG_IN;  break;
    default:
      #ifdef DEBUG
        printk ("set_config: invalid CLK setting (0x%08lx)\n",
	         bitmask & CLK_MASK);
      #endif
      return 0;
  }

  /*  program the i2c register */
  ireg = 0;
  if (bitmask & EXT_BNC_THRESH_ZERO) ireg |= IREG_THRESH_ZERO;

  /* If using +/-5 Volts then set the attenuation bit */
  if (bitmask & CH0_5V) ireg |= IREG_ATTEN_CH0;
  if (bitmask & CH1_5V) ireg |= IREG_ATTEN_CH1;
  if (bitmask & CH2_5V) ireg |= IREG_ATTEN_CH2;
  if (bitmask & CH3_5V) ireg |= IREG_ATTEN_CH3;

  switch (bitmask & ASRC_MASK) {
    case ASRC_BNC:        ireg |= IREG_ASRC_BNC;       break;
    case ASRC_CAL_AGND:   ireg |= IREG_ASRC_CAL_AGND;  break;
    case ASRC_CAL_0_625:  ireg |= IREG_ASRC_CAL_0_625; break;
    case ASRC_CAL_4_375:  ireg |= IREG_ASRC_CAL_4_375; break;
    case ASRC_CAL_HDR:    ireg |= IREG_ASRC_CAL_HDR;   break;
    default:
      #ifdef DEBUG
        printk( "set_config: invalid ASRC (0x%08lx)\n", bitmask & ASRC_MASK);
      #endif
      return 0;
      break;
  }

  /* config is OK...*/
  write_data_i2c(ireg, boardData);

  boardData->HardwareConfigReg = hw_conf;
  writew(boardData->HardwareConfigReg, HW_CONFIG_REG);
  
  boardData->bitmask = bitmask;
  boardData->scan_rate = scan_rate;
  boardData->nchannels = nchan;

#ifdef DEBUG
  printk("set_config: bitmask = %#lx   scan_rate = %ld   nchannels = %d   hw_config = %#x\n",
	 bitmask, scan_rate, nchan, hw_conf);
#endif

  // We program the scan_rate divisor, but set_pacer() is called at the beginning 
  // of all the conversions anyway. We call set_pacer here for good measure
  set_pacer(boardData->scan_rate, boardData);

  /* compute throughput (bytes/second) */
  
  boardData->throughput = scan_rate * nchan * sizeof (u16);

  /* setup selected channels  */
  r = boardData->DAQ_ControlReg1;
  r &= ~(CR1_CHANMODE_MASK | CR1_UCHAN_MASK | CR1_LCHAN_MASK | CR1_ATRIG_MD_MASK);
  r |= CR1_ATRIG_MD_INACTIVE;   // no analog trigger

  switch (nchan) {
    case 1:
      r |= CR1_CHANMODE_1;
      ii = find_first_chan (bitmask);
      r |= CR1_LCHAN (ii);
      r |= CR1_UCHAN (ii);
      break;
    case 2:
      r |= CR1_CHANMODE_2;
      r |= CR1_LCHAN (find_first_chan (bitmask));
      r |= CR1_UCHAN (find_second_chan (bitmask));
      break;
    case 4:
      r |= CR1_CHANMODE_4;
      break;
  }
  
  boardData->DAQ_ControlReg1 = r;
  writew(boardData->DAQ_ControlReg1, CTRL_REG_1);
  return 1;
}


/***************************************************************************
 *                                                                         *	
 * Handles software triggered read().                                      *	
 *                                                                         *
 * Bang! Bang! Method:                                                     *
 *                                                                         *
 *    o Get one sample at a time and put it in the kernel buffer           *
 *    o Get chan->count samples                                            *
 *                                                                         *
 ***************************************************************************/
static int adc_soft_conversion( ADC_ChanRec *chanRec, BoardRec *boardData ) 
{
  int i;
  unsigned long j = jiffies + HZ;
  u16 hwStatusRegister = 0;
  u8 *base2 = boardData->base2;
  u8 *base3 = boardData->base3;
  u32 *ADC_KernBuffPtr = boardData->dma_virt_tbl[0];
  int offset = 0;
  
#ifdef DEBUG
  u16* wValue;
  printk("Entering adc_soft_conversion()  Channel %d: Words to read = %d\n",
	 chanRec->lowChan , chanRec->count);
#endif

  /* disable acquisition */
  writew(0x0, CTRL_REG_0);
  writew(0x0, CTRL_REG_1);
  writew(0x0, INTRPT_EN_REG);

  /* Inserting appropriate counts into the samplescan registers */
  boardData->WordsToRead = chanRec->count;
  boardData->SampleScanHighReg = (boardData->WordsToRead >> 16) & 0xff;
  writew(boardData->SampleScanHighReg, SAMPLE_SCAN_REG_HIGH);
  boardData->SampleScanLowReg = boardData->WordsToRead & 0xffff;
  writew(boardData->SampleScanLowReg, SAMPLE_SCAN_REG_LOW);
  udelay(1000);            /* Give the hardware some time to settle down! */

  /* Clear ADC FIFO Pointer*/
  writew(0x0, ADC_FIFO_CLR);
  udelay(1000);

  /* set pacer frequency */
  set_pacer(chanRec->frequency, boardData);
  #ifdef DEBUG
    printk("adc_soft_conversion(): Pacer Frequency = %d\n",  chanRec->frequency);
  #endif

  /* Only enabling the DAQ, DMA disabled and selecting software trigger source for Trig1
     the software gate with level-sensitivity, enabling samplecount
  */
  if (chanRec->controlReg0 != 0) {
    boardData->DAQ_ControlReg0 = chanRec->controlReg0;
  } else {
    boardData->DAQ_ControlReg0 = (DMA_DSBL | SAMPCNTENB |  AGATE_LVL); 

    switch (chanRec->trigger) {
    case DISABLE:
      break;
    case SOFT_TRIGGER:
      boardData->DAQ_ControlReg0 |= TRIG1_SRC0;
      break;
    case EXTERNAL_TRIGGER:
      boardData->DAQ_ControlReg0 |= TRIG1_SRC1;
      break;
    case ANALOG_TRIGGER:
      boardData->DAQ_ControlReg0 |= (TRIG1_SRC1 | TRIG1_SRC0);
      break;
    }
    switch (chanRec->gate) {
    case DISABLE:
      break;
    case SOFT_GATE:
      boardData->DAQ_ControlReg0 |= AGATE_SRC0;
      break;
    case EXTERNAL_GATE:
      boardData->DAQ_ControlReg0 |= AGATE_SRC1;
      break;
    case ANALOG_GATE:
      boardData->DAQ_ControlReg0 |= (AGATE_SRC1 | AGATE_SRC0);
      break;
    }
  }
  writew(boardData->DAQ_ControlReg0, CTRL_REG_0);

  /* Set up DAQ Control Register 1 */
  boardData->DAQ_ControlReg1 = chanRec->controlReg1;
  boardData->DAQ_ControlReg1 |= (chanRec->lowChan << 8) | (chanRec->hiChan << 10);
  boardData->AtrigLowReg = chanRec->analog_trig_low;
  boardData->AtrigHighReg = chanRec->analog_trig_high;
  writew(boardData->AtrigLowReg , ATRIG_LOW_REG);
  writew(boardData->AtrigHighReg, ATRIG_HIGH_REG);
  
  if ((chanRec->atrigMode & CR1_ATRIG_MD_MASK) != INACTIVE) {
    boardData->DAQ_ControlReg1 &= ~(0x3f);
    writew(boardData->DAQ_ControlReg1, CTRL_REG_1);
    udelay(100);
    boardData->DAQ_ControlReg1 |= chanRec->atrigMode;
  } else {
    boardData->DAQ_ControlReg1 &= ~(0xf);
    boardData->DAQ_ControlReg1 |= CR1_SFT_GATE;
  }
  writew(boardData->DAQ_ControlReg1, CTRL_REG_1);

  /* Enable Data Acquisition */
  boardData->DAQ_ControlReg0 |= DAQ_ENB;
  writew(boardData->DAQ_ControlReg0, CTRL_REG_0); 
  udelay(1500);              /* Give the hardware some time to settle down! */


#ifdef DEBUG
  printk("soft_conversion(): Channel %d: Control Reg 0  = %#x   Control Reg 1 = %#x\n",  
           chanRec->lowChan, boardData->DAQ_ControlReg0, boardData->DAQ_ControlReg1);
 #endif

  while( boardData->WordsToRead > 0 ) {
    /* Issue soft start command */
    if (boardData->DAQ_ControlReg1 & CR1_SFT_GATE) {
      writew(0x1, DAQ_START_REG);   /* Force first conversion */
    }

    /* Poll until done */
    while ((hwStatusRegister = readw(HW_STAT_REG))) {
      if (jiffies > j) {  /* we got a problem */
        printk("adc_soft_conversion ERROR: DAQ_DONE bit not set after 1 second.\n");
        printk("adc_soft_conversion ERROR: Hardware Status Register = %#x.\n", hwStatusRegister);
        return -1;
      }

      if (hwStatusRegister & DAQ_DONE) {
	boardData->DAQ_ControlReg1 &= ~CR1_SFT_GATE; 
	writew(boardData->DAQ_ControlReg1, CTRL_REG_1);
        ADC_KernBuffPtr = (u32*) get_dmabuf_vaddr(boardData, offset);
	for (i = 0; i < (boardData->WordsToRead + 1)/2; i++) {     /* 2 samples per read */
	  *ADC_KernBuffPtr++ = readl(FIFO);                        /* Load into buffer   */
	}
	BoardData->WordsToRead = 0;
        #ifdef DEBUG
 	  wValue = boardData->dma_virt_tbl[0];
          printk("adc_soft_conversion(): HW Status Reg  = %#x\n",readw(HW_STAT_REG));
          printk("ADC Read Pointer register = %#x\n",  readw(ADC_READ_REG));
          printk("ADC Write Pointer register = %#x\n",  readw(ADC_WRITE_REG));
          printk("adc_soft_conversion(): value = %#x.\n", *(wValue));
        #endif
	return 0;
      }
      if (hwStatusRegister & ASRC_FLG) {
	boardData->DAQ_ControlReg1 &= ~CR1_SFT_GATE; 
	writew(boardData->DAQ_ControlReg1, CTRL_REG_1);
        ADC_KernBuffPtr = (u32 *) get_dmabuf_vaddr(boardData, offset);
  	for (i = 0; i < boardData->half_FIFO_size/4; i++) {     /* 2 samples per read */
	  *ADC_KernBuffPtr++ = readl(FIFO);                     /* Load into buffer   */
	}
	boardData->WordsToRead -= boardData->half_FIFO_size/2;
        offset += boardData->half_FIFO_size;
	boardData->DAQ_ControlReg1 |= CR1_SFT_GATE; 
	writew(boardData->DAQ_ControlReg1, CTRL_REG_1); 
	break;
      }

      if (hwStatusRegister & DAQ_OVERRUN) {
        printk("adc_soft_conversion ERROR: DAQ_OVERRUN bit  set.\n");
	boardData->WordsToRead = 0;
	break;
      }

      if (hwStatusRegister & PIPEFULL) {
        printk("adc_soft_conversion ERROR: PIPEFULL = %#x .\n", hwStatusRegister&PIPEFULL);
	boardData->WordsToRead = 0;
	break;
      }
      udelay(100);
    }
  }

  // Disabling the Gate
  boardData->DAQ_ControlReg1 &= ~CR1_SFT_GATE; 
  writew(boardData->DAQ_ControlReg1, CTRL_REG_1);
  

  return -1;
}

/***************************************************************************
 *                                                                         *
 * Handles dma read().                                                     *
 *                                                                         *
 *                                                                         *
 *    o Set up dma chains.                                                 *
 *                                                                         *
 ***************************************************************************/

static int dma_read( ADC_ChanRec *chanRec, struct file *readfile, BoardRec *boardData )
{
  u32 plx_intcsr;
  int i;
  int number_of_desc;
  u8 *base0 = boardData->base0;
  u8 *base2 = boardData->base2;
  int offset = 0;

  /*
    For Non Blocking Read:  check to see if all the words
    have been read and the DMA is complete.  If we are all
    done, clear the chain and exit
  */

  if (chanRec->nonBlockFlag && boardData->nonBlockFile) {  
    if (boardData->DMAComplete == 1) {
      free_chain(boardData);
      boardData->nonBlockFile = NULL;
      return 0;
    } else {
      return -EAGAIN;
    }
  }
  
  boardData->WordsToRead = chanRec->count;

  /* Setting up the DAQ regs */
  
  /* disable acquisition */
  writew(0x0, CTRL_REG_0);
  writew(0x0, CTRL_REG_1);
  writew(0x0, INTRPT_EN_REG);

  /* Inserting appropriate counts into the sample scan registers */
  boardData->SampleScanHighReg = (boardData->WordsToRead >> 16) & 0xff;
  writew(boardData->SampleScanHighReg, SAMPLE_SCAN_REG_HIGH);
  boardData->SampleScanLowReg = boardData->WordsToRead & 0xffff;
  writew(boardData->SampleScanLowReg, SAMPLE_SCAN_REG_LOW);
  udelay(100);            /* Give the hardware some time to settle down! */

  /*Clear ADC FIFO Pointer*/
  writew(0x0, ADC_FIFO_CLR);
  udelay(1000);

  /* set pacer frequency */
  set_pacer(chanRec->frequency, boardData);
#ifdef DEBUG
  printk("dma_read(): Pacer Frequency = %d\n",  chanRec->frequency);
#endif

  if (chanRec->controlReg0 != 0) {
    boardData->DAQ_ControlReg0 = chanRec->controlReg0;
  } else {
    boardData->DAQ_ControlReg0 = ( AGATE_LVL ); 

    switch (chanRec->trigger) {
    case DISABLE:
      break;
    case SOFT_TRIGGER:
      boardData->DAQ_ControlReg0 |= TRIG1_SRC0;
      break;
    case EXTERNAL_TRIGGER:
      boardData->DAQ_ControlReg0 |= TRIG1_SRC1;
      break;
    case ANALOG_TRIGGER:
      boardData->DAQ_ControlReg0 |= (TRIG1_SRC1 | TRIG1_SRC0);
      break;
    }
    switch (chanRec->gate) {
    case DISABLE:
      break;
    case SOFT_GATE:
      boardData->DAQ_ControlReg0 |= AGATE_SRC0;
      break;
    case EXTERNAL_GATE:
      boardData->DAQ_ControlReg0 |= AGATE_SRC1;
      break;
    case ANALOG_GATE:
      boardData->DAQ_ControlReg0 |= (AGATE_SRC1 | AGATE_SRC0);
      break;
    }
  }
  writew(boardData->DAQ_ControlReg0, CTRL_REG_0);

  /* Set up DAQ Control Register 1 */
  boardData->DAQ_ControlReg1 = chanRec->controlReg1;
  boardData->DAQ_ControlReg1 |= (chanRec->lowChan << 8) | (chanRec->hiChan << 10);
  boardData->AtrigLowReg = chanRec->analog_trig_low;
  boardData->AtrigHighReg = chanRec->analog_trig_high;
  writew(boardData->AtrigLowReg , ATRIG_LOW_REG); 
  writew(boardData->AtrigHighReg, ATRIG_HIGH_REG);

  if ((chanRec->atrigMode & CR1_ATRIG_MD_MASK) != INACTIVE) {
    boardData->DAQ_ControlReg1 &= ~(0x3f);
    writew(boardData->DAQ_ControlReg1, CTRL_REG_1);
    udelay(100);
    boardData->DAQ_ControlReg1 |= chanRec->atrigMode;
  } else {
    boardData->DAQ_ControlReg1 &= ~(0xf);
    // boardData->DAQ_ControlReg1 |= CR1_SFT_GATE;
  }
  writew(boardData->DAQ_ControlReg1, CTRL_REG_1);

#ifdef DEBUG
  printk("dma_read(): Channel %d: Control Reg 0  = %#x   Control Reg 1 = %#x\n",  
           chanRec->lowChan, boardData->DAQ_ControlReg0, boardData->DAQ_ControlReg1);
#endif

  /* Don't enable Interrupts */
  boardData->InterruptEnableReg = OVERRUN;
  writew(boardData->InterruptEnableReg, INTRPT_EN_REG); 
	
#ifdef DEBUG
  printk("dma_read(): WordsToRead = %d\n", boardData->WordsToRead);
#endif
 
  /***********************************
   * Set up the DMA in chaining mode *
   ***********************************/

  /* 1.Create the DMA descriptor structures based on the count that is specified in chanRec->count */
  number_of_desc = (2*chanRec->count)/boardData->half_FIFO_size;
  if (2*chanRec->count % boardData->half_FIFO_size) number_of_desc++;
   
  boardData->chain_desc_size = sizeof(struct dma_desc)*number_of_desc; // must be less than 2MB
  boardData->chain_desc = pci_alloc_consistent(boardData->pdev,
					       boardData->chain_desc_size,
					       &boardData->chain_desc_bus_addr);
  memset(boardData->chain_desc, 0, sizeof(struct dma_desc)*number_of_desc);

  for (i = 0; i < number_of_desc-1; i++) { 
    offset = i*boardData->half_FIFO_size;
    boardData->chain_desc[i].pci_addr = (u32)get_dmabuf_pciaddr(boardData, offset);

/********
    if (boardData->chain_desc[i].pci_addr + boardData->half_FIFO_size-1 != 
	(u32)get_dmabuf_pciaddr(boardData,(i+1)*boardData->half_FIFO_size-1)) {
      printk("Bad PCI address\n");
      return -EFAULT;
    }
*************/
    boardData->chain_desc[i].loc_addr  = 0x00003000L + 0x200;
    boardData->chain_desc[i].size      = boardData->half_FIFO_size;  /* size is in bytes and each sample is two bytes */
    boardData->chain_desc[i].next_desc_and_flags  = (u32)boardData->chain_desc_bus_addr + (i+1)*sizeof(struct dma_desc);
    boardData->chain_desc[i].next_desc_and_flags |= (PLX_DMA_DESC_IS_PCI | PLX_DMA_DESC_TO_HOST);
#ifdef DEBUG
    printk( "%d - pci_addr = %#x, loc_addr = %#x, size = %#x, next_desc_and_flags = %#x \n",i,
	    boardData->chain_desc[i].pci_addr, boardData->chain_desc[i].loc_addr,
	    boardData->chain_desc[i].size,     boardData->chain_desc[i].next_desc_and_flags);
#endif
  }
  offset = i*boardData->half_FIFO_size;
  boardData->chain_desc[i].pci_addr=(u32)get_dmabuf_pciaddr(boardData, offset);
/********
    if (boardData->chain_desc[i].pci_addr + boardData->half_FIFO_size-1 != 
	(u32)get_dmabuf_pciaddr(boardData,(i+1)*boardData->half_FIFO_size-1)) {
      printk("Bad PCI address\n");
      return -EFAULT;
    }
*************/
  boardData->chain_desc[i].loc_addr = 0x00003000L + 0x200;
  boardData->chain_desc[i].size     = (2*chanRec->count - i*boardData->half_FIFO_size);
  boardData->chain_desc[i].next_desc_and_flags  = (u32)boardData->chain_desc_bus_addr;
  boardData->chain_desc[i].next_desc_and_flags|=
                  (PLX_DMA_DESC_IS_PCI | PLX_DMA_DESC_TO_HOST|PLX_DMA_DESC_TC_IE | PLX_DMA_DESC_EOC);
  
#ifdef DEBUG
  printk( "%d - pci_addr = %#x, loc_addr = %#x, size = %#x, next_desc_and_flags = %#x \n",i,
	  boardData->chain_desc[i].pci_addr, boardData->chain_desc[i].loc_addr,
	  boardData->chain_desc[i].size,     boardData->chain_desc[i].next_desc_and_flags);
#endif

  /* 2 Clear off the DMA CSR */
  writeb(0x0, PLX_DMA1_CSR);		 	
   
  /* 3 Setting up the DMA Threshold register */
  writel(0x04000000, PLX_DMATHR);			

  /* 4. Setting up the DMA Mode reg for channel 1 */
  writel((   PLX_DMA_MODE_WIDTH32  	//32 -bit bus size 
	   | PLX_DMA_MODE_BURST  	// enable bursts, i.e. prevent one write cycle for every word 
	   | PLX_DMA_MODE_READY  	
	   | PLX_DMA_MODE_BTERM
	   | PLX_DMA_MODE_CHAIN  
	   | PLX_DMA_MODE_DONE_IE
	   | PLX_DMA_MODE_ADDR_HOLD
	   | PLX_DMA_MODE_DEMAND
	   | PLX_DMA_MODE_INTR_PCI),
	   PLX_DMA1_MODE);
	
  /* 5. Setting these to zero, since we will be using the Descriptor Pointer Register */
  writel(0, PLX_DMA1_PCI_ADDR); 
  writel(0, PLX_DMA1_LCL_ADDR);	
  writel(0, PLX_DMA1_SIZE);	

  /* 6. Setting the Descriptor Pointer Register */
  writel((PLX_DMA_DESC_IS_PCI | PLX_DMA_DESC_TO_HOST | (u32)boardData->chain_desc_bus_addr),
           PLX_DMA1_DESCRIPTOR_PTR);

#ifdef DEBUG
  printk("Wrote %#.8lx into the DMA1 Descriptor Pointer register\n",
      PLX_DMA_DESC_IS_PCI | PLX_DMA_DESC_TO_HOST | virt_to_bus((&boardData->chain_desc[0])));
#endif

  /* 7. Enabling PCI interrups */
  plx_intcsr = readl(PLX_INTCSR);
  plx_intcsr |= PLX_PCI_IE | PLX_PCI_LOCAL_IE | PLX_LCL_DMA1_IE | PLX_LCL_IE;
  writel(plx_intcsr, PLX_INTCSR);

#ifdef DEBUG
  printk("Wrote %#.8x into the PLX INTCSR register\n", plx_intcsr);
#endif

  /* 8. Enabling the DMA */
  writeb(PLX_DMA_EN, PLX_DMA1_CSR);		 	
  writeb(PLX_DMA_START | PLX_DMA_EN, PLX_DMA1_CSR); 	// Enabling and Starting
	
#ifdef DEBUG
  printk("DMA Channel 1 control and status reg after enabling: %#x \n", readb(PLX_DMA1_CSR));
#endif

  /* Enable Data Acquisition */
  boardData->DAQ_ControlReg0 |= DAQ_ENB;
  writew(boardData->DAQ_ControlReg0, CTRL_REG_0);
	
#ifdef DEBUG
  printk("ControlReg0 = %#x \n",boardData->DAQ_ControlReg0 );
#endif

  udelay(1000);

  /* Enabling the Software Gate */ 
  boardData->DAQ_ControlReg1 |= CR1_SFT_GATE;
  writew(boardData->DAQ_ControlReg1, CTRL_REG_1);
  
#ifdef DEBUG
  printk("ControlReg1 = %#x \n",boardData->DAQ_ControlReg1);
#endif

  /* Issue soft start command */
  writew(0x0, DAQ_START_REG);   /* Force first conversion */

  if (chanRec->nonBlockFlag) {  /* do this one time only */
    boardData->nonBlockFile = readfile;
    return -EAGAIN;
  } else {
    wait_event_interruptible(das4020_12_wait,(boardData->DMAComplete==1) != 0);
    free_chain(boardData);
  }

#ifdef DEBUG
  printk("ADC Read Pointer register = %#x\n",  readw(ADC_READ_REG));
  printk("ADC Write Pointer register = %#x\n",  readw(ADC_WRITE_REG));
#endif

  return(0);
}

int free_chain( BoardRec *boardData )
{
  u8 *base2 = boardData->base2;

  boardData->DMAComplete = 0;
  
  // Disabling the Gate
  boardData->DAQ_ControlReg1 &= ~CR1_SFT_GATE; 
  writew(boardData->DAQ_ControlReg1, CTRL_REG_1);

  /* free up chain */
  pci_free_consistent(boardData->pdev,
		      boardData->chain_desc_size,
		      boardData->chain_desc,
		      boardData->chain_desc_bus_addr);
  return(0);
}
