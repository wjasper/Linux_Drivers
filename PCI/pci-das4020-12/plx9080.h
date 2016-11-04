/***************************************************************************
 Copyright (C) 2002  Warren Jasper
 All rights reserved.

 This program, PCI-DAS4020/12, is free software; you can redistribute it
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
 * References:
 *
 *  PLX 9080 PCI Data Book version 1.06
 *      http://www.plxtech.com/download/9080/databook/9080db-106.pdf
 */

#ifndef PLX9080_H
#define PLX9080_H

/* BADR0: PLX 9080 Register Locations */

#define	PLX_CNTRL	        (base0+0x6c)  // EEPROM,PCI,User,Init Control/Status

/* Bit Definitions for PLX Control Register */
#define PLX_USEROUT		0x00010000
#define PLX_USERIN		0x00020000
#define PLX_EECK		0x01000000
#define PLX_EECS		0x02000000
#define PLX_EEWD		0x04000000
#define PLX_EERD		0x08000000
#define PLX_EEPRESENT		0x10000000
#define PLX_RELOAD_CONFIG	0x20000000
#define PLX_PCI_SW_RESET	0x40000000
#define PLX_LCL_INIT_STATUS	0x80000000

#define	PLX_INTCSR		(base0+0x68)	// Interrupt Control/Status

#define	PLX_LSERR_ENABLE	0x00000001
#define	PLX_LSERR_PE		0x00000002
#define	PLX_SERR		0x00000004
#define PLX_MBOX_IE 		0x00000008			
#define PLX_PCI_IE		0x00000100
#define	PLX_PCI_DOORBELL_IE	0x00000200
#define	PLX_PCI_ABORT_IE	0x00000400
#define	PLX_PCI_LOCAL_IE	0x00000800
#define	PLX_RETRY_ABORT_ENABLE	0x00001000
#define	PLX_PCI_DOORBELL_INT	0x00002000
#define	PLX_PCI_ABORT_INT	0x00004000
#define	PLX_PCI_LOCAL_INT	0x00008000
#define	PLX_LCL_IE		0x00010000
#define	PLX_LCL_DOORBELL_IE	0x00020000
#define	PLX_LCL_DMA0_IE		0x00040000
#define	PLX_LCL_DMA1_IE		0x00080000
#define	PLX_LCL_DOORBELL_INT	0x00100000
#define	PLX_LCL_DMA0_INT	0x00200000 	//This one indicates DMA channel 0 has pending interrupt. 
#define	PLX_LCL_DMA1_INT	0x00400000
#define	PLX_LCL_BIST_INT	0x00800000
#define	PLX_BM_DIRECT_		0x01000000
#define	PLX_BM_DMA0_		0x02000000
#define	PLX_BM_DMA1_		0x04000000
#define	PLX_BM_ABORT_		0x08000000
#define  PLX_MBOX0_INT 		0x10000000
#define	PLX_MBOX1_INT		0x20000000
#define  PLX_MBOX2_INT 		0x40000000
#define  PLX_MBOX3_INT 		0x80000000


#define PLX_DMATHR 		base0 + 0xB0	// DMA threshold register 

// DMA related registers for Channel 0

#define PLX_DMA0_MODE 		(base0+0x80)	//DMA mode register for channel 0
#define	PLX_DMA0_PCI_ADDR	(base0+0x84)	// DMA Channel 0 PCI Address for non-chanin' mode
#define	PLX_DMA0_LCL_ADDR	(base0+0x88)	// DMA Channel 0 LOcal Adress for non-chanin' mode
#define	PLX_DMA0_SIZE		(base0+0x8C)	// DMA Channel 0 Size register for non-chanin' mode
#define PLX_DMA0_DESCRIPTOR_PTR (base0+0x90)	// DMA Channel 0 Descriptor Pointer Register
#define	PLX_DMA0_CSR		(base0+0xa8)	// DMA Channel 0 Control and Status Register.


// DMA related registers for Channel 1

#define PLX_DMA1_MODE 		(base0+0x94)	//DMA mode register for channel 1
#define	PLX_DMA1_PCI_ADDR	(base0+0x98)	// DMA Channel 1 PCI Address for non-chanin' mode
#define	PLX_DMA1_LCL_ADDR	(base0+0x9c)	// DMA Channel 1 LOcal Adress for non-chanin' mode
#define	PLX_DMA1_SIZE		(base0+0xa0)	// DMA Channel 1 Size register for non-chanin' mode
#define PLX_DMA1_DESCRIPTOR_PTR (base0+0xa4)	// DMA Channel 1 Descriptor Pointer Register
#define	PLX_DMA1_CSR		(base0+0xa9)	// DMA Channel 0 Control and Status Register.

// Bit definitions for DMA Mode registers
#define PLX_DMA_MODE_WIDTH32	   0x00000003
#define PLX_DMA_MODE_WAITSTATES(x) (((x) & 0xf) << 2)	
#define PLX_DMA_MODE_READY	   0x00000040
#define PLX_DMA_MODE_BTERM	   0x00000080
#define PLX_DMA_MODE_BURST	   0x00000100
#define PLX_DMA_MODE_CHAIN	   0x00000200
#define PLX_DMA_MODE_DONE_IE	   0x00000400
#define PLX_DMA_MODE_ADDR_HOLD	   0x00000800
#define PLX_DMA_MODE_DEMAND	   0x00001000
#define PLX_DMA_MODE_WR_AND_INVL   0x00002000	// Write and invalidate
#define PLX_DMA_MODE_EOT_EN	   0x00004000
#define PLX_DMA_MODE_STOP	   0x00008000
#define PLX_DMA_MODE_CLR_CNT_EN	   0x00010000
#define PLX_DMA_MODE_INTR_PCI	   0x00020000	// Deliver DMA intr to PCI side
#define PLX_DMA_MODE_INTR_LOCAL	   0x00000000	// Deliver DMA intr to Local side

// Bit definitions for DMA CSR registers
#define PLX_DMA_EN		0x01		// Enable DMA on Channel [0/1].
#define PLX_DMA_START		0x02		// Start DMA on Channel [0/1].
#define PLX_DMA_ABORT		0x04		// writing one here causes ABORT of the transfer.
#define PLX_DMA_CLR		0x08		// Clear the interrupt for DMA channel [0/1].
#define PLX_DMA_DONE		0x10		// Indicates that channel[0/1]'s over.

// Bit definitions for the DMA Descriptor register

#define	PLX_DMA_DESC_IS_PCI	0x00000001	// desc is in PCI addr space
#define	PLX_DMA_DESC_IS_LCL	0x00000000 	// desc is in Local addr space
#define	PLX_DMA_DESC_EOC	0x00000002
#define	PLX_DMA_DESC_TC_IE	0x00000004
#define	PLX_DMA_DESC_TO_HOST	0x00000008
#define	PLX_DMA_DESC_TO_BOARD	0x00000000


struct dma_desc 
{
	u32 pci_addr;
	u32 loc_addr;
	u32 size;
	u32 next_desc_and_flags;
};

#endif

