/***************************************************************************
 Copyright (C) 2010  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-QUAD04, is free software; you can redistribute it
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
 * ctr05.h
 */

#ifndef PCI_QUAD04_H
#define PCI_QUAD04_H

#include <linux/ioctl.h>
#define IOCTL_MAGIC 'w'

/* ioctl */
#define LOAD_CMD_REG     _IO(IOCTL_MAGIC,   1)
#define GET_CMD_REG      _IOR(IOCTL_MAGIC,  2, long)
#define LOAD_IRCR_REG    _IO(IOCTL_MAGIC,   3)        // Interrupt Routing and Control Register
#define GET_IRCR_REG     _IOR(IOCTL_MAGIC,  4, long)
#define LOAD_ISCR_REG    _IO(IOCTL_MAGIC,   5)        // Input Signal Control Register
#define GET_ISCR_REG     _IOR(IOCTL_MAGIC,  6, long)
#define LOAD_PIC_A_REG   _IO(IOCTL_MAGIC,   7)        // 8259 Programmable Interrupt Controller Part A
#define LOAD_PIC_B_REG   _IO(IOCTL_MAGIC,   8)        // 8259 Programmable Interrupt Controller Part B

#define IOCTL_MAXNR 8       /* maxinum ordinal number */

/*******************************
 *  FLAG register  (Read Only) * 
 *******************************/
// BAR1 + 0x1
#define BT_FLAG    0x1   // Borrow Toggle
#define CT_FLAG    0x2   // Carry Toggle
#define CPT_FLAG   0x4   // Compare Toggle
#define S_FLAG     0x8   // Sign Flag
#define E_FLAG     0x10  // Error Flag
#define U_D        0x20  // Up/Down
#define IDX        0x40  // Index

/***************************************************
 *  Reset and Load (RLD) Signal Decoders Register  *
 ***************************************************/
// BAR1 + 0x1
#define RLD_NOP  0x0
// RLD Reset Fields
#define RESET_BP 0x1       // Reset Byte Pointer (BP)
#define RESET_CNTR 0x2     // Reset Counter
#define RESET_FLAGS 0x4    // Reset Borrow Toggle (BT), Carry Toggle (CT), Compare Toggle (CPT),
                           // and Sign Flag (S)
#define RESET_E 0x6        // Reset Error (E) flag
// RLD Transfer Fields
#define TRAN_PR_CNTR 0x8   // Transfer contents of Preset to Counter Register
#define TRAN_CNTR_OL 0x10  // Transfer contents of Counter to Output Latch Register
#define TRAN_PR_PSC 0x18   // Transfer contents of Preset to Filter Clock Prescalar Register

/********************************
 * Counter Mode Register (CMR)  *
 ********************************/
// Data Encoding
#define BIN    0x20       // Quadrature count in Binary
#define BCD    0x21       // Quadrature count in BCD
// Counting Modes
#define NORMAL 0x20       // Normal Count Mode
#define RANGE  0x22       // Range Limit Mode
#define NRCYC  0x24       // Non-Recycle Mode
#define MOD_N  0x26       // Modulo-N Mode
// Quadrature Scaling
#define X1 0x28
#define X2 0x30
#define X4 0x38

/***************************************
 * Input/Output Control Register (IOR) *
 ***************************************/
// Input A/B Configuration
#define DISABLE_AB  0x40  // Disable A and B inputs
#define ENABLE_AB   0x41  // Enable A and B inptus

// LCNTR/LOL pin configuration
#define LCNTR       0x40  // LCNTR/LOL is CNTR input
#define LOL         0x42  // LCNTR/LOL is Load OL input

// RCNTR/ABG pin configuration
#define RCNTR       0x40  // RCNTR/ABG is CNTR input
#define AB_GATE     0x44  // RCNTR/ABG is A/B gate

// FLAG 1 and 2 configuration
#define FLG_CB      0x40  // FLAG1 Carry, FLAG2 Borrow
#define FLG_CMB     0x48  // FLAG1 Compare, FLAG2 Borrow
#define FLG_CB_UPDN 0x50  // FLAG1 Carry/Borrow, FLAG2 U/D
#define FLG_IDX_E   0x58  // FLAG1 Index, FLAG2 Error

/********************************
 * Index Control Register (IDR) *
 ********************************/
// Enable/Disable Index
#define DISABLE_INDEX 0x60  // Disable Index
#define ENABLE_INDEX 0x61   // Enable Index
// Index Polarity Select
#define NEG_INDEX 0x60     // Negative Index Polarity
#define POS_INDEX 0x62     // Positive Index Polarity
// Index Pin Select
#define LCNTRL_INDEX 0x60  // Select LCNTRL/LOL pin as encoder index output.
#define RCNTR_INDEX 0x64   // Select RCNTR/ABG pin as selected for index.

/***************************************************
 * Index and Interrupt Routing Control (INDEX_REG) *
 ***************************************************/
// Interrupt Routing
#define CBINT1 0x10    // Channel 1  0 = FLG1 Carry/Compare/Index  1 = FLG2 Borrow/Up/Down
#define CBINT2 0x20    // Channel 2  0 = FLG1 Carry/Compare/Index  1 = FLG2 Borrow/Up/Down
#define CBINT3 0x40    // Channel 3  0 = FLG1 Carry/Compare/Index  1 = FLG2 Borrow/Up/Down
#define CBINT4 0x80    // Channel 4  0 = FLG1 Carry/Compare/Index  1 = FLG2 Borrow/Up/Down

// Index Routing 
#define INDXSEL1 0x01  // Channel 1 0 = RCNTR/ABG  1 = LCNTR/LOL
#define INDXSEL2 0x02  // Channel 2 0 = RCNTR/ABG  1 = LCNTR/LOL
#define INDXSEL3 0x04  // Channel 3 0 = RCNTR/ABG  1 = LCNTR/LOL
#define INDXSEL4 0x08  // Channel 4 0 = RCNTR/ABG  1 = LCNTR/LOL


#endif
