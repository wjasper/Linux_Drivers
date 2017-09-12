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
 * ctr10.h
 */

#ifndef PCI_CTR10_H
#define PCI_CTR10_H

#include <linux/ioctl.h>
#define IOCTL_MAGIC 'w'

/* open modes */
#define CTR10_COUNTER    (0x2)
#define CTR10_FREQUENCY  (0x12)

/* ioctl */
#define LOAD_CMD_REG          _IO(IOCTL_MAGIC,   1)
#define LOAD_MASTER_MODE_REG  _IO(IOCTL_MAGIC,   2)
#define LOAD_ALARM_REG1       _IO(IOCTL_MAGIC,   3)
#define LOAD_ALARM_REG2       _IO(IOCTL_MAGIC,   4)
#define LOAD_MODE_REG         _IO(IOCTL_MAGIC,   5)
#define LOAD_LOAD_REG         _IO(IOCTL_MAGIC,   6)
#define LOAD_HOLD_REG         _IO(IOCTL_MAGIC,   7)
#define GET_MASTER_MODE_REG   _IOR(IOCTL_MAGIC,  8, long)
#define GET_COUNTER_MODE_REG  _IOR(IOCTL_MAGIC,  9, long)
#define GET_STATUS_REG        _IOR(IOCTL_MAGIC, 10, long)
#define SET_GATE_INTERVAL     _IO(IOCTL_MAGIC,  11)
#define SET_CLOCK_INPUT       _IO(IOCTL_MAGIC,  12)
#define SET_SQUARE_FREQ       _IO(IOCTL_MAGIC,  13)

#define IOCTL_MAXNR 13         /* maxinum ordinal number */


/****************************************************************
 *              Master Mode Register Bit Assignments            *
 ****************************************************************/

/* Time Of Day Mode */
#define TOD_DISABLED (0x0) /* TOD Disabled           */
#define TOD_DIV_5    (0x1) /* TOD Enabled / 5 Input  */
#define TOD_DIV_6    (0x2) /* TOD Enabled / 6 Input  */
#define TOD_DIV_10   (0x3) /* TOD Enabled / 10 Input */

/* Compare 1 Enable */
#define COMPARE1     (0x4) /* Compare 1 Enabled */

/* Compare 2 Enable */
#define COMPARE2     (0x8) /* Compare 2 Enabled */


/* FOUT Source */
#define MM_E1        (0x00) /* Terminal count of previous counter */
#define MM_SRC1      (0x10) /* Counter Input 1 */
#define MM_SRC2      (0x20) /* Counter Input 2 */
#define MM_SRC3      (0x30) /* Counter Input 3 */
#define MM_SRC4      (0x40) /* Counter Input 4 */
#define MM_SRC5      (0x50) /* Counter Input 5 */
#define MM_GATE1     (0x60) /* Gate Input 1 */
#define MM_GATE2     (0x70) /* Gate Input 2 */
#define MM_GATE3     (0x80) /* Gate Input 3 */
#define MM_GATE4     (0x90) /* Gate Input 4 */
#define MM_GATE5     (0xa0) /* Gate Input 5 */
#define MM_FREQ1     (0xb0) /* Internal frequency from oscillator */
#define MM_FREQ2     (0xc0) /* Internal frequency from oscillator */
#define MM_FREQ3     (0xd0) /* Internal frequency from oscillator */
#define MM_FREQ4     (0xe0) /* Internal frequency from oscillator */
#define MM_FREQ5     (0xf0) /* Internal frequency from oscillator */

/* FOUT Divider */
#define DIV_BY_16    (0x0)     /* Divide by 16 */
#define DIV_BY_1     (0x100)   /* Divide by 1 */
#define DIV_BY_2     (0x200)   /* Divide by 2 */
#define DIV_BY_3     (0x300)   /* Divide by 3 */
#define DIV_BY_4     (0x400)   /* Divide by 4 */
#define DIV_BY_5     (0x500)   /* Divide by 5 */
#define DIV_BY_6     (0x600)   /* Divide by 6 */
#define DIV_BY_7     (0x700)   /* Divide by 7 */
#define DIV_BY_8     (0x800)   /* Divide by 8 */
#define DIV_BY_9     (0x900)   /* Divide by 9 */
#define DIV_BY_10    (0xa00)   /* Divide by 10 */
#define DIV_BY_11    (0xb00)   /* Divide by 11 */
#define DIV_BY_12    (0xc00)   /* Divide by 12 */
#define DIV_BY_13    (0xd00)   /* Divide by 13 */
#define DIV_BY_15    (0xe00)   /* Divide by 14 */

/* Data Pointer Control */
#define DISABLE_INCREMENT (0x4000)  /* Disable Increment */

/* Scalar Control */
#define BCD_DIV           (0x8000)  /* BCD Division */

/****************************************************************
 *              Counter Mode Register Bit Assignments           *
 ****************************************************************/

/* Ouput control */
#define INACTIVE_LOW   (0x0)  /* Inactive, Output Low */
#define ACTIVE_HIGH    (0x1)  /* Active High Terminal Count Pulse */
#define TC_Toggled     (0x2)  /* TC Toggled */
#define INACTIVE_HIGH  (0x4)  /* Inactive, Output High Impedance */

/* Count Direction */
#define COUNTDOWN      (0x0)  /* Count Down */
#define COUNTUP        (0x8)  /* Count Up */

/* BCD Mode */
#define BINARY         (0x0)  /* Binary Count */
#define BCD            (0x10) /* BCD Count */

/* Recycle Mode */
#define ONETIME        (0x0)  /* Count Once */
#define RECYCLE        (0x20) /* Count Repetitively */

/* RELOAD */
#define LOADREG        (0x0)   /* Reload from Load */
#define LOADANDHOLDREG (0x40)  /* Reload from Load or Hold except in 
                                  Mode X which reloads only from Load */
/* Special Gate */
#define SPECIALGATEOFF (0x0)   /* Disable Special Gate */
#define SPECIALGATE    (0x80)  /* Enable Special Gate */

/* Count Source Selection */
#define TCN1           (0x000) /* Terminal count of previous counter */
#define SRC1           (0x100) /* Counter Input 1 */
#define SRC2           (0x200) /* Counter Input 2 */
#define SRC3           (0x300) /* Counter Input 3 */
#define SRC4           (0x400) /* Counter Input 4 */
#define SRC5           (0x500) /* Counter Input 5 */
#define GATE1          (0x600) /* Gate Input 1 */
#define GATE2          (0x700) /* Gate Input 2 */
#define GATE3          (0x800) /* Gate Input 3 */
#define GATE4          (0x900) /* Gate Input 4 */
#define GATE5          (0xa00) /* Gate Input 5 */
#define FREQ1          (0xb00) /* Internal frequency from oscillator */
#define FREQ2          (0xc00) /* Internal frequency from oscillator */
#define FREQ3          (0xd00) /* Internal frequency from oscillator */
#define FREQ4          (0xe00) /* Internal frequency from oscillator */
#define FREQ5          (0xf00) /* Internal frequency from oscillator */

/* Source Edge */
#define POSITIVEEDGE   (0x0)    /* Count on Rising Edge */
#define NEGATIVEEDGE   (0x1000) /* Count on Falling Edge */

/* Gating Control */
#define NOGATE         (0x0000) /* No Gating                  */
#define AHLTCPREVCTR   (0x2000) /* Active High TCN-1          */
#define AHLNEXTGATE    (0x4000) /* Active High Level GATE N+1 */
#define AHLPREVGATE    (0x6000) /* Active High Level GATE N-1 */
#define AHLGATE        (0x8000) /* Active High Level GATE N   */
#define ALLGATE        (0xa000) /* Active Low Level GATE N    */
#define AHEGATE        (0xc000) /* Active High Edge GATE N    */
#define ALEGATE        (0xe000) /* Active Low Edge GATE N     */

/* Register Names */
#define LOAD_REG1 0x9
#define LOAD_REG2 0xa 
#define LOAD_REG3 0xb
#define LOAD_REG4 0xc
#define LOAD_REG5 0xd

#define HOLD_REG1 0x11
#define HOLD_REG2 0x12
#define HOLD_REG3 0x13
#define HOLD_REG4 0x14
#define HOLD_REG5 0x15

#define ALARM_REG1 0x07
#define ALARM_REG2 0x0f
#define STATUS_REG 0x1f

#define MASTER_MODE_REG  0x17

/* 9513 counter command codes */
#define ARM           0x20
#define LOAD          0x40
#define LOAD_ARM      0x60
#define DISARM_LATCH  0x80
#define CTR_LATCH     0xa0
#define DISARM        0xc0
#define SET_OUTPUT    0xe8
#define CLEAR_OUTPUT  0xe0
#define MASTER_RESET  0xff

/* Clock inputs to 9513 Counter/Timers */
#define C_5MHZ       0x12
#define C_1MHZ       0x16
#define C_3_33MHZ    0x32
#define C_1_67MHZ    0x36

#endif
