/*
 * Copyright (C) 2007   Warren Jasper
 * All rights reserved.
 *
 */

/*
 * pci-das08.h
 */

#include <linux/ioctl.h>
#define IOCTL_MAGIC 'w'

/*  ioctl() values */

#define ADC_COUNTER0        _IO(IOCTL_MAGIC,  1)
#define ADC_COUNTER1        _IO(IOCTL_MAGIC,  2)
#define ADC_COUNTER2        _IO(IOCTL_MAGIC,  3)
#define ADC_GET_STATUS      _IO(IOCTL_MAGIC,  4)
#define ADC_GET_DIO         _IOR(IOCTL_MAGIC, 5, long)
#define ADC_SET_DIO         _IO(IOCTL_MAGIC,  6)
#define INT_SELECT          _IO(IOCTL_MAGIC,  7)
#define INT_ENABLE          _IO(IOCTL_MAGIC,  8)
#define SW_INTERRUPT        _IO(IOCTL_MAGIC,  9)

#define IOCTL_MAXNR 9         /* maxinum ordinal number */

/* open mode values */
#define ADC_SOFT_TRIGGER         (0x0)
#define ADC_EXTERNAL_TRIGGER     (0x2)

/* Digital I/O Modes */
#define MODE_IO            0
#define MODE_STROBE_IO     1
#define MODE_BIDIRECTIONAL 2

/* ioctl(COUNTER0) mode values */
#define CTR0_MODE0 (0x0 << 17)
#define CTR0_MODE1 (0x1 << 17)
#define CTR0_MODE2 (0x2 << 17)
#define CTR0_MODE3 (0x3 << 17)
#define CTR0_MODE4 (0x4 << 17)
#define CTR0_MODE5 (0x5 << 17)
#define CTR0_NOLOAD (0xff000000)

/* digital output */
#define DIO_0 0x1
#define DIO_1 0x2
#define DIO_2 0x4
#define DIO_3 0x8

#define INTERRUPT_PIN24   0  // Select interrupt from connector pin 24 
#define COUNTER2_OUTPUT   1  // Select interrupt from counter 2 Output
#define INTERRUPT_ENABLE  1  // Enable interrupts
#define INTERRUPT_DISABLE 0  // Disable interrupts






