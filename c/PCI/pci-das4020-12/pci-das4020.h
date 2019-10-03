/* pci-das4020.h */

#include <linux/ioctl.h>
#define IOCTL_MAGIC 'w'

/*  ioctl() values */

#include <linux/ioctl.h>
#define IOCTL_MAGIC 'w'

#define ADC_SET_GAINS          _IO(IOCTL_MAGIC,   1)
#define ADC_GET_GAINS          _IOR(IOCTL_MAGIC,  2, long)
#define ADC_SET_PACER_FREQ     _IO(IOCTL_MAGIC,   3)
#define ADC_GET_PACER_FREQ     _IOR(IOCTL_MAGIC,  4, long)
#define ADC_SET_TRIGGER        _IO(IOCTL_MAGIC,   5)
#define ADC_SET_GATE           _IO(IOCTL_MAGIC,   6)
#define ADC_SET_CHAN_LOW       _IO(IOCTL_MAGIC,   7)
#define ADC_SET_CHAN_HIGH      _IO(IOCTL_MAGIC,   8)
#define ADC_SET_ATRIG_LOW      _IO(IOCTL_MAGIC,   9)
#define ADC_SET_ATRIG_HIGH     _IO(IOCTL_MAGIC,  10)
#define ADC_SET_ATRIG_MODE     _IO(IOCTL_MAGIC,  11)
#define ADC_GET_HALF_FIFO_SIZE _IO(IOCTL_MAGIC,  12)
#define ADC_SET_FIFO_SIZE      _IO(IOCTL_MAGIC,  13) // argument is FIFO size (see docs)
#define ADC_SET_HW_CONFIG_REG  _IO(IOCTL_MAGIC,  14)
#define ADC_SET_CONTROL_REG0   _IO(IOCTL_MAGIC,  15)
#define ADC_SET_CONTROL_REG1   _IO(IOCTL_MAGIC,  16)
#define ADC_GET_DMA_BUF_SIZE   _IO(IOCTL_MAGIC,  17)
#define ADC_NBIO_CANCEL        _IO(IOCTL_MAGIC,  18)
#define ADC_NBIO_PENDING       _IOR(IOCTL_MAGIC, 19, long)
#define ADC_NBIO_COMPLETE      _IOR(IOCTL_MAGIC, 20, long)
#define ADC_PSC_ENB            _IO(IOCTL_MAGIC,  21)

#define DIO_SET_MODE           _IO(IOCTL_MAGIC,  22)
#define DIO_SET_DIRECTION      _IO(IOCTL_MAGIC,  23)
#define DAC_SET_GAINS          _IO(IOCTL_MAGIC,  24)
#define DAC_GET_GAINS          _IOR(IOCTL_MAGIC, 25, long)

#define ADC_SET_CLK_SRC        _IO(IOCTL_MAGIC,  26)
#define ADC_GET_CLK_SRC        _IOR(IOCTL_MAGIC, 27, long)

#define IOCTL_MAXNR 27         /* maxinum ordinal number */
     
/* open() mode values  */
#define ADC_SOFT_CONVERSION      (0x0)
#define ADC_DMA_CONVERSION       (0x2)

/*Digital I/O Modes */
#define MODE_IO            0
#define MODE_STROBE_IO     1
#define MODE_BIDIRECTIONAL 2

/* Digital I/O Direction Settings */
#define PORT_OUTPUT        0
#define PORT_INPUT         1
#define HIGH_PORT_INPUT    2
#define HIGH_PORT_OUTPUT   3
#define LOW_PORT_INPUT     4
#define LOW_PORT_OUTPUT    5

/* Programmable Range and Gain Settings                          */
/* These are magic values for the driver                         */
#define BP_10_00V   (0x0f)         // DAC ONLY

#define BP_5_00V    (0xf0)         // +/- 5V for all 4 channels
#define BP_1_00V    (0x00)         // +/- 1V for all 4 channels

#define	BP_CH0_5V	 0x00000010	// ch0 range: +/- 5V
#define	BP_CH0_1V	 0x00000000	// ch0 range: +/- 1V
#define	BP_CH1_5V	 0x00000020	// ch1 range: +/- 5V
#define	BP_CH1_1V	 0x00000000	// ch1 range: +/- 1V
#define	BP_CH2_5V	 0x00000040	// ch2 range: +/- 5V
#define	BP_CH2_1V	 0x00000000	// ch2 range: +/- 1V
#define	BP_CH3_5V	 0x00000080	// ch3 range: +/- 5V
#define	BP_CH3_1V	 0x00000000	// ch3 range: +/- 1V


// Note:  The following are values to be used with the ioctl call
//          ADC_SET_ATRIG_MODE
//          arg = (ATRIG_CH[0-3] | MODE)

/* Analog Trigger Channel Source */

#define ATRIG_CH0  (0x0)
#define ATRIG_CH1  (0x1 << 4)
#define ATRIG_CH2  (0x2 << 4)
#define ATRIG_CH3  (0x3 << 4)

/* Analog Trigger/Gate Modes */
#define INACTIVE            (0x0)
#define POSITIVE_HYSTERESIS (0x4)
#define NEGATIVE_HYSTERESIS (0x6)
#define NEGATIVE_SLOPE      (0x8)
#define POSITIVE_SLOPE      (0xa)
#define WINDOW              (0xc)

// Note:  The following are values to be used with the ioctl call
//          ADC_SET_TRIGGER

/* Trigger Modes */
#define DISABLE            (0x0)
#define SOFT_TRIGGER       (0x1)
#define EXTERNAL_TRIGGER   (0x2)
#define ANALOG_TRIGGER     (0x3)

// Note:  The following are values to be used with the ioctl call
//          ADC_SET_GATE


/* Gating modes */
#define SOFT_GATE          (0x1)
#define EXTERNAL_GATE      (0x2)
#define ANALOG_GATE        (0x3)
#define AGATE_ACTIVE_HI    (0x1 << 3)
#define AGATE_ACTIVE_LO    (0x0)
#define AGATE_EDGE         (0x0)
#define AGATE_LEVEL        (0x1 << 2)


// Note:  The following are values to be used with the ioctl call
//          ADC_SET_CLK_SRC and ADC_GET_CLK_SRC

#define	CLK_INTERNAL	     0x00000000	// 40MHz internal clock
#define	CLK_EXT_BNC	     0x00001000	// external BNC connector
#define	CLK_AD_START_TRIG_IN 0x00002000	// external A/D start trigger in pin
