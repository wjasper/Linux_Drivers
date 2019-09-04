/*
 *
 *  Copyright (c) 2017 Warren J. Jasper <wjasper@ncsu.edu>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-1808.h"
#include "usb-1808.rbf"

/* Commands and USB Report ID for USB 1808  */
/* Digital I/O Commands */
#define DTRISTATE         (0x00) // Read/Write digital port tristate register
#define DPORT             (0x01) // Read digital port pins
#define DLATCH            (0x02) // Read/Write Digital port output latch register

/* Analog Input Commands */
#define AIN               (0x10) // Asynchronously read analog input channel
#define ADC_SETUP         (0x11) // Read/Write setup registers on the ADC
#define AIN_SCAN_START    (0x12) // Start input scan
#define AIN_SCAN_STOP     (0x13) // Stop input scan
#define AIN_CONFIG        (0x14) // Read/Write input scan queue
#define AIN_CLR_FIFO      (0x15) // Clear data in the input FIFO
#define AIN_BULK_FLUSH    (0x16) // Flush the input Bulk pipe

/* Analog Output Commands  */
#define AOUT              (0x18) // Asynchronously write analog output channel
#define AOUT_SCAN_CONFIG  (0x19) // Read/write output scan queue 
#define AOUT_SCAN_START   (0x1A) // Start analog ouput scan
#define AOUT_SCAN_STOP    (0x1B) // Stop analog output scan
#define AOUT_CLEAR_FIFO   (0x1C) // Clear data in analog output FIFO

/* Counter Commands (Encoders are Counters 2 & 3) */
#define COUNTER           (0x20) // Read/reset event counter
#define COUNTER_OPTIONS   (0x21) // Read/set the counter's options
#define COUNTER_LIMITS    (0x22) // Read/set the counter's range limits
#define COUNTER_MODE      (0x23) // Read/set the counter's mode
#define COUNTER_PARAM     (0x24) // Read/set the counter's mode and options

/* Timer Commands */
#define TIMER_CONTROL     (0x28) // Read/write timer control register
#define TIMER_PARAMETERS  (0x2D) // Read/write timer parameters

/* Memory Commands */
#define MEMORY            (0x30) // Read/Write EEPROM
#define MEM_ADDRESS       (0x31) // EEPROM read/write address value
#define MEM_WRITE_ENABLE  (0x32) // Enable writes to firmware area

/* Miscellaneous Commands */  
#define STATUS            (0x40) // Read device status
#define BLINK_LED         (0x41) // Causes LED to blink
#define RESET             (0x42) // Reset the device
#define TRIGGER_CONFIG    (0x43) // External trigger configuration
#define PATTERN_DETECT    (0x44) // Pattern Detection trigger configuration
#define SERIAL            (0x48) // Read/Write USB Serial Number

/* FPGA Configuration Commands */
#define FPGA_CONFIG       (0x50) // Start FPGA configuration
#define FPGA_DATA         (0x51) // Write FPGA configuration data
#define FPGA_VERSION      (0x52) // Read FPGA version

#define HS_DELAY 2000

static int wMaxPacketSize = 0;         // will be the same for all devices of this type so
                                       // no need to be reentrant. 

void usbBuildGainTableAI_USB1808(libusb_device_handle *udev, Calibration_AIN table[NCHAN_1808][NGAINS_1808])
{
  /* Builds a lookup table of calibration coefficients to translate values into voltages:
         voltage = value*table[chan#][gain#].slope + table[chan#][gain#].offset
     Calibration constants are the same for single ended and differential modes.  
     Stored as IEEE-754 4-byte floating point values.
  */

  int i, j;
  uint16_t address = 0x7000;  // base address of ADC calibration coefficients.

  for (i = 0; i < NCHAN_1808; i++) {
    for (j = 0; j < NGAINS_1808; j++) {
      usbMemAddressW_USB1808(udev, address);
      usbMemoryR_USB1808(udev, (uint8_t *) &table[i][j].slope, sizeof(float));
      address += 4;
      usbMemAddressW_USB1808(udev, address);
      usbMemoryR_USB1808(udev, (uint8_t *) &table[i][j].offset, sizeof(float));
      address += 4;
    }
  }
  return;
}

void usbBuildGainTableAO_USB1808(libusb_device_handle *udev, Calibration_AOUT table_AO[NCHAN_AO_1808])
{
  /*
    Builds a lookup table of calibration coefficients to translate values into voltages:
      corrected value = value*table[chan#].slope + table[chan#].offset
    Stored as IEEE-754 4-byte floating point values.
  */

  int i;
  uint16_t address = 0x07100;  // base address of DAC calibration coefficients
  
  for (i = 0; i < NCHAN_AO_1808; i++) {
    usbMemAddressW_USB1808(udev, address);
    usbMemoryR_USB1808(udev, (uint8_t *) &table_AO[i].slope, sizeof(float));
    address += 4;
    usbMemAddressW_USB1808(udev, address);
    usbMemoryR_USB1808(udev, (uint8_t *) &table_AO[i].offset, sizeof(float));
    address += 4;
  }
  return;
}

int usbInit_1808(libusb_device_handle *udev)
{
  int i;

  /* This function does the following:
     1. Configure the FPGA
     2. Finds the maxPacketSize for bulk transfers
  */
  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
  if (wMaxPacketSize < 0) {
    perror("usbInit_1808: error in getting wMaxPacketSize");
  }

  if (!(usbStatus_USB1808(udev) & FPGA_CONFIGURED)) {
    usbFPGAConfig_USB1808(udev);
    if (usbStatus_USB1808(udev) & FPGA_CONFIG_MODE) {
      for (i = 0; i <= (sizeof(FPGA_data) - 64); i += 64) {
	usbFPGAData_USB1808(udev, &FPGA_data[i], 64);
      }
      if (sizeof(FPGA_data) % 64) {
	usbFPGAData_USB1808(udev, &FPGA_data[i], sizeof(FPGA_data)%64);
      }
      if (!(usbStatus_USB1808(udev) & FPGA_CONFIGURED)) {
	printf("Error: FPGA for the USB-1808 is not configured.  status = %#x\n", usbStatus_USB1808(udev));
	return -1;
      }
    } else {
      printf("Error: could not put USB-1808 into FPGA Config Mode.  status = %#x\n", usbStatus_USB1808(udev));
      return -1;
    }
  } else {
    printf("USB-1808 FPGA configured.\n");
  }
  return 0;
}

void usbCalDate_USB1808(libusb_device_handle *udev, struct tm *date)
{
  /* This command reads the factory calibration date */

  calibrationTimeStamp calDate;
  uint16_t address = 0x7110;  // beginning of MFG Calibration date
  time_t time;

  usbMemAddressW_USB1808(udev, address);
  usbMemoryR_USB1808(udev, (uint8_t *) &calDate, sizeof(calDate));
  date->tm_year = calDate.year + 100;
  date->tm_mon = calDate.month - 1;
  date->tm_mday = calDate.day;
  date->tm_hour = calDate.hour;
  date->tm_min = calDate.minute;
  date->tm_sec = calDate.second;
  time = mktime(date);
  date = localtime(&time);
}

/***********************************************
 *            Digital I/O                      *
 ***********************************************/

/* Read/Write digital port tristate register */

uint8_t usbDTristateR_USB1808(libusb_device_handle *udev)
{
  /* This command reads or writes the digital tristate register.
   The tristate register determines if the latch register value is driven onto
   the port pin.  A '1' in the tristate register makes the corresponding
   pin an input, a '0' makes it an output.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDTristateR_USB1808: error in libusb_control_transfer().");
  }
  return data;
}

void usbDTristateW_USB1808(libusb_device_handle *udev, uint8_t value)
{

  /* This command reads or writes the digital port tristate register.
     The tristate register determines if the latch register value is driven onto
     the port pin.  A '1' in the tristate register makes the corresponding
     pin an input, a '0' makes it an output.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbDTristateW_USB1808: error in libusb_control_transfer()");
  }
  return;
}

/* reads digital word  */
uint8_t usbDPort_USB1808(libusb_device_handle *udev)
{
  /*
    This command reads the current state of the digital pins. DIO_0 - DIO_3
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data;

  if (libusb_control_transfer(udev, requesttype, DPORT, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDPort_USB1808: error in libusb_control_transfer().");
  }
  return data;
}

/* read/writes digital latch */
uint8_t usbDLatchR_USB1808(libusb_device_handle *udev)
{
  /*
    This command reads the digital latch register
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DLATCH, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDLatchR_USB1808: error in libusb_control_transfer().");
  }
  return data;
}

void usbDLatchW_USB1808(libusb_device_handle *udev, uint8_t value)
{
  /*
    This command writes the digital port latch register
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DLATCH, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbDLatchW_USB1808: error in libusb_control_transfer().");
  }
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/

int usbAIn_USB1808(libusb_device_handle *udev, uint32_t value[8], Calibration_AIN table[NCHAN_1808][NGAINS_1808], ScanList list[NCHAN_1808])
{
  /*
    This command performs an asynchronous read of all analog input channels.
  */

  int ret;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int i;
  double data;

  ret = libusb_control_transfer(udev, requesttype, AIN, 0x0, 0x0, (unsigned char *) value, 8*sizeof(uint32_t), HS_DELAY);
  if (ret < 0) {
    perror("usbAIn_USB1808: error in libusb_control_transfer.");
  }

  for (i = 0; i < NCHAN_1808; i++) {
    if ((list[i].range == BP_10V) || (list[i].range == BP_5V)) {
      value[i] = rint(value[i]*table[i][list[i].range].slope + table[i][list[i].range].offset);
    } else {
      data = value[i]*table[i][list[i].range].slope + table[i][list[i].range].offset;
      if (data < 0) {
	value[i] = 0x0;
      } else if (data > 0x3ffff) {
	value[i] = 0x3ffff;
      } else {
	value[i] = rint(data);
      }
    }
  }
  return ret;
}

int usbADCSetupW_USB1808(libusb_device_handle *udev, ScanList list[8])
{
  /* 
     This command reads or writes the range configuration for all
     analog input channels (each channel can have its own unique
     range).  Each bye in this array corresponds to the analog input
     channel and the value determines that channel's range and input
     type.
     Bits 1-0: +/- 10V    = 0
               +/-  5V    = 1
               0 - 10V    = 2
               0 -  5V    = 3
     Bits 3-2: Differential = 0
               Single Ended = 1
               Grounded     = 3
     Bits 7-4: Reserved 
  */

  int i;
  int ret;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t config[8];

  for (i = 0; i < NCHAN_1808; i++) {
    config[i] = list[i].range | (list[i].mode << 0x2);
  }
  ret = libusb_control_transfer(udev, requesttype, ADC_SETUP, 0x0, 0x0, (unsigned char *) config, sizeof(config), HS_DELAY);
  if (ret < 0) {
    perror("usbADCSetupW_USB1808: error in libusb_control_transfer.");
  }
  return ret;
}

int usbADCSetupR_USB1808(libusb_device_handle *udev, ScanList list[8])
{
  int ret;
  int i;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t config[8];

  ret = libusb_control_transfer(udev, requesttype, ADC_SETUP, 0x0, 0x0, (unsigned char *) config, sizeof(config), HS_DELAY);
  if (ret < 0) {
    perror("usbADCSetupR_USB1808: error in libusb_control_transfer.");
  }

  for (i = 0; i < NCHAN_1808; i++) {
    list[i].range = config[i] & 0x3;
    list[i].mode =  (config[i] >> 2) & 0x3;
  }
  return ret;
}

int usbAInScanStart_USB1808(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options)
{
  /* This command starts the analog input channel scan.  The gain
     ranges that are currently set on the desired channels will be
     used (these may be changed with AInConfig) This command will
     result in a bus stall if an AInScan is currently running.

     Notes:

     The pacer rate is set by an internal 32-bit incrementing timer
     running at a base rate of 100 MHz.  The timer is controlled by
     pacer_period. A pulse will be output at the ICLKO pin at
     every pacer_period interval regardless of the mode.

     If pacer_period is set to 0, the device does not generate an A/D
     clock.  It uses the ICLKO pin as the pacer source.  

     The timer will be reset and sample acquired when its value equals
     timer_period.  The equation for calculating timer_period is:

     timer_period = [100MHz / (sample frequency)] - 1

     The data will be returned in packets utilizing a bulk IN endpoint.
     The data will be in the format:

     lowchannel sample 0: lowchannel + 1 sample 0: ... :hichannel sample 0
     lowchannel sample 1: lowchannel + 1 sample 1: ... :hichannel sample 1
     ...
     lowchannel sample n: lowchannel + 1 sample n: ... :hichannel sample n

     Important: Since the analog input data is 18 bits wide, each
     input sample will be 32 bits wide to account for this.  This
     includes the digital channels, and zeros will be padded for
     unused bits.  The scan will not begin until the InScanStart
     command is sent (and any trigger conditions are met.)  Data will
     be sent until reaching the specified count or an InScanStop
     command is sent.

     The packet_size parameter is used for low sampling rates to avoid
     delays in receiving the sampled data. The buffer will be sent,
     rather than waiting for the buffer to fill.  This mode should
     not be used for high sample rates in order to avoid data loss.

     The external trigger may be used to start data collection
     synchronously.  If the bit is set, the device will wait until the
     appropriate trigger edge is detected, then begin sampling data at
     the specified rate.  No messages will be sent until the trigger
     is detected.

     Pattern detection is used with the PatternDetectConfig command
     to set up a specified number of bits to watch, and then trigger
     when those bits reach the specified value.

     The retrigger mode option and the retrig_count parameter are only
     used if trigger is used.  This option will cause the trigger to
     be rearmed after retrig_count samples are acquired, with a total
     of count samples being returned from the entire scan.
  */

    struct AInScan_t {
    uint32_t count;        // The total number of scans to perform (0 for continuous scan)
    uint32_t retrig_count; // The numer of scans to perform for each trigger in retrigger mode.
    uint32_t pacer_period; // The pacer timer period value. (0 for external clock)
    uint8_t packet_size;   // Number of samples - 1 to transfer at a time.
    uint8_t options;    /* bit 0:  1 = use external trigger
                           bit 1:  1 = use Pattern Detection trigger
	                   bit 2:  1 = retrigger mode, 0 = normal trigger
                           bit 3:  1 = Maintain counter value on scan start,
                                   0 = Clear counter value on scan start
                           bit 4:  Reserved
			   bit 5:  Reserved
			   bit 6:  Reserved
			   bit 7:  Reserved
		        */
    uint8_t pad[2];
  } AInScan;

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  AInScan.count = count;
  AInScan.retrig_count = retrig_count;
  if (frequency == 0.0) {
    AInScan.pacer_period = 0; // use external pacer
  } else {
    AInScan.pacer_period = rint((100.E6 / frequency) - 1);
  }
  AInScan.options = options;

  if (options & SINGLE_IO) {
    AInScan.packet_size = 0x1;
  } else {
    AInScan.packet_size = 0xff;
  }

  /* Pack the data into 14 bytes */
  ret = libusb_control_transfer(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (unsigned char *) &AInScan, 14, HS_DELAY);
  if (ret < 0) {
    perror("usbAInScanStart_USB1808: Error");
  }
  return ret;
}

int usbAInScanRead_USB1808(libusb_device_handle *udev, int nScan, int nChan, uint32_t *data, unsigned int timeout, int options)
{
  char value[PACKET_SIZE];
  int ret = -1;
  int nbytes = nChan*nScan*4;    // number of bytes to read;
  int transferred;
  uint8_t status;

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) data, nbytes, &transferred, timeout);

  if (ret < 0) {
    perror("usbAInScanRead_USB1808: error in libusb_bulk_transfer.");
  }
  if (transferred != nbytes) {
    fprintf(stderr, "usbAInScanRead_USB1808: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
    status = usbStatus_USB1808(udev);
    if ((status & AIN_SCAN_OVERRUN)) {
      fprintf(stderr, "usbAInScanRead: Analog In scan overrun.\n");
      usbAInScanStop_USB1808(udev);
      usbAInScanClearFIFO_USB1808(udev);
    }
    return ret;
  }

  if (options & CONTINUOUS) return transferred;

  status = usbStatus_USB1808(udev);
  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
  if ((nbytes%wMaxPacketSize) == 0 && !(status & AIN_SCAN_RUNNING)) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) value, 2, &ret, 100);
  }

  if ((status & AIN_SCAN_OVERRUN)) {
    printf("usbAInScanRead: Analog In scan overrun.\n");
    usbAInScanStop_USB1808(udev);
    usbAInScanClearFIFO_USB1808(udev);
  }

  return transferred;
}

int usbAInScanConfigW_USB1808(libusb_device_handle *udev, uint8_t scanQueue[13], uint8_t lastChan)
{
  /*
    This command writes the input scan queue to the FPGA.  The max
    queue is 13 elements: 8 AIn, 2 Counters, 2 Encoders, 1 DIO.

    Each element of the scanQueue array corresponds to the element in
    the scan queue of the FPGA.  The data will determine which channel
    is read as follows:

    Analog Inputs:   0 - 7
    DIO:             8
    Counter 0:       9
    Counter 1:       10
    Encoder 0:       11
    Encoder 1:       12
  */

  int ret;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (lastChan > 12) {
    fprintf(stderr, "usbAInScanConfigW_USB1808: lastChan larger than 12.\n");
    return -1;
  }
  
  ret = libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, lastChan-1, (unsigned char *) scanQueue, 13, HS_DELAY);
  if (ret < 0) {
    perror("usbAInScanConfigW_USB1808: error in libusb_control_transfer");
  }
  return ret;
}

int usbAInScanConfigR_USB1808(libusb_device_handle *udev, uint8_t *scanQueue, uint8_t lastChan)
{
  int ret;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (lastChan > 12) {
    fprintf(stderr, "usbAInScanConfigR_USB1808: lastChan larger than 12.\n");
    return -1;
  }
  
  ret = libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, lastChan-1, (unsigned char *) scanQueue, 13, HS_DELAY);

  if (ret < 0) {
    perror("usbAInScanConfigR_USB1808: error in libusb_control_transfer");
  }
  return ret;
}

void usbAInScanStop_USB1808(libusb_device_handle *udev)
{
  /*
    This command stops the analog input scan (if running).
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAInScanClearFIFO_USB1808(libusb_device_handle *udev)
{
  /* This command clears the input firmware buffer */
  
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_CLR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAInBulkFlush_USB1808(libusb_device_handle *udev, uint8_t count)
{
  /* This command clears the input Bulk pipe a nubmer of times */
  
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_BULK_FLUSH, count, 0x0, NULL, 0x0, HS_DELAY);
}

/***********************************************
 *            Analog Output                    *
 ***********************************************/

void usbAOut_USB1808(libusb_device_handle *udev, uint8_t channel, double voltage, Calibration_AOUT table_AO[NCHAN_AO_1808])
{
  /*
    This command reads or writes the values for the analog output channels.
    The values are 16-bit unsigned numbers.  Both read and write will result
    in a control pipe stall if an output scan is running.  The equation for the
    output voltage is:

             ( value - 2^15 )
    V_out = -----------------   * V_ref
                 2^15

     were value is the value written to the channel and V_ref = 10.0V.
  */

  double dvalue;
  uint16_t value;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (channel > 2) {
    printf("usbAOut_USB1808: channel must be 0 or 1.\n");
    return;
  }

  /* correct voltage */
  dvalue = (voltage/10.*32768. + 32768.);
  dvalue = dvalue*table_AO[channel].slope + table_AO[channel].offset;

  if (dvalue > 0xffff) {
    value = 0xffff;
  } else if (dvalue < 0.0) {
    value = 0x0;
  } else {
    value = rint(dvalue);
  }
  libusb_control_transfer(udev, requesttype, AOUT, value, channel, NULL, 0x0, HS_DELAY);
}

int usbAOutScanConfigW_USB1808(libusb_device_handle *udev, uint8_t scanQueue[3], uint8_t lastChan)
{
  /*
    This command writes the output scan queue to the FPGA.  The max
    queue is 3 elements: 2 AOUT, 1 DIO.

    LastChan is the offset to the last channel that will be read in the queue.

    Each element of the scanQueue array corresponds to the element in
    the scan queue of the FPGA.  The data will determine which channel
    is read as follows:

    Analog Out 0 :   0
    Analog Out 1 :   1
    DIO:             2
  */

  int ret;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (lastChan > 3) {
    fprintf(stderr, "usbAOutScanConfigW_USB1808: lastChan larger than 3.\n");
    return -1;
  }
  
  ret = libusb_control_transfer(udev, requesttype, AOUT_SCAN_CONFIG, 0, lastChan-1, (unsigned char *) scanQueue, 3, HS_DELAY);
  if (ret < 0) {
    perror("usbAOutScanConfigW_USB1808: error in libusb_control_transfer");
  }
  return ret;
}

int usbAOutScanConfigR_USB1808(libusb_device_handle *udev, uint8_t *scanQueue, uint8_t lastChan)
{
  int ret;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (lastChan > 13) {
    fprintf(stderr, "usbAOutScanConfigR_USB1808: lastChan larger than 3.\n");
    return -1;
  }
  
  ret = libusb_control_transfer(udev, requesttype, AOUT_SCAN_CONFIG, 0x0, lastChan-1, (unsigned char *) scanQueue, 3, HS_DELAY);

  if (ret < 0) {
    perror("usbAInScanConfigR_USB1808: error in libusb_control_transfer");
  }
  return ret;
}

int usbAOutScanStart_USB1808(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options)
{
  /*
    This command starts the analog output channel scan.  This command
    will result in a bus stall if an AOutScan is currently running.

    count:        the total number of scans to perform (0 = continuous mode)
    retrig_count: the number of scans to perform for each trigger in
                  retrigger mode
    frequency:    pacer frequency (0 for OCLKI pin) .023 Hz < frequency < 125 kHz
    options:      bit 0: 1 = use external trigger
                  bit 1: 1 = use Pattern Detection trigger
                  bit 2: 1 = retrigger mode, 0 = normal trigger
                  bit 3: reserved
                  bit 4: reserved
                  bit 5: reserved
		  bit 6: reserved
		  bit 7: reserved
    Notes:
		  
    The output scan operates with the host continuously transferring
    data for the outputs until the end of the scan.  If the "count"
    parameter is 0, the scan will run until the OutScanStop command is
    issued by the host; if it is nonzero, the scan will stop
    automatically after the specified number of scans have been
    output.  The channels in the scan are selected in the options bit
    field.  Scans refers to the number of updates to the channels (if
    both channels are used, one scan is an update to both channels).

    The time base is controlled by an internal 32-bit timer running at
    a base rate of 100MHz.  The timer is controlled by pacer period.
    The equation for calculating the pacer period is:

        pacer_period = (100MHz / (frequency)) - 1

    The same time base is used for all channels when the scan involves
    multiple channels.  A pulse will be output at the OCLKO pin at
    every pacer_period internal regardless of mode.

    If pacer period is set to 0, the device does not generate a clock.
    It uses the OCLKI pin as the pacer source.

    The output data is to be sent using the bulk out endpoint.  The data must be in the format:

          low channel sample 0: [high channel sample 0]
          low channel sample 1: [high channel sample 1]
          .
          .
          .
          low channel sample n: [high channel sample n]

    The output data is written to an internal FIFO.  The bulk endpoint
    data is only accepted if there is room in the FIFO.  Output data
    may be sent to the FIFO before the start of the scan, and the FIFO
    is cleared when the OutScanClearFIFO command is received.  The scan
    will not begin until the OutScanStart command is sent (and output
    data is in the FIFO).  Data will be output until reaching the
    specified number of scans (in single execution mode) or an
    OutScanStop command is sent.
  */

  struct AOutScan_t {
    uint32_t count;         // The total number of scans to perform.  0 = run forever.
    uint32_t retrig_count;  // The number of scans to perform for each trigger in retrigger mode.
    uint32_t pacer_period;  // Pacer timer period value (0 for AO_CLK_IN)
    uint8_t options;
  } AOutScan;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (frequency < 0.23 || frequency > 125000.) {
    fprintf(stderr, "frequency = %f out of range.\n", frequency);
    return -1;
  }
  AOutScan.pacer_period = (100.E6 / frequency) - 1;
  AOutScan.count = count;
  AOutScan.retrig_count = retrig_count;
  AOutScan.options = options;
  
  ret = libusb_control_transfer(udev, requesttype, AOUT_SCAN_START, 0x0, 0x0, (unsigned char *) &AOutScan, sizeof(AOutScan), HS_DELAY);
  if (ret < 0) {
    perror("usbAOutScanStart_USB1808: error in libusb_control_transfer.");
  }
  return ret;
}

void usbAOutScanStop_USB1808(libusb_device_handle *udev)
{
  /* This command stops the analog output scan (if running). */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AOUT_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAOutScanClearFIFO_USB1808(libusb_device_handle *udev)
{
  /* This command clears any remaining output FIFO data after a scan */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AOUT_CLEAR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

/***********************************************
 *            Counter/Timer                    *
 ***********************************************/
int usbCounterW_USB1808(libusb_device_handle *udev, uint8_t counter, uint32_t count)
{
  /*
    This command reads or sets the value of the counters.  Counter 0
    and 1 are the event counters, while Counter 2 and 3 are Encoder 0
    and 1, respectively.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (counter > 3) {
    perror("usbCounterW_USB1808: counter must be less than 4.");
    return -1;
  }
  
  ret = libusb_control_transfer(udev, requesttype, COUNTER, 0x0, counter, (unsigned char *) &count, sizeof(count),  HS_DELAY);
  return ret;
}

int usbCounterR_USB1808(libusb_device_handle *udev, uint8_t counter, uint32_t *count)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (counter > 3) {
    perror("usbCounterR_USB1808: counter must be less than 4.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, COUNTER, 0x0, counter, (unsigned char *) count, sizeof(count),  HS_DELAY);
  return ret;
}

int usbCounterOptionsW_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t options)
{
  /* 
     This command reads or sets the options of a counter.
     counter: the counter to set (0-3)
     options: the options for this counter's mode and will differ depending on the counter type.
       Counter:
         bit 0: 1 = Clear on Read,  0 = Read has no effect
         bit 1: 1 = No recycle mode (counter stops at 2^32 or 0, unless Range Limit is enabled)
                0 = counter rolls over to minimum (or max) and continues counting.
         bit 2: 1 = Count down, 0 = Count up
         bit 3: 1 = Range Limit on (use max and min limits), 0 = 32-bit counter (max = 2^32, min = 0)
         bit 4: 1 = Count on the falling edge, 0 = Rising edge
         bits 5-8: Reserved

       Encoder:
         bit 0-1: Encoder Type:
                  0 = X1
                  1 = X2
                  2 = X4
         bit 2: Clear on Z: 1 = clear when Z goes high, 0 = do not clear when Z goes high
         bit 3: Latch on Z: 1 = Counter will be latched when Z goes high, 
                            0 = Counter will be latched when asynchronously read or on a pacer clock in a scan.
         bit 4: 1 = No recycle mode (counter stops at 2^32 or 0, unless Range Limit is enabled)
                0 = counter rolls over to minimum (or max) and continue counting.
         bit 5: 1 = Range Limit on (use max and min limits)
                0 = 32-bit counter (max = 2^32, 0 = min)
         bits 6-7: Reserved
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (counter > 3) {
    perror("usbCounterOptionsW_USB1808: counter must be less than 4.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, COUNTER_OPTIONS, options, counter, NULL, 0x0,  HS_DELAY);
  return ret;
}

int usbCounterOptionsR_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t *options)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (counter > 3) {
    perror("usbCounterOptionsR_USB1808: counter must be less than 4.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, COUNTER_OPTIONS, 0x0, counter, (unsigned char *) options, sizeof(options),  HS_DELAY);
  return ret;
}

int usbCounterLimitsW_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint32_t value)
{
  /* 
     This command reads or sets a counter's count limit values.
         counter: the counter to set (0-3)
         index:   the index of the value to set. 0 = Minimum Limit Value, 1 = Maximum Limit Value
         value:   when the counter reaches this value, rolls over or stops, depending on the options.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (counter > 3) {
    perror("usbCounterLimitsW_USB1808: counter must be less than 4.");
    return -1;
  }

  if (index > 1) {
    perror("usbCounterLimitsW_USB1808: index must be 0 or 1.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, COUNTER_LIMITS, index, counter, (unsigned char *) &value, sizeof(value),  HS_DELAY);
  return ret;
}

int usbCounterLimitsR_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint32_t *value)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (counter > 3) {
    perror("usbCounterLimitsR_USB1808: counter must be less than 4.");
    return -1;
  }

  if (index > 1) {
    perror("usbCounterLimitsR_USB1808: index must be 0 or 1.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, COUNTER_LIMITS, index, counter, (unsigned char *) value, sizeof(value),  HS_DELAY);
  return ret;
}

int usbCounterModeW_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t mode)
{
  /*
    This command reads or sets the mode of a counter.  This does not functions on the Encoder counters.
    
    counter: the counter to set (0-1)
    mode:    the mode to set the counter:
      Bits 0-1: Mode
               Totalize:   0
               Period:     1
               Pulsewidth: 2
               Timing:     3
      Bits 2-3: Resolution of Period mode
               0 = Period Mode x1
               1 = Period Mode x10
               2 = Period Mode x100
               3 = Period Mode x1000
      Bits 4-5: Tick size (fundamental unit of time for period, pulsewidth, and timing modes)
               0 = 20 ns
               1 = 200 ns
               2 = 2000 ns
               3 = 20000 ns
      Bits (6-7): Reserved
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (counter > 1) {
    perror("usbCounterModeW_USB1808: counter must be 0-1.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, COUNTER_MODE, mode, counter,  NULL, 0x0,  HS_DELAY);
  return ret;
}

int usbCounterModeR_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t *mode)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (counter > 1) {
    perror("usbCounterModeR_USB1808: counter must be 0-1.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, COUNTER_MODE, 0x0, counter, (unsigned char *) mode, sizeof(mode),  HS_DELAY);
  return ret;
}

int usbCounterParametersW_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t mode, uint8_t options)
{
  /* This command reads and sets the mode and options of a counter.  This only works on the Options on the Encoder counters. */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  uint8_t value[2];

  value[0] = mode;
  value[1] = options;

  if (counter > 3) {
    perror("usbCounterParametersW_USB1808: counter must be 0-3");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, COUNTER_PARAM, 0x0, counter, (unsigned char *) value, sizeof(value),  HS_DELAY);
  return ret;
}

int usbCounterParametersR_USB1808(libusb_device_handle *udev, uint8_t counter, uint8_t *mode, uint8_t *options)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  uint8_t value[2];

  if (counter > 3) {
    perror("usbCounterParametersR_USB1808: counter must be 0-3.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, COUNTER_PARAM, 0x0, counter, (unsigned char *) value, sizeof(mode),  HS_DELAY);

  *mode = value[0];
  *options = value[1];
  return ret;
}

int usbTimerControlW_USB1808(libusb_device_handle *udev, uint8_t timer, uint8_t control)
{
  /* 
    This command reads or write the timer control register.  Timer output is 0 or 5V.

    timer: the timer selected: (0-1)
    control: the new control register value:
      bit 0: 1 = enable timer, 0 = disable timer (This needs to be a 0 when bit 4 is a 1)
      bit 1: 1 = timer running, 0 = timer stopped.
      bit 2: 1 = timer inverted output (active low), 0 = timer normal output (active high (5V))
      bit 3: Reserved
      bit 4: 1 = Timer will begin output when the OTRIG pin has triggered, 0 = Normal timer operation
      bit 5: Reserved
      bit 6: 1 = When bit 4 is equal to 1, Timer will continue to output on every OTRIG it receives
             0 = When bit 4 is equal to 1, Timer will output only on the first OTRIG received.
      bit 7: Reserved
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (timer > 1) {
    perror("usbTimerControlW_USB1808: timer must be 0-1.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, TIMER_CONTROL, control, timer, NULL, 0x0,  HS_DELAY);
  return ret;
}

int usbTimerControlR_USB1808(libusb_device_handle *udev, uint8_t timer, uint8_t *control)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  if (timer > 1) {
    perror("usbTimerControlR_USB1808: timer must be 0-1.");
    return -1;
  }

  ret = libusb_control_transfer(udev, requesttype, TIMER_CONTROL, 0x0, timer, (unsigned char *) control, sizeof(control),  HS_DELAY);
  return ret;
}

int usbTimerParametersW_USB1808(libusb_device_handle *udev, uint8_t timer, double frequency, double dutyCycle, uint32_t count, uint32_t delay)
{
  /* 
    This command reads or writes all of a given timer's parameters in one call.

    The timer is based on a 100MHz input clock and has a 32-bit
    period register.  The frequency of the output is set to:

        frequency = 100MHz / (period + 1)

    Note that the value for pulseWidth should always be smaller than
    the value for the period register or you may get unexpected
    results.  This results in a minimum allowable value for period of
    1, which sets the maximm freqauency to 100MHz/2 (50MHz).

    The timer has a 32-bit pulse width register.  The width of the ouput pulse isset to 

        pulse width = (pulseWidth + 1) / 100MHz

    Noe tht the value for pulseWidth should always be smaller than
    the value for the period register or you may get unexpted
    results.

    The number of output pulses can be controlled with the Count
    register.  Setting this register to 0 will result in pulses being
    generated until the timer is disabled.  Setting it to a non-zero
    value will result in the specified number of pulses being
    generated then the output will go low until the timer is disabled.

    The Delay register is the amount of time to delay before starting
    the timer output after enabling the output.  The value specifies
    the number of 100MHz clock pulses to delay.  This value may not be
    written while the timer output is enabled.

  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  uint32_t value[4];

  if (timer > 1) {
    perror("usbTimerParametersW_USB1808: timer must be 0-1.");
    return -1;
  }

  value[0] = 100.E6/frequency - 1;     // period
  value[1] = value[0] * dutyCycle;     // pulseWidth
  value[2] = count;
  value[3] = delay;

  ret = libusb_control_transfer(udev, requesttype, TIMER_PARAMETERS, 0x0, timer, (unsigned char *) value, sizeof(value),  HS_DELAY);
  return ret;
}

int usbTimerParametersR_USB1808(libusb_device_handle *udev, uint8_t timer, uint32_t *period, uint32_t *pulseWidth, uint32_t *count, uint32_t *delay)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  uint32_t value[4];

  if (timer > 1) {
    perror("usbTimerParametersR_USB1808: timer must be 0-1.");
    return -1;
  }
  ret = libusb_control_transfer(udev, requesttype, TIMER_PARAMETERS, 0x0, timer, (unsigned char *) value, sizeof(value),  HS_DELAY);

  *period = value[0];
  *pulseWidth = value[1];
  *count = value[2];
  *delay = value[3];
  
  return ret;
}

/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/
  
uint16_t usbStatus_USB1808(libusb_device_handle *udev)
{
  /* This command retrieves the status of the device. */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t status = 0x0;

  libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY);
  return status;
}

void usbBlink_USB1808(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return;
}

void usbReset_USB1808(libusb_device_handle *udev)
{
  /* 
     The function causes the defice to perform a reset.  The device
     disconnects from the USB bus and resets its microcontroller.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, (unsigned char *) NULL, 0x0, HS_DELAY);
}

int usbTriggerConfigW_USB1808(libusb_device_handle *udev, uint8_t options)
{
  /* This function configures the Scan trigger. Once the trigger is
     received, the Scan will proceed as configured.  The "use trigger"
     option must be used in the ScanStart command to utilize this
     feature.

     options: 
       bit 0: trigger mode     (0 = level, 1 = edge)
       bit 1: trigger polarity (0 = low/falling, 1 = high/rising)
       bits 2-7: Reserved
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  ret = libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, (unsigned char *) &options, sizeof(options),  HS_DELAY);
  return ret;
}

int usbTriggerConfigR_USB1808(libusb_device_handle *udev, uint8_t *options)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  ret = libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, (unsigned char *) options, sizeof(options),  HS_DELAY);
  return ret;
}

int usbPatternDetectConfigW_USB1808(libusb_device_handle *udev, uint8_t value, uint8_t mask, uint8_t options)
{
  /*
    This function configures the Pattern Detection trigger. Once the
    trigger is received, the Scan will proceed as configured.  The
    "use Pattern Detection trigger " opiton must be used in the
    InScanStart command to utilize this feature.

    value: the pattern on which to trigger 
    mask: these bits will mask the inputs such that only bits set to 1 here will be compared to the pattern.
    options: bit field that controls various options
      bit 0: Reserved
      bits 1-2:  00 = Equal to pattern
                 01 = Not equal to pattern
                 10 = Greater than pattern's numeric value
                 11 = Less than pattern's numeric value
      bits 3-7:  Reserved

  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  uint8_t data[3];

  data[0] = value;
  data[1] = mask;
  data[2] = options;

  ret = libusb_control_transfer(udev, requesttype, PATTERN_DETECT, 0x0, 0x0, (unsigned char *) data, sizeof(data), HS_DELAY);
  return ret;
}

int usbPatternDetectConfigR_USB1808(libusb_device_handle *udev, uint8_t *value, uint8_t *mask, uint8_t *options)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  uint8_t data[3];

  ret = libusb_control_transfer(udev, requesttype, PATTERN_DETECT, 0x0, 0x0, (unsigned char *) data, sizeof(data), HS_DELAY);

  *value = data[0];
  *mask = data[1];
  *options = data[2];

  return ret;
}

void usbGetSerialNumber_USB1808(libusb_device_handle *udev, char serial[9])
{
  /*
    This commands reads the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal
    digits (i.e. "00000001"). The new serial number will be programmed
    but not used until hardware reset.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, SERIAL, 0x0, 0x0, (unsigned char *) serial, 8, HS_DELAY);
  serial[8] = '\0';
  return;
}


/***********************************************
 *            Memory Commands                  *
 ***********************************************/
void usbMemoryR_USB1808(libusb_device_handle *udev, uint8_t *data, uint16_t length)
{
  /*
    This command reads or writes data from the EEPROM memory.  The
    read will begin at the current address, which may be set with
    MemAddress.  The address will automatically increment during a
    read or write but stay within the range allowed for the EEPROM.
    The amount of data to be written or read is specified in wLength.

    The range from 0x0000 to 0x6FF7 is used for storing the
    microcontroller firmware and is write-protected during normal
    operation.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  ret = libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
  if (ret != length) {
    perror("usbMemoryR_USB1808: error in reading memory.");
  }
}

void usbMemoryW_USB1808(libusb_device_handle *udev, uint8_t *data, uint16_t length)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
}

void usbMemAddressR_USB1808(libusb_device_handle *udev, uint16_t *address)
{
  /*
    This command reads or writes the address used for memory accesses.
    The upper byte is used to denominate different memory areas.  The
    memory map for this device is:

       Address                            Description
    =============               ============================
    0x0000-0x6FF7               Microcontroller firmware (write protected)
    0x6FF8-0x6FF8               Serial Number
    0X7000-0X7FFF               User data (Calibration Coefficients)

    The firmware area is protected by a separate command so is not typically
    write-enabled.  The calibration area is unlocked by writing the value 0xAA55
    to address 0x8000.  The area will remain unlocked until the device is reset
    or a value other than 0xAA55 is written to address 0x8000.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) address, sizeof(address), HS_DELAY);
}

void usbMemAddressW_USB1808(libusb_device_handle *udev, uint16_t address)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY);
}

void usbMemWriteEnable_USB1808(libusb_device_handle *udev)
{
  /*
    This command enables writes to the EEPROM memory in the range
    0x0000-0x6FFF.  This command is only to be used when updating the
    microcontroller firmware.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t unlock_code = 0xad;
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &unlock_code, sizeof(unlock_code), HS_DELAY);
}


/***********************************************
 *          FPGA Commands                      *
 ***********************************************/

void usbFPGAConfig_USB1808(libusb_device_handle *udev)
{
  /*
    This command puts the device into FPGA configuration update mode,
    which allows downloading the configuration for the FPGA.  The
    unlock code must be correct as a further safely device.  If the
    device is not in FPGA config mode, then the FPGAData command will
    result in a control pipe stall.

    Use the Status command to determine if the FPGA needs to be
    configured.  If so, use this command to enter configuration mode.
    Open the .rbf file containing the FPGA configuration and stream
    the data to the device using FPGAData.  After the FPGA is
    configured, then the DAQ commands will work.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t unlock_code = 0xad;
  libusb_control_transfer(udev, requesttype, FPGA_CONFIG, 0x0, 0x0, (unsigned char*) &unlock_code, sizeof(unlock_code), HS_DELAY);
}

void usbFPGAData_USB1808(libusb_device_handle *udev, uint8_t *data, uint8_t length)
{
  /*
    This command writes the FPGA configuration data to the device.  This
    command is not accepted unless the device is in FPGA config mode.  The
    number of bytes to be written must be specified in wLength.

    data: max length is 64 bytes
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (length > 64) {
    fprintf(stderr, "usbFPGAData_USB1808: max length = 64 bytes\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, FPGA_DATA, 0x0, 0x0, (unsigned char*) data, length, HS_DELAY);
}

void usbFPGAVersion_USB1808(libusb_device_handle *udev, uint16_t *version)
{
  /*
    This command reads the FPGA version.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, FPGA_VERSION, 0x0, 0x0, (unsigned char *) version, sizeof(uint16_t), HS_DELAY);
}

void cleanup_USB1808( libusb_device_handle *udev )
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|6);
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT|2);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}
 
double volts_USB1808(const uint8_t gain, uint32_t value)
{
  double volt = 0.0;

  switch (gain) {
    case BP_10V:
      volt = (value - 131072.)*10./131072.;
      break;
    case BP_5V:
      volt = (value - 131072.)*5./131072.;
      break;
    case UP_10V:
      volt = value*10./262143.;
      break;
    case UP_5V:
      volt = value*5./262143.;
      break;
  }
  return volt;
}
