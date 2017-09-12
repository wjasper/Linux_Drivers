/*
 *
 *  Copyright (c) 2014 Warren J. Jasper <wjasper@ncsu.edu>
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
#include <stdint.h>

#include "pmd.h"
#include "usb-1208FS-Plus.h"

/* Commands and USB Report ID for USB 1208FS-Plus and USB 1408FS-Plus */
/* Digital I/O Commands */
#define DTRISTATE       (0x00) // Read/Write Tristate register
#define DPORT           (0x01) // Read digital port pins
#define DLATCH          (0x02) // Read/Write Digital port output latch register

/* Analog Input Commands */
#define AIN             (0x10) // Read analog input channel
#define AIN_SCAN_START  (0x11) // Start analog input scan
#define AIN_SCAN_STOP   (0x12) // Stop analog input scan
#define AIN_CONFIG      (0x14) // Analog input channel configuration
#define AIN_CLR_FIFO    (0x15) // Clear the analog input scan FIFO
#define AIN_BULK_FLUSH  (0x16) // Flush the bulk IN endpoint with empty packets

/* Analog Output Commands */
#define AOUT            (0x18) // Read/Write analog output channel
#define AOUT_SCAN_START (0x1A) // Start analog output scan
#define AOUT_SCAN_STOP  (0x1B) // Stop analog output scan
#define AOUT_CLR_FIFO   (0x1C) // Clear the analog output scan FIFO

/* Counter/Timer Commands */
#define COUNTER         (0x20)  // Read/reset event counter

/* Memory Commands */
#define CAL_MEMORY      (0x30)  // Read/Write Calibration Memory
#define USER_MEMORY     (0x31)  // Read/Write User Memory
#define MBD_MEMORY      (0x32)  // Read/Write MBD Memory

/* Miscellaneous Commands */  
#define BLINK_LED       (0x41)  // Causes LED to blink
#define RESET           (0x42)  // Reset device
#define STATUS          (0x44)  // Read device status
#define SERIAL          (0x48)  // Read/Write serial number
#define DFU             (0x50)  // Enter device firmware upgrade mode

/* MBD */
#define MBD_COMMAND     (0x80) // Text-based MBD command / response
#define MBD_RAW         (0x81) // Raw MBD response

#define HS_DELAY 2000

static int wMaxPacketSize;    // will be the same for all devices of this type so
                              // no need to be reentrant.
static int nBits(uint8_t num)
{
  int count = 0;
  
  // counts the number of bits in a number
  while(num) {
    if ((num & 0x1) == 0x1) count++;
    num >>=  0x1;
  }
  return count;
}

void usbBuildGainTable_DE_USB1208FS_Plus(libusb_device_handle *udev, float table_DE[NGAINS_USB1208FS_PLUS][NCHAN_DE][2])
{
  /* Builds a lookup table of differential mode calibration coefficents to translate values into voltages:
     The calibration coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  int i, j, k;
  uint16_t address = 0x0;

  for (i = 0; i < NGAINS_USB1208FS_PLUS; i++ ) {
    for (j = 0; j < NCHAN_DE; j++) {
      for (k = 0; k < 2; k++) {
	usbReadCalMemory_USB1208FS_Plus(udev, address, 4, (uint8_t *) &table_DE[i][j][k]);
	address += 4;
      }
    }
  }

  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
  if (wMaxPacketSize < 0) {
    perror("usb1208FS-Plus: error in getting wMaxPacketSize");
  }
}

void usbBuildGainTable_SE_USB1208FS_Plus(libusb_device_handle *udev, float table_SE[NCHAN_SE][2])
{
  /* Builds a lookup table of differential mode calibration
     coefficents to translate values into voltages: The calibration
     coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  int i, j;
  uint16_t address = 0x100;

  for (i = 0; i < NCHAN_SE; i++) {
    for (j = 0; j < 2; j++) {
      usbReadCalMemory_USB1208FS_Plus(udev, address, 4, (uint8_t *) &table_SE[i][j]);
      address += 4;
    }
  }
}

void usbCalDate_USB1208FS_Plus(libusb_device_handle *udev, struct tm *date)
{
  /* This command reads the factory calibration date */

  calibrationTimeStamp calDate;
  uint16_t address = 0x200;  // beginning of MFG Calibration date
  time_t time;

  usbReadCalMemory_USB1208FS_Plus(udev, address, sizeof(calDate), (uint8_t *) &calDate);
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
/* reads tristate port regiser */
uint8_t usbDTristateR_USB1208FS_Plus(libusb_device_handle *udev, uint8_t port)
{
  /*
    This command reads or writes the digital port tristate register.
    The tristate register determines if the latch register value is
    driven onto the port pin.  A '1' in the tristate register makes
    the corresponding pin an input, a '0' makes it an output. The power
    on default is all input.  This board only supports setting the
    entire port as input or output.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, 0x0, port, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDTristateR_USB1208FS_Plus: error in libusb_control_transfer().\n");
  }
  return data;
}

void usbDTristateW_USB1208FS_Plus(libusb_device_handle *udev, uint8_t port, uint8_t value)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, value, port, NULL, 0x0, HS_DELAY) < 0) {
    printf("usbDTristateW_USB1208HS: error in libusb_control_transfer().\n");
  }
  return;
}

/* reads digital port  */
uint8_t usbDPort_USB1208FS_Plus(libusb_device_handle *udev, uint8_t port)
{
  /*
    This command reads the current state of the digital port pins
    or writes to the latch register.
   */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data;

  if (libusb_control_transfer(udev, requesttype, DPORT, 0x0, port, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDPort_USB1208FS_Plus: error in libusb_control_transfer().\n");
  }
  return data;
}

/* read digital port */
uint8_t usbDLatchR_USB1208FS_Plus(libusb_device_handle *udev, uint8_t port)
{
  /*
    This command reads or writes the digital port latch register.  The
    power on default is all 0.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;
  
  if (libusb_control_transfer(udev, requesttype, DLATCH, 0x0, port, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDLatchR_USB1208FS_Plus: error in libusb_control_transfer().\n");
  }
  return data;
}

void usbDLatchW_USB1208FS_Plus(libusb_device_handle *udev, uint8_t port, uint8_t value)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DLATCH, value, port, NULL, 0x0, HS_DELAY) < 0) {
    printf("usbDLatchW_USB1208FS_Plus: error in libusb_control_transfer().\n");
  }
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/
uint16_t usbAIn_USB1208FS_Plus(libusb_device_handle *udev, uint8_t channel, uint8_t mode, uint8_t range)
{
  /*
    This command reads the value of an anlog input channel.  This command will result
    in a bus stall if an AInScan is currently running.

    channel: the channel to read (0-7)
    mode:    the input mode (0 = single ended, 1 = differential)
    range:   the input range for the channel (0-7)
  */

  uint16_t value;
  uint16_t wValue;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  wValue = (mode << 0x8) | channel;
  libusb_control_transfer(udev, requesttype, AIN, wValue, range, (unsigned char *) &value, sizeof(value), HS_DELAY);
  return value;
}

void usbAInScanStart_USB1208FS_Plus(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, 
				    double frequency, uint8_t channels, uint8_t options)
{
  /*
     The command starts an analog input scan.  The command will result
     in a bus stall if an AInScan is currently running.  The
     USB-1208FS-Plus will not generate an internal pacer faster than
     100 kHz;

     The ADC is paced such that the pacer controls the ADC
     conversions.  The internal pacer rate is set by an internal
     32-bit timer running at a base rate of 60 MHz.  The timer is
     controlled by pacer_period.  This value is the period of the scan
     and the ADCs are clocked at this rate.  A pulse will be output at
     the SYNC pin at every pacer_period interval if SYNC is configred
     as an output.  The equation for calucating pacer_period is:

     pacer_period = [60 MHz / (A/D frequency)] - 1

     If pacer_period is set to 0, the device does not generate an A/D
     clock.  It uses the SYNC pin as an input and the user must
     provide the pacer sourece.  The A/Ds acquire data on every rising
     edge of SYNC; the manimum allowable input frequency is 52 kHz.

     The data will be returned in packets untilizing a bulk endpoint.
     The data will be in the format:

     lowchannel sample 0: lowchannel + 1 sample 0: ... : hichannel sample 0
     lowchannel sample 1: lowchannel + 1 sample 1: ... : hichannel sample 1
     ...
     lowchannel sample n: lowchannel + 1 sample n: ... : hichannel sample n

     The scan will not begin until the AInScanStart command is sent
     and any trigger conditions are met.  Data will be sent until
     reaching the specified count or an usbAInScanStop_USB1208FS_Plus()
     command is sent.

     The external trigger may be used to start the scan.  If enabled,
     the device will wait until the appropriate trigger condition is
     detected than begin sampling data at the specified rate.  No
     packets will be sent until the trigger is detected.  In retrigger
     mode, the trigger will be automatically rearmed and the scan will
     restart after retrig_count samples have been acquired.  The count
     parameter specifies the total number of samples to acquire and
     shold be greater than or equal to retrig_count.  Specifying 0 for
     count causes continuous retrigger scans.  The data is stll sent
     as a continuous stream during retrigger scan so the last data
     from a previous scan will not be transferred until the beginning
     of the next retrigger scan if it does no end on a packet
     boundary.

     In block transfer mode, the data is sent in 64-byte packets as
     soon as data is available from the A/D.  In immediate transfer
     mode, the data is sent after each scan, resulting in packets that
     are 1-8 samples (2-16 bytes) long.  This mode should only be used
     for low pacer rates, typically under 100 Hz, because overruns
     will occur if the rate is too high.

     Overruns are indicated by the device stalling the bulk endpoint
     during the scan.  The host may read the status to verify and must
     clear the stall condition before further scan can be performed.
  */

  struct AInScan_t {
    uint32_t count;         // The total number of samples to perform (0 for continuous scan)
    uint32_t retrig_count;  // The number of samples to acquire for each trigger in retrigger mode
    uint32_t pacer_period;  // The pacer timer period value (0 for external clock).
    uint8_t channels;       // bit field that selects the channels in the scan,
                            // Upper 4 bits ignored in differential mode
    uint8_t options;        /* bit field that controls scan options:
			        bit 0:   0 = block transfer mode, 1 = immediate transfer mode
                                bit 1:   0 = single ended mode,   1 = differential mode
	                        bit 2-4: Trigger setting:
                                   0: to trigger
                                   1: Edge / rising
                                   2: Edge / falling
                                   3: Level / high
                                   4: Level / low
			        bit 5:  0 = normal trigger mode, 1 = retrigger mode
			        bit 6:  Reserved
			        bit 7:  0 = stall on overrun, 1 = inhibit stall
		             */
    uint8_t pad[2];         //  align along 12 byte boundary
  } AInScan;

  uint8_t nchan = nBits(channels);;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (frequency > 100000.) frequency = 100000.;
  if (frequency > 0.) {
    AInScan.pacer_period = rint((60.E6 / frequency) - 1);
  } else {
    AInScan.pacer_period = 0;
  }
  AInScan.count = count*nchan;  // total number of samples = scans * channels
  AInScan.retrig_count = retrig_count*nchan;
  AInScan.channels = channels;
  AInScan.options = options;

  usbAInScanStop_USB1208FS_Plus(udev);
  usbAInScanClearFIFO_USB1208FS_Plus(udev);
  /* Pack the data into 14 bytes */
  libusb_control_transfer(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (unsigned char *) &AInScan, 14, HS_DELAY);
}

int usbAInScanRead_USB1208FS_Plus(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data, uint8_t options, int timeout)
{
  /* Bulk transfers from endpoint 1 IN are used for receiving analog
     input scan data.  The bulk IN max packet size is 64 bytes.  Bulk
     transactions will be considered complete when a packet is sent
     that is less than the max packet size.  If an integer multiple of
     the max packet size of data is to be sent during a transaction,
     an empty packet will be sent to indicate the end of the
     transaction. The bulk IN endpoint will stall if an overrun is
     detected during the scan.  The host must issue a
     ClearFeature(HALT) USB command in order to clar the stall.
  */

  int i;
  int ret = -1;
  int nbytes = nChan*nScan*2;    // number of bytes to read in 64 bit chunks
  int transferred;
  uint16_t status;
  unsigned char value[MAX_PACKET_SIZE];

  if (options & IMMEDIATE_TRANSFER_MODE) {
    for (i = 0; i < nbytes/2; i++) {
      ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) &data[i], 2, &transferred, timeout);
      if (ret < 0) {
	perror("usbAInScanRead_USB1208FS_Plus: error in usb_bulk_transfer.");
      }
      if (transferred != 2) {
	fprintf(stderr, "usbAInScanRead_USB1208FS_Plus: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
      }
    }
  } else { 
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) data, nbytes, &transferred, timeout);
    if (ret < 0) {
      perror("usbAInScanRead_USB1208FS_Plus: error in usb_bulk_transfer.");
    }
    if (transferred != nbytes) {
      fprintf(stderr, "usbAInScanRead_USB1208FS_Plus: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
      status = usbStatus_USB1208FS_Plus(udev);
      if ((status & AIN_SCAN_OVERRUN)) {
        fprintf(stderr, "Analog AIn scan overrun.\n");
      }
      return ret;
    }      
  }

  if (options & CONTINUOUS) return nbytes;

  status = usbStatus_USB1208FS_Plus(udev);
  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
  if ((nbytes%wMaxPacketSize) == 0 && !(status & AIN_SCAN_RUNNING)) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) value, 2, &ret, 100);
  }

  if ((status & AIN_SCAN_OVERRUN)) {
    fprintf(stderr, "Analog AIn scan overrun.\n");
  }

  return nbytes;
}

void usbAInScanStop_USB1208FS_Plus(libusb_device_handle *udev)
{
  /*
    This command stops the analog input scan (if running).
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAInScanConfig_USB1208FS_Plus(libusb_device_handle *udev, uint8_t ranges[8])
{
  /*
    This command reads or writes the analog input configuration.  This
    command will result in a bus stall if an AIn scan is currently
    running.
  */
  int ret = -1;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  ret = libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (unsigned char *) &ranges[0], 8, HS_DELAY);
  if (ret < 0) {
    perror("usbAInScanConfig_USB1208FS_Plus error in writing configuration ranges.");
  }
}

void usbAInScanConfigR_USB1208FS_Plus(libusb_device_handle *udev, uint8_t *ranges)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret = -1;
  ret = libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (unsigned char *) ranges, 0x8, HS_DELAY);
  if (ret < 0) {
    perror("usbAInScanConfigR_USB1208FS_Plus: error in reading ranges.");
  }
}

void usbAInScanClearFIFO_USB1208FS_Plus(libusb_device_handle *udev)
{
  /*
    This command clears the internal scan endpoint FIFOs.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (libusb_control_transfer(udev, requesttype, AIN_CLR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbAInScanClearFIFO_USB1208FS_Plus: error in libusb_control_transfer.");
  }
}

void usbAInBulkFlush_USB1208FS_Plus(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will send a specified number of empty bulk IN packets.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (libusb_control_transfer(udev, requesttype, AIN_BULK_FLUSH, 0x0, 0x0, &count, sizeof(count), HS_DELAY) < 0) {
    perror("usbAInBulkFlush_USB1208FS_Plus: error in libusb_control_transfer.");
  }
}


/***********************************************
 *            Analog Output                    *
 ***********************************************/
void usbAOut_USB1208FS_Plus(libusb_device_handle *udev, uint8_t channel, uint16_t value)
{
  /*
    This command reads the or writes the value of an analog output channel.  This command
    will result in a bus stall if an AOut scan is currently running.

  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (value > 0xfff) value = 0xfff;
  libusb_control_transfer(udev, requesttype, AOUT, value, channel, (unsigned char *) NULL, 0x0, HS_DELAY);
}

uint16_t usbAOutR_USB1208FS_Plus(libusb_device_handle *udev, uint8_t channel)
{
  uint16_t value[2];
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, AOUT, 0x0, 0x0, (unsigned char *) value, sizeof(value), HS_DELAY);
  if (channel == 0) {
    return value[0];
  } else {
    return value[1];
  }
}

void usbAOutScanStop_USB1208FS_Plus(libusb_device_handle *udev)
{
  /*
    This command stops the analog output scan (if running).
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AOUT_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAOutScanClearFIFO_USB1208FS_Plus(libusb_device_handle *udev)
{
  /*
    This command clears the internal scan endpoint FIFOs.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AOUT_CLR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAOutScanStart_USB1208FS_Plus(libusb_device_handle *udev, uint32_t count, double frequency, uint8_t options)
{
  /*
    This command starts an analog output scan. This command will
    result in a bus stall if an AOut scan is currently running.  The
    device will not generate an interal pacer faster than 50 kHz.  The
    pacer rate is set by an internal 32-bit timer running at a base
    rate of 60 MHz.  The timer is controller by pacer_period.  This
    value is the period of the scan and the D/As are clocked at this
    rate.  The equation for calculating pacer_period is:

         pacer_period = [60 MHz / (sample frequency)] - 1

     The data will be sent by the host in packets utilizing the bul
     out endpoint.  The data will be in the format

     lowchannel sample 0: hichannel sample 0
     lowchannel sample 1: hichannel sample 1
     ...
     lowchannel sample n: hichannel sample n

     The output data is written to an internal FIFO.  The bulk
     endpoint data is only accepted if there is room in the FIFO.
     Output data may be sent to the FIFO before the start of the scan.
     The FIFO contents may be cleared with the AOutScanClearFIFO
     command.

     The scan will continue until reaching the specified count or an
     AOutScanStop command is sent.

     Underruns are indicated by the device stalling the bulk endpoint
     during the scan.  The host may read the status to verify and must
     clear the stall condition before further scans can be performed.

  */

  struct AOutScan_t {
    uint32_t count;         // The total number of scans to perform (0 for continuous scan)
    uint32_t pacer_period;  // The pacer timer period
    uint8_t options;        /* bit field that controls scan options:
			    bit 0:    1 = include channel 0 in output scan
			    bit 1:    1 = include channel 1 in output scan
			    bits 2-6: Reserved
			    bit 7:    0 = stall on underrun, 1 = inhibit stall
			 */
  } AOutScan;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (frequency < 0) {
    printf("usbAOutrScanStart_USB1208FS_Plus: frequency must be positive.\n");
    return;
  }
  if (frequency > 50000) {
    frequency = 50000;
    printf("usbAOutrScanStart_USB1208FS_Plus: max frequency 50kHz\n");
  }

  AOutScan.count = count;
  AOutScan.pacer_period = rint((60.E6 / frequency) - 1);
  AOutScan.options = options;
  usbAOutScanStop_USB1208FS_Plus(udev);
  usbAOutScanClearFIFO_USB1208FS_Plus(udev);
  libusb_control_transfer(udev, requesttype, AOUT_SCAN_START, 0x0, 0x0, (unsigned char *) &AOutScan, 9, HS_DELAY);
}

/***********************************************
 *            Counter/Timer                    *
 ***********************************************/
void usbCounterInit_USB1208FS_Plus(libusb_device_handle *udev)
{
  /*
    This command initializes the 32-bit event counter.  On a write, the
    specified counter (0 or 1) will be reset to zero.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, NULL, 0x0, HS_DELAY);
  return;
}

uint32_t usbCounter_USB1208FS_Plus(libusb_device_handle *udev)
{
  /*
    This command reads the 32-bit event counter.  
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t counts = 0x0;

  libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, (unsigned char *) &counts, sizeof(counts), HS_DELAY);
  return counts;
}

/***********************************************
 *            Memory Commands                  *
 ***********************************************/
void usbReadCalMemory_USB1208FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  /*
    This command allows for reading and writing the nonvolatile
     calibration memory.  The cal memory is 768 bytes (address
     0-0x2FF).  The cal memory is write protected and must be unlocked
     in order to write the memory.  The unlock procedure is to write
     the unlock code 0xAA55 to address 0x300.  Writes to the entire
     memory range is then possible.  Write any other value to address
     0x300 to lock the memory after writing.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 768) {
    printf("usbReadCalMemory_USB1208FS_Plus: max bytes that can be written is 768.\n");
    return;
  }

  if (address > 0x2ff) {
    printf("usbCalMemoryR_USB1208FS_Plus: address must be in the range 0 - 0x2ff.\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, CAL_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbWriteCalMemory_USB1208FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{

  uint16_t unlock_code = 0xaa55;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 768) {
    printf("usbWriteCalMemory_USB1208FS_Plus: max bytes that can be written is 768.");
    return;
  }

  if (address > 0x2ff) {
    printf("usbWriteCalMemory_USB1208FS_Plus: address must be in the range 0 - 0x2ff.");
    return;
  }

  libusb_control_transfer(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (unsigned char *) &unlock_code, sizeof(unlock_code), HS_DELAY); // unlock memory
  libusb_control_transfer(udev, requesttype, CAL_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
  libusb_control_transfer(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (unsigned char *) 0x0, sizeof(uint16_t), HS_DELAY); // lock memory
}

void usbReadUserMemory_USB1208FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
    
  /*
    These commands allow for reading and writing the nonvolatile user
     memory. wLength specifies the number of bytes to read or write.
     The user memory is 256 bytes (address 0-0xff)
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 256) {
    printf("usbReadUserMemory_USB1208FS_Plus: max bytes that can be written is 256.");
    return;
  }

  if (address > 0xff) {
    printf("usbReadUserMemory_USB1208FS_Plus: address must be in the range 0 - 0xff.");
    return;
  }
  libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbWriteUserMemory_USB1208FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 255) {
    printf("usbWriteUserMemory_USB1208FS_Plus: max bytes that can be written is 768.");
    return;
  }

  if (address > 0xff) {
    printf("usbWriteUserMemory_USB1208FS_Plus: address must be in the range 0 - 0x2ff.");
    return;
  }
  libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbReadMBDMemory_USB1208FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  /*
    These commands allow for reading and writing the nonvolatile MBD memory. count must
    be less than or equal to 1024 (address 0-0x3ff).
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbReadMBDMemory_USB1208FS_Plus: max bytes that can be written is 1024.");
    return;
  }

  if (address > 0x3ff) {
    printf("usbReadMBDMemory_USB1208FS_Plus: address must be in the range 0 - 0x3ff.");
    return;
  }
  libusb_control_transfer(udev, requesttype, MBD_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbWriteMBDMemory_USB1208FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbWriteMBDMemory_USB1208FS_Plus: max bytes that can be written is 1024.");
    return;
  }

  if (address > 0x3ff) {
    printf("usbWriteUserMemory_USB1208FS_Plus: address must be in the range 0 - 0x3ff");
    return;
  }
  libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/

void usbBlink_USB1208FS_Plus(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return;
}

void usbReset_USB1208FS_Plus(libusb_device_handle *udev)
{
  /*
    This command resets the device
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

uint16_t usbStatus_USB1208FS_Plus(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device.  
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t status = 0x0;

  libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY);
  return status;
}

void usbGetSerialNumber_USB1208FS_Plus(libusb_device_handle *udev, char serial[9])
{
  /*
    This commands reads the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). 
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, SERIAL, 0x0, 0x0, (unsigned char *) serial, 8, HS_DELAY);
  serial[8] = '\0';
  return;
}

void usbDFU_USB1208FS_Plus(libusb_device_handle *udev)
{
  /*
    This command places the device in firmware upgrade mode by erasing
    a portion of the program memory.  The next time the device is
    reset, it will enumerate in the bootloader and is unusable as a
    DAQ device until new firmware is loaded.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t key = 0xadad;

  libusb_control_transfer(udev, requesttype, DFU, key, 0x0, NULL, 0, HS_DELAY);
  return;
} 

void usbMBDCommand_USB1208FS_Plus(libusb_device_handle *udev, uint8_t str[])
{
  /*
    This command is the interface for text-based MBD commands and
    responses.  The length of the string must be passed in wLength for an
    OUT transfer.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MBD_COMMAND, 0x0, 0x0, (unsigned char *) str, strlen((char *) str), HS_DELAY);

}

void usbMBDRaw_USB1208FS_Plus(libusb_device_handle *udev, uint8_t cmd[], uint16_t size)
{
  /*
    This command is the interface for binary responses to certain MBD commands.
   */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MBD_RAW, 0x0, 0x0, (unsigned char *) cmd, size, HS_DELAY);
}

void cleanup_USB1208FS_Plus(libusb_device_handle *udev)
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|1);
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT|2);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}

double volts_USB1208FS_Plus(uint16_t value, uint8_t range)
{
  double volt = 0.0;
  switch(range) {
    case BP_20V:   volt = (value - 0x800)*20.0/2048.; break;
    case BP_10V:   volt = (value - 0x800)*10.0/2048.; break;
    case BP_5V:    volt = (value - 0x800)*5.0/2048.; break;
    case BP_4V:    volt = (value - 0x800)*4.0/2048.; break;
    case BP_2_5V:  volt = (value - 0x800)*2.5/2048.; break;
    case BP_2V:    volt = (value - 0x800)*2.0/2048.; break;
    case BP_1_25V: volt = (value - 0x800)*1.25/2048.; break;
    case BP_1V:    volt = (value - 0x800)*1.0/2048.; break;
    case UP_5V:    volt = value*5.0/4096.0; break;
    default: printf("Unknown range.\n"); break;
  }
  return volt;
}

double volts_USB1408FS_Plus(uint16_t value, uint8_t range)
{
  double volt = 0.0;
  switch(range) {
    case BP_20V:   volt = (value - 0x2000)*20.0/8192.; break;
    case BP_10V:   volt = (value - 0x2000)*10.0/8192.; break;
    case BP_5V:    volt = (value - 0x2000)*5.0/8192.; break;
    case BP_4V:    volt = (value - 0x2000)*4.0/8192.; break;
    case BP_2_5V:  volt = (value - 0x2000)*2.5/8192.; break;
    case BP_2V:    volt = (value - 0x2000)*2.0/8192.; break;
    case BP_1_25V: volt = (value - 0x2000)*1.25/8192.; break;
    case BP_1V:    volt = (value - 0x2000)*1.0/8192.; break;
    default: printf("Unknown range.\n"); break;
  }
  return volt;
}
