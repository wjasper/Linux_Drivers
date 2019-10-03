/*
 *
 *  Copyright (c) 2016 Warren J. Jasper <wjasper@ncsu.edu>
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
#include <stdint.h>
#include <math.h>
#include <time.h>

#include "pmd.h"
#include "bth-1208LS.h"

#define LS_DELAY 30000

static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant.
enum Mode {Differential, SingleEnded};

void usbBuildGainTable_DE_BTH1208LS(libusb_device_handle *udev, float table_DE[NGAINS][NCHAN_DE][2])
{
  /* Builds a lookup table of differential mode calibration
     coefficents to translate values into voltages: The calibration
     coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  int i, j, k;
  uint16_t address = 0x0;

  for (i = 0; i < NGAINS; i++ ) {
    for (j = 0; j < NCHAN_DE; j++) {
      for (k = 0; k < 2; k++) {
	usbCalMemoryR_BTH1208LS(udev, address, 4, (uint8_t *) &table_DE[i][j][k]);
	address += 4;
      }
    }
  }

  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
  if (wMaxPacketSize < 0) {
    perror("BTH1208LS: error in getting wMaxPacketSize");
  }
}

void usbBuildGainTable_SE_BTH1208LS(libusb_device_handle *udev, float table_SE[NCHAN_SE][2])
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
      usbCalMemoryR_BTH1208LS(udev, address, 4, (uint8_t *) &table_SE[i][j]);
      address += 4;
    }
  }
}

void usbCalDate_BTH1208LS(libusb_device_handle *udev, struct tm *date)
{
  /* This command reads the factory calibration date */

  calibrationTimeStamp calDate;
  uint16_t address = 0x200;  // beginning of MFG Calibration date
  time_t time;

  usbCalMemoryR_BTH1208LS(udev, address, sizeof(calDate), (uint8_t *) &calDate);
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
/* reads digital port  */
void usbDIn_BTH1208LS(libusb_device_handle *udev, uint8_t* value)
{
  /* This command reads the current state of the DIO pins */
  
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DIN, 0x0, 0x0, (unsigned char *) value, sizeof(value), LS_DELAY) < 0) {
    perror("usbDIn_BTH1208LS_Plus: error in libusb_control_transfer().");
  }
  return;
}

void usbDOut_BTH1208LS(libusb_device_handle *udev, uint8_t value)
{
  /* This command writes the DIO output latch values.  The factory
     power on default is all 1 (pins are floting).  Since the outputs
     are open drain, writing a 0 turns on the low side transistor and
     drives a 0 to the port pin and writing a 1 turns off the
     transistor and allows the pin to float.  The bits are mapped to
     the individual port pins.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DOUT, value, 0x0, NULL, 0x0, LS_DELAY) < 0) {
    perror("usbDOut_BTH1208LS: error in libusb_control_transfer().");
  }
  return;
}

void usbDOutR_BTH1208LS(libusb_device_handle *udev, uint8_t *value)
{
  /* This command reads the DIO output latch values.  The factory
     power on default is all 1 (pins are floting).  Since the outputs
     are open drain, writing a 0 turns on the low side transistor and
     drives a 0 to the port pin and writing a 1 turns off the
     transistor and allows the pin to float.  The bits are mapped to
     the individual port pins.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DOUT, 0x0, 0x0, value, sizeof(value), LS_DELAY) < 0) {
    perror("usbDOutR_BTH1208LS: error in libusb_control_transfer().");
  }
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/
void usbAIn_BTH1208LS(libusb_device_handle *udev, uint8_t channel, uint8_t mode, uint8_t range, uint16_t *value)
{
  /* This command reads the value of an analog input channel.  This
     command will result in a bus stall if an AInScan is currently
     running.  The range parameter is ignored if the mode is specified
     as single ended.

     channel: the channel to read (0-7)
     mode:    the input mode 0 - single ended, 1 - differential
     range:   the input range for the channel (0-7)
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t wValue, wIndex;

  wValue = channel | (mode << 0x8);
  wIndex = range;

  if (libusb_control_transfer(udev, requesttype, AIN, wValue, wIndex, (unsigned char *) value, sizeof(value), LS_DELAY) < 0) {
    perror("usbAIn_BTH1208LS: error in libusb_control_transfer().");
  }
  return;
}

void usbAInScanStart_BTH1208LS(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count,
			       double frequency, uint8_t channels, uint8_t options)
{
  /* This command starts an analog input scan. This command will
     result in a bus stall if an AIn scan is currently running. The
     device will not generate an internal pacer faster than 50 kHz.

     The pacer rate is set by an internal 32-bit timer running at a
     base rate of 40 MHz. The timer is controlled by
     pacer_period. This value is the period of the scan and the A/Ds
     are clocked at this rate. A pulse will be output at the SYNC pin
     at every pacer_period interval if SYNC is configured as an
     output. The equation for calculating pacer_period is:
   
          pacer_period = [40 MHz / (sample frequency)] - 1 

     If pacer_period is set to 0 the device does not generate an A/D
     clock. It uses the SYNC pin as an input and the user must provide
     the pacer source. The A/Ds acquire data on every rising edge of
     SYNC; the maximum allowable input frequency is 50 kHz.  The data
     will be returned in packets utilizing a bulk in endpoint. The
     data will be in the format: 

     lowchannel sample 0 : lowchannel + 1 sample 0 : … : hichannel sample 0 
     lowchannel sample 1 : lowchannel + 1 sample 1 : … : hichannel sample 1
      … 
     lowchannel sample n : lowchannel + 1 sample n : … : hichannel sample n 

     The scan will not begin until this command is sent and any
     trigger conditions are met. Data will be sent until reaching the
     specified count or an AInScanStop command is sent.

     The external trigger may be used to start the scan. If enabled,
     the device will wait until the appropriate trigger condition is
     detected then begin sampling data at the specified rate. No
     packets will be sent until the trigger is detected. In retrigger
     mode the trigger will be automatically rearmed and the scan will
     restart after retrig_count samples have been acquired. The count
     parameter specifies the total number of samples to acquire and
     should be >= retrig_count. Specifying 0 for count causes
     continuous retrigger scans. The data is still sent as a
     continuous stream during retrigger scan so the last data from a
     previous scan will not be transferred until the beginning of the
     next retrigger scan if it does not end on a packet boundary.

     In block transfer mode the data is sent in 64-byte packets as
     soon as enough data is available from the A/D. In immediate
     transfer mode the data is sent after each sample period,
     resulting in packets that are always 2 bytes (1 sample.) This
     mode should only be used for low pacer rates, typically under 100
     Hz, because it will overrun much easier.  

     Overruns are indicated by the device stalling the bulk in
     endpoint during the scan. The host may read the status to verify
     and must clear the stall condition before further scans can be
     performed.
  */

  struct AInScan_t {
      uint32_t count;         // the total number of samples to acquire, 0 for continuous scan
      uint32_t retrig_count;  // the number of samples to acquire for each trigger in retrigger mode
      uint32_t pacer_period;  // the pacer timer period (0 for external clock)
      uint8_t channels;       // bit field that selects the channels in the scan, upper 4 bits ignored 
                              // in differential mode.
      uint8_t options;        /* bit field that controls scan options
                                 bit 0: 1 = immediate transfer mode, 0 = block
                                 bit 1: 1 = differential mode, 0 = single ended mode
                                 bits 2-4: Trigger setting:
                                   0: no trigger
                                   1: Edge / rising
                                   2: Edge / falling
                                   3: Level / high
                                   4: Level / low
                                 bit 5: 1 = retrigger mode, 0 = normal trigger mode
                                 bit 6: Reserved
                                 bit 7: 0 = stall on overrun, 1 = inhibit stall
			      */
    uint8_t pad[2];           // align along 16 byte boundary
  } AInScan;

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (frequency > 50000.) frequency = 50000.;
  if (frequency > 0.) {
    AInScan.pacer_period = rint((40.E6 / frequency) - 1);
  } else {
    AInScan.pacer_period = 0;
  }
  AInScan.count = count;
  AInScan.retrig_count = retrig_count;
  AInScan.channels = channels;
  AInScan.options = options;

  usbAInScanStop_BTH1208LS(udev);
  usbAInScanClearFIFO_BTH1208LS(udev);

  /* Pack the data into 14 bytes */
  if (libusb_control_transfer(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (unsigned char *) &AInScan, 14, LS_DELAY) < 0) {
    perror("usbAInScanStart_BTH1208LS: error in libusb_control_transfer().");
  }
}

int usbAInScanRead_BTH1208LS(libusb_device_handle *udev, uint32_t count, uint16_t *data, uint8_t options)
{
  int i;
  int ret = -1;
  int nbytes = 2*count;    // number of bytes to read in 64 bit chunks
  int transferred;
  uint16_t status;
  unsigned char value[MAX_PACKET_SIZE];

  if (count == 0) {  // in continuous mode
    nbytes = 256;
  }

  if (options & IMMEDIATE_TRANSFER_MODE) {
    for (i = 0; i < nbytes/2; i++) {
      ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) &data[i], 2, &transferred, 2000);
      if (ret < 0) {
	perror("usbAInScanRead_BTH1208LS: error in usb_bulk_transfer.");
      }
      if (transferred != 2) {
	fprintf(stderr, "usbAInScanRead_BTH1208LS: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
      }
    }
  } else { 
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) data, nbytes, &transferred, LS_DELAY);
    if (ret < 0) {
      perror("usbAInScanRead_BTH1208LS: error in usb_bulk_transfer.");
    }
    if (transferred != nbytes) {
      fprintf(stderr, "usbAInScanRead_BTH1208LS: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
    }
  }

  status = usbStatus_BTH1208LS(udev);
  if ((status & AIN_SCAN_OVERRUN)) {
    printf("Analog AIn scan overrun.\n");
  }

  if (count == 0) {
    return nbytes/2;
  }

  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
  if ((nbytes%wMaxPacketSize) == 0 && !(status & AIN_SCAN_RUNNING)) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) value, 2, &ret, 100);
  }

  return nbytes/2;
}

void usbAInScanStop_BTH1208LS(libusb_device_handle *udev)
{
  /*  This command stops the analog input scan (if running). */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, LS_DELAY) < 0) {
    perror("usbAInScanStop_BTH1208LS: error in libusb_control_transfer().");
  }
}

void usbAInConfigW_BTH1208LS(libusb_device_handle *udev, uint8_t ranges[4])
{
  /* This command reads or writes the analog input ranges used for
     AInScan in differential mode.  The command will result in a bus
     stall if an AInScan is currently running.  See bth-1208LS.h for
     values of ranges.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (unsigned char *) &ranges[0], 4, LS_DELAY) < 0) {
    perror("usbAInConfigW_BTH1208LS error in writing configuration ranges.");
  }
}

void usbAInConfigR_BTH1208LS(libusb_device_handle *udev, uint8_t ranges[4])
{
  
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (unsigned char *) &ranges[0], 4, LS_DELAY) < 0) {
    perror("usbAInConfigR_BTH1208LS error in reading configuration ranges.");
  }
}

void usbAInScanClearFIFO_BTH1208LS(libusb_device_handle *udev)
{
  /* This command clears the internal scan endpoint FIFOs. */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AIN_SCAN_CLEAR_FIFO, 0x0, 0x0, NULL, 0, LS_DELAY) < 0) {
    perror("usbAInScanClearFIFO_BTH1208LS: error in libusb_control_transfer.");
  }
}

/***********************************************
 *            Analog Output                    *
 ***********************************************/
void usbAOut_BTH1208LS(libusb_device_handle *udev, uint8_t channel, uint16_t value)
{
  /* This command writes the values of an analog output channel */
  
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (channel > 2) {
    printf("usbAOut_BTH1208LS: channel must be 0 or 1.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, AOUT, value, channel, NULL, 0x0, LS_DELAY) < 0) {
    perror("usbAOut_BTH1208LS error libusb_control_transfer.");
  }
}

uint16_t usbAOutR_BTH1208LS(libusb_device_handle *udev, uint8_t channel)
{
  /* This command reads the value of  both  analog output channels */

  uint16_t value[2];
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AOUT, 0x0, 0x0, (unsigned char *) value, sizeof(value), LS_DELAY) < 0) {
    perror("usbAOutR_BTH1208LS error libusb_control_transfer.");
  }
  if (channel == 0) {
    return value[0];
  } else {
    return value[1];
  }
}

/***************************************
 *            Counter                  *
 ***************************************/
void usbCounter_BTH1208LS(libusb_device_handle *udev, uint32_t *counter)
{
  /* The command reads the event counter. */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, (unsigned char *) counter, 4, LS_DELAY) < 0) {
    perror("usbCounter_BTH1208LS error libusb_control_transfer.");
  }
}

void usbResetCounter_BTH1208LS(libusb_device_handle *udev)
{
  /* This command resets the event counter to 0 */
  
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, NULL, 0, LS_DELAY) < 0) {
    perror("usbCounterReset_BTH1208LS error libusb_control_transfer.");
  }
}

/***********************************************
 *            Memory Commands                  *
 ***********************************************/
void usbCalMemoryR_BTH1208LS(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  /*
    This command allows for reading the nonvolatile calibration
    memory.  The cal memory is 768 bytes (address 0-0x2FF).  The cal
    memory is write protected and must be unlocked in order to write
    the memory.  The unlock procedure is to write the unlock code
    0xAA55 to address 0x300.  Writes to the entire memory range is
    then possible.  Write any other value to address 0x300 to lock the
    memory after writing.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 768) {
    printf("usbCalMemoryR_BTH1208LS: max bytes that can be read is 768.\n");
    return;
  }

  if (address > 0x2ff) {
    printf("usbCalMemoryR_BTH1208LS: address must be in the range 0 - 0x2FF.\n");
    return;
  }
  
  if (libusb_control_transfer(udev, requesttype, CAL_MEMORY, address, 0x0, (unsigned char *) data, count, LS_DELAY) < 0) {
    perror("usbCalMemoryR_BTH1208LS: error in libusb_control_transfer()");
  }
}

void usbCalMemoryW_BTH1208LS(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  uint16_t unlock_code = 0xaa55;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 768) {
    printf("usbCalMemoryW_BTH1208LS: max bytes that can be written is 768.");
    return;
  }

  if (address > 0x2ff) {
    printf("usbCalMemoryW_BTH1208LS: address must be in the range 0 - 0x2ff.");
    return;
  }
  
  if (libusb_control_transfer(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (unsigned char *) &unlock_code, sizeof(unlock_code), LS_DELAY) < 0) { // unlock memory
    perror("usbCalMemoryW_BTH1208LS: error in unlocking memory.");
  }
  if (libusb_control_transfer(udev, requesttype, CAL_MEMORY, address, 0x0, (unsigned char *) data, count, LS_DELAY) < 0) {
    perror("usbCalMemoryW_BTH1208LS: error in reading calibration memory.");
  }
  if (libusb_control_transfer(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (unsigned char *) 0x0, sizeof(uint16_t), LS_DELAY) < 0) {  // lock memory
    perror("usbCalMemoryW_BTH1208LS: error in locking memory.");
  }
}

void usbUserMemoryR_BTH1208LS(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  /*
    This command allow for reading and writing the nonvolatile user
     memory. wLength specifies the number of bytes to read or write.
     The user memory is 256 bytes (address 0-0xFF)
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 256) {
    printf("usbUserMemoryR_BTH1208LS: max bytes that can be read is 256.");
    return;
  }

  if (address > 0xff) {
    printf("usbUserMemoryR_BTH1208LS: address must be in the range 0 - 0xff.");
    return;
  }
  if (libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, LS_DELAY) < 0) {
    perror("usbUserMemoryR_BTH1208LS: error in libusb_control_transfer().");
  }
}

void usbUserMemoryW_BGTH1208LS(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 255) {
    printf("usbUserMemoryW_BTH1208LS: max bytes that can be written is 768.");
    return;
  }

  if (address > 0xff) {
    printf("usbUserMemoryW_BTH1208LS: address must be in the range 0 - 0x2ff.");
    return;
  }
  if (libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, LS_DELAY) < 0) {
    perror("usbUserMemoryW_BTH1208LS: error in libusb_control_transfer().");
  }
}

void usbSettingsMemoryR_BTH1208LS(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  /* This command allows for reading and writing the nonvolite
     settings memory.  wLength specifies the number of bytes to read
     or write.  The use memory is 1024 bytes (address 0 - 0x3FF).  If
     the settings are written they will be implemented immediately.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 512) {
    printf("usbSettingsMemoryR_BTH1208LS: max bytes that can be read is 512.");
    return;
  }

  if (address > 0x3ff) {
    printf("usbSettingsMemoryR_BTH1208LS: address must be in the range 0 - 0x3FF.");
    return;
  }
  if (libusb_control_transfer(udev, requesttype, SETTINGS_MEMORY, address, 0x0, (unsigned char *) data, count, LS_DELAY) < 0) {
    perror("usbSettingsMemoryR_BTH1208LS: error in libusb_control_transfer().");
  }
}

void usbSettingsMemoryW_BTH1208LS(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 512) {
    printf("usbSettingsMemoryW_BTH1208LS: max bytes that can be written is 512.");
    return;
  }

  if (address > 0x3ff) {
    printf("usbSettngsMemoryW_BTH1208LS: address must be in the range 0 - 0x2ff.");
    return;
  }
  if (libusb_control_transfer(udev, requesttype, SETTINGS_MEMORY, address, 0x0, (unsigned char *) data, count, LS_DELAY) < 0) {
    perror("usbSettingsMemoryW_BTH1208LS: error in libusb_control_transfer().");
  }
}

/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/

void usbBlinkLED_BTH1208LS(libusb_device_handle *udev, uint8_t count)
{
  /*  This command will blink the device LED "count" number of times */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), LS_DELAY) < 0) {
    perror("usbBlinkLED_BTH1208LS: error in libusb_control_transfer");
  }
  return;
}

void usbReset_BTH1208LS(libusb_device_handle *udev)
{
  /* This command resets the device */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, LS_DELAY) < 0) {
    perror("usbReset_BTH1208LS: error in libusb_control_transfer.");
  }
  return;
}

uint16_t usbStatus_BTH1208LS(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device.  Writing the command will clear
    the error indicators.

    status:      bit 0:      Reserved
                 bit 1:      1 = AIn scan running
                 bit 2:      1 = AIn scan overrun
                 bits 3-7:   Reserved
                 bits 8-10:  Charger status:
                              0 = no battery
                              1 = fast charge
                              2 = maintenance charge
                              3 = fault (not charging)
                              4 = disabled
                 bits 11-15   Reserved
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t status = 0x0;

  if (libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), LS_DELAY) < 0) {
    perror("usbStatus_BTH1208LS: error in libusb_control_transfer.");
  }
  return status;
}

void usbInitRadio_BTH1208LS(libusb_device_handle *udev)
{
  /* This command sets the radio module to factory default settings.
     This should be used during manufacturing to initialize the radio.
  */
  
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  
  if (libusb_control_transfer(udev, requesttype, INIT_RADIO, 0x0, 0x0, (unsigned char *) NULL, 0x0, LS_DELAY) < 0) {
    perror("usbInitRadio_BTH1208LS: error in libusb_control_transfer.");
  }
}

void usbBluetoothPinR_BTH1208LS(libusb_device_handle *udev, char *pin)
{
  /* This command reads the Bluetooth PIN code.  The PIN code can be
     up to 16 ASCII characters.  The length of the code is specified
     in wLength.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);  

  if (libusb_control_transfer(udev, requesttype, BLUETOOTH_PIN, 0x0, 0x0, (unsigned char *) pin, 16, LS_DELAY) < 0) {
    perror("usbBluetoothPinR_BTH1208LS: error in libusb_control_transfer.");
  }
}

void usbBluetoothPinW_BTH1208LS(libusb_device_handle *udev, char *pin)
{
  /* This command writes the Bluetooth PIN code.  The PIN code can be
     up to 16 ASCII characters.  The length of the code is specified
     in wLength.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);  

  if (libusb_control_transfer(udev, requesttype, BLUETOOTH_PIN, 0x0, 0x0, (unsigned char *) pin, strlen(pin), LS_DELAY) < 0) {
    perror("usbBluetoothPinW_BTH1208LS: error in libusb_control_transfer.");
  }
}

void usbBatteryVoltage_BTH1208LS(libusb_device_handle *udev, uint16_t *voltage)
{
  /* This command reads the voltage of the battery.  The voltage is
     returned in millivolts.
  */
  
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);  

  if (libusb_control_transfer(udev, requesttype, BATTERY_VOLTAGE, 0x0, 0x0, (unsigned char *) voltage, sizeof(voltage), LS_DELAY) < 0) {
    perror("usbBatteryVoltage_BTH1208LS: error in libusb_control_transfer.");
  }
}

void usbGetSerialNumber_BTH1208LS(libusb_device_handle *udev, char serial[9])
{
  /*
    This commands reads the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). 
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, SERIAL, 0x0, 0x0, (unsigned char *) serial, 8, LS_DELAY) < 0) {
    perror("usbGetSerialNuber_BTH1208LS: error in libusb_control_transfer.");
  }
  serial[8] = '\0';
}

void usbRadioFirmwareVersion_BTH1208LS(libusb_device_handle *udev, uint16_t *version)
{
  /* This command reads the radio firmware version.  The version
    consists of 16 bits in hexadecimal BCD notation.  i.e. version
    6.15 would be 0x0615. 
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, RADIO_FIRMWARE_VERSION, 0x0, 0x0, (unsigned char *) version, sizeof(version), LS_DELAY) < 0) {
    perror("usbRadioFirmwareVersion_BTH1208LS: error in libusb_control_transfer.");
  }
}

void usbDFU_BTH1208LS(libusb_device_handle *udev)
{
  /*
    This command places the device in firmware upgrade mode by erasing
    a portion of the program memory.  The next time the device is
    reset, it will enumerate in the bootloader and is unusable as a
    DAQ device until new firmware is loaded.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t key = 0xadad;

  if (libusb_control_transfer(udev, requesttype, DFU, key, 0x0, NULL, 0, LS_DELAY) < 0) {
    perror("usbDFU_BTH1208LS: error in libusb_control_transfer.");
  }
  return;
} 

void cleanup_BTH1208LS(libusb_device_handle *udev)
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|1);
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT|2);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}

double volts_BTH1208LS(uint16_t value, uint8_t range)
{
  double volt = 0.0;
  switch(range) {
    case BP_20V:   volt = (value - 0x800)*20.0/2048.; break;
    case BP_10V:   volt = (value - 0x800)*10.0/2048.; break;
    case BP_5V:    volt = (value - 0x800)*5.0/2048.;  break;
    case BP_4V:    volt = (value - 0x800)*4.0/2048.;  break;
    case BP_2_5V:  volt = (value - 0x800)*2.5/2048.;  break;
    case BP_2V:    volt = (value - 0x800)*2.0/2048.;  break;
    case BP_1_25V: volt = (value - 0x800)*1.25/2048.; break;
    case BP_1V:    volt = (value - 0x800)*1.0/2048.;  break;
    case UP_2_5V:  volt = value*2.5/4096.0;           break; // analog output
    default:       printf("Unknown range.\n");        break;
  }
  return volt;
}
