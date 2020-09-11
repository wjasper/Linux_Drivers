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
#include <string.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-1608G.h"
#include "usb-1608G.rbf"
#include "usb-1608G-2.rbf" 

/* Commands and USB Report ID for the USB 1608G */
/* Digital I/O Commands */
#define DTRISTATE         (0x00) // Read/Write Tristate register
#define DPORT             (0x01) // Read digital port pins
#define DLATCH            (0x02) // Read/Write Digital port output latch register

/* Analog Input Commands */
#define AIN               (0x10) // Read analog input channel
#define AIN_SCAN_START    (0x12) // Start input scan
#define AIN_SCAN_STOP     (0x13) // Stop input scan
#define AIN_CONFIG        (0x14) // Analog input channel configuration
#define AIN_CLR_FIFO      (0x15) // Clear any remaining input in FIFO after scan.

/* Analog Output Commands  USB-1608GX-2A0 only*/
#define AOUT              (0x18) // Read/Write analog output channel
#define AOUT_SCAN_START   (0x1A) // Start analog ouput scan
#define AOUT_SCAN_STOP    (0x1B) // Stop analog output scan
#define AOUT_CLEAR_FIFO   (0x1C) // Clear data in analog output FIFO

/* Counter/Timer Commands */
#define COUNTER           (0x20) // Read/reset event counter
#define TIMER_CONTROL     (0x28) // Read/write timer control register
#define TIMER_PERIOD      (0x29) // Read/write timer period register
#define TIMER_PULSE_WIDTH (0x2A) // Read/write timer pulse width register
#define TIMER_COUNT       (0x2B) // Read/write timer counter register
#define TIMER_START_DELAY (0x2C) // Read/write timer start delay register
#define TIMER_PARAMETERS  (0x2D) // Read/write timer parameters

/* Memory Commands */
#define MEMORY            (0x30) // Read/Write EEPROM
#define MEM_ADDRESS       (0x31) // EEPROM read/write address value
#define MEM_WRITE_ENABLE  (0x32) // Enable writes to firmware area

/* Miscellaneous Commands */  
#define STATUS            (0x40) // Read device status
#define BLINK_LED         (0x41) // Causes LED to blink
#define RESET             (0x42) // Reset device
#define TRIGGER_CONFIG    (0x43) // External trigger configuration
#define CAL_CONFIG        (0x44) // Calibration voltage configuration
#define TEMPERATURE       (0x45) // Read internal temperature
#define SERIAL            (0x48) // Read/Write USB Serial Number

/* FPGA Configuration Commands */
#define FPGA_CONFIG       (0x50) // Start FPGA configuration
#define FPGA_DATA         (0x51) // Write FPGA configuration data
#define FPGA_VERSION      (0x52) // Read FPGA version

#define HS_DELAY 2000

static int wMaxPacketSize = 0;          // will be the same for all devices of this type so
                                        // no need to be reentrant. 

void usbBuildGainTable_USB1608G(libusb_device_handle *udev, float table[NGAINS_1608G][2])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
       voltage = value*table[gain#][0] + table[gain#][1]
     only needed for fast lookup.
  */
  int i, j;
  uint16_t address = 0x7000;

  usbMemAddressW_USB1608G(udev, address);  // Beginning of Calibration Table
  for (i = 0; i < NGAINS_1608G; i++) {
    for (j = 0; j < 2; j++) {
      usbMemoryR_USB1608G(udev, (uint8_t *) &table[i][j], sizeof(float));
    }
  }
  return;
}

void usbBuildGainTable_USB1608GX_2AO(libusb_device_handle *udev, float table_AO[NCHAN_AO_1608GX][2])
{
  /*
    Builds a lookup table of calibration coefficents to translate values into voltages:
    corrected value = value*table[VDAC#][0] + table[VDAC][1]
  */

  int j, k;
  uint16_t address = 0x07080;

  usbMemAddressW_USB1608G(udev, address);
  
  for (j = 0; j < NCHAN_AO_1608GX; j++) {
    for (k = 0; k < 2; k++) {
      usbMemoryR_USB1608G(udev, (uint8_t *) &table_AO[j][k], sizeof(float));
    }
  }
  return;
}

void usbInit_1608G(libusb_device_handle *udev, int version)
{
  int i;

  /* This function does the following:
     1. Configure the FPGA
     2. Finds the maxPacketSize for bulk transfers
  */
  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
  if (wMaxPacketSize < 0) {
    perror("usbInit_1608G: error in getting wMaxPacketSize");
  }

  if (version == 1) {  // older version
    if (!(usbStatus_USB1608G(udev) & FPGA_CONFIGURED)) {
      usbFPGAConfig_USB1608G(udev);
      if (usbStatus_USB1608G(udev) & FPGA_CONFIG_MODE) {
	for (i = 0; i <= (sizeof(FPGA_data) - 64); i += 64) {
	  usbFPGAData_USB1608G(udev, &FPGA_data[i], 64);
	}
	if (sizeof(FPGA_data) % 64) {
	  usbFPGAData_USB1608G(udev, &FPGA_data[i], sizeof(FPGA_data)%64);
	}
	if (!(usbStatus_USB1608G(udev) & FPGA_CONFIGURED)) {
	  printf("Error: FPGA for the USB-1608G is not configured.  status = %#x\n", usbStatus_USB1608G(udev));
	  return;
	}
      } else {
	printf("Error: could not put USB-1608G into FPGA Config Mode.  status = %#x\n", usbStatus_USB1608G(udev));
	return;
      }
    } else {
      printf("USB-1608G FPGA configured.\n");
      return;
    }
  } else {  // newer version of FPGA
    if (!(usbStatus_USB1608G(udev) & FPGA_CONFIGURED)) {
      usbFPGAConfig_USB1608G(udev);
      if (usbStatus_USB1608G(udev) & FPGA_CONFIG_MODE) {
	for (i = 0; i <= (sizeof(FPGA_V2_data) - 64); i += 64) {
	  usbFPGAData_USB1608G(udev, &FPGA_V2_data[i], 64);
	}
	if (sizeof(FPGA_V2_data) % 64) {
	  usbFPGAData_USB1608G(udev, &FPGA_V2_data[i], sizeof(FPGA_V2_data)%64);
	}
	if (!(usbStatus_USB1608G(udev) & FPGA_CONFIGURED)) {
	  printf("Error: FPGA for the USB-1608G is not configured.  status = %#x\n", usbStatus_USB1608G(udev));
	  return;
	}
      } else {
	printf("Error: could not put USB-1608G into FPGA Config Mode.  status = %#x\n", usbStatus_USB1608G(udev));
	return;
      }
    } else {
      printf("USB-1608G FPGA configured.\n");
      return;
    }
  }
}

void usbCalDate_USB1608G(libusb_device_handle *udev, struct tm *date)
{
  /* This command reads the factory calibration date 
     Note: The calibration date is stored in the EEPROM
     starting at address 0x7098.  The six date elements
     (year, month, day, hour, minute, second) are stored
     as big endian unsigned short.
*/

  uint8_t calDate[12];
  uint16_t address = 0x7098;  // beginning of MFG Calibration date
  time_t time;

  usbMemAddressW_USB1608G(udev, address);
  usbMemoryR_USB1608G(udev, (uint8_t *) &calDate, sizeof(calDate));
  date->tm_year = ((calDate[0] << 8) | calDate[1]) - 1900;  // years since 1900
  date->tm_mon = calDate[3] - 1;                            // month starts at 0
  date->tm_mday = calDate[5];
  date->tm_hour = calDate[7];
  date->tm_min = calDate[9];
  date->tm_sec = calDate[11]; 
  time = mktime(date);
  date = localtime(&time);
  
}

/***********************************************
 *            Digital I/O                      *
 ***********************************************/

/* Read/Write digital port tristate register */

uint16_t usbDTristateR_USB1608G(libusb_device_handle *udev)
{
  /* This command reads or writes the digital port tristate register.
   The tristate register determines if the latch register value is driven onto
   the port pin.  A '1' in the tristate register makes the corresponding
   pin an input, a '0' makes it an output.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDTristateR_USB1608G: error in libusb_control_transfer().");
  }
  return data;
}

void usbDTristateW_USB1608G(libusb_device_handle *udev, uint16_t value)
{

  /* This command reads or writes the digital port tristate register.
   The tristate register determines if the latch register value is driven onto
   the port pin.  A '1' in the tristate register makes the corresponding
   pin an input, a '0' makes it an output.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbDTristateW_USB1608G: error in libusb_control_transfer()");
  }
  return;
}

/* reads digital word  */
uint16_t usbDPort_USB1608G(libusb_device_handle *udev)
{
  /*
    This command reads the current state of the digital pins.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data;

  if (libusb_control_transfer(udev, requesttype, DPORT, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDPort_USB1608G: error in libusb_control_transfer().");
  }
  return data;
}

/* read/writes digital port latch */
uint16_t usbDLatchR_USB1608G(libusb_device_handle *udev)
{
  /*
    This command reads the digital port latch register
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DLATCH, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDLatchR_USB1608G: error in libusb_control_transfer().");
  }
  return data;
}

void usbDLatchW_USB1608G(libusb_device_handle *udev, uint16_t value)
{
  /*
    This command writes the digital port latch register
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DLATCH, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbDLatchW_USB1608G: error in libusb_control_transfer().");
  }
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/

uint16_t usbAIn_USB1608G(libusb_device_handle *udev, uint16_t channel)
{
  /*
    This command returns the  value from an analog input channel.  This
    command will result in a bus stall if AInScan is currently running.
  */
  uint16_t value;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AIN, channel, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY) < 0) {
    perror("usbAIn_USB1608G: error in libusb_control_transfer.");
  }
  return value;
}

void usbAInScanStart_USB1608G(libusb_device_handle *udev, usbDevice1608G *usb1608G)
{
  /* This command starts the analog input channel scan.  The gain
     ranges that are currently set on the desired channels will be
     used (these may be changed with AInConfig) This command will
     result in a bus stall if an AInScan is currently running.

     count:        the total number of scans to perform (0 for continuous scan)
     retrig_count: the number of scan to perform for each trigger in retrigger mode
     pacer_period: pacer timer period value (0 for AI_CLK_IN)
     frequency:    pacer frequency in Hz
     packet_size:  number of samples to transfer at a time
     options:      bit field that controls various options
                   bit 0: 1 = burst mode, 0 = normal mode
                   bit 1: Reserved
                   bit 2: Reserved
                   bit 3: 1 = use trigger  0 = no trigger
                   bit 4: Reserved
                   bit 5: Reserved
                   bit 6: 1 = retrigger mode, 0 = normal trigger
                   bit 7: Reserved
    mode:  mode bits:
           bit 0:   0 = counting mode,  1 = CONTINUOUS_READOUT
           bit 1:   1 = SINGLEIO
           bit 2:   1 = use packet size passed usbDevice1608G->packet_size

     Notes:

     The pacer rate is set by an internal 32-bit incrementing timer
     running at a base rate of 64 MHz.  The timer is controlled by
     pacer_period. If burst mode is specified, then this value is the
     period of the scan and the A/D is clocked at this maximum rate
     (500 kHz) for each channel in the scan.  If burst mode is not
     specified, then this value is the period of the A/D readings.  A
     pulse will be output at the AI_CLK_OUT pin at every pacer_period
     interval regardless of the mode.

     If pacer_period is set to 0, the device does not generate an A/D
     clock.  It uses the AI_CLK_IN pin as the pacer source.  Burst
     mode operates in the same fashion: if specified, the scan starts
     on every rising edge of AI_CLK_IN and the A/D is clocked at 500 kHz
     for the number of channels in the scan; if not specified, the A/D
     is clocked on every rising edge of AI_CLK_IN.

     The timer will be reset and sample acquired when its value equals
     timer_period.  The equation for calculating timer_period is:

     timer_period = [64MHz / (sample frequency)] - 1

     The data will be returned in packets utilizing a bulk IN endpoint.
     The data will be in the format:

     lowchannel sample 0: lowchannel + 1 sample 0: ... :hichannel sample 0
     lowchannel sample 1: lowchannel + 1 sample 1: ... :hichannel sample 1
     ...
     lowchannel sample n: lowchannel + 1 sample n: ... :hichannel sample n

     The packet_size parameter is used for low sampling rates to avoid
     delays in receiving the sampled data. The buffer will be sent,
     rather than waiting for the buffer to fill.  This mode should
     not be used for high sample rates in order to avoid data loss.

     The external trigger may be used to start data collection
     synchronously.  If the bit is set, the device will wait until the
     appropriate trigger edge is detected, then begin sampling data at
     the specified rate.  No messages will be sent until the trigger
     is detected.

     The retrigger mode option and the retrig_count parameter are only
     used if trigger is used.  This option will cause the trigger to
     be rearmed after retrig_count samples are acquired, with a total
     of count samples being returned from the entire scan.
     
  */

  struct AInScan_t {
    uint32_t count;        // The total number of scans to perform (0 for continuous scan)
    uint32_t retrig_count; // The numer of scans to perform for each trigger in retrigger mode.
    uint32_t pacer_period; // The pacer timer period value. (0 for AI_CLK_IN).
    uint8_t packet_size;   // Number of samples - 1 to transfer at a time.
    uint8_t options;    /* bit 0:  1 = burst mode
                           bit 1:  Reserved
	                   bit 2:  Reserved
                           bit 3:  1 = use trigger
                           bit 4:  Reserved
			   bit 5:  Reserved
			   bit 6:  1 = retrigger mode, 0 = normal trigger
			   bit 7:  Reserved
		        */
    uint8_t pad[2];
  } AInScan;

  uint16_t packet_size;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int bytesPerScan = (usb1608G->lastElement+1)*2;    // number of bytes transferred in one scan;

  if (usb1608G->frequency == 0.0) {
    AInScan.pacer_period = 0; // use external pacer
  } else {
    AInScan.pacer_period = rint((BASE_CLOCK / usb1608G->frequency) - 1);
  }

  if (usb1608G->count == 0) {
    usb1608G->mode |= USB_CONTINUOUS_READOUT;
    usb1608G->bytesToRead = -1;                                          // disable and sample forever
  } else {
    usb1608G->bytesToRead = usb1608G->count*(usb1608G->lastElement+1)*2;  // total number of bytes to read
  }

  if (usb1608G->mode & USB_FORCE_PACKET_SIZE) {
    packet_size = usb1608G->packet_size;
  } else if (usb1608G->mode & USB_SINGLEIO) {
    packet_size = usb1608G->lastElement + 1;
  } else if (usb1608G->mode & USB_CONTINUOUS_READOUT) {
    packet_size = (( (wMaxPacketSize/bytesPerScan) * bytesPerScan) / 2);
  } else {
    packet_size = wMaxPacketSize/2;
  }
  usb1608G->packet_size = packet_size;

  if (usb1608G->mode & USB_CONTINUOUS_READOUT) {
    AInScan.count = 0;
  } else {
    AInScan.count = usb1608G->count;
  }
  AInScan.retrig_count = usb1608G->retrig_count;
  AInScan.packet_size = (uint8_t) packet_size - 1;   // force to uint8_t since in range 0-255
  AInScan.options = usb1608G->options;

  /* Pack the data into 14 bytes */
  if (libusb_control_transfer(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (unsigned char *) &AInScan, 14, HS_DELAY) < 0) {
    perror("usbAInScanStart_USB1608G: Error");
  }
  usb1608G->status = usbStatus_USB1608G(udev);
}

 int usbAInScanRead_USB1608G(libusb_device_handle *udev, usbDevice1608G *usb1608G,  uint16_t *data)
{
  char value[PACKET_SIZE];
  int ret = -1;
  int nbytes;     // number of bytes to read
  int transferred;

  if ((usb1608G->status & AIN_SCAN_RUNNING) == 0x0) {
    perror("usbScanRead_USB1608G: pacer must be running to read from buffer");
    return -1;
  }

  if ((usb1608G->mode & USB_CONTINUOUS_READOUT) || (usb1608G->mode & USB_SINGLEIO)) {
    nbytes = 2*(usb1608G->packet_size);
  } else {
    nbytes = usb1608G->count*(usb1608G->lastElement+1)*2;
  }

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) data, nbytes, &transferred, HS_DELAY);

  if (transferred != nbytes) {
    fprintf(stderr, "usbAInScanRead_USB1608G: number of bytes transferred = %d, nbytes = %d   bytesToRead = %d\n",
	    transferred, nbytes, usb1608G->bytesToRead);
    return transferred;
  }

  if (ret < 0) {
    perror("usbAInScanRead_USB1608G: error in libusb_bulk_transfer.");
    return ret;
  }

  if (usb1608G->bytesToRead > transferred) {
    usb1608G->bytesToRead -= transferred;
  } else if (usb1608G->bytesToRead > 0 && usb1608G->bytesToRead < transferred) {  // all done
    usbAInScanStop_USB1608G(udev);
    usbAInScanClearFIFO_USB1608G(udev);
    usb1608G->status = usbStatus_USB1608G(udev);
    return usb1608G->bytesToRead;
  }

  if (usb1608G->mode & USB_CONTINUOUS_READOUT) { // continuous mode
    return transferred;
  }

  int dummy;
  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
  if ((nbytes%wMaxPacketSize) == 0) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) value, 2, &dummy, 100);
  }

  usb1608G->status = usbStatus_USB1608G(udev);
  if ((usb1608G->status & AIN_SCAN_OVERRUN)) {
    perror("Scan overrun.");
    usbAInScanStop_USB1608G(udev);
    usbAInScanClearFIFO_USB1608G(udev);
  }

  return transferred;
}


void usbAInConfig_USB1608G(libusb_device_handle *udev, usbDevice1608G *usb1608G)
{
  /*
    This command reads or writes the analog input channel
    configurations.  This command will result in a bus stall if an
    AInScan is currently running.

    mode:   SINGLE_ENDED  (Single-Ended)
            DIFFERENTIAL  (Differential)
            CALIBRATION   (Calibration mode)
            LAST_CHANNEL  (End of scan)               

    range:  0: +/- 10V range
            1: +/- 5V range
            2: +/- 2V range
	    3: _/- 1V range

  */

  int i;
  int ret;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  for (i = 0; i < NCHAN_1608G; i++) {
    if ((usb1608G->list[i].mode & 0x3) == SINGLE_ENDED && usb1608G->list[i].channel >= 0 && usb1608G->list[i].channel < 8) {
      usb1608G->scan_list[i] = (0x20 | (usb1608G->list[i].channel & 0x7));
    } else if ((usb1608G->list[i].mode & 0x3) == SINGLE_ENDED && usb1608G->list[i].channel >= 8 && usb1608G->list[i].channel < 16) {
      usb1608G->scan_list[i] = (0x40 | (usb1608G->list[i].channel & 0x7));
    } else if ((usb1608G->list[i].mode & 0x3) == DIFFERENTIAL && usb1608G->list[i].channel >= 0 && usb1608G->list[i].channel < 8) {
      usb1608G->scan_list[i] =  (usb1608G->list[i].channel & 0x7);
    } else if ((usb1608G->list[i].mode & 0x3) == CALIBRATION) {
      usb1608G->scan_list[i] = 0x60;
    } else {
      printf("Error in Scan List[%d]  mode = %#x   channel = %d  range = %#x\n",
	     i, usb1608G->list[i].mode, usb1608G->list[i].channel, usb1608G->list[i].range);
      return;
    }
    
    usb1608G->scan_list[i] |= (usb1608G->list[i].range << 3);

    /*
    printf("Scan List[%d]  mode = %#x   channel = %d  range = %#x  raw = %#x\n",
	   i, scanList[i].mode, scanList[i].channel, scanList[i].range, scan_list[i]);
    */

    if (usb1608G->list[i].mode & LAST_CHANNEL) {
      usb1608G->scan_list[i] |= LAST_CHANNEL;
      usb1608G->lastElement = i;
      break;
    }
  }

  if (usbStatus_USB1608G(udev) | AIN_SCAN_RUNNING) {
    usbAInScanStop_USB1608G(udev);
  }

  ret = libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (unsigned char*) &usb1608G->scan_list[0], 16, HS_DELAY);
  if (ret < 0) {
    perror("usbAInConfig_USB1608G Error.");
  }
}

int usbAInConfigR_USB1608G(libusb_device_handle *udev, usbDevice1608G *usb1608G)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  int i;

  if (usbStatus_USB1608G(udev) | AIN_SCAN_RUNNING) {
    usbAInScanStop_USB1608G(udev);
  }
  
  ret = libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (unsigned char *) usb1608G->scan_list, 15, HS_DELAY);
  if (ret < 0) {
    perror("usbAInConfigR_USB1608G Error.");
  }

  for (i = 0; i < NCHAN_1608G; i++) {
    if (usb1608G->scan_list[i] & LAST_CHANNEL) break;
  }
  usb1608G->lastElement = i;
  return ret;
}

void usbAInScanStop_USB1608G(libusb_device_handle *udev)
{
  /*
    This command stops the analog input scan (if running).
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAInScanClearFIFO_USB1608G(libusb_device_handle *udev)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_CLR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}


/***********************************************
 *            Analog Output                    *
 ***********************************************/

void usbAOut_USB1608GX_2AO(libusb_device_handle *udev, uint8_t channel, double voltage, float table_AO[NCHAN_AO_1608GX][2])
{
  /*
    This command reads or writes the values for the analog output channels.
    The values are 16-bit unsigned numbers.  Both read and write will result
    in a control pipe stall if an output scan is running.  The equation for the
    output voltage is:

             ( value - 2^15 )
    V_out = -----------------  * V_ref
                 2^15

     were value is the value written to the channel and V_ref = 10.0V.
  */

  double dvalue;
  uint16_t value;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (channel > 2) {
    printf("usbAOut_USB1608GX_2AO: channel must be 0 or 1.\n");
    return;
  }

  /* correct voltage */
  dvalue = (voltage/10.*32768. + 32768.);
  dvalue = dvalue*table_AO[channel][0] + table_AO[channel][1];

  if (dvalue > 0xffff) {
    value = 0xffff;
  } else if (dvalue < 0.0) {
    value = 0x0;
  } else {
    value = rint(dvalue);
  }
  libusb_control_transfer(udev, requesttype, AOUT, value, channel, NULL, 0x0, HS_DELAY);
}

void usbAOutR_USB1608GX_2AO(libusb_device_handle *udev, uint8_t channel, double *voltage, float table_AO[NCHAN_AO_1608GX][2])
{
  uint16_t value[4];
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  
  libusb_control_transfer(udev, requesttype, AOUT, 0x0, 0x0, (unsigned char *) value, sizeof(value), HS_DELAY);
  *voltage = ((double)(value[channel] - table_AO[channel][1])) / (double) table_AO[channel][0];
  *voltage = (*voltage - 32768.)*10./32768.;
}

void usbAOutScanStop_USB1608GX_2AO(libusb_device_handle *udev)
{
  /* This command stops the analog output scan (if running). */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AOUT_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAOutScanClearFIFO_USB1608GX_2AO(libusb_device_handle *udev)
{
  /* This command clears any remaining output FIFO data after a scan */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AOUT_CLEAR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAOutScanStart_USB1608GX_2AO(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options)
{
  /*
    This command starts the analog output channel scan.  This command
    will result in a bus stall if an AOutScan is currently running.

    count:        the total number of scans to perform (0 = continuous mode)
    retrig_count: the number of scans to perform for each trigger in
                  retrigger mode
    frequency:    pacer frequency (0 for AO_CLK_IN)
    options:      bit 0: 1 = include channel 0 in output scan
                  bit 1: 1 = include channel 1 in output scan
                  bit 2: reserved
                  bit 3: reserved
                  bit 4: 1 = use trigger
                  bit 5: 1 = retirgger mode, 0 = normal trigger
		  bit 6: reserved
		  bit 7: reserved
    Notes:
		  
    The output scan operates with the host continuously transferring
    data for the outputs until the end of the scan.  If the "count"
    parameter is 0, the scan will run until the AOutScanStop command
    is issued by the host; if it is nonzero, the scan will stop
    automatically after the specified number of scans have been
    output.  The channels in the scan are selected in the options bit
    field.  Scan refer to the number of updates to the channels (if
    both channels are used, one scan is an update to both channels).

    The time base is controlled by an internal 32-bit timer running at
    a base rate of 64 MHz.  The timer is controlled by pacer_period.  
    The equation for calculating pacer_period is:

        pacer_period = (64 MHz / sample_frequency) - 1

    The same time base is used for all channels when the scan involved
    multiple channels.  The output data is to be sent using the bulk
    out endpoint.  The data must be in the format:

      low channel sample 0 : [high channel sample 0]
      low channel sample 1 : [high channel sample 1]
      ...
      low channel sample n : [high channel sample n]

    The output is written to an internal FIFO.  The bulk endpoint data
    is only accepted if there is room in the FIFO.  Output data bay be
    sent to the FIFO before the start of the scan, and the FIFO is
    cleared when the AOutScanClearFIFO command is received.  The scan
    will not begin until the AOutScanStart command is sent (and outupt
    data is in the FIFO).  Data will be output until reaching the
    specified number of scans (in single execution mode) or an
    AOutScanStrop command is sent.
  */  

  struct AOutScan_t {
    uint32_t count;         // The total number of scans to perform.  0 = run forever.
    uint32_t retrig_count;  // The number of scans to perform for each trigger in retrigger mode.
    uint32_t pacer_period;  // Pacer timer period value (0 for AO_CLK_IN)
    uint8_t options;
  } AOutScan;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  
  AOutScan.pacer_period = (BASE_CLOCK / frequency) - 1;
  AOutScan.count = count;
  AOutScan.retrig_count = retrig_count;
  AOutScan.options = options;
  
  libusb_control_transfer(udev, requesttype, AOUT_SCAN_START, 0x0, 0x0, (unsigned char *) &AOutScan, sizeof(AOutScan), HS_DELAY);
}


/***********************************************
 *            Counter/Timer                    *
 ***********************************************/
void usbCounterInit_USB1608G(libusb_device_handle *udev, uint8_t counter)
{
  /*
    This command initializes the 32-bit event counter.  On a write, the
    specified counter (0 or 1) will be reset to zero.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, COUNTER, counter, 0x0, NULL, 0x0, HS_DELAY);
  return;
}

uint32_t usbCounter_USB1608G(libusb_device_handle *udev, uint8_t counter)
{
  /*
    This command reads the 32-bit event counter.  
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t counts[2] = {0x0, 0x0};

  libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, (unsigned char *) &counts, sizeof(counts), HS_DELAY);
  if (counter == COUNTER0) {
    return counts[0];
  } else {
    return counts[1];
  }
}

void usbTimerControlR_USB1608G(libusb_device_handle *udev, uint8_t *control)
{
  /*
    This command reads/writes the timer control register
    control:   bit 0:    1 = enable timer,  0 = disable timer
               bit 1:    1 = timer running, 0 = timer stopped
                         (read only, useful when using count)
               bit 2:    1 = inverted output (active low)
	                 0 = normal output (active high)
	       bits 3-7: reserved
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TIMER_CONTROL, 0x0, 0x0, (unsigned char *) control, sizeof(control), HS_DELAY);
}

void usbTimerControlW_USB1608G(libusb_device_handle *udev, uint8_t control)
{
  /* This command reads/writes the timer control register */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TIMER_CONTROL, control, 0x0, NULL, 0x0, HS_DELAY);
}

void usbTimerPeriodR_USB1608G(libusb_device_handle *udev, float *period)
{
  /*
    The timer is based on a 64 MHz input clock and has a 32-bit period register. The
    frequency of the output is set to:

          frequency = 64 MHz / (period + 1)

    Note that the value for pulseWidth should always be smaller than the value for
    the period register or you may get unexpected results.  This results in a minimum
    allowable value for the period of 1, which sets the maximum frequency to 64 MHz/2.

    period: timer period in ms
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t value;
  
  if (libusb_control_transfer(udev, requesttype, TIMER_PERIOD, 0x0, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY) < 0) {
    perror("usbTimerPeriodR_USB1608G error");
  }
  *period = (value + 1)*1000./BASE_CLOCK;
}    

void usbTimerPeriodW_USB1608G(libusb_device_handle *udev, float period)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t value;
  uint16_t wValue;
  uint16_t wIndex;

  value = rint(period*BASE_CLOCK/1000. - 1);
  wValue = value & 0xffff;
  wIndex = (value >> 16) & 0xffff;
  if (libusb_control_transfer(udev, requesttype, TIMER_PERIOD, wValue, wIndex, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbTimerPeriodW_USB_1608G error.");    
  }
}

void usbTimerPulseWidthR_USB1608G(libusb_device_handle *udev, float *pulseWidth)
{
  /*
    This command reads/writes the timer pulse width register.
    The timer is based on a 40 MHz input clock and has a 32-bit pulse width register.
    The width of the output pulse is set to:

    pulse width = (pulseWidth + 1) / 40 MHz

    Note that the value for pulseWidth should always be smaller than the value for
    the period register or you may get unexpected results.
    pulseWidth: the pulse width in ms
  */
  
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t pulse_width;
  
  if (libusb_control_transfer(udev, requesttype, TIMER_PULSE_WIDTH, 0x0, 0x0, (unsigned char *) &pulse_width, sizeof(pulse_width), HS_DELAY) < 0) {
    perror("usbTimerPulseWidthR_USB1608G error.");
  }
  *pulseWidth = (pulse_width + 1)*1000./BASE_CLOCK;
}

void usbTimerPulseWidthW_USB1608G(libusb_device_handle *udev, float pulseWidth)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t pulse_width;
  uint16_t wValue;
  uint16_t wIndex;

  pulse_width = rint(pulseWidth*BASE_CLOCK/1000. - 1);
  wValue = pulse_width & 0xffff;
  wIndex = (pulse_width >> 16) & 0xffff;
  if (libusb_control_transfer(udev, requesttype, TIMER_PULSE_WIDTH, wValue, wIndex, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbTimerPulseWidthW_USB1608G: error.");
  }
}

void usbTimerCountR_USB1608G(libusb_device_handle *udev, uint32_t *count)
{
  /*
    This command reads/writes the timer count register.
    The number of output pulses can be controlled with the count register.  Setting
    this register to 0 will result in pulses being generated until the timer is disabled.
    Setting it to a non-zero value will results in the specified number of pulses being
    generated then the output will go low until the timer is disabled.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (libusb_control_transfer(udev, requesttype, TIMER_COUNT, 0x0, 0x0, (unsigned char *) count, sizeof(count), HS_DELAY) < 0) {
    perror("usbTimerCountR_USB1608G error.");
  }
}

void usbTimerCountW_USB1608G(libusb_device_handle *udev, uint32_t count)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t wValue = count & 0xffff;
  uint16_t wIndex = (count >> 16) & 0xffff;
  if (libusb_control_transfer(udev, requesttype, TIMER_COUNT, wValue, wIndex, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbTimerCountW_USB1608G error.");
  }
}

void usbTimerDelayR_USB1608G(libusb_device_handle *udev, float *delay)
{
  /*
     This command reads/writes the timer start delay register.  This
     register is the amount of time to delay before starting the timer
     output after enabling the output.  The value specifies the number
     of 40 MHZ clock pulses to delay.  This value may not be written
     while the timer output is enabled.

     Note: delay is in ms
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t value;
    
  if (libusb_control_transfer(udev, requesttype, TIMER_START_DELAY, 0x0, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY) < 0) {
    perror("usbTimerDelayR_USB1608G: error.");
  }
  *delay = value*1000./BASE_CLOCK;
}

void usbTimerDelayW_USB1608G(libusb_device_handle *udev, float delay)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t value;
  uint16_t wValue;
  uint16_t wIndex;
  
  value = rint(delay*BASE_CLOCK/1000.);
  wValue = value & 0xffff;
  wIndex = (value >> 16) & 0xffff;
  if (libusb_control_transfer(udev, requesttype, TIMER_START_DELAY, wValue, wIndex, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbTimerDelayW_USB1608G: error.");
  }
}

void usbTimerParamsR_USB1608G(libusb_device_handle *udev, timerParams *params)
{
  /*
    This command reads/writes all timer parameters in one call.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TIMER_PARAMETERS, 0x0, 0x0, (unsigned char *) params, sizeof(timerParams), HS_DELAY);
}

void usbTimerParamsW_USB1608G(libusb_device_handle *udev, timerParams *params)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TIMER_PARAMETERS, 0x0, 0x0, (unsigned char *) params, sizeof(timerParams), HS_DELAY);
}

/***********************************************
 *            Memory Commands                  *
 ***********************************************/
void usbMemoryR_USB1608G(libusb_device_handle *udev, uint8_t *data, uint16_t length)
{
  /*
    This command reads or writes data from the EEPROM memory.  The
    read will begin at the current address, which may be set with
    MemAddress.  The address will automatically increment during a
    read or write but stay within the range allowed for the EEPROM.
    The amount of data to be written or read is specified in wLength.

    The range from 0x0000 to 0x3FFF is used for storing the
    microcontroller firmware and is write-protected during normal
    operation.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  ret = libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
  if (ret != length) {
    perror("usbMemoryR_USB1608G: error in reading memory.");
  }
}

void usbMemoryW_USB1608G(libusb_device_handle *udev, uint8_t *data, uint16_t length)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
}

void usbMemAddressR_USB1608G(libusb_device_handle *udev, uint16_t address)
{
  /*
    This command reads or writes the address used for memory accesses.
    The upper byte is used to denominate different memory areas.  The
    memory map for this device is

       Address                            Description
    =============               ============================
    0x0000-0x3FFF               Microcontroller firmware (write protected)
    0x4000-0x40FF               Calibration storage      (write protected)
    0X4100-0X7FFF               User data

    The firmware area is protected by a separate command so is not typically
    write-enabled.  The calibration area is unlocked by writing the value 0xAA55
    to address 0x8000.  The area will remain unlocked until the device is reset
    or a value other than 0xAA55 is written to address 0x8000.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY);
}

void usbMemAddressW_USB1608G(libusb_device_handle *udev, uint16_t address)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY);
}

void usbMemWriteEnable_USB1608G(libusb_device_handle *udev)
{
  /*
    This command enables writes to the EEPROM memory in the range
    0x0000-0x3FFF.  This command is only to be used when updating the
    microcontroller firmware.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t unlock_code = 0xad;
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &unlock_code, sizeof(unlock_code), HS_DELAY);
}

/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/
  
/* blinks the LED of USB device */
void usbBlink_USB1608G(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return;
}

uint16_t usbStatus_USB1608G(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t status = 0x0;

  libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY);
  return status;
}

void usbReset_USB1608G(libusb_device_handle *udev)
{
  /*
    This function causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

void usbTriggerConfig_USB1608G(libusb_device_handle *udev, uint8_t options)
{
  /*
    This function configures the AInScan trigger.  Once the trigger is
    received, the AInScan will proceed as configured.  The "use
    trigger" option must be used in the AInScanStart command to
    utilize this feature.

    options:     bit 0: trigger mode (0 = level,  1 = edge)
                 bit 1: trigger polarity (0 = low / falling, 1 = high / rising)
                 bits 2-7: reserved
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, (unsigned char*) &options, sizeof(options), HS_DELAY);
}

void usbTriggerConfigR_USB1608G(libusb_device_handle *udev, uint8_t *options)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, (unsigned char*) options, sizeof(options), HS_DELAY);
}

void usbTemperature_USB1608G(libusb_device_handle *udev, float *temperature)
{
  /*
    This command reads the internal temperature.  The temperature in degrees
    Celsius is calculated as:

     T = 128.(value/2^15)
  */

  int16_t temp;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  ret =  libusb_control_transfer(udev, requesttype, TEMPERATURE, 0x0, 0x0, (unsigned char*) &temp, sizeof(temp), HS_DELAY);
  if (ret < 0) {
    perror("usbTemperature_USB1608G: error in reading temperature.");
  }
  *temperature = temp/256.0;
}

void usbGetSerialNumber_USB1608G(libusb_device_handle *udev, char serial[9])
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

/***********************************************
 *          FPGA Commands                      *
 ***********************************************/

void usbFPGAConfig_USB1608G(libusb_device_handle *udev)
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

void usbFPGAData_USB1608G(libusb_device_handle *udev, uint8_t *data, uint8_t length)
{
  /*
    This command writes the FPGA configuration data to the device.  This
    command is not accepted unless the device is in FPGA config mode.  The
    number of bytes to be written must be specified in wLength.

    data: max length is 64 bytes
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (length > 64) {
    printf("usbFPGAData_USB1608G: max length = 64 bytes\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, FPGA_DATA, 0x0, 0x0, (unsigned char*) data, length, HS_DELAY);
}

void usbFPGAVersion_USB1608G(libusb_device_handle *udev, uint16_t *version)
{
  /*
    This command reads the FPGA version.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, FPGA_VERSION, 0x0, 0x0, (unsigned char *) version, sizeof(uint16_t), HS_DELAY);
}

void cleanup_USB1608G( libusb_device_handle *udev )
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|6);
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT|2);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}

uint16_t voltsTou16_USB1608GX_AO(double volts, int channel, float table_AO[NCHAN_AO_1608GX][2])
{
  double dvalue;
  uint16_t value;

  if (channel > 2) {
    printf("voltsTou16_USB1608GX_AO: channel must be between 0 and 2.\n");
    return -1;
  }

  /* correct voltage */
  dvalue = (volts/10.*32768. + 32678);
  dvalue = dvalue*table_AO[channel][0] + table_AO[channel][1];

  if (dvalue > 0xffff) {
    value = 0xffff;
  } else if (dvalue < 0.0) {
    value = 0x0;
  } else {
    value = rint(dvalue);
  }

  return value;
}

double volts_USB1608G(const uint8_t gain, uint16_t value)
{

  double volt = 0.0;

  switch (gain) {
    case BP_10V:
      volt = (value - 32768.)*10./32768.;
      break;
    case BP_5V:
      volt = (value - 32768.)*5./32768.;
      break;
    case BP_2V:
      volt = (value - 32768.)*2./32768.;
      break;
    case BP_1V:
      volt = (value - 32768.)*1./32768; 
      break;
  }
  return volt;
}
