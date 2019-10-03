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
#include "usb-2020.h"
#include "usb-2020.rbf"

/* Commands and  Report ID for the USB 2020 */
/* Digital I/O Commands */
#define DTRISTATE        (0x00) // Read/Write Tristate register
#define DPORT            (0x01) // Read digital port pins
#define DLATCH           (0x02) // Read/Write Digital port output latch register

/* Analog Input Commands */
#define AIN              (0x10) // Read analog input channel
#define AIN_SCAN_START   (0x12) // Start input scan
#define AIN_SCAN_STOP    (0x13) // Stop input scan
#define AIN_CONFIG       (0x14) // Analog input channel configuration
#define AIN_CLR_FIFO     (0x15) // Clear firmware analog input FIFO

/* Memory Commands */
#define MEMORY           (0x30) // Read/Write EEPROM
#define MEM_ADDRESS      (0x31) // EEPROM read/write address value
#define MEM_WRITE_ENABLE (0x32) // Enable writes to firmware area

/* Miscellaneous Commands */  
#define STATUS           (0x40) // Read device status
#define BLINK_LED        (0x41) // Causes LED to blink
#define RESET            (0x42) // Reset device
#define TRIGGER_CONFIG   (0x43) // External trigger configuration
#define CAL_CONFIG       (0x44) // Calibration configuration
#define TEMPERATURE      (0x45) // Read internal temperature
#define SERIAL           (0x48) // Read/Write USB Serial Number

/* FPGA Configuration Commands */
#define FPGA_CONFIG      (0x50) // Start FPGA configuration
#define FPGA_DATA        (0x51) // Write FPGA configuration data
#define FPGA_VERSION     (0x52) // Read FPGA version

#define HS_DELAY 2000

static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant. 

void usbBuildGainTable_USB2020(libusb_device_handle *udev, float table[NGAINS_2020][2])
{
  /* 
     Builds a lookup table of calibration coefficents to translate values into voltages:
       voltage = value*table[gain#][0] + table[gain#][1]
     only needed for fast lookup.
  */

  int i, j;
  uint16_t address = 0x7000;

  usbMemAddressW_USB2020(udev, address);  // Beginning of Calibration Table
  for (i = 0; i < NGAINS_2020; i++) {
    for (j = 0; j < 2; j++) {
      usbMemoryR_USB2020(udev, (uint8_t *) &table[i][j], sizeof(float));
    }
  }
  return;
}

void usbInit_USB2020(libusb_device_handle *udev)
{
  int i;
  /* This function does the following:
     1. Configure the FPGA
     2. Finds the maxPacketSize for bulk transfers
  */

  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
  if (wMaxPacketSize < 0) {
    perror("usbInit_USB2020: error in getting wMaxPacketSize");
  }

  if (!(usbStatus_USB2020(udev) & FPGA_CONFIGURED)) {
    usbFPGAConfig_USB2020(udev);
    if (usbStatus_USB2020(udev) & FPGA_CONFIG_MODE) {
      for (i = 0; i <= (sizeof(FPGA_data) - 64); i += 64) {
	usbFPGAData_USB2020(udev, &FPGA_data[i], 64);
      }
      if (sizeof(FPGA_data) % 64) {
	usbFPGAData_USB2020(udev, &FPGA_data[i], sizeof(FPGA_data)%64);
      }
      if (!(usbStatus_USB2020(udev) & FPGA_CONFIGURED)) {
	printf("Error: FPGA for the USB-2020 is not configured.  status = %#x\n", usbStatus_USB2020(udev));
	return;
      }
    } else {
      printf("Error: could not put USB-2020 into FPGA Config Mode.  status = %#x\n", usbStatus_USB2020(udev));
      return;
    }
  } else {
    printf("USB-2020 FPGA configured.\n");
    return;
  }
}

void usbCalDate_USB2020(libusb_device_handle *udev, struct tm *date)
{
  /* This command reads the factory calibration date */

  calibrationTimeStamp calDate;
  uint16_t address = 0x7098;  // beginning of MFG Calibration date
  time_t time;

  usbMemAddressW_USB2020(udev, address);  // Beginning of Calibration Date
  usbMemoryR_USB2020(udev, (uint8_t *) &calDate, sizeof(calDate));
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

uint16_t usbDTristateR_USB2020(libusb_device_handle *udev)
{
  /* This command reads or writes the digital port tristate register.
   The tristate register determines if the latch register value is driven onto
   the port pin.  A '1' in the tristate register makes the corresponding
   pin an input, a '0' makes it an output.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDTristateR_USB2020: error in libusb_control_transfer().");
  }
  return data;
}

void usbDTristateW_USB2020(libusb_device_handle *udev, uint16_t value)
{
  /* This command reads or writes the digital port tristate register.
   The tristate register determines if the latch register value is driven onto
   the port pin.  A '1' in the tristate register makes the corresponding
   pin an input, a '0' makes it an output.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, value, 0x0, (unsigned char *) NULL, 0x0, HS_DELAY) < 0) {
    perror("usbDTristateW_USB2020: error in usb_control_msg().");
  }
  return;
}

/* reads digital word  */
uint16_t usbDPort_USB2020(libusb_device_handle *udev)
{
  /*
    This command reads the current state of the digital pins.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data;

  if (libusb_control_transfer(udev, requesttype, DPORT, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDPort_USB2020: error in libusb_control_transfer().");
  }
  return data;
}

/* read/writes digital port latch */
uint16_t usbDLatchR_USB2020(libusb_device_handle *udev)
{
  /*
    This command reads the digital port latch register
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DLATCH, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDLatchR_USB2020: error in libusb_control_transfer().\n");
  }
  return data;
}

void usbDLatchW_USB2020(libusb_device_handle *udev, uint16_t value)
{
  /*
    This command writes the digital port latch register
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DLATCH, value, 0x0, (unsigned char *) NULL, 0x0, HS_DELAY) < 0) {
    perror("usbDLatchW_USB2020: error in libusb_control_transfer()");
  }
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/

uint16_t usbAIn_USB2020(libusb_device_handle *udev, uint16_t channel)
{
  /*
    This command returns the  value from an analog input channel.  This
    command will result in a bus stall if AInScan is currently running.
  */
  uint16_t value;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AIN, channel, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY) < 0) {
    perror("usbAIN_USB2020: error in libusb_control_transfer()");
  }
  return value;
}

void usbAInScanStart_USB2020(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency,
			       uint32_t packet_size, uint8_t options)
{
  /* This command starts the analog input channel scan.  The gain
     ranges that are currently set on the desired channels will be
     used (these may be changed with AInConfig) This command will
     result in a bus stall if an AInScan is currently running.

     Notes:

     The pacer rate is set by an internal 32-bit incrementing timer
     running at a base rate of 80 MHz.  The timer is controlled by
     pacer_period. The maximum rate on this ADC is 20 MHz.

     If pacer_period is set to 0 the device does not generate an A/D clock.  It uses
     PACER_IN pin as the pacer source.

     The timer will be reset and sample acquired when its value equals
     timer_period.  The equation for calculating timer_period is:

     timer_period = [80MHz / (sample frequency)] - 1

     The data will be returned in packets utilizing a bulk IN endpoint.
     The data will be in the format:

     lowchannel sample 0: lowchannel + 1 sample 0: ... :hichannel sample 0
     lowchannel sample 1: lowchannel + 1 sample 1: ... :hichannel sample 1
     ...
     lowchannel sample n: lowchannel + 1 sample n: ... :hichannel sample n

     The scan will not begin until the AInScanStart command is sent (and
     any trigger conditiions are met.)  Data will be sent until reaching
     the specified count or an AInScanStop command is sent.

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
    uint8_t options;       /* bit 0:  Reserved
                              bit 1:  Reserved
	                      bit 2:  Reserved
                              bit 3:  1 = use trigger or gate
                              bit 4:  Reserved
			      bit 5:  1 = External Pacer Output, 0 = External Pacer Input
			      bit 6:  1 = retrigger mode, 0 = normal trigger
			      bit 7:  1 = Use DDR RAM as storage, 0 = Stream via USB
		           */
    uint8_t pad[2];
  } AInScan;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t status;

  if (count == 0) packet_size = 255;

  AInScan.count = count;
  AInScan.retrig_count = retrig_count;
  if (frequency == 0.0) {
    AInScan.pacer_period = 0;
  } else {
    AInScan.pacer_period = rint((80.E6 / frequency) - 1);
  }
  if (packet_size > wMaxPacketSize/2 - 1) packet_size = wMaxPacketSize/2 - 1;
  AInScan.packet_size = (uint8_t) packet_size;
  AInScan.options = options;
  if (options & DDR_RAM) {
    /* If using the onboard DDR RAM (BURSTIO Mode), there are 3 constraints:
       1. The total count must be greater than or equal to 256
       2. The total count must be less than 64 MB
       3. The total count must be a multiple of 256
    */
    if (count < 256) {
      printf("usbAInScanStart_USB2020: count must be greater than or equal to 256.\n");
      return;
    }
    if (count > 64*1024*1024) {
      printf("usbAInScanStart_USB2020: count must be less than 64MB.\n");
      return;
    }
    if (count % 256 ) {
      printf("usbAInScanStart_USB2020: count must be a multiple of 256.\n");
      return;
    }
  }

  status = usbStatus_USB2020(udev);
  if (!(status & AIN_SCAN_DONE)) {
    printf("Analog In scan not done.\n");
    usbAInScanStop_USB2020(udev);
  }

  /* Pack the data into 14 bytes */
  libusb_control_transfer(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (unsigned char *) &AInScan, 14, HS_DELAY);
}

int usbAInScanRead_USB2020(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data, unsigned int timeout, int options)
{
  /*
    Bulk transactions will be considered complete when a packet is sent
    that is less than the max packet size.  If an integer muliple of the
    max packet size of data is to be sent during a transaction, an empty
    packet must be sent to indicate the end of the transaction.
  */

  char value[MAX_PACKET_SIZE_HS];
  int ret = -1;
  int nbytes = nChan*nScan*2;    // number of bytes to read;
  int transferred;
  uint8_t status;

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) data, nbytes, &transferred, timeout);

  if (ret  < 0) {
    perror("usbAInScanRead_USB2020: error in libusb_bulk_transferred (1).");
  }
  if (transferred != nbytes) {
    fprintf(stderr, "usbAInScanRead_USB2020: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
    status = usbStatus_USB2020(udev);
    if ((status & AIN_SCAN_OVERRUN)) {
      printf("usbAInScanRead: Analog In scan overrun.\n");
    }
    return ret;
  }

  if (options & CONTINUOUS) return transferred;
  
  status = usbStatus_USB2020(udev);
  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.

  if ((nbytes%wMaxPacketSize) == 0 && !(status & AIN_SCAN_RUNNING)) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) value, 2, &ret, 100);
  }

  if ((status & AIN_SCAN_OVERRUN)) {
    printf("usbAInScanRead: Analog In scan overrun.\n");
  }

  return transferred ;
}

void usbAInConfig_USB2020(libusb_device_handle *udev, ScanList scanList[NCHAN_2020])
{
  /*
    This command reads or writes the analog input channel
    configurations.  This command will result in a bus stall if an
    AInScan is currently running.

    Scan List: channel configuration
            bit   0: Channel Number
            bit 1-2: Range
            bit 3: Last Channel: 1 = last channel, 0 = not last channel
            bit 4: Calibratin Mode: 1 = Calibration voltage reference is input to ADC
                                    0 = Channel input is tied to ADC

    range:  0: +/- 10V range
            1: +/- 5V range
            2: +/- 2V range
	    3: _/- 1V range
  */

  int i;
  static uint8_t Scan_list[2];
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  for (i = 0; i < NCHAN_2020; i++) {
    if (scanList[i].channel >= 0 && scanList[i].channel < NCHAN_2020) {
      Scan_list[i] = ((scanList[i].channel & 0x1)           |
		      ((scanList[i].range  & 0x3) << 0x1)   |
		      ((scanList[i].mode   & 0x1) << 0x4));
    } else {
      printf("Error in Scan List[%d]  mode = %#x   channel = %d  range = %#x\n",
	     i, scanList[i].mode, scanList[i].channel, scanList[i].range);
      return;
    }
    
    if (scanList[i].mode & LAST_CHANNEL) {
      Scan_list[i] |= (0x1 << 3);
      break;
    }
  }

  if (usbStatus_USB2020(udev) | AIN_SCAN_RUNNING) {
    usbAInScanStop_USB2020(udev);
  }
  libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, i, (unsigned char *) &Scan_list[0], sizeof(Scan_list), HS_DELAY);
}

void usbAInConfigR_USB2020(libusb_device_handle *udev, ScanList scanList[NCHAN_2020])
{
  static uint8_t Scan_list[NCHAN_2020];
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (usbStatus_USB2020(udev) | AIN_SCAN_RUNNING) {
    usbAInScanStop_USB2020(udev);
  }
  
  libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (unsigned char *) Scan_list, sizeof(Scan_list), HS_DELAY);
}

void usbAInScanStop_USB2020(libusb_device_handle *udev)
{
  /*
    This command stops the analog input scan (if running).
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, (unsigned char *) NULL, 0x0, HS_DELAY);
}

void usbAInScanClearFIFO_USB2020(libusb_device_handle *udev)
{
  /*
    The command clears the analog input firmware buffer.
   */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_CLR_FIFO, 0x0, 0x0, (unsigned char *) NULL, 0x0, HS_DELAY);
}

/***********************************************
 *            Memory Commands                  *
 ***********************************************/
void usbMemoryR_USB2020(libusb_device_handle *udev, uint8_t *data, uint16_t length)
{
  /*
    This command reads or writes data from the EEPROM memory.  The
    read will begin at the current address, which may be set with
    MemAddress.  The address will automatically increment during a
    read or write but stay within the range allowed for the EEPROM.
    The amount of data to be written or read is specified in wLength.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  ret = libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
  if (ret != length) {
    printf("usbMemoryR_USB2020: error in reading memory\n");
  }
}

void usbMemoryW_USB2020(libusb_device_handle *udev, uint8_t *data, uint16_t length)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
}

void usbMemAddressR_USB2020(libusb_device_handle *udev, uint16_t address)
{
  /*
    This command reads or writes the address used for memory accesses.
    The upper byte is used to denominate different memory areas.  The
    memory map for this device is:

       Address                            Description
    =============               ============================
    0x0000-0x6FFF               Microcontroller firmware (write protected)
    0x7000-0x70FF               Calibration storage      (write protected)
    0X7100-0X7FFF               User data

    The firmware area is protected by a separate command so is not typically
    write-enabled.  The calibration area is unlocked by writing the value 0xAA55
    to address 0x8000.  The area will remain unlocked until the device is reset
    or a value other than 0xAA55 is written to address 0x8000.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY);
}

void usbMemAddressW_USB2020(libusb_device_handle *udev, uint16_t address)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY);
}

void usbMemWriteEnable_USB2020(libusb_device_handle *udev)
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
 *          Miscellaneous Commands             *
 ***********************************************/
  
/* blinks the LED of USB device */
void usbBlink_USB2020(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return;
}

uint16_t usbStatus_USB2020(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t status = 0x0;

  libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY);
  return status;
}

void usbReset_USB2020(libusb_device_handle *udev)
{
  /*
    This function causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

void usbTriggerConfig_USB2020(libusb_device_handle *udev, TriggerConfig *triggerConfig)
{
  /*
    This function configures the AInScan trigger or gating function.
    Once the trigger is received, the AInScan will proceed as
    configured.  If gating is used, the scan will only contginue when
    the gate condition is met.  The HIGH and LOW thresholds will need
    to be in uncalibrated counts. The "use trigger" option must be
    used in the AInScanStart command to utilize this feature.

    options:     bit    0: Trigger Type (0 = Trigger,  1 = Gate)
                 bit    1: Trigger Source (0 = Digital, 1 = Analog)
                 bits 2-3: Mode (00 = level, 01 = edge, 10 = Hysteresis, 11 = Window)
                 bit    4: Polarity (0 = low/falling/negative/out, 1 = high/rising/positive/in)
                 bits 5-7: Reserved
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, 
		  (unsigned char *) triggerConfig, sizeof(triggerConfig), HS_DELAY);
}

void usbTriggerConfigR_USB2020(libusb_device_handle *udev, TriggerConfig *triggerConfig)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, 
		  (unsigned char *) triggerConfig, sizeof(triggerConfig), HS_DELAY);
}

void usbTemperature_USB2020(libusb_device_handle *udev, float *temperature)
{
  /*
    This command reads the internal temperature.  The temperature in degrees
    Celsius is calculated as:

     T = 128.(value/2^15)
  */

  int16_t temp = 0x1234;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  ret =  libusb_control_transfer(udev, requesttype, TEMPERATURE, 0x0, 0x0, (unsigned char *) &temp, sizeof(temp), HS_DELAY);
  if (ret < 0) {
    printf("usbTemperature_USB2020: error in reading temperature.  Error = %d\n", ret);
  }
  *temperature = temp/256.0;
}

void usbGetSerialNumber_USB2020(libusb_device_handle *udev, char serial[9])
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

void usbFPGAConfig_USB2020(libusb_device_handle *udev)
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
  libusb_control_transfer(udev, requesttype, FPGA_CONFIG, 0x0, 0x0, (unsigned char *) &unlock_code, sizeof(unlock_code), HS_DELAY);
}

void usbFPGAData_USB2020(libusb_device_handle *udev, uint8_t *data, uint8_t length)
{
  /*
    This command writes the FPGA configuration data to the device.  This
    command is not accepted unless the device is in FPGA config mode.  The
    number of bytes to be written must be specified in wLength.

    data: max length is 64 bytes
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (length > 64) {
    printf("usbFPGAData_USB2020: max length = 64 bytes\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, FPGA_DATA, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
}

void usbFPGAVersion_USB2020(libusb_device_handle *udev, uint16_t *version)
{
  /*
    This command reads the FPGA version.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, FPGA_VERSION, 0x0, 0x0, (unsigned char *) version, sizeof(uint16_t), HS_DELAY);
}

void cleanup_USB2020( libusb_device_handle *udev )
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|1);
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT|1);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}

double volts_USB2020(const uint8_t gain, uint16_t value)
{
  double volt = 0.0;

  switch (gain) {
    case BP_10V:
      volt = (value - 2048.)*10./2048.;
      break;
    case BP_5V:
      volt = (value - 2048.)*5./2048.;
      break;
    case BP_2V:
      volt = (value - 2048.)*2./2048.;
      break;
    case BP_1V:
      volt = (value - 2048.)*1./2048.; 
      break;
  }
  return volt;
}
