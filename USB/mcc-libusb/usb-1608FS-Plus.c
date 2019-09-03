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
#include <math.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-1608FS-Plus.h"

/* Commands and USB Report ID for the USB 1608FS-Plus */
/* Digital I/O Commands */
#define DTRISTATE         (0x00) // Read/Write Tristate register
#define DPORT             (0x01) // Read digital port pins
#define DLATCH            (0x02) // Read/Write Digital port output latch register

/* Analog Input Commands */
#define AIN               (0x10) // Read analog input channel
#define AIN_SCAN_START    (0x11) // Start analog input scan
#define AIN_SCAN_STOP     (0x12) // Stop analog input scan
#define AIN_CONFIG        (0x14) // Analog input channel configuration
#define AIN_CLR_FIFO      (0x15) // Clear the analog input scan FIFO

/* Counter/Timer Commands */
#define COUNTER           (0x20) // Read/reset event counter

/* Memory Commands */
#define CAL_MEMORY        (0x30) // Read/Write Calibration Memory
#define USER_MEMORY       (0x31) // Read/Write User Memory
#define MBD_MEMORY        (0x32) // Read/Write MBD Memory

/* Miscellaneous Commands */  
#define BLINK_LED         (0x41) // Causes LED to blink
#define RESET             (0x42) // Reset device
#define STATUS            (0x44) // Read device status
#define SERIAL            (0x48) // Read/Write serial number
#define DFU               (0x50) // Enter device firmware upgrade mode

/* MBD */
#define MBD_COMMAND       (0x80) // Text-based MBD command / response
#define MBD_RAW           (0x81) // Raw MBD response

#define HS_DELAY 2000

static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant. 

void usbBuildGainTable_USB1608FS_Plus(libusb_device_handle *udev, float table[NGAINS_USB1608FS_PLUS][NCHAN_USB1608FS_PLUS][2])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
     The calibration coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  int i, j, k;
  uint16_t address = 0x0;

  for (i = 0; i < NGAINS_USB1608FS_PLUS; i++ ) {
    for (j = 0; j < NCHAN_USB1608FS_PLUS; j++) {
      for (k = 0; k < 2; k++) {
	usbReadCalMemory_USB1608FS_Plus(udev, address, 4, (uint8_t *) &table[i][j][k]);
	address += 4;
      }
    }
  }

  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
}

void usbCalDate_USB1608FS_Plus(libusb_device_handle *udev, struct tm *date)
{
  /* This command reads the factory calibration date */

  calibrationTimeStamp calDate;
  uint16_t address = 0x200;  // beginning of MFG Calibration date
  time_t time;

  usbReadCalMemory_USB1608FS_Plus(udev, address, sizeof(calDate), (uint8_t *) &calDate);
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
uint8_t  usbDTristateR_USB1608FS_Plus(libusb_device_handle *udev)
{
  /*
    This command reads or writes the digital port tristate
    register.  The tristate register determines if the
    latch register value is driven onto the port pin.  A
    '1' in the tristate register makes the corresponding
    pin an input, a '0' makes it an output.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDTristateR_USB1608FS_Plus: error in libusb_control_transfer().\n");
  }
  return data;
}

void usbDTristateW_USB1608FS_Plus(libusb_device_handle *udev, uint8_t value)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    printf("usbDTristateW_USB1608FS_Plus: error in libusb_control_transfer().\n");
  }
  return;
}

/* reads digital port  */
uint8_t usbDPort_USB1608FS_Plus(libusb_device_handle *udev)
{
  /*
    This command reads the current state of the digital port pins
    or writes to the latch
   */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data;

  if (libusb_control_transfer(udev, requesttype, DPORT, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDPort_USB1608FS_Plus: error in libusb_control_transfer().\n");
  }
  return data;
}

/* read digital port */
uint8_t usbDLatchR_USB1608FS_Plus(libusb_device_handle *udev)
{
  /*
    This command reads or writes the digital port latch register.  The
    power on default is all 0.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;
  
  if (libusb_control_transfer(udev, requesttype, DLATCH, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDLatchR_USB1608FS_Plus: error in libusb_control_transfer().\n");
  }
  return data;
}

void usbDLatchW_USB1608FS_Plus(libusb_device_handle *udev, uint8_t value)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DLATCH, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    printf("usbDLatchW_USB1608FS_Plus: error in libusb_control_transfer().\n");
  }
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/
uint16_t usbAIn_USB1608FS_Plus(libusb_device_handle *udev, uint8_t channel, uint8_t range)
{
  /*
    This command reads the value of an anlog input channel.  This command will result
    in a bus stall if an AInScan is currently running.
  */
  uint16_t value;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, AIN, channel, range, (unsigned char *) &value, sizeof(value), HS_DELAY);
  return value;
}

void usbAInScanStart_USB1608FS_Plus(libusb_device_handle *udev, uint32_t count, double frequency, uint8_t channels, uint8_t options)
{
  /*
     The command starts an analog input scan.  The command will result
     in a bus stall if an AInScan is currently running.  The
     USB-1608FS-Plus will not generate an internal pacer faster than
     100 kHz;

     The ADC is paced such that the pacer controls the ADC
     conversions.  The internal pacer rate is set by an internal
     32-bit timer running at a base rate of 40 MHz.  The timer is
     controlled by pacer_period.  This value is the period of the scan
     and the ADCs are clocked at this rate.  A pulse will be output at
     the SYNC pin at every pacer_period interval if SYNC is configred
     as an output.  The equation for calucating pacer_period is:

     pacer_period = [40 MHz / (A/D frequency)] - 1

     If pacer_period is set to 0, the device does not generate an A/D
     clock.  It uses the SYNC pin as an input and the user must
     provide the pacer sourece.  The A/Ds acquire data on every rising
     edge of SYNC; the maximum allowable input frequency is 100 kHz.

     The data will be returned in packets untilizing a bulk endpoint.
     The data will be in the format:

     lowchannel sample 0: lowchannel + 1 sample 0: ... : hichannel sample 0
     lowchannel sample 1: lowchannel + 1 sample 1: ... : hichannel sample 1
     ...
     lowchannel sample n: lowchannel + 1 sample n: ... : hichannel sample n

     The scan will not begin until the AInScan Start command is sent
     and any trigger conditions are met.  Data will be sent until
     reaching the specified count or an usbAInScanStop_USB1608FS_Plus()
     command is sent.

     The external trigger may be used to start the scan.  If enabled,
     the device will wait until the appropriate trigger condition is
     detected than begin sampling data at the specified rate.  No
     packets will be sent until the trigger is detected.

     In block transfer mode, the data is sent in 64-byte packets as
     soon as data is available from the A/D.  In immediate transfer
     mode, the data is sent after each scan, resulting in packets that
     are 1-8 samples (2-16 bytes) long.  This mode should only be used
     for low pacer rates, typically under 100 Hz, because overruns
     will occur if the rate is too high.

     There is a 32,768 sample FIFO, and scans under 32 kS can be
     performed at up to 100 kHz*8 channels without overrun.

     Overruns are indicated by the device stalling the bulk endpoint
     during the scan.  The host may read the status to verify and ust
     clear the stall condition before further scan can be performed.
     
  */
  struct AInScan_t {
    uint32_t count;         // The total number of scans to perform (0 for continuous scan)
    uint32_t pacer_period;  // The pacer timer period value. (0 for external clock).
    uint8_t channels;       // bit field that selects the channels in the scan;
    uint8_t options;     /* bit field that controls scan options:
			  bit 0:   0 = block transfer mode, 1 = immediate transfer mode
                          bit 1:   0 = do not output internal pacer (ignored whn using external clock for pacing).
                                   1 = output internal pacer on SYNC
	                  bit 2-4: Trigger setting:
                                   0: to trigger
                                   1: Edge / rising
                                   2: Edge / falling
                                   3: Level / high
                                   4: Level / low
			  bit 5:  0 = normal A/D data, 1 = debug mode (data returned is incrementing counter)
			  bit 6:  Reserved
			  bit 7:  0 = stall on overrun, 1 = inhibit bulk pipe stall
		         */
    uint8_t pad[2];         //  align along 12 byte boundary
  } AInScan;

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);


  if (frequency > 500000.) frequency = 500000.;
  if (frequency > 0.) {
      AInScan.pacer_period = rint((40.E6 / frequency) - 1);
  } else {
    AInScan.pacer_period = 0;
  }
  AInScan.count = count;
  AInScan.channels = channels;
  AInScan.options = options;

  usbAInScanStop_USB1608FS_Plus(udev);
  usbAInScanClearFIFO_USB1608FS_Plus(udev);
  /* Pack the data into 10 bytes */
  libusb_control_transfer(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (unsigned char *) &AInScan, 10, HS_DELAY);
}

int usbAInScanRead_USB1608FS_Plus(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data, uint8_t options)
{
  int i;
  int ret = -1;
  int nbytes = nChan*nScan*2;    // number of bytes to read in 64 bit chunks
  int transferred;
  uint16_t status = 0;
  char value[MAX_PACKET_SIZE];

  if (options & IMMEDIATE_TRANSFER_MODE) {
    for (i = 0; i < nbytes/2; i++) {
      ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) &data[i], 2, &transferred, 2000);
      if (ret < 0) {
	perror("usbAInScanRead_USB1608FS_Plus: error in usb_bulk_transfer.");
      }
      if (transferred != 2) {
	fprintf(stderr, "usbAInScanRead_USB1608FS_Plus: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
      }
    }
  } else { 
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) data, nbytes, &transferred, HS_DELAY);
    if (ret < 0) {
      perror("usbAInScanRead_USB1608FS_Plus: error in usb_bulk_transfer.");
    }
    if (transferred != nbytes) {
      fprintf(stderr, "usbAInScanRead_USB1608FS_Plus: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
      status = usbStatus_USB1608FS_Plus(udev);
      if ((status & AIN_SCAN_OVERRUN)) {
	fprintf(stderr, "Analog AIn scan overrun.\n");
	usbAInScanStop_USB1608FS_Plus(udev);
	usbAInScanClearFIFO_USB1608FS_Plus(udev);
      }
      return ret;
    }
  }

  if (options & CONTINUOUS) return nbytes;

  status = usbStatus_USB1608FS_Plus(udev);
  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
  if ((nbytes%wMaxPacketSize) == 0 && !(status & AIN_SCAN_RUNNING)) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) value, 2, &ret, 100);
  }

  if ((status & AIN_SCAN_OVERRUN)) {
    fprintf(stderr, "Analog AIn scan overrun.\n");
    usbAInScanStop_USB1608FS_Plus(udev);
    usbAInScanClearFIFO_USB1608FS_Plus(udev);
  }
  return nbytes;
}

void usbAInScanStop_USB1608FS_Plus(libusb_device_handle *udev)
{
  /*
    This command stops the analog input scan (if running).
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAInScanConfig_USB1608FS_Plus(libusb_device_handle *udev, uint8_t ranges[8])
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
    perror("usbAinScanConfig_USB1608FS_Plus error in writing configuration ranges.");
  }
}

void usbAInScanConfigR_USB1608FS_Plus(libusb_device_handle *udev, uint8_t *ranges)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret = -1;
  ret = libusb_control_transfer(udev, requesttype, AIN_CONFIG, 0x0, 0x0, (unsigned char *) ranges, 0x8, HS_DELAY);
  if (ret < 0) {
    perror("usbAInScanConfigR_USB1608FS_Plus: error in reading ranges.");
  }
}

void usbAInScanClearFIFO_USB1608FS_Plus(libusb_device_handle *udev)
{
  /*
    This command clears the internal scan endpoint FIFOs.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_CLR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

/***********************************************
 *            Counter/Timer                    *
 ***********************************************/
void usbCounterInit_USB1608FS_Plus(libusb_device_handle *udev)
{
  /*
    This command initializes the 32-bit event counter.  On a write, the
    specified counter (0 or 1) will be reset to zero.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, NULL, 0x0, HS_DELAY);
  return;
}

uint32_t usbCounter_USB1608FS_Plus(libusb_device_handle *udev)
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
void usbReadCalMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
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
    printf("usbReadCalMemory_USB1608FS_Plus: max bytes that can be written is 768.\n");
    return;
  }

  if (address > 0x2ff) {
    printf("usbReadCalMemory_USB1608FS_Plus: address must be in the range 0 - 0x2ff.\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, CAL_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbWriteCalMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{

  uint16_t unlock_code = 0xaa55;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 768) {
    printf("usbWriteCalMemory_USB1608FS_Plus: max bytes that can be written is 768.");
    return;
  }

  if (address > 0x2ff) {
    printf("usbWriteCalMemory_USB1608FS_Plus: address must be in the range 0 - 0x2ff.");
    return;
  }

  libusb_control_transfer(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (unsigned char *) &unlock_code, sizeof(unlock_code), HS_DELAY); // unlock memory
  libusb_control_transfer(udev, requesttype, CAL_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
  libusb_control_transfer(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (unsigned char *) 0x0, sizeof(uint16_t), HS_DELAY); // lock memory
}

void usbReadUserMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
    
  /*
    These commands allow for reading and writing the nonvolatile user
    memory. wLength specifies the number of bytes to read or write.
    The user memory is 256 bytes (address 0-0xff)
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 256) {
    printf("usbReadUserMemory_USB1608FS_Plus: max bytes that can be written is 256.");
    return;
  }

  if (address > 0xff) {
    printf("usbReadUserMemory_USB1608FS_Plus: address must be in the range 0 - 0xff.");
    return;
  }
  libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbWriteUserMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 256) {
    printf("usbWriteUserMemory_USB1608FS_Plus: max bytes that can be written is 256.");
    return;
  }

  if (address > 0xff) {
    printf("usbWriteUserMemory_USB1608FS_Plus: address must be in the range 0 - 0xff.");
    return;
  }
  libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbReadMBDMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  /*
    These commands allow for reading and writing the nonvolatile MBD memory. count must
    be less than or equal to 1024 (address 0-0x3ff).
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbReadMBDMemory_USB1608FS_Plus: max bytes that can be written is 1024.");
    return;
  }

  if (address > 0x3ff) {
    printf("usbReadMBDMemory_USB1608FS_Plus: address must be in the range 0 - 0x3ff.");
    return;
  }
  libusb_control_transfer(udev, requesttype, MBD_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbWriteMBDMemory_USB1608FS_Plus(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbWriteMBDMemory_USB1608FS_Plus: max bytes that can be written is 1024.");
    return;
  }

  if (address > 0x3ff) {
    printf("usbWriteMBDMemory_USB1608FS_Plus: address must be in the range 0 - 0x3ff");
    return;
  }
  libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/

void usbBlink_USB1608FS_Plus(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return;
}

void usbReset_USB1608FS_Plus(libusb_device_handle *udev)
{
  /*
    This command resets the device
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

uint16_t usbStatus_USB1608FS_Plus(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device.  Writing the command will clear
    the error indicators.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t status = 0x0;

  libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY);
  return status;
}

void usbGetSerialNumber_USB1608FS_Plus(libusb_device_handle *udev, char serial[9])
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

void usbDFU_USB1608FS_Plus(libusb_device_handle *udev)
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

void usbMBDCommand_USB1608FS_Plus(libusb_device_handle *udev, uint8_t str[])
{
  /*
    This command is the interface for text-based MBD commands and
    responses.  The length of the string must be passed in wLength for an
    OUT transfer.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MBD_COMMAND, 0x0, 0x0, (unsigned char *) str, strlen((char *) str), HS_DELAY);

}

void usbMBDRaw_USB1608FS_Plus(libusb_device_handle *udev, uint8_t cmd[], uint16_t size)
{
  /*
    This command is the interface for binary responses to certain MBD commands.
   */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MBD_RAW, 0x0, 0x0, (unsigned char *) cmd, size, HS_DELAY);
}

void cleanup_USB1608FS_Plus(libusb_device_handle *udev)
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|1);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}

double volts_USB1608FS_Plus(uint16_t value, uint8_t range)
{
  double volt = 0.0;
  switch(range) {
    case BP_10V:   volt = (value - 0x8000)*10.0/32768.; break;
    case BP_5V:    volt = (value - 0x8000)*5.0/32768.; break;
    case BP_2V:    volt = (value - 0x8000)*2.0/32768.; break;
    case BP_1V:    volt = (value - 0x8000)*1.0/32768.; break;
    default: printf("Unknown range.\n"); break;
  }
  return volt;
}
