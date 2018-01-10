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
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include "pmd.h"
#include "usb-20X.h"

/* Commands and Report ID for USB 1208HS */
/* Digital I/O Commands */
#define DTRISTATE            (0x00) // Read/write digital tristate register
#define DPORT                (0x01) // Read digital port pins
#define DLATCH               (0x02) // Read/write digital port output latch register

/* Analog Input Commands */
#define AIN                  (0x10) // Read analog input channel
#define AIN_SCAN_START       (0x11) // Start analog input scan
#define AIN_SCAN_STOP        (0x12) // Stop analog input scan
#define AIN_SCAN_CLR_FIFO    (0x15) // Clear the analog input scan FIFO
#define AIN_BULK_FLUSH       (0x16) // Flush the bulk endpoint with empty packets

/* Analog Output Commands (USB-202/205 only) */
#define AOUT                 (0x18) // Read/Write analog output channel

/* Counter Commands */
#define COUNTER              (0x20) // Read/reset event counter

/* Memory Commands */
#define CAL_MEMORY           (0x30) // Read/write calibration memory
#define USER_MEMORY          (0x31) // Read/write user memory
#define MBD_MEMORY           (0x32) // Read/write MBD memory
  
 /* Miscellaneous Commands */
#define BLINK_LED            (0x41) // Blink the LED
#define RESET                (0x42) // Reset the device
#define STATUS               (0x44) // Device Status
#define SERIAL               (0x48) // Read/write USB serial number
#define DFU                  (0x50) // Enter device firmware upgrade mode

/* MBD */
#define MBD_COMMAND          (0x80) // Text-based MBD command/response
#define MBD_RAW              (0x81) // Raw MBD response

#define HS_DELAY 2000

static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant. 

void usbBuildGainTable_USB20X(libusb_device_handle *udev, float table[NCHAN_USB20X][2])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
     The calibration coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  int i, j;
  uint16_t address = 0;

  for (i = 0; i < NCHAN_USB20X; i++) {
    for (j = 0; j < 2; j++) {
      usbReadCalMemory_USB20X(udev, address, 4, (uint8_t *) &table[i][j]);
      address += 4;
    }
  }

  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
}

void usbCalDate_USB20X(libusb_device_handle *udev, struct tm *date)
{
  /* This command reads the factory calibration date */

  calibrationTimeStamp calDate;
  uint16_t address = 0x040;  // beginning of MFG Calibration date
  time_t time;

  usbReadCalMemory_USB20X(udev, address, sizeof(calDate), (uint8_t *) &calDate);
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
uint8_t  usbDTristateR_USB20X(libusb_device_handle *udev)
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
    printf("usbDTristateR_USB20X: error in usb_control_msg().\n");
  }
  return data;
}

void usbDTristateW_USB20X(libusb_device_handle *udev, uint8_t value)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    printf("usbDTristateW_USB1208HS: error in libusb_control_transfer().\n");
  }
  return;
}

/* reads digital port  */
uint8_t usbDPort_USB20X(libusb_device_handle *udev)
{
  /*
    This command reads the current state of the digital port pins
   */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data;

  if (libusb_control_transfer(udev, requesttype, DPORT, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDPort_USB20X: error in libusb_control_transfer().\n");
  }
  return data;
}

/* read digital port */
uint8_t usbDLatchR_USB20X(libusb_device_handle *udev)
{
  /*
    This command reads or writes the digital port latch register
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;
  
  if (libusb_control_transfer(udev, requesttype, DLATCH, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    printf("usbDLatchR_USB20X: error in libusb_control_transfer().\n");
  }
  return data;
}

void usbDLatchW_USB20X(libusb_device_handle *udev, uint8_t data)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DLATCH, data, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    printf("usbDLatchW_USB20X: error in libusb_control_transfer().\n");
  }
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/
uint16_t usbAIn_USB20X(libusb_device_handle *udev, uint8_t channel)
{
  /*
    This command reads the value of an anlog input channel.  This command will result
    in a bus stall if an AInScan is currently running.
  */
  uint16_t value;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, AIN, channel, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY);
  return value;
}

void usbAInScanStart_USB20X(libusb_device_handle *udev, uint32_t count, double frequency, uint8_t channels, uint8_t options, uint8_t trigger_source, uint8_t trigger_mode)
{
  /*
     The command starts an analog input scan.  The command will result
     in a bus stall if an AInScan is currently running.  The USB-201
     will not generate an internal pacer faster than 100 kHz; the
     USB-204 will not generate an interal pacer faster than 500 kHz.

     The ADC is paced such that the pacer controls the ADC
     conversions.  The internal pacer rate is set by an internal
     32-bit timer running at a base rate of 70 MHz.  The timer is
     controlled by pacer_period.  The value is the period of the ADC
     conversions.  A pulse will be output at the PACER_OUT pin at
     every pacer_period internval The equation for calculating
     pacer_period is:

     pacer_period = [70 MHz / (A/D frequency)] - 1

     If pacer_period is set to 0, the device does not generate an A/D
     clock.  This uses the PACER_IN pin as the pacer source.  Each
     rising edge of PACER_IN starts a conversion.

     The data will be returned in packets untilizing a bulk endpoint.
     The data will be in the format:

     lowchannel sample 0: lowchannel + 1 sample 0: ... : hichannel sample 0
     lowchannel sample 1: lowchannel + 1 sample 1: ... : hichannel sample 1
     ...
     lowchannel sample n: lowchannel + 1 sample n: ... : hichannel sample n

     The scan will not begin until the AInScan Start command is sent
     and any trigger conditions are met.  Data will be sent until
     reaching the specified count or an usbAInScanStop_USB20X()
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

     Overruns are indicated by the device stalling the bulk endpoint
     during the scan.  The host may read the status to verify and must
     clear the stall condition before further scan can be performed.
  */

  struct AInScan_t {
    uint32_t count;         // The total number of scans to perform (0 for continuous scan)
    uint32_t pacer_period;  // The pacer timer period value. (0 for external clock PACER_IN).
    uint8_t channels;       // bit field that selects the channels in the scan;
    uint8_t options;        /* bit field that controls scan options:
			       bit 0:  0 = block transfer mode, 1 = immediate transfer mode
                               bit 1:  Reserved
   	                       bit 2:  Reserved
                               bit 3:  Reserved
                               bit 4:  Reserved
			       bit 5:  Reserved
			       bit 6:  Reserved
			       bit 7:  0 = stall on overrun, 1 = inhibit stall
		            */
    uint8_t trigger_source; // 0 = no trigger,  1 = digital trigger
    uint8_t trigger_mode;   // 0 = Edge/rising, 1 = Edge/falling, 2 = Level/high 3 = Level/low
  } AInScan;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (frequency > 0 && frequency < 500000.) {
      AInScan.pacer_period = rint((70.E6 / frequency) - 1);
  } else if (frequency == 0.0) {
    AInScan.pacer_period = 0;
  } else {
    printf("usbainScanStart_USB20X: frequency out of range\n");
    return;
  }

  AInScan.count = count;
  AInScan.channels = channels;
  AInScan.options = options;
  AInScan.trigger_source = trigger_source;
  AInScan.trigger_mode = trigger_mode;

  usbAInScanStop_USB20X(udev);
  usbAInScanClearFIFO_USB20X(udev);

  /* Pack the data into 12 bytes */
  libusb_control_transfer(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (unsigned char *) &AInScan, 12, HS_DELAY);
}

int usbAInScanRead_USB20X(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data, uint8_t options, unsigned int timeout)
{
  int i;
  int ret = -1;
  int nbytes = nScan*nChan*2;    // number of bytes to read in 64 bit chunks
  int transferred;               // number of bytes actually transferred
  uint16_t status = 0;
  char value[MAX_PACKET_SIZE];

  if (options & IMMEDIATE_TRANSFER_MODE) {  // data returned after each scan
    for (i = 0; i < nbytes/2; i++) {
      ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) &data[i], 2*nChan, &transferred, timeout);
      if (ret < 0) {
	perror("usbAInScanRead_USB20X: error in usb_bulk_transfer. (immediate transfer mode)");
      }
      if (transferred != 2) {
	fprintf(stderr, "usbAInScanRead_USB20X: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
      }
    }
  } else { 
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) data, nbytes, &transferred, timeout);
    if (ret < 0) {
      perror("usbAInScanRead_USB20X: error in usb_bulk_transfer (bulk transfer mode).");
    }
    if (transferred != nbytes) {
      fprintf(stderr, "usbAInScanRead_USB20X: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
      status = usbStatus_USB20X(udev);
      if ((status & AIN_SCAN_OVERRUN)) {
        fprintf(stderr, "Analog AIn scan overrun.\n");
      }
    return ret;
    }
  }

  if (options & CONTINUOUS) { // continuous mode
    return transferred;
  }

  status = usbStatus_USB20X(udev);
  if ((status & AIN_SCAN_OVERRUN)) {
    fprintf(stderr, "Analog AIn scan overrun.\n");
  }

  usbAInScanStop_USB20X(udev);
  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
  if ((nbytes%wMaxPacketSize) == 0) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) value, 32, &ret, 100);
  }

  return nbytes;
}

void usbAInScanStop_USB20X(libusb_device_handle *udev)
{
  /*
    This command stops the analog input scan (if running).
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAInScanClearFIFO_USB20X(libusb_device_handle *udev)
{
  /*
    This command clears the internal scan endpoint FIFOs.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_SCAN_CLR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbAInBulkFlush_USB20X(libusb_device_handle *udev, uint8_t count)
{
  /* This comman will send a specified number of empty bulk IN packets. */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_BULK_FLUSH, 0x0, 0x0, (unsigned char *) &count, 0x1, HS_DELAY);
}

/***********************************************
 *            Analog Output                    *
 ***********************************************/
void usbAOut_USB20X(libusb_device_handle *udev, uint8_t channel, uint16_t value)
{
  /* This command reads or writes the value  of an analog output channel */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (channel > 1) {  // channel must be 0 or 1
    printf("usbAOut_USB20X: channel must be 0 or 1.\n");
    return;
  }
  if (value > 4095) {  // value must be (0-4095)
    printf("usbAOut_USB20X: value must be less than 4096.\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, AOUT, (int) value, (int) channel, (unsigned char *) 0x0, 0x0, HS_DELAY);
  return;
}


/***********************************************
 *            Counter/Timer                    *
 ***********************************************/
void usbCounterInit_USB20X(libusb_device_handle *udev)
{
  /*
    This command initializes the 32-bit event counter.  On a write, the
    specified counter (0 or 1) will be reset to zero.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, NULL, 0x0, HS_DELAY);
  return;
}

uint32_t usbCounter_USB20X(libusb_device_handle *udev)
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
void usbReadCalMemory_USB20X(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
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
    printf("usbReadCalMemory_USB20X: max bytes that can be written is 768.");
    return;
  }

  if (address > 0x2ff) {
    printf("usbCalMemoryR_USB20X: address must be in the range 0 - 0x2ff.");
    return;
  }
  libusb_control_transfer(udev, requesttype, CAL_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbWriteCalMemory_USB20X(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{

  uint16_t unlock_code = 0xaa55;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 768) {
    printf("usbWriteCalMemory_USB20X: max bytes that can be written is 768.");
    return;
  }

  if (address > 0x2ff) {
    printf("usbWriteCalMemory_USB20X: address must be in the range 0 - 0x2ff.");
    return;
  }

  libusb_control_transfer(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (unsigned char *) &unlock_code, sizeof(unlock_code), HS_DELAY); // unlock memory
  libusb_control_transfer(udev, requesttype, CAL_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
  libusb_control_transfer(udev, requesttype, CAL_MEMORY, 0x300, 0x0, (unsigned char *) 0x0, sizeof(uint16_t), HS_DELAY); // lock memory
}

void usbReadUserMemory_USB20X(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
    
  /*
    These commands allow for reading and writing the nonvolatile user memory. count must
    be less than or equal to 256
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 256) {
    printf("usbReadUserMemory_USB20X: max bytes that can be written is 256.");
    return;
  }

  if (address > 0xff) {
    printf("usbReadUserMemory_USB20X: address must be in the range 0 - 0xff.");
    return;
  }
  libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbWriteUserMemory_USB20X(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 255) {
    printf("usbWriteUserMemory_USB20X: max bytes that can be written is 768.");
    return;
  }

  if (address > 0xff) {
    printf("usbWriteUserMemory_USB20X: address must be in the range 0 - 0x2ff.");
    return;
  }
  libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbReadMBDMemory_USB20X(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{
  /*
    These commands allow for reading and writing the nonvolatile MBD memory. count must
    be less than or equal to 1024
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbReadMBDMemory_USB20X: max bytes that can be written is 1024.");
    return;
  }

  if (address > 0x400) {
    printf("usbReadMBDMemory_USB20X: address must be in the range 0 - 0x400.");
    return;
  }
  libusb_control_transfer(udev, requesttype, MBD_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbWriteMBDMemory_USB20X(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t data[])
{

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbWriteMBDMemory_USB20X: max bytes that can be written is 1024.");
    return;
  }

  if (address > 0x400) {
    printf("usbWriteUserMemory_USB20X: address must be in the range 0 - 0x400.");
    return;
  }
  libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, HS_DELAY);
}

  
/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/

void usbReset_USB20X(libusb_device_handle *udev)
{
  /*
    This command resets the device
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

uint16_t usbStatus_USB20X(libusb_device_handle *udev)
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

void usbGetSerialNumber_USB20X(libusb_device_handle *udev, char serial[9])
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

void usbBlink_USB20X(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return;
}

void cleanup_USB20X(libusb_device_handle *udev)
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|1);
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT|1);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}

double volts_USB20X(uint16_t value)
{
  double volt = 0.0;
  volt = (value - 2048.)*10./2048.;
  return volt;
}
