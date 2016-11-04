/*
 *
 *  Copyright (c) 2014 Warren J. Jasper <wjasper@tx.ncsu.edu>
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
#include "usb-dio32HS.h"
#include "usb-dio32HS.rbf"

#define HS_DELAY 1000

static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant. 
void usbInit_DIO32HS(libusb_device_handle *udev)
{
  int i;
  /* This function does the following:
     1. Configure the FPGA
     2. Finds the maxPacketSize for bulk transfers
  */

  wMaxPacketSize = usb_get_max_packet_size(udev, 0);

  if (!(usbStatus_USBDIO32HS(udev) & FPGA_CONFIGURED)) {
    usbFPGAConfig_USBDIO32HS(udev);
    if (usbStatus_USBDIO32HS(udev) & FPGA_CONFIG_MODE) {
      for (i = 0; i <= (sizeof(FPGA_data) - 64); i += 64) {
	usbFPGAData_USBDIO32HS(udev, &FPGA_data[i], 64);
      }
      if (sizeof(FPGA_data) % 64) {
	usbFPGAData_USBDIO32HS(udev, &FPGA_data[i], sizeof(FPGA_data)%64);
      }
      if (!(usbStatus_USBDIO32HS(udev) & FPGA_CONFIGURED)) {
	printf("Error: FPGA for the USB-DIO32HS is not configured.  status = %#x\n", usbStatus_USBDIO32HS(udev));
	return;
      }
    } else {
      printf("Error: could not put USB-DIO32HS into FPGA Config Mode.  status = %#x\n", usbStatus_USBDIO32HS(udev));
      return;
    }
  } else {
    //  printf("USB-DIO32HS FPGA configured.\n");
    return;
  }
}

/***********************************************
 *            Digital I/O                      *
 ***********************************************/
/* reads tristate port regiser */
uint16_t  usbDTristateR_USBDIO32HS(libusb_device_handle *udev, uint8_t port)
{
  /*
    This command reads or writes the digital port tristate
    register.  The tristate register determines if the
    latch register value is driven onto the port pin.  A
    '1' in the tristate register makes the corresponding
    pin an input, a '0' makes it an output.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, 0x0, port, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDTristateR_USBDIO32HS: error in libusb_control_transfer().");
  }
  return data;
}

void usbDTristateW_USBDIO32HS(libusb_device_handle *udev, uint16_t value, uint8_t port)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, value, port, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbDTristateW_USBDIO32HS: error in libusb_control_transfer().");
  }
  return;
}

/* reads digital port  */
uint32_t usbDPort_USBDIO32HS(libusb_device_handle *udev)
{
  /*
    This command reads the current state of the digital port pins (port 0 |  port 1)
   */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t data;

  if (libusb_control_transfer(udev, requesttype, DPORT, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDPort_USBDIO32HS: error in libusb_control_transfer().");
  }
  return data;
}

/* writes digital port */
uint32_t usbDLatchR_USBDIO32HS(libusb_device_handle *udev, uint8_t port)
{
  /*
    This command reads or writes the digital port latch register
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t data = 0x0;
  
  if (libusb_control_transfer(udev, requesttype, DLATCH, 0x0, port, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDLatchR_USBDIO32HS: error in libusb_control_transfer().");
  }
  return data;
}

void usbDLatchW_USBDIO32HS(libusb_device_handle *udev, uint32_t data, uint8_t port)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DLATCH, 0x0, port, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDLatchW_USBDIO32HS: error in libusb_control_transfer().");
  }
  return;
}

/***********************************************
 *            Register Commands                *
 ***********************************************/
void usbReadReg_USBDIO32HS(libusb_device_handle *udev, uint8_t address, uint8_t *value)
{
  /* This command reads the FPGA register at the specified address */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, READ_REG, 0x0, address, value, 1, HS_DELAY);
}

void usbWriteReg_USBDIO32HS(libusb_device_handle *udev, uint8_t address, uint8_t value)
{
  /* This command writes the FPGA register at the specified address.
     The user can change the tristate settings with this command, so
     any time it is sent, the software must re-check the DTristate
     status to know the current state.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, WRITE_REG, value, address, NULL, 0x0, HS_DELAY);
}

/***********************************************
 *            Acquisition Commands             *
 ***********************************************/
void usbInScanStart_USBDIO32HS(libusb_device_handle *udev, uint8_t channel_map, uint32_t count, uint32_t retrig_count,
			       double frequency, uint8_t packet_size, uint8_t options)
{
  /* This command starts the input channel scan.  This command will
     result in a bus stall if an input scan is currently running.

    Notes:

    The pacer rate is set by an internal 32-bit incrementing timer
    running at a base rate of 96MHz.  The timer is controlled by
    pacer_period.  A pulse will be output at the INPUT_PACER_OUT pin
    at every pacer_period interval regardless of mode.

    If pacer_period is set to 0, the device does not generate a clock.
    It uses the INPUT_PACER_IN pin as the pacer source.

    The timer will be reset and sample acquired when its value equals
    timer_period.  The equation for calculating timer_period is:

        timer_period = [96MHz / (sample frequency)] - 1

    The data will be returned in packets utilizing a bulk
    endpoint. The scan will not begin until the InScanStart command is
    sent (and any trigger conditions are met.)  Data will be sent
    until reaching the specified count or a InScanStop command is
    sent.

    The packet_size parameter is used for low sampling rates to avoid
    delays in receiving the sampled data.  The buffer will be sent,
    rather than waiting for the buffer to fill.  This mode should not
    be used for high sample rates in order to avoid data loss.

    Pattern detection is used with the PatternDetectConfig command to
    set up a specified number of bits to watch, and then trigger when
    those bits reach the specified value.

    The retrigger mode option and retrig_count parameter are only used
    if trigger is used.  This option will cause the trigger to be
    rearmed after retrig_count samples are acquired, with a total of
    count samples being returned for the entire scan.
  */

  
  struct InScan_t {
    uint8_t channel_map;     /* bit field marking which channels are in the scan    
			        bit 0: 1 = Port 0
			        bit 1: 1 = Port 1
			        bits 2-7:  Reserved
			     */
    uint8_t count[4];        // the total number of scans to perform (0 for continuous scan)
    uint8_t retrig_count[4]; // the number of scans to perform for each trigger in retrigger mode
    uint8_t pacer_period[4]; // pacer timer period value (0 for external clock)
    uint8_t packet_size;     // (number of samples - 1) to transfer at a time
    uint8_t options;         /* bit field that controls various options:
                                bit 0:   1 = use external trigger
                                bit 1:   1 = use Pattern Detection Trigger
                                bit 2:   1 = retrigger mode,  0 = normal trigger
                                bits 3-7 Reserved
			     */
  } InScan;

  uint32_t pacer_period;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (frequency == 0.0) {
    pacer_period = 0;  // use INPUT_PACER_IN pin
  } else {
    pacer_period = rint((96.E6 / frequency) - 1);
  }

  InScan.channel_map = channel_map;
  memcpy(&InScan.count, &count, 4);
  memcpy(&InScan.retrig_count, &retrig_count, 4);
  memcpy(&InScan.pacer_period, &pacer_period, 4);
  InScan.options = options;

  libusb_control_transfer(udev, requesttype, IN_SCAN_START, 0x0, 0x0, (unsigned char *) &InScan, sizeof(InScan), HS_DELAY);
}

int usbInScanRead_USBDIO32HS(libusb_device_handle *udev, int count, uint16_t *data)
{
  char value[64];
  int ret = -1;
  int nbytes = 2*count;    // nuber of bytes to read;
  int transferred;
  uint8_t status;
  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) data, nbytes, &transferred, HS_DELAY);
  if (ret < 0) {
    perror("usbScanRead_USBDIO32HS: error in usb_bulk_transfer.");
  }
  if (transferred != nbytes) {
    fprintf(stderr, "usbAInScanRead_USBDIO32HS: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
  }

  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
  if ((nbytes%wMaxPacketSize) == 0) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) value, 2, &ret, 100);
  }

  status = usbStatus_USBDIO32HS(udev);
  if ((status & IN_SCAN_OVERRUN)) {
    printf("Scan overrun.\n");
    usbInScanStop_USBDIO32HS(udev);
    usbInScanClearFIFO_USBDIO32HS(udev);
    usbInScanBulkFlush_USBDIO32HS(udev, 5);
  }

  return ret;
}

void usbInScanStop_USBDIO32HS(libusb_device_handle *udev)
{
  /*
    This command stops the analog input scan (if running).
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, IN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbInScanClearFIFO_USBDIO32HS(libusb_device_handle *udev)
{
  /* This command clears the input firmware buffer */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, IN_SCAN_CLEAR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbInScanBulkFlush_USBDIO32HS(libusb_device_handle *udev, uint8_t count)
{
  /* The command fluses the input Bulk pipe a number of times */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, IN_BULK_FLUSH, count, 0x0, NULL, 0x0, HS_DELAY);
}

void usbOutScanStart_USBDIO32HS(libusb_device_handle *udev, uint8_t channel_map, uint32_t count, uint32_t retrig_count,
			       double frequency, uint8_t options)
{
  /* This command starts the output channel scan.  This command will result in a bus stall
     if an input scan is currently running.

    Notes:
    
    The output scan operates with the host continuously transferring
    data from the outputs until the end of the scan.  If the count
    parameter is 0, the scan will run until the OutScanStop command is
    issued by the host; if it is nonzero, the scan will stop
    automatically after the specified number of scans have been output.
    The channels in the scan are selected in the options bit field.
    Scans refers to the number of updates to the channels (if both
    channels are used, one scan is an update to both channels.)

    The time base is controlled by an internal 32-bit timer running at
    a base rate of 96MHz.  The timer is controlled by pacer_period.
    The equation for calculating pacer_period is:
  
      pacer_period = [96MHz / (sample frequency)] - 1

    The same time base is used for all channels when the scan involved
    multiple channels.  The output data is to be sent using bulk out
    endpoints.  The data must be in the format:

    low channel sample 0: [high channel sample 0]
    low channel sample 1: [high channel sample 1]
    ...
    low channel sample 1: [high channel sample n]

    The output data is written to an internal FIFO.  The bulk endpoint
    data is only accepted if there is room in the FIFO.  Output data
    may be sent to the FIFO before the start of the scan, and the FIFO
    is cleared when the OutScanClearFIFO command is received.  The
    scan will not begin until the OutScanStart command is sent (and
    output data is in the FIFO).  Data will be output until reaching
    the specified number of scans (in single execution mode) or an
    OutScanStop command is sent.
  */

  struct OutScan_t {
    uint8_t channel_map;     /* bit field marking which channels are in the scan    
			        bit 0: 1 = Port 0
			        bit 1: 1 = Port 1
			        bits 2-7:  Reserved
			     */
    uint8_t count[4];        // the total number of scans to perform (0 for continuous scan)
    uint8_t retrig_count[4]; // the number of scans to perform for each trigger in retrigger mode
    uint8_t pacer_period[4]; // pacer timer period value (0 for external clock)
    uint8_t options;         /* bit field that controls various options:
                                bit 0:   1 = use external trigger
                                bit 1:   1 = use Pattern Detection Trigger
                                bit 2:   1 = retrigger mode,  0 = normal trigger
                                bits 3-7 Reserved
			     */
  } OutScan;

  uint32_t pacer_period;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (frequency == 0.0) {
    pacer_period = 0;  // user ICLKO
  } else {
    pacer_period = rint((96.E6 / frequency) - 1);
  }
  
  OutScan.channel_map = channel_map;
  memcpy(&OutScan.count, &count, 4);
  memcpy(&OutScan.retrig_count, &retrig_count, 4);
  memcpy(&OutScan.pacer_period, &pacer_period, 4);
  OutScan.options = options;

  libusb_control_transfer(udev, requesttype, OUT_SCAN_START, 0x0, 0x0, (unsigned char *) &OutScan, sizeof(OutScan), HS_DELAY);
}

int usbOutScanWrite_USBDIO32HS(libusb_device_handle *udev, int count, uint16_t *data)
{
  int transferred;
  int ret = -1;
  int nbytes = 2*count;    // nuber of bytes to read;;

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT|2, (unsigned char *) data, nbytes, &transferred, HS_DELAY);
  if (ret < 0) {
    perror("Error in usbOutScan_USBDIO32HS: libusb_bulk_transfer error");
  }
  return transferred;
}

void usbOutScanStop_USBDIO32HS(libusb_device_handle *udev)
{
  /*
    This command stops the analog output scan (if running).
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, OUT_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbOutScanClearFIFO_USBDIO32HS(libusb_device_handle *udev)
{
  /* This command clears the output firmware buffer */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, OUT_SCAN_CLEAR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

/***********************************************
 *            Memory Commands                  *
 ***********************************************/
void usbMemoryR_USBDIO32HS(libusb_device_handle *udev, uint8_t *data, uint16_t length)
{
  /*
    This command reads or writes data from the EEPROM memory.  The
    read will begin at the current address, which may be set with
    MemAddress.  The address will automatically increment during a
    read or write but stay within the range allowed for the EEPROM.
    The amount of data to be written or read is specified in wLength.

    The range from 0x0000 to 0x6FFF is used for storing the
    microcontroller firmware and is write-protected during normal
    operation.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  ret = libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
  if (ret != length) {
    printf("usbMemoryR_USBDIO32HS: error in reading memory\n");
  }
}
void usbMemoryW_USBDIO32HS(libusb_device_handle *udev, uint8_t *data, uint16_t length)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
}

void usbMemAddressR_USB1024HS(libusb_device_handle *udev, uint16_t address)
{
  /*
    This command reads or writes the address used for memory accesses.
    The upper byte is used to denominate different memory areas.  The
    memory map for this device is

       Address                            Description
    =============               ============================
    0x0000-0x6FFF               Microcontroller firmware (write protected)
    0X7000-0X7FFF               User data

    The firmware area is protected by a separate command so is not typically
    write-enabled.  The calibration area is unlocked by writing the value 0xAA55
    to address 0x8000.  The area will remain unlocked until the device is reset
    or a value other than 0xAA55 is written to address 0x8000.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY);
}

void usbMemAddressW_USBDIO3208HS(libusb_device_handle *udev, uint16_t address)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY);
}

void usbMemWriteEnable_USBDIO32HS(libusb_device_handle *udev)
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

uint16_t usbStatus_USBDIO32HS(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t status = 0x0;

  libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY);
  return status;
}

void usbBlink_USBDIO32HS(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY) < 0) {
    perror("usbBlink_USBDIO32HS: error in libusb_control_transfer().");
  }
  return;
}
void usbReset_USBDIO32HS(libusb_device_handle *udev)
{
  /*
    This function causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

void usbTriggerConfig_USBDIO32HS(libusb_device_handle *udev, uint8_t options)
{
  /*
    This function configures the AInScan trigger.  Once the trigger is
    received, the AInScan will proceed as configured.  The "use
    trigger" option must be used in the ScanStart command to
    utilize this feature.

    options:     bit 0: trigger mode (0 = level,  1 = edge)
                 bit 1: trigger polarity (0 = low / falling, 1 = high / rising)
                 bits 2-7: reserved
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, (unsigned char *) &options, sizeof(options), HS_DELAY);
}

void usbTriggerConfigR_USBDIO32HS(libusb_device_handle *udev, uint8_t *options)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, (unsigned char *) options, sizeof(options), HS_DELAY);
}

void usbPatternDetectConfig(libusb_device_handle *udev, uint16_t value, uint16_t mask, uint8_t options)
{
  /* This function configures the Pattern Detection trigger.  Once the
     tgrigger is received, the Scan will proceed as configured.  The
     "use Pattern Detection trigger" option must be used in
     InScanStart command to utilize this feature.
  */

  struct PatternDectect_t {
    uint16_t value;   // the pattern on which to trigger
    uint16_t mask;    // these bits will mask the inputs such that only bits set to 1 here will be compared to the pattern.
    uint8_t  options; // bit field that controls various options:
                      // bit 0: Trigger Port (set to 1 for Port 1, 0 for Port 0)
                      // bit 1-2  00 = Equal to Pattern
                      //          01 = Not equal to Pattern
                      //          10 = Greater than Patter's numeric value
                      //          11 = Less than Pattern's numeric value
                      // bits 3-7 Reserved
  } patternDetect;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  patternDetect.value = value;
  patternDetect.mask = mask;
  patternDetect.options = options;
  libusb_control_transfer(udev, requesttype, PATTERN_DETECT, 0x0, 0x0, (unsigned char *) &patternDetect, 0x5, HS_DELAY);
}

void usbGetSerialNumber_USBDIO32HS(libusb_device_handle *udev, char serial[9])
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

/***************************************
 *          FPGA  Commands             *
 ***************************************/

void usbFPGAConfig_USBDIO32HS(libusb_device_handle *udev)
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

void usbFPGAData_USBDIO32HS(libusb_device_handle *udev, uint8_t *data, uint8_t length)
{
  /*
    This command writes the FPGA configuration data to the device.  This
    command is not accepted unless the device is in FPGA config mode.  The
    number of bytes to be written must be specified in wLength.

    data: max length is 64 bytes
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (length > 64) {
    printf("usbFPGAData_USBDIO32HS: max length = 64 bytes\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, FPGA_DATA, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
}

void usbFPGAVersion_USBDIO32HS(libusb_device_handle *udev, uint16_t *version)
{
  /*
    This command reads the FPGA version in hexadecimal BCD.  eg. 0x0102 is version 01.02
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, FPGA_VERSION, 0x0, 0x0, (unsigned char *) version, sizeof(uint16_t), HS_DELAY);
}
