/*
 *
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@tx.ncsu.edu>
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
#include "usb-7204.h"

#define HS_DELAY 2000

static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant. 

void usbBuildGainTable_USB7204(libusb_device_handle *udev, Calibration_AIN table[NMODE][NGAINS_USB7204][NCHAN_USB7204])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
     The calibration coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  uint16_t address;
  int i;

  address = 0xb0;
  for (i = 0; i < NCHAN_USB7204/2; i++) {
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_20_00V][i].slope);
    address += 4;
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *)&table[DF][BP_20_00V][i].intercept);
    address += 4;
  }

  address = 0xd0;
  for (i = 0; i < NCHAN_USB7204/2; i++) {
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_10_00V][i].slope);
    address += 4;
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_10_00V][i].intercept);
    address += 4;
  }

  address = 0xf0;
  for (i = 0; i < NCHAN_USB7204/2; i++) {
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *)&table[DF][BP_5_00V][i].slope);
    address += 4;
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_5_00V][i].intercept);
    address += 4;
  }

  address = 0x110;
  for (i = 0; i < NCHAN_USB7204; i++) {
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_4_00V][i].slope);
    address += 4;
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_4_00V][i].intercept);
    address += 4;
  }

  address = 0x130;
  for (i = 0; i < NCHAN_USB7204; i++) {
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_2_50V][i].slope);
    address += 4;
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_2_50V][i].intercept);
    address += 4;
  }

  address = 0x150;
  for (i = 0; i < NCHAN_USB7204; i++) {
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_2_00V][i].slope);
    address += 4;
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_2_00V][i].intercept);
    address += 4;
  }

  address = 0x170;
  for (i = 0; i < NCHAN_USB7204/2; i++) {
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_1_25V][i].slope);
    address += 4;
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_1_25V][i].intercept);
    address += 4;
  }

  address = 0x190;
  for (i = 0; i < NCHAN_USB7204/2; i++) {
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_1_00V][i].slope);
    address += 4;
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[DF][BP_1_00V][i].intercept);
    address += 4;
  }

  address = 0x1b0;
  for (i = 0; i < NCHAN_USB7204; i++) {
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[SE][BP_10_00V][i].slope);
    address += 4;
    usbWriteMemoryAddress_USB7204(udev, address);
    usbReadMemory_USB7204(udev, 4, (uint8_t *) &table[SE][BP_10_00V][i].intercept);
    address += 4;
  }

  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
}

void getMFGCAL_USB7204(libusb_device_handle *udev, struct tm *date)
{
  // get the manufactures calibration date

  time_t time;
  uint16_t address = 0x50;
  uint8_t data;

  // get the year (since 1900)
  address = 0x50;
  usbWriteMemoryAddress_USB7204(udev, address);
  usbReadMemory_USB7204(udev, 1, (uint8_t *) &data);
  date->tm_year = data + 100;

  // get the month 0 - 11
  address = 0x51;
  usbWriteMemoryAddress_USB7204(udev, address);
  usbReadMemory_USB7204(udev, 1, (uint8_t *) &data);
  date->tm_mon = data  - 1;  // Linux starts from 0
  
  // get the day of the month 1-31
  address = 0x52;
  usbWriteMemoryAddress_USB7204(udev, address);
  usbReadMemory_USB7204(udev, 1, (uint8_t *) &data);
  date->tm_mday = data;
  
  // get the hour 0-23
  address = 0x53;
  usbWriteMemoryAddress_USB7204(udev, address);
  usbReadMemory_USB7204(udev, 1, (uint8_t *) &data);
  date->tm_hour = data;

  // get the minute 0 - 59
  address = 0x54;
  usbWriteMemoryAddress_USB7204(udev, address);
  usbReadMemory_USB7204(udev, 1, (uint8_t *) &data);
  date->tm_min = data;

  // get the second 0 - 60
  address = 0x55;
  usbWriteMemoryAddress_USB7204(udev, address);
  usbReadMemory_USB7204(udev, 1, (uint8_t *) &data);
  date->tm_sec = data;

  time = mktime(date);
  date = localtime(&time);
}


/***********************************************
 *            Digital I/O                      *
 ***********************************************/

void usbDConfigPortR_USB7204(libusb_device_handle *udev, uint8_t port, uint8_t *direction)
{
  /* This command reads/writes the direction of the DIO port bits. 

     Port     : 0 = port 0,   1 = port 1
     Direction: 0 = output,   1 = input
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DCONFIG_PORT, port, 0x0, (unsigned char *) direction, sizeof(direction), HS_DELAY) < 0) {
    perror("usbDConfigPortR_USB7204: error in libusb_control_transfer().");
  }
}

void usbDConfigPort_USB7204(libusb_device_handle *udev, uint8_t port, uint8_t direction)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t cmd[2];
  
  cmd[0] = port;
  cmd[1] = direction;

  if (libusb_control_transfer(udev, requesttype, DCONFIG_PORT, 0x0, 0x0, (unsigned char *) cmd, sizeof(cmd), HS_DELAY) < 0) {
    perror("usbDConfigPort_USB7204: error in libusb_control_transfer().");
  }
}

uint8_t usbDPortR_USB7204(libusb_device_handle *udev, uint8_t port)
{
  /* This command reads/writes the DIO port. The digital port is bitwise configurable, so may be any 
     conbination  of inputs and outputs.  The return value will be the value seen at the port pins. 
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t value;


  if (libusb_control_transfer(udev, requesttype, DPORT, port, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY) < 0) {
    perror("usbDPortR_USB7204: error in libusb_control_transfer().");
  }
  return value;
}

void usbDPortW_USB7204(libusb_device_handle *udev, uint8_t port, uint8_t value)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t cmd[2];

  cmd[0] = port;
  cmd[1] = value;

  if (libusb_control_transfer(udev, requesttype, DPORT, 0x0, 0x0, (unsigned char *) cmd, sizeof(cmd), HS_DELAY) < 0) {
    perror("usbDPortW_USB7204: error in libusb_control_transfer().");
  }
  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/

uint16_t usbAIn_USB7204(libusb_device_handle *udev, uint8_t mode, uint8_t channel, uint8_t range)
{
  /* 
     This command reads the value from an analog input channel, setting the desired gain range first.

       channel:  the channel to read 
                     (0-3)  differential
                     (4-7)  differential reverse polarity
                     (8-15) single ended
       range:    the gain range to select (0-7)
   */

  uint16_t value;
  uint16_t wValue;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (mode == SE) channel += 8;
  wValue = (range << 8) | channel;

  if (libusb_control_transfer(udev, requesttype, AIN, wValue, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY) < 0) {
    perror("usbAIn_USB7204: error in libusb_control_transfer().");
  }
  return value;
}

void usbAInScan_USB7204(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count, float *frequency, uint8_t options)
{
  /* 
    This command scans a range of analog input channels and sends the
     readings in BULK IN transfers.  The gain ranges that are currently
     set on the desired channels will be used (these may be changed
     with AIn or ALoadQueue

      lowchannel:   the first channel of the scan (0-15)
      highchannel:  the last channel of the scan (0-15)
      count:        the total number of scans to perform, used only in single execution and burst modes.
                    value of 0 sets for continuous execution.
                    Note: the actual number of smaples returned is count*(highchannel - lowchannel +1)
      frequency:    sample frequency in Samples/second
      options:      bit 0: 1 = single execution         0 = continusous execution (note also set count = 0) 
                    bit 1: 1 = immediate transfer mode, 0 = block transfer mode
                    bit 2: 1 = use external trigger
                    bit 3: not used
                    bit 4: 1 = use channel gain queue,  0 = use channel parameters specified
                    bit 5: 1 = retrigger mode,          0 = normal trigger
                    bit 6: 1 = debug mode (scan returns consecutive integers instead of sampled data, used for 
                                           checking missed data, etc.)
                    bit 7: 1 = stall
  
     The rate of data collection is set by the internal 16-bit
     incrementing timer running at a base rate of 10 MHz.  The timer
     is controlled by the timer_prescale and timer_preload.  The
     timer will reset and provide an internal interrupt when its value
     equals timer_preload.  This allows for the lowest rate of 0.596
     Hz (1:256 prescale, preload = 0xFFFF).  It is preferable to keep
     the prescaler to the lowest values that will achieve the desired
     rate.

     preload = (10MHz / (frequency * prescaler)) - 1  (0 <= prescaler <= 8)

    The data will be returned in transfers on the bulk IN endpoint.

    The data will transfer until reaching the specified count, and
    AInStop is sent, or a data overrun occurs resulting in a Status of
    OVERRUN (and the BULK IN endpoint STALLing.  The BULK IN endpoint
    must be RESET in order to acquire again).  The size of the bulk
    transfers will vary depending on the munber of channels in the
    scan.

    Burst I/O mode will sample data to the onboard SRAM FIFO until
    full, and then return the data in continuous transfers.  Prescaler
    voalues above 1:8 are not allowed in burst I/O mode.  Single
    execution and immediate transfer bits will be ignored in this
    mode.

    Immediate transfer mode is used for low sampling rates to avoid
    delays in receiving the sampled data.  The data will be sent at
    the end of every timer period, rather than waiting for the buffer
    to fill.  This mode should not be used if the aggregate sampling
    rate is greater than 32,000 samples per second in order to avoid
    data loss.

    The external trigger may be used to start data collection
    synchronously.  If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data the
    specified rate.  No messages will be sent until the trigger is
    detected.

    External sync may be used to synchronize the sampling of multiple
    USB-1608FS devices, or to sample data using an external clock.
    The device must be set to be a sync slave with the usbSetSync
    command prior to using this mode.  Data will be acquired on all
    specified channels when the sync edge is detected.

    The retrigger mode option is only used if trigger is used.  This
    option will cause the trigger to be rearmed after count samples are
    acquired and forces continuous mode.

  */
  
  uint32_t preload;

  struct AInScan_t {
    uint8_t lowchannel;     // the first channel of the scan (0-7)
    uint8_t hichannel;      // the last channel in the scan (0-7)
    uint8_t count[4];       // the total number of scans to perform, used only in single execution and burst mode.
                            //  value of 0 sets for continuous execution
    uint8_t timer_prescale;    
    uint8_t timer_preload[2];  
    uint8_t options;
  } AInScan;

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (highchannel > 7) {
    printf("usbAInScan_USB7204: highchannel out of range.\n");
    return;
  }

  if (lowchannel > 7) {
    printf("usbAInScan_USB7204: lowchannel out of range.\n");
    return;
  }

  if (lowchannel > highchannel) {
    printf("usbAInScan_USB7204: lowchannel greater than highchannel.\n");
    return;
  }

  if (*frequency > 0.6 && *frequency < 50000.) {
    for (AInScan.timer_prescale = 0; AInScan.timer_prescale <= 8; AInScan.timer_prescale++) {
      preload = 10.0e6/((*frequency) * (1<<AInScan.timer_prescale));
      if (preload < 0xffff) { 
	AInScan.timer_preload[0] = (uint8_t) preload & 0xff;        // low byte
	AInScan.timer_preload[1] = (uint8_t) (preload >> 8) & 0xff; // high byte
	*frequency = 10.0e6/(preload*(1<<AInScan.timer_prescale));
	break;
      }
    }
  } else if (*frequency == 0.0) {
    options |= AIN_EXTERN_TRIGGER;
  } else {
    printf("usbAInScan_USB7204: frequency out of range.\n");
    return;
  }
    
  AInScan.lowchannel = lowchannel;
  AInScan.hichannel = highchannel;
  AInScan.count[0] = (uint8_t) (count & 0xff);         // low byte
  AInScan.count[1] = (uint8_t) ((count >> 8) & 0xff);
  AInScan.count[2] = (uint8_t) ((count >> 16) & 0xff);
  AInScan.count[3] = (uint8_t) ((count >> 24) & 0xff); // high byte
  AInScan.options = options;

  usbAInStop_USB7204(udev);
  libusb_control_transfer(udev, requesttype, AIN_SCAN, 0x0, 0x0, (unsigned char *) &AInScan, 10, HS_DELAY);
} 

int usbAInScanRead_USB7204(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data)
{
  int ret = -1;
  int nbytes = nChan*nScan*2;    // number of bytes to read in 64 bit chunks
  uint16_t status = 0;
  char value[wMaxPacketSize];
  int transferred;

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) data, nbytes, &transferred, HS_DELAY);  
  if (ret < 0) {
    perror("usbAInScanRead_USB7204: error in usb_bulk_transfer.");
  }
  if (transferred != nbytes) {
    fprintf(stderr, "usbAInScanRead_USB7204: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
  }

  status = usbStatus_USB7204(udev);
  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
  if ((nbytes%wMaxPacketSize) == 0) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) value, 2, &ret, 100);
  }

  if ((status & AIN_SCAN_OVERRUN)) {
    printf("Analog AIn scan overrun.\n");
    usbAInStop_USB7204(udev);
  }
  return transferred;
}


void usbAInStop_USB7204(libusb_device_handle *udev)
{
  /* This command stops the analog scan (if running). */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AIN_STOP, 0x0, 0x0, (unsigned char *) NULL, 0, HS_DELAY) < 0) {
    perror("usbAInStop_USB7204: error in libusb_control_transfer().");
  }
  return;
}

void usbAInLoadQueue_USB7204(libusb_device_handle *udev, LoadQueue *gainArray)
{
  /* 
     The device can scan analog input channels with different gain
     settings.  This function provides the mechanism for configuring
     each channel with a unique range.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, ALOAD_QUEUE, 0x0, 0x0, (unsigned char *) gainArray, sizeof(LoadQueue), HS_DELAY) < 0) {
    perror("usbAInLoadQueue_USB7204: error in libusb_control_transfer().");
  }
  return;
}

/***********************************************
 *            Analog Output                    *
 ***********************************************/

void usbAOut_USB7204(libusb_device_handle *udev, uint8_t channel, float vout)
{
  /*
    This command writes the value to an analog output channel.  The value is a 16-bit
    unsigned value, but the DAC is a 12-bit DAC.  The lower 4-bits of the value are
    ignored by the DAC.  The equation for the output voltage is:
   
      V_out =  ( k / 2^16) * V_ref

    where k is the value written to the device, and V_ref = 4.096 V.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t cmd[3];
  uint16_t value;

  if (vout >= 4.096) {
    value = 0xffff;
  } else if (vout < 0.0) {
    value = 0x0;
  } else {
    value = vout * 65536. / 4.096;
  }

  value >>= 0x4;  // pack in lower 12 bits.

  cmd[0] = channel;
  cmd[1] = value & 0xff;
  cmd[2] = (value >> 8) & 0xff;

  if (libusb_control_transfer(udev, requesttype, AOUT, 0x0, 0x0, (unsigned char *) cmd, sizeof(cmd), HS_DELAY) < 0) {
    perror("usbAOut_USB7204: error in libusb_control_transfer().");
  }
}

void usbAOutScanStop_USB7204(libusb_device_handle *udev)
{
  //  This command stops the analog output scan (if running) and clears flags except the error flag.

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AOUT_STOP, 0x0, 0x0, (unsigned char *) NULL, 0, HS_DELAY) < 0) {
    perror("usbAOutScanStop_USB7204: error in libusb_control_transfer().");
  }
}

void usbAOutScanReset_USB7204(libusb_device_handle *udev)
{
  /*
    This command stops ad clears the internal USB BULK buffers (x2), resets
    internal FIFO pointers and FIFO Depth.  Also unstalls Bulk Endpoint
    in the event of an underrun.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (libusb_control_transfer(udev, requesttype, AOUT_RESET, 0x0, 0x0, (unsigned char *) NULL, 0, HS_DELAY) < 0) {
    perror("usbAOutScanReset_USB7204: error in libusb_control_transfer().");
  }
}

int usbAOutScan_USB7204(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel,
			  uint32_t count, float *frequency, uint16_t data[], uint8_t options)
{
  int num_samples;
  int transferred;
  uint32_t preload;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  struct scanReport_t {
    uint8_t lowchannel;   // the first channel of the scan
    uint8_t highchannel;  // the last channel of the scan
    uint8_t count[4];     // the total number of scans to perform
    uint8_t prescale;     // timer prescale
    uint8_t preload[2];   // timer preload
    uint8_t options;      // bit 0: 1 = single execution  0 = continuous
                          // bit 1: 1 = use external trigger
  } scanReport;

  if (highchannel > 1) {
    printf("usbAOutScan: highchannel out of range.\n");
    return -1;
  }
  if (lowchannel > 1) {
    printf("usbAOutScan: lowchannel out of range.\n");
    return -1;
  }

  if (lowchannel > highchannel) {
    printf("usbAOutScan: lowchannel greater than highchannel.\n");
    return -1;
  }

  if (*frequency <= 0.596) {
    printf("usbAOutScan: frequency must be greater than 0.596 Hz\n");
    return -1;
  }

  num_samples = count*(highchannel - lowchannel + 1);

  scanReport.lowchannel = lowchannel;
  scanReport.highchannel = highchannel;
  scanReport.count[0] = (uint8_t) count & 0xff;           // low byte
  scanReport.count[1] = (uint8_t) (count >>  8) & 0xff;
  scanReport.count[2] = (uint8_t) (count >> 16) & 0xff;
  scanReport.count[3] = (uint8_t) (count >> 24) & 0xff;   // high byte
  scanReport.options = options;

  for (scanReport.prescale = 0; scanReport.prescale <= 8; scanReport.prescale++) {
    preload = 10e6/((*frequency) * (1<<scanReport.prescale));
    if (preload <= 0xffff) {
      scanReport.preload[0] = (uint8_t) preload & 0xff;          // low byte
      scanReport.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
      break;
    }
  }

  if (scanReport.prescale == 9 || preload == 0) {
    printf("usbAOutScan_USB7204: frequency out of range.\n");
    return -1;
  }
  
  *frequency = 10e6/((1<<scanReport.prescale)*preload);

  ret = libusb_control_transfer(udev, requesttype, AOUT_SCAN, 0x0, 0x0, (unsigned char*) &scanReport, sizeof(scanReport), 5000);
  if (ret < 0) {
    perror("Error in usbAOutScan_USB7204: libusb_control_transfer error");
    return ret;
  }
  transferred = 0;
  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT | 2, (unsigned char *) data, num_samples, &transferred, HS_DELAY);
  if (ret < 0) {
    perror("Error in usbAOutScan_USB7204: libusb_bulk_transfer error");
  }
  return transferred;
}


/***********************************************
 *            Counter                          *
 ***********************************************/

/* Initialize the counter */
void usbInitCounter_USB7204(libusb_device_handle *udev)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, (unsigned char *) NULL, 0, HS_DELAY) < 0) {
    perror("usbInitCounter_USB7204: error in libusb_control_transfer().");
  }
  return;
}

uint32_t usbReadCounter_USB7204(libusb_device_handle *udev)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t counter;

  if (libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, (unsigned char *) &counter, sizeof(counter), HS_DELAY) < 0) {
    perror("usbReadCounter_USB7204: error in libusb_control_transfer().");
  }
  return counter;
}

void usbReadMemory_USB7204(libusb_device_handle *udev, uint8_t count, uint8_t* data)
{
  /*
    This command reads/writes data to the configuration memory
    (EEPROM).  Writes to calibration (Slop/Offset) data and
    calibration date only provided when the unlock code (0xAA55) is
    written to memory address 0x400.  All other access is valid
    without unlock.  Any other value written to the memory address
    0x400 will relock the calibration memory.

    count:  the number of bytes to read (64 max)
    data:   the data that was read
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 64)  {
    printf("usbReadMemory_USB7204.  max count is 64 bytes.\n");
    count = 64;
  }

  if (libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, count, HS_DELAY) < 0) {
    perror("usbReadMemory_USB7204: error in libusb_control_transfer().");
  }
}

void usbWriteMemory_USB7204(libusb_device_handle *udev, uint8_t count, uint8_t* data)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (count > 64)  {
    printf("usbWriteMemory_USB7204.  max count is 64 bytes.\n");
    count = 64;
  }

  if (libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, count, HS_DELAY) < 0) {
    perror("usbWriteMemory_USB7204: error in libusb_control_transfer().");
  }
}

uint16_t usbReadMemoryAddress_USB7204(libusb_device_handle *udev)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t address;

  if (libusb_control_transfer(udev, requesttype, MEMORY_ADDR, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY) < 0) {
    perror("usbReadMemoryAddress_USB7204: error in libusb_control_transfer().");
  }
  return address;
}

void usbWriteMemoryAddress_USB7204(libusb_device_handle *udev, uint16_t address)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, MEMORY_ADDR, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY) < 0) {
    perror("usbWriteMemoryAddress_USB7204: error in libusb_control_transfer().");
  }
}

void usbBlinkLED_USB7204(libusb_device_handle *udev, uint8_t count)
{
  /* This command causes te LED to flash "count" number of times. */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY) < 0) {
    perror("usbBlinkLED_USB7204: error in libusb_control_transfer().");
  }
}

void usbReset_USB7204(libusb_device_handle *udev, uint8_t type)
{
  /*
    This function causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.

    type: 0 = DAQ Defaults- Parameters reset for scans, inputs, ranges, etc.  Does not
              impact any I/O states
          1 = Device resets with disconnect from the USB.  Will impact all outputs as it
              reinitializes.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (libusb_control_transfer(udev, requesttype, MEMORY_ADDR, 0x0, 0x0, (unsigned char *) &type, sizeof(type), HS_DELAY) < 0) {
    perror("usbReset_USB7204: error in libusb_control_transfer().");
  }
}

void usbTriggerConfig_USB7204(libusb_device_handle *udev, uint8_t type)
{
  /*
    This function configures the external trigger for ananlog input.  The trigger
    may be configured to activate with either a logic rising edge or falling edge
    input.  Once the trigger is received, the analog input will proceed as configured.
    the EXTTRIG option must be used in the AInScan command to utilize this feature.

    type: 0 = external trigger falling edge
          1 = external trigger rising edge
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, (unsigned char *) &type, sizeof(type), HS_DELAY) < 0) {
    perror("usbTriggerConfig_USB7204: error in libusb_control_transfer().");
  }
}

void usbSyncConfig_USB7204(libusb_device_handle *udev, uint8_t type)
{
  /*
    This command configures the sync signal.  The sync signal may be
    used to synchronize the analog input scan of multiple devices.
    When multiple devices are to be used, one device is selected as
    the master and the rest as slaves.  The sync signal of all devices
    must be wired together.  The master will output a pulse every
    sample, and all the devices will acquire their samples
    simultaneously.  Theis may also be used to pace one or more
    devices from an external TTL/CMOS clock signal (max rate = 50 kHz).

    This may also be used with an external trigger; the external
    trigger signal should be brought to the master device, and all
    devices will begin sampling when the master is triggered.

    If a device is configed as a slave, it will not acquire data when
    given the AInScan command until it detects a pulse on the sync
    input.
    
     type:    0 = master
              1 = slave

  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, SYNC_CONFIG, 0x0, 0x0, (unsigned char *) &type, sizeof(type), HS_DELAY) < 0) {
    perror("usbSyncConfig_USB7204: error in libusb_control_transfer().");
  }
}

uint16_t usbStatus_USB7204(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device:
    status: bit 0: 0 = Sync slave,            1 = sync master
            bit 1: 0 = trigger falling edge,  1 = trigger rising edge
            bit 2: 1 = overrun error during AInScan, cleared when 
                       starting a new scan.
            bits 3-14 TbD
            bit 15: 1 = program memory update mode
  */ 

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t status;

  if (libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY) < 0) {
    perror("usbStatus_USB7204: error in libusb_control_transfer().");
  }
  return status;
}

void usbCalConfig_USB7204(libusb_device_handle *udev, uint8_t setting)
{
  /*
    This command controls the CAL mux.  The mux will be disabled on power up.
    setting:     bits 0-2     voltage value
                              0 = 0V
                              1 = 0.625V
                              2 = 1.25V
                              3 = 2.5V
                              4 = 5V
                              5 = Temp sensor
                 bit 3: polarity,  0 = positive,  1 = negative
                 bit 4: enable     0 = disabled (inputs connected to analog channels)
                                   1 = enabled (CAL mode)
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, CAL_CONFIG, 0x0, 0x0, (unsigned char *) &setting, sizeof(setting), HS_DELAY) < 0) {
    perror("usbCalConfig_USB7204: error in libusb_control_transfer().");
  }
}

void usbGetSerialNumber_USB7204(libusb_device_handle *udev, char serial[9])
{
  /*
    This commands reads the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). The new serial number will be programmed but not used
    until hardware reset.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, SERIAL, 0x0, 0x0, (unsigned char *) serial, 8, HS_DELAY) < 0) {
    perror("usbGetSerialNumber_USB7204: error in libusb_control_transfer().");
  }
  serial[8] = '\0';
}

void usbSetSerialNumber_USB7204(libusb_device_handle *udev, char serial[9])
{
  /*
    This commands writes the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). The new serial number will be programmed but not used
    until hardware reset.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, SERIAL, 0x0, 0x0, (unsigned char *) serial, 8, HS_DELAY) < 0) {
    perror("usbSetSerialNumber_USB7204: error in libusb_control_transfer().");
  }
}

void usbUpdateMode_USB7204(libusb_device_handle *udev)
{
  /*
    This command puts the device into code download mode, initializes
    the external SRAM, and resets the program memory checksum to 0.
    The unlock code must be correct as a further safety device.  Call
    this once before sending code with UpdateData.  If not in code
    download mode, any downloaded code will be ignored.
  */

  uint8_t unlock_code = 0xad;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, UPDATE_MODE, 0x0, 0x0, (unsigned char *) &unlock_code, sizeof(unlock_code), HS_DELAY) < 0) {
    perror("usbUpdateMode_USB7204: error in libusb_control_transfer().");
  }
}

void usbUpdateAddress_USB7204(libusb_device_handle *udev, uint8_t address[3])
{
  /* 
     This command sends the address for downloading program memory data to the device
     address: 24 bit start address for this portion of program memory.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, UPDATE_ADDR, 0x0, 0x0, (unsigned char *) address, 0x3, HS_DELAY) < 0) {
    perror("usbUpdateAddress_USB7204: error in libusb_control_transfer().");
  }
}

void usbUpdateData_USB7204(libusb_device_handle *udev, uint8_t count, uint8_t *data)
{
  /*
    This command sends the new program memory image to the device.  The downloaded program
    memory image is stored in external SRAM.  The image will be written to FLASH program
    memory when the UpdateCode command is issued (updates must bne enabled with UnlockCode
    first.) A running checksum will be calculated as the program memory image is sent, and the host
    should compare its own checksum with this value (retrieved with ReadChecksum) prior to
    sending the UpdateCode command.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, UPDATE_DATA, 0x0, 0x0, (unsigned char *) data, count, HS_DELAY) < 0) {
    perror("usbUpdateData_USB7204: error in libusb_control_transfer().");
  }
}

void usbUpdateChecksum_USB7204(libusb_device_handle *udev, uint16_t *checksum)
{
  /*
    This command returns the current code download checksum
  */
 
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, UPDATE_CHECKSUM, 0x0, 0x0, (unsigned char *) checksum, sizeof(checksum), HS_DELAY) < 0) {
    perror("usbUpdateChecksum_USB7204: error in libusb_control_transfer().");
  }
}

double volts_USB7204(uint16_t value, uint8_t range)
{
  double volt = 0.0;
  switch(range) {
    case BP_20_00V: volt = (value - 0x800)*20.0/2048.; break;
    case BP_10_00V: volt = (value - 0x800)*10.0/2048.; break;
    case BP_5_00V:  volt = (value - 0x800)*5.00/2048.; break;
    case BP_4_00V:  volt = (value - 0x800)*4.00/2048.; break;
    case BP_2_50V:  volt = (value - 0x800)*2.50/2048.; break;
    case BP_2_00V:  volt = (value - 0x800)*2.00/2048.; break;
    case BP_1_25V:  volt = (value - 0x800)*1.25/2048.; break;
    case BP_1_00V:  volt = (value - 0x800)*1.00/2048.; break;
    default: printf("Unknown range.\n"); break;
  }
  return volt;
}
