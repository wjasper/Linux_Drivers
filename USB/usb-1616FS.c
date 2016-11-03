/*
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
#include <stdint.h>

#include "pmd.h"
#include "usb-1616FS.h"

#define FS_DELAY 300

void usbBuildCalTable_USB1616FS(libusb_device_handle *udev, float table_AIN[NGAINS_USB1616FS][NCHAN_USB1616FS][2])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
       voltage = value*table[gain#][chan#][0] + table[gain#][chan#][1]
     only needed for fast lookup.
  */

  float y0, y1, v0, v1, x0, x1;
  int j;
  uint16_t addr;
  uint16_t data[32];

  /* Read in the internal ground reference at 0x90 in the EEPROM */
  addr = 0x90;
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 4, (uint8_t *) &v0);

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./20. + 0x8000;
  
  /* Read in the internal reference for +/- 10V at 0x80 in the EEPROM */
  addr = 0x80;
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);

  /* Calculate the corresponding calibrated value y1 */
  y1 = v1*65536./20. + 0x8000;

  addr = 0xb0;         // +/- 10V Uncalibrated readings
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 32, (uint8_t *) data);
  addr += 32;
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 32, (uint8_t *) &data[16]);
  for (j = 0; j < NCHAN_USB1616FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // gain
    table_AIN[0][j][0] = (y1 - y0)/(x1 - x0);        // slope
    table_AIN[0][j][1] = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./10. + 0x8000;

  /* Read in the internal reference for +/- 5V at 0x84 in the EEPROM */
  addr = 0x84;
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);

  /* Calculate the corresponding calibrated value y1. */
  y1 = v1*65536./10. + 0x8000;

  addr = 0xf0;         // +/- 5V Uncalibrated readings
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 32, (uint8_t *) data);
  addr += 32;			    
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 32, (uint8_t *) &data[16]);
  for (j = 0; j < NCHAN_USB1616FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // gain
    table_AIN[1][j][0] = (y1 - y0)/(x1 - x0);        // slope
    table_AIN[1][j][1] = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }
     
  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./4. + 0x8000;

  /* Read in the internal reference for +/- 2V at 0x88 in the EEPROM */
  addr = 0x88;
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);

  /* Calculate the corresponding calibrated value y1. */
  y1 = v1*65536./4. + 0x8000;
  
  addr = 0x130;         // +/- 2V Uncalibrated readings
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 32, (uint8_t *) &data);
  addr += 32;			    
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 32, (uint8_t *) &data[16]);
  for (j = 0; j < NCHAN_USB1616FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // gain
    table_AIN[2][j][0] = (y1 - y0)/(x1 - x0);        // slope
    table_AIN[2][j][1] = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./2. + 0x8000;
     
  /* Read in the internal reference for +/- 1V at 0x8c in the EEPROM */
  addr = 0x8c;
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);

  /* Calculate the corresponding calibrated value y1. */
  y1 = v1*65536./2. + 0x8000;

  addr = 0x170;         // +/- 1V Uncalibrated readings
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 32, (uint8_t *) &data);
  addr += 32;			    
  usbReadMemory_USB1616FS(udev, addr, EEPROM, 32, (uint8_t *) &data[16]);
  for (j = 0; j < NCHAN_USB1616FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // gain
    table_AIN[3][j][0] = (y1 - y0)/(x1 - x0);        // slope
    table_AIN[3][j][1] = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }

  // claim all the needed interfaces for AInScan
  for (j = 1; j <= 6; j++) {
    libusb_detach_kernel_driver(udev, j);
    libusb_claim_interface(udev, j);
  }
  return;
}

/***********************************************
 *            Digital I/O                      *
 ***********************************************/

/* configures digital port */
void usbDConfigPort_USB1616FS(libusb_device_handle *udev, uint8_t direction)
{
  struct config_port_t {
    uint8_t reportID;
    uint8_t direction;
  } config_port;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | DCONFIG;   // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  config_port.reportID = DCONFIG;
  config_port.direction = direction;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &config_port, sizeof(config_port), 5000);
  if (ret < 0) {
    perror("Error in usbDConfigPort_USB1616FS: libusb_control_transfer error");
  }
}

/* configures digital bit */
void usbDConfigBit_USB1616FS(libusb_device_handle *udev, uint8_t bit_num, uint8_t direction)
{
  struct config_bit_t {
    uint8_t reportID;
    uint8_t bit_num;      
    uint8_t direction;
  } config_bit;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                    // HID Set_Report
  uint16_t wValue = (2 << 8) | DCONFIG_BIT; // HID ouptut
  uint16_t wIndex = 0;                      // Interface

  config_bit.reportID = DCONFIG_BIT;
  config_bit.bit_num = bit_num;
  config_bit.direction = direction;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &config_bit, sizeof(config_bit), 5000);
  if (ret < 0) {
    perror("Error in usbConfigBit_USB1616FS: libusb_control_transfer error");
  }
}

/* reads digital port  */
uint8_t usbDIn_USB1616FS(libusb_device_handle *udev)
{
  int transferred;
  uint8_t reportID = DIN;

  struct read_port_t {
    uint8_t reportID;
    uint8_t value;
  } read_port;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;             // HID Set_Report
  uint16_t wValue = (2 << 8) | DIN;  // HID ouptut
  uint16_t wIndex = 0;               // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbDIn_USB1616FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN  | 2, (unsigned char*) &read_port, sizeof(read_port), &transferred, FS_DELAY);

  return (read_port.value);
}

/* reads digital bit  */
uint8_t usbDInBit_USB1616FS(libusb_device_handle *udev, uint8_t bit_num)
{
  int transferred;

  struct read_bit_t {
    uint8_t reportID;
    uint8_t value;
  } read_bit;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                 // HID Set_Report
  uint16_t wValue = (2 << 8) | DBIT_IN;  // HID ouptut
  uint16_t wIndex = 0;                   // Interface

  read_bit.reportID = DBIT_IN;
  read_bit.value = bit_num;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &read_bit, sizeof(read_bit), 5000);
  if (ret < 0) {
    perror("Error in usbDInBit_USB1616FS: libusb_control_transfer error");
  }
  ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char*) &read_bit, sizeof(read_bit), &transferred, FS_DELAY);
  if (ret < 0) {
    perror("Error in usbDInBit_USB1616FS: libusb_interrupt_transfer error");
  }
  return  (read_bit.value);
}

/* writes digital port */
void usbDOut_USB1616FS(libusb_device_handle *udev, uint8_t value)
{
  struct write_port_t {
    uint8_t reportID;
    uint8_t value;
  } write_port;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;              // HID Set_Report
  uint16_t wValue = (2 << 8) | DOUT;  // HID ouptut
  uint16_t wIndex = 0;                // Interface

  write_port.reportID = DOUT;
  write_port.value = value;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &write_port, sizeof(write_port), 5000);
  if (ret < 0) {
    perror("Error in usbDOut_USB1616FS: libusb_control_transfer error");
  }
}

/* writes digital bit  */
void usbDOutBit_USB1616FS(libusb_device_handle *udev, uint8_t bit_num, uint8_t value)
{
  struct write_bit_t {
    uint8_t reportID;
    uint8_t bit_num;
    uint8_t value;
  } write_bit;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | DBIT_OUT;  // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  write_bit.reportID = DBIT_OUT;
  write_bit.bit_num = bit_num;
  write_bit.value = value;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &write_bit, sizeof(write_bit), 5000);
  if (ret < 0) {
    perror("Error in usbDOutBit_USB1616FS: libusb_control_transfer error");
  }

  return;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/

/* reads from analog in */
int16_t usbAIn_USB1616FS(libusb_device_handle *udev, uint8_t channel, uint8_t range, float table_AIN[NGAINS_USB1616FS][NCHAN_USB1616FS][2])
{
  int transferred;

  uint16_t data;
  int16_t value = 0;
  uint8_t report[3];

  struct ain_t {
    uint8_t reportID;
    uint8_t channel;
    uint8_t range;
  } ain;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;              // HID Set_Report
  uint16_t wValue = (2 << 8) | AIN;   // HID ouptut
  uint16_t wIndex = 0;                // Interface

  ain.reportID = AIN;
  ain.channel = channel;
  ain.range = range;
  if (channel > NCHAN_USB1616FS - 1 ) {
    printf("usbAIN: channel out of range for differential mode.\n");
    return -1;
  }
  if (range > 7) {
    printf("usbAIN: range setting too large.\n");
    return -1;
  }

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &ain, sizeof(ain), 5000);
  if (ret < 0) {
    perror("Error in usbAIn_USB1616FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2,(unsigned char*) report, sizeof(report), &transferred, FS_DELAY);

  data = (uint16_t) ( report[1] | (report[2] << 8));

  /* apply calibration correction 
        slope:  m = table_AIN[gain][channel][0]
        offset: b = table_AIN[gain][channel][1]
        correction = m*raw_value + b
  */

  switch (range) {
    case  BP_10_00V:
      value = (int) (table_AIN[0][channel][0]*((float) data) + table_AIN[0][channel][1]);
      break;
    case BP_5_00V:
      value = (int) (table_AIN[1][channel][0]*((float) data) + table_AIN[1][channel][1]);
      break;
    case BP_2_50V:  // use data from 2 V
    case BP_2_00V:
      value = (int) (table_AIN[2][channel][0]*((float) data) + table_AIN[2][channel][1]);
      break;
    case BP_1_25V:  // use data from 1 V
    case BP_1_00V:
    case BP_0_625V:
    case BP_0_3125V:
      value = (int) (table_AIN[3][channel][0]*((float) data) + table_AIN[3][channel][1]);
      break;
    default:
      break;
  }
	 
  if ( value >= 0x8000 ) {
    value -=  0x8000;
  } else {
    value = (0x8000 - value);
    value *= (-1);
  }

  return value;
}

void usbAInStop_USB1616FS(libusb_device_handle *udev)
{
  uint8_t reportID = AIN_STOP;;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | AIN_STOP;  // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbAInStop_USB1616FS: libusb_control_transfer error");
  }
}

int usbAInScan_USB1616FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t nScan,
			 float *frequency, uint8_t options, int16_t sdata[], float table_AIN[NGAINS_USB1616FS][NCHAN_USB1616FS][2], uint8_t gainArray[NCHAN_USB1616FS])
{
  /*
    This command scans a range of analog input channels and sends the
     readings in inerrupt transfers.  The gain ranges that are currently
     set on the desired channels will be used (these may be changed
     with AIn or ALoadQueue

      lowchannel:   the first channel of the scan (0-15)
      highchannel:  the last channel of the scan (0-15)
      count:        the total number of scans to perform, used only in single execution and burst modes.
                    Note: the actual number of samples returned is count*(highchannel - lowchannel + 1)
      frequency:    sample frequency in Samples/second
      options:      bit 0: 1 = single execution         0 = continusous execution 
                    bit 1: 1 = burst I/O mode,          0 = normal I/O mode
                    bit 2: 1 = immediate transfer mode, 0 = block transfer mode
                    bit 3: 1 = use external trigger
                    bit 4: 1 = use external sync
                    bit 5: 1 = debug mode (scan returns consecutive integers instead of sampled data, used for 
                                           checking missed data, etc.)
                    bits 6-7:  not used
  
     The rate of data collection is set by the internal 16-bit
     incrementing timer running at a base rate of 10 MHz.  The timer
     is controlled by the timer_prescale and timer_preload. The timer
     will provide an internal interrupt with its value rolls over from
     0xFFF to 0x0000.  The timer is preloaded with the value specified
     in timer_preload.  Thes allows for a lowest rate of 0.596Hz
     (1:256 prescale, preload = 0).  The preload value is calculated
     by 65536 - desired number of counts.  Th is preferable to keep
     the prescaler to the lowest value that will achieve the desired
     rate.

       preload = (10MHz / (frequency * prescaler))  (0 <= prescaler <= 8)

    The data will be returned in packets utilizing interrupt
    endpoints.  Five endpoints will be used; each endpoint allows for
    64 bytes of data (31 raw data values plus a scan value) to be sent
    every millisecond, so the theoretical limit is:

       6 endpoints * 31 S / ms = 186 kS/s

    The data will use successive endpoints, beginning with the first 
    endpoint at the start of the scan and cycling through all 6 endpoints
    until reaching the specified count or an AInStop is sent.
    
    Burst I/O mode will sample data to the onboard SRAM FIFO until
    full, and then return the data in continuous messages using all 5
    endpoints.  Prescaler values above 1:8 are not allowed in burst
    I/O mode.  Single execution and immediate transfer bits will be
    ignored in this mode.

    Immediate transfer mode is used for low sampling rates to avoid
    delays in receiving the sampled data.  The data will be sent at
    the end of every timer period, rather than waiting for the buffer
    to fill.  All 6 endpoints will still be used in a sequential
    manner.  This mode should not be used if the aggregate sampling
    rate is greater then 32,000 samples per second in order to avoid
    data loss.

    The external trigger may be used to start data collection
    synchronously.  If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data the
    specified rate.  No messages will be sent until the trigger is
    detected.

    External sync may be used to synchronize the sampling of multiple
    USB-1616FS devices, or to sample data using an external clock.
    The device must be set to be a sync slave with the usbSetSync
    command prior to using this mode.  Data will be acquired on all
    specified channels when the sync edge is detected.
   */

  int nSamples = 0;  // current number of samples taken;
  int tSamples;      // total number of samples to take;
  int i;
  int chan = lowchannel;
  int ep;
  uint32_t preload;
  int transferred;
  int timeout = FS_DELAY;

  int16_t value;
  uint16_t uvalue;
  uint8_t gain;    

  struct data_t {
    uint16_t scan_index0;       //    16 bit scan count
    uint16_t value0[31];        // 31 16-bit samples
    uint16_t scan_index1;       //    16 bit scan count
    uint16_t value1[31];        // 31 16-bit samples
  } data;
    
  struct arg {
    uint8_t  reportID;
    uint8_t  lowchannel;
    uint8_t  highchannel;
    uint8_t  count[4];
    uint8_t  prescale;
    uint8_t  preload[2];
    uint8_t  options;
  } arg;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | AIN_SCAN;  // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  if (highchannel > NCHAN_USB1616FS - 1) {
    printf("usbAInScan: highchannel out of range.\n");
    return -1;
  }
  if (lowchannel > NCHAN_USB1616FS - 1) {
    printf("usbAInScan: lowchannel out of range.\n");
    return -1;
  }
  if (highchannel >= lowchannel) {
    chan = lowchannel;
  } else {
    printf("usbAINScan: lowchannel can not be greater than higchannel.\n");
    return -1;
  }

  if (*frequency < 0.596) {
    printf("frequency must be greater than 0.596 Hz.\n");
    return 0;
  }

  arg.reportID = AIN_SCAN;
  arg.lowchannel = lowchannel;
  arg.highchannel = highchannel;
  arg.count[0] = (uint8_t) nScan & 0xff;           // low byte
  arg.count[1] = (uint8_t) (nScan >>  8) & 0xff;
  arg.count[2] = (uint8_t) (nScan >> 16) & 0xff;
  arg.count[3] = (uint8_t) (nScan >> 24) & 0xff;   // high byte
  arg.options = options;                        

  for ( arg.prescale = 0; arg.prescale <= 8; arg.prescale++ ) {
    preload = 10e6/((*frequency) * (1<<arg.prescale));
    if ( preload <= 0xffff ) {
      arg.preload[0] = (uint8_t) preload & 0xff;          // low byte
      arg.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
      break;
    }
  }

  *frequency = 10.e6/(preload*(1<<arg.prescale));

  if ( arg.prescale == 9 || preload == 0) {
    printf("usbAInScan_PMD1616FS: frequency out of range.\n");
    return -1;
  }

  tSamples = nScan*(highchannel - lowchannel + 1);

  usbAInStop_USB1616FS(udev);
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 500);
  if (ret < 0) {
    perror("Error in usbAInScan_USB1616FS: libusb_control_transfer error");
  }

  ep = 0;   //cycle through the endpoints 1 - 6;

  while (tSamples > nSamples) {
    ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN |(ep+3), (unsigned char *) &data, sizeof(data), &transferred, timeout);
    if (ret < 0 ) {
      printf("usbAInScan_USB1616FS: libusb_interrupt_read error. scan index = %d return code = %d\n", data.scan_index0, transferred);
    }
    for (i = 0; i < 31; i++) {
      if (nSamples == tSamples) goto cont;
      sdata[nSamples++] = data.value0[i];
      //       printf("sample # = %d   ep = %d raw value of data = %#x  %#x index = %hd  bytes = %d\n",
      //      	     nSamples, ep+1, data.value0[i], sdata[i], data.scan_index0, transferred);

    }
    for (i = 0; i < 31; i++) {
      if (nSamples == tSamples) goto cont;
      sdata[nSamples++] = data.value1[i];
      //       printf("sample # = %d   ep = %d raw value of data = %#x  %#x index = %hd  bytes = %d\n",
      //      	     nSamples, ep+1, data.value1[i], sdata[i], data.scan_index1, transferred);

    }
    ep = (ep + 1) % 6;  // increment to the next endpoint
  }

cont:  
  chan = lowchannel;
  for (i = 0; i < nSamples; i++) {
    /* apply calibration correction 
       slope:  m = table_AIN[gain][channel][0]
       offset: b = table_AIN[gain][channel][1]
       correction = m*raw_value + b
    */
    switch (gainArray[chan]) {
      case  BP_10_00V:
	gain = 0;
	break;
      case BP_5_00V:
	gain = 1;
	break;
      case BP_2_50V:  // use data from 2 V
      case BP_2_00V:
	gain = 2;
	break;
      case BP_1_25V:  // use data from 1 V
      case BP_1_00V:
      case BP_0_625V:
      case BP_0_3125V:
	gain = 3;
	break;
      default:
	gain = 0;
	break;
    }

    //    printf("sample = %d  chan = %d  gain = %d sdata = %#x\n", i, chan, gainArray[chan], sdata[i]);
    uvalue = sdata[i];
    value = (int) (table_AIN[gain][chan][0]*((float) uvalue) + table_AIN[gain][chan][1]);

    if ( value >= 0x8000 ) {
      value -=  0x8000;
    } else {
      value = (0x8000 - value);
      value *= (-1);
    }
    sdata[i] = value;
    chan++;                    // each sample is one channel higher.
    if (chan > highchannel) {  // wrap around which is one scan.
      chan = lowchannel;
    }
  }	

  printf("Total number of samples returned = %d\n", nSamples);
  usbAInStop_USB1616FS(udev);
  return nSamples;
}

void usbAInLoadQueue_USB1616FS(libusb_device_handle *udev, uint8_t gainArray[NCHAN_USB1616FS])
{
   struct loadQueue_t{
    uint8_t reportID;
    uint8_t gainArray[NCHAN_USB1616FS];
  } loadQueue;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                    // HID Set_Report
  uint16_t wValue = (2 << 8) | ALOAD_QUEUE; // HID ouptut
  uint16_t wIndex = 0;                      // Interface

  loadQueue.reportID = ALOAD_QUEUE;
  memcpy(loadQueue.gainArray, gainArray, NCHAN_USB1616FS);

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &loadQueue, sizeof(loadQueue), 5000);
  if (ret < 0) {
    perror("Error in usbAInLoadQueue_USB1616FS: libusb_control_transfer error");
  }
}

/* Initialize the counter */
void usbInitCounter_USB1616FS(libusb_device_handle *udev)
{
  uint8_t reportID = CINIT;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;               // HID Set_Report
  uint16_t wValue = (2 << 8) | CINIT;  // HID ouptut
  uint16_t wIndex = 0;                 // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbInitCounter_USB1616FS: libusb_control_transfer error");
  }
}

uint32_t usbReadCounter_USB1616FS(libusb_device_handle *udev)
{
  uint32_t value;
  int transferred;

  struct counter_t {
    uint8_t reportID;
    uint8_t value[4];
  } counter;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;             // HID Set_Report
  uint16_t wValue = (2 << 8) | CIN;  // HID ouptut
  uint16_t wIndex = 0;               // Interface

  counter.reportID = CIN;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &counter, 1, 5000);
  if (ret < 0) {
    perror("Error in usbCounter_USB1616FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &counter, sizeof(counter), &transferred, FS_DELAY);
  value =   counter.value[0] | (counter.value[1] << 8) |
    (counter.value[2] << 16) | (counter.value[3] << 24);

  return value;
}

/* blinks the LED of USB device */
void usbBlink_USB1616FS(libusb_device_handle *udev)
{
  uint8_t reportID = BLINK_LED;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | BLINK_LED; // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbBlink_USB1616FS: libusb_control_transfer error");
  }
}

int usbReset_USB1616FS(libusb_device_handle *udev)
{
  uint8_t reportID = RESET;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | BLINK_LED; // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbBlink_USB1616FS: libusb_control_transfer error");
  }
  return 0;
}

void usbSetTrigger_USB1616FS(libusb_device_handle *udev, uint8_t type)
{
  uint8_t cmd[2];
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                    // HID Set_Report
  uint16_t wValue = (2 << 8) | SET_TRIGGER; // HID ouptut
  uint16_t wIndex = 0;                      // Interface

  cmd[0] = SET_TRIGGER;
  cmd[1] = type;
  
   ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &cmd, sizeof(cmd), 5000);
  if (ret < 0) {
    perror("Error in usbSetTriggert_USB1616FS: libusb_control_transfer error");
  }
}

void usbSetSync_USB1616FS(libusb_device_handle *udev, uint8_t type)
{
  /*
    This command configures the sync signal.  The sync signal may be used to synchronize the
    analog input scan of multiple devices.  When multiple devices are to be used, one device is
    selected as the master and the rest as slaves.  The sync signal of all devices must be wired
    together.  The master will output a pulse every sample, and all the devices will acquire thier
    samples simultaneously.  The may also be used to pace one or more devices from an external
    TTL/CMOS clock signal (max rate = 50kHz)

    This may also be used with an external trigger; the external trigger signal should be brought to
    the master device, and all devices will begin sampling wihen the master is triggered.

    If the device is configured as a slave, it will not acquire data when given an AInScan command
    until it detects a pulse onthe syc input.

    type   0 = master,   1 = slave
  */
  uint8_t cmd[2];
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | SET_SYNC;  // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  cmd[0] = SET_SYNC;
  cmd[1] = type;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &cmd, sizeof(cmd), 5000);
  if (ret < 0) {
    perror("Error in usbSetSync_USB1616FS: libusb_control_transfer error");
  }
}

uint16_t usbGetStatus_USB1616FS(libusb_device_handle *udev)
{
  /*
    Bit  0:   0 = Sync slave,            1 = sync master
    Bit  1:   0 = trigger falling edge,  1 = trigger rising edge
    Bit 16:   1 = program memory update mode
  */

  int transferred;
  uint16_t status;

  struct statusReport_t {
  uint8_t reportID;
  uint8_t status[2];
  } statusReport;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                   // HID Set_Report
  uint16_t wValue = (2 << 8) | GET_STATUS; // HID ouptut
  uint16_t wIndex = 0;                     // Interface

  statusReport.reportID = GET_STATUS;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &statusReport, sizeof(statusReport), 5000);
  if (ret < 0) {
    perror("Error in usbGetStatus_USB1616FS: libusb_control_transfer error");
  }

  do {
    statusReport.reportID = 0;
    libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &statusReport, sizeof(statusReport), &transferred, FS_DELAY);
  } while ( statusReport.reportID != GET_STATUS);
  status = (uint16_t) (statusReport.status[0] | (statusReport.status[1] << 8));

  return status;
}

void usbReadMemory_USB1616FS( libusb_device_handle *udev, uint16_t address, uint8_t type, uint8_t count, uint8_t* memory)
{
  /*
    This command reads data from the configuration memory (EEPROM) or
    sample memory (SRAM).  All of the memory may be read.

   Addresses 0x000 - 0x07F are reserved for firmware data
   Addresses 0x080 - 0x3FF are available for use as calibration or user data
  */

  int ret;
  int transferred;

  struct arg_t {
    uint8_t reportID;
    uint8_t address[2];
    uint8_t type;
    uint8_t count;
  } arg;

  struct memRead_t {
    uint8_t reportID;
    uint8_t data[62];
  } memRead;

  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | MEM_READ;  // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  if ( count > 62 ) count = 62;
  arg.reportID = MEM_READ;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte
  arg.type = type;
  arg.count = count;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
  if (ret < 0) {
    perror("Error in usbReadMemory_USB1616FS: libusb_control_transfer error");
  }

  memRead.reportID = 0x0;

  // always read 63 bytes regardless.  Only the first count are meaningful.
  ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &memRead, 63, &transferred, FS_DELAY);
  if (ret < 0) {
    perror("Error in usbReadMemory_USB1616FS: usb_interrupt_read() reading error.");
    printf("Address = %#x  Count = %d  Number of bytes read = %d \n", address, count, transferred);
  }
  if (memRead.reportID != MEM_READ) {
    printf("Error in Reading Memory from EEPROM!\n");
    libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
    libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &memRead, count+1, &transferred, FS_DELAY);
  }
  memcpy(memory, &memRead.data[0], count);
}

int usbWriteMemory_USB1616FS(libusb_device_handle *udev, uint16_t address, uint8_t count, uint8_t data[])
{
  // Locations 0x00-0x7F are reserved for firmware and my not be written.
  int i;
  struct mem_write_report_t {
    uint8_t reportID;
    uint8_t address[2];
    uint8_t count;
    uint8_t data[count];
  } arg;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | MEM_WRITE; // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  if ( address <=0x7f ) return -1;
  if ( count > 59 ) count = 59;

  arg.reportID = MEM_WRITE;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte

  arg.count = count;
  for ( i = 0; i < count; i++ ) {
    arg.data[i] = data[i];
  }

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
  if (ret < 0) {
    perror("Error in usbWriteMemory_USB1616FS: libusb_control_transfer error");
  }
  return 0;
}

void usbPrepareDownload_USB1616FS(libusb_device_handle *udev)
{
  /*
    This command puts the device into code update mode.  The unlock code must be correct as a
    further safety device.  Call this once before sending code with usbWriteCode.  If not in
    code update mode, any usbWriteCode will be ignored.  A usbReset command must be issued at
    the end of the code download in order to return the device to operation with the new code.
  */

  struct prepare_download_report_t {
    uint8_t reportID;
    uint8_t unlock_code;
  } prepare_download_report;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                         // HID Set_Report
  uint16_t wValue = (2 << 8) | PREPARE_DOWNLOAD; // HID ouptut
  uint16_t wIndex = 0;                           // Interface

  prepare_download_report.reportID = PREPARE_DOWNLOAD;
  prepare_download_report.unlock_code = 0xad;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &prepare_download_report, sizeof(prepare_download_report), 5000);
  if (ret < 0) {
    perror("Error in usbPrepareDownload_USB1616FS: libusb_control_transfer error");
  }
}

void usbWriteCode_USB1616FS(libusb_device_handle *udev, uint32_t address, uint8_t count, uint8_t data[])
{
  /*
    This command send the new program memory image to the device.  The download program
    memory image is stored in external SRAM.  The image will be written to FLASH program
    memory when the UpdateCode command is issued (updates must be enabled with UnlockCode
    first.)  A running checksum will be calculated a the program memory image is sent, and the host
    should compare its own checksum with this value (retrieved with ReadChecksum) prior to
    sending the UpdateCode command.
  */

  struct arg_t {
    uint8_t reportID;
    uint8_t address[3];
    uint8_t count;        // 58 bytes max
    uint8_t data[58];     // the program data, 58 bytes max
  } arg;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                    // HID Set_Report
  uint16_t wValue = (2 << 8) | WRITE_CODE;  // HID ouptut
  uint16_t wIndex = 0;                      // Interface

  if (count > 58) return;

  arg.reportID = WRITE_CODE;
  memcpy(&arg.address[0], &address, 3);   // 24 bit address
  arg.count = count;
  memcpy(&arg.data[0], data, count);      // 24 bit address

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, count+5, 5000);
  if (ret < 0) {
    perror("Error in usbWriteCode_USB1616FS: libusb_control_transfer error");
  }
}

uint16_t usbReadChecksum_USB1616FS(libusb_device_handle *udev)
{
  struct checksum_report_t {
    uint8_t reportID;
    uint8_t checksum[2];
  } checksum_report;

  int ret;
  int transferred;

  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                      // HID Set_Report
  uint16_t wValue = (2 << 8) | READ_CHECKSUM; // HID ouptut
  uint16_t wIndex = 0;                        // Interface

  checksum_report.reportID = READ_CHECKSUM;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &checksum_report, sizeof(checksum_report), 5000);      
  if (ret < 0) {
    perror("Error in usbReadChecksum_USB1616FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &checksum_report, sizeof(checksum_report), &transferred, FS_DELAY);

  return (checksum_report.checksum[0] | (checksum_report.checksum[1] << 8));
}

void usbWriteSerial_USB1616FS(libusb_device_handle *udev, uint8_t serial[8])
{
  // Note: The new serial number will be programmed but not used until hardware reset.
  struct write_serial_t {
    uint8_t reportID;
    uint8_t serial[8];
  } write_serial;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                     // HID Set_Report
  uint16_t wValue = (2 << 8) | WRITE_SERIAL; // HID ouptut
  uint16_t wIndex = 0;                       // Interface

  write_serial.reportID = WRITE_SERIAL;
  memcpy(write_serial.serial, serial, 8);
  
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &write_serial, sizeof(write_serial), 5000);
  if (ret < 0) {
    perror("Error in usbWriteSerial_USB1616FS: libusb_control_transfer error");
  }
}

void usbUpdateCode_USB1616FS(libusb_device_handle *udev)
{
  int ret;
  uint8_t reportID = UPDATE_CODE;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                    // HID Set_Report
  uint16_t wValue = (2 << 8) | UPDATE_CODE; // HID ouptut
  uint16_t wIndex = 0;                      // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbUpdateCode_USB1616FS: libusb_control_transfer error");
  }
}

int usbReadCode_USB1616FS(libusb_device_handle *udev, uint32_t address, uint8_t count, uint8_t data[])
{
  int ret;
  int transferred;

  struct arg_t {
    uint8_t reportID;
    uint8_t address[3];
    uint8_t count;     // 62 max.
  } arg;

  struct read_code_t {
    uint8_t reportID;
    uint8_t data[62];
  } read_code;

  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | MEM_READ;  // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  if (count > 62) count = 62;  

  memcpy(&arg.address[0], &address, 3);   // 24 bit address
  arg.reportID = READ_CODE;
  arg.count = count;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
  if (ret < 0) {
    perror("Error in usbReadCode_USB1616FS: libusb_control_transfer error");
  }

  ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &read_code, count+1, &transferred, FS_DELAY);
  if (ret < 0) {
    perror("Error in usbReadCode_USB1616FS: usb_interrupt_transfer() reading error.");
  }

  memcpy(data, read_code.data, count);
  return transferred;
}

float volts_USB1616FS(const int gain, const signed short num)
{
  float volt = 0.0;

  switch( gain ) {
    case BP_10_00V:
      volt = num * 10.0 / 0x7fff;
      break;
    case BP_5_00V:
      volt = num * 5.0 / 0x7fff;
      break;
    case BP_2_50V:
      volt = num * 2.5 / 0x7fff;
      break;
    case BP_2_00V:
      volt = num * 2.0 / 0x7fff;
      break;
    case BP_1_25V:
      volt = num * 1.25 / 0x7fff;
      break;
    case BP_1_00V:
      volt = num * 1.0 / 0x7fff;
      break;
    case BP_0_625V:
      volt = num * 0.625 / 0x7fff;
      break;
    case BP_0_3125V:
      volt = num * 0.3125 / 0x7fff;
      break;
  }
  return volt;
}
