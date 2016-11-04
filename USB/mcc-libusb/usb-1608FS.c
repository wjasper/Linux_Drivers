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
#include "usb-1608FS.h"

// delay for full speed devices in ms
#define FS_DELAY 1000

void usbBuildCalTable_USB1608FS(libusb_device_handle *udev, Calibration_AIN table_AIN[NGAINS_USB1608FS][NCHAN_USB1608FS])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
       voltage = value*table[gain#][chan#][0] + table[gain#][chan#][1]
     only needed for fast lookup.
  */

  float y0, y1, y2, v0, v1, v2, x0, x1, x2;
  float m, b, target_sum, raw_sum, raw_sqr;
  int j, ret;
  uint16_t addr;
  uint16_t data[2*NCHAN_USB1608FS], data1[NCHAN_USB1608FS];

#ifndef EEPROM_VERSION_1_0 
  /* Use 3 point calibration.  Negative values stored in EEPROM at factory */

  /* Read in the internal ground reference at 0x90 in the EEPROM */
  addr = 0x90;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v0);
  y0 = v0*65536./20. + 0x8000;  //Calculate the corresponding calibrated value y0 
  
  /* Read in the internal reference for +/- 10V at 0x80 in the EEPROM  (+5.0 Nonminal) */
  addr = 0x80;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);
  y1 = v1*65536./20. + 0x8000;  //  Calculate the corresponding calibrated value y1

  /* Read in the internal reference for +/- 10V at 0xa0 in the EEPROM  (-5.0 Nonminal) */
  addr = 0xa0;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v2);
  y2 = v2*65536./20. + 0x8000;  // Calculate the corresponding calibrated value y2 

  addr = 0xb0;         // +/- 10V Uncalibrated readings
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 32, (uint8_t *) data);
  addr = 0x130;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 16, (uint8_t *) data1);

  for (j = 0; j < NCHAN_USB1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    x2 = data1[j];       // negative gain

    target_sum = y0 + y1 + y2;
    raw_sum = x0 + x1 + x2;
    raw_sqr = x0*x0 + x1*x1 + x2*x2;
    m = x0*y0 + x1*y1 + x2*y2;
    m = 3*m - raw_sum*target_sum;
    m /= (3*raw_sqr - raw_sum*raw_sum);
    b = (target_sum - m*raw_sum)/3.;

    table_AIN[0][j].slope = m;   // slope
    table_AIN[0][j].offset = b;  // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./10. + 0x8000;

  /* Read in the internal reference for +/- 5V at 0x84 in the EEPROM */
  addr = 0x84;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);
  y1 = v1*65536./10. + 0x8000;    // Calculate the corresponding calibrated value y1.

  /* Read in the internal reference for +/- 5V at 0x9c in the EEPROM  (-2.5 Nonminal) */
  addr = 0x9c;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v2);
  y2 = v2*65536./10. + 0x8000;  // Calculate the corresponding calibrated value y2 

  addr = 0xd0;         // +/- 5V Uncalibrated readings
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 32, (uint8_t *) data);
  addr = 0x140;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 16, (uint8_t *) data1);

  for (j = 0; j < NCHAN_USB1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    x2 = data1[j];       // negative gain

    target_sum = y0 + y1 + y2;
    raw_sum = x0 + x1 + x2;
    raw_sqr = x0*x0 + x1*x1 + x2*x2;
    m = x0*y0 + x1*y1 + x2*y2;
    m = 3*m - raw_sum*target_sum;
    m /= (3*raw_sqr - raw_sum*raw_sum);
    b = (target_sum - m*raw_sum)/3.;

    table_AIN[1][j].slope = m;   // slope
    table_AIN[1][j].offset = b;  // intercept
  }
     
  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./4. + 0x8000;

  /* Read in the internal reference for +/- 2V at 0x88 in the EEPROM */
  addr = 0x88;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);
  y1 = v1*65536./4. + 0x8000;    // Calculate the corresponding calibrated value y1.

  /* Read in the internal reference for +/- 2V at 0x98 in the EEPROM  (-2.5 Nonminal) */
  addr = 0x98;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v2);
  y2 = v2*65536./4. + 0x8000;  // Calculate the corresponding calibrated value y2 
  
  addr = 0xf0;         // +/- 2V Uncalibrated readings
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 32, (uint8_t *) &data);
  addr = 0x150;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 16, (uint8_t *) data1);

  for (j = 0; j < NCHAN_USB1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    x2 = data1[j];       // negative gain

    target_sum = y0 + y1 + y2;
    raw_sum = x0 + x1 + x2;
    raw_sqr = x0*x0 + x1*x1 + x2*x2;
    m = x0*y0 + x1*y1 + x2*y2;
    m = 3*m - raw_sum*target_sum;
    m /= (3*raw_sqr - raw_sum*raw_sum);
    b = (target_sum - m*raw_sum)/3.;

    table_AIN[2][j].slope = m;   // slope
    table_AIN[2][j].offset = b;  // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./2. + 0x8000;
     
  /* Read in the internal reference for +/- 1V at 0x8c in the EEPROM */
  addr = 0x8c;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);
  y1 = v1*65536./2. + 0x8000;   // Calculate the corresponding calibrated value y1.

  /* Read in the internal reference for +/- 1V at 0x94 in the EEPROM  (-2.5 Nonminal) */
  addr = 0x94;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v2);
  y2 = v2*65536./2. + 0x8000;   // Calculate the corresponding calibrated value y2 

  addr = 0x110;         // +/- 1V Uncalibrated readings
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 32, (uint8_t *) &data);
  addr = 0x160;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 16, (uint8_t *) data1);
  
  for (j = 0; j < NCHAN_USB1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    x2 = data1[j];       // negative gain

    target_sum = y0 + y1 + y2;
    raw_sum = x0 + x1 + x2;
    raw_sqr = x0*x0 + x1*x1 + x2*x2;
    m = x0*y0 + x1*y1 + x2*y2;
    m = 3*m - raw_sum*target_sum;
    m /= (3*raw_sqr - raw_sum*raw_sum);
    b = (target_sum - m*raw_sum)/3.;

    table_AIN[3][j].slope = m;   // slope
    table_AIN[3][j].offset = b;  // intercept
  }

  // claim all the needed interfaces for AInScan
  // libusb_set_auto_detach_kernel_driver(udev, 1);
  for (j = 1; j <= 6; j++) {
    libusb_detach_kernel_driver(udev, j);
    if ((ret = libusb_claim_interface(udev, j)) < 0) {
      perror("Error claiming interface");
    }
  }
  return;
}

#else

  /* Use 2 point calibration.  Only for older versions that don't have the negative
     calibration points stored in the EEPROM
  */
  /* Read in the internal ground reference at 0x90 in the EEPROM */
  addr = 0x90;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v0);
  y0 = v0*65536./20. + 0x8000;  //Calculate the corresponding calibrated value y0 

  /* Read in the internal reference for +/- 10V at 0x80 in the EEPROM  (+5.0 Nonminal) */
  addr = 0x80;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);
  y1 = v1*65536./20. + 0x8000;  //  Calculate the corresponding calibrated value y1

  addr = 0xb0;         // +/- 10V Uncalibrated readings
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 32, (uint8_t *) data);

  for (j = 0; j < NCHAN_USB1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    table_AIN[0][j].slope = (y1 - y0)/(x1 - x0);          // slope
    table_AIN[0][j].offset = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./10. + 0x8000;

  /* Read in the internal reference for +/- 5V at 0x84 in the EEPROM */
  addr = 0x84;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);
  y1 = v1*65536./10. + 0x8000;    // Calculate the corresponding calibrated value y1.

  addr = 0xd0;         // +/- 5V Uncalibrated readings
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 32, (uint8_t *) data);

  for (j = 0; j < NCHAN_USB1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    table_AIN[1][j].slope = (y1 - y0)/(x1 - x0);          // slope
    table_AIN[1][j].offset = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./4. + 0x8000;

  /* Read in the internal reference for +/- 2V at 0x88 in the EEPROM */
  addr = 0x88;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);
  y1 = v1*65536./4. + 0x8000;    // Calculate the corresponding calibrated value y1.

  addr = 0xf0;         // +/- 2V Uncalibrated readings
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 32, (uint8_t *) &data);

  for (j = 0; j < NCHAN_USB1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    table_AIN[2][j].slope = (y1 - y0)/(x1 - x0);          // slope
    table_AIN[2][j].offset = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }

  /**************************************************/  

  /* Calculate the corresponding calibrated value y0 */
  y0 = v0*65536./2. + 0x8000;
     
  /* Read in the internal reference for +/- 1V at 0x8c in the EEPROM */
  addr = 0x8c;
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 4, (uint8_t *) &v1);
  y1 = v1*65536./2. + 0x8000;   // Calculate the corresponding calibrated value y1.

  addr = 0x110;         // +/- 1V Uncalibrated readings
  usbReadMemory_USB1608FS(udev, addr, EEPROM, 32, (uint8_t *) &data);

  for (j = 0; j < NCHAN_USB1608FS; j++) {
    x0 = data[2*j];      // offset
    x1 = data[2*j+1];    // positive gain
    table_AIN[3][j].slope = (y1 - y0)/(x1 - x0);          // slope
    table_AIN[3][j].offset = (y0*x1 - y1*x0)/(x1 - x0);  // intercept
  }

  // claim all the needed interfaces for AInScan
  for (j = 1; j <= 6; j++) {
    libusb_detach_kernel_driver(udev, j);
    libusb_claim_interface(udev, j);
  }

  return;
}

#endif

/***********************************************
 *            Digital I/O                      *
 ***********************************************/

/* configures digital port */
void usbDConfigPort_USB1608FS(libusb_device_handle *udev, uint8_t direction)
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
    perror("Error in usbDConfigPort_USB1608FS: libusb_control_transfer error");
  }
}

/* configures digital bit */
void usbDConfigBit_USB1608FS(libusb_device_handle *udev, uint8_t bit_num, uint8_t direction)
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
    perror("Error in usbConfigBit_USB1608FS: libusb_control_transfer error");
  }
}

/* reads digital port  */
void usbDIn_USB1608FS(libusb_device_handle *udev, uint8_t* value)
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
    perror("Error in usbDIn_USB1608FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN  | 2, (unsigned char*) &read_port, sizeof(read_port), &transferred, FS_DELAY);
  *value = read_port.value;
  return;
}

/* reads digital bit  */
void usbDInBit_USB1608FS(libusb_device_handle *udev, uint8_t bit_num, uint8_t* value)
{
  int transferred;

  struct read_bit_t {
    uint8_t reportID;
    uint8_t value;  // contains bit_number on send and value on receive.
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
    perror("Error in usbDInBit_USB1608FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char*) &read_bit, sizeof(read_bit), &transferred, FS_DELAY);
  *value = read_bit.value;
  return;
}

/* writes digital port */
void usbDOut_USB1608FS(libusb_device_handle *udev, uint8_t value)
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
    perror("Error in usbDOut_USB1608FS: libusb_control_transfer error");
  }
}

/* writes digital bit  */
void usbDOutBit_USB1608FS(libusb_device_handle *udev, uint8_t bit_num, uint8_t value)
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
    perror("Error in usbDOutBit_USB1608FS: libusb_control_transfer error");
  }

  return;
}

/* reads from analog in */
int16_t usbAIn_USB1608FS(libusb_device_handle *udev, uint8_t channel, uint8_t range, Calibration_AIN table_AIN[NGAINS_USB1608FS][NCHAN_USB1608FS])
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
  if (channel > NCHAN_USB1608FS - 1) {
    printf("usbAIN: channel out of range for differential mode.\n");
    return -1;
  }
  if (range > 7) {
    printf("usbAIN: range setting too large.\n");
    return -1;
  }
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &ain, sizeof(ain), 5000);
  if (ret < 0) {
    perror("Error in usbAIn_USB1608FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2,(unsigned char*) report, sizeof(report), &transferred, FS_DELAY);

  data = (uint16_t) ( report[1] | (report[2] << 8));
  
  /* apply calibration correction 
        slope:  m = table_AIN[gain][channel].slope
        offset: b = table_AIN[gain][channel].offset
        correction = m*raw_value + b
  */

  switch (range) {
    case  BP_10_00V:
      value = (int) (table_AIN[0][channel].slope*((float) data) + table_AIN[0][channel].offset);
      break;
    case BP_5_00V:
      value = (int) (table_AIN[1][channel].slope*((float) data) + table_AIN[1][channel].offset);
      break;
    case BP_2_50V:  // use data from 2 V
    case BP_2_00V:
      value = (int) (table_AIN[2][channel].slope*((float) data) + table_AIN[2][channel].offset);
      break;
    case BP_1_25V:  // use data from 1 V
    case BP_1_00V:
    case BP_0_625V:
    case BP_0_3125V:
      value = (int) (table_AIN[3][channel].slope*((float) data) + table_AIN[3][channel].offset);
      break;
    default:
      break;
  }

  if (value >= 0x8000) {
    value -=  0x8000;
  } else {
    value = (0x8000 - value);
    value *= (-1);
  }

  return value;
}

void usbAInStop_USB1608FS(libusb_device_handle *udev)
{
  uint8_t reportID = AIN_STOP;;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | AIN_STOP;  // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbAInStop_USB1608FS: libusb_control_transfer error");
  }
}

int usbAInScan_USB1608FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t nSamples,
			 float *frequency, uint8_t options, int16_t sdata[], Calibration_AIN table_AIN[NGAINS_USB1608FS][NCHAN_USB1608FS],
			 uint8_t gainArray[NCHAN_USB1608FS])
{
  /*
    This command scans a range of analog input channels and sends the
     readings in inerrupt transfers.  The gain ranges that are currently
     set on the desired channels will be used (these may be changed
     with AIn or ALoadQueue

      lowchannel:   the first channel of the scan (0-7)
      highchannel:  the last channel of the scan (0-7)
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
    USB-1608FS devices, or to sample data using an external clock.
    The device must be set to be a sync slave with the usbSetSync
    command prior to using this mode.  Data will be acquired on all
    specified channels when the sync edge is detected.
   */

  int i, k;
  int nScans;
  int pipe;
  uint32_t preload;
  uint32_t nscans;
  int nchan;
  int transferred;
  int timeout = FS_DELAY;

  struct data_t{
    uint16_t value[31];        // 31 16-bit samples
    uint16_t scan_index;       //  1 16 bit scan count
  } data;
    
  struct arg_t {
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

  if (highchannel > 7) {
    printf("usbAInScan: highchannel out of range.\n");
    return -1;
  }

  if (lowchannel > 7) {
    printf("usbAInScan: lowchannel out of range.\n");
    return -1;
  }

  if (lowchannel > highchannel) {
    printf("usbAInScan: lowchannel greater than highchannel.\n");
    return -1;
  }

  nchan = highchannel - lowchannel + 1;  // total number of channels in a scan.
  nscans = nSamples;                     // total number of scans
  nSamples *= nchan;                     // total number of samples
  
  if (options & AIN_TRIGGER) {
    timeout = 0;  // set wait forever
  }

  arg.reportID = AIN_SCAN;
  arg.lowchannel = lowchannel;
  arg.highchannel = highchannel;
  arg.count[0] = (uint8_t) nscans & 0xff;           // low byte
  arg.count[1] = (uint8_t) (nscans >>  8) & 0xff;
  arg.count[2] = (uint8_t) (nscans >> 16) & 0xff;
  arg.count[3] = (uint8_t) (nscans >> 24) & 0xff;   // high byte
  arg.options = options;

  for (arg.prescale = 0; arg.prescale <= 8; arg.prescale++) {
    preload = 1.0e7/((*frequency) * (1<<arg.prescale));
    if (preload <= 0xffff) {
      arg.preload[0] = (uint8_t) preload & 0xff;          // low byte
      arg.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
      break;
    }
  }

  if (arg.prescale == 9 || preload == 0) {
    printf("usbAInScan_USB1608FS: frequency out of range.\n");
    return -1;
  }

  *frequency = 1.0e7/(preload*(1<<arg.prescale));
  nScans = nSamples / 31 + 1;
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 500);
  if (ret < 0) {
    perror("Error in usbAInScan_USB1608FS: libusb_control_transfer error");
  }

  pipe = 1;  // Initial Endpoint to receive data.
  for (i = 0; i < nScans; i++, pipe = (pipe)%6 + 1) {  //pipe should take the values 1-6
    ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN |(pipe+2), (unsigned char *) &data, sizeof(data), &transferred, timeout);
    if (ret < 0) {
      perror("usbAInScan_USB1608FS interrupt pipes");
    }
    if (i != data.scan_index) {
      printf("AInScan_USB1608FS: ret = %d  scan = %d    nSamples = %d       scan_index = %d  Endpoint = %#x  transferred = %d\n",
    	   ret, i, nSamples, data.scan_index, LIBUSB_ENDPOINT_IN |(pipe+2), transferred); 
    }

    if (nSamples <= 31) {  // only collect partial scan
      for (k = 0; k < nSamples; k++) {
	if (data.value[k] >= 0x8000) {
	  sdata[data.scan_index*31+k] = (data.value[k] - 0x8000);
	} else {
	  sdata[data.scan_index*31+k] = (0x8000 - data.value[k]);
	  sdata[data.scan_index*31+k] *= (-1);
	}
      }
      break;
    } else {     // put all 31 samples into buffer
      for (k = 0; k < 31;  k++) {
	if (data.value[k] >= 0x8000) {
	  sdata[data.scan_index*31+k] = (data.value[k] - 0x8000);
	} else {
	  sdata[data.scan_index*31+k] = (0x8000 - data.value[k]);
	  sdata[data.scan_index*31+k] *= (-1);
	}
      }
      nSamples -= 31;
    }
  }

  usbAInStop_USB1608FS(udev);
  return nscans;
}


void usbAInLoadQueue_USB1608FS(libusb_device_handle *udev, uint8_t gainArray[8])
{
  struct loadQueue_t{
    uint8_t reportID;
    uint8_t gainArray[8];
  } loadQueue;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                    // HID Set_Report
  uint16_t wValue = (2 << 8) | ALOAD_QUEUE; // HID ouptut
  uint16_t wIndex = 0;                      // Interface

  loadQueue.reportID = ALOAD_QUEUE;
  memcpy(loadQueue.gainArray, gainArray, 8);
  
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &loadQueue, sizeof(loadQueue), 5000);
  if (ret < 0) {
    perror("Error in usbAInLoadQueue_USB1608FS: libusb_control_transfer error");
  }
}

/* Initialize the counter */
void usbInitCounter_USB1608FS(libusb_device_handle *udev)
{
  uint8_t reportID = CINIT;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;               // HID Set_Report
  uint16_t wValue = (2 << 8) | CINIT;  // HID ouptut
  uint16_t wIndex = 0;                 // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbInitCounter_USB1608FS: libusb_control_transfer error");
  }
}

uint32_t usbReadCounter_USB1608FS(libusb_device_handle *udev)
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
    perror("Error in usbCounter_USB1608FS: libusb_control_transfer error");
  }

  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &counter, sizeof(counter), &transferred, FS_DELAY);
  value =   counter.value[0] | (counter.value[1] << 8) |
    (counter.value[2] << 16) | (counter.value[3] << 24);

  return value;
}

/* blinks the LED of USB device */
void usbBlink_USB1608FS(libusb_device_handle *udev)
{
  uint8_t reportID = BLINK_LED;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | BLINK_LED; // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbBlink_USB1608FS: libusb_control_transfer error");
  }
}

int usbReset_USB1608FS(libusb_device_handle *udev)
{
  int ret;
  uint8_t reportID = RESET;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | RESET;   // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbReset_USB1608FS: libusb_control_transfer error");
  }
  return 0;
}

void usbSetTrigger_USB1608FS(libusb_device_handle *udev, uint8_t type)
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
    perror("Error in usbSetTriggert_USB1608FS: libusb_control_transfer error");
  }
}

void usbSetSync_USB1608FS(libusb_device_handle *udev, uint8_t type)
{
  /*
    This command configures the sync signal.  The sync signal may be
    used to synchronize the analog input scan of multiple PMD-160FS
    devices.  When multiple devices are to be used, one device is
    selected as the master and the rest as slaves.  The sync signal of
    all devices must be wired together.  The master will output a
    pulse every sample, and all of the devices will acquire their
    samples simultaneously. This may also be used to pace one or more
    PMD-1608 devices from an external TTL/CMOS clock signal (max rate = 50 kHz).

    This may also be used with an external trigger.  The external
    trigger signal should be brought to the master device, and all
    devices will begin sampling when the master is triggered.

    If a device is configured as a slave, it will not acquire data
    given an AInScan command until it detects a pulse on the sync
    input.
   */

  uint8_t cmd[2];
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | SET_SYNC;  // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  cmd[0] = SET_SYNC;
  cmd[1] = type;   // 0 = master, 1 = slave
  
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &cmd, sizeof(cmd), 5000);
  if (ret < 0) {
    perror("Error in usbSetSync_USB1608FS: libusb_control_transfer error");
  }
}

uint16_t usbGetStatus_USB1608FS(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device.

    Bit 0: 0 = Sync slave,   1 = sync master
    Bit 1: 0 = trigger falling edge,  1 = trigger rising edge
    Bit 2-14:  Not used
    Bit 15:   1 = program memory update mode
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
    perror("Error in usbGetStatus_USB1608FS: libusb_control_transfer error");
  }

  do {
    statusReport.reportID = 0;
    libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &statusReport, sizeof(statusReport), &transferred, FS_DELAY);
  } while ( statusReport.reportID != GET_STATUS);
  status = (uint16_t) (statusReport.status[0] | (statusReport.status[1] << 8));

  return status;
}

void usbReadMemory_USB1608FS(libusb_device_handle *udev, uint16_t address, uint8_t type, uint8_t count, uint8_t* memory)
{
  // Addresses 0x000 - 0x07F are reserved for firmware data
  // Addresses 0x080 - 0x3FF are available for use as calibration or user data

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
    uint8_t pad[2];
  } memRead;

  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | MEM_READ;  // HID ouptut
  uint16_t wIndex = 0;                    // Interface

  if (count > 62) count = 62;
  arg.reportID = MEM_READ;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte
  arg.type = type;
  arg.count = count;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
  if (ret < 0) {
    perror("Error in usbReadMemory_USB1608FS: libusb_control_transfer error");
  }
  
  memRead.reportID = 0x0;
  // always read 63 bytes regardless.  Only the first count are meaningful.
  ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &memRead, 63, &transferred, FS_DELAY);
  if (ret < 0) {
    perror("Error in usbReadMemory_USB1608FS: usb_interrupt_read() reading error.");
    printf("Address = %#x  Count = %d  Number of bytes read = %d \n", address, count, transferred);
  }
  if (memRead.reportID != MEM_READ) {
    printf("Error in Reading Memory from EEPROM!\n");
    libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
    libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &memRead, count+1, &transferred, FS_DELAY);
  }
  memcpy(memory, &memRead.data[0], count);
}

int usbWriteMemory_USB1608FS(libusb_device_handle *udev, uint16_t address, uint8_t count, uint8_t* data)
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

  if (address <= 0x7f) return -1;
  if (count > 59) count = 59;

  arg.reportID = MEM_WRITE;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte

  arg.count = count;
  for ( i = 0; i < count; i++ ) {
    arg.data[i] = data[i];
  }
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
  if (ret < 0) {
    perror("Error in usbWriteMemory_USB1608FS: libusb_control_transfer error");
  }
  return 0;
}

void usbGetAll_USB1608FS(libusb_device_handle *udev, uint8_t data[])
{
  int transferred;
  uint8_t reportID = GET_ALL;

  struct get_all_t {
    uint8_t reportID;
    uint8_t values[19];
  } get_all;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                 // HID Set_Report
  uint16_t wValue = (2 << 8) | GET_ALL;  // HID ouptut
  uint16_t wIndex = 0;                   // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbGetAll_USB1608FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &get_all, sizeof(get_all), &transferred, FS_DELAY);
  memcpy(data, get_all.values, 19);
  return;
}

float volts_USB1608FS( const int gain, const signed short num )
{
  float volt = 0.0;
  
  switch( gain ) {
    case BP_10_00V:
      volt = num * 10.0 / 0x7fff;
      break;
    case BP_5_00V:
      volt = num * 5.00 / 0x7fff;
      break;
    case BP_2_50V:
      volt = num * 2.50 / 0x7fff;
      break;
    case BP_2_00V:
      volt = num * 2.00 / 0x7fff;
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
