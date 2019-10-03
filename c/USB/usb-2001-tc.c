/*
 *
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@ncsu.edu>
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
#include "usb-2001-tc.h"

#define HS_DELAY 2000

static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant. 

// Globals
Thermocouple_Data ThermocoupleData[8];
double TypeKReverseExtra[3];


void usbBuildGainTable_USB2001TC(libusb_device_handle *udev, float table[NGAINS_USB2001_TC][2])
{
  /* Builds a lookup table of calibration coefficents to translate values into voltages:
     The calibration coefficients are stored in onboard FLASH memory on the device in
     IEEE-754 4-byte floating point values.

     calibrated code = code * slope + intercept
  */

  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
}

void usbAIn_USB2001TC(libusb_device_handle *udev, uint32_t *data)
{
  /*
    This command reads the latest ADC value.  Note, the reading of bad/stale data is possible
    if the device is BUSY, or ERROR, refer to the STATUS of the device for further details or
    use GetAll.  Apply CJC, CJC Gradient, ADC Slope & Offset and account for the ADC resolution
    in to the real temperature calculation.  For RAW voltages, use only the resolution and range 
    used.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  ret = libusb_control_transfer(udev, requesttype, AIN, 0x0, 0x0, (unsigned char *) data, 4, HS_DELAY);
  if (ret < 0) {
    perror("usbAIN_USB2001TC Error in reading raw data.");
  }
  return;
}

void usbGetAll_USB2001TC(libusb_device_handle *udev, TC_data data)
{
  /*
    This command reads all of the sensor parameters required to make a temperature measurement or
    to ensure a valid RAW reading is performed.  The CJC value does not apply the CJC Gradient or
    convert it to degrees C.  The following conversions are required:
    
    CJC Temperature = (CJC Value / 2^15) * 128 - (CJC Gradient)
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;
  uint8_t values[7];

  ret = libusb_control_transfer(udev, requesttype, GET_ALL, 0x0, 0x0, (unsigned char *) values, 7, HS_DELAY);
  if (ret < 0) {
    perror("usbGetAll_USB2001TC Error in reading raw data.");
  }

  memcpy(&data.status,    &values[0], 1);
  memcpy(&data.CJC,       &values[1], 2);
  memcpy(&data.ADC_Value, &values[3], 4);
  
  return;
}

void usbMemoryR_USB2001TC(libusb_device_handle *udev, uint8_t count, uint8_t data[])
{
  /* 
     This command reads/writes data to the configuration memory (EEPROM).  Writes to
     the Clibration (Slope/Offset) data and Calibratin Date only provided when Unlock code
     (0xAA55) is written to memory address 0x400.  All other access is valid without unlock.
     Any other value written to the Memory Address will re-lock the Calibration memory.
 */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 64) {
    printf("usbMemoryR_USB2001TC: count = %d.  Max count is 64.\n", count);
    return;
  }
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbMemoryW_USB2001TC(libusb_device_handle *udev, uint8_t count, uint8_t data[])
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
 
  if (count > 64) {
    printf("usbMemoryW_USB2001TC: count = %d.  Max count is 64.\n", count);
    return;
  }
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, count, HS_DELAY);
}

void usbMemoryUnlock_USB2001TC(libusb_device_handle *udev)
{
  /* unlock the memory by writing the value 0xAA55 to memory location 0x400 */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t unlock = 0xAA55;
  
  usbSetMemoryAddress_USB2001TC(udev, 0x400);
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) &unlock, sizeof(unlock), HS_DELAY);
}

void usbMemoryLock_USB2001TC(libusb_device_handle *udev)
{
  /* unlock the memory by writing the value 0xAA55 to memory location 0x400 */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t lock = 0x1;
  uint16_t address = 0x400;
  
  usbSetMemoryAddress_USB2001TC(udev, address);
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) &lock, sizeof(lock), HS_DELAY);
}

void usbSetMemoryAddress_USB2001TC(libusb_device_handle *udev, uint16_t address) 
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEMADDRESS, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY);
}

void usbGetMemoryAddress_USB2001TC(libusb_device_handle *udev, uint16_t *address) 
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEMADDRESS, 0x0, 0x0, (unsigned char *) address, sizeof(address), HS_DELAY);
}

void usbBlink_USB2001TC(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return;
}

void usbReset_USB2001TC(libusb_device_handle *udev, uint8_t type)
{
  /*
    This command causes the device to perform a reset.  The device disconnects from the
    USB bus and resets its microcontroller.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, (unsigned char *) &type, sizeof(type), HS_DELAY);
  return;
}

void usbGetSerialNumber_USB2001TC(libusb_device_handle *udev, char serial[9])
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

void usbSetSerialNumber_USB2001TC(libusb_device_handle *udev, char serial[9])
{
  /*
    This commands writes the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). 
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, SERIAL, 0x0, 0x0, (unsigned char *) serial, 8, HS_DELAY);
  serial[8] = '\0';
  return;
}

void cleanup_USB2001TC(libusb_device_handle *udev)
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|1);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}

void setVoltageRange_USB2001TC(libusb_device_handle *udev, int range)
{
  char message[MAX_MESSAGE_LENGTH];

  switch (range) {
    case 3:
      strcpy(message, "AI{0}:RANGE=BIP146.25E-3V");
      break;
    case 4:
      strcpy(message, "AI{0}:RANGE=BIP73.125E-3V");
      break;
    default:
      strcpy(message, "AI{0}:RANGE=BIP146.25E-3V");
      break;
  }
  sendStringRequest(udev, message);
}

void getVoltageRange_USB2001TC(libusb_device_handle *udev, int *range)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?AI{0}:RANGE");
  getStringReturn(udev, message);

  if (strcmp(message, "AI{0}:RANGE=BIP146.25E-3V") == 0) {
    *range = 3;
    return;
  } else if (strcmp(message, "AI{0}:RANGE=BIP73.125E-3V") == 0) {
    *range = 4;
  } else {
    *range = -1;  //invalid range
    printf("%s\n", message);
  }
}

void sendSensorType_USB2001TC(libusb_device_handle *udev, char type)
{
  char message[MAX_MESSAGE_LENGTH];
  int i;
  int ret;
  
  sprintf(message, "AI{0}:SENSOR=TC/%c", type);
  for (i = 0; i < 5; i++) {
    if ((ret = sendStringRequest(udev, message)) >= 0) break;
  }
  if (ret < 0) {
    //    perror("sendSensorType_USB2001TC");
  }
}

void getSensorType_USB2001TC(libusb_device_handle *udev, char *type)
{
  char message[MAX_MESSAGE_LENGTH];
  int i;
  int ret;

  for (i = 0; i < 5; i++) {
    if ((ret = sendStringRequest(udev, "?AI{0}:SENSOR")) >= 0) break;
  }
  if (ret < 0) {
    perror("getSensorType_USB2001TC");
  }
  
  for (i = 0; i < 5; i++) {
    if ((ret = getStringReturn(udev, message))  >= 0) break;
  }
  if (ret < 0) {
    perror("getSensorType_USB2001TC");
  }

  *type = message[16];
}

int getStatus_USB2001TC(libusb_device_handle *udev)
{
  char message[MAX_MESSAGE_LENGTH];
  int status;

  sendStringRequest(udev, "?AI{0}:STATUS");
  getStringReturn(udev, message);
  if (strcmp(&message[13], "READY") == 0) {
    status = READY;
  } else if (strcmp(&message[13], "BUSY") == 0) {
    status = BUSY;
  } else if (strcmp(&message[13], "ERROR") == 0) {
    status = ERROR;
  } else {
    status = UNKNOWN;
  }
  return status;
}

void getCJC_USB2001TC(libusb_device_handle *udev, double *CJC_temperature)
{
  char message[MAX_MESSAGE_LENGTH];

  while(getStatus_USB2001TC(udev) == BUSY) {
    usleep(10000);
  }
  sendStringRequest(udev, "?AI{0}:CJC");
  getStringReturn(udev, message);
  *CJC_temperature = atof(&message[15]);
}

void getCJCDegC_USB2001TC(libusb_device_handle *udev, double *CJC_temperature)
{
  char message[MAX_MESSAGE_LENGTH];
  int i;
  int ret;

  for (i = 0; i < 5; i++) {
    if ((ret = sendStringRequest(udev, "?AI{0}:CJC/DEGC")) >= 0) break;
  }
  if (ret < 0) {
    perror("getCJCDegC_USB2001TC");
  }

  for (i = 0; i < 5; i++) {
    if ((ret = getStringReturn(udev, message)) >= 0) break;
  }
  if (ret < 0) {
    perror("getCJCDegC_USB2001TC");
  }
  
  *CJC_temperature = atof(&message[15]);
}

void getCJCDegF_USB2001TC(libusb_device_handle *udev, double *CJC_temperature)
{
  char message[MAX_MESSAGE_LENGTH];
  int i;
  int ret;

  for (i = 0; i < 5; i++) {
    if ((ret = sendStringRequest(udev, "?AI{0}:CJC/DEGF")) >= 0) break;
  }
  if (ret < 0) {
    perror("getCJCDegF_USB2001TC");
  }

  for (i = 0; i < 5; i++) {
    if ((ret = getStringReturn(udev, message)) >= 0) break;
  }
  if (ret < 0) {
    perror("getCJCDegF_USB2001TC");
  }
  
  *CJC_temperature = atof(&message[15]);
}

void getCJCDegKelvin_USB2001TC(libusb_device_handle *udev, double *CJC_temperature)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?AI{0}:CJC/KELVIN");
  getStringReturn(udev, message);
  *CJC_temperature = atof(&message[17]);
}

void getSlope_USB2001TC(libusb_device_handle *udev, double *slope)
{
  char message[MAX_MESSAGE_LENGTH];
  char value[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?AI{0}:SLOPE");
  getStringReturn(udev, message);
  strncpy(value, &message[12], 8);
  *slope = atof(value);

  if (*slope == 0.0) {
    printf("getSlope_USB2001TC: check calibration constants in EEPROM.\n");
    *slope = 1;
  }
}

void getOffset_USB2001TC(libusb_device_handle *udev, double *offset)
{
  char message[MAX_MESSAGE_LENGTH];
  char value[10];

  sendStringRequest(udev, "?AI{0}:OFFSET");
  getStringReturn(udev, message);
  strncpy(value, &message[13], 8);
  *offset = atof(value);
}

void getFirmwareVersion_USB2001TC(libusb_device_handle *udev, char *version)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?DEV:FWV");
  getStringReturn(udev, message);
  strncpy(version, &message[8], 12);
}

void getMFGCAL_USB2001TC(libusb_device_handle *udev, struct tm *date)
{
  /* Manufactuering Calibration Date. */

  //  char message[MAX_MESSAGE_LENGTH];
  //  sendStringRequest(udev, "?DEV:MFGCAL");
  //  getStringReturn(udev, message);
  //  printf("%s\n", message);
  time_t time;

  date->tm_year = getMFGCALYear_USB2001TC(udev) - 1900;
  date->tm_mon = getMFGCALMonth_USB2001TC(udev) - 1;
  date->tm_mday = getMFGCALDay_USB2001TC(udev);
  date->tm_hour = getMFGCALHour_USB2001TC(udev);
  date->tm_min = getMFGCALMinute_USB2001TC(udev);
  date->tm_sec = getMFGCALSecond_USB2001TC(udev);

  time = mktime(date);
  date = localtime(&time);
}

int getMFGCALYear_USB2001TC(libusb_device_handle *udev)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?DEV:MFGCAL{YEAR}");
  getStringReturn(udev, message);
  return atoi(&message[17]);
}

int getMFGCALMonth_USB2001TC(libusb_device_handle *udev)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?DEV:MFGCAL{MONTH}");
  getStringReturn(udev, message);
  return atoi(&message[18]);
}

int getMFGCALDay_USB2001TC(libusb_device_handle *udev)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?DEV:MFGCAL{DAY}");
  getStringReturn(udev, message);
  return atoi(&message[16]);
}

int getMFGCALHour_USB2001TC(libusb_device_handle *udev)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?DEV:MFGCAL{HOUR}");
  getStringReturn(udev, message);
  return atoi(&message[17]);
}

int getMFGCALMinute_USB2001TC(libusb_device_handle *udev)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?DEV:MFGCAL{MINUTE}");
  getStringReturn(udev, message);
  return atoi(&message[19]);
}

int getMFGCALSecond_USB2001TC(libusb_device_handle *udev)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?DEV:MFGCAL{SECOND}");
  getStringReturn(udev, message);
  return atoi(&message[19]);
}

void resetDefault_USB2001TC(libusb_device_handle *udev)
{
  sendStringRequest(udev, "?DEV:RESET/DEFAULT");
}

void reset_USB2001TC(libusb_device_handle *udev)
{
  sendStringRequest(udev, "?DEV:RESET/SYSTEM");
}

int getValue_USB2001TC(libusb_device_handle *udev, uint32_t *value)
{
  char message[MAX_MESSAGE_LENGTH];

  sendStringRequest(udev, "?AI{0}:VALUE");
  getStringReturn(udev, message);
  
  *value = atoi(&message[12]);
  return 0;
}

int tc_temperature_USB2001TC(libusb_device_handle *udev, char tc_type, double *temperature)
{
  uint32_t value = 0x0;        // integer value of the temperature
  double tc_voltage;
  double CJC_Temp;
  double slope, offset;


  // Set the voltage range (Mode = 4, Range = +/- .078125V)
  //setVoltageRange_USB2001TC(udev, 4);

  // Get Slope and Offset
  getSlope_USB2001TC(udev, &slope);
  getOffset_USB2001TC(udev, &offset);

  // Apply calibration slope and offset 
  if (getValue_USB2001TC(udev, &value) < 0) return -1;
  value = value*slope + offset;

  // Calculate the TC voltage (in mV) from the corrected values
  tc_voltage = ((value - 524288.)/524288.) * 73.125;

  // Read the CJC value in Celsius
  getCJCDegC_USB2001TC(udev, &CJC_Temp);

  // Calculate the CJC voltage using the NIST polynomials and add to tc_voltage in millivolts
  tc_voltage = NISTCalcVoltage(tc_type, CJC_Temp) + tc_voltage;

  // Calcualate actual temperature using reverse NIST polynomial.
  *temperature = NISTCalcTemp(tc_type, tc_voltage);

  return 0;
}

  










