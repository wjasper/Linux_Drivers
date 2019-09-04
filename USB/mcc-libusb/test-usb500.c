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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <ctype.h>
#include <math.h>
#include "usb-500.h"

static unsigned long startTimeOffset = 0;

float celsius2fahr( float celsius )
{
  return (celsius*9.0/5.0 + 32.);
}

float fahr2celsius( float fahr )
{
  return (fahr - 32.)*5.0/9.0;
}

void cleanup_USB500( libusb_device_handle *udev )
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|2);
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT|2);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}

libusb_device_handle* usb_device_find_USB500( int vendorId, int productId )
{
  struct libusb_device_handle *udev = NULL;
  int ret = 0;

  libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB500: Failed to initialize libusb");
    exit(1);
  }
  udev = libusb_open_device_with_vid_pid(NULL, vendorId, productId);
  if (udev == NULL) {
    return 0;
  }
  printf("Vendor ID = %#x    Product ID = %#x\n", vendorId, productId);
  
  ret = libusb_set_configuration(udev, 1);
  if (ret < 0) {
    perror("Error setting configuration\n");
    goto out;
  }
  
  /* claim interface */
  ret = libusb_claim_interface(udev, 0);
  if (ret < 0) {
    perror("Error claiming usb interface 0\n");
    goto out_release;
  }
  
  return udev;

 out_release:
  libusb_release_interface(udev,0);
 out:
  libusb_close(udev);
  libusb_exit(NULL);
  exit(0);
}

void unlock_device_USB500( libusb_device_handle *udev )
{
  int requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int request = 0x02;
  int index = 0x0;
  int value = 0x2;
  int size = 0x0;
  unsigned char payload[8];
  int timeout = 0;

  libusb_control_transfer(udev, requesttype, request, value, index, payload, size, timeout);
}

void lock_device_USB500(libusb_device_handle *udev)
{
  int requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  int request = 0x02;
  int index = 0x0;
  int value = 0x4;
  int size = 0x0;
  unsigned char payload[8];
  int timeout = 0;

  libusb_control_transfer(udev, requesttype, request, value, index, payload, size, timeout);
}

int read_configuration_block_USB500( libusb_device_handle *udev, configurationBlock *cblock )
{
  unsigned char request_configuration[3] = {0x00, 0xff, 0xff};  // Request Configuration String
  uint8_t acknowledge[3] =                 {0x00, 0x00, 0x00};  // Acknowledge string
  unsigned int size = 0;
  int ret;
  int try = 0;
  int transferred;

  unlock_device_USB500(udev);

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT | 2,  request_configuration, 3, &transferred, USB500_WAIT_WRITE);
  if (ret < 0) {
    perror("Error in requesting configuration (Bulk Write)");
    lock_device_USB500(udev);
    return 0;
  }

  acknowledge[0] = 0x0;
  do {
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|2, (unsigned char *) acknowledge, 3, &transferred, USB500_WAIT_READ);
    if (ret < 0) {
      perror("Error in acknowledging configuration from USB 500");
      printf("Bytes transferred = %d\n", transferred);
      lock_device_USB500(udev);
      return 0;
    }
    if (transferred == 3 && acknowledge[0] != 0x2) {
      printf("read_configuration_block_USB500: try = %d  first byte = %x\n", try, acknowledge[0]);
      libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT | 2,  (unsigned char *) request_configuration, 3, &transferred, USB500_WAIT_WRITE);
      try++;
    }
    if (try > 10) {
      printf("read_configuration_block_USB500: try = %d  Stopping now \n", try);
      lock_device_USB500(udev);
      return 0;
      exit(-1);
    }
  } while (acknowledge[0] != 0x2);

  size = ((acknowledge[2] << 8) | acknowledge[1]);
  do {
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) cblock, size, &transferred, USB500_WAIT_READ);
    if (ret < 0) {
      perror("Error in reading configuration block from USB 500");
      printf("Size = %d   transferred = %d\n", size, transferred);
    }
  } while (transferred != size);

  lock_device_USB500(udev);

  return cblock->type;
}

void write_configuration_block_USB500( libusb_device_handle *udev, configurationBlock *cblock )
{
  uint8_t send_configuration[3] = {0x01, 0xff, 0xff}; // Send Configuration String
  uint8_t acknowledge = 0x0;                          // Acknowledge 
  unsigned int size = 0;
  int ret;
  int transferred;

  /* Determine the size of the Configuration Block */
  switch(cblock->type) {
      case 0x1:                      // EL-USB-1 Temperature Logger Packet size 64 bytes
      case 0x2:                      // EL-USB-1 Temperature Logger Packet size 64 bytes
      send_configuration[1] = 0x40;  // Low Byte
      send_configuration[2] = 0x00;  // High Byte
      size = 64;
      break;
    
    case 0x3:                        // EL-USB-2 Temperature/Humidity Logger Packet size 128 bytes
      send_configuration[1] = 0x80;  // Low Byte
      send_configuration[2] = 0x00;  // High Byte
      size = 128;
      break;
    
    case 0x4:                        // EL-USB-3 Voltage Logger 256 bytes
    case 0x6:                        // EL-USB-3 Voltage Logger 256 bytes
    case 0x5:                        // EL-USB-4 Current Logger 256 bytes
    case 0x7:                        // EL-USB-4 Current Logger 256 bytes
      send_configuration[1] = 0x00;  // Low Byte
      send_configuration[2] = 0x01;  // High Byte
      size = 256;
      break;
    
  default:
    printf("Unknown type device = %d\n", cblock->type);
    return;
    break;
  }

  unlock_device_USB500(udev);

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT | 2,  (unsigned char *) send_configuration, 3, &transferred, USB500_WAIT_WRITE);
  if (ret < 0) {
    perror("Error in sending configuration acknowledgement (Bulk Write)");
  }

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT | 2,  (unsigned char *) cblock, size, &transferred, USB500_WAIT_WRITE);
  if (ret < 0) {
    perror("Error in sending configuration block (Bulk Write)");
  }

  acknowledge = 0x0;
  do {
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|2, (unsigned char *) &acknowledge, 1, &transferred, USB500_WAIT_READ);
  } while (acknowledge != 0xff);

  lock_device_USB500(udev);
}

void stop_logging_USB500( libusb_device_handle *udev, configurationBlock *cblock )
{
  read_configuration_block_USB500(udev, cblock);
  cblock->flagBits &= ~(LOGGING_STATE);
  startTimeOffset = cblock->startTimeOffset;
  cblock->startTimeOffset = 0;
  write_configuration_block_USB500(udev, cblock);
}

void start_logging_USB500( libusb_device_handle *udev, configurationBlock *cblock )
{
  time_t currentTime;
  struct tm *tp;

  read_configuration_block_USB500(udev, cblock);
  cblock->flagBits |= (LOGGING_STATE);
  cblock->startTimeOffset = cblock->startTimeOffset;

  time(&currentTime);
  tp = localtime(&currentTime);	
  cblock->startTimeHours = tp->tm_hour;
  cblock->startTimeMinutes = tp->tm_min;
  cblock->startTimeSeconds = tp->tm_sec;
  cblock->startTimeDay = tp->tm_mday;
  cblock->startTimeMonth = tp->tm_mon + 1;    // months from 1-12
  cblock->startTimeYear = tp->tm_year - 100;  // years start from 2000 not 1900.

  write_configuration_block_USB500(udev, cblock);
}

void set_alarm( libusb_device_handle *udev, configurationBlock *cblock )
{
  char ans[80];
  float temperature;
  float humidity;

  read_configuration_block_USB500(udev, cblock);

  printf("Enable High Alarm? [y/n] ");
  scanf("%s", ans);
  if (ans[0] == 'y') {
    cblock->flagBits |= HIGH_ALARM_STATE;
  } else {
    cblock->flagBits &= ~HIGH_ALARM_STATE;
  }

  printf("Enable High Alarm Latch? [y/n] ");
  scanf("%s", ans);
  if (ans[0] == 'y') {
    cblock->flagBits |= HIGH_ALARM_LATCH;
  } else {
    cblock->flagBits &= ~HIGH_ALARM_LATCH;
  }

  if (cblock->flagBits & (HIGH_ALARM_STATE | HIGH_ALARM_LATCH)) {
    printf("Enter High Alarm Level (Temp): ");
    scanf("%f", &temperature);
    cblock->alarm1.channel1.highAlarm =
      (unsigned char) (temperature - cblock->calibrationCValue) / cblock->calibrationMValue;
  }

  printf("Enable Low Alarm? [y/n] ");
  scanf("%s", ans);
  if (ans[0] == 'y') {
    cblock->flagBits |= LOW_ALARM_STATE;
  } else {
    cblock->flagBits &= ~LOW_ALARM_STATE;
  }

  printf("Enable Low Alarm Latch? [y/n] ");
  scanf("%s", ans);
  if (ans[0] == 'y') {
    cblock->flagBits |= LOW_ALARM_LATCH;
  } else {
    cblock->flagBits &= ~LOW_ALARM_LATCH;
  }

  if (cblock->flagBits & (LOW_ALARM_STATE | LOW_ALARM_LATCH)) {
    printf("Enter Low Alarm Level (Temp): ");
    scanf("%f", &temperature);
    cblock->alarm1.channel1.lowAlarm =
      (unsigned char) (temperature - cblock->calibrationCValue) / cblock->calibrationMValue;
  }

  if (cblock->type == 3) {  // USB-502 only
    printf("Enable Channel 2 High Alarm? [y/n] ");
    scanf("%s", ans);
    if (ans[0] == 'y') cblock->flagBits |= CH2_HIGH_ALARM_STATE; 

    printf("Enable Channel 2 High Alarm Latch? [y/n] ");
    scanf("%s", ans);
    if (ans[0] == 'y') cblock->flagBits |= CH2_HIGH_ALARM_LATCH;

    if (cblock->flagBits & (CH2_HIGH_ALARM_STATE | CH2_HIGH_ALARM_LATCH)) {
      printf("Enter Channel 2 High Alarm Level (Humdity): ");
      scanf("%f", &humidity);
      cblock->alarm2.channel2.highAlarm = (unsigned char) (humidity - 0.0) / 0.5;
    }

    printf("Enable Channel 2 Low Alarm? [y/n] ");
    scanf("%s", ans);
    if (ans[0] == 'y') cblock->flagBits |= CH2_LOW_ALARM_STATE;

    printf("Enable Channel 2 Low Alarm Latch? [y/n] ");
    scanf("%s", ans);
    if (ans[0] == 'y') cblock->flagBits |= CH2_LOW_ALARM_LATCH;

    if (cblock->flagBits & (CH2_LOW_ALARM_STATE | CH2_LOW_ALARM_LATCH)) {
      printf("Enter Channel 2 Low Alarm Level (Humidity): ");
      scanf("%f", &humidity);
      cblock->alarm2.channel2.lowAlarm = (unsigned char) (humidity - 0.0) / 0.5;
    }
  }
  write_configuration_block_USB500(udev, cblock);
}

int read_recorded_data_USB501( libusb_device_handle *udev, configurationBlock *cblock, usb501_data *data_501 )
{
  int packet_size;
  int memory_size = 64;
  int num_packets = 0x4000;
  unsigned int size = 0;
  int ret;
  int transferred;
  int i;
  struct tm ltime;
  time_t currentTime;
  uint8_t request_data[3] = {0x03, 0xff, 0xff};  // Request Recorded Data
  uint8_t acknowledge[3] =  {0x00, 0x00, 0x00};  // Acknowledge string
  uint8_t rdata[0x4000];  // max data size

   time(&currentTime);
  localtime_r(&currentTime, &ltime);  // set daylight savings field

  stop_logging_USB500(udev, cblock);  // must stop logging before reading the data.

  switch(cblock->type) {
    case 0x1:                         // EL-USB-1 Temperature Logger Packet size 64 bytes
      packet_size = 64;
      memory_size = 0x4000;
      break;
    
    case 0x2:                         // EL-USB-1 Temperature Logger Packet size 512 bytes
      packet_size = 512;
      memory_size = 0x4000;
      break;

    default:
      printf("Unknown type device = %d\n", cblock->type);
      return -1;
      break;
  }

  unlock_device_USB500(udev);

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT | 2, (unsigned char *) request_data, 3, &transferred, USB500_WAIT_WRITE);
  if (ret < 0) {
    perror("Error in requesting configuration (Bulk Write)");
  }
  do {
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|2, (unsigned char *) acknowledge, 3, &transferred, USB500_WAIT_READ);
    if ( ret < 0 ) {
      perror("Error in acknowledging read data from USB 500");
    }
  } while (acknowledge[0] != 0x2);

  size = ((acknowledge[2] << 8) | acknowledge[1]);
  if (size != memory_size) {
    printf("Memory Error mismatch. size = %#x  should be %#x\n", size, memory_size);
    return -1;
  }

  num_packets = memory_size / packet_size;
  for ( i = 0; i < num_packets; i++ ) {
    do {
      ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &rdata[i*packet_size], packet_size, &transferred, USB500_WAIT_READ);
      if (ret < 0) {
	perror("Error reading data.  Retrying.");
      }
    } while (transferred != packet_size);
  }

  lock_device_USB500(udev);

  ltime.tm_sec = cblock->startTimeSeconds;
  ltime.tm_min = cblock->startTimeMinutes;
  ltime.tm_hour = cblock->startTimeHours;
  ltime.tm_mday = cblock->startTimeDay;
  ltime.tm_mon = cblock->startTimeMonth - 1;
  ltime.tm_year = cblock->startTimeYear + 100;
  data_501[0].time = mktime(&ltime);  // get local time stamp

  for (i = 0; i < cblock->sampleCount; i++) {
      data_501[i].time = data_501[0].time + i*cblock->sampleRate;
      data_501[i].temperature = cblock->calibrationMValue*rdata[i] + cblock->calibrationCValue;
  }

  start_logging_USB500(udev, cblock); 
  return cblock->sampleCount;
}

int read_recorded_data_USB502( libusb_device_handle *udev, configurationBlock *cblock, usb502_data *data_502 )
{
  int packet_size = 512;
  int memory_size = 0x8000;
  int num_packets;
  unsigned int size = 0;
  int ret;
  int transferred;
  int i;
  struct tm ltime;
  time_t currentTime;
  uint8_t request_data[3] = {0x03, 0xff, 0xff};  // Request Recorded Data
  uint8_t acknowledge[3] =  {0x00, 0x00, 0x00};  // Acknowledge string
  uint8_t rdata[0x8000];  // max data size

  time(&currentTime);
  localtime_r(&currentTime, &ltime);  // set daylight savings field

  stop_logging_USB500(udev, cblock);  // must stop logging before reading the data.
  unlock_device_USB500(udev);

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT | 2,  (unsigned char *)request_data, 3, &transferred, USB500_WAIT_WRITE);
  if (ret < 0) {
    perror("Error in requesting configuration (Bulk Write)");
  }
  do {
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|2, (unsigned char *) acknowledge, 3, &transferred, USB500_WAIT_READ);
    if (ret < 0) {
      perror("Error in acknowledging read data from USB 500");
    }
  } while (acknowledge[0] != 0x2);

  size = ((acknowledge[2] << 8) | acknowledge[1]);
  if (size != memory_size) {
    printf("Memory Error mismatch. size = %#x  should be %#x\n", size, memory_size);
    return -1;
  }

  num_packets = memory_size / packet_size;
  for ( i = 0; i < num_packets; i++ ) {
    do {
      ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &rdata[i*packet_size], 
				 packet_size, &transferred, USB500_WAIT_READ);
      if (ret < 0) {
	perror("Error reading data.  Retrying.");
      }
    } while (transferred != packet_size);
  }

  lock_device_USB500(udev);

  ltime.tm_sec = cblock->startTimeSeconds;
  ltime.tm_min = cblock->startTimeMinutes;
  ltime.tm_hour = cblock->startTimeHours;
  ltime.tm_mday = cblock->startTimeDay;
  ltime.tm_mon = cblock->startTimeMonth - 1;
  ltime.tm_year = cblock->startTimeYear + 100;
  data_502[0].time = mktime(&ltime);  // get local time stamp

  for (i = 0; i < cblock->sampleCount; i++) {
    data_502[i].time = data_502[0].time + i*cblock->sampleRate;
    data_502[i].temperature = cblock->calibrationMValue*rdata[2*i] + cblock->calibrationCValue;  // temperature first byte
    data_502[i].humidity = 0.5*rdata[2*i+1];                                                     // humidity second byte
  }
  start_logging_USB500(udev, cblock); 
  return cblock->sampleCount;
}

int read_recorded_data_USB503( libusb_device_handle *udev, configurationBlock *cblock, usb503_data *data_503 )
{
  int packet_size = 512;
  int memory_size = 0xfe00;
  int num_packets;
  unsigned int size = 0;
  int ret;
  int transferred;
  int i;
  struct tm ltime;
  time_t currentTime;
  uint16_t value;
  uint8_t request_data[3] = {0x03, 0xff, 0xff};  // Request Recorded Data
  uint8_t acknowledge[3] =  {0x00, 0x00, 0x00};  // Acknowledge string
  uint8_t rdata[0xfe00];  // max data size

  time(&currentTime);
  localtime_r(&currentTime, &ltime);  // set daylight savings field
  stop_logging_USB500(udev, cblock);  // must stop logging before reading the data.

  unlock_device_USB500(udev);

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT | 2,  (unsigned char *)request_data, 3, &transferred, USB500_WAIT_WRITE);
  if (ret < 0) {
    perror("Error in requesting configuration (Bulk Write)");
  }
  do {
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|2, (unsigned char *) acknowledge, 3, &transferred, USB500_WAIT_READ);
    if (ret < 0) {
      perror("Error in acknowledging read data from USB 500");
    }
  } while (acknowledge[0] != 0x2);

  size = ((acknowledge[2] << 8) | acknowledge[1]);
  if (size != memory_size) {
    printf("Memory Error mismatch. size = %#x  should be %#x\n", size, memory_size);
    return -1;
  }

  num_packets = memory_size / packet_size;
  for ( i = 0; i < num_packets; i++ ) {
    do {
      ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &rdata[i*packet_size], packet_size, &transferred, USB500_WAIT_READ);
      if (ret < 0) {
	perror("Error reading data.  Retrying.");
      }
    } while (transferred != packet_size);
  }

  lock_device_USB500(udev);

  ltime.tm_sec = cblock->startTimeSeconds;
  ltime.tm_min = cblock->startTimeMinutes;
  ltime.tm_hour = cblock->startTimeHours;
  ltime.tm_mday = cblock->startTimeDay;
  ltime.tm_mon = cblock->startTimeMonth - 1;
  ltime.tm_year = cblock->startTimeYear + 100;
  data_503[0].time = mktime(&ltime);  // get local time stamp

  for (i = 0; i < cblock->sampleCount; i++) {
      data_503[i].time = data_503[0].time + i*cblock->sampleRate;
      value = (rdata[2*i] << 0x8) | rdata[2*i+1];
      data_503[i].voltage = cblock->calibrationMValue*value + cblock->calibrationCValue;
      data_503[i].voltage *= cblock->capScalingFactor;
  }
  start_logging_USB500(udev, cblock); 
  return cblock->sampleCount;
}

int read_recorded_data_USB504( libusb_device_handle *udev, configurationBlock *cblock, usb504_data *data_504 )
{
  uint8_t request_data[3] = {0x03, 0xff, 0xff};  // Request Recorded Data
  uint8_t acknowledge[3] =  {0x00, 0x00, 0x00};  // Acknowledge string
  int packet_size = 512;
  int memory_size = 0xfe00;
  uint8_t rdata[0xfe00];  // max data size
  int num_packets;
  unsigned int size = 0;
  int ret;
  int transferred;
  int i;
  struct tm ltime;
  time_t currentTime;

  time(&currentTime);
  localtime_r(&currentTime, &ltime);  // set daylight savings field

  stop_logging_USB500(udev, cblock);  // must stop logging before reading the data.
  unlock_device_USB500(udev);

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT | 2,  (unsigned char *)request_data, 3, &transferred, USB500_WAIT_WRITE);
  if (ret < 0) {
    perror("Error in requesting configuration (Bulk Write)");
  }
  do {
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|2, (unsigned char *) acknowledge, 3, &transferred, USB500_WAIT_READ);
    if (ret < 0) {
      perror("Error in acknowledging read data from USB 500");
    }
  } while (acknowledge[0] != 0x2);

  size = ((acknowledge[2] << 8) | acknowledge[1]);
  if (size != memory_size) {
    printf("Memory Error mismatch. size = %#x  should be %#x\n", size, memory_size);
    return -1;
  }

  num_packets = memory_size / packet_size;
  for ( i = 0; i < num_packets; i++ ) {
    do {
      ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN | 2, (unsigned char *) &rdata[i*packet_size], packet_size, &transferred, USB500_WAIT_READ);
      if (ret < 0) {
	perror("Error reading data.  Retrying.");
      }
    } while (transferred != packet_size);
  }

  lock_device_USB500(udev);

  ltime.tm_sec = cblock->startTimeSeconds;
  ltime.tm_min = cblock->startTimeMinutes;
  ltime.tm_hour = cblock->startTimeHours;
  ltime.tm_mday = cblock->startTimeDay;
  ltime.tm_mon = cblock->startTimeMonth - 1;
  ltime.tm_year = cblock->startTimeYear + 100;
  data_504[0].time = mktime(&ltime);  // get local time stamp

  for (i = 0; i < cblock->sampleCount; i++) {
      data_504[i].time = data_504[0].time + i*cblock->sampleRate;
      data_504[i].current = cblock->calibrationMValue*rdata[i] + cblock->calibrationCValue;
      data_504[i].current *= cblock->capScalingFactor;
  }

  start_logging_USB500(udev, cblock); 
  return cblock->sampleCount;
}

void write_recorded_data_USB501(configurationBlock *cblock, usb501_data *data_501 )
{
  /* Create CVS file to be read into OpenOffice.  When reading file, choose file type as TEXT cvs */
  char filename[80];
  FILE *fp;
  int i;
  struct tm ltime;
  char dates[40];
 
  printf("Enter filename: ");
  scanf("%s", filename);
  fp = fopen(filename, "w");
 
  fprintf(fp, "\"Name\",\"%s\"\n", cblock->name);
  fprintf(fp, "\"Serial Number\",\"%d\"\n", cblock->serialNumber);
  fprintf(fp, "\"Model\",\"USB-501\"\n");

  if (cblock->inputType == 0) {
    fprintf(fp, "\"Sample\",\"Date/Time\",\"Temperature (C)\"\n");
  } else {
    fprintf(fp, "\"Sample\",\"Date/Time\",\"Temperature (F)\"\n");
  }
  for (i = 0; i < cblock->sampleCount; i++) {
    localtime_r(&data_501[i].time, &ltime);
    strftime(dates, 79, "%D %r", &ltime);
    fprintf(fp,"%5d,%s,%.2f\n", i+1, dates, data_501[i].temperature);
  }
  fclose(fp);
}

void write_recorded_data_USB502(configurationBlock *cblock, usb502_data *data_502 )
{
  /* Create CVS file to be read into OpenOffice.  When reading file, choose file type as TEXT cvs */
  char filename[80];
  FILE *fp;
  int i;
  float t, logew, dewPoint;
  struct tm ltime;
  char dates[40];
 
  printf("Enter filename: ");
  scanf("%s", filename);
  fp = fopen(filename, "w");
 
  fprintf(fp, "\"Name\",\"%s\"\n", cblock->name);
  fprintf(fp, "\"Serial Number\",\"%d\"\n", cblock->serialNumber);
  fprintf(fp, "\"Model\",\"USB-502\"\n");
  if (cblock->inputType == 0) {
    fprintf(fp, "\"Sample\",\"Date/Time\",\"Temperature (C)\",\"Humidity\",\"Dew point (C)\"\n");
  } else {
    fprintf(fp, "\"Sample\",\"Date/Time\",\"Temperature (F)\",\"Humidity\",\"Dew point (F)\"\n");
  }
  for (i = 0; i < cblock->sampleCount; i++) {
    localtime_r(&data_502[i].time, &ltime);
    strftime(dates, 79, "%D %r", &ltime);
    if (cblock->inputType == 0) {
      t = data_502[i].temperature;
    } else {
      t = fahr2celsius(data_502[i].temperature);
    }
    logew = (0.66077 +(7.5*t/(237.3 + t))) + (log10(data_502[i].humidity) - 2.0);
    dewPoint = ((0.66077 - logew)*237.3) / (logew - 8.16077);
    if (cblock->inputType == 1) dewPoint = celsius2fahr(dewPoint);
    strftime(dates, 79, "%D %r", &ltime);
    fprintf(fp,"%5d,%s,%.2f,%.1f,%.1f\n", i+1, dates, data_502[i].temperature, data_502[i].humidity, dewPoint);
  }
  fclose(fp);
}

void write_recorded_data_USB503(configurationBlock *cblock, usb503_data *data_503 )
{
  /* Create CVS file to be read into OpenOffice.  When reading file, choose file type as TEXT cvs */
  char filename[80];
  FILE *fp;
  int i;
  struct tm ltime;
  char dates[40];
 
  printf("Enter filename: ");
  scanf("%s", filename);
  fp = fopen(filename, "w");
 
  fprintf(fp, "\"Name\",\"%s\"\n", cblock->name);
  fprintf(fp, "\"Serial Number\",\"%d\"\n", cblock->serialNumber);
  fprintf(fp, "\"Model\",\"USB-503\"\n");

  fprintf(fp, "\"Sample\",\"Date/Time\",\"Voltage (V)\"\n");
  for (i = 0; i < cblock->sampleCount; i++) {
    localtime_r(&data_503[i].time, &ltime);
    strftime(dates, 79, "%D %r", &ltime);
    fprintf(fp,"%5d,%s,%.2f\n", i+1, dates, data_503[i].voltage);
  }
  fclose(fp);
}

void write_recorded_data_USB504(configurationBlock *cblock, usb504_data *data_504 )
{
  /* Create CVS file to be read into OpenOffice.  When reading file, choose file type as TEXT cvs */
  char filename[80];
  FILE *fp;
  int i;
  struct tm ltime;
  char dates[40];
 
  printf("Enter filename: ");
  scanf("%s", filename);
  fp = fopen(filename, "w");
 
  fprintf(fp, "\"Name\",\"%s\"\n", cblock->name);
  fprintf(fp, "\"Serial Number\",\"%d\"\n", cblock->serialNumber);
  fprintf(fp, "\"Model\",\"USB-504\"\n");

  fprintf(fp, "\"Sample\",\"Date/Time\",\"Current (mA)\"\n");
  for (i = 0; i < cblock->sampleCount; i++) {
    localtime_r(&data_504[i].time, &ltime);
    strftime(dates, 79, "%D %r", &ltime);
    fprintf(fp,"%5d,%s,%.2f\n", i+1, dates, data_504[i].current);
  }
  fclose(fp);
}

int main( void )
{
  libusb_device_handle *udev = NULL;
  configurationBlock cb;
  time_t currentTime;
  struct tm ltime;
  float t, logew, dewPoint;
  int i, j;
  usb501_data data_501[USB500_MAX_VALUES];
  usb502_data data_502[USB500_MAX_VALUES];
  usb503_data data_503[USB500_MAX_VALUES];
  usb504_data data_504[USB500_MAX_VALUES];  
  char dates[40];
  char name[16];
  char ans[80];
  int type;

  udev = usb_device_find_USB500(USB500_VID, USB500_PID);
  if (udev) {
    printf("Success, found a USB 500!\n");
    printf("Size of configuration block: %zu\n", sizeof(cb));
  } else {
    printf("Failure, did not find a USB 500!\n");
    return 0;
  }
  printf("\n\n");

  while (1) {
    printf("1. Status\n");
    printf("2. Configure USB 500 device for logging\n");
    printf("3. Download data\n");
    printf("4. Repair SRAM\n");
    printf("5. Exit\n");
    printf("Please select from 1-5:  \n");

    scanf("%s", ans);
    switch(ans[0]) {
      case '1':
        type = read_configuration_block_USB500(udev, &cb);
	printf("\n\n");
	switch(type) {
	  case 1:  printf("Device:\t\t\t USB-501 Temperature Logger (version 1.6 and earlier)\n"); break;
	  case 2:  printf("Device:\t\t\t USB-501 Temperature Logger (version 1.7 and later)\n"); break;
	  case 3:  printf("Device:\t\t\t USB-502 Humidity and Temperature Logger\n"); break;
	  case 4:  printf("Device:\t\t\t USB-503 Voltage Logger\n"); break;
    	  case 5:  printf("Device:\t\t\t USB-504 Current Logger\n"); break;
	  case 6:  printf("Device:\t\t\t USB-503B Voltage Logger\n"); break;
    	  case 7:  printf("Device:\t\t\t USB-504B Current Logger\n"); break;
          default: printf("Device:\t\t\t Unkown\n"); break;
	}
	printf("Device Name:\t\t %s\n", cb.name);
	if (cb.flagBits & LOGGING_STATE) {
          if (cb.startTimeOffset == 0) {
	    printf("Status:\t\t\t Logging\n");
	  } else {
	    printf("Status:\t\t\t Delayed Start\n");
            printf("Delay:\t\t\t %d\n", cb.startTimeOffset);
	  }
	} else {
  	  printf("Status:\t\t\t Off\n");
	}

        ltime.tm_sec = cb.startTimeSeconds;
        ltime.tm_min = cb.startTimeMinutes;
        ltime.tm_hour = cb.startTimeHours;
        ltime.tm_mday = cb.startTimeDay;
        ltime.tm_mon = cb.startTimeMonth - 1;
        ltime.tm_year = cb.startTimeYear + 100;
        strftime(dates, 79, "%m/%d/%Y", &ltime);
	printf("Start Date:\t\t %s\n", dates);
        strftime(dates, 79, "%r", &ltime);
	printf("Start Time:\t\t %s\n", dates);

	printf("Number of Readings:\t %-d\n", cb.sampleCount);
	printf("Sample Interval:\t %d seconds\n", cb.sampleRate);
	if (type < 4) {            // USB-501 or USB-502
  	  if (cb.inputType == 0) { // Celsius
	    printf("Scale:\t\t\t Celsius");
	  } else {
	    printf("Scale:\t\t\t Fahrenheit");
	  }
	  if (cb.type == 3) {
	    printf(", %%rh\n");
	  } else {
	    printf("\n");
	  }
	} else {                   // USB-503 or USB-504
	  if (cb.inputType == 0) {
	    printf("Calibration:\t\t Standard Calibration.\n");
	  } else {
	    printf("Calibration:\t\t Custom Calibration.\n");
	  }
	  if (cb.flagBits2 & 0x1) {
	    printf("LEDs:\t\t\t Disabled to save battery power.\n");
	  } else {
	    printf("LEDs:\t\t\t Enabled.\n");
	  }
	  if (cb.flagBits2 & 0x2) {
	    printf("Scale:\t\t\t Negative gradient scaling.\n");
	  } else {
	    printf("Scale:\t\t\t Normal positive gradient scaling.\n");
	  }
	}
	if (cb.flagBits & HIGH_ALARM_STATE) {
	  printf("High Alarm:\t\t Enabled\n");
	} else {
	  printf("High Alarm:\t\t Disabled\n");	  
	}
	if (cb.flagBits & LOW_ALARM_STATE) {
	  printf("Low Alarm:\t\t Enabled\n");
	} else {
	  printf("Low Alarm:\t\t Disabled\n");	  
	}
	printf("Serial Number:\t\t %d\n", cb.serialNumber);
	strncpy(name, cb.version, 4);
	printf("Firmware Version:\t %s\n", name);
	if (type >= 4) {
	  printf("Measurement Unit:\t %s.\n", cb.displayUnitText);
	  printf("User Calibration Input 1: %s.\n", cb.calibrationInput1Text);
  	  printf("User Calibration Output 1: %s.\n", cb.calibrationInput1Text);
  	  printf("User Calibration Input 2: %s.\n", cb.calibrationOutput2Text);
    	  printf("User Calibration Output 2: %s.\n", cb.calibrationOutput2Text);
	  printf("Scaling Factor:\t\t %f\n", cb.capScalingFactor);
	  printf("High Alarm Level Text: %s.\n", cb.highAlarmLevelText);
	  printf("Low Alarm Level Text: %s.\n", cb.lowAlarmLevelText);
	  printf("Default Range Description Text: %s.\n", cb.defaultRangeDescriptionText);
	  printf("Default Input Unit Text: %s.\n", cb.defaultInputUnitText);
	  printf("Default Display Unit: %s.\n", cb.defaultDisplayUnit);
	  printf("Default Calibration Input 1 Text: %s.\n", cb.defaultCalibrationInput1Text);
	  printf("Default Calibration Output 1 Text: %s.\n", cb.defaultCalibrationOutput1Text);
	  printf("Default Calibration Input 2 Text: %s.\n", cb.defaultCalibrationInput2Text);
	  printf("Default Calibration Output 2 Text: %s.\n", cb.defaultCalibrationOutput2Text);
	  printf("Default High Alarm Level Text: %s.\n", cb.defaultHighAlarmLevelText);
	  printf("Default Low Alarm Level Text: %s.\n", cb.defaultLowAlarmLevelText);
	}
	printf("\n\n");
	break;
      case '2':
        stop_logging_USB500(udev, &cb);
        /* initialize the parameter block */

        // sanity check
        printf("cb.type = %d\n", cb.type);
        printf("serial number = %d\n", cb.serialNumber);
	cb.command = 0x0;
	printf("Device name = \"%s\": ",cb.name);
	tcflush(0, TCIOFLUSH);
	getchar();
	fgets(name, 15, stdin);
	if (name[0] != '\n') {
  	  cb.name[15] = '\0';  // null terminate
	  j = 0;
	  for (i = 0; i < strlen(cb.name); i++) {
	    switch (name[i]) {
	    case '\r':
	    case '\n':
	    case '%':
	    case '&':
	    case '*':
	    case ',':
	    case '.':
	    case '/':
	    case ':':
	    case '<':
	    case '>':
	    case '?':
	    case '\\':
	    case '(':
	    case ')':
	      break;
	    default:
	      cb.name[j++] = name[i];
	    }
	  }
	}
        if (type < 4) {
  	  printf("0 = Celsius, 1 = Fahrenheit: ");
	  scanf("%hhd", &cb.inputType);
	  if (cb.inputType == 0) { // Celsius
	    cb.calibrationMValue = 0.5;
	    cb.calibrationCValue = -40;
	  } else {                // Fahrenheit
	    cb.inputType = 1;
	    cb.calibrationMValue = 1.0;
	    cb.calibrationCValue = -40;
	  }
	}

	printf("Enter sample rate [seconds/sample].  Must be > 10:  ");
	scanf("%hd", &cb.sampleRate);
	if (cb.sampleRate < 10) cb.sampleRate = 10;
        write_configuration_block_USB500(udev, &cb);

        /* uncomment to set alarms */
        set_alarm(udev, &cb);
	
        time(&currentTime);
	printf("Setting Device time to: %s and start logging\n", ctime(&currentTime));
        start_logging_USB500(udev, &cb); 
	break;

      case '3':
        type = read_configuration_block_USB500(udev, &cb);
        switch(type) {
          case 1:
          case 2:
            read_recorded_data_USB501(udev, &cb, data_501);
   	    printf("\n\nModel: USB-501\n");
	    if (cb.inputType == 0) {
	      printf("Sample         Date/Time            Temperature (C) \n");
	    } else {
	      printf("Sample         Date/Time            Temperature (F) \n");
	    }
	    for (i = 0; i < cb.sampleCount; i++) {
              localtime_r(&data_501[i].time, &ltime);
              strftime(dates, 79, "%D %r", &ltime);
              printf("%5d    %s           %.2f  \n", i+1, dates, data_501[i].temperature);
	    }
	    printf("\n\n");
            printf("Save to file? [y/n] ");
            scanf("%s", ans);
	    if (ans[0] == 'y') write_recorded_data_USB501(&cb, data_501);
	    break;
          case 3:
            read_recorded_data_USB502(udev, &cb, data_502);
	    printf("\n\nModel: USB-502\n");
	    if (cb.inputType == 0) {
	      printf("Sample         Date/Time            Temperature (C)       Humidity    Dew point (C)\n");
  	    } else {
	      printf("Sample         Date/Time            Temperature (F)       Humidity    Dew point (F)\n");
	    }
	    for (i = 0; i < cb.sampleCount; i++) {
	      localtime_r(&data_502[i].time, &ltime);
	      if (cb.inputType == 0) {
	        t = data_502[i].temperature;
	      } else {
	        t = fahr2celsius(data_502[i].temperature);
	      }
	      logew = (0.66077 +(7.5*t/(237.3 + t))) + (log10(data_502[i].humidity) - 2.0);
	      dewPoint = ((0.66077 - logew)*237.3) / (logew - 8.16077);
	      if (cb.inputType == 1) dewPoint = celsius2fahr(dewPoint);
	      strftime(dates, 79, "%D %r", &ltime);
	      printf("%5d    %s           %.2f               %.1f        %.1f\n",
		     i+1, dates, data_502[i].temperature, data_502[i].humidity, dewPoint);
	    }	
	    printf("\n\n");
	    printf("Save to file? [y/n] ");
	    scanf("%s", ans);
	    if (ans[0] == 'y') write_recorded_data_USB502(&cb, data_502);
	    break;
          case 4:
          case 6:
            read_recorded_data_USB503(udev, &cb, data_503);
	    printf("\n\nModel: USB-503\n");
	    printf("Sample         Date/Time            Voltage (V) \n");
	    for (i = 0; i < cb.sampleCount; i++) {
              localtime_r(&data_503[i].time, &ltime);
              strftime(dates, 79, "%D %r", &ltime);
              printf("%5d    %s           %.2f  \n", i+1, dates, data_503[i].voltage);
	    }
	    printf("\n\n");
            printf("Save to file? [y/n] ");
            scanf("%s", ans);
	    if (ans[0] == 'y') write_recorded_data_USB503(&cb, data_503);
	    break;
          case 5:
          case 7:
	    read_recorded_data_USB504(udev, &cb, data_504);
	    printf("Sample         Date/Time            Current (mA)\n");
	    for (i = 0; i < cb.sampleCount; i++) {
              localtime_r(&data_504[i].time, &ltime);
              strftime(dates, 79, "%D %r", &ltime);
              printf("%5d    %s           %.2f  \n", i+1, dates, data_504[i].current);
	    }
	    printf("\n\n");
            printf("Save to file? [y/n] ");
            scanf("%s", ans);
	    if (ans[0] == 'y') write_recorded_data_USB504(&cb, data_504);
	    break;
	}
        break;

      case '4':
        printf("******* WARNING*******\n");
        printf("Only proceed if your device is not working.\n");
        printf("Do you want to proceed? [y/n]");
        scanf("%s", ans);
        if (ans[0] == 'n') break;
        /* get the serial number and software version */
        read_configuration_block_USB500(udev, &cb);
        printf("    1. USB-501 version 1.6 and earlier.\n");
        printf("    2. USB-501 version 1.7 and later.\n");
        printf("    3. USB-502\n");
        printf("Enter your Device type [1-3]: \n");
        scanf("%hhd", &cb.type);
        strncpy(cb.name, "EasyLog USB", 15);
	// strncpy(cb.version, "v2.0", 4);
        cb.inputType = 1;
        cb.command = 0x0;
        cb.calibrationMValue = 1.0;
	cb.calibrationCValue = -40;
        cb.sampleRate = 60;
        cb.flagBits = 0x0;
        time(&currentTime);
        localtime_r(&currentTime, &ltime);	
        cb.startTimeHours = ltime.tm_hour;
        cb.startTimeMinutes = ltime.tm_min;
	cb.startTimeSeconds = ltime.tm_sec;
	cb.startTimeDay = ltime.tm_mday;
	cb.startTimeMonth = ltime.tm_mon + 1;    // months from 1-12
	cb.startTimeYear = ltime.tm_year - 100;  // years start from 2000 not 1900.
        cb.alarm1.channel1.highAlarm = 0x0;
        cb.alarm1.channel1.lowAlarm = 0x0;
        write_configuration_block_USB500(udev, &cb);
	break;
      case '5':
	printf("Remove unit from USB port.\n");
	cleanup_USB500(udev);
        return 0;
        break;
    }
  }
}

