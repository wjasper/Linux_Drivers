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
#include <stdint.h>

#include "pmd.h"
#include "usb-1408FS.h"

/* Commands and HID Report ID for USB 1408FS */
#define DCONFIG          (0x01) // Configure digital port
#define DIN              (0x03) // Read digital port
#define DOUT             (0x04) // Write digital port

#define AIN              (0x10) // Read analog input channel
#define AIN_SCAN         (0x11) // Scan analog channels
#define AIN_STOP         (0x12) // Stop input scan
#define ALOAD_QUEUE      (0x13) // Load the channel/gain queue

#define AOUT             (0x14) // Write analog output channel
#define AOUT_SCAN        (0x15) // Output values to a range of output channels
#define AOUT_STOP        (0x16) // Stop output scan

#define CINIT            (0x20) // Initialize counter
#define CIN              (0x21) // Read Counter

#define MEM_READ         (0x30) // Read Memory
#define MEM_WRITE        (0x31) // Write Memory

#define BLINK_LED        (0x40) // Causes LED to blink
#define RESET            (0x41) // Reset USB interface
#define SET_TRIGGER      (0x42) // Configure external trigger
#define SET_SYNC         (0x43) // Configure sync input/output
#define GET_STATUS       (0x44) // Get device status
#define SET_CAL          (0x45) // Set calibaration output
#define GET_ALL          (0x46) // Get all analog and digital input values

#define PREPARE_DOWNLOAD (0x50) // Prepare for program memory download
#define WRITE_CODE       (0x51) // Write program memory
#define WRITE_SERIAL     (0x53) // Write a new serial number to device
#define READ_CODE        (0x55) // Read program memory

#define FS_DELAY 2000

enum Mode {Differential, SingleEnded};
static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant. 

int init_USB1408FS(libusb_device_handle *udev)
{
  int i;
  int ret;
  
  // claim all the needed interfaces for AInScan
  for (i = 1; i <= 3; i++) {
    ret = libusb_detach_kernel_driver(udev, i);
    if (ret < 0) {
      perror("usb1408FS: Can't detach kernel from interface");
      usbReset_USB1408FS(udev);
      return ret;  
    }
    ret = libusb_claim_interface(udev, i);
    if (ret < 0) {
      perror("usb1408FS: Can't claim interface.");
      return ret;
    }
  }

  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
  if (wMaxPacketSize < 0) {
    perror("usb1408FS: error in getting wMaxPacketSize");
  }

  return 0;
}

/* configures digital port */
void usbDConfigPort_USB1408FS(libusb_device_handle *udev, uint8_t port, uint8_t direction)
{
  struct config_port_t {
    uint8_t reportID;
    uint8_t port;
    uint8_t direction;
  } config_port;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | DCONFIG;   // HID output
  uint16_t wIndex = 0;                    // Interface

  config_port.reportID = DCONFIG;
  config_port.port = port;
  config_port.direction = direction;
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &config_port, sizeof(config_port), 5000);
  if (ret < 0) {
    perror("Error in usbDConfigPort_USB1408FS: libusb_control_transfer error");
  }
}

/* reads digital port  */
void usbDIn_USB1408FS(libusb_device_handle *udev, uint8_t port, uint8_t* din_value)
{
  int transferred;
  uint8_t reportID = DIN;

  struct read_port_t {
    uint8_t reportID;
    uint8_t value[2];
  } read_port;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;             // HID Set_Report
  uint16_t wValue = (2 << 8) | DIN;  // HID output
  uint16_t wIndex = 0;               // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbDIn_USB1408FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN  | 1, (unsigned char*) &read_port, sizeof(read_port), &transferred, FS_DELAY);

  /* don't return values off the stack*/
  if (port == DIO_PORTA) {
    *din_value = read_port.value[0];
  } else {
    *din_value = read_port.value[1];
  }
  return;
}

/* writes digital port */
void usbDOut_USB1408FS(libusb_device_handle *udev, uint8_t port, uint8_t value)
{
  struct write_port_t {
    uint8_t reportID;
    uint8_t port;
    uint8_t value;
  } write_port;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;              // HID Set_Report
  uint16_t wValue = (2 << 8) | DOUT;  // HID output
  uint16_t wIndex = 0;                // Interface

  write_port.reportID = DOUT;
  write_port.port = port;
  write_port.value = value;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &write_port, sizeof(write_port), 5000);
  if (ret < 0) {
    perror("Error in usbDOut_USB1408FS: libusb_control_transfer error");
  }
}

/* writes to analog out */
void usbAOut_USB1408FS(libusb_device_handle *udev, uint8_t channel, uint16_t value)
{
  value <<= 0x4;

  struct aout_t {
    uint8_t reportID;
    uint8_t channel;
    uint8_t value[2];
  } aout;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | AOUT;      // HID output
  uint16_t wIndex = 0;                    // Interface

  aout.reportID = AOUT;
  aout.channel = channel;                             // 0 or 1
  aout.value[0] = (uint8_t) (value & 0xf0);           // low byte
  aout.value[1] = (uint8_t) ((value >> 0x8) & 0xff);  // high byte

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &aout, sizeof(aout), 5000);
  if (ret < 0) {
    perror("Error in usbAOut_USB1408FS: libusb_control_transfer error");
  }
}

int usbAOutScan_USB1408FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel,
			  uint32_t count, float *frequency, uint16_t data[], uint8_t options)
{
  int num_samples;
  int transferred;
  int i;
  uint32_t preload;
  uint8_t byte[64];
  
  struct scanReport_t {
    uint8_t reportID;
    uint8_t lowchannel;   // the first channel of the scan
    uint8_t highchannel;  // the last channel of the scan
    uint8_t count[4];     // the total number of scans to perform
    uint8_t prescale;     // timer prescale
    uint8_t preload[2];   // timer preload
    uint8_t options;      // bit 0: 1 = single execution  0 = continuous
                          // bit 1: 1 = use external trigger
  } scanReport;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | AOUT_SCAN; // HID output
  uint16_t wIndex = 0;                    // Interface

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

  num_samples = count*(highchannel - lowchannel + 1);
  // num_samples = count;

  scanReport.reportID = AOUT_SCAN;
  scanReport.lowchannel = lowchannel;
  scanReport.highchannel = highchannel;
  scanReport.count[0] = (uint8_t) count & 0xff;           // low byte
  scanReport.count[1] = (uint8_t) (count >>  8) & 0xff;
  scanReport.count[2] = (uint8_t) (count >> 16) & 0xff;
  scanReport.count[3] = (uint8_t) (count >> 24) & 0xff;   // high byte
  scanReport.options = options;                        // single execution

  byte[0] = AOUT_SCAN;

  if (*frequency > 0.596 && *frequency < 10000) {
    for (scanReport.prescale = 0; scanReport.prescale <= 8; scanReport.prescale++) {
      preload = 10e6/((*frequency) * (1<<scanReport.prescale));
      if (preload <= 0xffff) {
	scanReport.preload[0] = (uint8_t) preload & 0xff;          // low byte
	scanReport.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
	*frequency = 10.e6/((1<<scanReport.prescale)*preload);
	break;
      }
    }
  } else if (*frequency == 0.0) {  // external sync
    preload = 0xff;
    scanReport.preload[0] = (uint8_t) preload & 0xff;          // low byte
    scanReport.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
  } else {
    printf("usbAOutScan_USB1408FS: frequency out of range.\n");
    return -1;
  }
    
  if (scanReport.prescale == 9 || preload == 0) {
    printf("usbAOutScan_USB1408FS: frequency out of range.\n");
    return -1;
  }

  /* shift over all data 4 bits */
  for (i = 0; i < num_samples; i++) {
    data[i] <<= 4;
  }

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &scanReport, sizeof(scanReport), 5000);
  if (ret < 0) {
    perror("Error in usbAOutScan_USB1408FS: libusb_control_transfer error");
  }
  i = 0;
  while (num_samples >= 32) {
    libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_OUT | 2, (unsigned char *) &data[i], 64, &transferred, 1000);
    libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) byte, 2, &transferred, 1000);
    num_samples -= 32;
    i += 32;
  }
  if (num_samples > 0) {
    libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_OUT | 2, (unsigned char *) &data[i], num_samples*2, &transferred, 1000);
  }
  return 0;
}

void usbAOutStop_USB1408FS(libusb_device_handle *udev)
{
  uint8_t reportID = AOUT_STOP;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                   // HID Set_Report
  uint16_t wValue = (2 << 8) | AOUT_STOP;  // HID output
  uint16_t wIndex = 0;                     // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbAOutStop_USB1408FS: libusb_control_transfer error");
  }
}

/* reads from analog in */
signed short usbAIn_USB1408FS(libusb_device_handle *udev, uint8_t channel, uint8_t range)
{
  enum Mode mode;
  int transferred;

  int16_t value;
  uint16_t uvalue;
  uint8_t report[3];
  
  struct t_ain {
    uint8_t reportID;
    uint8_t channel;
    uint8_t range;
  } ain;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;              // HID Set_Report
  uint16_t wValue = (2 << 8) | AIN;   // HID output
  uint16_t wIndex = 0;                // Interface

  if (channel > 7) {
    printf("usbAIN: channel out of range for differential/single ended  mode.\n");
    return -1;
  }

  ain.reportID = AIN;
  ain.channel = channel;
  ain.range = range;

  if (range == SE_10_00V) {
    mode = SingleEnded;
    ain.channel += 8;
    ain.range = 0;
  } else {
    mode = Differential;
  }

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &ain, sizeof(ain), 5000);
  if (ret < 0) {
    perror("Error in usbAIn_USB1408FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char*) report, sizeof(report), &transferred, FS_DELAY);

  if (mode == Differential) {
    /* the data is a 2's compliment signed 14 bit number */
    value = (int16_t) ( report[1] | (report[2] << 8));
    value /= (1<<2);
  } else {
    /* the data is a  2's compliment signed 13 bit number */
    uvalue = (uint16_t) ( report[1] | (report[2] << 8));
    if (uvalue > 0x7ffc) {
      uvalue = 0;
    } else if (uvalue > 0x7ff8) {
      uvalue = 0x3fff;
    } else {
      uvalue >>= 1;
      uvalue &= 0x3fff;
    }
    value = uvalue - 0x2000;
  }
  return value;
}

void usbAInStop_USB1408FS(libusb_device_handle *udev)
{
  uint8_t reportID = AIN_STOP;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | AIN_STOP;  // HID output
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbAInStop_USB1408FS: libusb_control_transfer error");
  }
}

int usbAInScan_USB1408FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count,
			  float *frequency, uint8_t options, int16_t sdata[])
{
  int num_samples;
  int i, k;
  int pipe;
  uint32_t preload;

  struct {
    int16_t value[31];
    uint16_t scan_index;
  } data;
  
  struct arg {
    uint8_t reportID;
    uint8_t lowchannel;
    uint8_t highchannel;
    uint8_t count[4];
    uint8_t prescale;
    uint8_t preload[2];
    uint8_t options;
  } arg;

  int ret;
  int transferred;
  int timeout = FS_DELAY;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | AIN_SCAN;  // HID output
  uint16_t wIndex = 0;                    // Interface

  if (highchannel > 7) {
    printf("usbAInScan: highchannel out of range.\n");
    return -1;
  }
  if (lowchannel > 7) {
    printf("usbAInScan: lowchannel out of range.\n");
    return -1;
  }

  num_samples = count;
  count += count%31;    // fill up entire scan line

  arg.reportID = AIN_SCAN;
  arg.lowchannel = lowchannel;
  arg.highchannel = highchannel;
  arg.count[0] = (uint8_t) count & 0xff;           // low byte
  arg.count[1] = (uint8_t) (count >>  8) & 0xff;
  arg.count[2] = (uint8_t) (count >> 16) & 0xff;
  arg.count[3] = (uint8_t) (count >> 24) & 0xff;   // high byte
  arg.options = options;                        

  for (arg.prescale = 0; arg.prescale <= 8; arg.prescale++) {
    preload = 10e6/((*frequency) * (1<<arg.prescale));
    if (preload <= 0xffff) {
      arg.preload[0] = (uint8_t) preload & 0xff;          // low byte
      arg.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
      break;
    }
  }

  *frequency = 10.e6/(preload*(1<<arg.prescale));

  // printf("AInScan: actual frequency = %f\n", *frequency);

  if (arg.prescale == 9 || preload == 0) {
    printf("usbAInScan_USB1408FS: frequency out of range.\n");
    return -1;
  }
  
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 500);
  if (ret < 0) {
    perror("Error in usbAInScan_USB1408FS: libusb_control_transfer error");
  }

  i = 0;
  pipe = 1;   // Initial Enpoint to receive data.

  while (num_samples > 0) {         
    ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN |(pipe+2), (unsigned char *) &data, sizeof(data), &transferred, timeout);
    if (ret < 0) {
      perror("usbAInScan_USB1408FS interrupt pipes");
    }
    if (num_samples > 31) {
      for ( k = 0; k < 31;  k++ ) {
	sdata[i+k] = data.value[k] / (1<<2);
      }
      num_samples -= 31;
      i += 31;
    } else {
      for (k = 0; k < num_samples;  k++) {
	sdata[i+k] = data.value[k] / (1<<2);
      }
      num_samples -= 31;
      i += 31;
      break;
    }
    pipe = (pipe%3) + 1;  //pipe should take the values 1, 2 or 3
  }

  usbAInStop_USB1408FS(udev);
  return count;
}
/* scan for Single Ended */
int usbAInScan_USB1408FS_SE(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count,
			  float *frequency, uint8_t options, int16_t sdata[])
{
  int num_samples;
  int i, k;
  int pipe;
  uint32_t preload;
  uint8_t nchan, chan[8], gains[8];

  struct {
    uint16_t value[31];
    uint16_t scan_index;
  } data;
  
  struct arg {
    uint8_t reportID;
    uint8_t lowchannel;
    uint8_t highchannel;
    uint8_t count[4];
    uint8_t prescale;
    uint8_t preload[2];
    uint8_t options;
  } arg;

  int ret;
  int transferred;
  int timeout = FS_DELAY;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | AIN_SCAN;  // HID output
  uint16_t wIndex = 0;                    // Interface

  if (highchannel > 7) {
    printf("usbAInScan_SE: highchannel out of range.\n");
    return -1;
  }
  if (lowchannel > 7) {
    printf("usbAInScan_SE: lowchannel out of range.\n");
    return -1;
  }

  if (lowchannel > highchannel) {
    printf("usbAInScan_SE: lowchannel greater than highchannel.\n");
    return -1;
  }

  num_samples = count;

  // Set the channel gain.
  arg.reportID = AIN_SCAN;
  arg.lowchannel = lowchannel+8;
  arg.highchannel = highchannel+8;
  arg.count[0] = (uint8_t) count & 0xff;           // low byte
  arg.count[1] = (uint8_t) (count >>  8) & 0xff;
  arg.count[2] = (uint8_t) (count >> 16) & 0xff;
  arg.count[3] = (uint8_t) (count >> 24) & 0xff;   // high byte
  arg.options = options;                        

  for ( arg.prescale = 0; arg.prescale <= 8; arg.prescale++ ) {
    preload = 10e6/((*frequency) * (1<<arg.prescale));
    if (preload <= 0xffff) {
      arg.preload[0] = (uint8_t) preload & 0xff;          // low byte
      arg.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
      break;
    }
  }

  *frequency = 10.e6/(preload*(1<<arg.prescale));

  if (arg.prescale == 9 || preload == 0) {
    printf("usbAInScan_USB1408FS_SE: frequency out of range.\n");
    return -1;
  }

  /* Set the gain +/-10V */
  nchan = highchannel - lowchannel + 1;
  for (i = 0; i < nchan; i++) {
    chan[i] = lowchannel + i;
    gains[i] = SE_10_00V;
  }
  usbALoadQueue_USB1408FS(udev, nchan, chan, gains);

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 500);
  if (ret < 0) {
    perror("Error in usbAInScan_USB1408FS_SE: libusb_control_transfer error");
  }

  i = 0;
  pipe = 1;   // Initial Enpoint to receive data.
  transferred = 0;

  /*
    The device returns a 16 bit signed value.  However, the
    actual resolution is 13 bits in SE mode offset.  bit 0 and bit 1 and the rest of
    14 bits represent the data.   In SE mode, bit 14 is also 0 since
    only 13 bits are relavent.
  */
  
  while ( num_samples > 0 ) {
    ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN |(pipe+2), (unsigned char *) &data, sizeof(data), &transferred, timeout);
    if (ret < 0) {
      perror("Error in usbAInScan_USB1408FS_SE: libusb_control_transfer error");
    }
    if (num_samples > 31) {
      for ( k = 0; k < 31;  k++ ) {
	if (data.value[k] > 0x7ffc) {
	  data.value[k] = 0;
	} else if (data.value[k] > 0x7ff8) {
	  data.value[k] = 0x3fff;
	} else {
	  data.value[k] >>= 1;
	  data.value[k] &= 0x3fff;
	}
	sdata[i+k] =  data.value[k]- 0x2000;
      }
      num_samples -= 31;
      i += 31;
    } else {   // only copy in a partial scan
      for (k = 0; k < num_samples;  k++) {
	if (data.value[k] > 0x7ffc) {
	  data.value[k] = 0;
	} else if (data.value[k] > 0x7ff8) {
	  data.value[k] = 0x3fff;
	} else {
	  data.value[k] >>= 1;
	  data.value[k] &= 0x3fff;
	}
	sdata[i+k] =  data.value[k] - 0x2000;
      }
      num_samples = 0;
      break;
    }
    pipe = (pipe)%3 + 1;  //pipe should take the values 1, 2 or 3
  }

  usbAInStop_USB1408FS(udev);
  return 0;
}

void usbALoadQueue_USB1408FS(libusb_device_handle *udev, uint8_t num, uint8_t chan[], uint8_t gains[])
{
  struct t_aLoadQueue {
    uint8_t reportID;
    uint8_t num;        // number of channel/gain pairs to follow (max 8).
    uint8_t gains[16];  // channel/gain pairs.
  } aLoadQueue;
  int i;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                    // HID Set_Report
  uint16_t wValue = (2 << 8) | ALOAD_QUEUE; // HID output
  uint16_t wIndex = 0;                      // Interface

  num = (num <= 8) ? num : 8;

  aLoadQueue.reportID = ALOAD_QUEUE;
  aLoadQueue.num = num;
  for (i = 0; i < num; i++) {
    if (gains[i] == SE_10_00V) {
      aLoadQueue.gains[2*i] = chan[i] + 8;
      aLoadQueue.gains[2*i+1] = 0;
    } else {
      aLoadQueue.gains[2*i] = chan[i];
      aLoadQueue.gains[2*i+1] = gains[i];
    }
  }
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &aLoadQueue, sizeof(aLoadQueue), 5000);
  if (ret < 0) {
    perror("Error in usbAInLoadQueue_USB1408FS: libusb_control_transfer error");
  }
}

/* Initialize the counter */
void usbInitCounter_USB1408FS(libusb_device_handle *udev)
{
  uint8_t reportID = CINIT;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;               // HID Set_Report
  uint16_t wValue = (2 << 8) | CINIT;  // HID output
  uint16_t wIndex = 0;                 // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbInitCounter_USB1408FS: libusb_control_transfer error");
  }
}

uint32_t usbReadCounter_USB1408FS(libusb_device_handle *udev)
{
   uint32_t value;
  struct t_counter {
    uint8_t reportID;
    uint8_t value[4];
  } counter;

  int ret;
  int transferred;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;             // HID Set_Report
  uint16_t wValue = (2 << 8) | CIN;  // HID output
  uint16_t wIndex = 0;               // Interface

  counter.reportID = CIN;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &counter, 1, 5000);
  if (ret < 0) {
    perror("Error in usbCounter_USB1408FS: libusb_control_transfer error");
  }

  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) &counter, sizeof(counter), &transferred, FS_DELAY);
  value =   counter.value[0] | (counter.value[1] << 8) | (counter.value[2] << 16) | (counter.value[3] << 24);

  return value;
}

/* blinks the LED of USB device */
void usbBlink_USB1408FS(libusb_device_handle *udev)
{
  uint8_t reportID = BLINK_LED;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | BLINK_LED; // HID output
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbBlink_USB1408FS: libusb_control_transfer error");
  }
}

int usbReset_USB1408FS(libusb_device_handle *udev)
{
  int ret;
  uint8_t reportID = RESET;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                // HID Set_Report
  uint16_t wValue = (2 << 8) | RESET;   // HID output
  uint16_t wIndex = 0;                  // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbReset_USB1408FS: libusb_control_transfer error");
  }
  return 0;
}

void usbSetTrigger_USB1408FS(libusb_device_handle *udev, uint8_t type)
{
  uint8_t cmd[2];
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                    // HID Set_Report
  uint16_t wValue = (2 << 8) | SET_TRIGGER; // HID output
  uint16_t wIndex = 0;                      // Interface

  cmd[0] = SET_TRIGGER;
  cmd[1] = type;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &cmd, sizeof(cmd), 5000);
  if (ret < 0) {
    perror("Error in usbSetTriggert_USB1408FS: libusb_control_transfer error");
  }
}

void usbSetSync_USB1408FS(libusb_device_handle *udev, uint8_t type)
{
  uint8_t cmd[2];
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | SET_SYNC;  // HID output
  uint16_t wIndex = 0;                    // Interface

  cmd[0] = SET_SYNC;
  cmd[1] = type;
  
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &cmd, sizeof(cmd), 5000);
  if (ret < 0) {
    perror("Error in usbSetSync_USB1408FS: libusb_control_transfer error");
  }
}

uint16_t usbGetStatus_USB1408FS(libusb_device_handle *udev)
{
  /* This command retrieves the status of the device.

       Bit 0: 0 = Sync slave,               1 = sync master
       Bit 1: 0 = Trigger falling edge,     1 = trigger rising edge
       Bit 2: 0 = Normal sync (slave mode)  1 = gated sync
       Bit 3: 0 = EEPROM cal memory locked  1 = cal memory unlocked
       Bits 4-15 unused.
  */
  
  uint16_t status;
  int transferred;
    
  struct t_statusReport {
  uint8_t reportID;
  uint8_t status[2];
  } statusReport;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                   // HID Set_Report
  uint16_t wValue = (2 << 8) | GET_STATUS; // HID output
  uint16_t wIndex = 0;                     // Interface

  statusReport.reportID = GET_STATUS;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &statusReport, sizeof(statusReport), 5000);
  if (ret < 0) {
    perror("Error in usbGetStatus_USB1408FS: libusb_control_transfer error");
    return -1;
  }
  do {
    statusReport.reportID = 0;
    ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) &statusReport, sizeof(statusReport), &transferred, FS_DELAY);
    if (ret < 0) {
      perror("Error in usbGetStatus_USB1408FS: libusb_interrupt_transfer error");
      return -1;
    }
  } while (statusReport.reportID != GET_STATUS);

  status = (uint16_t) (statusReport.status[0] | (statusReport.status[1] << 8));
  status &= 0x000f;                                    // mask off top 12 bits

  return status;
}

void usbReadMemory_USB1408FS( libusb_device_handle *udev, uint16_t address, uint8_t count, uint8_t memory[])
{
  // Addresses 0x000 - 0x07F are reserved for firmware data
  // Addresses 0x080 - 0x3FF are available for use as calibraion or user data

  int transferred;
  uint8_t data[64];
  int i;

  struct arg {
    uint8_t reportID;
    uint8_t address[2];
    uint8_t type;
    uint8_t count;
  } arg;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | MEM_READ;  // HID output
  uint16_t wIndex = 0;                    // Interface

  if (count > 62) count = 62;
  arg.reportID = MEM_READ;
  arg.address[0] = address & 0xff;         // low byte
  arg.address[1] = (address >> 8) & 0xff;  // high byte
  arg.count = count;

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
  if (ret < 0) {
    perror("Error in usbReadMemory_USB1408FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) data, count+1, &transferred, FS_DELAY);
  for (i = 0; i < count; i++) {
    memory[i] = data[i+1];
  }
}

int usbWriteMemory_USB1408FS(libusb_device_handle *udev, uint16_t address, uint8_t count, uint8_t* data)
{
  // Locations 0x00-0x7F are reserved for firmware and my not be written.
  int i;
  struct arg {
    uint8_t  reportID;
    uint8_t  address[2];
    uint8_t  count;
    uint8_t  data[count];
  } arg;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | MEM_WRITE; // HID output
  uint16_t wIndex = 0;                    // Interface

  if (address <=0x7f) return -1;
  if (count > 59) count = 59;

  arg.reportID = MEM_WRITE;
  arg.address[0] = address & 0xff;
  arg.address[1] = (address >> 8) & 0xff;
  arg.count = count;
  for ( i = 0; i < count; i++ ) {
    arg.data[i] = data[i];
  }
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
  if (ret < 0) {
    perror("Error in usbWriteMemory_USB1408FS: libusb_control_transfer error");
  }
  return 0;
}

void usbGetAll_USB1408FS(libusb_device_handle *udev, getAllValues* value)
{
  int i;
  int transferred;
  uint8_t reportID = GET_ALL;

  struct t_getAll {
    uint8_t reportID;
    uint8_t values[34];
  } getAll;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                 // HID Set_Report
  uint16_t wValue = (2 << 8) | GET_ALL;  // HID output
  uint16_t wIndex = 0;                   // Interface

  getAll.reportID = GET_ALL;
    
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbGetAll_USB1408FS: libusb_control_transfer error");
  }

  getAll.reportID = 0x0;

  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) &getAll, sizeof(getAll), &transferred, FS_DELAY);

  for (i = 0; i < 4; i++) {
    value->ref_Low[i] =  (int16_t) (getAll.values[2*i] + (getAll.values[2*i+1] << 0x8));
  }
  for (i = 0; i < 4; i++) {
    value->ref_High[i] =  (int16_t) (getAll.values[8 + (2*i)] + (getAll.values[8 + (2*i+1)] << 0x8));
  }
  for (i = 0; i < 8; i++) {
    value->se[i] =  (int16_t) (getAll.values[16 + (2*i)] + (getAll.values[16 + (2*i+1)] << 0x8));
  }
  
  value->dio_portA = getAll.values[32];
  value->dio_portB = getAll.values[33];
}

/* converts signed short value to volts for Single Ended Mode */
float volts_1408FS_SE( const signed short num )
{
  float volt;
  volt = num * 10.0 / 0x1fff + 0.00;
  return volt;
}

/* converts signed short value to volts for Differential Mode */     
float volts_1408FS( const int gain, const signed short num )
{
  float volt = 0.0;

  switch (gain) {
    case BP_20_00V:
      volt = num * 20.0 / 0x1fff;
      break;
    case BP_10_00V:
      volt = num * 10.0 / 0x1fff;
      break;
    case BP_5_00V:
      volt = num * 5.0 / 0x1fff;
      break;
    case BP_4_00V:
      volt = num * 4.0 / 0x1fff;
      break;
    case BP_2_50V:
      volt = num * 2.5 / 0x1fff;
      break;
    case BP_2_00V:
      volt = num * 2.0 / 0x1fff;
      break;
    case BP_1_25V:
      volt = num * 1.25 / 0x1fff;
      break;
    case BP_1_00V:
      volt = num * 1.0 / 0x1fff;
      break;
  }
  return volt;
}
