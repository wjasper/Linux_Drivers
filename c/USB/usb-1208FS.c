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
#include "usb-1208FS.h"

/* Commands and HID Report ID for the USB 1208FS */
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

#define FS_DELAY 200

enum Mode {Differential, SingleEnded};
static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant. 

int init_USB1208FS(libusb_device_handle *udev)
{
  int ret;

  // claim interfaces 1-3 for the USB-1208FS
  //libusb_set_auto_detach_kernel_driver(udev, 1);
  if ((ret = libusb_detach_kernel_driver(udev, 1)) < 0) {
    perror("init_USB1208FS: error detaching kernel");
    return ret;
  }
  if ((ret = libusb_claim_interface(udev, 1)) < 0) {
    perror("init_USB1208FS: Error claiming interface 1");
    return ret;
  }
  if ((ret = libusb_detach_kernel_driver(udev, 2)) < 0) {
    perror("init_USB1208FS: error detaching kernel");
    return ret;
  }
  if ((ret = libusb_claim_interface(udev, 2)) < 0) {
    perror("init_USB1208FS: Error claiming interface 2");
  }
  if ((ret = libusb_detach_kernel_driver(udev, 3)) < 0) {
    perror("init_USB1208FS: error detaching kernel");
    return ret;
  }
  if ((ret = libusb_claim_interface(udev, 3)) < 0) {
    perror("init_USB1208FS: Error claiming interface 3");
  }
  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
  if (wMaxPacketSize < 0) {
  perror("usb1208FS: error in getting wMaxPacketSize");
  }
  return 0;
}

/* configures digital port */
int usbDConfigPort_USB1208FS(libusb_device_handle *udev, uint8_t port, uint8_t direction)
{
/* This command sets the direction of the DIO port to input or output. 
     Port:      0 = Port A,  1 = Port B
     Direction: 0 = output,  1 = input
*/

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
    perror("Error in usbDConfigPort_USB1208FS: libusb_control_transfer error");
    return ret;
  }
  return 0;
}

int usbDIn_USB1208FS(libusb_device_handle *udev, uint8_t port, uint8_t* din_value)
{
  /* This command reads the current state of the DIO ports. The return
     value will be the value seen at the port pins.
  */
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
    perror("Error in usbDIn_USB1208FS: libusb_control_transfer error");
    return ret;
  }
  ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN  | 1, (unsigned char*) &read_port, sizeof(read_port), &transferred, FS_DELAY);
  if (ret < 0) {
    return ret;
  }

  /* don't return values off the stack*/
  if (port == DIO_PORTA) {
    *din_value = read_port.value[0];
  } else {
    *din_value = read_port.value[1];
  }
  return 0;
}

void usbDOut_USB1208FS(libusb_device_handle *udev, uint8_t port, uint8_t value)
{
  /*
   This command writes data to the DIO port bits that are configured as outputs.
     Port: 0 = Port A, 1 = Port B
     Data: value to write to the port
  */
  
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
    perror("Error in usbDOut_USB1208FS: libusb_control_transfer error");
  }
}

void usbAOut_USB1208FS(libusb_device_handle *udev, uint8_t channel, uint16_t value)
{
/*
  This command writes the value to an analog output channel. The value
  is a 16-bit unsigned value, but the DAC is a 12-bit DAC. The lower 4
  bits of the value are ignored by the DAC. The equation for the
  output voltage is:

      V_out = ( k / 2^16 ) * V_ref 

  where k is the value written to the device and V_ref = 4.096V.

  channel: the channel to write (0 or 1)
  value:   the value to write

*/

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
    perror("Error in usbAOut_USB1208FS: libusb_control_transfer error");
  }
}

int usbAOutScan_USB1208FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel,
			  uint32_t count, float *frequency, uint16_t data[], uint8_t options)
{
  /* This command writes values to the analog output channels at a fixed rate. 

      lowchannel:   the first channel of the scan (0 – 1)
      highchannel:  the last channel of the scan (0 – 1)
      count:        the total number of scans to perform

      options:    bit field that controls various options
                  bit 0: 1 = single execution, 0 = continuous execution
                  bit 1: 1 = use external trigger
                  bits 2-7: not used


    The values lowchannel and hichannel specify the channel range for
    the scan. If lowchannel is higher than hichannel, the parameters
    will be reversed in the device (lowchannel must be less than
    hichannel.)  The rate of data output is set by the internal 16-bit
    incrementing timer running at a base rate of 10MHz. The timer is
    controlled by timer_prescale and timer_preload.

    
    The data will be sent in packets utilizing the interrupt out
    endpoint on interface 1. The endpoint allows 64 bytes of data to
    be sent every millisecond, so the theoretical limit is:

         64 bytes/ms = 64,000byte/s = 32,000S/s

    The data will be in the format:
    lowchannel sample 0 : [hichannel sample 0]
    lowchannel sample 1 : [hichannel sample 1]
    lowchannel sample n : [hichannel sample n]

    The external trigger may be used to start data output
    synchronously. If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin outputting data
    at the specified rate.  The data transfer is controlled by the PMD
    using data requests. The USB-1208FS will send an input report on
    interface 0 with the report ID = CMD_AOUTSCAN to request a new
    packet of data.  It will maintain its internal FIFO by requesting
    new data when it is ready.  The count parameter is only used in
    single execution mode. In continuous execution mode data will be
    sent by the host indefinitely, with the host sending an AOutStop
    command to end the scan.
*/

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

  scanReport.reportID = AOUT_SCAN;
  scanReport.lowchannel = lowchannel;
  scanReport.highchannel = highchannel;
  scanReport.count[0] = (uint8_t) count & 0xff;           // low byte
  scanReport.count[1] = (uint8_t) (count >>  8) & 0xff;
  scanReport.count[2] = (uint8_t) (count >> 16) & 0xff;
  scanReport.count[3] = (uint8_t) (count >> 24) & 0xff;   // high byte
  scanReport.options = options;

  byte[0] = AOUT_SCAN;

  if (*frequency > 0.596 && *frequency < 50000) {
    for (scanReport.prescale = 0; scanReport.prescale <= 8; scanReport.prescale++) {
      preload = 10.e6/((*frequency) * (1<<scanReport.prescale));
      if (preload <= 0xffff) {
	scanReport.preload[0] = (uint8_t) preload & 0xff;          // low byte
	scanReport.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
	*frequency = 10.e6/((1<<scanReport.prescale)*preload);
	break;
      }
    }
  } else if (*frequency == 0.0) {
    preload = 0xffff;
    scanReport.preload[0] = (uint8_t) 0xff;                    // low byte
    scanReport.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
    scanReport.prescale = 1;
  } else {
    printf("usbAOutScan_USB1208FS: frequency out of range.\n");
    return -1;
  }
     
  if (scanReport.prescale == 9 || preload == 0) {
    printf("usbAOutScan_USB1208FS: frequency out of range.\n");
    return -1;
  }
  
  /* shift over all data 4 bits */
  for (i = 0; i < num_samples; i++) {
    data[i] <<= 4;
  }
  
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &scanReport, sizeof(scanReport), 5000);
  if (ret < 0) {
    perror("Error in usbAOutScan_USB1208FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) byte, 2, &transferred, FS_DELAY);
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

void usbAOutStop_USB1208FS(libusb_device_handle *udev)
{
  /* This command stops the analog output scan (if running.) */

  uint8_t reportID = AOUT_STOP;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                   // HID Set_Report
  uint16_t wValue = (2 << 8) | AOUT_STOP;  // HID output
  uint16_t wIndex = 0;                     // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbAOutStop_USB1208FS: libusb_control_transfer error");
  }
}


signed short usbAIn_USB1208FS(libusb_device_handle *udev, uint8_t channel, uint8_t range)
{
  /* This command reads the value from an analog input channel,
     setting the desired gain range first.  The returned value is a
     2’s-complement signed 16-bit number.

       channel: the channel to read (0-7)
       range:   the gain range (0-7)
  */

  enum Mode mode;
  int transferred;

  int16_t value;
  uint16_t uvalue;
  uint8_t report[3];
  
  struct ain_t {
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
    perror("Error in usbAIn_USB1208FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1,(unsigned char*) report, sizeof(report), &transferred, FS_DELAY);

  if (mode == Differential) {
    /* the data is a 2's compliment signed 12 bit number */
    value = (int16_t) ( report[1] | (report[2] << 8));
    value /= (1<<4);
  } else {
    /* the data is a  2's compliment signed 11 bit number */
    uvalue = (uint16_t) ( report[1] | (report[2] << 8));
    if (uvalue > 0x7ff0) {
      uvalue = 0;
    } else if (uvalue > 0x7fe0) {
      uvalue = 0xfff;
    } else {
      uvalue >>= 3;
      uvalue &= 0xfff;
    }
    value = uvalue - 0x800;
  }
  return value;
}

void usbAInStop_USB1208FS(libusb_device_handle *udev)
{
  /* This command stops the analog input scan (if running.) */
  
  uint8_t reportID = AIN_STOP;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | AIN_STOP;  // HID output
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbAInStop_USB1208FS: libusb_control_transfer error");
  }
}

int usbAInScan_USB1208FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count,
			  float *frequency, uint8_t options, int16_t sdata[])
{
/*
  This command scans a range of analog input channels and sends the
  readings in interrupt transfers. The gain ranges that are
  currently set on the desired channels will be used (these may be
  changed with AIn or ALoadQueue.

    lowchannel:  the first channel of the scan (0 – 7)
    highchannel: the last channel of the scan (0 – 7)
    count:       the total number of samples to perform, used only in single execution mode
    options:     bit 0: 1 = single execution, 0 = continuous execution
                 bit 1: 1 = immediate transfer mode, 0 = block transfer mode
                 bit 2: 1 = use external trigger
                 bit 3: 1 = debug mode (scan returns consecutive integers instead of
                            sampled data, used for checking for missed data, etc.)
                 bit 4: 1 = use channel gain queue, 0 = use channel parameters specified
                 bits 5-7: not used
    
  The sample rate is set by the internal 16-bit incrementing timer
  running at a base rate of 10MHz. The timer is controlled by
  timer_prescale and timer_preload. These values are only used if the
  device has been set to master the SYNC pin with the SetSync command.

  The data will be returned in packets utilizing interrupt in endpoints. Two endpoints will be
  used; each endpoint allows 64 bytes of data to be sent every millisecond, so the theoretical
  limit is:
      2 endpoints * 64 bytes/ms = 128 bytes/ms = 128,000 bytes/s = 64,000 samples/s

  The data will be in the format:
  lowchannel sample 0 : lowchannel + 1 sample 0 :… : hichannel sample 0
  lowchannel sample 1 : lowchannel + 1 sample 1 :… : hichannel sample 1
  .
  .
  .
  lowchannel sample n : lowchannel + 1 sample n : … : hichannel sample n

  The data will use successive endpoints, beginning with the first
  endpoint at the start of a scan and cycling through the second
  endpoint until reaching the specified count or an AScanStop is sent.
  Immediate transfer mode is used for low sampling rates to avoid
  delays in receiving the sampled data. The data will be sent at the
  end of every timer period, rather than waiting for the buffer to
  fill. Both endpoints will still be used in a sequential manner. This
  mode should not be used if the aggregate sampling rate is greater
  than 2,000 samples per second in order to avoid data loss.

  The external trigger may be used to start data collection
  synchronously. If the bit is set, the device will wait until the
  appropriate trigger edge is detected, then begin sampling data at
  the specified rate. No messages will be sent until the trigger is
  detected.

*/
  
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

  if (lowchannel > highchannel) {
    printf("usbAInScan: lowchannel greater than highchannel.\n");
    return -1;
  }

  num_samples = count;
  if (count%31) {
    count += (31 - count%31);    // fill up entire scan line
  }

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

  // printf("usbAInScan: actual frequency = %f\n", *frequency);

  if (arg.prescale == 9 || preload == 0) {
    printf("usbAInScan: frequency out of range.\n");
    return -1;
  }

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 500);
  if (ret < 0) {
    perror("Error in usbAInScan_USB1208FS: libusb_control_transfer error");
  }

  i = 0;
  pipe = 1;   // Initial Enpoint to receive data.
  
  while (num_samples > 0) {
    ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN |(pipe+2), (unsigned char *) &data, sizeof(data), &transferred, timeout);
    if (ret < 0) {
      perror("usbAInScan_USB1208FS interrupt pipes");
      printf("usbAInScan_USB1208FS: pipe = %d  ret = %d  transferred = %d  num_samples = %d\n",
	     pipe, ret, transferred, num_samples);
      return ret;
    }

    if (num_samples > 31) {
      for (k = 0; k < 31;  k++) {
	sdata[i+k] = data.value[k] / (1<<4);
      }
      num_samples -= 31;
      i += 31;
    } else {   // only copy in a partial scan
      for (k = 0; k < num_samples;  k++) {
	sdata[i+k] = data.value[k] / (1<<4);
      }
      num_samples -= 31;
      i += 31;
      break;
      }
    pipe = pipe%3 + 1;  //pipe should take the values 1, 2 or 3
  }

  usbAInStop_USB1208FS(udev);
  return count;
}

/* scan for Single Ended */
int usbAInScan_USB1208FS_SE(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count,
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
  if (count%31) {
    count += (31 - count%31);    // fill up entire scan line
  }
  
  // Set the channel gain.
  arg.reportID = AIN_SCAN;
  arg.lowchannel = lowchannel + 8;
  arg.highchannel = highchannel + 8;
  arg.count[0] = (uint8_t) count & 0xff;           // low byte
  arg.count[1] = (uint8_t) (count >>  8) & 0xff;
  arg.count[2] = (uint8_t) (count >> 16) & 0xff;
  arg.count[3] = (uint8_t) (count >> 24) & 0xff;   // high byte
  arg.options = options;                        

  if (*frequency > 0.596 && *frequency < 50000) {
    for (arg.prescale = 0; arg.prescale <= 8; arg.prescale++) {
      preload = 10e6/((*frequency) * (1<<arg.prescale));
      if (preload <= 0xffff) {
	arg.preload[0] = (uint8_t) preload & 0xff;          // low byte
	arg.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
	*frequency = 10.e6/((1<<arg.prescale)*preload);
	break;
      }
    }
  } else if (*frequency == 0.0) {
    preload = 0xffff;
    arg.preload[0] = (uint8_t) 0xff;                    // low byte
    arg.preload[1] = (uint8_t) (preload >> 8) & 0xff;   // high byte
    arg.prescale = 1;
  } else {
    printf("usbAOutScan_USB1208FS_SE: frequency out of range.\n");
    return -1;
  }

  if (arg.prescale == 9 || preload == 0) {
    printf("usbAInScan_USB1208FS_SE: frequency out of range.\n");
    return -1;
  }

  /* Set the gain +/-10V */
  nchan = highchannel - lowchannel + 1;
  for (i = 0; i < nchan; i++) {
    chan[i] = lowchannel + i;
    gains[i] = SE_10_00V;
  }
  usbALoadQueue_USB1208FS(udev, nchan, chan, gains);
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 500);
  if (ret < 0) {
    perror("Error in usbAInScan_USB1208FS_SE: libusb_control_transfer error");
  }

  i = 0;
  pipe = 1;   // Initial Enpoint to receive data.
  while ( num_samples > 0 ) {
    ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN |(pipe+2), (unsigned char *) &data, sizeof(data), &transferred, timeout);
    if (ret < 0) {
      perror("Error in usbAInScan_USB1208FS_SE: libusb_control_transfer error");
    } else {
      printf("Scan = %d   num_samples = %d\n", data.scan_index, num_samples);
    }

    if (num_samples > 31) {
      for (k = 0; k < 31;  k++) {
	if (data.value[k] > 0x7ff0) {
	  data.value[k] = 0;
	} else if (data.value[k] > 0x7fe0) {
	  data.value[k] = 0xfff;
	} else {
	  data.value[k] >>= 3;
	  data.value[k] &= 0xfff;
	}
	sdata[i+k] =  data.value[k]- 0x800;
      }
      num_samples -= 31;
      i += 31;
    } else {   // only copy in a partial scan
      for (k = 0; k < num_samples;  k++) {
	if (data.value[k] > 0x7ff0) {
	  data.value[k] = 0;
	} else if (data.value[k] > 0x7fe0) {
	  data.value[k] = 0xfff;
	} else {
	  data.value[k] >>= 3;
	  data.value[k] &= 0xfff;
	}
	sdata[i+k] =  data.value[k] - 0x800;
      }
      num_samples = 0;
      break;
    }
    pipe = (pipe)%3 + 1;  //pipe should take the values 1, 2 or 3
  }

  usbAInStop_USB1208FS(udev);
  return 0;
}

void usbALoadQueue_USB1208FS(libusb_device_handle *udev, uint8_t num, uint8_t chan[], uint8_t gains[])
{
  /*
    
  The device can scan analog input channels with different gain
  settings. This function provides the mechanism for configuring each
  channel with a unique range. 

    num:  the number of channel / gain pairs to follow (max 8)
    chan[]: array of the channel numbers (0 – 7)
    gain[]: array of the  gain ranges (0 – 7)
  */

  struct aLoadQueue_t {
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
    perror("Error in usbAInLoadQueue_USB1208FS: libusb_control_transfer error");
  }
}

void usbInitCounter_USB1208FS(libusb_device_handle *udev)
{
  /* This command initializes the event counter and resets the count to zero. */
  
  uint8_t reportID = CINIT;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;               // HID Set_Report
  uint16_t wValue = (2 << 8) | CINIT;  // HID output
  uint16_t wIndex = 0;                 // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbInitCounter_USB1208FS: libusb_control_transfer error");
  }
}

uint32_t usbReadCounter_USB1208FS(libusb_device_handle *udev)
{
  /*
    This function reads the 32-bit event counter on the device. This
    counter tallies the transitions of an external input attached to
    the CTR pin on the screw terminal of the device.
   */

  uint32_t value;
  struct counter_t {
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
    perror("Error in usbCounter_USB1208FS: libusb_control_transfer error");
  }

  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) &counter, sizeof(counter), &transferred, FS_DELAY);
  value =   counter.value[0] | (counter.value[1] << 8) | (counter.value[2] << 16) | (counter.value[3] << 24);

  return value;
}


void usbBlink_USB1208FS(libusb_device_handle *udev)
{
  /* blinks the LED of USB device */
  
  uint8_t reportID = BLINK_LED;
  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                  // HID Set_Report
  uint16_t wValue = (2 << 8) | BLINK_LED; // HID output
  uint16_t wIndex = 0;                    // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbBlink_USB1208FS: libusb_control_transfer error");
  }
}

int usbReset_USB1208FS(libusb_device_handle *udev)
{
  /* This function causes the device to perform a reset. The device disconnects from the USB bus and
     resets its microcontroller.
  */
  int ret;
  uint8_t reportID = RESET;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                // HID Set_Report
  uint16_t wValue = (2 << 8) | RESET;   // HID output
  uint16_t wIndex = 0;                  // Interface

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbReset_USB1208FS: libusb_control_transfer error");
  }
  return 0;
}

void usbSetTrigger_USB1208FS(libusb_device_handle *udev, uint8_t type)
{
  /* This function configures the external trigger for analog
     input. The trigger may be configured to activate with either a
     logic rising edge or falling edge input. Once the trigger is
     received, the analog input will proceed as configured. The
     EXTTRIG option must be used in the AInScan command to utilize
     this feature.

     type: the type of trigger 
           0 = external trigger falling edge, 1 = external trigger rising edge
    */
  
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
    perror("Error in usbSetTriggert_USB1208FS: libusb_control_transfer error");
  }
}

void usbSetSync_USB1208FS(libusb_device_handle *udev, uint8_t type)
{
  /* 
    This command configures the sync signal. The sync signal may be
    used to synchronize the analog input scan of multiple
    devices. When multiple devices are to be used, one device is
    selected as the master and the rest as slaves. The sync signal of
    all devices must be wired together. The master will output a pulse
    every sample, and all of the devices will acquire their samples
    simultaneously.

    This may also be used to pace one or more devices from an external
    TTL/CMOS clock signal (max rate = 50kHz.)  This may also be used
    with an external trigger; the external trigger signal should be
    brought to the master device, and all devices will begin sampling
    when the master is triggered.

    If a device is configured as a slave, it will not acquire data
    when given an AInScan command until it detects a pulse on the sync
    input.  The device will switch the SYNC pin to the appropriate
    input / output state when this command is received.

    type: 0 = master, 1 = slave
  */
  
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
    perror("Error in usbSetSync_USB1208FS: libusb_control_transfer error");
  }
}

uint16_t usbGetStatus_USB1208FS(libusb_device_handle *udev)
{
  /* This command retrieves the status of the device. */
  
  int transferred;
  uint16_t status;

  struct statusReport_t {
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
    perror("Error in usbGetStatus_USB1208FS: libusb_control_transfer error");
  }

  do {
    statusReport.reportID = 0;
    ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) &statusReport, sizeof(statusReport), &transferred, FS_DELAY);
    if (ret < 0) {
    perror("Error in usbGetStatus_USB1208FS: libusb_interrupt_transfer error");
    return -1;
    }
  } while (statusReport.reportID != GET_STATUS);

  status = (uint16_t) (statusReport.status[0] | (statusReport.status[1] << 8));
  status &= 0x8003;
  return status;
}

void usbReadMemory_USB1208FS( libusb_device_handle *udev, uint16_t address, uint8_t count, uint8_t memory[])
{
  /* This command reads data from the configuration memory (EEPROM.) All of the memory may be read

     Addresses 0x000 - 0x07F are reserved for firmware data
     Addresses 0x080 - 0x3FF are available for use as calibraion or user data

     address: the start address for the read.
     count:  the number of bytes to read (62 max)
  */
  
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

  int ret;
  int transferred;
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
    perror("Error in usbReadMemory_USB1208FS: libusb_control_transfer error");
  }

  memRead.reportID = 0x0;
  // always read 63 bytes regardless.  Only the first count are meaningful.
  ret = libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) &memRead, 63, &transferred, FS_DELAY);
  if (ret < 0) {
    perror("Error in usbReadMemory_USB1208FS: usb_interrupt_read() reading error.");
    printf("Address = %#x  Count = %d  Number of bytes read = %d \n", address, count, transferred);
  }
  if (memRead.reportID != MEM_READ) {
    printf("Error in Reading Memory from EEPROM!\n");
    libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
    libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) &memRead, count+1, &transferred, FS_DELAY);
  }
  memcpy(memory, &memRead.data[0], count);
}

int usbWriteMemory_USB1208FS(libusb_device_handle *udev, uint16_t address, uint8_t count, uint8_t* data)
{
  /*
    This command writes to the non-volatile EEPROM memory on the
    device. The non-volatile memory is used to store calibration
    coefficients, system information, and user data. 

    Locations 0x00-0x7F are reserved for firmware and my not be written.
  */
  
  int i;
  struct mem_write_report_t {
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
  for (i = 0; i < count; i++) {
    arg.data[i] = data[i];
  }

  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &arg, sizeof(arg), 5000);
  if (ret < 0) {
    perror("Error in usbWriteMemory_USB1208FS: libusb_control_transfer error");
  }
  return 0;
}

void usbGetAll_USB1208FS(libusb_device_handle *udev, uint8_t data[])
{
  /* This command reads the value from all analog input channels and digital I/Os.  */

  int transferred;
  uint8_t reportID = GET_ALL;

  struct getAll_t {
    uint8_t reportID;
    uint8_t chan0[0];
    uint8_t chan1[0];
    uint8_t chan2[0];
    uint8_t chan3[0];
    uint8_t chan4[0];
    uint8_t chan5[0];
    uint8_t chan6[0];
    uint8_t chan7[0];
    uint8_t dio_portA;
    uint8_t dio_portB;
  } getAll;

  int ret;
  uint8_t request_type = LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT;
  uint8_t request = 0x9;                 // HID Set_Report
  uint16_t wValue = (2 << 8) | GET_ALL;  // HID output
  uint16_t wIndex = 0;                   // Interface

  getAll.reportID = GET_ALL;
    
  ret = libusb_control_transfer(udev, request_type, request, wValue, wIndex, (unsigned char*) &reportID, sizeof(reportID), 5000);
  if (ret < 0) {
    perror("Error in usbGetAll_USB1208FS: libusb_control_transfer error");
  }
  libusb_interrupt_transfer(udev, LIBUSB_ENDPOINT_IN | 1, (unsigned char *) &getAll, sizeof(getAll), &transferred, FS_DELAY);
}

/* converts signed short value to volts for Single Ended Mode */
float volts_SE( const signed short num )
{
  float volt = 0.0;
  volt = num * 10.0 / 0x7ff;
  return volt;
}

/* converts signed short value to volts for Differential Mode */     
float volts_FS( const int gain, const signed short num )
{
  float volt = 0.0;

  switch( gain ) {
    case BP_20_00V:
      volt = num * 20.0 / 0x7ff;
      break;
    case BP_10_00V:
      volt = num * 10.0 / 0x7ff;
      break;
    case BP_5_00V:
      volt = num * 5.0 / 0x7ff;
      break;
    case BP_4_00V:
      volt = num * 4.0 / 0x7ff;
      break;
    case BP_2_50V:
      volt = num * 2.5 / 0x7ff;
      break;
    case BP_2_00V:
      volt = num * 2.0 / 0x7ff;
      break;
    case BP_1_25V:
      volt = num * 1.25 / 0x7ff;
      break;
    case BP_1_00V:
      volt = num * 1.0 / 0x7ff;
      break;
  }
  return volt;
}
