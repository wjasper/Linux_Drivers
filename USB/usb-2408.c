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

#include "pmd.h"
#include "usb-2408.h"

#define HS_DELAY 1000

// Globals
Thermocouple_Data ThermocoupleData[8];
double TypeKReverseExtra[3];

void usbBuildGainTable_USB2408(libusb_device_handle *udev, double table[NGAINS_2408][2])
{
  /*
    Builds a lookup table of calibration coefficents to translate values into voltages:
         voltage = value*table[gain#][0] + table[gain#][1]
    only needed for fast lookup.
  */
  int j, k;
  uint16_t address = 0x00B0;

  for (j = 1; j < NGAINS_2408; j++) {
    for (k = 0; k < 2; k++) {
      usbReadMemory_USB2408(udev, 8, address, (uint8_t *) &table[j][k]);
      address += 0x8;
    }
  }
  table[0][0] = table[1][0];
  table[0][1] = table[1][1];
  return;
}

void usbBuildGainTable_USB2408_2AO(libusb_device_handle *udev, double table_AO[NCHAN_AO_2408][2])
{
  /*
    Builds a lookup table of calibration coefficents to translate values into voltages:
       corrected value = value*table[VDAC#][0] + table[VDAC][1]
  */

  int j, k;
  uint16_t address = 0x0180;

  for (j = 0; j < NCHAN_AO_2408; j++) {
    for (k = 0; k < 2; k++) {
      usbReadMemory_USB2408(udev, 8, address, (uint8_t *) &table_AO[j][k]);
      address += 0x8;
    }
  }
  return;
}

void usbBuildCJCGradientTable_USB2408(libusb_device_handle *udev, float CJCGradients[nCJCGrad_2408])
{
  int i;
  uint16_t address = 0x0140;

  for (i = 0; i < nCJCGrad_2408; i++) {
    usbReadMemory_USB2408(udev, 8, address, (uint8_t *) &CJCGradients[i]);
    address += 0x4;
  }
  return;
}

void usbCalDate_USB2408(libusb_device_handle *udev, struct tm *date)
{
    /* This command reads the factory calibration date */

  calibrationTimeStamp calDate;
  uint16_t address = 0x640;  // beginning of MFG Calibration date
  time_t time;

  usbReadMemory_USB2408(udev, sizeof(calDate), address, (uint8_t *) &calDate);
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

/* reads digital port  */
uint8_t  usbDIn_USB2408(libusb_device_handle *udev, uint8_t port)
{
  /*
    This command reads the current state of the digital port pins.
    port:  the port to read (there is only one)
           0  onboard (pins 0-7)
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data = 0x0;

  libusb_control_transfer(udev, requesttype, DIN, (uint16_t) port,  0x0, (unsigned char *) &data, sizeof(data), HS_DELAY);
  return data;
}

/* read/writes digital port latch */
void usbDOut_USB2408(libusb_device_handle *udev, uint8_t value, uint8_t port)
{
  /*
    This command writes the DOut port latch.
    port:  the port latch to write:
           0  onboard (pins 0-7)

	   NOTE: The DIO are open-drain, which when used as an output is capable of sinking up to 150 mA.
	   Writing a "1" to a bit will cause its voltage to go LOW (0V), and writing a "0" to
	   the bit will cause the voltage to go HIGH (5V) by the 47k Ohm pullup resister.
	   See page 23 of the users manual.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  unsigned char buf[2];

  buf[0] = port;
  buf[1] = value;

  libusb_control_transfer( udev, requesttype, DOUT, 0x0, 0x0, buf, sizeof(buf), HS_DELAY );
  return;
}

uint8_t usbDOutR_USB2408(libusb_device_handle *udev, uint8_t port)
{
  /*
    This command reads the DOut port latch.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data;  // value currently present in the output latch

  libusb_control_transfer(udev, requesttype, DOUT, port, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY);
  return data;
}

/***********************************************
 *            Analog Input                     *
 ***********************************************/

int usbAIn_USB2408(libusb_device_handle *udev, uint8_t channel, uint8_t mode, uint8_t range, uint8_t rate, uint8_t *flags)
{
  /* 
     The command returns the value from the specified analog input channel.  The channel
     configuration is specified in the command.  The command will result in a bus stall
     if an AInScan is currently running.

     channel: 0-16
     mode:    0-10 (see usb-2408.h)
     range:   0-8  (see usb-2408.h)
     rate:    0-15 (see usb-2408.h)
  */

  uint32_t data;
  uint16_t input1;
  uint16_t input2;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  input1 = (mode << 8) | channel;
  input2 = (rate << 8) | range;

  libusb_control_transfer(udev, requesttype, AIN, input1, input2, (unsigned char *) &data, sizeof(data), HS_DELAY);
  //  printf("input1 = %#x    input2 = %#x    data = %#x\n", input1, input2, data);
  *flags = (data >> 24);
  data &= 0x00ffffff;
  return int24ToInt(data);
}

void usbAInScanStop_USB2408(libusb_device_handle *udev)
{
  /*
    This command stops the analog input scan (if running)
  */
  
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

uint8_t  usbAInScanStatus_USB2408(libusb_device_handle *udev, uint16_t *depth)
{
  /*
    This command reads the status of the analog input scan.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct AInStatus_t {
    uint16_t depth;    // number of samples currently in the FIFO (max 512)
    uint8_t status;    // bit 0: 1 = scan running
                       // bit 1: 1 = scan overrun due to fifo full
                       // bit 2: 1 = scan overrun due to pacer period too short for queue
                       // bit 3-7: reserved
  } AInStatus;
  libusb_control_transfer(udev, requesttype, AIN_SCAN_STATUS, 0x0, 0x0, (unsigned char *) &AInStatus, sizeof(AInStatus), HS_DELAY);
  *depth =  AInStatus.depth;
  return AInStatus.status;
}

void usbAInScanQueueWrite_USB2408(libusb_device_handle *udev, AInScanQueue *queue)
{
  /*
    This command reads or writes the analog input scan channel queue.  The
    queue may have a maximum of 64 entries.  The queue can not be mondified
    during an AInScan.

    The minimum pacer period can be calculated from the queue data.  The formula is:
                  
                ---
                \      1
       period = /     ---  + 640 us
		---   rate

      i.e. If you have a queue with the following elements:
           channel 0: 100 SPS
           channel 1: 500 SPS
           channel 2: 60 SPS

     Then the minimum allowable pacer period for this queue would be:
        (1/100 + 640us) + (1/500 + 640us) + (1/60 + 640us) = 30.59 ms

    This results in a maximum rate of 32.69 Hz.

    For each queue entry (See usb-2408.h):
      channel:  the analog input channel
      mode:     the input mode (see AIn)
      range:    the input range (see AIn)
      rate:     the A/D data rate (see AIn)
  */
  
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (queue->count > MAX_QUEUE_SIZE) queue->count = MAX_QUEUE_SIZE;
  libusb_control_transfer(udev, requesttype, AIN_SCAN_QUEUE, 0x0, 0x0, (unsigned char *) queue, (1+queue->count*4), HS_DELAY);
}

void usbAInScanQueueRead_USB2408(libusb_device_handle *udev, AInScanQueue *queue)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AIN_SCAN_QUEUE, 0x0, 0x0, (unsigned char *) queue, sizeof(AInScanQueue), HS_DELAY);
}

double usbAInMinPacerPeriod_USB2408(libusb_device_handle *udev)
{
  AInScanQueue scanQueue;
  double period = 0.0;
  int i;

  // Calculate the minimum allowable pacer period
  usbAInScanQueueRead_USB2408(udev, &scanQueue);
  for(i = 0; i < scanQueue.count; i++) {
    switch(scanQueue.queue[i].rate) {
      case HZ30000: period += 1./30000. + 640.E-6; break;
      case HZ15000: period += 1./15000. + 640.E-6; break;
      case HZ7500:  period += 1./7500.  + 640.E-6; break;
      case HZ3750:  period += 1./3750.  + 640.E-6; break;
      case HZ2000:  period += 1./2000.  + 640.E-6; break;
      case HZ1000:  period += 1./1000.  + 640.E-6; break;
      case HZ500:   period += 1./500.   + 640.E-6; break;
      case HZ100:   period += 1./100.   + 640.E-6; break;
      case HZ60:    period += 1./60.    + 640.E-6; break;
      case HZ50:    period += 1./50.    + 640.E-6; break;
      case HZ30:    period += 1./30.    + 640.E-6; break;
      case HZ25:    period += 1./25.    + 640.E-6; break;
      case HZ15:    period += 1./15.    + 640.E-6; break;
      case HZ10:    period += 1./10.    + 640.E-6; break;
      case HZ5:     period += 1./5.     + 640.E-6; break;
      case HZ2_5:   period += 1./2.5    + 640.E-6; break;      
      default:  printf("Unknown rate.\n"); break;
    }      
  }
  return period;
}

void usbAInScanFlush_USB2408(libusb_device_handle *udev)
{
  int transferred;
  int ret = -1;
  uint8_t data[64];

  do {
    ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) data, 64, &transferred, HS_DELAY);
  } while ((ret >= 0) && (transferred > 0));
}

void usbAInScanStart_USB2408(libusb_device_handle *udev, double frequency, uint16_t count, uint8_t packet_size)
{
  /*
    This command starts an analog input channel scan.  The channel
    configuration for the scan is set with AInScanQueueWrite_US2408().
    This command will result in a bus stall if usbAInScan is currently
    running.

    The sample rate is set by an internal incrementing timer running
    at a base rate of 50 kHz.  The timer is controllered by
    pacer_period.  The timer will be reset and samples acquired when
    its value equals pacer_period.  The equation for calculating
    paercer_period is:

        pacer_period = 50,000 / (sample frequency)

    Every time the pacer timer expires the firmware will acquire all
    of the samples specified in the channel gain queue, using the A/D
    data rates specified in the queue.  Therefore, the pacer period
    specified here is the per-channel scan rate.  The time to acquire
    all of the samples will depend on the number of entries in the
    queue and the data rate for each entry.

    The data will be returned in packets utilizing a bulk IN endpoint.
    Each sample will consist of 4 bytes: the MSB will be the scan
    queue index for the samples (0-63) followed by the 24-bit signed
    data value.  Each packet will have packet_size + 1 samples; use a
    low value of packet_size for slow scan rates to improve latency
    and a high value for fast rates for better performance.

    If a queue item is a thermocouple measurement, the MSB of the data
    will also reflect the burnout detection status in the high bit
    (i.e. If burnout is detected the high bit will be 1).  The next
    highest bit (bit 6 of the MSB) will indicate overrun if set.

    Data will be sent until reaching the specified count or an
    AInScanStop() command is sent.
  */

  double period = 0.0;
  uint32_t pacer_period;
  uint16_t depth;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  struct scanPacket_t {
    uint8_t pacer_period[4]; // pacer timer period = 50 kHz / (sample frequency)
    uint8_t count[2];        // the total number of scans to perform (0 = continuous)
    uint8_t packet_size;     // the number of samples per bulk transfer (0-15)
  } scanPacket;

  period = usbAInMinPacerPeriod_USB2408(udev);
  if (period > 1./frequency) {
    pacer_period = rint(period*50000.);
  } else {
    pacer_period = rint(50000./frequency);
  }
  
  memcpy(scanPacket.pacer_period, &pacer_period, 4);
  memcpy(scanPacket.count, &count, 2);
  scanPacket.packet_size = packet_size;

  if (usbAInScanStatus_USB2408(udev, &depth) & INPUT_SCAN_RUNNING) {
    printf("There are currently %d samples in the FIFO buffer.\n", depth);
    return;
  }
  libusb_control_transfer(udev, requesttype, AIN_SCAN_START, 0x0, 0x0, (unsigned char *) &scanPacket,  sizeof(scanPacket), HS_DELAY);
}

int usbAInScanRead_USB2408(libusb_device_handle *udev, uint16_t count, uint8_t nChan, int32_t *data)
{
  int nbytes = nChan*count*4;
  int transferred;
  int ret = -1;
  
  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) data, nbytes, &transferred, HS_DELAY);
  if (ret < 0) {
    perror("usbAInScanRead_USB2408: error in libusb_bulk_transfer");
  }

  usbAInScanStop_USB2408(udev);
  usbAInScanFlush_USB2408(udev);
  
  return transferred;
}


/***********************************************
 *          Analog Output                      *
 ***********************************************/
void usbAOutScanStop_USB2408_2AO(libusb_device_handle *udev)
{
  /* This command stops the analog output scan (if running) and
     clears the output FIFO data.  Any data in the endpoint buffers will
     be flushed, so this command is useful to issue prior to the
     beginning of an output scan.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, AOUT_SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

uint8_t  usbAOutScanStatus_USB2408_2AO(libusb_device_handle *udev, uint16_t *depth)
{
  /*  This comamnd reads the status of the analog output scan:
      depth: the number of samples currently in the FIFO (max 1024)
      status: bit 0: 1 = scan running
              bit 1: 1 = scan underrun
              bits 2-7: reserved
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct AOutStatus_t {
    uint16_t depth;    // number of samples currently in the FIFO (max 512)
    uint8_t status;    // bit 0: 1 = scan running
                    // bit 1: 1 = scan overrun due to fifo full
                    // bit 2: 1 = scan overrun due to pacer period too short for queue
                    // bit 3-7: reserved
  } AOutStatus;
  libusb_control_transfer(udev, requesttype, AOUT_SCAN_STATUS, 0x0, 0x0, (unsigned char *) &AOutStatus, sizeof(AOutStatus), HS_DELAY);
  *depth =  AOutStatus.depth;
  return AOutStatus.status;
}

void usbAOutScanStart_USB2408_2AO(libusb_device_handle *udev, double frequency, uint16_t scans, uint8_t options)
{
  /* This command configures the analog output channel scan.
     This command will result in a bus stall if an AOUT_SCAN is
     currently running.

     Notes:
     The output scan operates with the host continuously transferring data for the
     outputs until the end of the scan.  If the "scans" parameter is 0, the scan will run
     until the AOutScanStop command is issued by the host; if it is nonzero, the scan
     will stop automatically after the specified number of scans have been output.
     The channels in the scan are selected in the options bit field.  "Scans" refers to
     the number of updates to the channels (if all channels are used, one scan s an
     update to all 2 channels).

     period = 50kHz / frequency

     Multiple channels are updated simultaneously using the same time base.

     The output data is sent using the bulk out endpoint.  The data format is:
     low channel sample 0 : ... : [high channel sample 0]
     low channel sample 1 : ... : [high channel sample 1]
     .
     .
     .
     low channel sample n : ... : [high channel sample n]

     The output data is written to a 512-sample FIFO in the device.  The bulk endpoint
     data is only accepted if there is room in the FIFO.  Output data may be sent to the
     FIFO before the start of the scan, and the FIFO is cleared when the AOutScanStop command
     is received.  The scan will not begin until the command is sent (and output data is in
     the FIFO).  Data will be output until reaching the specified number of scans (in single
     execution mode)or an AOutScanStop command is sent.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  struct scanPacket_t {
    uint16_t pacer_period; // pacer timer period = 50 kHz / (scan frequency)
    uint8_t scans[2];      // the total number of scans to perform (0 = continuous)
    uint8_t options;       // bit 0: 1 = include channel 0 in output scan
		  	   // bit 1: 1 = include channel 1 in output scan
			   // bit 2: 1 = include channel 2 in output scan
			   // bit 3: 1 = include channel 3 in output scan
			   // bits 4-7 reserved
  } scanPacket;
  uint16_t depth;
  
  scanPacket.pacer_period = (uint16_t) rint(50000./frequency);
  memcpy(scanPacket.scans, &scans, 2);
  scanPacket.options = options;

  if (usbAOutScanStatus_USB2408_2AO(udev, &depth) & OUTPUT_SCAN_RUNNING) {
    printf("There are currently %d samples in the Output FIFO buffer.\n", depth);
    return;
  }
  libusb_control_transfer(udev, requesttype, AOUT_SCAN_START, 0x0, 0x0, (unsigned char *) &scanPacket, sizeof(scanPacket), HS_DELAY);
}

void usbAOut_USB2408_2AO(libusb_device_handle *udev, int channel, double voltage, double table_AO[NCHAN_AO_2408][2])
{
  /* This command writes the values for the analog output channels.  The
     values are 16-bit unsigned numbers.  This command will result in a control
     pipe stall if an output scan is running.  The equation for the output voltage is:

           V_out = (value - 32768 / 32768)* V_ref

     where "value" is the value written to the channel and V_ref = 10V.  

     command values:
                     0x00  -  write value to buffer 0
                     0x04  -  write value to buffer 1
                     0x10  -  write value to buffer 0 and load DAC 0
                     0x14  -  write value to buffer 1 and load DAC 0
                     0x20  -  write value to buffer 0 and load DAC 1
                     0x24  -  write value to buffer 1 and load DAC 1
                     0x30  -  write value to buffer 0 and load DAC 0 & DAC 1
                     0x34  -  write value to buffer 1 and load DAC 0 & DAC 1
  */

  double dvalue;
  uint16_t depth;
  uint16_t value;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  struct aOut_t {
    uint8_t value[2];
    uint8_t command;
  } aOut;

  dvalue = voltage*32768./10. + 32768.;
  dvalue = dvalue*table_AO[channel][0] + table_AO[channel][1];  

  if (dvalue >= 0xffff) {
    value = 0xffff;
  } else if (dvalue <= 0) {
    value = 0x0;
  } else {
    value = (short int) dvalue;
  }

  memcpy(aOut.value, &value, 2);
  if (channel == 0) {
    aOut.command = 0x10;
  } else {
    aOut.command = 0x24;
  }

  if (usbAOutScanStatus_USB2408_2AO(udev, &depth) & OUTPUT_SCAN_RUNNING) {
    printf("There are currently %d samples in the Output FIFO buffer.\n", depth);
    return;
  }
  libusb_control_transfer(udev, requesttype, AOUT, 0x0, 0x0, (unsigned char *) &aOut, sizeof(aOut), HS_DELAY);
}

/***********************************************
 *          Miscellaneous Commands             *
 ***********************************************/

void usbCounterInit_USB2408(libusb_device_handle *udev, uint8_t counter)
{
  /*
    This command initializes the 32-bit event counter.  On a write, the
     counter will be initialized to zero.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, (unsigned char *) &counter, sizeof(counter), HS_DELAY);
  
  return;
}

uint32_t usbCounter_USB2408(libusb_device_handle *udev, uint8_t counter)
{
  /*
    This command reads the 32-bit event counter.  
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint32_t counts[2] = {0x0, 0x0};

  libusb_control_transfer(udev, requesttype, COUNTER, 0x0, 0x0, (unsigned char *) &counts, sizeof(counts), HS_DELAY);
  if (counter == COUNTER0) {
    return counts[0];
  } else {
    return counts[1];
  }
}

void usbCJC_USB2408(libusb_device_handle *udev, float temp[2])
{
  /*
    This command reads the CJC sensors.  The value returned is the raw
    CJC sensor reading, not compensated with the stored gradient
    value.  The temperature in degrees Celsius is calculated as:

     T = 128.(value/2^15)
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  short int value[2];
  int i;

  libusb_control_transfer(udev, requesttype, CJC, 0x0, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY);
  for (i = 0; i < 2; i++) {
    temp[i] = value[i]/256.0;  // return value in Celsius
  }
}

/* blinks the LED of USB device */
void usbBlink_USB2408(libusb_device_handle *udev, uint8_t bcount)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t cmd = BLINK_LED;

  printf("Blinking LED (%x) for %d counts\n", cmd, bcount);
  libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &bcount, 1, HS_DELAY);
  return;
}

uint8_t usbStatus_USB2408(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device.
    status:   bit 0:    1 = isolated micro ready for commands
              bits 1-7: reserved
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t status = 0x0;

  libusb_control_transfer(udev, requesttype, GET_STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY);
  return status;
}  

void usbGetSerialNumber_USB2408(libusb_device_handle *udev, char serial[9])
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

void usbSetSerialNumber_USB2408(libusb_device_handle *udev, char serial[9])
{
  /*
    This command writes the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). The new serial number will be programmed but not used until
    hardware reset.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, SERIAL, 0x0, 0x0, (unsigned char *) serial, 8, HS_DELAY);
  return;
}

void usbGetVersion_USB2408(libusb_device_handle *udev, uint16_t version[4])
{
  /*
    This command reads the microcontroller firmware versions.  The firmware
    versions are returned as packed hexadecmal BCD values, i.e. if version
    = 0x0132, then the firmware version is 1.32.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, VERSION, 0x0, 0x0, (unsigned char *) version, 8, HS_DELAY);
}

void usbReadMemory_USB2408(libusb_device_handle *udev, uint16_t length,  uint16_t address, uint8_t *data)
{
  /* This command reads data from the available data EEPROM memory.
     The number of bytes to read is specified in the wLength (for
     writes it is wLength - sizeof(address)).
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, MEMORY, address, 0x0, (unsigned char *) data, length, HS_DELAY);
  return;
}

void usbWriteMemory_USB2408(libusb_device_handle *udev, uint16_t length,  uint16_t address, uint8_t *data)
{
  /* This command writes data to the available data EEPROM memory. 
     The number of bytes to read is specified in the wLength (for
     writes it is wLength - sizeof(address)).  The first 2 byes of data is
     the address.

     Note: this function is not reentrant
  */

  unsigned char *buf;
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  buf = malloc(length + 2);
  memcpy(buf, &address, 2);
  memcpy(&buf[2], data, length);
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, buf, length+2, HS_DELAY);
  free(buf);
  return;
}

void usbReset_USB2408(libusb_device_handle *udev)
{
  /*
    This function causes the device to perform a reset.  The device disconnects from the USB bus and resets
    its microcontroller.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

void usbCalConfig_USB2408(libusb_device_handle *udev, uint8_t value)
{
  /*
    This command will configure the calibration source.
    value =  0:  +0.078125V
             1:  -0.078125V
             2:  +0.15625V
	     3:  -0.15625V
	     4:  +0.3125V
     	     5:  -0.3125V
	     6:  +0.625V
     	     7:  -0.625V
	     8:  +1.25V
     	     9:  -1.25V
	    10:  +2.50V
     	    11:  -2.50V
	    12:  +5.00V
     	    13:  -5.00V
	    14:  +10.0V
     	    15:  -10.0V
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, CAL_CONFIG, 0x0, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY);
  return;
}

uint8_t usbADCal_USB2408(libusb_device_handle *udev)
{
  /*
    The command will start the A/D self-calibration process and return the status of the calibration.
  */
  uint8_t requesttype;
  uint8_t status;

  requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, ADCAL, 0x0, 0x0, NULL, 0x0, HS_DELAY);

  requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, ADCAL, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY);

  return status;
}

void usbTCCalMeasure_USB2408(libusb_device_handle *udev, uint8_t value)
{
  /* The command will enable measurement of the TC cal source
     value: 0: normal operation
            1: TC cal source measurment mode (JP3)
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TC_CAL, 0x0, 0x0, (unsigned char *) &value, sizeof(value), HS_DELAY);
  return;
}

void usbUpdateMode_USB2408(libusb_device_handle *udev, uint8_t micro)
{
  /* 
     The command puts the device into firmware update mode which
     allows updating the code for either microcontroller.  The unlock
     code must be correct as a safety mechanism to prevent inadvertent
     code corruption.  If the device is not in update ode, all of the
     other firmware update commands will result in a control pipe
     stall.  A reset command must be issued at the end of the code
     update in order to return the device to operation with the new
     code.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  struct updateMode_t {
    uint8_t unlock_code;
    uint8_t micro;        //0 = USB mciro, 1 = isolated micro 
  } updateMode;
    
  updateMode.unlock_code = 0xad;
  updateMode.micro = micro;

  libusb_control_transfer(udev, requesttype, UPDATE_MODE, 0x0, 0x0, (unsigned char *) &updateMode, sizeof(updateMode), HS_DELAY);
}

void cleanup_USB2408( libusb_device_handle *udev )
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|1);
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT|1);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}

void voltsTos16_USB2408_2AO(double *voltage, int16_t *data, int nSamples, double table_AO[])
{
  /* This routine converts an array of voltages (-10 to 10 volts) to singed 24 bit ints for the DAC */
  int i;
  double dvalue;

  for (i = 0; i < nSamples; i++) {
    dvalue = voltage[i]*(1<<15)/10.;                             /* convert voltage to signed value */
    dvalue = dvalue*table_AO[0] + table_AO[1];                   /* correct for calibration errors */
    if (dvalue >= 32767.) {
      data[i] = 0x7fff;
    } else if (dvalue <= -32768.) {
      data[i] = 0x8000;
    } else {
      data[i] = (short int) dvalue;
    }
  }
}

double volts_USB2408(const int gain, const int value)
{
  double volt = 0.0;
  
  switch (gain) {
    case BP_10V:
      volt = value * 10.0 / 0x7fffff;
      break;
    case BP_5V:
      volt = value * 5.0 / 0x7fffff;
      break;
    case BP_2_5V:
      volt = value * 2.5 / 0x7fffff;
      break;
    case BP_1_25V:
      volt = value * 1.25 / 0x7fffff;
      break;
    case BP_625V:
      volt = value * 0.625 / 0x7fffff;
      break;
    case BP_312V:
      volt = value * 0.312 / 0x7fffff;
      break;
    case BP_156V:
      volt = value * 0.156 / 0x7fffff;
      break;
    case BP_078V:
      volt = value * 0.078 / 0x7fffff;
      break;
  }
  return volt;
}

double tc_temperature_USB2408(libusb_device_handle *udev, int tc_type, uint8_t channel)
{
  int value;          // integer value of the temperature
  uint8_t flag;
  double tc_voltage;
  double CJC_Temp;
  float cjc_array[8];
  double table_AI[NGAINS_2408][2];
  float CJCGradients[nCJCGrad_2408];

  usbBuildCJCGradientTable_USB2408(udev, CJCGradients);
  usbBuildGainTable_USB2408(udev, table_AI);

  // Read the raw voltage (Mode = 4, Range = +/- .078V, Rate = 1kS/s)
  value = usbAIn_USB2408(udev, channel, 4, 8, 5, &flag);
  if (flag & 0x80) {
    printf("TC open detected.  Check wiring on channel %d\n", channel);
    return -1;
  }
  // Apply calibration offset from Gain Table (EEPROM) address 0x0130 (slope) and 0x0138 (offset)
  value = value*table_AI[BP_078V][0] + table_AI[BP_078V][1];
  // Calculate the TC voltage from the corrected values
  tc_voltage = (value * 2. * 0.078125) / 16777216.;
  // Read the correct CJC block from the array
  usbCJC_USB2408(udev, cjc_array);
  // Correct the CJC Temperature by the CJCGradiant for the appropriate channel
  CJC_Temp = cjc_array[channel/4] - CJCGradients[channel];
  // Calculate the CJC voltage using the NIST polynomials and add to tc_voltage in millivolts
  tc_voltage = NISTCalcVoltage(tc_type, CJC_Temp) + 1000.*tc_voltage;
  // Calcualate actual temperature using reverse NIST polynomial.
  return (NISTCalcTemp(tc_type, tc_voltage));
}
