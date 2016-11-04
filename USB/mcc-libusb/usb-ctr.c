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
#include <string.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-ctr.h"
#include "usb-ctr.rbf"

#define HS_DELAY 2000

static int wMaxPacketSize;  // will be the same for all devices of this type so
                            // no need to be reentrant.
void usbInit_CTR(libusb_device_handle *udev)
{
  int i;
  /* This function does the following:
     1. Configure the FPGA
     2. Finds the maxPacketSize for bulk transfers
  */
  wMaxPacketSize = usb_get_max_packet_size(udev, 0);
  if (wMaxPacketSize < 0) {
    perror("usbInit_1608G: error in getting wMaxPacketSize");
  }

  if (!(usbStatus_USB_CTR(udev) & FPGA_CONFIGURED)) {
    usbFPGAConfig_USB_CTR(udev);
    if (usbStatus_USB_CTR(udev) & FPGA_CONFIG_MODE) {
      for (i = 0; i <= (sizeof(FPGA_data) - 64); i += 64) {
	usbFPGAData_USB_CTR(udev, &FPGA_data[i], 64);
      }
      if (sizeof(FPGA_data) % 64) {
	usbFPGAData_USB_CTR(udev, &FPGA_data[i], sizeof(FPGA_data)%64);
      }
      if (!(usbStatus_USB_CTR(udev) & FPGA_CONFIGURED)) {
	printf("Error: FPGA for the USB-CTR is not configured.  status = %#x\n", usbStatus_USB_CTR(udev));
	return;
      }
    } else {
      printf("Error: could not put USB-CTR into FPGA Config Mode.  status = %#x\n", usbStatus_USB_CTR(udev));
      return;
    }
  } else {
    printf("USB-CTR FPGA configured.\n");
    return;
  }
}


/***********************************************
 *            Digital I/O                      *
 ***********************************************/

/* Read/Write digital port tristate register */

uint16_t usbDTristateR_USB_CTR(libusb_device_handle *udev)
{
  /* This command reads or writes the digital port tristate register.
     The tristate register determines if the latch register value is driven onto
     the port pin.  A '1' in the tristate register makes the corresponding
     pin an input, a '0' makes it an output.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDTristateR_USB_CTR: error in libusb_control_transfer().");
  }
  return data;
}

void usbDTristateW_USB_CTR(libusb_device_handle *udev, uint16_t value)
{

  /* This command reads or writes the digital port tristate register.
     The tristate register determines if the latch register value is driven onto
     the port pin.  A '1' in the tristate register makes the corresponding
     pin an input, a '0' makes it an output.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DTRISTATE, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbDTristateW_USB_CTR: error in libusb_control_transfer()");
  }
  return;
}

/* reads digital word  */
uint16_t usbDPort_USB_CTR(libusb_device_handle *udev)
{
  /*
    This command reads the current state of the digital pins.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data;

  if (libusb_control_transfer(udev, requesttype, DPORT, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDPort_USB_CTR: error in libusb_control_transfer().");
  }
  return data;
}

/* read/writes digital port latch */
uint16_t usbDLatchR_USB_CTR(libusb_device_handle *udev)
{
  /*
    This command reads the digital port latch register
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t data = 0x0;

  if (libusb_control_transfer(udev, requesttype, DLATCH, 0x0, 0x0, (unsigned char *) &data, sizeof(data), HS_DELAY) < 0) {
    perror("usbDLatchR_USB_CTR: error in libusb_control_transfer().");
  }
  return data;
}

void usbDLatchW_USB_CTR(libusb_device_handle *udev, uint16_t value)
{
  /*
    This command writes the digital port latch register
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DLATCH, value, 0x0, NULL, 0x0, HS_DELAY) < 0) {
    perror("usbDLatchW_USB_CTR: error in libusb_control_transfer().");
  }
  return;
}

/***********************************************
 *            Counter                          *
 ***********************************************/

void usbCounterSet_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint64_t count)
{
  /*
    This command reads or sets the value of the 64-bit counters.
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterSet_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER, 0x0, counter, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return;
}

uint64_t usbCounter_USB_CTR(libusb_device_handle *udev, uint8_t counter)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint64_t count;

  if (counter >= NCOUNTER) {
    printf("usbCounter_USB_CTR: counter > %d\n", NCOUNTER);
    return 0;
  }
  libusb_control_transfer(udev, requesttype, COUNTER, 0x0, counter, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return count;
}

void usbCounterModeR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *mode)
{
  /*
    This command reads or sets the mode of the counter.
    Mode:
      bits(0-1): type of mode
                Totalize   = 0
                Period     = 1
                Pulsewidth = 2
                Timing     = 3
      bits(2-3): resolution of Period Mode
                Period Mode x1    = 0
                Period Mode x10   = 1
                Period Mode x100  = 2
                Period Mode x1000 = 3
      bits(4-5): Tick size (fundamental unit of time for period, pulsewidth and timing modes)
		20.83 ns   = 0
                208.3 ns   = 1
		2083.3 ns  = 2
		20833.3 ns = 3
      bits(6-7): Reserved 		
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterModeR_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_MODE, 0x0, counter, (unsigned char *) mode, 0x1, HS_DELAY);
}

void usbCounterModeW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t mode)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterModeW_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_MODE, mode, counter, NULL, 0x0, HS_DELAY);
}

void usbCounterOptionsR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *options)
{
  /*
    This command reads or sets the options of the counter.
    Options:
      bit(0):   1 = Clear on Read,  0 = Read has no effect
      bit(1):   1 = No recycle mode (counter stops at 2^64 or 0, unless Range Limit is enabled)
                0 = counter rolls over to a minimum (or maximum) and continues counting.
      bit(2):   1 = Count down,  0 = Count up
      bit(3):   1 = Range Limit on (use max and min limits) 0 = 64-bit counter (max = 2^64, min = 0)
      bit(4):   1 = Count on falling edge,  0 = Count on rising edge
      bit(5-7): Reserved
  */
      
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterOptionsR_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_OPTIONS, 0x0, counter, (unsigned char *) options, 0x1, HS_DELAY);
}

void usbCounterOptionsW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t options)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterOptionsW_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_OPTIONS, options, counter, NULL, 0x0, HS_DELAY);
}

void usbCounterDebounceR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *debounce)
{
  /*
    This command reads or sets the debounce options of a counter.
    Debounce:
      bits(0-4): No debounce:   0
                 500ns:         1
                 1500ns:        2
                 3500ns:        3
		 7500ns:        4
		 15500ns:       5
		 31500ns:       6
		 63500ns:       7
		 127500ns:      8
		 100us:         9
		 300us:        10
		 700us:        11
		 1500us:       12
		 3100us:       13
		 6300us:       14
		 12700us:      15
                 25500us:      16
      bit(5):    1 = Trigger before stable, 0 = Trigger after stable
      bits(6-7): Reserved
  */
      
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterDebounceR_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_DEBOUNCE, 0x0, counter, (unsigned char *) debounce, 0x1, HS_DELAY);
}

void usbCounterDebounceW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t debounce)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterDebounceW_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_DEBOUNCE, debounce, counter, NULL, 0x0, HS_DELAY);
}

void usbCounterGateConfigR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *options)
{
  /*
    This command reads and sets the options of a counter's gate

    Options:
      bit(0):  1 = Enable Gate pin,  0 = Disable Gate pin
      bit(1):  1 = Active Low/Falling/Gate Low increments counter
               0 = Active High/Rising/Gate High increments counter
      bit(2-3): Gate Mode:
        0 = Gate state determines whether counter input is live (typical gate functionality)
        1 = Gate state determines the direction of the counter (instead of CounterOptions(bit 2))
        2 = Gate going active will clear counter
        3 = Gate Trigger: Counter In and Out pins are inactive until the active edge of the gate.
	    This can be used repeatedly with Stop at the Top set to provide controlled outputs.
      bits(4-7) Reserved
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  
  if (counter >= NCOUNTER) {
    printf("usbCounterGateConfigR_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_GATE_CONFIG, 0x0, counter, (unsigned char *) options, 0x1, HS_DELAY);
}

void usbCounterGateConfigW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t options)
{
    uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterGateConfigW_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_GATE_CONFIG, options, counter, NULL, 0x0, HS_DELAY);
}

void usbCounterOutConfigR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t *options)
{
  /*
    The command reads or sets the options of a counter's output
    Options:
      bit(0): 1 = Output on,  0 = Output off
      bit(1): 1 = Initial State is High,  0 = Initial State is Low
      bits(2-7): Reserved
  */
uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  
  if (counter >= NCOUNTER) {
    printf("usbCounterOutConfigR_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_OUT_CONFIG, 0x0, counter, (unsigned char *) options, 0x1, HS_DELAY);
}

void usbCounterOutConfigW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t options)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterOutConfigW_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_OUT_CONFIG, options, counter, NULL, 0x0, HS_DELAY);
}

void usbCounterOutValuesR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint64_t *value)
{
  /*
    This command reads or sets the counter's output values.  These values determine
    when the ouptut changes state.  The output will change state when the counter
    reaches value0, and then change back when the counter reaches value1.  If both
    values are set to the same count, the output will only change state once - at that value.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterOutValuesR_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  if (index >= 2) {
    printf("usbCounterOutValuesR_USB_CTR: index must be 0 or 1.\n");
    return;
  }
  if (libusb_control_transfer(udev, requesttype, COUNTER_OUT_VALUES, index, counter, (unsigned char *) value, 0x8, HS_DELAY) < 0) {
    perror("usbCounterOutValuesR_USB_CTR: error in libusb_control_transfer().");
  }
}

void usbCounterOutValuesW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint64_t value)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterOutValuesW_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  if (index >= 2) {
    printf("usbCounterOutValuesW_USB_CTR: index must be 0 or 1.\n");
    return;
  }
  if (libusb_control_transfer(udev, requesttype, COUNTER_OUT_VALUES, index, counter, (unsigned char *) &value, 0x8, HS_DELAY) < 0) {
    perror("usbCounterOutValuesW_USB_CTR: error in libusb_control_transfer().");
  }
}

void usbCounterLimitValuesR_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint64_t *value)
{
  /*
    This command reads or sets the counter's limit values.
      index: 0 = Minimum Limit Value, 1 = Maximum Limit Value
      value: when the counter reaches this value, roos over or stops depending on the options.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterLimitValuesR_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  if (index >= 2) {
    printf("usbCounterLimitValuesR_USB_CTR: index must be 0 or 1.\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_LIMIT_VALUES, index, counter, (unsigned char *) value, 0x8, HS_DELAY);
}

void usbCounterLimitValuesW_USB_CTR(libusb_device_handle *udev, uint8_t counter, uint8_t index, uint64_t value)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (counter >= NCOUNTER) {
    printf("usbCounterLimitValuesW_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  if (index >= 2) {
    printf("usbCounterLimitValuesW_USB_CTR: index must be 0 or 1.\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_LIMIT_VALUES, index, counter, (unsigned char *) &value, 0x8, HS_DELAY);
}

void usbCounterParamsR_USB_CTR(libusb_device_handle *udev, uint8_t counter, CounterParams *params)
{
  /*
    This command reads or writes all of a given counter's parameters in one call
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[5];

  if (counter >= NCOUNTER) {
    printf("usbCounterParamsR_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }
  libusb_control_transfer(udev, requesttype, COUNTER_PARAMETERS, 0x0, counter, (unsigned char *) data, sizeof(data), HS_DELAY);

  params->counter = counter;
  params->modeOptions = data[0];
  params->counterOptions = data[1];
  params->gateOptions = data[2];
  params->outputOptions = data[3];
  params->debounce = data[4];
}

void usbCounterParamsW_USB_CTR(libusb_device_handle *udev, uint8_t counter, CounterParams params)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[5];

  if (counter >= NCOUNTER) {
    printf("usbCounterParamsW_USB_CTR: counter > %d\n", NCOUNTER);
    return;
  }

  data[0] = params.modeOptions;
  data[1] = params.counterOptions;
  data[2] = params.gateOptions;
  data[3] = params.outputOptions;
  data[4] = params.debounce;

  libusb_control_transfer(udev, requesttype, COUNTER_PARAMETERS, 0x0, counter, (unsigned char *) data, sizeof(data), HS_DELAY);
}

/***********************************************
 *            Counter Scan Functions           *
 ***********************************************/
void usbScanConfigR_USB_CTR(libusb_device_handle *udev, uint8_t lastElement, ScanList *scanList)
{
  /*
    This command reads or writes the input channel configurations.  This command will result
    in a bus stall if a scan is currently running.  The scan list is setup to acquire data
    from the channels in the order that they are placed in the scan list.

    Each counter has 4 banks of 16-bit registers that can be scanned in any order.
    Bank 0 contains bit 0-15, Bank 1 contains bit 16-31, Bank 2 contains bit 32-47
    and Bank 4 contains bit 48-63.  If bit(5) is set to 1, this element will
    be a read of the DIO, otherwise it will use the specified counter and bank.

    ScanList[33] channel configuration:
      bit(0-2): Counter Nuber (0-7)
      bit(3-4): Counter Bank (0-3)
      bit(5): 1 = DIO,  0 = Counter
      bit(6): 1 = Fill with 16-bits of 0's,  0 = normal (This allows for the creation
      of a 32 or 64-bit element of the DIO when it is mixed with 32 or 64-bit elements of counters)
   */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, SCAN_CONFIG, 0x0, lastElement, (unsigned char *) scanList->scanList, 33, HS_DELAY);
  scanList->lastElement = lastElement;
}

void usbScanConfigW_USB_CTR(libusb_device_handle *udev, uint8_t lastElement, ScanList scanList)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, SCAN_CONFIG, 0x0, lastElement, (unsigned char *) scanList.scanList, 33, HS_DELAY);
}

void usbScanStart_USB_CTR(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, uint32_t pacer_period, uint8_t options)
{
  /*
    count:         the total number of scans to perform (0 for continusous scan)
    retrig_count:  the number of scans to perform for each trigger in retrigger mode
    pacer_period:  pacer timer period value (0 for external clock)
    packet_size:   number of samples - 1 to transfer at a time.
    options:       bit field that controls various options
      bit 0:   1 = Maintain counter value on scan start, 0  = Clear counter value on scan start
      bit 1:   Reserved
      bit 2:   Reserved
      bit 3:   1 = use trigger
      bit 4:   Reserved
      bit 5:   Reserved
      bit 6:   1 = retrigger mode,  0 = normal trigger
      bit 7:   Reserved
 
    Notes:

    The pacer rate is set by an internal 32-bit incrementing timer
    running at a base rate of 96 MHz.  The timer is controlled by
    pacer_period.  A pulse will be output at the PACER_OUT pin every
    pacer_period interval regarless of mode.

    If pacer_period is set to 0, the device does not generate an A/D
    clock.  It uses the PACER_IN pin as the pacer source.

    The timer will be reset and sample acquired when its value equal
    timer_period.  The equation for calculating timer_period is:

           timer_period = [96 MHz / (sample frequency)]  - 1

    The data will be returned in packets utilizing a bulk IN endpint.
    The data will be in the format:

    lowchannel sample 0:  low channel + 1 sample 0: ... : hichannel sample 0
    lowchannel sample 1:  low channel + 1 sample 1: ... : hichannel sample 1
    ...
    lowchannel sample n:  low channel + 1 sample n: ... : hichannel sample n
    
    The scan will not begin until the ScanStart command is sent (and
    any trigger conditions are met.)  Data will be sent until reaching
    the specified count or an ScanStop command is sent.

    The packet_size parameter is used for low sampling rates to avoid
    delays in receiving the sampled data.  The buffer will be sent,
    rather than waiting for the buffer to fill.  This mode should not
    be used for high sample rates in order to avoid data loss.

    The external trigger may be used to start data collection
    synchronously.  If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data at
    the specified rate.  No messages will be sent until the trigger is
    detected.

    The external trigger may be used to start data collection
    synchronously.  If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data at
    the specified rate.  No messages will be sent until the trigger is
    detected.
    
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[14];

  memcpy(&data[0], &count, 4);
  memcpy(&data[4], &retrig_count, 4);
  memcpy(&data[8], &pacer_period, 4);
  data[12] = wMaxPacketSize - 1;
  data[13] = options;

  libusb_control_transfer(udev, requesttype, SCAN_START, 0x0, 0x0,(unsigned char *) data, sizeof(data), HS_DELAY);
}

void usbScanStop_USB_CTR(libusb_device_handle *udev)
{
  /*
    The command stops the input scan (if running)
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, SCAN_STOP, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbScanClearFIFO_USB_CTR(libusb_device_handle *udev)
{
  /*
    This command clears the input firmware buffer
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, SCAN_CLEAR_FIFO, 0x0, 0x0, NULL, 0x0, HS_DELAY);
}

void usbScanBulkFlush_USB_CTR(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command flushes the Bulk pipe "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, BULK_FLUSH, count, 0x0, NULL, 0x0, HS_DELAY);
}

int usbScanRead_USB_CTR(libusb_device_handle *udev, int count, int lastElement, uint16_t *data)
{
  char value[64];
  int ret = -1;
  int nbytes = count*lastElement*2;    // nuber of bytes to read;
  int transferred;
  uint8_t status;

  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) data, nbytes, &transferred, HS_DELAY);

  if (ret < 0) {
    perror("usbScanRead_USB_CTR: error in usb_bulk_transfer.");
  }
  if (transferred != nbytes) {
    fprintf(stderr, "usbAInScanRead_USB_CTR: number of bytes transferred = %d, nbytes = %d\n", transferred, nbytes);
  }

  // if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
  if ((nbytes%wMaxPacketSize) == 0) {
    libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|6, (unsigned char *) value, 2, &ret, 100);
  }

  status = usbStatus_USB_CTR(udev);
  if ((status & SCAN_OVERRUN)) {
    printf("Scan overrun.\n");
    usbScanStop_USB_CTR(udev);
    usbScanClearFIFO_USB_CTR(udev);
    usbScanBulkFlush_USB_CTR(udev, 5);
  }

  return ret;
}
  

/***********************************************
 *            Timer                            *
 ***********************************************/
void usbTimerControlR_USB_CTR(libusb_device_handle *udev, uint8_t timer,  uint8_t *control)
{
  /*
    This command reads/writes the timer control register
    control:   bit 0:    1 = enable timer,  0 = disable timer
               bit 1:    1 = timer running, 0 = timer stopped
                        (read only, useful when using count)
               bit 2:    1 = inverted output (active low)
                         0 = normal output (active high)
               bits 3-7: reserved
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (timer > NTIMER) {
    printf("usbTimerControlR_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_CONTROL, 0x0, timer, (unsigned char *) control, sizeof(control), HS_DELAY);
}

void usbTimerControlW_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint8_t control)
{
  /* This command reads/writes the timer control register */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (timer > NTIMER) {
    printf("usbTimerControlW_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_CONTROL, control, timer, NULL, 0x0, HS_DELAY);
}

void usbTimerPeriodR_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t *period)
{
  /*
    The timer is based on a 96 MHz input clock and has a 32-bit period register. The
    frequency of the output is set to:

    frequency = 96 MHz / (period + 1)

    Note that the value for pulseWidth should always be smaller than the value for
    the period register or you may get unexpected results.  This results in a minimum
    allowable value for the period of 1, which sets the maximum frequency to 96 MHz/2.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (timer > NTIMER) {
    printf("usbTimerPeriodR_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_PERIOD, 0x0, timer, (unsigned char *) period, sizeof(period), HS_DELAY);
}

void usbTimerPeriodW_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t period)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (timer > NTIMER) {
    printf("usbTimerControlW_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_PERIOD, 0x0, timer, (unsigned char *) &period, sizeof(period),  HS_DELAY);
}

void usbTimerPulseWidthR_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t *pulseWidth)
{
  /*
    This command reads/writes the timer pulse width register.
    The timer is based on a 96 MHz input clock and has a 32-bit pulse width register.
    The width of the output pulse is set to:

    pulse width = (pulseWidth + 1) / 96 MHz

    Note that the value for pulseWidth should always be smaller than the value for
    the period register or you may get unexpected results.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (timer > NTIMER) {
    printf("usbTimerPulseWidthR_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_PULSE_WIDTH, 0x0, timer, (unsigned char *) pulseWidth, sizeof(pulseWidth), HS_DELAY);
}

void usbTimerPulseWidthW_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t pulseWidth)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (timer > NTIMER) {
    printf("usbTimerPulseWidthW_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_PULSE_WIDTH, 0x0, timer, (unsigned char *) &pulseWidth, sizeof(pulseWidth), HS_DELAY);
}

void usbTimerCountR_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t *count)
{
  /*
    This command reads/writes the timer count register.
    The number of output pulses can be controlled with the count register.  Setting
    this register to 0 will result in pulses being generated until the timer is disabled.
    Setting it to a non-zero value will results in the specified number of pulses being
    generated then the output will go low until the timer is disabled.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (timer > NTIMER) {
    printf("usbTimerCountR_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_COUNT, 0x0, timer, (unsigned char *) count, sizeof(count), HS_DELAY);
}

void usbTimerCountW_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t count)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (timer > NTIMER) {
    printf("usbTimerCountW_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_COUNT, 0x0, timer, (unsigned char *) &count, sizeof(count), HS_DELAY);
}

void usbTimerDelayR_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t *delay)
{
  /*
    This command reads/writes the timer start delay register.  This
    register is the amount of time to delay before starting the timer
    output after enabling the output.  The value specifies the number
    of 64 MHZ clock pulses to delay.  This value may not be written
    while the timer output is enabled.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (timer > NTIMER) {
    printf("usbTimerDelayR_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_START_DELAY, 0x0, timer, (unsigned char *) delay, sizeof(delay), HS_DELAY);
}

void usbTimerDelayW_USB_CTR(libusb_device_handle *udev, uint8_t timer, uint32_t delay)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (timer > NTIMER) {
    printf("usbTimerDelayW_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_START_DELAY, 0x0, timer, (unsigned char *) &delay, sizeof(delay), HS_DELAY);
}

void usbTimerParamsR_USB_CTR(libusb_device_handle *udev, uint8_t timer, TimerParams *params)
{
  /*
    This command reads/writes all timer parameters in one call.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[16];

  if (timer > NTIMER) {
    printf("usbTimerParamsR_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_PARAMETERS, 0x0, timer, (unsigned char *) data, sizeof(data), HS_DELAY);

  params->timer = timer;
  memcpy(&(params->period), &data[0], 4);
  memcpy(&(params->pulseWidth), &data[4], 4);
  memcpy(&(params->count), &data[8], 4);
  memcpy(&(params->delay), &data[12], 4);
}

void usbTimerParamsW_USB_CTR(libusb_device_handle *udev, uint8_t timer, TimerParams params)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[16];

  memcpy(&data[0], &params.period, 4);
  memcpy(&data[4], &params.pulseWidth, 4);
  memcpy(&data[8], &params.count, 4);
  memcpy(&data[12], &params.delay, 4);

  if (timer > NTIMER) {
    printf("usbTimerParamsW_USB_CTR: timer >= %d\n", NTIMER);
    return;
  }
  libusb_control_transfer(udev, requesttype, TIMER_PARAMETERS, 0x0, timer, (unsigned char *) data, sizeof(data), HS_DELAY);
}


/***********************************************
 *            Memory Commands                  *
 ***********************************************/
void usbMemoryR_USB_CTR(libusb_device_handle *udev, uint8_t *data, uint16_t length)
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
    perror("usbMemoryR_USB_CTR: error in reading memory.");
  }
}

void usbMemoryW_USB_CTR(libusb_device_handle *udev, uint8_t *data, uint16_t length)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEMORY, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
}

void usbMemAddressR_USB_CTR(libusb_device_handle *udev, uint16_t address)
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

void usbMemAddressW_USB_CTR(libusb_device_handle *udev, uint16_t address)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, MEM_ADDRESS, 0x0, 0x0, (unsigned char *) &address, sizeof(address), HS_DELAY);
}

void usbMemWriteEnable_USB_CTR(libusb_device_handle *udev)
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

/* blinks the LED of USB device */
void usbBlink_USB_CTR(libusb_device_handle *udev, uint8_t count)
{
  /*
    This command will blink the device LED "count" number of times
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), HS_DELAY);
  return;
}

uint16_t usbStatus_USB_CTR(libusb_device_handle *udev)
{
  /*
    This command retrieves the status of the device.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t status = 0x0;

  libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &status, sizeof(status), HS_DELAY);
  return status;
}

void usbReset_USB_CTR(libusb_device_handle *udev)
{
  /*
    This function causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, NULL, 0, HS_DELAY);
  return;
}

void usbTriggerConfig_USB_CTR(libusb_device_handle *udev, uint8_t options)
{
  /*
    This function configures the Scan trigger.  Once the trigger is
    received, the Scan will proceed as configured.  The "use
    trigger" option must be used in the ScanStart command to
    utilize this feature.

    options:     bit 0: trigger mode (0 = level,  1 = edge)
                 bit 1: trigger polarity (0 = low / falling, 1 = high / rising)
                 bits 2-7: reserved
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, (unsigned char *) &options, sizeof(options), HS_DELAY);
}

void usbTriggerConfigR_USB_CTR(libusb_device_handle *udev, uint8_t *options)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  libusb_control_transfer(udev, requesttype, TRIGGER_CONFIG, 0x0, 0x0, (unsigned char *) options, sizeof(options), HS_DELAY);
}

void usbGetSerialNumber_USB_CTR(libusb_device_handle *udev, char serial[9])
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

/***********************************************
 *          FPGA Commands                      *
 ***********************************************/

void usbFPGAConfig_USB_CTR(libusb_device_handle *udev)
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

void usbFPGAData_USB_CTR(libusb_device_handle *udev, uint8_t *data, uint8_t length)
{
  /*
    This command writes the FPGA configuration data to the device.  This
    command is not accepted unless the device is in FPGA config mode.  The
    number of bytes to be written must be specified in wLength.

    data: max length is 64 bytes
  */
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (length > 64) {
    printf("usbFPGAData_USB_CTR: max length = 64 bytes\n");
    return;
  }
  libusb_control_transfer(udev, requesttype, FPGA_DATA, 0x0, 0x0, (unsigned char *) data, length, HS_DELAY);
}

void usbFPGAVersion_USB_CTR(libusb_device_handle *udev, uint16_t *version)
{
  /*
    This command reads the FPGA version.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  libusb_control_transfer(udev, requesttype, FPGA_VERSION, 0x0, 0x0, (unsigned char *) version, sizeof(uint16_t), HS_DELAY);
}

void cleanup_USB_CTR( libusb_device_handle *udev )
{
  if (udev) {
    libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN|6);
    libusb_release_interface(udev, 0);
    libusb_close(udev);
  }
}
