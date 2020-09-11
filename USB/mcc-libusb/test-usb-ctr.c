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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include <unistd.h>

#include "pmd.h"
#include "usb-ctr.h"

#define FALSE 0
#define TRUE 1

/* Test Program */
int toContinue()
{
  int answer;
  answer = 0; //answer = getchar();
  printf("Continue [yY]? ");
  while((answer = getchar()) == '\0' ||
    answer == '\n');
  return ( answer == 'y' || answer == 'Y');
}

int main (int argc, char **argv)
{
  libusb_device_handle *udev = NULL;

  int ch;
  int i, j; 
  int temp;
  int ret;
  int flag;
  int device;
  float period;
  uint16_t version;
  uint16_t status;

  char serial[9];
  uint8_t input;
  uint8_t options;
  uint8_t timer;
  uint8_t debounce;
  uint8_t control;

  int count;
  int numCounters = 4;
  int counter;
  int numBanks = 4;           // each couter comprised of 1-4 banks of 16 bit counters
  int bank;
  int offset;
  double frequency;
  double timer_frequency;

  TimerParams timerParameters[4];
  CounterParams counterParameters[8];
  ScanData scanData;
  uint16_t data[32000];
  uint64_t counter_data[8];     // 64 bit counts

  uint64_t Count;
  double Period;
  double Frequency;

  udev = NULL;
  scanData.frequency = 0;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB_CTR08_PID, NULL))) {
    printf("Success, found a USB-CTR08!\n");
    device = USB_CTR08_PID;
  } else if ((udev = usb_device_find_USB_MCC(USB_CTR04_PID, NULL))) {
    printf("Success, found a USB-CTR04!\n");
    device = USB_CTR04_PID;
  } else {
    printf("Failure, did not find a USB-CTR04/08 series device!\n");
    return 0;
  }
  // some initialization
  usbInit_CTR(udev);
  scanData.status = usbStatus_USB_CTR(udev);

  //print out the wMaxPacketSize.  Should be 512
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev, 0));

  while(1) {
    if (device == USB_CTR08_PID) {
      printf("\nUSB-CTR08 Testing\n");
    } else {
      printf("\nUSB-CTR04 Testing\n");
    }
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'P' to print out the counter parameters\n");
    printf("Hit 'i' for scan input\n");
    printf("Hit 'I' for scan input with continuous readout\n");
    printf("Hit 'C' for continuous scan input\n");
    printf("Hit 'f' for frequency count\n");
    printf("Hit 'd' to test digital I/O\n");
    printf("Hit 'D' to set counter debounce\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 't' to test the timers\n");
    printf("Hit 'T' to print out the timer parameters\n");
    printf("Hit 'L' to print out the scan list\n");
    printf("Hit 'v' to get version numbers\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
        usbBlink_USB_CTR(udev, options);
	break;
      case 'c':
	usbCounterSet_USB_CTR(udev, 0, 0x0);       // set counter 0 to zero
	usbCounterModeW_USB_CTR(udev, 0, 0x0);
	usbCounterOptionsW_USB_CTR(udev, 0, 0x0);  // count on rising edge
        usbCounterGateConfigW_USB_CTR(udev, 0, 0); // disable gate
	usbCounterOutConfigW_USB_CTR(udev, 0, 0);  // Output off
	printf("Connect DIO0 to CTR0\n");
	usbDTristateW_USB_CTR(udev, 0xf0);
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDLatchW_USB_CTR(udev, 0x0);
	  usbDLatchW_USB_CTR(udev, 0x1);
        }
        printf("Count = %lld.  Should read 100.\n", (long long) usbCounter_USB_CTR(udev, 0));
        break;
      case 'P':
	for (i = 0; i < USB_CTR_NCOUNTER; i++) {
	  usbCounterParamsR_USB_CTR(udev, i, &counterParameters[i]);
	  printf("Counter: %d\t Mode Options: %#x\tCounter Options: %#x\tGate Options: %#x\tOutputOptions: %#x\tdebounce: %#x\n",
		 i, counterParameters[i].modeOptions, counterParameters[i].counterOptions, counterParameters[i].gateOptions,
		 counterParameters[i].outputOptions, counterParameters[i].debounce);
	}
	break;
      case 'i':
	printf("Testing scan input\n");
	printf("Connect Timer 1 to Counter 1\n");
	count = 100;       // total number of scans to perform
	frequency = 1000;  // scan rate at 1000 Hz
	numCounters = 5;   // 4 counters and 1 DIO padded with zeros

	// Set up the scan list (use 4 counter 0-3)
	for (counter = 0; counter < numCounters-1; counter++) {
	  for (bank = 0; bank < numBanks; bank++) {
	    scanData.scanList[numBanks*counter + bank] = (counter & 0x7) | (bank & 0x3) << 3 | (0x2 << 5);
	  }
	}
	counter = numCounters - 1;

	//	scanData.scanList[counter*numBanks]   =  USB_CTR_ZERO_FILL;
        scanData.scanList[counter*numBanks]   =  USB_CTR_SCAN_DIO;
	scanData.scanList[counter*numBanks+1] =  USB_CTR_PADZERO;
	scanData.scanList[counter*numBanks+2] =  USB_CTR_PADZERO;
	scanData.scanList[counter*numBanks+3] =  USB_CTR_PADZERO;
	scanData.lastElement = numCounters*numBanks - 1;
	usbScanConfigW_USB_CTR(udev,scanData);
	usbScanConfigR_USB_CTR(udev, &scanData);

	// set up the counters
	for (counter = 0; counter < numCounters; counter++) {
	  usbCounterSet_USB_CTR(udev, counter, 0x0);       // set counter to 0
	  usbCounterModeW_USB_CTR(udev, counter, 0x0);
	  usbCounterOptionsW_USB_CTR(udev, counter, 0);    // count on rising edge
	  usbCounterGateConfigW_USB_CTR(udev, counter, 0); // deable gate
	  usbCounterOutConfigW_USB_CTR(udev, counter, 0);  // output off
	}

	// set up the timer to generate some pulses
	timer = 1;
	timer_frequency = 1000.;
	period = 1000./timer_frequency;
	usbTimerPeriodW_USB_CTR(udev, timer, period);
	usbTimerPulseWidthW_USB_CTR(udev, timer, period/2);
	usbTimerCountW_USB_CTR(udev, timer, 0);
	usbTimerDelayW_USB_CTR(udev, timer, 0);
	usbTimerControlW_USB_CTR(udev, timer, 0x1);

	scanData.count = count;
	scanData.retrig_count = 0;
	scanData.frequency = frequency;
	scanData.options = 0x0;
	scanData.mode = 0x0;

	usbScanStart_USB_CTR(udev, &scanData);
        usbScanRead_USB_CTR(udev, &scanData, data);
	usbTimerControlW_USB_CTR(udev, timer, 0x0);

	for (i = 0; i < count; i++) {
	  for (counter = 0; counter < numCounters; counter++) {
	    offset = i*numCounters*numBanks + counter*numBanks;
            counter_data[counter] = 0;
            for (bank = 0; bank < numBanks; bank++) {
              counter_data[counter] +=  (uint64_t) (data[offset+bank] & 0xffff) << (16*bank);
             }
	  }
	  printf("Scan: %d     ", i);
	  for (counter = 0; counter < numCounters-1; counter++) {
            printf("%lld   ", (long long)counter_data[counter]);
          }
	  offset = (i+1)*numCounters*numBanks - numBanks;
	  printf("    %#x %#x %#x %#x", data[offset], data[offset+1], data[offset+2], data[offset+3]);
          printf("\n");
	}
	break;
      case 'I':
	printf("Testing scan input with continuous readout\n");
	printf("Connect Timer 1 to Counter 1\n");
	count = 100;       // total number of scans to perform
	frequency = 1000;  // scan rate at 1000 Hz
	numCounters = 3;   // 3 counters fails; 4 works but prints error message because final buffer is smaller than 512 bytes
	numBanks = 4;

	// Set up the scan list (use 4 counter 0-3)
	for (counter = 0; counter < numCounters; counter++) {
	  for (bank = 0; bank < numBanks; bank++) {
	    scanData.scanList[numBanks*counter + bank] = (counter & 0x7) | (bank & 0x3) << 3 | (0x2 << 5);
	  }
	}
	scanData.lastElement = numCounters*numBanks - 1;
	usbScanConfigW_USB_CTR(udev,scanData);
	usbScanConfigR_USB_CTR(udev, &scanData);

	// set up the counters
	for (counter = 0; counter < numCounters; counter++) {
	  usbCounterSet_USB_CTR(udev, counter, 0x0);       // set counter to 0
	  usbCounterModeW_USB_CTR(udev, counter, 0x0);
	  usbCounterOptionsW_USB_CTR(udev, counter, 0);    // count on rising edge
	  usbCounterGateConfigW_USB_CTR(udev, counter, 0); // disable gate
	  usbCounterOutConfigW_USB_CTR(udev, counter, 0);  // output off
	}

	// set up the timer to generate some pulses
	timer = 1;
	timer_frequency = 1000.;
	period = 1000./timer_frequency;	 // period in ms
	usbTimerPeriodW_USB_CTR(udev, timer, period);
	usbTimerPulseWidthW_USB_CTR(udev, timer, period / 2);
	usbTimerCountW_USB_CTR(udev, timer, 0);
	usbTimerDelayW_USB_CTR(udev, timer, 0);
	usbTimerControlW_USB_CTR(udev, timer, 0x1);
      
	scanData.count = count;
	scanData.retrig_count = 0;
	scanData.frequency = frequency;
	scanData.options = 0x0;
	scanData.mode = USB_CTR_CONTINUOUS_READOUT;
	//scanData.mode = 0x0;
      
	// Make sure the FIFO is cleared
	usbScanStop_USB_CTR(udev);
	usbScanClearFIFO_USB_CTR(udev);
	usbScanBulkFlush_USB_CTR(udev, 5);

	usbScanStart_USB_CTR(udev, &scanData);
	printf("scanData: lastElement=%d, packet_size=%d, options=0x%x, mode=0x%x\n", 
	       scanData.lastElement, scanData.packet_size, scanData.options, scanData.mode);
	int currentCounter = 0;
	while (currentCounter < count) {
	  int numBytesRead = usbScanRead_USB_CTR(udev, &scanData, data);
	  if (numBytesRead <= 0) {
	    printf("Error, usbScanRead_USB_CTR returned %d bytes\n", numBytesRead);
	    break;
	  }
	  int numSamples = numBytesRead/(numCounters * numBanks * 2);
	  printf("numBytesRead=%d, numSamples=%d\n", numBytesRead, numSamples); 
	  for (i = 0; i < numSamples; i++) {
	    for (counter = 0; counter < numCounters; counter++) {
	      offset = i*numCounters*numBanks + counter*numBanks;
	      counter_data[counter] = 0;
	      for (bank = 0; bank < numBanks; bank++) {
		counter_data[counter] +=  (uint64_t) (data[offset+bank] & 0xffff) << (16*bank);
	      }
	    }
	    printf("Scan: %d     ", currentCounter);
	    for (counter = 0; counter < numCounters; counter++) {
	      printf("%lld   ", (long long)counter_data[counter]);
	    }
	    printf("\n");
	    currentCounter++;
	  }
	}
	usbTimerControlW_USB_CTR(udev, timer, 0x0);
	break;
      case 'C':
	printf("Testing continuous scan input\n");
	printf("Connect Timer 1 to Counter 1\n");
	count = 0;             // set to 0 for continuous scan.  Returns 63 samples per read
	frequency = 10000;     // scan rate at 10000 Hz
	numCounters = 7;
        numBanks = 4;

	// Set up the scan list 
	for (counter = 0; counter < numCounters; counter++) {
	  for (bank = 0; bank < numBanks; bank++) {
	    scanData.scanList[numBanks*counter + bank] = (counter & 0x7) | (bank & 0x3) << 3 | (0x2 << 5);
	  }
	}
	scanData.lastElement = numCounters*numBanks - 1;
	usbScanConfigW_USB_CTR(udev, scanData);
	usbScanConfigR_USB_CTR(udev, &scanData);

	// set up the counters
	for (counter = 0; counter < numCounters; counter++) {
	  usbCounterSet_USB_CTR(udev, counter, 0x0);       // set counter to 0
	  usbCounterModeW_USB_CTR(udev, counter, 0x0);
	  usbCounterOptionsW_USB_CTR(udev, counter, 0);    // count on rising edge
	  usbCounterGateConfigW_USB_CTR(udev, counter, 0); // deable gate
	  usbCounterOutConfigW_USB_CTR(udev, counter, 0);  // output off
	}

	// set up the timer to generate some pulses
	timer = 1;
	timer_frequency = 100000.;
	period = 1000./timer_frequency;	  // period in ms
	usbTimerPeriodW_USB_CTR(udev, timer, period);
	usbTimerPulseWidthW_USB_CTR(udev, timer, period / 2);
	usbTimerCountW_USB_CTR(udev, timer, 0);
	usbTimerDelayW_USB_CTR(udev, timer, 0);
	usbTimerControlW_USB_CTR(udev, timer, 0x1);

	scanData.count = count;
	scanData.retrig_count = 0;
	scanData.frequency = frequency;
	scanData.options = 0x0;
	scanData.mode = USB_CTR_CONTINUOUS_READOUT;

	usbScanStart_USB_CTR(udev, &scanData);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
        j = 0;
	do {
          ret = usbScanRead_USB_CTR(udev, &scanData, data);
	  printf("Scan: %d  samples read: %d  ", j++, ret/8);  // note 8 bytes per counter
	  // print out the first four values
	  for (counter = 0; counter < numCounters; counter++) {
	    offset = counter*numBanks;
	    counter_data[counter] = 0;
	    for (bank = 0; bank < numBanks; bank++) {
              counter_data[counter] +=  (uint64_t) (data[offset+bank] & 0xffff) << (16*bank);
             }
	  }
	  printf("  %lld  %lld  %lld  %lld\n", (long long) counter_data[0], (long long) counter_data[1],
		 (long long) counter_data[2], (long long) counter_data[3]);
	  if (j == 100) break;
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbScanStop_USB_CTR(udev);
	usbScanClearFIFO_USB_CTR(udev);
	usbScanBulkFlush_USB_CTR(udev, 5);
	usbTimerControlW_USB_CTR(udev, timer, 0x0);
	break;
      case 'f':
	printf("Testing frequency count\n");
	printf("Connect Timer 1 to Counter 1\n");
	// set up the timer to generate some pulses
	timer = 1;
	timer_frequency = 1000.;        // 1000 Hz
	period = 1000./timer_frequency;	// period in ms
	usbTimerPeriodW_USB_CTR(udev, timer, period);
	usbTimerPulseWidthW_USB_CTR(udev, timer, period / 2);
	usbTimerCountW_USB_CTR(udev, timer, 0);
	usbTimerDelayW_USB_CTR(udev, timer, 0);
	usbTimerControlW_USB_CTR(udev, timer, 0x1);

	// configure counter 1
	counter = 1;
        usbCounterSet_USB_CTR(udev, counter, 0x0);       // set counter to 0
        usbCounterModeW_USB_CTR(udev, counter, USB_CTR_PERIOD | USB_CTR_PERIOD_MODE_1000X);
        usbCounterOptionsW_USB_CTR(udev, counter, 0);    // count on rising edge
        usbCounterGateConfigW_USB_CTR(udev, counter, 0); // deable gate
        usbCounterOutConfigW_USB_CTR(udev, counter, 0);  // output off

	sleep(2);
	Count = usbCounter_USB_CTR(udev, counter);
	Period = Count*20.83E-9/1000.;
	Frequency = 1./Period;
	printf("count = %lld    period = %f    frequency = %.1lf Hz  timer frequency = %.1lf Hz\n",
	       (long long) Count, Period, Frequency, timer_frequency);
	break;
      case 'd':
        printf("\nTesting Digital I/O...\n");
	printf("connect pins DIO[0-3] <--> DIO[4-7]\n");
	usbDTristateW_USB_CTR(udev,0xf0);
	printf("Digital Port Tristate Register = %#x\n", usbDTristateR_USB_CTR(udev));
	do {
          printf("Enter a byte number [0-0xf] : " );
          scanf("%x", &temp);
	  temp &= 0xf;
          usbDLatchW_USB_CTR(udev, (uint16_t)temp);
	  temp = usbDLatchR_USB_CTR(udev);
          input = usbDPort_USB_CTR(udev) >> 4;
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, temp);
	  for (i = 0; i < 4; i++) {
	    printf("Bit %d = %d\n", i, (temp>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'D':
	printf("Set and test debounce options.\n");
	printf("Connect TMR0 to COIN.\n");
        frequency = 1000;              // set
	printf("frequency = %.1f\n", frequency);
	timer = 0;
	i = 0;
        for (debounce = 0; debounce <= 16; debounce++) {
	  printf("Setting debounce = %d:  ", debounce);
	  usbCounterDebounceW_USB_CTR(udev, 0, debounce);
	  usbCounterDebounceR_USB_CTR(udev, 0, &input);
	  usbCounterSet_USB_CTR(udev, 0, 0x0);       // set counter 0 to zero
	  usbCounterModeW_USB_CTR(udev, 0, 0x0);
	  usbCounterOptionsW_USB_CTR(udev, 0, 0x0);  // count on rising edge
	  usbCounterGateConfigW_USB_CTR(udev, 0, 0); // disable gate
	  usbCounterOutConfigW_USB_CTR(udev, 0, 0);  // Output off
	  period = 1000./frequency;                  // period in ms
	  usbTimerPeriodW_USB_CTR(udev, timer, period);
	  usbTimerPulseWidthW_USB_CTR(udev, timer, period/2);
	  usbTimerCountW_USB_CTR(udev, timer, 0);
	  usbTimerDelayW_USB_CTR(udev, timer, 0);
	  usbTimerControlW_USB_CTR(udev, timer, 0x1); // turn timer 0 on
	  sleep(1);
	  usbTimerControlW_USB_CTR(udev, timer, 0x0); // turn timer 0 off
	  printf("Count = %lld.\n", (long long) usbCounter_USB_CTR(udev, 0));
	}
	debounce = 0x0;     // disable debounce
	printf("\n");
	for (i = 0; i < 8; i++) {
	  usbCounterDebounceW_USB_CTR(udev, i, debounce);
	  usbCounterDebounceR_USB_CTR(udev, i, &input);
	}
        break;    
      case 'e':
        cleanup_USB_CTR(udev);
        return 0;
      case 'r':
	usbReset_USB_CTR(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB_CTR(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
	status = usbStatus_USB_CTR(udev);
        printf("Status = %#x\n", status);
	if (status & USB_CTR_PACER_RUNNING) {
	  printf("USB-CTR: Pacer running.\n");
	}
	if (status & USB_CTR_SCAN_OVERRUN) {
	  printf("USB-CTR: Scan overrun.\n");
	}
	if (status & USB_CTR_SCAN_DONE) {
	  printf("USB-CTR: Scan done.\n");
	}
	if (status & USB_CTR_FPGA_CONFIGURED) {
	  printf("USB-CTR: FPGA configured.\n");
	}
	if (status & USB_CTR_FPGA_CONFIG_MODE) {
	  printf("USB-CTR: FPGA config mode.\n");
	}
	break;
      case 't':
        printf("Enter frequency of timer: ");
        scanf("%lf", &frequency);
	printf("Enter timer [0-3]: ");
	scanf("%hhd", &timer);
	period = 1000./frequency;     // period in ms
	usbTimerPeriodW_USB_CTR(udev, timer, period);
	usbTimerPulseWidthW_USB_CTR(udev, timer, period / 2);
	usbTimerCountW_USB_CTR(udev, timer, 0);
	usbTimerDelayW_USB_CTR(udev, timer, 0);
	usbTimerControlW_USB_CTR(udev, timer, 0x1);
	usbTimerControlR_USB_CTR(udev, timer, &control);
	printf("Timer control = %hhd\n", control);
	toContinue();
	usbTimerControlW_USB_CTR(udev, timer, 0x0);
	usbTimerControlR_USB_CTR(udev, timer, &control);
	printf("Timer control = %hhd\n", control);
        break;
      case 'T':
	for (i = 0; i < 4; i++) {
  	  usbTimerParamsR_USB_CTR(udev, i, &timerParameters[i]);
	  printf("Timer: %d   period: %#x   pulseWidth: %#x    count: %#x    delay: %#x\n",
		 i, timerParameters[i].period, timerParameters[i].pulseWidth,
		 timerParameters[i].count, timerParameters[i].delay);
	}
	break;
      case 'v':
	version = 0xbeef;
        usbFPGAVersion_USB_CTR(udev, &version);
	printf("FPGA version %02x.%02x\n", version >> 0x8, version & 0xff);
      	break;
      case 'L':
	usbScanConfigR_USB_CTR(udev, &scanData);
	printf("Scan List: ");
	for (i = 0; i < 33; i++) {
	  printf("%#x ", scanData.scanList[i]);
	}
        printf("\n");
	printf("lastElement = %d   count = %d   retrig_count = %d   frequency = %lf  options = %#x   mode = %#x\n",
	       scanData.lastElement, scanData.count, scanData.retrig_count, scanData.frequency, scanData.options, scanData.mode);
	printf("packet_size = %d\n", scanData.packet_size);
        break;    
      default:
        break;
    }
  }
}
