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
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include "pmd.h"
#include "usb-20X.h"

#define MAX_COUNT     (0xffff)
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
  float table_AIN[NCHAN_USB20X][2];
  struct tm calDate;
  int ch, nchan, chan, sample, scan;
  int i;
  uint8_t input;
  int temp;
  uint8_t options;
  char serial[9];
  uint8_t channel;
  uint16_t value;
  uint32_t count;
  double frequency;
  int ret;
  uint16_t *sdataIn; // holds 16 bit unsigned analog input data
  uint16_t dataC[1024][8];  // corrected data
  int aOutEnabled = 0;
  int flag;
  unsigned int timeout;  // in milliseconds

start:
  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB201_PID, NULL))) {
    printf("Success, found a USB 201!\n");
  } else if ((udev = usb_device_find_USB_MCC(USB202_PID, NULL))) {
    printf("Success, found a USB 202!\n");
    aOutEnabled = 1;
  } else if ((udev = usb_device_find_USB_MCC(USB204_PID, NULL))) {
    printf("Success, found a USB 204!\n");
  } else if ((udev = usb_device_find_USB_MCC(USB205_PID, NULL))) {
    printf("Success, found a USB 205!\n");
    aOutEnabled = 1;
  } else {
    printf("Failure, did not find a USB 20X!\n");
    return 0;
  }

  // some initialization

  //print out the wMaxPacketSize.  Should be 64
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  usbBuildGainTable_USB20X(udev, table_AIN);
  for (i = 0; i < NCHAN_USB20X; i++) {
    printf("Calibration Table: %d   Slope = %f   Offset = %f\n", i, table_AIN[i][0], table_AIN[i][1]);
  }

  usbCalDate_USB20X(udev, &calDate);
  printf("\n");
  printf("MFG Calibration date = %s\n", asctime(&calDate));
    
  while(1) {
    printf("\nUSB 200 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to test digital IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'C' to test continuous sampling.\n");
    printf("Hit 'o' to test Analog Output (202/205 only).\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
        usbBlink_USB20X(udev, options);
	break;
      case 'c':
        usbCounterInit_USB20X(udev);
	usbDTristateW_USB20X(udev, 0xf0);
        printf("Connect DIO0 to CTR0\n");
	usbDLatchW_USB20X(udev, 0x0);
        toContinue();
	for (i = 0; i < 100; i++) {
	  usbDLatchW_USB20X(udev, 0x1);
	  usbDLatchW_USB20X(udev, 0x0);
        }
        printf("Count = %d.  Should read 100.\n", usbCounter_USB20X(udev));
        break;      
      case 'd':
        printf("\nTesting Digital I/O...\n");
	printf("connect pins DIO[0-3] <--> DIO[4-7]\n");
	usbDTristateW_USB20X(udev,0xf0);
	printf("Digital Port Tristate Register = %#x\n", usbDTristateR_USB20X(udev));
	do {
          printf("Enter a  number [0-0xf] : " );
          scanf("%x", &temp);
	  temp &= 0xf;
          usbDLatchW_USB20X(udev, (uint8_t)temp);
	  temp = usbDLatchR_USB20X(udev);
          input = usbDPort_USB20X(udev);
	  input = (input >> 0x4) & 0xf;
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, temp);
	  for (i = 0; i < 4; i++) {
	    printf("Bit %d = %d\n", i, (temp>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'i':
	printf("Input channel [0-7]: ");
	scanf("%hhd", &channel);
	for (i = 0; i < 20; i++) {
	  value = usbAIn_USB20X(udev, channel);
	  value = rint(value*table_AIN[channel][0] + table_AIN[channel][1]);
	  printf("Channel %d   Sample[%d] = %#x Volts = %lf\n",
		 channel,  i, value, volts_USB20X(value));
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing USB-20X Analog Input Scan.\n");
        printf("Enter number of scans: ");
        scanf("%d", &count);
	printf("Enter number of channels [1-8]: ");
        scanf("%d", &nchan);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
	if (frequency < 100.) {
	  options = IMMEDIATE_TRANSFER_MODE;
	} else {
	  options = 0x0;
	}
	// The total number of bytes returned is 2*nchan*count
	if ((sdataIn = malloc(2*nchan*count)) == NULL) {
	  perror("Error in malloc");
	  break;
	}
	for (i = 0; i < nchan*count; i++) {
	  sdataIn[i] = 0xbeef;
	}
        usbAInScanStop_USB20X(udev);
	usbAInScanClearFIFO_USB20X(udev);
	channel = 0x0;
	// Set one bit for each active channel
	for (i = 0; i < nchan; i++) {
	  channel |= (0x1 << i);
	}
	timeout = 400000*frequency/(count) + 1000;
	usbAInScanStart_USB20X(udev, count, frequency, channel,  options, 0, 0);
	ret = usbAInScanRead_USB20X(udev, count, nchan, sdataIn, options, timeout);
	if (ret < 0) {
	  printf("try increasing timeout.\n");
	  break;
	}
	printf("Number samples read = %d\n", ret/2);
	for (i = 0; i < count; i++) { // scan count
	  for (chan = 0; chan < nchan; chan++) { // channel count
	    sample = i*nchan + chan;
	    sdataIn[sample] = rint(sdataIn[sample]*table_AIN[chan][0] + table_AIN[chan][1]);
	    printf("Channel %d  Sample[%d] = %#x Volts = %lf\n", chan,
		   sample, sdataIn[sample], volts_USB20X(sdataIn[sample]));
	  }
	}
	free(sdataIn);
        break;
      case 'C':
	printf("Testing USB-20X Analog Input Scan in Continuous mode \n");
	printf("Enter number of channels [1-8]: ");
        scanf("%d", &nchan);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
	printf("Hit any key to exit\n");

	if (frequency < 100.) {
	  options = IMMEDIATE_TRANSFER_MODE;
	} else {
	  options = 0x0;
	}
	for (i = 0; i < nchan; i++) {
	  channel |= (0x1 << i);
	}
	usbAInScanStop_USB20X(udev);
	count = 128;          // number of scans
        usbAInScanStop_USB20X(udev);
	usbAInScanClearFIFO_USB20X(udev);
	// The total number of bytes returned is 2*nchan*count
	if ((sdataIn = malloc(2*nchan*count)) == NULL) {
	  perror("Error in malloc");
	  break;
	}
        sleep(1);
	i = 0;
	usbAInScanStart_USB20X(udev, 0, frequency, channel, options, 0, 0);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  ret = usbAInScanRead_USB20X(udev, count, nchan, sdataIn, options | CONTINUOUS, 5000);
	  for (scan = 0; scan < count; scan++) {    // for each scan 
	    for (chan = 0; chan < nchan; chan++) {  // for each channel in a scan
	      dataC[scan][chan] = rint(sdataIn[scan*nchan+chan]*table_AIN[chan][0] + table_AIN[chan][1]);
	    }
	  }
          if (i%50 == 0) {
            printf("Scan = %d\n", i);
	  }
          i++;
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	free(sdataIn);
        usbAInScanStop_USB20X(udev);
        sleep(2); // let things settle down.
        break;
      case 'o':
	if (!aOutEnabled) {
          printf("Analog Output not supported for this device.\n");
          break;
	}
	printf("Enter output channel [0-1]: ");
	scanf("%hhd", &channel);
	printf("Input value [0-4095]: ");
	scanf("%hd", &value);
	usbAOut_USB20X(udev, channel, value);
        break;
      case 'r':
	// with a reset, we need to reestabilsh communications with the device
	printf("Resetting USB20X.  Restarting in 5 seconds ...\n");
        usbReset_USB20X(udev);
	cleanup_USB20X(udev);
	sleep(5); // let things settle down.
	goto start;
        break;
      case 's':
        usbGetSerialNumber_USB20X(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB20X(udev));
	break;
      case 'e':
        cleanup_USB20X(udev);
        return 0;
      default:
        break;
    }
  }
}

