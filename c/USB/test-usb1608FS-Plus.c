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
#include "usb-1608FS-Plus.h"

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
  libusb_device_handle *udev2 = NULL;  // second device
  struct tm calDate;

  float table_AIN[NGAINS_USB1608FS_PLUS][NCHAN_USB1608FS_PLUS][2];

  int ch;
  int i, j, k, m;
  uint8_t input;
  int temp;
  uint8_t options;
  char serial[9];
  uint8_t channel, channels;
  uint8_t range;
  uint8_t ranges[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int value;
  uint32_t count;
  double frequency;
  int ret;
  uint16_t sdataIn[8*512]; // holds 16 bit unsigned analog input data
  uint16_t dataC[256][8];  // corrected data
  uint16_t data;
  int nchan, repeats, scan;
  int flag;

start:
  udev = NULL;
  udev2 = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB1608FS_PLUS_PID, NULL))) {
    printf("Success, found a USB 1608FS-Plus!\n");
  } else {
    printf("Failure, did not find a USB 1608FS-Plus!\n");
    return 0;
  }

  /******************************** Finding a second device has issues on the Raspberry Pi **************/
  // See if there is a second device:
#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000103)
  if ((udev2 = usb_device_find_USB_MCC(USB1608FS_PLUS_PID, NULL))) {
    printf("Success, found a second USB 1608FS-Plus!\n");
  } else {
    printf("Did not find a second device.\n");
  }
#endif
  
  // some initialization
  //print out the wMaxPacketSize.  Should be 64.
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  usbBuildGainTable_USB1608FS_Plus(udev, table_AIN);
  for (i = 0; i < NGAINS_USB1608FS_PLUS; i++ ) {
    for (j = 0; j < NCHAN_USB1608FS_PLUS; j++) {
      printf("Calibration Table: Range = %d Channel = %d Slope = %f   Offset = %f\n", 
	     i, j, table_AIN[i][j][0], table_AIN[i][j][1]);
    }
  }

  usbCalDate_USB1608FS_Plus(udev, &calDate);
  printf("\n");
  printf("MFG Calibration date = %s\n", asctime(&calDate));
  
  while(1) {
    printf("\nUSB 1608FS-Plus Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'C' to test continuous sampling at 1000 Hz.\n");
    printf("Hit 'd' to test digital IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'x' to test Analog Input Scan (Multi-channel)\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
        usbBlink_USB1608FS_Plus(udev, options);
        if (udev2 != NULL) usbBlink_USB1608FS_Plus(udev2, options);
	break;
      case 'c':
        usbCounterInit_USB1608FS_Plus(udev);
        printf("Connect DIO0 to CTR0\n");
	usbDTristateW_USB1608FS_Plus(udev, 0xf0);
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDLatchW_USB1608FS_Plus(udev, 0x0);
	  usbDLatchW_USB1608FS_Plus(udev, 0x1);
        }
        printf("Count = %d.  Should read 100.\n", usbCounter_USB1608FS_Plus(udev));
        break;      
      case 'd':
        printf("\nTesting Digital I/O...\n");
	printf("connect pins DIO[0-3] <--> DIO[4-7]\n");
	usbDTristateW_USB1608FS_Plus(udev,0xf0);
	printf("Digital Port Tristate Register = %#x\n", usbDTristateR_USB1608FS_Plus(udev));
	do {
          printf("Enter a  number [0-0xf] : " );
          scanf("%x", &temp);
	  temp &= 0xf;
          usbDLatchW_USB1608FS_Plus(udev, (uint8_t)temp);
	  temp = usbDLatchR_USB1608FS_Plus(udev);
          input = usbDPort_USB1608FS_Plus(udev);
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
        printf("Input range [0-7]: ");
	scanf("%hhd", &range);
	for (i = 0; i < 20; i++) {
	  value = (int) usbAIn_USB1608FS_Plus(udev, channel, range);
	  value = rint(value*table_AIN[range][channel][0] + table_AIN[range][channel][1]);
	  if (value > 0xffff) value = 0xffff;    // check for overflow
	  if (value < 0) value = 0x0;            // check for underflow
	  printf("Range %d  Channel %d   Sample[%d] = %#x Volts = %lf\n",
		 range, channel,  i, value, volts_USB1608FS_Plus(value, range));
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing USB-1608FS_Plus Analog Input Scan.\n");
	usbAInScanStop_USB1608FS_Plus(udev);
        printf("Enter number of scans (less than 512): ");
        scanf("%d", &count);
	printf("Input channel 0-7: ");
        scanf("%hhd", &channel);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
        printf("Enter Range [0-7]: ");
        scanf("%hhd", &range);
        ranges[channel] = range;
        usbAInScanStop_USB1608FS_Plus(udev);
	usbAInScanClearFIFO_USB1608FS_Plus(udev);
        usbAInScanConfig_USB1608FS_Plus(udev, ranges);
	sleep(1);
        usbAInScanConfigR_USB1608FS_Plus(udev, ranges);
        for (i = 0; i < 8; i++) {
          printf("Channel %d     range %d\n", i, ranges[i]);
	}
	if (frequency < 100.) {
	  options = (IMMEDIATE_TRANSFER_MODE | INTERNAL_PACER_ON);
	} else {
	  options = (BLOCK_TRANSFER_MODE | INTERNAL_PACER_ON);
	}
	memset(sdataIn, 0x0, sizeof(sdataIn));
	usbAInScanStart_USB1608FS_Plus(udev, count, frequency, (0x1<<channel), options);
	ret = usbAInScanRead_USB1608FS_Plus(udev, count, 1, sdataIn, options);
	printf("Number samples read = %d\n", ret/2);
	for (i = 0; i < count; i++) {
	  value = rint(sdataIn[i]*table_AIN[range][channel][0] + table_AIN[range][channel][1]);
	  if (value > 0xffff) value = 0xffff;   // check for overflow
	  if (value < 0) value = 0;             // check for underflow
	  sdataIn[i] = (uint16_t) value;
	  printf("Range %d Channel %d  Sample[%d] = %#x Volts = %lf\n", range, channel,
		 i, sdataIn[i], volts_USB1608FS_Plus(sdataIn[i], range));
	}
        break;
      case 'C':
      	printf("Testing USB-1608FS_Plus Analog Input Scan in Continuous mode 8 channels\n");
        printf("Hit any key to exit\n");
        count = 0;        // for continuous mode
	printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
	printf("Enter number of channels [1-8]: ");
	scanf("%d", &nchan);
	channels = 0;
	for (i = 0; i < nchan; i++) {
	  channels |= (0x1 << i);
        }
        range = 0;
        for (i = 0; i < 8; i++) {
          ranges[i] = range;
	}
        usbAInScanStop_USB1608FS_Plus(udev);
	usbAInScanClearFIFO_USB1608FS_Plus(udev);
        usbAInScanConfig_USB1608FS_Plus(udev, ranges);
	sleep(1);
        i = 0;
	usbAInScanStart_USB1608FS_Plus(udev, count, frequency, channels, 0);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
  	  ret = usbAInScanRead_USB1608FS_Plus(udev, 256, nchan, sdataIn, CONTINUOUS);
          for (scan = 0; scan < 256; scan++) { //for each scan
	    for (channel = 0; channel < nchan; channel++) {  // for each channel in a scan
              dataC[scan][channel] = rint(sdataIn[scan*8+channel]*table_AIN[range][channel][0] + table_AIN[range][channel][1]);
	    }
	  }
          if (i%100 == 0) {
            printf("Scan = %d\n", i);
	  }
          i++;
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
        usbAInScanStop_USB1608FS_Plus(udev);
	usbReset_USB1608FS_Plus(udev);
        cleanup_USB1608FS_Plus(udev);
        sleep(2); // let things settle down.
        goto start;
        break;
      case 'x':
        printf("Testing USB-1608FS_Plus Mult-Channel Analog Input Scan.\n");
        usbAInScanStop_USB1608FS_Plus(udev);
        printf("enter number of channels (1-8) :");
        scanf("%d", &nchan);
        printf("Enter number of scans (less than 512): ");
        scanf("%d", &count);
        printf("Enter number of repeats: ");
        scanf("%d", &repeats);
        printf("Enter sampling frequency: ");
        scanf("%lf", &frequency);
        // Build bitmap for the first nchan in channels.
        channels = 0;
        for (i = 0; i < nchan; i++) {
	  channels |= (1 << i);
	}
        printf ("channels: %02X   count:%d\n", channels, count);
        // Always use BP_10V to make it easy (BP_10V is 0...)
        memset(ranges, 0, sizeof(ranges));
	range = 0;
        usbAInScanConfig_USB1608FS_Plus(udev, ranges);
        // Run a loop for the specified number of repeats and
        // show the results...
        for (m = 0; m < repeats; m++) {
	  printf("\n\n---------------------------------------");
	  printf("\nrepeat: %d\n", m);
	  usbAInScanStart_USB1608FS_Plus(udev, count, frequency, channels, 0);
	  ret = usbAInScanRead_USB1608FS_Plus(udev, count, nchan, sdataIn, 0);
	  printf("Number samples read = %d\n", ret/2);
	  if (ret != count * nchan * 2) {
	    printf("***ERROR***  ret = %d   count = %d  nchan = %d\n", ret, count, nchan);
	    continue;
	  } /* if (ret != count * nchan * 2) */
	  for (i = 0; i < count; i++) {
	    printf("%6d", i);
	    for (j = 0; j < nchan; j++)	{
	      k = i*nchan + j;
	      data = rint(sdataIn[k]*table_AIN[range][j][0] + table_AIN[range][j][1]);
	      printf(", %8.4f", volts_USB1608FS_Plus(data, range));
	    } /* for (j - 0; j < 8, j++) */
	    printf("\n");
	  } /* for (i = 0; i < count; i++) */
	} /* for (m = 0; m < repeats; m++) */
	printf("\n\n---------------------------------------");
	break;
      case 'r':
        usbReset_USB1608FS_Plus(udev);
        break;
      case 's':
        usbGetSerialNumber_USB1608FS_Plus(udev, serial);
        printf("Serial number = %s\n", serial);
	if (udev2 != NULL) {
	  usbGetSerialNumber_USB1608FS_Plus(udev2, serial);
	  printf("Device 2: Serial number = %s\n", serial);
	}
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB1608FS_Plus(udev));
	if (udev2 != NULL) {
	  printf("Device 2 Status = %#x\n", usbStatus_USB1608FS_Plus(udev2));
	}
	break;
      case 'e':
        cleanup_USB1608FS_Plus(udev);
        if (udev2 != NULL) cleanup_USB1608FS_Plus(udev2);
        return 0;
      default:
        break;
    }
  }
}

