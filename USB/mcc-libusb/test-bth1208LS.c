/*
 *
 *  Copyright (c) 2016 Warren J. Jasper <wjasper@tx.ncsu.edu>
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
#include "bth-1208LS.h"

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
  struct tm calDate;

  float table_DE_AIN[NGAINS][NCHAN_DE][2];
  float table_SE_AIN[NCHAN_SE][2];

  int ch;
  int i, j, k, m;
  uint8_t options;
  char serial[9];
  uint8_t channel, channels;
  uint8_t range;
  uint8_t ranges[4] = {0, 0, 0, 0};
  uint16_t value;
  uint32_t count;
  double frequency, voltage;
  int ret;
  uint16_t dataAIn[8*512];  // holds 16 bit unsigned analog input data
  uint16_t data;
  int nchan, repeats;

  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(BTH1208LS_PID, NULL))) {
    printf("Success, found a BTH 1208LS!\n");
  } else {
    printf("Failure, did not find a BTH 1208LS!\n");
    return 0;
  }

  // some initialization
  //print out the wMaxPacketSize.  Should be 64
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  usbBuildGainTable_DE_BTH1208LS(udev, table_DE_AIN);
  usbBuildGainTable_SE_BTH1208LS(udev, table_SE_AIN);
  for (i = 0; i < NGAINS; i++ ) {
    for (j = 0; j < NCHAN_DE; j++) {
      printf("Calibration Table: Range = %d Channel = %d Slope = %f   Offset = %f\n", 
	     i, j, table_DE_AIN[i][j][0], table_DE_AIN[i][j][1]);
    }
  }
  printf("\n");
  for (i = 0; i < NCHAN_SE; i++ ) {
    printf("Calibration Single Ended Table: Channel = %d Slope = %f   Offset = %f\n", 
	   i, table_SE_AIN[i][0], table_SE_AIN[i][1]);
  }

  usbCalDate_BTH1208LS(udev, &calDate);
  printf("\n");
  printf("MFG Calibration date = %s\n", asctime(&calDate));

  while(1) {
    printf("\nBTH 1208LS Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to test digitial IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'o' to test Analog Output\n");
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
        usbBlinkLED_BTH1208LS(udev, options);
	break;
      case 'e':
        cleanup_BTH1208LS(udev);
        return 0;
      case 'c':
        usbCounterReset_BTH1208LS(udev);
        printf("Connect AO 0 to CTR.\n");
        toContinue();
        for (i = 0; i < 100; i++) {                   // toggle
    	  usbAOut_BTH1208LS(udev, 0, 4095);
	  usbAOut_BTH1208LS(udev, 0, 0);
        }
	usbCounter_BTH1208LS(udev, &count);
        printf("Count = %d.  Should read 100.\n", count);
        break;
      case 'i':
	printf("Input channel DE [0-3]: ");
	scanf("%hhd", &channel);
        printf("Input range [0-7]: ");
	scanf("%hhd", &range);
	for (i = 0; i < 20; i++) {
	  usbAIn_BTH1208LS(udev, channel, DIFFERENTIAL, range, &value);
	  value = rint(value*table_DE_AIN[range][channel][0] + table_DE_AIN[range][channel][1]);
  	  printf("Range %d  Channel %d   Sample[%d] = %#x Volts = %lf\n",
		   range, channel,  i, value, volts_BTH1208LS(value, range));
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing BTH-1208lS Analog Input Scan.\n");
	usbAInScanStop_BTH1208LS(udev);
        printf("Enter number of scans (less than 512): ");
        scanf("%d", &count);
	printf("Input channel 0-3: ");
        scanf("%hhd", &channel);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
        printf("Enter Range [0-7]: ");
        scanf("%hhd", &range);
        ranges[channel] = range;
	if (frequency > 100.) {
	  options = DIFFERENTIAL_MODE;
	} else {
	  options = DIFFERENTIAL_MODE | IMMEDIATE_TRANSFER_MODE;
	}
	usbAInScanStop_BTH1208LS(udev);
	usbAInScanClearFIFO_BTH1208LS(udev);
        usbAInScanConfig_BTH1208LS(udev, ranges);
	memset(dataAIn, 0x0, sizeof(dataAIn));
	sleep(1);
	usbAInScanConfigR_BTH1208LS(udev, ranges);
	for (i = 0; i < 4; i++) {
	  printf("Channel %d     range %d\n", i, ranges[i]);
	}
	usbAInScanStart_BTH1208LS(udev, count, 0x0, frequency, (0x1<<channel), options);
	ret = usbAInScanRead_BTH1208LS(udev, count, dataAIn, options);
	printf("Number samples read = %d\n", ret/2);
	for (i = 0; i < count; i++) {
	  dataAIn[i] = rint(dataAIn[i]*table_DE_AIN[range][channel][0] + table_DE_AIN[range][channel][1]);
          printf("Range %d Channel %d  Sample[%d] = %#x Volts = %lf\n", range, channel,
		 i, dataAIn[i], volts_BTH1208LS(dataAIn[i], range));
	}
        break;
      case 'x':
        printf("Testing BTH-1208LS Multi-Channel Analog Input Scan.\n");
        usbAInScanStop_BTH1208LS(udev);
        printf("Enter number of channels (1-4) :");
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
        frequency = 10000.;
	options = DIFFERENTIAL_MODE;
        // Always use BP_20V to make it easy (BP_20V is 0...)
        memset(ranges, BP_20V, sizeof(ranges));
        usbAInScanConfig_BTH1208LS(udev, ranges);
        // Run a loop for the specified number of repeats and
        // show the results...
        for (m = 0; m < repeats; m++) {
	  printf("\n\n---------------------------------------");
	  printf("\nrepeat: %d\n", m);
	  usbAInScanStart_BTH1208LS(udev, count*nchan, 0x0, frequency, channels, options);
	  ret = usbAInScanRead_BTH1208LS(udev, count*nchan,  dataAIn, options);
	  printf("Number samples read = %d\n", ret/2);
	  if (ret != nchan*count*2) { /* if (ret != count*2) */
	    printf("***ERROR***  ret = %d   count = %d  nchan = %d\n", ret, count, nchan);
	    continue;
	  } 
	  for (i = 0; i < count/nchan; i++) {
	    printf("%6d", i);
	    for (j = 0; j < nchan; j++)	{
	      k = i*nchan + j;
	      data = rint(dataAIn[k]*table_DE_AIN[range][j][0] + table_DE_AIN[range][j][1]);
	      printf(", %8.4f", volts_BTH1208LS(data, range));
	    } /* for (j - 0; j < 8, j++) */
	    printf("\n");
	  } /* for (i = 0; i < count; i++) */
	} /* for (m = 0; m < repeats; m++) */
	printf("\n\n---------------------------------------");
	break;
      case 'r':
        usbReset_BTH1208LS(udev);
        break;
      case 'o':
        printf("Test Analog Output\n");
        printf("Enter Channel [0-1] ");
        scanf("%hhd", &channel);
        printf("Enter voltage [0-2.5V]: ");
	scanf("%lf", &voltage);
        value = voltage * 4095 / 2.5;
	usbAOut_BTH1208LS(udev, channel, value);
	value = usbAOutR_BTH1208LS(udev, channel);
	printf("Analog Output Voltage = %f V\n", volts_BTH1208LS(value, UP_2_5V));
        break;
      case 's':
        usbGetSerialNumber_BTH1208LS(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_BTH1208LS(udev));
	break;
      default:
        break;
    }
  }
}
