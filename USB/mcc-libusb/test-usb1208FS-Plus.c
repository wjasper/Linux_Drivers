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
#include "usb-1208FS-Plus.h"

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
  
  float table_DE_AIN[NGAINS_USB1208FS_PLUS][NCHAN_DE][2];
  float table_SE_AIN[NCHAN_SE][2];

  int ch;
  int i, j, k, m;
  int flag;
  int device;
  uint8_t input;
  int temp;
  uint8_t options;
  char serial[9];
  uint8_t channel, channels;
  uint8_t range;
  uint8_t ranges[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint16_t value;
  uint32_t count;
  double frequency, voltage;
  int ret;
  uint16_t dataAIn[8*512];  // holds 16 bit unsigned analog input data
  uint16_t dataAOut[128];   // holds 12 bit unsigned analog input data
  uint16_t data;
  int nchan, repeats;
  int transferred, nread;
  int timeout;

  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB1208FS_PLUS_PID, NULL))) {
    printf("Success, found a USB 1208FS-Plus!\n");
    device = USB1208FS_PLUS_PID;
  } else if ((udev = usb_device_find_USB_MCC(USB1408FS_PLUS_PID, NULL))) {
    printf("Success, found a USB 1408FS-Plus!\n");
    device = USB1408FS_PLUS_PID;
  } else {
    printf("Failure, did not find a USB 1208FS-Plus/ USB 1408FS-Plus!\n");
    return 0;
  }

  // some initialization
  //print out the wMaxPacketSize.  Should be 64
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  usbBuildGainTable_DE_USB1208FS_Plus(udev, table_DE_AIN);
  usbBuildGainTable_SE_USB1208FS_Plus(udev, table_SE_AIN);
  for (i = 0; i < NGAINS_USB1208FS_PLUS; i++ ) {
    for (j = 0; j < NCHAN_DE; j++) {
      printf("Calibration Table: Range = %d Channel = %d Slope = %f   Offset = %f\n", 
	     i, j, table_DE_AIN[i][j][0], table_DE_AIN[i][j][1]);
    }
  }
  for (i = 0; i < NCHAN_SE; i++ ) {
    printf("Calibration Single Ended Table: Channel = %d Slope = %f   Offset = %f\n", 
	   i, table_SE_AIN[i][0], table_SE_AIN[i][1]);
  }

  usbCalDate_USB1208FS_Plus(udev, &calDate);
  printf("\n");
  printf("MFG Calibration date = %s\n", asctime(&calDate));
  
  while(1) {
    printf("\nUSB 1208FS-Plus/USB 1408FS-Plus Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'C' to test continuous sampling.\n");
    printf("Hit 'd' to test digital IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'o' to test Analog Output\n");
    printf("Hit 'O' to test Analog Scan Output\n");
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
        usbBlink_USB1208FS_Plus(udev, options);
	break;
      case 'c':
        usbCounterInit_USB1208FS_Plus(udev);
        printf("Connect DIO Port A0 to CTR0\n");
	usbDTristateW_USB1208FS_Plus(udev, PORTA, 0x0);
	usbDLatchW_USB1208FS_Plus(udev, PORTA, 0x0);  // put pin 0 into known state
        toContinue();
        for (i = 0; i < 100; i++) {                   // toggle
	  usbDLatchW_USB1208FS_Plus(udev, PORTA, 0x1);
	  usbDLatchW_USB1208FS_Plus(udev, PORTA, 0x0);
        }
        printf("Count = %d.  Should read 100.\n", usbCounter_USB1208FS_Plus(udev));
        break;      
      case 'd':
        printf("\nTesting Digital I/O...\n");
	printf("connect PORTA <--> PORTB\n");
	usbDTristateW_USB1208FS_Plus(udev, PORTA, 0x0);
	printf("Digital Port Tristate Register = %#x\n", usbDTristateR_USB1208FS_Plus(udev, PORTA));
	do {
          printf("Enter a  number [0-0xff] : " );
          scanf("%x", &temp);
          usbDLatchW_USB1208FS_Plus(udev, PORTA, (uint8_t)temp);
	  temp = usbDLatchR_USB1208FS_Plus(udev, PORTA);
          input = usbDPort_USB1208FS_Plus(udev, PORTB);
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, temp);
        } while (toContinue());
        break;
      case 'i':
	printf("Input channel [0-7]: ");
	scanf("%hhd", &channel);
        printf("Input range [0-7]: ");
	scanf("%hhd", &range);
	for (i = 0; i < 20; i++) {
	  value = usbAIn_USB1208FS_Plus(udev, channel, DIFFERENTIAL, range);
	  value = rint(value*table_DE_AIN[range][channel][0] + table_DE_AIN[range][channel][1]);
          if (device == USB1208FS_PLUS_PID) {
  	    printf("Range %d  Channel %d   Sample[%d] = %#x Volts = %lf\n",
		   range, channel,  i, value, volts_USB1208FS_Plus(value, range));
	  } else {
  	    printf("Range %d  Channel %d   Sample[%d] = %#x Volts = %lf\n",
		   range, channel,  i, value, volts_USB1408FS_Plus(value, range));
	  }
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing USB-1208FS_Plus Analog Input Scan, Differential Mode.\n");
        printf("Enter number of scans (less than 512): ");
        scanf("%d", &count);
	printf("Input channel 0-7: ");
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
	usbAInScanStop_USB1208FS_Plus(udev);
	usbAInScanClearFIFO_USB1208FS_Plus(udev);
        usbAInScanConfig_USB1208FS_Plus(udev, ranges);
	memset(dataAIn, 0xbeef, sizeof(dataAIn));
	sleep(1);
        usbAInScanConfigR_USB1208FS_Plus(udev, ranges);
        for (i = 0; i < 4; i++) {
          printf("Channel %d     range %d\n", i, ranges[i]);
	}
	usbAInScanStart_USB1208FS_Plus(udev, count, 0x0, frequency, (0x1<<channel), options);
	ret = usbAInScanRead_USB1208FS_Plus(udev, count, 1, dataAIn, options, 2000);
	printf("Number samples read = %d\n", ret/2);
	for (i = 0; i < count; i++) {
	  dataAIn[i] = rint(dataAIn[i]*table_DE_AIN[range][channel][0] + table_DE_AIN[range][channel][1]);
          if (device == USB1208FS_PLUS_PID) {
	    printf("Range %d Channel %d  Sample[%d] = %#x Volts = %lf\n", range, channel,
		   i, dataAIn[i], volts_USB1208FS_Plus(dataAIn[i], range));
	  } else {
	    printf("Range %d Channel %d  Sample[%d] = %#x Volts = %lf\n", range, channel,
		   i, dataAIn[i], volts_USB1408FS_Plus(dataAIn[i], range));
	  }
	}
        break;
      case 'C':
      	printf("Testing USB-1208FS/1408FS-Plus Analog Input Scan in continuous mode 8 channels\n");
        printf("Hit any key to exit\n");
	printf("Enter desired sampling frequency: ");
	scanf("%lf", &frequency);
        count = 0;         // for continuous mode
	nchan = 8;         // number of channels
	range = BP_10V;
	nread = 256;
		
	if (frequency < 1000) {
	  options = DIFFERENTIAL_MODE | IMMEDIATE_TRANSFER_MODE;
	  timeout = nread*nchan*1000./frequency;
	} else {
	  options = DIFFERENTIAL_MODE;
	  timeout = 10000*nread*nchan/frequency;
	}
        for (channel = 0; channel < nchan; channel++) {
	  ranges[channel] = range;
	}
        usbAInScanConfig_USB1208FS_Plus(udev, ranges);
	memset(dataAIn, 0xbeef, sizeof(dataAIn));
	sleep(1);
        i = 0;
	usbAInScanStart_USB1208FS_Plus(udev, count, 0x0, frequency, 0xff, options);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
	  ret = usbAInScanRead_USB1208FS_Plus(udev, nread, nchan, dataAIn, options | CONTINUOUS, timeout);
          if (i%10 == 0) {
            printf("Scan = %d\n", i);
	  }
          i++;
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbAInScanStop_USB1208FS_Plus(udev);
	usbAInScanClearFIFO_USB1208FS_Plus(udev);
        sleep(2); // let things settle down.
        break;
      case 'o':
        printf("Test Analog Output\n");
        printf("Enter Channel [0-1] ");
        scanf("%hhd", &channel);
        printf("Enter voltage: ");
	scanf("%lf", &voltage);
        value = voltage * 4096 / 5;
	usbAOut_USB1208FS_Plus(udev, channel, value);
	value = usbAOutR_USB1208FS_Plus(udev, channel);
	printf("Analog Output Voltage = %f V\n", volts_USB1208FS_Plus(value, UP_5V));
        break;
      case 'O':
        printf("Test of Analog Output Scan. \n");
        printf("Hook scope up to VDAC 0\n");
        printf("Enter desired frequency of sine wave [Hz]: ");
        scanf("%lf", &frequency);
	frequency *= 2.;

        for (i = 0; i < 32; i++) {
          dataAOut[4*i] =   0x0;
	  dataAOut[4*i+1] = 0x800;
	  dataAOut[4*i+2] = 0xfff;
	  dataAOut[4*i+3] = 0xcff;
	}
	usbAOutScanStop_USB1208FS_Plus(udev);
        options = 0x3;   // output channel 0 and 1 output scan
	usbAOutScanStart_USB1208FS_Plus(udev, 0, frequency, options);
	printf("Hit \'s <CR>\' to stop ");
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  if (libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT|2, (unsigned char *) dataAOut, sizeof(dataAOut), &transferred, 1000) < 0) {
	    perror("usb_bulk_transfer error in AOutScan.");
	  }
	  //	  usb_bulk_write(udev, USB_ENDPOINT_OUT|2, (char *) dataAOut, 0x0, 400);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbAOutScanStop_USB1208FS_Plus(udev);
	break;
      case 'x':
        printf("Testing USB-1208FS_Plus Multi-Channel Analog Input Scan, Differential Mode\n");
        printf("Enter number of channels (1-8) :");
        scanf("%d", &nchan);
        printf("Enter number of scans (less than 512): ");
        scanf("%d", &count);
        printf("Enter number of repeats: ");
        scanf("%d", &repeats);
        // Build bitmap for the first nchan in channels.
        channels = 0x0;
	options = DIFFERENTIAL_MODE;
        for (i = 0; i < nchan; i++) {
	  channels |= (1 << i);
	}
        frequency = 10000.;
        usbAInScanStop_USB1208FS_Plus(udev);

        // Always use BP_20V to make it easy.
        range = BP_20V;   // Bi-polar +/- 20V
        memset(ranges, range, sizeof(ranges));
        usbAInScanConfig_USB1208FS_Plus(udev, ranges);
	memset(dataAIn, 0xbeef, sizeof(dataAIn));
	printf ("channels: %02X   count:%d   range:%d \n", channels, count, range);

        // Run a loop for the specified number of repeats and
        // show the results...
        for (m = 0; m < repeats; m++) {
	  printf("\n\n---------------------------------------");
	  printf("\nrepeat: %d\n", m);
	  // count is the total number of scans to take not samples.
	  usbAInScanStart_USB1208FS_Plus(udev, count, 0x0, frequency, channels, options);
	  ret = usbAInScanRead_USB1208FS_Plus(udev, count, nchan, dataAIn, 0, 2000);
	  printf("Number samples read = %d\n", ret/2);
	  if (ret != count * nchan * 2) {
	    printf("***ERROR***  ret = %d   count = %d  nchan = %d\n", ret, count, nchan);
	    continue;
	  } /* if (ret != count * nchan * 2) */
	  for (i = 0; i < count; i++) {
	    printf("%6d", i);
	    for (j = 0; j < nchan; j++)	{
	      k = i*nchan + j;
	      data = rint(dataAIn[k]*table_DE_AIN[range][j][0] + table_DE_AIN[range][j][1]);
	      if (device == USB1208FS_PLUS_PID) {
		printf(", %8.4lf", volts_USB1208FS_Plus(data, range));
	      } else {
		printf(", %8.4lf", volts_USB1408FS_Plus(data, range));
	      }
	    } /* for (j - 0; j < 8, j++) */
	    printf("\n");
	  } /* for (i = 0; i < count; i++) */
	} /* for (m = 0; m < repeats; m++) */
	printf("\n\n---------------------------------------");
	break;
      case 'r':
        usbReset_USB1208FS_Plus(udev);
        break;
      case 's':
        usbGetSerialNumber_USB1208FS_Plus(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB1208FS_Plus(udev));
	break;
      case 'e':
        cleanup_USB1208FS_Plus(udev);
        return 0;
      default:
        break;
    }
  }
}
