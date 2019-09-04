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
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdint.h>
#include <math.h>

#include "pmd.h"
#include "usb-7202.h"

/* Test Program */
int toContinue()
{
  int answer;
  answer = 0; //answer = getchar();
  printf("Continue [yY]? ");
  while((answer = getchar()) == '\0' || answer == '\n');
  return ( answer == 'y' || answer == 'Y');
}

int main (int argc, char **argv) 
{
  int flag;
  int toggle = 1;
  unsigned char serial[9];
  uint8_t channel, gain;
  uint8_t lowchannel, hichannel;
  int temp, i,j;
  int ch;
  int scan;
  uint8_t gainArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint16_t in_data[8*512]; // holds 16 bit unsigned analog input data
  uint16_t dataC[256][8];  // corrected data
  int count;
  uint8_t options;
  double freq;
  uint16_t wvalue;

  libusb_device_handle *udev = NULL;
  int ret;
  Calibration_AIN table_AIN[NGAINS_USB7202][NCHAN_USB7202];
  struct tm date;
  
start:  
  udev = NULL;
  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("libusb_init: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB7202_PID, NULL))) {
    printf("USB-7202 Device is found!\n");
  } else {
    printf("No device found.\n");
    exit(0);
  }

  // some initialization
  printf("Building calibration table.  This may take a while ...\n");
  usbBuildGainTable_USB7202(udev, table_AIN);
  for (i = 0; i < NGAINS_USB7202; i++ ) {
    for (j = 0; j < NCHAN_USB7202; j++) {
      printf("Calibration Table: Range = %d Channel = %d Slope = %f   Offset = %f\n", 
	     i, j, table_AIN[i][j].slope, table_AIN[i][j].intercept);
    }
  }

  //print out the wMaxPacketSize.  Should be 64.
  printf("\nwMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  // Print the calibration date
  getMFGCAL_USB7202(udev, &date);
  printf("\nLast Calibration date: %s", asctime(&date));

  while(1) {
    printf("\nUSB-7202 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'C' to test continuous sampling at 1000 Hz.\n");
    printf("Hit 'd' to read digital word\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'i' to test analog input\n");
    printf("Hit 'I' to test analog input scan\n");    
    printf("Hit 'r' to reset\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'w' to write digital word\n");

    while((ch = getchar()) == '\0' ||
	  ch == '\n');

    switch(ch) {
    case 'b': /* test to see if led blinks */
      printf("Enter number or times to blink: ");
      scanf("%hhd", &options);
      usbBlinkLED_USB7202(udev, options);
      break;
    case 'c':
      printf("connect pin 38 and 21.\n");
      usbDConfigPort_USB7202(udev, DIO_DIR_OUT);
      usbInitCounter_USB7202(udev);
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
	usbDPortW_USB7202(udev, 1);
	usleep(5000);
	usbDPortW_USB7202(udev, 0);
	usleep(100000);
        printf("Counter = %d\n",usbReadCounter_USB7202(udev));
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    case 'w':
      usbDConfigPort_USB7202(udev, DIO_DIR_OUT);
      printf("Enter value to write to DIO port: ");
      scanf("%hx", &wvalue);
      usbDPortW_USB7202(udev, (uint8_t) wvalue);
      break;
    case 'd':
      usbDConfigPort_USB7202(udev, DIO_DIR_IN);
      wvalue = usbDPortR_USB7202(udev);
      printf("Port = %#hx\n", wvalue);
      break;
    case 'I':
      printf("Testing USB-7202 Analog Input Scan.\n");
      printf("Input channel 0-7: ");
      scanf("%hhd", &channel);
      printf("Enter desired frequency [Hz]: ");
      scanf("%lf", &freq);
      printf("Enter number of samples [1-1024]: ");
      scanf("%d", &count);
      printf("\t\t1. +/- 10.V\n");
      printf("\t\t2. +/- 5.V\n");
      printf("\t\t3. +/- 2.5V\n");
      printf("\t\t4. +/- 2.V\n");
      printf("\t\t5. +/- 1.25V\n");
      printf("\t\t6. +/- 1.0V\n");
      printf("\t\t7. +/- 0.625V\n");
      printf("\t\t8. +/- 0.3125V\n");
      printf("Select gain: [1-8]\n");
      scanf("%d", &temp);
      switch(temp) {
        case 1: gain = BP_10_00V;
  	  break;
        case 2: gain = BP_5_00V;
	  break;
        case 3: gain = BP_2_50V;
	  break;
        case 4: gain = BP_2_00V;
	  break;
        case 5: gain = BP_1_25V;
  	  break;
        case 6: gain = BP_1_00V;
	  break;
        case 7: gain = BP_0_625V;
	  break;
        case 8: gain = BP_0_3125V;
	  break;
        default:
	  break;
      }
      usbAInStop_USB7202(udev);
      // Load the gain queue
      gainArray[channel] = gain;
      usbAInLoadQueue_USB7202(udev, gainArray);

      options = AIN_EXECUTION;

      for ( i = 0; i < 1024; i++ ) {  // load data with known value
	in_data[i] = 0xbeef;
      }
      usbAInScan_USB7202(udev, channel, channel, count, &freq, options);
      printf("Actual frequency = %f\n", freq);
      ret = usbAInScanRead_USB7202(udev, count, 1, in_data);
      for ( i = 0; i < count; i++ ) {
        in_data[i] = rint(in_data[i]*table_AIN[gain][channel].slope + table_AIN[gain][channel].intercept);
	printf("data[%d] = %#hx  %.4fV\n", i, in_data[i], volts_USB7202(in_data[i], gain));
      }
	break;
    case 'i':
      printf("Connect pin 1 - pin 23\n");
      printf("Select channel [0-7]: ");
      scanf("%d", &temp);
      if ( temp < 0 || temp > 7 ) break;
      channel = (uint8_t) temp;
      printf("\t\t1. +/- 10.V\n");
      printf("\t\t2. +/- 5.V\n");
      printf("\t\t3. +/- 2.5V\n");
      printf("\t\t4. +/- 2.V\n");
      printf("\t\t5. +/- 1.25V\n");
      printf("\t\t6. +/- 1.0V\n");
      printf("\t\t7. +/- 0.625V\n");
      printf("\t\t8. +/- 0.3125V\n");
      printf("Select gain: [1-8]\n");
      scanf("%d", &temp);
      switch(temp) {
        case 1: gain = BP_10_00V;
	  break;
        case 2: gain = BP_5_00V;
          break;
        case 3: gain = BP_2_50V;
	  break;
        case 4: gain = BP_2_00V;
          break;
        case 5: gain = BP_1_25V;
          break;
        case 6: gain = BP_1_00V;
          break;
        case 7: gain = BP_0_625V;
          break;
        case 8: gain = BP_0_3125V;
          break;
        default:
          break;
	}
        usbDConfigPort_USB7202(udev, DIO_DIR_OUT);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
          usbDPortW_USB7202(udev, 0);
	  sleep(1);
	  wvalue = usbAIn_USB7202(udev, channel, gain);
          wvalue = rint(wvalue*table_AIN[gain][channel].slope + table_AIN[gain][channel].intercept);
	  printf("Channel: %d: value = %#hx, %.4fV\n",
		 channel, wvalue, volts_USB7202(wvalue, gain));
          usbDPortW_USB7202(udev, 0xf);
	  sleep(1);
	  wvalue = usbAIn_USB7202(udev, channel, gain);
          wvalue = rint(wvalue*table_AIN[gain][channel].slope + table_AIN[gain][channel].intercept);
	  printf("Channel: %d: value = %#hx, %.4fV\n",
		 channel, wvalue, volts_USB7202(wvalue, gain));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'C':
	printf("Testing USB-7202 Analog Input Scan in Continuous mode 8 channels\n");
	printf("Connect pin 1 - pin 23.\n");
	printf("Hit any key to exit\n");
	usbAInStop_USB7202(udev);
	usbDConfigPort_USB7202(udev, DIO_DIR_OUT);
	count = 0;        // for continuous mode
        lowchannel = 0;   // sample all 8 channels
	hichannel = 7;
	gain = BP_10_00V;
	for (i = 0; i < 8; i++) {  // modify this if you want different gain values
	  gainArray[i] = gain;
	}
	usbAInLoadQueue_USB7202(udev, gainArray);
	freq = 7500.;    // sample 8 channels @  7.5 kHz  make smaller if you get bulk transfer errors
	options = 0x0;  // set for continuous mode
	for ( i = 0; i < 1024; i++ ) {  // load data with known value
	  in_data[i] = 0xbeef;
	}
	usbAInScan_USB7202(udev, lowchannel, hichannel, count, &freq, options);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	i = 0;
	do {
  	  ret = usbAInScanRead_USB7202(udev, 256, 8, in_data);  // read 256 scans 8 channels in a chunk
          for (scan = 0; scan < 256; scan++) { //for each scan
	    for (channel = 0; channel < 8; channel++){  // for each channel in a scan
              dataC[scan][channel] = rint(in_data[scan*8+channel]*table_AIN[gain][channel].slope + table_AIN[gain][channel].intercept);
	    }
	  }
          if (i%100 == 0) {  // print every 100 chunks or 25,600 scans
	    if (toggle == -1) {
	      usbDPortW_USB7202(udev, 0);  //toggle just to show stuff is working
	    } else {
	      usbDPortW_USB7202(udev, 0xf);
	    }
	    toggle*= -1;
	    printf("Scan = %d\t\t", i);
	    printf("dataC[%d][0] = %f dataC[%d][1] = %f  dataC[%d][2]= %f  dataC[%d][3] = %f\n",
		   i, volts_USB7202(dataC[0][0],gain),
		   i, volts_USB7202(dataC[0][1],gain),
		   i, volts_USB7202(dataC[0][2],gain),
		   i, volts_USB7202(dataC[0][3], gain));
	  }
	  i++;
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
        usbAInStop_USB7202(udev);
	usbReset_USB7202(udev,1);
 	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 0);
        libusb_release_interface(udev, 0);
        libusb_close(udev);
        sleep(2); // let things settle down.
        goto start;
        break;
      case 's':
	 getUsbSerialNumber(udev, serial);
         printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB7202(udev));
	break;
      case 'r':
        usbReset_USB7202(udev, 1);
 	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 0);
        libusb_release_interface(udev, 0);
        libusb_close(udev);
  	goto start;
	break;
    case 'e':
      libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 0);
      libusb_close(udev);
      return 0;
      break;
    default:
      break;
    }
  }
}
