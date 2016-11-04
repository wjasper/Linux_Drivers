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
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdint.h>
#include <math.h>

#include "pmd.h"
#include "usb-7204.h"

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
  int flag;
  unsigned char serial[9];
  uint8_t channel, gain;
  int temp, i,j;
  int ch;
  LoadQueue  gainArray;
  uint16_t in_data[1024];
  int count;
  uint8_t options;
  float freq;
  float voltage;
  uint16_t wvalue;
  uint16_t out_data[512];

  libusb_device_handle *udev = NULL;
  int ret;
  Calibration_AIN table_AIN[NMODE][NGAINS_USB7204][NCHAN_USB7204];
  struct tm date;
  
start:  
  udev = 0;
  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("libusb_init: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB7204_PID, NULL))) {
    printf("USB-7204 Device is found!\n");
  } else {
    printf("No device found.\n");
    exit(0);
  }

  // some initialization
  printf("Building calibration table.  This may take a while ...\n");
  usbBuildGainTable_USB7204(udev, table_AIN);
  for (i = 0; i < NGAINS_USB7204; i++ ) {
    for (j = 0; j < NCHAN_USB7204/2; j++) {
      printf("Calibration Table: Range = %d Channel = %d Slope = %f   Offset = %f\n", 
	     i, j, table_AIN[DF][i][j].slope, table_AIN[DF][i][j].intercept);
    }
  }
  i = BP_10_00V;
  for (j = 0; j < NCHAN_USB7204; j++) {
    printf("Calibration Table: Range = %d Channel = %d Slope = %f   Offset = %f\n", 
	   i, j, table_AIN[SE][i][j].slope, table_AIN[SE][i][j].intercept);
  }

  //print out the wMaxPacketSize.  Should be 64.
  printf("\nwMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  // Print the calibration date
  getMFGCAL_USB7204(udev, &date);
  printf("\nLast Calibration date: %s", asctime(&date));

  while(1) {
    printf("\nUSB-7204 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to read digial word\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'i' to test analog input\n");
    printf("Hit 'I' to test analog input scan\n");    
    printf("Hit 'o' to test analog output\n");
    printf("Hit 'O' to test Analog Scan Output\n");
    printf("Hit 'r' to reset\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'w' to write digial word\n");

    while((ch = getchar()) == '\0' ||
	  ch == '\n');

    switch(ch) {
    case 'b': /* test to see if led blinks */
      printf("Enter number or times to blink: ");
      scanf("%hhd", &options);
      usbBlinkLED_USB7204(udev, options);
      break;
    case 'c':
      printf("connect pin 39 and 20\n");
      usbDConfigPort_USB7204(udev, PORT1, DIO_DIR_OUT);
      usbInitCounter_USB7204(udev);
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
	usbDPortW_USB7204(udev, PORT0, 0xff);
	usbDPortW_USB7204(udev, PORT1, 0xff);
	usleep(5000);
	usbDPortW_USB7204(udev, PORT0, 0x0);
	usbDPortW_USB7204(udev, PORT1, 0x0);
	usleep(100000);
        printf("Counter = %d\n",usbReadCounter_USB7204(udev));
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    case 'w':
      usbDConfigPort_USB7204(udev, PORT0, DIO_DIR_OUT);
      usbDConfigPort_USB7204(udev, PORT1, DIO_DIR_IN);
      printf("Enter value to write to DIO port1: ");
      scanf("%hx", &wvalue);
      usbDPortW_USB7204(udev, PORT0, (uint8_t) wvalue);
      break;
    case 'd':
      usbDConfigPort_USB7204(udev, PORT1, DIO_DIR_IN);
      wvalue = usbDPortR_USB7204(udev, PORT1);
      printf("Port 1 = %#hx\n", wvalue);
      break;
    case 'I':
      printf("Testing USB-7204 Analog Input Scan.\n");
      printf("Input channel 0-3: ");
      scanf("%hhd", &channel);
      printf("Enter desired frequency [Hz]: ");
      scanf("%f", &freq);
      printf("Enter number of samples [1-1024]: ");
      scanf("%d", &count);
      printf("\t\t1. +/- 20.V\n");
      printf("\t\t2. +/- 10.V\n");
      printf("\t\t2. +/- 5.0V\n");
      printf("\t\t3. +/- 4.0V\n");
      printf("\t\t4. +/- 2.5V\n");
      printf("\t\t5. +/- 2.0V\n");
      printf("\t\t6. +/- 1.25V\n");
      printf("\t\t7. +/- 1.00V\n");
      printf("Select gain: [1-8]\n");
      scanf("%d", &temp);
      switch(temp) {
        case 1: gain = BP_20_00V;
  	  break;
        case 2: gain = BP_10_00V;
	  break;
        case 3: gain = BP_5_00V;
	  break;
        case 4: gain = BP_4_00V;
	  break;
        case 5: gain = BP_2_50V;
  	  break;
        case 6: gain = BP_2_00V;
	  break;
        case 7: gain = BP_1_25V;
	  break;
        case 8: gain = BP_1_00V;
	  break;
        default:
	  break;
      }
      usbAInStop_USB7204(udev);
      // Load the gain queue
      gainArray.num = 1;
      gainArray.chan_0 = channel;
      gainArray.gain_0 = gain;
      usbAInLoadQueue_USB7204(udev, &gainArray);

      options = AIN_EXECUTION;

      for ( i = 0; i < 1024; i++ ) {  // load data with known value
	in_data[i] = 0xbeef;
      }
      usbAInScan_USB7204(udev, channel, channel, count, &freq, options);
      printf("Actual frequency = %f\n", freq);
      ret = usbAInScanRead_USB7204(udev, count, 1, in_data);
      for ( i = 0; i < count; i++ ) {
        in_data[i] >>= 4;
        in_data[i] = rint(in_data[i]*table_AIN[DF][gain][channel].slope + table_AIN[DF][gain][channel].intercept);
	printf("data[%d] = %#hx  %.4fV\n", i, in_data[i], volts_USB7204(in_data[i], gain));
      }
	break;
    case 'i':
      printf("Connect pin 1 - pin 13\n");
      printf("Select channel [0-3]: ");
      scanf("%d", &temp);
      if ( temp < 0 || temp > 7 ) break;
      channel = (uint8_t) temp;
      printf("\t\t1. +/- 10.V\n");
      printf("\t\t2. +/- 5.V\n");
      printf("\t\t3. +/- 4.V\n");
      printf("\t\t4. +/- 2.5V\n");
      printf("\t\t5. +/- 2.0V\n");
      printf("\t\t6. +/- 1.25V\n");
      printf("\t\t7. +/- 1.0V\n");
      printf("Select gain: [1-7]\n");
      scanf("%d", &temp);
      switch(temp) {
        case 1: 
          gain = BP_10_00V;
          voltage = 5.0;
          break;
        case 2: 
          gain = BP_5_00V;
          voltage = 4.9;
	  break;
        case 3: 
          gain = BP_4_00V;
          voltage = 3.9;
          break;
        case 4: 
          gain = BP_2_50V;
          voltage = 2.4;
          break;
        case 5: 
	  gain = BP_2_00V;
          voltage = 1.9;
          break;
        case 6: 
	  gain = BP_1_25V;
          voltage = 1.20;
          break;
        case 7: 
	  gain = BP_1_00V;
          voltage = .9;
          break;
        default:
          break;
	}
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
          usbAOut_USB7204(udev, 0, 0.);
	  sleep(1);
	  wvalue = usbAIn_USB7204(udev, DF, channel, gain);
          wvalue = rint(wvalue*table_AIN[DF][gain][channel].slope + table_AIN[DF][gain][channel].intercept);
	  printf("Channel: %d: value = %#hx, %.4fV\n",
		 channel, wvalue, volts_USB7204(wvalue, gain));
          usbAOut_USB7204(udev, 0, voltage);
	  sleep(1);
	  wvalue = usbAIn_USB7204(udev, DF, channel, gain);
          wvalue = rint(wvalue*table_AIN[DF][gain][channel].slope + table_AIN[DF][gain][channel].intercept);
	  printf("Channel: %d: value = %#hx, %.4fV\n",
		 channel, wvalue, volts_USB7204(wvalue, gain));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'o': /* test the analog output */
        printf("Testing the analog output...\n");
        printf("Enter channel [0-1] => (pin 13-14):");
        scanf("%d", &temp);
        channel = (uint8_t) temp;
        printf("Enter a voltage 0-4V: ");
        scanf("%f", &voltage);
        usbAOut_USB7204(udev, channel, voltage);
        break;
      case 'O': /* test Analog Output Scan */
        printf("Enter desired frequency [Hz]: ");
        scanf ("%f", &freq);
        usbAOutScanReset_USB7204(udev);
        freq *= 2.0;  // double frequency since 2 cycles of square wave

        for ( j = 0; j <  200; j++ ) {
	  for (i = 0; i < 512; i++) {
	    out_data[i] = i%2 ? 0 : 0xfff;
	  }
          usbAOutScanStop_USB7204(udev);
	  usbAOutScan_USB7204(udev, 0, 0, 512, &freq, out_data, 0);
	}
        usbAOutScanStop_USB7204(udev);
	usbAOut_USB7204(udev, 0, 0.0);  // Reset the voltage to 0V.
	break;
      case 's':
	 getUsbSerialNumber(udev, serial);
         printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB7204(udev));
	break;
      case 'r':
        usbReset_USB7204(udev, 1);
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
