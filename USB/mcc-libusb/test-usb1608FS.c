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

#include "pmd.h"
#include "usb-1608FS.h"

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
  signed short svalue;
  uint8_t channel, gain;
  int temp, i,j;
  int ch;
  uint8_t gainArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  signed short in_data[1024];
  int count;
  int options;
  float freq;
  uint16_t wvalue;

  libusb_device_handle *udev = NULL;
  int ret;
  Calibration_AIN table_AIN[NGAINS_USB1608FS][NCHAN_USB1608FS];
  
start:  
  udev = 0;
  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("libusb_init: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB1608FS_PID, NULL))) {
    printf("USB-1608FS Device is found!\n");
  } else {
    printf("No device found.\n");
    exit(0);
  }
 
  printf("Building calibration table.  This may take a while ...\n");
  usbBuildCalTable_USB1608FS(udev, table_AIN);
  // print the table
  for (i = 0; i < NGAINS_USB1608FS; i++) {
    for (j = 0; j < NCHAN_USB1608FS; j++) {
      printf("calibration table_AIN[%d][%d].slope = %f    table_AIN[%d][%d].offset = %f\n",
  	     i, j, table_AIN[i][j].slope, i, j, table_AIN[i][j].offset);
    }	
  }

  while(1) {
    printf("\nUSB 1608FS Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to read digial word\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'g' to test analog input scan\n");    
    printf("Hit 'i' to test analog input\n");
    printf("Hit 'r' to reset\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'w' to write digial word\n");

    while((ch = getchar()) == '\0' ||
	  ch == '\n');

    switch(ch) {
    case 'b': /* test to see if led blinks */
      usbBlink_USB1608FS(udev);
      break;
    case 'c':
      printf("connect pin 38 and 21\n");
      usbDConfigPort_USB1608FS(udev, DIO_DIR_OUT);
      usbInitCounter_USB1608FS(udev);
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
	usbDOut_USB1608FS(udev, 1);
	usleep(5000);
	usbDOut_USB1608FS(udev, 0);
	usleep(100000);
        printf("Counter = %d\n",usbReadCounter_USB1608FS(udev));
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    case 'w':
      usbDConfigPort_USB1608FS(udev, DIO_DIR_OUT);
      printf("Enter value to write to DIO port: ");
      scanf("%hx", &wvalue);
      usbDOut_USB1608FS(udev, (uint8_t) wvalue);
      break;
    case 'd':
      usbDConfigPort_USB1608FS(udev, DIO_DIR_IN);
      usbDIn_USB1608FS(udev, (uint8_t*) &wvalue);
      printf("Port = %#hx\n", wvalue);
      break;
    case 'g':
      printf("Enter desired frequency [Hz]: ");
      scanf("%f", &freq);
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
      usbAInStop_USB1608FS(udev);
      // Load the gain queue
      gainArray[0] = gain;
      usbAInLoadQueue_USB1608FS(udev, gainArray);

      // configure options
      //options = AIN_EXECUTION | AIN_DEBUG_MODE;
      // options = AIN_EXECUTION | AIN_TRANSFER_MODE;
      options = AIN_EXECUTION;

      for ( i = 0; i < 1024; i++ ) {  // load data with known value
	in_data[i] = 0xbeef;
      }
      usbAInScan_USB1608FS(udev, 0, 0, count, &freq, options, in_data, table_AIN, gainArray);
      printf("Actual frequency = %f\n", freq);
      for ( i = 0; i < count; i++ ) {
	printf("data[%d] = %#hx  %.4fV\n", i, in_data[i], volts_USB1608FS(gain, in_data[i]));
      }
	break;
    case 'i':
      printf("Connect pin 1 - pin 23\n");
      printf("Select channel [0-7]: ");
      scanf("%d", &temp);
      if ( temp < 0 || temp > 3 ) break;
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
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
        usbDConfigPort_USB1608FS(udev, DIO_DIR_OUT);
        do {
          usbDOut_USB1608FS(udev, 0);
	  sleep(1);
	  svalue = usbAIn_USB1608FS(udev, channel, gain, table_AIN);
	  printf("Channel: %d: value = %#hx, %.4fV\n",
		 channel, svalue, volts_USB1608FS(gain, svalue));
          usbDOut_USB1608FS(udev, 0x2);
	  sleep(1);
	  svalue = usbAIn_USB1608FS(udev, channel, gain, table_AIN);
	  printf("Channel: %d: value = %#hx, %.4fV\n",
		 channel, svalue, volts_USB1608FS(gain, svalue));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 's':
	 getUsbSerialNumber(udev, serial);
         printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbGetStatus_USB1608FS(udev));
	break;
      case 'r':
        usbReset_USB1608FS(udev);
        for (i = 2; i <= 6; i++ ) {
  	  libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | i);
        }
        for (i = 0; i <= 6; i++) {
          libusb_release_interface(udev, i);
	}
        libusb_close(udev);
  	goto start;
	break;
    case 'e':
      for (i = 2; i <= 6; i++ ) {
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | i);
      }
      for (i = 0; i <= 6; i++) {
        libusb_release_interface(udev, i);
      }
      usbReset_USB1608FS(udev);
      libusb_close(udev);
      return 0;
      break;
    default:
      break;
    }
  }
}
