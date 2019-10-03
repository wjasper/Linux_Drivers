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

#include "pmd.h"
#include "usb-1616FS.h"

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

int main (int argc, char **argv) {

  int flag;
  unsigned char serial[9];
  signed short svalue;
  uint8_t channel, gain;
  int temp, i, j;
  int ch;
  uint8_t gainArray[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0};
  signed short in_data[1024];
  int count;
  int options;
  float freq;
  libusb_device_handle *udev = NULL;
  int ret;
  float table_AIN[NGAINS_USB1616FS][NCHAN_USB1616FS][2];

  udev = 0;  
  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("libusb_init: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB1616FS_PID, NULL))) {
    printf("USB-1616FS Device is found!\n");
  } else {
    printf("No device found.\n");
    exit(0);
  }

  /* config mask 0x01 means all inputs */
  usbDConfigPort_USB1616FS(udev, DIO_DIR_OUT);
  usbDOut_USB1616FS(udev, 0);
  printf("Building calibration table.  This will take a while ...\n");
  usbBuildCalTable_USB1616FS(udev, table_AIN);
  // print the table
  for (i = 0; i < NGAINS_USB1616FS; i++) {
    for (j = 0; j < NCHAN_USB1616FS; j++) {
      printf("calibration table_AIN[%d][%d].slope = %f    table_AIN[%d][%d].offset = %f\n",
  	     i, j, table_AIN[i][j][0], i, j, table_AIN[i][j][1]);
    }	
  }

  while(1) {
    printf("\nUSB 1616FS Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'g' to get serial number\n");
    printf("Hit 'I' to test analog input scan\n");    
    printf("Hit 'i' to test analog input\n");
    printf("Hit 'r' to reset\n");
    printf("Hit 's' to get status\n");
    printf("Hit 't' to test digital bit I/O\n");

    while((ch = getchar()) == '\0' ||
      ch == '\n');

    switch(ch) {
      case 'b': /* test to see if led blinks */
        usbBlink_USB1616FS(udev);
        break;
      case 't':
	printf("\nTesting Digital Bit I/O....\n");
        printf("connect DIO 0-3 ,<=> DIO 4-7\n");
	for (i = 0; i < 4; i++) {
  	  usbDConfigBit_USB1616FS(udev, i, DIO_DIR_OUT);
	  usbDOutBit_USB1616FS(udev, i, 0x0);
	  usbDConfigBit_USB1616FS(udev, i+4, DIO_DIR_IN);
	  temp = usbDInBit_USB1616FS(udev, i+4);
	}
	temp = 0x0;
	do {
	  printf("Enter a nibble [0-f]: ");
	  scanf("%x", &temp);
	  //write out the nibble in bits 0-3
	  for (i = 0; i < 4; i++) {
	    usbDOutBit_USB1616FS(udev, i, (temp & 0x1));
	    temp >>= 0x1;
	  }
	  temp = 0x0;
	  // read in the nibble on bits 4-7
	  for (i = 0; i < 4; i++) {
	    temp |= (usbDInBit_USB1616FS(udev, i+4) << i);
	  }
	  printf("your value = %x\n", temp);
	} while (toContinue());
	break;
      case 'c':
        printf("connect DIO 0 to CTR\n");
        usbInitCounter_USB1616FS(udev);
        usbDConfigBit_USB1616FS(udev, 0, DIO_DIR_OUT);
        sleep(1);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
          usbDOutBit_USB1616FS(udev, 0, 1);
	  usleep(300000);
          usbDOutBit_USB1616FS(udev, 0, 0);
	  printf("Counter = %d\n",usbReadCounter_USB1616FS(udev));
        } while (!isalpha(getchar()));
        fcntl(fileno(stdin), F_SETFL, flag);
        break;
      case 'g':
	 getUsbSerialNumber(udev, serial);
         printf("Serial number = %s\n", serial);
	 break;
      case 'I':
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
	// Load the gain queue
        gainArray[0] = gain;
	usbAInLoadQueue_USB1616FS(udev, gainArray);

	// configure options
        //options = AIN_EXECUTION | AIN_DEBUG_MODE;
	options = AIN_EXECUTION ;
	for ( i = 0; i < 1024; i++ ) {  // load data with known value
	  in_data[i] = 0xbeef;
	}
        usbAInScan_USB1616FS(udev, 0, 0, count, &freq, options, in_data, table_AIN, gainArray);
	printf("Actual frequency = %f\n", freq);
	for ( i = 0; i < count; i++ ) {
	  printf("data[%d] = %#hx  %.2fV\n", i, in_data[i], volts_USB1616FS(gain, in_data[i]));
	}
	break;
      case 'i':
        printf("Select channel [0-15]: ");
        scanf("%d", &temp);
        if ( temp < 0 || temp > 15 ) break;
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
        do {
	  svalue = usbAIn_USB1616FS(udev, channel, gain, table_AIN);
	  printf("Channel: %d: value = %#hx, %.2fV\n",
		 channel, svalue, volts_USB1616FS(gain, svalue));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 's':
        printf("Status = %#x\n", usbGetStatus_USB1616FS(udev));
	break;
      case 'r':
        usbReset_USB1616FS(udev);
        return 0;
	break;
      case 'e':
	for (i = 2; i <= 6; i++ ) {
	  libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | i);
	}
	for (i = 0; i <= 6; i++) {
	  libusb_release_interface(udev, i);
	}
	libusb_close(udev);
	return 0;
	break;
      default:
        break;
    }
  }
}
