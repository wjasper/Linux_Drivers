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

#include "pmd.h"
#include "usb-1208LS.h"

#define MAX_STR 255

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
  signed short svalue;
  int temp, i, j;
  int ch;
  int rate;
  short *sdata;
  uint16_t value;
  uint16_t count;
  uint8_t gains[8];
  uint8_t options;
  uint8_t input, pin = 0, channel, gain;

  hid_device*  hid = 0x0;
  int ret;
  wchar_t serial[64];
  wchar_t wstr[MAX_STR];

 start:

  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USB1208LS_PID, NULL)) >  0) {
    printf("USB-1208LS Device is found!\n");
  } else {
    fprintf(stderr, "USB-1208LS not found.\n");
    exit(1);
  }

  /* config mask 0x01 means all inputs */
  usbDConfigPort_USB1208LS(hid, DIO_PORTB, DIO_DIR_IN);
  usbDConfigPort_USB1208LS(hid, DIO_PORTA, DIO_DIR_OUT);
  usbDOut_USB1208LS(hid, DIO_PORTA, 0x0);
  
  while(1) {
    printf("\nUSB 1208LS Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED.\n");
    printf("Hit 's' to set user id.\n");
    printf("Hit 'g' to get user id.\n");
    printf("Hit 'f' to get serial number.\n");
    printf("Hit 'j' for information\n");
    printf("Hit 'c' to test counter. \n");
    printf("Hit 'd' to test digital I/O.\n");
    printf("Hit 't' to test digital bit I/O.\n");
    printf("Hit 'o' to test analog output.\n");
    printf("Hit 'i' to test analog input.\n");
    printf("Hit 'n' to test analog input scan.\n");
    printf("Hit 'r' to reset the device.\n");
    printf("Hit 'e' to exit.\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    
    switch(tolower(ch)) {
    case 'b': /* test to see if led blinks */
      usbBlink_USB1208LS(hid);
      break;
    case 's':
      printf("enter a user id :");
      scanf("%d",&temp);
      usbSetID_USB1208LS(hid, temp);
      printf("User ID is set to %d\n", usbGetID_USB1208LS(hid));      
      break;
    case 'f':
      hid_get_serial_number_string(hid, serial, 64);
      printf("Serial Number = %ls\n", serial);
      break;
    case 'g':
      printf("User ID = %d\n", usbGetID_USB1208LS(hid));      
      break;
    case 'j':
      // Read the Manufacuter String
      ret = hid_get_manufacturer_string(hid, wstr, MAX_STR);
      printf("Manufacturer String: %ls\n", wstr);
      // Read the Product String
      ret = hid_get_product_string(hid, wstr, MAX_STR);
      printf("Product String: %ls\n", wstr);
      // Read the Serial Number String
      ret = hid_get_serial_number_string(hid, wstr, MAX_STR);
      printf("Serial Number String: %ls\n", wstr);
      break;            
    case 'o': /* test the analog output */
      printf("Testing the analog output...\n");
      printf("Enter channel [0-1] => (pin 13-14):");
      scanf("%d", &temp);
      channel = (uint8_t) temp;
      printf("Enter value [0-0x3ff]: ");
      scanf("%hx", &value);
      usbAOut_USB1208LS(hid, channel, value);
      break;
    case 'c':
      printf("connect pin 20 and 21\n");
      usbInitCounter_USB1208LS(hid);
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
        usbDOut_USB1208LS(hid, DIO_PORTA, 1);
        usbDOut_USB1208LS(hid, DIO_PORTA, 0);
	printf("Counter = %d\n",usbReadCounter_USB1208LS(hid));
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    case 'd':
      printf("\nTesting Digital I/O....\n");
      printf("connect pins 21 through 28 <=> 32 through 39\n");
      do {
        printf("Enter a byte number [0-0xff]: ");
        scanf("%x", &temp);
        usbDOut_USB1208LS(hid, DIO_PORTA, (uint8_t)temp);
        usbDIn_USB1208LS(hid, DIO_PORTB, &input);
        printf("The number you entered = %#x\n",input);
      } while (toContinue());
      break;
    case 't':
      //reset the pin values
      usbDOut_USB1208LS(hid, DIO_PORTA, 0x0);
      printf("\nTesting Bit  I/O....\n");
      printf("Enter a bit value for output (0 | 1): ");
      scanf("%d", &temp);
      input = (uint8_t) temp;
      printf("Select the Pin in port A [0-7]: ");
      scanf("%d", &temp);
      pin = (uint8_t) temp;
      usbDBitOut_USB1208LS(hid, DIO_PORTA, pin, input);
      usbDIn_USB1208LS(hid, DIO_PORTB, &input);
      printf("The number you entered 2^%d = %d \n",temp, input);
      break;
    case 'n':
      printf("\t\t1. +/- 20.V\n");
      printf("\t\t2. +/- 10.V\n");
      printf("\t\t3. +/- 5.V\n");
      printf("\t\t4. +/- 4.V\n");
      printf("\t\t5. +/- 2.5V\n");
      printf("\t\t6. +/- 2.0V\n");
      printf("\t\t7. +/- 1.25V\n");
      printf("\t\t8. +/- 1.0V\n");
      printf("Select gain: [1-8]\n");
      scanf("%d", &temp);
      switch(temp) {
        case 1: gains[0] = BP_20_00V;
                break;
        case 2: gains[0] = BP_10_00V;
                break;
        case 3: gains[0] = BP_5_00V;
                break;
        case 4: gains[0] = BP_4_00V;
                break;
        case 5: gains[0] = BP_2_50V;
                break;
        case 6: gains[0] = BP_2_00V;
                break;
        case 7: gains[0] = BP_1_25V;
                break;
        case 8: gains[0] = BP_1_00V;
                break;
        default:
                break;
      }
      for (i = 1; i < 8; i++) {
	gains[i] = gains[0];
      }
      //      printf("Select sampling rate [Hz]: ");
      //      scanf("%d", &rate);
      //      printf("Select number of samples: ");
      //      scanf("%hd", &count);
      rate = 100;
      count = 64;
      options = AIN_EXECUTION;
      sdata = (short*) malloc(2048);
      usbAInScan_USB1208LS(hid, count, rate, 0, 3, options, sdata, gains);
      for ( i = 0; i < count/4; i++ ) {
	printf("scan %d: ", i);
	for (j = 0; j < 4; j++) {
	  printf("%.2fV ", volts_LS(gain, sdata[4*i+j]));
	}
	printf("\n");
      }
      free(sdata);
      break;
    case 'i':
      printf("Select channel [0-3]: ");
      scanf("%d", &temp);
      if ( temp < 0 || temp > 3 ) break;
      channel = (uint8_t) temp;
      printf("\t\t1. +/- 20.V\n");
      printf("\t\t2. +/- 10.V\n");
      printf("\t\t3. +/- 5.V\n");
      printf("\t\t4. +/- 4.V\n");
      printf("\t\t5. +/- 2.5V\n");
      printf("\t\t6. +/- 2.0V\n");
      printf("\t\t7. +/- 1.25V\n");
      printf("\t\t8. +/- 1.0V\n");
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
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
	 svalue = usbAIn_USB1208LS(hid, channel, gain);
	 printf("Channel: %d: value = %#hx, %.2fV\n",
		channel, svalue, volts_LS(gain, svalue));
	 sleep(1);
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    case 'r':
      usbReset_USB1208LS(hid);
      sleep(2);
      hid_close(hid);
      goto start;       
    case 'e':
      hid_close(hid);
      hid_exit();
      exit(0);
      break;
    default:
      break;
    }
  }
}
  



