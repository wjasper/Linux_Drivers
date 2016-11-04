/*
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

#include "pmd.h"
#include "usb-dio24.h"

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
  uint8_t input, pin = 0;
  int temp;
  int ch;
  int ret;
  hid_device*  hid = 0x0;
  wchar_t serial[64];
  wchar_t wstr[MAX_STR];

  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USBDIO24_PID, NULL)) > 0) {
      printf("USB-DIO24 Device is found!\n");
    } else if ((hid = hid_open(MCC_VID, USBDIO24H_PID, NULL)) > 0) {
    printf("USB-DIO24H Device is found!\n");
  } else {
    fprintf(stderr, "USB-DIO24 and USB-DIO24H not found.\n");
    exit(1);
  }

  usbDConfigPort_USBDIO24(hid, DIO_PORTA, DIO_DIR_OUT);
  usbDConfigPort_USBDIO24(hid, DIO_PORTB, DIO_DIR_OUT);
  usbDConfigPort_USBDIO24(hid, DIO_PORTC_LOW, DIO_DIR_OUT);
  usbDConfigPort_USBDIO24(hid, DIO_PORTC_HI, DIO_DIR_IN);
  usbDOut_USBDIO24(hid, DIO_PORTA, 0x0);  // set all output to zero
  usbDOut_USBDIO24(hid, DIO_PORTB, 0x0);  // set all output to zero
  
  while(1) {
    printf("\nUSB DIO24 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 's' to set user id\n");
    printf("Hit 'g' to get user id\n");
    printf("Hit 'n' to get serial number\n");
    printf("Hit 'c' to test counter \n");
    printf("Hit 'd' to test digital I/O \n");
    printf("Hit 'i' for information\n");
    printf("Hit 't' to test digital bit I/O\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    
    switch(tolower(ch)) {
    case 'b': /* test to see if led blinks */
      usbBlink_USBDIO24(hid);
      break;
    case 's':
      printf("enter a user id: ");
      scanf("%d",&temp);
      usbSetID_USBDIO24(hid, temp);
      printf("User ID is set to %d\n", usbGetID_USBDIO24(hid));      
      break;
    case 'g':
      printf("User ID = %d\n", usbGetID_USBDIO24(hid));      
      break;
    case 'i':
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
    case 'c':
      usbDConfigPort_USBDIO24(hid, DIO_PORTA, DIO_DIR_OUT);
      printf("connect pin 1 and 37\n");
      usbInitCounter_USBDIO24(hid);
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
        usbDOut_USBDIO24(hid, DIO_PORTA, 1);
        usbDOut_USBDIO24(hid, DIO_PORTA, 0);
	printf("Counter = %d\n",usbReadCounter_USBDIO24(hid));
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    case 'd':
      usbDConfigPort_USBDIO24(hid, DIO_PORTA, DIO_DIR_OUT);
      usbDConfigPort_USBDIO24(hid, DIO_PORTB, DIO_DIR_IN);
      usbDConfigPort_USBDIO24(hid, DIO_PORTC_LOW, DIO_DIR_OUT);
      usbDConfigPort_USBDIO24(hid, DIO_PORTC_HI, DIO_DIR_IN);
   
      printf("\nTesting Digital I/O....\n");
      printf("connect pins 30 through 37 <=> 3 through 10 and pins 22-25 <==> 26-28\n");
      do {
        printf("Enter a byte number [0-0xff] : " );
        scanf("%x", &temp);
        usbDOut_USBDIO24(hid, DIO_PORTA, (uint8_t)temp);
        usbDIn_USBDIO24(hid, DIO_PORTB, &input);
        printf("The number you entered = %#hhx\n\n", input);
	printf("Enter a nibble [0-0xf] : " );
	scanf("%x", &temp);
	usbDOut_USBDIO24(hid, DIO_PORTC_LOW, (uint8_t)temp);
	usbDIn_USBDIO24(hid, DIO_PORTC_HI, &input);
	printf("The number you entered = %#x\n", input);
      } while (toContinue());
      break;
    case 't':
      //reset the pin values
      usbDOut_USBDIO24(hid, DIO_PORTA, 0x0);
      printf("\nTesting Bit  I/O....\n");
      printf("Enter a bit value for output (0 | 1) : ");
      scanf("%d", &temp);
      input = (uint8_t) temp;
      printf("Select the Pin in port A [0-7] :");
      scanf("%d", &temp);
      pin = (uint8_t) temp;
      usbDBitOut_USBDIO24(hid, DIO_PORTA, pin, input);
      usbDIn_USBDIO24(hid, DIO_PORTB, &input);
      printf("The number you entered 2^%d = %d \n", temp, input);
      break;
    case 'e':
      hid_close(hid);
      hid_exit();
      return 0;
      break;
    case 'n':
      hid_get_serial_number_string(hid, serial, 64);
      printf("Serial Number = %ls\n", serial);
      break;
    default:
      break;
    }
  }
}
