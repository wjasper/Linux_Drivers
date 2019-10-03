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
#include "usb-pdiso8.h"

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
  int temp;
  int ch;
  
  hid_device*  hid = 0x0;
  int ret;
  wchar_t wstr[MAX_STR];

  uint8_t port;
  uint8_t pin = 0;
  uint8_t bit_value;
  
  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d.\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USBPDISO8_PID, NULL)) > 0) {
    printf("USB PDISO8 Device is found! \n");
  } else if ((hid = hid_open(MCC_VID, USBSWITCH_AND_SENSE_PID, NULL)) > 0) {
    printf("USB Switch & Sense 8/8  Device is found!\n");
  } else {
    fprintf(stderr, "USB PDISO8 and Switch & Sense 8/8 not found.\n");
    exit(-1);
  }

  while(1) {
    printf("\nUSB PDISO8 or Switch & Sense Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'd' to test digital I/O \n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'g' to get serial number\n");
    printf("Hit 'j' for information\n");
    printf("Hit 't' to test digital bit I/O\n");
    
    while((ch = getchar()) == '\0' || ch == '\n');
    
    switch(ch) {
    case 'b': /* test to see if led blinks */
      usbBlink_USBPDISO8(hid);
      break;
    case 'd':
      printf("\nTesting Digital I/O....\n");
      do {
	printf("Enter a port number: 0 - Relay Port, 1 - ISO Port: ");
	scanf("%hhd", &port);
	switch (port) {
        case 0:  // Relay Port output only
          printf("Enter a byte number [0-0xff] : " );
          scanf("%x", &temp);
          usbDOut_USBPDISO8(hid, port, (uint8_t)temp);
          break;
        case 1:  // ISO Port input only
	  printf("ISO Port = %#x\n", usbDIn_USBPDISO8(hid, port));
	  break;
      default:
	printf("Invalid port number.\n");
        break;
	}
      } while (toContinue());
      break;
    case 't':
      do {
	printf("\nTesting Digital Bit I/O....\n");
	printf("Enter a port number: 0 - Relay Port, 1 - ISO Port: ");
	scanf("%hhd", &port);
	printf("Select the Pin in port  %d  [0-7] :", port);
	scanf("%hhd", &pin);
	if (pin > 7) break;
	switch (port) {
        case 0:  // Relay Port output only
	  printf("Enter a bit value for output (0 | 1) : ");
	  scanf("%hhd", &bit_value);
	  usbDBitOut_USBPDISO8(hid, port, pin, bit_value);
          break;
	case 1:
	  printf("ISO Port = %d  Pin = %d, Value = %d\n", port, pin, usbDBitIn_USBPDISO8(hid, port, pin));
          break;
	default:
	  printf("Invalid port number.\n");
	  break;
	}
      } while (toContinue());
      break;    
    case 'g':
      ret = hid_get_serial_number_string(hid, wstr, MAX_STR);
      printf("Serial Number = %ls\n", wstr);
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
    case 'e':
      hid_close(hid);
      hid_exit();
      exit(0);
    default:
      break;
    }
  }
}
  



