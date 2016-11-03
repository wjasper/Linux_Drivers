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
#include <stdint.h>

#include "pmd.h"
#include "usb-4303.h"

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

int main (int argc, char **argv) {
  int i;
  int ch;
  wchar_t serial[64];
  wchar_t wstr[MAX_STR];

  uint8_t pin = 0;
  uint8_t bit_value = 0x0;
  uint8_t tmp;

  hid_device *hid = 0x0;
  int ret;

  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USB4301_PID, NULL)) > 0) {
      printf("USB 4301 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB4303_PID, NULL)) > 0) {
    printf("USB 4303 Device is found!\n");
  } else {
    fprintf(stderr, "USB 4301 or 4303 not found.\n");
    exit(-1);
  }

  while(1) {
    printf("\nUSB 430X Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'c' for event counter.\n");
    printf("Hit 'd' to test digital I/O \n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'g' to get serial number\n");
    printf("Hit 'i' for information\n");
    printf("Hit 'r' to reset\n");
    printf("Hit 's' to get status\n");
    printf("Hit 't' to test digital bit I/O\n");

    
    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
    case 'b': /* test to see if led blinks 5 times*/
      usbBlink_USB4303(hid, 5);
      break;
    case 'c':
      printf("Configures Chip 1 Counter 1 as a simple event counter.\n");
      printf("Uses 1INP1 as the source input.  Connect DO0 to 1INP1\n");
      usbSetRegister_USB4303(hid, 1, CNT_1_MODE_REG,
			     (NEGATIVEEDGE|COUNTUP|ONETIME|BINARY|LOADREG|SRC1|SPECIALGATEOFF|NOGATE));
      usbSetRegister_USB4303(hid, 1, CNT_1_LOAD_REG, 0);
      usbLoad_USB4303(hid, 1, COUNTER_1);
      usbArm_USB4303(hid, 1, COUNTER_1);
      for ( i = 0; i < 13; i++ ) {
	usleep(10000);
	usbDOut_USB4303(hid, 0);
	usleep(10000);
	usbDOut_USB4303(hid, 1);
      }
      printf("Value should be 13.  Read value = %d\n", usbRead_USB4303(hid, 1, COUNTER_1));
      break;
    case 'd':
      printf("\nTesting Digital I/O Write \n");
      printf("Enter a byte number [0-0xff] : " );
      scanf("%hhx", &tmp);
      usbDOut_USB4303(hid, (uint8_t)tmp);
      printf("\nTesting Digital I/O Read \n");
      printf("%#hhx\n", usbDIn_USB4303(hid));
      break;
    case 't':
      printf("\nTesting Digital Bit I/O Write \n");
      printf("Select the bit [0-7] : ");
      scanf("%hhd", &pin);
      printf("Enter a bit value for output (0 | 1) : ");
      scanf("%hhd", &bit_value);
      usbDBitOut_USB4303(hid, pin, bit_value);
      printf("\nTesting Digital Bit I/O Read \n");
      printf("Bit No: %d   Value = %hhx\n", pin, usbDBitIn_USB4303(hid, pin));
      break;
    case 'g':
      hid_get_serial_number_string(hid, serial, 64);
      printf("Serial Number = %ls\n", serial);
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
    case 'r':
      usbReset_USB4303(hid);
      return 0;
      break;
    case 'e':
      hid_close(hid);
      hid_exit();
      return 0;
      break;
    case 's':
      printf("Status = %#x\n", usbGetStatus_USB4303(hid));
      break;
    default:
      break;
    }
  }
}
