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
#include "usb-dio96H.h"

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
  wchar_t serial[64];
  wchar_t wstr[MAX_STR];

  hid_device *hid = 0x0;
  int ret;
  
  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USB1096HFS_PID, NULL)) > 0) {
    printf("USB 1096HFS Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USBDIO96H_PID, NULL)) > 0) {
    printf("USB DIO96H Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USBDIO96H_50_PID, NULL)) > 0) {
    printf("USB DIO96H/50 Device is found!\n");
  } else {
    printf("USB 1096HFS, DIO96H or DIO96H/50  not found.\n");
    exit(0);
  }

  usbDConfigPort_USBDIO96H(hid, DIO_PORT0A, DIO_DIR_OUT);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT0B, DIO_DIR_IN);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT0C_LOW, DIO_DIR_OUT);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT0C_HI, DIO_DIR_IN);

  usbDConfigPort_USBDIO96H(hid, DIO_PORT1A, DIO_DIR_OUT);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT1B, DIO_DIR_IN);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT1C_LOW, DIO_DIR_OUT);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT1C_HI, DIO_DIR_IN);

  usbDConfigPort_USBDIO96H(hid, DIO_PORT2A, DIO_DIR_OUT);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT2B, DIO_DIR_IN);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT2C_LOW, DIO_DIR_OUT);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT2C_HI, DIO_DIR_IN);

  usbDConfigPort_USBDIO96H(hid, DIO_PORT3A, DIO_DIR_OUT);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT3B, DIO_DIR_IN);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT3C_LOW, DIO_DIR_OUT);
  usbDConfigPort_USBDIO96H(hid, DIO_PORT3C_HI, DIO_DIR_IN);

  while(1) {
    printf("\nUSB DIO96H Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'c' to test counter \n");
    printf("Hit 'd' to test digital I/O \n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'i' for information\n");
    printf("Hit 'g' to get serial number\n");
    printf("Hit 's' to get status\n");
    printf("Hit 'r' to reset\n");
    printf("Hit 't' to test digital bit I/O\n");
    
    while((ch = getchar()) == '\0' || ch == '\n');
    
    switch(ch) {
    case 'b': /* test to see if led blinks */
      usbBlink_USBDIO96H(hid);
      break;
    case 'c':
      printf("connect pin P1A0 to CTR and P1B0\n");
      usbInitCounter_USBDIO96H(hid);
      sleep(1);
      usbDOut_USBDIO96H(hid, DIO_PORT0A, 0x0);
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
	// usbDBitOut_USBDIO96H(hid, DIO_PORT0A, 0x1, 0x1);
        usbDOut_USBDIO96H(hid, DIO_PORT0A, 0x1);
	printf("Bit 0 is a %#x    ", usbDBitIn_USBDIO96H(hid, DIO_PORT0B, 0x0));
	sleep(1);
	//usbDBitOut_USBDIO96H(hid, DIO_PORT0A, 0x1, 0x0);
	usbDOut_USBDIO96H(hid, DIO_PORT0A, 0x0);
	printf("Bit 0 is a %#x\n", usbDBitIn_USBDIO96H(hid, DIO_PORT0B, 0x0));
	printf("Counter = %d\n",usbReadCounter_USBDIO96H(hid));
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    case 'd':
      printf("\nTesting Digital I/O....\n");
      printf("connect pins P1A0 through P1A7 <=> P1B0 through P1B7 and pins P1C0-P1C3 <==> P1C4-P1C7\n");
      do {
        printf("Enter a byte number [0-0xff] : " );
        scanf("%x", &temp);
        usbDOut_USBDIO96H(hid, DIO_PORT0A, (uint8_t)temp);
        input = usbDIn_USBDIO96H(hid, DIO_PORT0B);
        printf("The number you entered = %#x\n\n",input);
        printf("Enter a nibble [0-0xf] : " );
        scanf("%x", &temp);
        usbDOut_USBDIO96H(hid, DIO_PORT0C_LOW, (uint8_t)temp);
	input = usbDIn_USBDIO96H(hid, DIO_PORT0C_HI);
        printf("The number you entered = %#x\n",input);
      } while (toContinue());
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
    case 't':
      //reset the pin values
      usbDOut_USBDIO96H(hid,DIO_PORT0A,0x0);
      printf("\nTesting Bit  I/O....\n");
      printf("Enter a bit value for output (0 | 1) : ");
      scanf("%d", &temp);
      input = (uint8_t) temp;
      printf("Select the Pin in port A [0-7] :");
      scanf("%d", &temp);
      pin = (uint8_t) temp;
      usbDBitOut_USBDIO96H(hid, DIO_PORT0A, pin, input);
      printf("The number you entered 2^%d = %d \n",
	     temp,usbDIn_USBDIO96H(hid, DIO_PORT0B));
      break;
    case 's':
      printf("Status = %#x\n", usbGetStatus_USBDIO96H(hid));
      break;
    case 'r':
      usbReset_USBDIO96H(hid);
      return 0;
      break;
    case 'e':
      hid_close(hid);
      hid_exit();
      return 0;
      break;
    default:
      break;
    }
  }
}
  



