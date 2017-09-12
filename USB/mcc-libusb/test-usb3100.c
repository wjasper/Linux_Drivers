/*
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
#include "usb-3100.h"

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

int main(int argc, char **argv) {

  hid_device *hid = 0x0;
  float volts;
  int flag;
  uint8_t channel, range;
  int temp, i;
  int ch;
  uint16_t value;
  wchar_t serial[64];
  wchar_t wstr[MAX_STR];

  int ret;
  uint8_t memory[62];

  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USB3101_PID, NULL)) > 0) {
    printf("USB 3101 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB3102_PID, NULL)) > 0) {
    printf("USB 3102 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB3103_PID, NULL)) > 0) {
    printf("USB 3103 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB3104_PID, NULL)) > 0) {
    printf("USB 3104 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB3105_PID, NULL)) > 0) {
    printf("USB 3105 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB3106_PID, NULL)) > 0) {
    printf("USB 3106 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB3110_PID, NULL)) > 0) {
    printf("USB 3110 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB3112_PID, NULL)) > 0) {
    printf("USB 3112 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB3114_PID, NULL)) > 0) {
    printf("USB 3114 Device is found!\n");
  } else {
    fprintf(stderr, "USB 31XX  not found.\n");
    exit(1);	
  }

  /* config mask 0x01 means all inputs */
  usbDConfigPort_USB31XX(hid, DIO_DIR_OUT);
  usbDOut_USB31XX(hid, 0);

  // Configure all analog channels for 0-10V output
  for (i = 0; i < 8; i++) {
    usbAOutConfig_USB31XX(hid, i, UP_10_00V);
  }

    while(1) {
    printf("\nUSB 31XX Testing\n");
    printf("----------------\n");
    printf("Hit 'a' for analog output of different levels.\n");
    printf("Hit 'o' for analog output.\n");
    printf("Hit 'b' to blink \n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to test digital output\n");
    printf("Hit 'i' to test digital input\n");
    printf("Hit 'I' for information\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'g' to get serial number\n");
    printf("Hit 'r' to reset\n");
    printf("Hit 's' to get status\n");
    printf("Hit 'R' to read memory\n");

    while((ch = getchar()) == '\0' ||
      ch == '\n');

    switch(ch) {
      case 'a':
	printf("Testing the analog output...\n");
        printf("Enter channel [0-15]: ");
        scanf("%d", &temp);
        channel = (uint8_t) temp;
	for (value = 0; value < 0xfff0; value += 0xf) {
	  usbAOut_USB31XX(hid, channel, (uint16_t) value, 0);
	}
        usbAOut_USB31XX(hid, channel, 0x0, 0);
        break;
      case 'o':
        printf("Testing the analog output for a single channel.\n");
	printf("Enter channel [0-15]: ");
        scanf("%d", &temp);
        channel = (uint8_t) temp;
       	printf("Enter a range: 0 = 0-10V, 1 = +/- 10V, 2 = 0-20mA ");
	scanf("%d", &temp);
        range = (uint8_t) temp;
        printf("Enter a voltage: ");
        scanf("%f", &volts);
        value = volts_USB31XX(range, volts);
        usbAOutConfig_USB31XX(hid, channel, range);
        usbAOut_USB31XX(hid, channel, value, 0);
        break;
      case 'b': /* test to see if led blinks  4 times*/
        usbBlink_USB31XX(hid, 4);
        break;
      case 'c':
        printf("connect CTR and DIO0\n");
        usbInitCounter_USB31XX(hid);
        sleep(1);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
          usbDOut_USB31XX(hid, 1);
	  usleep(200000);
          usbDOut_USB31XX(hid, 0);
	  printf("Counter = %d\n",usbReadCounter_USB31XX(hid));
        } while (!isalpha(getchar()));
        fcntl(fileno(stdin), F_SETFL, flag);
        break;
      case 'd':
	printf("\nTesting Digital I/O....\n");
        printf("Enter a byte number [0-0xff]: " );
        scanf("%x", &temp);
        usbDConfigPort_USB31XX(hid, DIO_DIR_OUT);
        usbDOut_USB31XX(hid, (uint8_t)temp);
        break;
      case 'i':
      	printf("\nTesting Digital Input....\n");
	usbDConfigPort_USB31XX(hid, DIO_DIR_IN);
	usbDIn_USB31XX(hid, (uint8_t*) &temp);
	temp &= 0xff;
	printf("Digital Input = %#x\n", temp);
	break;
      case 'I':
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
      case 's':
        printf("Status = %#x\n", usbGetStatus_USB31XX(hid));
	break;
      case 'r':
        usbReset_USB31XX(hid);
        return 0;
	break;
      case 'e':
        hid_close(hid);
        hid_exit();
        return 0;
	break;
      case 'g':
        hid_get_serial_number_string(hid, serial, 64);
        printf("Serial Number = %ls\n", serial);
        break;
      case 'R':
        memset(memory, 0x0, 62);
        usbReadMemory_USB31XX(hid, 0x0000, 60, memory);
        printf("reading from EEPROM: \n");
	for (i = 0; i < 62; i+=2) {
	  printf("address = %#x \t value = %#x \t\t", i, memory[i]);
  	  printf("address = %#x \t value = %#x \t\n", i+1, memory[i+2]);
	}
	memset(memory, 0x0, 62);
        usbReadMemory_USB31XX(hid, 0x0100, 62, memory);
        printf("\nreading from FLASH: \n");
	for (i = 0; i < 62; i+=2) {
	  printf("address = %#x \t value = %#x \t\t", i+0x100, memory[i]);
  	  printf("address = %#x \t value = %#x \t\n", i+0x101, memory[i+2]);
	}
        break;
      default:
        break;
    }
  }
}
