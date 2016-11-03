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
#include <math.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-dio32HS.h"

#define FALSE 0
#define TRUE 1

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
  double frequency;
  libusb_device_handle *udev = NULL;
  char serial[9];
  int ret;
  int ch, i;
  int temp;
  int input;
  int count;
  uint16_t version, pattern;
  uint16_t outData[60000];
  uint8_t options = 0x0;

  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USBDIO32HS_PID, NULL))) {
    printf("Success, found a USB-DIO32HS!\n");
    printf("Initializing the USB-DIO32HS.  This may take a while ...    ");
    fflush(NULL);
    usbInit_DIO32HS(udev);
    printf("Initialized!\n");
  } else {
    printf("Failure, did not find a USB-DIO32HS!\n");
    return 0;
  }

  //print out the wMaxPacketSize.  Should be 512
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  while(1) {
    printf("\nUSB DIO32HS Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'd' to test digitial IO\n");
    printf("Hit 'o' to test DIO Out Scan\n");
    printf("Hit 'p' for pattern triggering\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 'v' to get version numbers\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
	scanf("%d", &temp);
        usbBlink_USBDIO32HS(udev, temp);
        break;
      case 'd':
	printf("\nTesting Digital I/O....\n");
	usbDTristateW_USBDIO32HS(udev, 0x0,    DIO_PORTA); // port A all output
	usbDTristateW_USBDIO32HS(udev, 0xffff, DIO_PORTB); // port B all input
	printf("Digital Port A Tristate Register = %#x\n", usbDTristateR_USBDIO32HS(udev, DIO_PORTA));
	printf("Digital Port B Tristate Register = %#x\n", usbDTristateR_USBDIO32HS(udev, DIO_PORTB));
        do {
	  printf("Enter a 2 byte number [0-0xffff] : " );
	  scanf("%x", &temp);
          usbDLatchW_USBDIO32HS(udev, (uint16_t)temp, DIO_PORTA);
	  temp = usbDLatchR_USBDIO32HS(udev, DIO_PORTA);
          input = usbDPort_USBDIO32HS(udev) >> 16;
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, temp);
	  for (i = 0; i < 16; i++) {
	    printf("Bit %d = %d\n", i, (temp>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'o':
	usbDTristateW_USBDIO32HS(udev, 0x0, DIO_PORTA); // port A all output
	usbOutScanStop_USBDIO32HS(udev);
	usbOutScanClearFIFO_USBDIO32HS(udev);
	printf("Test of DIO Output Scan.  Connect P0D0 to scope\n");
	printf("Enter frequency: ");
	scanf("%lf", &frequency);
	for (i = 0; i < 60000; i += 2) {
	  outData[i] = 0;
	  outData[i+1] = 1;
	}
	count = 1000;
	usbOutScanStart_USBDIO32HS(udev,PORT0,0,0,frequency,options);
	for (i = 0; i < 10; i++) {
	  usbOutScanStart_USBDIO32HS(udev,PORT0,count,0,frequency,options);
	  usbOutScanWrite_USBDIO32HS(udev,count,outData);
	  usleep(count*1.E6/frequency);
	}
	usbOutScanStop_USBDIO32HS(udev);
	usbOutScanClearFIFO_USBDIO32HS(udev);
	break;
      case 'p':
        printf("Test of Pattern Triggering.  Connect Port A to Port B.\n");
	printf("Enter bit pattern to trigger [0-0xffff]: ");
	scanf("%hx", &pattern);
	usbDTristateW_USBDIO32HS(udev, 0x0, DIO_PORTA);         // port A all output
	usbDTristateW_USBDIO32HS(udev, 0xffff, DIO_PORTB);      // port B all input
	usbDLatchW_USBDIO32HS(udev, 0x0, DIO_PORTA);             // write 0 to output port
	options = 0x1;   // Trigger on Port 1 when equal to pattern
	usbPatternDetectConfig(udev, pattern, 0xffff, options);  // Configure Pattern Detection trigger
	printf("Pattern = %#x  ", pattern);
	usbInScanStart_USBDIO32HS(udev, PORT1, 2, 0, 10000, 1, 0x2);
	for (i = 0; i < 0xffff; i++) {
	  usbDLatchW_USBDIO32HS(udev, (uint16_t) i, DIO_PORTA);  // write a trial number
	  usleep(800);
	  if (!(usbStatus_USBDIO32HS(udev) & IN_SCAN_RUNNING)) {
	    printf("Pattern Detected!!\n");
	    break;
	  }
	}
	usbInScanStop_USBDIO32HS(udev);
	usbInScanClearFIFO_USBDIO32HS(udev);
        break;
      case 'v':
	version = 0xbeef;
        usbFPGAVersion_USBDIO32HS(udev, &version);
	printf("FPGA version %02x.%02x\n", version >> 0x8, version & 0xff);
	break;
      case 's':
        usbGetSerialNumber_USBDIO32HS(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USBDIO32HS(udev));
	break;
      case 'e':
        //cleanup_USBDIO32HS(udev);
        return 0;
	break;
    }
  }
}


