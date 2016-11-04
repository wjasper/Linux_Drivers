/*
 *
 *  Copyright (c) 2014 Warren J. Jasper <wjasper@tx.ncsu.edu>
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
#include <unistd.h>

#include "pmd.h"
#include "usb-ctr.h"

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
  libusb_device_handle *udev = NULL;

  double frequency;
  int ch;
  int i; 
  int temp;
  int ret;
  int device;
  uint32_t period;

  uint16_t version;

  char serial[9];
  uint8_t input;
  uint8_t options;
  uint8_t timer;


  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB_CTR08_PID, NULL))) {
    printf("Success, found a USB-CTR08!\n");
    device = USB_CTR08_PID;
  } else if ((udev = usb_device_find_USB_MCC(USB_CTR04_PID, NULL))) {
    printf("Success, found a USB-CTR04!\n");
    device = USB_CTR04_PID;
  } else {
    printf("Failure, did not find a USB-CTR04/08 series device!\n");
    return 0;
  }
  // some initialization
  usbInit_CTR(udev);

  //print out the wMaxPacketSize.  Should be 512
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev, 0));

  while(1) {
    if (device == USB_CTR08_PID) {
      printf("\nUSB-CTR08 Testing\n");
    } else {
      printf("\nUSB-CTR04 Testing\n");
    }
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to test digitial IO\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 't' to test the timers\n");
    printf("Hit 'v' to get version numbers\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
        usbBlink_USB_CTR(udev, options);
	break;
      case 'c':
	usbCounterSet_USB_CTR(udev, 0, 0x0);       // set counter 0 to zero
	usbCounterModeW_USB_CTR(udev, 0, 0x0);
	usbCounterOptionsW_USB_CTR(udev, 0, 0x0);  // count on rising edge
        usbCounterGateConfigW_USB_CTR(udev, 0, 0); // disable gate
	usbCounterOutConfigW_USB_CTR(udev, 0, 0);  // Output off
	printf("Connect DIO0 to CTR0\n");
	usbDTristateW_USB_CTR(udev, 0xf0);
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDLatchW_USB_CTR(udev, 0x0);
	  usbDLatchW_USB_CTR(udev, 0x1);
        }
        printf("Count = %lld.  Should read 100.\n", (long long) usbCounter_USB_CTR(udev, 0));
        break;      
      case 'd':
        printf("\nTesting Digital I/O...\n");
	printf("connect pins DIO[0-3] <--> DIO[4-7]\n");
	usbDTristateW_USB_CTR(udev,0xf0);
	printf("Digital Port Tristate Register = %#x\n", usbDTristateR_USB_CTR(udev));
	do {
          printf("Enter a byte number [0-0xf] : " );
          scanf("%x", &temp);
	  temp &= 0xf;
          usbDLatchW_USB_CTR(udev, (uint16_t)temp);
	  temp = usbDLatchR_USB_CTR(udev);
          input = usbDPort_USB_CTR(udev) >> 4;
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, temp);
	  for (i = 0; i < 4; i++) {
	    printf("Bit %d = %d\n", i, (temp>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'e':
        cleanup_USB_CTR(udev);
        return 0;
      case 'r':
	usbReset_USB_CTR(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB_CTR(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB_CTR(udev));
	break;
      case 't':
        printf("Enter frequency of timer: ");
        scanf("%lf", &frequency);
	printf("Enter timer [0-3]: ");
	scanf("%hhd", &timer);
	period = 96.E6/frequency - 1;
	usbTimerPeriodW_USB_CTR(udev, timer, period);
	usbTimerPulseWidthW_USB_CTR(udev, timer, period / 2);
	usbTimerCountW_USB_CTR(udev, timer, 0);
	usbTimerDelayW_USB_CTR(udev, timer, 0);
	usbTimerControlW_USB_CTR(udev, timer, 0x1);
	toContinue();
	usbTimerControlW_USB_CTR(udev, timer, 0x0);
        break;
      case 'v':
	version = 0xbeef;
        usbFPGAVersion_USB_CTR(udev, &version);
	printf("FPGA version %02x.%02x\n", version >> 0x8, version & 0xff);
	break;
    default:
        break;
    }
  }
}
