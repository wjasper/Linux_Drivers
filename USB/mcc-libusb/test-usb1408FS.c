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
#include <time.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-1408FS.h"

/* Test Program */
int toContinue()
{
  int answer;
  answer = 0; //answer = getchar();
  printf("Continue [yY]? ");
  while((answer = getchar()) == '\0' || answer == '\n');
  return ( answer == 'y' || answer == 'Y');
}

int main (int argc, char **argv)
{
  int flag;
  signed short svalue;
  uint8_t input, channel, gain, options;
  uint8_t gains[8], channels[8];
  uint16_t value;
  uint16_t out_data[512];
  signed short in_data[1024];
  int count;
  int temp, i, j;
  int ch;
  float freq;
  float *fvalue;
  time_t startTime, endTime;
  getAllValues allValue;
  uint16_t address;
  uint8_t memory[64];

  libusb_device_handle *udev = NULL;
  int ret;
  unsigned char serial[9];

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("libusb_init: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB1408FS_PID, NULL))) {
    printf("USB-1408FS Device is found!\n");
  } else {
    printf("No device found.\n");
    exit(0);
  }

  init_USB1408FS(udev);
  
  /* config mask 0x01 means all inputs */
  usbDConfigPort_USB1408FS(udev, DIO_PORTA, DIO_DIR_OUT);
  usbDConfigPort_USB1408FS(udev, DIO_PORTB, DIO_DIR_IN);
  usbDOut_USB1408FS(udev, DIO_PORTA, 0);
  usbDOut_USB1408FS(udev, DIO_PORTA, 0);

  while(1) {
    printf("\nUSB 1408FS Testing\n");
    printf("----------------\n");
    printf("Hit 'a' to test analog output scan.\n");    
    printf("Hit 'b' to blink LED.\n");
    printf("Hit 'c' to test counter.\n");
    printf("Hit 'd' to test digital I/O.\n");
    printf("Hit 'e' to exit.\n");
    printf("Hit 'f' to get all values.\n");
    printf("Hit 'g' to test analog input scan (differential).\n");
    printf("Hit 'h' to test analog input (single ended).\n");
    printf("Hit 'i' to test analog input (differential mode).\n");
    printf("Hit 'j' to test analog input scan (single ended).\n");
    printf("Hit 'o' to test analog output.\n");
    printf("Hit 'm' to read memory\n");
    printf("Hit 'r' to reset.\n");
    printf("Hit 'S' to get status.\n");
    printf("Hit 's' to get serial number\n");

    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
      case 'b': /* test to see if led blinks */
        usbBlink_USB1408FS(udev);
        break;
      case 'c':
	printf("Test the counter: ");
        printf("connect pin 20 and 39\n");
        usbDConfigPort_USB1408FS(udev, DIO_PORTB, DIO_DIR_OUT);
        usbInitCounter_USB1408FS(udev);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
          usbDOut_USB1408FS(udev, DIO_PORTB, 0x80);
	  usleep(300000);
          usbDOut_USB1408FS(udev, DIO_PORTB, 0x0);
	  printf("Counter = %d\n",usbReadCounter_USB1408FS(udev));
        } while (!isalpha(getchar()));
        fcntl(fileno(stdin), F_SETFL, flag);
        break;
      case 'd':
        printf("\nTesting Digital I/O....\n");
        printf("connect pins 21 through 28 <=> 32 through 39\n");
        usbDConfigPort_USB1408FS(udev, DIO_PORTA, DIO_DIR_OUT);
        usbDConfigPort_USB1408FS(udev, DIO_PORTB, DIO_DIR_IN);
        do {
          printf("Enter a byte number [0-0xff]: " );
          scanf("%x", &temp);
          usbDOut_USB1408FS(udev, DIO_PORTA, (uint8_t)temp);
          usbDIn_USB1408FS(udev, DIO_PORTB, &input);
          printf("The number you entered = %#x\n",input);
        } while (toContinue());
        break;
      case 'o': /* test the analog output */
        printf("Testing the analog output...\n");
        printf("Enter channel [0-1] => (pin 13-14):");
        scanf("%d", &temp);
        channel = (uint8_t) temp;
        printf("Enter a value: ");
        scanf("%hx", &value);
        usbAOut_USB1408FS(udev, channel, value);
        break;
      case 'a': /* test Analog Output Scan */
        printf("Enter desired frequency [Hz]: ");
        scanf("%f", &freq);
        for ( j = 0; j <  2; j++ ) {
	  for (i = 0; i < 512; i++) {
	    out_data[i] = i%2 ? 0 : 0xfff;
	  }
	  usbAOutScan_USB1408FS(udev, 0, 0, 512, &freq, out_data, 1);
	}
	usbAOutStop_USB1408FS(udev);
	break;
      case 'g':
        printf("Enter desired frequency [Hz]: ");
        scanf("%f", &freq);
        printf("Enter number of samples [1-1024]: ");
        scanf("%d", &count);
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
	for (i = 0; i < 8; i++) {
	  channels[i] = i;
	  gains[i] = gain;
	}
	usbALoadQueue_USB1408FS(udev, 1, channels, gains);
	options = AIN_EXECUTION | AIN_GAIN_QUEUE;
        for ( i = 0; i < 1024; i++ ) {  // load data with known value
	  in_data[i] = 0xbeef;
	}
        usbAInScan_USB1408FS(udev, 0, 0, count, &freq, options, in_data);
	for ( i = 0; i < count; i++ ) {
	  printf("data[%d] = %#hx  %.2fV\n", i, in_data[i], volts_1408FS(gain, in_data[i]));
	}
	usbAInStop_USB1408FS(udev);
	break;
      case 'h':
        printf("Testing Analog Input Single Ended Mode\n");
        printf("Select channel [0-7]: ");
        scanf("%d", &temp);
	channel = (uint8_t) (temp);
	gain =  SE_10_00V;
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
          sleep(1);
	  svalue = usbAIn_USB1408FS(udev, channel, gain);
	  printf("Channel: %d: value = %#hx, %.2fV\n",
		 channel, svalue, volts_1408FS_SE(svalue));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'i':
        printf("Connect pin 1 - pin 21  and pin 2 - pin 3\n");
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
          usbDOut_USB1408FS(udev, DIO_PORTA, 0);
	  sleep(1);
	  svalue = usbAIn_USB1408FS(udev, channel, gain);
	  printf("Channel: %d: value = %#hx, %.2fV\n",
		 channel, svalue, volts_1408FS(gain, svalue));
          usbDOut_USB1408FS(udev, DIO_PORTA, 0x1);
	  sleep(1);
	  svalue = usbAIn_USB1408FS(udev, channel, gain);
	  printf("Channel: %d: value = %#hx, %.2fV\n",
		 channel, svalue, volts_1408FS(gain, svalue));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
        printf("Doing a timing test.  Please wait ...\n");
        time(&startTime);
        for (count = 0; count < 500; count++) {
	  svalue = usbAIn_USB1408FS(udev, channel, gain);
	}
        time(&endTime);
        printf("Sampling speed is %.0f Hz.\n", 500./(endTime - startTime));
	break;
      case 'j':
        printf("Test of scan mode (single ended).\n");
        printf("Enter desired frequency [Hz]: ");
        scanf("%f", &freq);
        printf("Enter number of samples [1-1024]: ");
        scanf("%d", &count);
	options = AIN_EXECUTION | AIN_GAIN_QUEUE;
	usbAInStop_USB1408FS(udev);
	
        for ( i = 0; i < 1024; i++ ) {  // load data with known value
	  in_data[i] = 0xbeef;
	}
        usbAInScan_USB1408FS_SE(udev, 0, 0, count, &freq, options, in_data);
	for ( i = 0; i < count; i++ ) {
	  printf("data[%d] = %#hx  %.2fV\n", i, in_data[i], volts_1408FS_SE(in_data[i]));
	}
	break;
      case 'f':
        usbGetAll_USB1408FS(udev, &allValue);
	for (i = 0; i < 4; i++) {
	  printf("Differential Reference Low channel[%d] = %hd \n", i, allValue.ref_Low[i]);
	}
	for (i = 0; i < 4; i++) {
	  printf("Differential Reference High channel[%d] = %hd \n", i, allValue.ref_High[i]);
	}
	for (i = 0; i < 8; i++) {
	  printf("Single Ended Input channel[%d] = %hd \n", i, allValue.se[i]);
	}
	printf("DIO Port A = %#x\n", allValue.dio_portA);
	printf("DIO Port B = %#x\n", allValue.dio_portB);
	printf("\n\n\n");
	break;
      case 's':
        getUsbSerialNumber(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbGetStatus_USB1408FS(udev));
	break;
      case 'r':
        usbReset_USB1408FS(udev);
        return 0;
	break;
      case 'm':
	do {
	  printf("Enter address [0x200 - 0x033c]: ");
	  scanf("%hx", &address);
	  usbReadMemory_USB1408FS(udev, address, 4, memory);
	  fvalue = (float *) memory;
	  printf("%#hx %#hx %#hx %#hx %f\n", memory[0], memory[1], memory[2], memory[3], *fvalue);
	  
        } while (toContinue());
      case 'e':
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 1);
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT| 2);
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 3);
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 4);
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 5);
	for (i = 0; i <= 3; i++) {
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
