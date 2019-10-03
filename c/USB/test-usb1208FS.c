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
#include "usb-1208FS.h"

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
  uint8_t lowChan, highChan, nChan, gains[8], chan[8];
  int nScan;
  uint16_t value;
  uint16_t out_data[512];
  signed short in_data[1024];
  int count;
  int temp, i, j;
  int ch;
  float freq;
  time_t startTime, endTime;

  libusb_device_handle *udev = NULL;
  int ret;
  unsigned char serial[9];
  
  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("libusb_init: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB1208FS_PID, NULL))) {
    printf("USB-1208FS Device is found!\n");
    init_USB1208FS(udev);
  } else {
    printf("No device found.\n");
    exit(0);
  }

  //print out the wMaxPacketSize.  Should be 64
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  /* config mask 0x01 means all inputs */
  usbDConfigPort_USB1208FS(udev, DIO_PORTA, DIO_DIR_OUT);
  usbDConfigPort_USB1208FS(udev, DIO_PORTB, DIO_DIR_IN);
  usbDOut_USB1208FS(udev, DIO_PORTA, 0);
  usbDOut_USB1208FS(udev, DIO_PORTA, 0);

  while(1) {
    printf("\nUSB 1208FS Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'd' to test digital I/O\n");
    printf("Hit 'g' to test analog input scan (differential).\n");
    printf("Hit 'j' to test analog input scan (single ended).\n");
    printf("Hit 'i' to test analog input (differential mode)\n");
    printf("Hit 'h' to test analog input (single ended)\n");
    printf("Hit 'o' to test analog output\n");
    printf("Hit 'O' to test analog output scan\n");    
    printf("Hit 'r' to reset\n");
    printf("Hit 'S' to get status\n");
    printf("Hit 's' to get serial number\n");

    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
      case 'b': /* test to see if led blinks */
        usbBlink_USB1208FS(udev);
        break;
      case 'c':
        printf("connect pin 20 and 21\n");
        usbInitCounter_USB1208FS(udev);
        sleep(1);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
          usbDOut_USB1208FS(udev, DIO_PORTA, 1);
	  sleep(1);
          usbDOut_USB1208FS(udev, DIO_PORTA, 0);
	  printf("Counter = %d\n",usbReadCounter_USB1208FS(udev));
        } while (!isalpha(getchar()));
        fcntl(fileno(stdin), F_SETFL, flag);
        break;
      case 'd':
        printf("\nTesting Digital I/O....\n");
        printf("connect pins 21 through 28 <=> 32 through 39\n");
        do {
          printf("Enter a byte number [0-0xff]: " );
          scanf("%x", &temp);
          usbDOut_USB1208FS(udev, DIO_PORTA, (uint8_t)temp);
          usbDIn_USB1208FS(udev, DIO_PORTB, &input);
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
        usbAOut_USB1208FS(udev, channel, value);
        break;
      case 'O': /* test Analog Output Scan */
        printf("Enter desired frequency [Hz]: ");
        scanf ("%f", &freq);
        for ( j = 0; j <  200; j++ ) {
	  for (i = 0; i < 512; i++) {
	    out_data[i] = i%2 ? 0 : 0xfff;
	  }
	  usbAOutScan_USB1208FS(udev, 0, 0, 512, &freq, out_data, 0);
	}
        usbAOutStop_USB1208FS(udev);
	break;
      case 'g':
        printf("Enter desired frequency [Hz]: ");
        scanf("%f", &freq);
        printf("Enter number of scans [1-256]: ");
        scanf("%d", &nScan);
	printf("Enter low channel [0-3]: ");
	scanf("%hhd", &lowChan);
	printf("Enter high channel [0-3]: ");
	scanf("%hhd", &highChan);
	nChan = highChan - lowChan + 1;  // number of channels
	count = nScan*nChan;             // total number of samples
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
	for (i = 0; i < nChan; i++) {
	  chan[i] = lowChan + i;
	  gains[i] = gain;
	}
	usbALoadQueue_USB1208FS(udev, nChan, chan, gains);
	options = AIN_EXECUTION | AIN_GAIN_QUEUE;
	
        for ( i = 0; i < 1024; i++ ) {  // load data with known value
	  in_data[i] = 0xbeef;
	}
        usbAInScan_USB1208FS(udev, lowChan, highChan, count, &freq, options, in_data);
	for ( i = 0; i < nScan; i++ ) {
	  for (j = 0; j < nChan; j++ ) {
	    printf("Scan = %d  Channel = %d  data = %#hx  %.2fV\n",
		   i, j, in_data[i*nChan+j], volts_FS(gain, in_data[i*nChan+j]));
	  }
	}
	break;
      case 'j':
        printf("Test of scan mode (single ended).\n");
        printf("Enter desired frequency [Hz]: ");
        scanf("%f", &freq);
        printf("Enter number of samples [1-1024]: ");
        scanf("%d", &count);
	printf("Enter Low Channel [0-7]: ");
        scanf("%hhd", &lowChan);
	printf("Enter High Channel [0-7]: ");
        scanf("%hhd", &highChan);

	options = AIN_EXECUTION | AIN_GAIN_QUEUE;
        for ( i = 0; i < 1024; i++ ) {  // load data with known value
	  in_data[i] = 0xbeef;
	}
        usbAInScan_USB1208FS_SE(udev, lowChan, highChan, count, &freq, options, in_data);
	for ( i = 0; i < count; i++ ) {
	  printf("data[%d] = %#hx  %.2fV\n", i, in_data[i], volts_SE(in_data[i]));
	}
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
	  svalue = usbAIn_USB1208FS(udev, channel, gain);
	  printf("Channel: %d: value = %#hx, %.2fV\n",
		 channel, svalue, volts_SE(svalue));
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
          usbDOut_USB1208FS(udev, DIO_PORTA, 0);
	  sleep(1);
	  svalue = usbAIn_USB1208FS(udev, channel, gain);
	  printf("Channel: %d: value = %#hx, %.2fV\n",
		 channel, svalue, volts_FS(gain, svalue));
          usbDOut_USB1208FS(udev, DIO_PORTA, 0x1);
	  sleep(1);
	  svalue = usbAIn_USB1208FS(udev, channel, gain);
	  printf("Channel: %d: value = %#hx, %.2fV\n",
		 channel, svalue, volts_FS(gain, svalue));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
        printf("Doing a timing test.  Please wait ...\n");
        time(&startTime);
        for (count = 0; count < 500; count++) {
	  svalue = usbAIn_USB1208FS(udev, channel, gain);
	}
        time(&endTime);
        printf("Sampling speed is %ld Hz.\n", 500/(endTime - startTime));
	break;
      case 'S':
        printf("Status = %#x\n", usbGetStatus_USB1208FS(udev));
	break;
      case 'r':
        usbReset_USB1208FS(udev);
        return 0;
	break;
      case 'e':
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 1);
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_OUT| 2);
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 3);
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 4);
	libusb_clear_halt(udev, LIBUSB_ENDPOINT_IN | 5);
	for (i = 0; i < 4; i++) {
	  libusb_release_interface(udev, i);
	}
	libusb_close(udev);
	return 0;
	break;
      case 's':
	 getUsbSerialNumber(udev, serial);
         printf("Serial number = %s\n", serial);
	break;
      default:
        break;
    }
  }
}
