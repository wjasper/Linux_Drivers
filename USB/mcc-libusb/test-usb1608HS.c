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

#include "pmd.h"
#include "usb-1608HS.h"

#define MAX_COUNT     (0xffff)
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
  int usb1608HS_2AO = FALSE;
  int ch;
  int i, j;
  int transferred;
  uint8_t input;
  uint8_t config_array[8];
  char serial[9];
  int temp;
  int fd;       // file descriptor
  int ret;
  
  uint8_t chan;    // begining channel number
  uint8_t nchan;   // number of channels to scan
  int nbytes;   // number of bytes to read per scan
  float freq;   // sampling frequency [Hz]
  uint32_t count;  // number of samples to take
  uint16_t data[MAX_COUNT]; // data
  uint16_t data1[512], data2[512];
  uint8_t options;
  int channel;
  uint8_t gain;
  float voltage;

  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB1608HS_PID, NULL))) {
    printf("Success, found a USB 1608HS!\n");
  } else if ((udev = usb_device_find_USB_MCC(USB1608HS_2AO_PID, NULL))) {
    printf("Success, found a USB 1608HS-2AO!\n");
    usb1608HS_2AO = TRUE;
  } else {
    printf("Failure, did not find a USB 1608HS!\n");
    return 0;
  }
  // some initialization

  //print out the wMaxPacketSize. Should be 512.
  printf("wMaxPacketSize = %d\n\n", usb_get_max_packet_size(udev,0));

  for (i = 0; i < 8; i++) {
    config_array[i] = GROUND;  
  }
  usbAInConfig_USB1608HS(udev, config_array);
  usbDOut_USB1608HS(udev, 0x0);    // clear all ouptuts.
  usbAInConfigRead_USB1608HS(udev, config_array);
  for (i = 0; i < 8; i++) {
    printf("Channel %d Config = %#x\n", i, config_array[i]);
  }
  if (usb1608HS_2AO) {
    usbAOutConfig_USB1608HS(udev, 0x0);
  }

  while(1) {
    printf("\nUSB 1608HS Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to test digitial IO\n");
    printf("Hit 'l' to test digitial bit IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'C' to test Continuous input Scan\n");
    printf("Hit 't' for internal temperature\n");
    printf("Hit 'o' to test Analog Output\n");
    printf("Hit 'O' to test Analog Output Scan\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to set serial number\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
      case 'b': /* test to see if led blinks */
        printf("Enter number or times to blink: ");
	scanf("%d", &temp);
        usbBlink_USB1608HS(udev, temp);
        break;
    case 'c':
      usbCounterInit_USB1608HS(udev);
      printf("Connect DO0 to CTR\n");
      toContinue();
      for (i = 0; i < 100; i++) {
	usbDOutBit_USB1608HS(udev, 0, 0);
	usbDOutBit_USB1608HS(udev, 0, 1);
      }
      printf("Count = %d.  Should read 100.\n", usbCounter_USB1608HS(udev));
      break;      
      case 'd':
        printf("\nTesting Digital I/O....\n");
        printf("connect pins DIO IN <=>  DIO OUT\n");
        do {
          printf("Enter a byte number [0-0xff] : " );
          scanf("%x", &temp);
          usbDOut_USB1608HS(udev, (uint8_t)temp);
          input = usbDIn_USB1608HS(udev);
          printf("The number you entered = %#x\n\n",input);
	  for (i = 0; i < 8; i++) {
	    printf("Bit %d = %d\n", i, usbDInBit_USB1608HS(udev, i));
	  }
        } while (toContinue());
	break;
    case 'i':
        printf("Input channel [0-7]: ");
        scanf("%d", &channel);
	printf("Gain Range for channel %d: 1 = 10V  2 = 5V  3 = 2V  4 = 1V Differential: ", channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = DE_10_00V; break;
  	  case '2': gain = DE_5_00V; break;
	  case '3': gain = DE_2_00V; break;
	  case '4': gain = DE_1_00V; break;
	  default:  gain = DE_10_00V; break;
	}
        config_array[channel] = gain;
	usbAInConfig_USB1608HS(udev, config_array);
        usleep(500000);
	for (i = 0; i < 60; i++) {
  	  usbAIn_USB1608HS(udev, data);
	  printf("Channel %d  Sample[%d] = %#x  Volts = %f\n", channel, i, data[channel],
		 volts_USB1608HS(udev, channel, gain, data[channel]));
	  usleep(500000);
	}
	break;
      case 'I':
        printf("Simple test for AInScan.\n");
	printf("Input beginning channel number [0-7]: ");
	scanf("%hhd", &chan);
	channel = chan;
	printf("Enter number of channels to scan - 1 [0-7]: ");
	scanf("%hhd", &nchan);
	printf("Input sampling frequency [Hz]: ");
	scanf("%f", &freq);
	printf("Input number of scans [1-1024]: ");
	scanf("%d", &count);
	options = AIN_SINGLE_MODE;
	for (i = 0; i <= nchan; i++) {
	  printf("Gain Range for channel %d: 1 = 10V  2 = 5V  3 = 2V  4 = 1V Differential: ", (chan + i)%8);
	  while((ch = getchar()) == '\0' || ch == '\n');
	  switch(ch) {
	    case '1': gain = DE_10_00V; break;
  	    case '2': gain = DE_5_00V; break;
   	    case '3': gain = DE_2_00V; break;
	    case '4': gain = DE_1_00V; break;
	    default:  gain = DE_10_00V; break;
	  }
	  config_array[(chan+i)%8] = gain;
	}
     	usbAInConfig_USB1608HS(udev, config_array);
	usbAInScanConfig_USB1608HS(udev, chan, nchan, count, freq, options);
	do {
	  usbAInScanStart_USB1608HS(udev);
	  usbAInScan_USB1608HS(udev, data);
	  usbAInScanStop_USB1608HS(udev);
	  for (i = 0; i < count; i++) {
	    for (j = 0; j <= nchan; j++) {
 	      printf("Sample %d  Channel %d Value = %#x  Volts = %f\n", i, (chan+j)%8, data[i*(nchan+1)+j],
		     volts_USB1608HS(udev, (chan+j)%8, config_array[(chan+j)%8], data[i*(nchan+1)+j]));
	    }
	  }
	} while (toContinue());
	break;
      case 'C':
        printf("Simple test for AInScan for Continuous scan.\n");
        printf("Put in a 25 kHz Squre wave: 5 samples hi, 5 samples low\n");
        printf("Sample Rate = 250kHz,  Gain = +/- 5V,  Channel 0\n");
        fd = open("./A_D_Test.dat", O_CREAT | O_NONBLOCK | O_RDWR);
        channel = 0;
        nchan = 3;
        freq = 250000;
        gain = DE_5_00V;
        nbytes = 1024;
	config_array[0] = gain;
        options = 0x0;
     	usbAInConfig_USB1608HS(udev, config_array);
	usbAInScanConfig_USB1608HS(udev, channel, nchan, count, freq, options);
	usbAInScanStart_USB1608HS(udev);
        for (i = 0; i < freq*30*60/(nbytes); i++) {
	  libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) data1, nbytes, &transferred, 1000);
	  write(fd, data1, nbytes);
	  libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) data2, nbytes, &transferred, 1000);
	  write(fd, data2, nbytes);
	}
        close(fd);
	usbAInScanStop_USB1608HS(udev);

        printf("Analyzing Data ...\n");
        for (i = 0; i < nbytes/2; i+=4) {
	    printf("Sample %d  Channel %d Value = %#x  Volts = %f\n", i, channel, (int) data1[i],
		   volts_USB1608HS(udev, channel, config_array[0],  data1[i]));
	}
        break;
      case 'D': 
        printf("Analyzing Data ...\n");
        nbytes = 200;
        channel = 0;
        gain = DE_5_00V;
        fd = open("./A_D_Test.dat", O_RDWR);
        j = 1;
	while (read(fd, data, nbytes*8) != EOF) {
          voltage = 0.0;
	  for (i = 0; i < nbytes*4; i+=4) {
	    voltage += volts_USB1608HS(udev, channel, gain, data[i]);
	    printf("Sample %d  Channel %d Value = %#x  Volts = %f\n", i, channel, (int) data[i],
		   volts_USB1608HS(udev, channel, gain, data[i]));
	  }
          voltage /= nbytes;
	  if (fabs(voltage) > .01) {
	    printf("Error in data: voltage = %f\n", voltage);
	  }
          printf("\n\n\n\n");
	  if (j%10 == 0) printf("Read %d samples of data\n", j);
          j++;
	}
        close(fd);
        printf("End of Scan ...\n");
        break;
      case 'l':
	for (i = 0; i < 7; i++) {
	  usbDOut_USB1608HS(udev, 0X0); // clear all the LEDs
	  usbDOutBit_USB1608HS(udev, i, 0x1); // clear all the LEDs
	  sleep(1);
	}
	for (i = 7; i >= 0; i--) {
	  usbDOut_USB1608HS(udev, 0X0); // clear all the LEDs
	  usbDOutBit_USB1608HS(udev, i, 0x1); // clear all the LEDs
	  sleep(1);
	}
	break;
      case 'e':
        if (usb1608HS_2AO) {
	  usbAOutScanStop_USB1608HS(udev);	
	  data[0] = 0x8000;
	  data[1] = 0x8000;
	  usbAOut_USB1608HS(udev, data);
	  usbAOut_USB1608HS(udev, data);
	}
	usbDOut_USB1608HS(udev, 0x0);    // clear all ouptuts.
        cleanup_USB1608HS(udev);
        return 0;
      case 'o':
        if (!(usb1608HS_2AO)) {
	  printf("Analog output only on the USB-1608HS-2AO model.\n");
	  break;
	}
        printf("Enter desired output voltage ch 0 [0-10]: ");
	scanf("%f", &voltage);
	data[0] = voltage*32767/10. + 0x8000;
        printf("Enter desired output voltage ch 1 [0-10]: ");
	scanf("%f", &voltage);
	data[1] = voltage*32767/10. + 0x8000;
	usbAOut_USB1608HS(udev, data);
	break;
      case 'O':
        if (!(usb1608HS_2AO)) {
	  printf("Analog output only on the USB-1608HS-2AO model.\n");
	  break;
	}
        printf("Output square wave at 5kHz.\n");
	usbAOutScanStop_USB1608HS(udev);
        j = 0;
	for (i = 0; i < 0xffff; i+=8) {
	  data[i+0] = 0x0;
  	  data[i+1] = 0xffff;
	  data[i+2] = 0x0;
  	  data[i+3] = 0xffff;
	  data[i+4] = 0x0;
  	  data[i+5] = 0xffff;
	  data[i+6] = 0x0;
  	  data[i+7] = 0xffff;
	}
        freq = 10000.;  // Hz (rate DAC is changing, which is 2x square wave)
	usbAOutScanConfig_USB1608HS(udev, 0, freq, 0x1);
	usbAOutScanStart_USB1608HS(udev);
	for (j = 0; j < 5; j++ ) {
  	  usbAOutScan_USB1608HS(udev, data, 0xffff);
	  printf("j = %d\n", j);
	}
	usbAOutScanStop_USB1608HS(udev);
        break;
      case 't':
        printf("Internal Temperature = %.2f degree Celsius.\n", usbTemperature_(udev));
        break;
      case 'r':
	usbReset_USB1608HS(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB1608HS(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      default:
        break;
    }
  }
}
