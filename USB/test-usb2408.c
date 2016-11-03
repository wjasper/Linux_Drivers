/*
 *
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
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>

#include "pmd.h"
#include "usb-2408.h"

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
  int flag;
  libusb_device_handle *udev = NULL;
  struct tm calDate;
  int ch;
  int temp;
  int transferred;
  int i, j;
  int tc_type;
  uint8_t input;
  uint8_t channel, gain, rate, mode, flags;
  char serial[9];
  uint16_t version[4];
  float cjc[2];
  int queue_index;
  int data;
  int ret;
  double table_AIN[NGAINS_2408][2];
  double table_AO[NCHAN_AO_2408][2];
  float table_CJCGrad[nCJCGrad_2408];
  AInScanQueue scanQueue;
  int usb2408_2AO = FALSE;
  double voltage;
  double temperature;
  double frequency;
  double sine[512];
  int16_t sdata[512];  // holds 16 bit signed analog output data
  int32_t idata[512];  // holds 24 bit signed analog input data
  uint8_t status;
  uint16_t depth;
  uint16_t count;

  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB2408_PID, NULL))) {
    printf("Success, found a USB 2408!\n");
  } else if ((udev = usb_device_find_USB_MCC(USB2408_2AO_PID, NULL))) {
    printf("Success, found a USB 2408_2AO!\n");
    usb2408_2AO = TRUE;
  } else {
    printf("Failure, did not find a USB 2408 or 2408_2AO!\n");
    return 0;
  }

  //print out the wMaxPacketSize.  Should be 64
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  usbBuildGainTable_USB2408(udev, table_AIN);
  for (i = 0; i < NGAINS_2408; i++) {
    printf("Gain: %d    Slope = %lf    Offset = %lf\n", i, table_AIN[i][0], table_AIN[i][1]);
  }

  printf("\n");
  usbBuildCJCGradientTable_USB2408(udev, table_CJCGrad);
  for (i = 0; i < nCJCGrad_2408; i++) {
    printf("Ch: %d    CJC gradient = %lf\n", i, table_CJCGrad[i]);
  }

  if (usb2408_2AO) {
    usbBuildGainTable_USB2408_2AO(udev, table_AO);
    printf("\n");
    for (i = 0; i < NCHAN_AO_2408; i++) {
      printf("VDAC%d:    Slope = %lf    Offset = %lf\n", i, table_AO[i][0], table_AO[i][1]);
    }
  }

  usbCalDate_USB2408(udev, &calDate);
  printf("\n");
  printf("MFG Calibration date = %s\n", asctime(&calDate));

  while(1) {
    printf("\nUSB 2408 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'C' to test continuous sampling at 1000 Hz.\n");
    printf("Hit 'd' to test digitial IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'j' read CJC sensors.\n");
    printf("Hit 'o' to test Analog Output\n");
    printf("Hit 'O' to test Analog Output Scan\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 't' to get TC temperature\n");
    printf("Hit 'v' to get version numbers\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
      case 'b': /* test to see if led blinks */
        printf("Enter number or times to blink: ");
	scanf("%d", &temp);
        usbBlink_USB2408(udev, temp);
        break;
      case 'c':
        usbCounterInit_USB2408(udev, COUNTER0);
        printf("Connect DO0 to CTR0\n");
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDOut_USB2408(udev, 0x0, 0);
	  usbDOut_USB2408(udev, 0xff, 0);
        }
        printf("Count = %d.  Should read 100.\n", usbCounter_USB2408(udev, COUNTER0));
        break;
      case 'C':
	printf("Testing USB-2408 Analog Input Scan in Continuous mode 8 channels\n");
	printf("Hit any key to exit\n");
	usbAInScanStop_USB2408(udev);
	count = 0;        // for continuous mode
	rate = HZ1000;
	mode = DIFFERENTIAL;
	scanQueue.count = 8;
	for (i = 0; i < 8; i++ ) {
	  scanQueue.queue[i].channel = i;
	  scanQueue.queue[i].mode = mode;
	  scanQueue.queue[i].range = BP_10V;
	  scanQueue.queue[i].rate = rate;
	}
	usbAInScanQueueWrite_USB2408(udev, &scanQueue);
	
	for (i = 0; i < 512; i++) {
	  idata[i] = 0xdeadbeef;
	}
	usbAInScanStart_USB2408(udev, 100, count, 15);
	
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
        j = 0;
	do {
          ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_IN|1, (unsigned char *) idata, 512*4, &transferred, 1000);
	  if (ret < 0) {
	    perror(" Continuous scan error in libusb_bulk_transfer");
	  }
	  for (i = 0; i < 512; i++) {
	    queue_index = idata[i] >> 24;                         // MSB of data contains the queue index;
	    gain = scanQueue.queue[queue_index].range;
	    channel = scanQueue.queue[queue_index].channel;
	    data = int24ToInt(idata[i]);                          // convert from signed 24 to signed 32
	    data = data*table_AIN[gain][0] + table_AIN[gain][1];  // correct for non-linearities in A/D
	    printf("Sample %d Index %d Channel %d   gain = %d raw = %#x  voltage = %f\n",
		   i, queue_index, channel, gain, idata[i], volts_USB2408(gain, data));
	  }
	  // printf("bulk transfer = %d\n", j); // Each transfer contains 512 samples or 64 scans
	  j++;
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
        usbAInScanStop_USB2408(udev);
	usbReset_USB2408(udev);
        libusb_close(udev);
        sleep(2); // let things settle down.
        break;
      case 'd':
        printf("\nTesting Digital I/O....\n");
        do {
  	  printf("Enter a number [0-0xff] : " );
  	  scanf("%x", &temp);
	  usbDOut_USB2408(udev, (uint8_t)temp, 0);
	  input = usbDOutR_USB2408(udev, 0);
	  printf("The number you entered = %#x\n\n",input);
  	  input = usbDIn_USB2408(udev, 0);
  	  printf("The number you entered = %#x\n\n",input);
	} while (toContinue());
	break;
      case 'e':
        cleanup_USB2408(udev);
        return 0;
      case 'i':
        printf("Input channel [0-7]: ");
        scanf("%hhd", &channel);
	printf("Gain Range for channel %d:  1 = 10V  2 = 5V  2 = 2.5V Differential: ", channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
  	  case '1': gain = BP_10V; break;
	  case '2': gain = BP_5V; break;
	  case '3': gain = BP_2_5V; break;
	  default:  gain = BP_10V; break;
	}
	rate = HZ1000;
	mode = DIFFERENTIAL;
	for (i = 0; i < 20; i++) {
  	  data = usbAIn_USB2408(udev, channel, mode, gain, rate, &flags);
	  data = data*table_AIN[gain][0] + table_AIN[gain][1];
	  printf("Channel %d  Sample[%d] = %#x  Volts = %lf\n", channel, i, data,
		 volts_USB2408(gain, data));
	  usleep(50000);
	}
	break;
      case 'I':
        printf("Testing USB-2408 Analog Input Scan\n");
	usbAInScanStop_USB2408(udev);
	printf("Input channel [0-7]: ");
        scanf("%hhd", &channel);
	printf("Input the number of scans (1-512): ");
	scanf("%hd", &count);
	printf("Gain Range for channel %d: 1 = 10V  2 = 5V  3 = 2.5V Differential: ", channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
  	  case '1': gain = BP_10V; break;
	  case '2': gain = BP_5V; break;
	  case '3': gain = BP_2_5V; break;
	  default:  gain = BP_10V; break;
	}
	rate = HZ1000;
	mode = DIFFERENTIAL;
	scanQueue.count = 1;
	scanQueue.queue[0].channel = channel;
	scanQueue.queue[0].mode = mode;
	scanQueue.queue[0].range = gain;
	scanQueue.queue[0].rate = rate;

	for (i = 0; i < 512; i++) {
	  idata[i] = 0xdeadbeef;
	}

	usbAInScanQueueWrite_USB2408(udev, &scanQueue);
	usbAInScanStart_USB2408(udev, 900, count, 15);
	usbAInScanRead_USB2408(udev, count, 1, idata);
	usbAInScanStop_USB2408(udev);

	usbAInScanQueueRead_USB2408(udev, &scanQueue);
	for (i = 0; i < count; i++) {
	  queue_index = idata[i] >> 24;                         // MSB of data contains the queue index;
	  gain = scanQueue.queue[queue_index].range;
	  channel = scanQueue.queue[queue_index].channel;
	  data = int24ToInt(idata[i]);                          // convert from signed 24 to signed 32
  	  data = data*table_AIN[gain][0] + table_AIN[gain][1];  // correct for non-linearities in A/D
	  printf("Sample %d Index %d Channel %d   gain = %d raw = %#x  voltage = %f\n",
		 i, queue_index, channel, gain, idata[i], volts_USB2408(gain, data));
	}
	break; 

      case 'j':
	usbCJC_USB2408(udev, cjc);
	for (i = 0; i < 2; i++) {
	  printf("CJC sensor[%d] = %f degree C.\n", i, cjc[i]);
	}
	break;
      case 'O':
        if (!usb2408_2AO) {
	  printf("Analog output only on the USB-2408-2AO model.\n");
	  break;
	}
        channel = 0;
	printf("Test of Analog Output Scan.\n");
	printf("Hook scope up to VDAC 0\n");
	printf("Enter desired frequency of sine wave [Hz]: ");
	scanf("%lf", &frequency);
	for (i = 0; i < 512; i ++) {
	  sine[i] = 10*sin(2.*M_PI*i/128.);
	}
	voltsTos16_USB2408_2AO(sine, sdata, 512, table_AO[channel]);
	usbAOutScanStop_USB2408_2AO(udev);
	usbAOutScanStart_USB2408_2AO(udev, 128.*frequency, 0, (1 << channel));
	printf("Hit \'s <CR>\' to stop ");
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT|1, (unsigned char *) sdata, sizeof(sdata), &transferred, 1000);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbAOutScanStop_USB2408_2AO(udev);
	break;
      case 'o':
	if (!usb2408_2AO) {
	  printf("Analog output only on the USB-2408-2AO model.\n");
	  break;
	}
        printf("output value on VDAC 0\n");
        do {
	  printf("Enter output voltage [-10 to 10]: ");
	  scanf("%lf", &voltage);
	  usbAOut_USB2408_2AO(udev, 0, voltage, table_AO);
        } while (toContinue());
	break;
      case 'r':
	usbReset_USB2408(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB2408(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB2408(udev));
	status = usbAInScanStatus_USB2408(udev, &depth);
        printf("Analog In status = %#x, depth = %d\n", status, depth);

        break;
      case 't':
	printf("Input channel [0-7]: ");
        scanf("%hhd", &channel);
	printf("Input Thermocouple type [J,K,R,S,T,N,E,B]: ");
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case 'J': 
  	  case 'j':
	    tc_type = TYPE_J; break;
  	  case 'K': 
  	  case 'k': 
	    tc_type = TYPE_K; break;
	  case 'R':
	  case 'r':
	    tc_type = TYPE_R; break;
  	  case 'S':
	  case 's':
	    tc_type = TYPE_S; break;
   	  case 'T':
	  case 't':
	    tc_type = TYPE_T; break;
     	  case 'N':
	  case 'n':
	    tc_type = TYPE_N; break;
       	  case 'E':
  	  case 'e':
	    tc_type = TYPE_E; break;
	  case 'B':
 	  case 'b':
	    tc_type = TYPE_B; break;
	  default: tc_type = TYPE_J; break;
	}
	temperature = tc_temperature_USB2408(udev, tc_type, channel);
	printf("Temperature = %.3f C  %.3f F\n", temperature, temperature*9./5. + 32.);
        break;
      case 'v':
        usbGetVersion_USB2408(udev, version);
	printf("USB micro firmware version = %x.%2.2x\n", version[0]/0x100, version[0]%0x100);
	printf("USB update firmware version = %x.%2.2x\n", version[1]/0x100, version[1]%0x100);
	printf("isolated micro firmware version = %x.%2.2x\n", version[2]/0x100, version[2]%0x100);
	printf("isolated update firmware version = %x.%2.2x\n", version[3]/0x100, version[3]%0x100);
	break;
      default:
        break;
    }
  }
}

