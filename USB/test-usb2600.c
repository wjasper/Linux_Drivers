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
#include "usb-2600.h"

#define MAX_COUNT  (0xffff)
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
  double voltage;
  float temperature;
  float table_AIN[NGAINS_2600][2];
  float table_AOUT[NCHAN_AO_26X7][2];
  ScanList list[NCHAN_2600];  // scan list used to configure the A/D channels.
  int usb26X7 = FALSE;
  int ch;
  int i, flag;
  int transferred;
  int nSamples = 0;
  uint8_t input, timer;
  int temp, ret;
  uint8_t options;
  char serial[9];
  uint32_t period;
  uint16_t version;

  uint16_t value;
  uint16_t dataAIn[512];  // holds 16 bit unsigned analog input data
  uint16_t dataAOut[512]; // holds 16 bit unsigned analog output data

  uint8_t mode, gain, channel;

  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB2637_PID, NULL))) {
    printf("Success, found a USB 2637.\n");
    usb26X7 = TRUE;
  } else if ((udev = usb_device_find_USB_MCC(USB2633_PID, NULL))) {
    printf("Success, found a USB 2633.\n");
  } else if ((udev = usb_device_find_USB_MCC(USB2623_PID, NULL))) {
    printf("Success, found a USB 2623.\n");
  } else if ((udev = usb_device_find_USB_MCC(USB2627_PID, NULL))) {
    printf("Success, found a USB 2627.\n");
    usb26X7 = TRUE;
  } else {
    printf("Failure, did not find a USB 2600!\n");
    return 0;
  }
  // some initialization
  usbInit_2600(udev);

  //print out the wMaxPacketSize.  Should be 512
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  usbBuildGainTable_USB2600(udev, table_AIN);
  for (i = 0; i < NGAINS_2600; i++) {
    printf("Gain: %d   Slope = %f   Offset = %f\n", i, table_AIN[i][0], table_AIN[i][1]);
  }
  if (usb26X7) {
    usbBuildGainTable_USB26X7(udev, table_AOUT);
    printf("\nAnalog Output Table\n");
    for (i = 0; i < NCHAN_AO_26X7; i++) {
      printf("Gain: %d   Slope = %f   Offset = %f\n", i, table_AOUT[i][0], table_AOUT[i][1]);
    }
  }

  while(1) {
    printf("\nUSB 2600 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'd' to test digitial IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'o' to test Analog Output\n");
    printf("Hit 'O' to test Analog Output Scan\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 't' to test the timers\n");
    printf("Hit 'T' to get temperature\n");
    printf("Hit 'v' to get version numbers\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
        usbBlink_USB2600(udev, options);
	break;
      case 'c':
        usbCounterInit_USB2600(udev, COUNTER0);
        printf("Connect A0 to CTR0\n");
	usbDTristateW_USB2600(udev,0, 0x0);
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDLatchW_USB2600(udev, 0, 0x0);
	  usbDLatchW_USB2600(udev, 0, 0x1);
        }
        printf("Count = %d.  Should read 100.\n", usbCounter_USB2600(udev,COUNTER0));
        break;      
      case 'd':
        printf("\nTesting Digital I/O...\n");
	printf("connect pins Port A [0-8] <--> Port B[0-8]\n");
	usbDTristateW_USB2600(udev,0,0x00);
	usbDTristateW_USB2600(udev,1,0xff);
	printf("Digital Port 0 Tristate Register = %#x\n", usbDTristateR_USB2600(udev,0));
	printf("Digital Port 1 Tristate Register = %#x\n", usbDTristateR_USB2600(udev,1));
	do {
          printf("Enter a byte number [0-0xff]: " );
          scanf("%x", &temp);
          usbDLatchW_USB2600(udev, 0, (uint16_t)temp);
	  temp = usbDLatchR_USB2600(udev,0);
          input = usbDPort_USB2600(udev,1);
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, temp);
	  for (i = 0; i < 8; i++) {
	    printf("Bit %d = %d\n", i, (temp>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'o':
        if (!usb26X7) {
	  printf("Ananlog output only on USB-26X7 models.\n");
	  break;
	}
        printf("Output value on VDAC 0\n");
	do {
	  printf("Enter output voltage [-10 to 10]: ");
	  scanf("%lf", &voltage);
	  usbAOut_USB26X7(udev, 0, voltage, table_AOUT);
	} while (toContinue());
	break;
      case 'O':
        if (!usb26X7) {
	  printf("Analog output only on the USB-26X7 model.\n");
	  break;
	}
        channel = 0;
	printf("Test of Analog Output Scan.\n");
	printf("Hook scope up to VDAC 0\n");
	printf("Enter desired frequency of sine wave [Hz]: ");
	scanf("%lf", &frequency);
        frequency *= 512.;

	for (i = 0; i < 512; i++) {
	  voltage = 10*sin(2.*M_PI*i/512.);
          voltage = (voltage/10.*32768. + 32768.);
	  dataAOut[i] = voltage*table_AOUT[channel][0] + table_AOUT[channel][1];
	  //  dataAOut[i] = 0x0;
          //  dataAOut[i+1] = 0xffff;
	}
	usbAOutScanStop_USB26X7(udev);
	usbAOutScanStart_USB26X7(udev, 0, 0, frequency, (0x1 << channel));
	printf("Hit \'s <CR>\' to stop ");
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  if (libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT|2, (unsigned char *) dataAOut, sizeof(dataAOut), &transferred, 1000) < 0) {
	    perror("libusb_bulk_transfer error in AOutScan.");
	  }
	  libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT|2, (unsigned char *) dataAOut, 0x0, &transferred, 100);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbAOutScanStop_USB26X7(udev);
	break;
      case 'e':
        cleanup_USB2600(udev);
        return 0;
      case 'i':
	printf("Input channel [0-7]: ");
	scanf("%hhd", &channel);
	printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = +/- 2V  4 = +/- 1V: ",channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_10V; break;
  	  case '2': gain = BP_5V; break;
	  case '3': gain = BP_2V; break;
	  case '4': gain = BP_1V; break;
	  default:  gain = BP_10V; break;
	}
	mode = (LAST_CHANNEL | SINGLE_ENDED);
	list[0].range = gain;
        list[0].mode = mode;
	list[0].channel = channel;
	usbAInConfig_USB2600(udev, list);
	for (i = 0; i < 20; i++) {
	  value = usbAIn_USB2600(udev, channel);
	  value = rint(value*table_AIN[gain][0] + table_AIN[gain][1]);
	  printf("Channel %d  Mode = %#x  Gain = %d Sample[%d] = %#x Volts = %lf\n",
		 list[0].channel, list[0].mode, list[0].range, i, value, volts_USB2600(gain, value));
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing USB-2600 Analog Input Scan.\n");
	usbAInScanStop_USB2600(udev);
	usbAInScanClearFIFO_USB2600(udev);
        printf("Enter number of samples (less than 512): ");
        scanf("%d", &nSamples);
	printf("Input channel [0-7]: ");
        scanf("%hhd", &channel);
	printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = +/- 2V  4 = +/- 1V: ",channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_10V; break;
  	  case '2': gain = BP_5V; break;
	  case '3': gain = BP_2V; break;
	  case '4': gain = BP_1V; break;
	  default:  gain = BP_10V; break;
	}
	mode = (LAST_CHANNEL | SINGLE_ENDED);
        list[0].range = gain;
        list[0].mode = mode;
        list[0].channel = channel;
	usbAInConfig_USB2600(udev, list);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
	usbAInScanStart_USB2600(udev, nSamples, 0, frequency, 0xff, 0);
	ret = usbAInScanRead_USB2600(udev, nSamples, 1, dataAIn);
	printf("Number samples read = %d\n", ret/2);
	for (i = 0; i < nSamples; i++) {
	  dataAIn[i] = rint(dataAIn[i]*table_AIN[gain][0] + table_AIN[gain][1]);
	  printf("Channel %d  Mode = %d  Gain = %d Sample[%d] = %#x Volts = %lf\n", channel,
		 mode, gain, i, dataAIn[i], volts_USB2600(gain, dataAIn[i]));
	}
        break;
      case 'r':
	usbReset_USB2600(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB2600(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        printf("Status = %#x\n", usbStatus_USB2600(udev));
	break;
      case 't':
        printf("Enter timer [0-3]: ");
        scanf("%hhd", &timer);
        printf("Enter frequency of timer: ");
        scanf("%lf", &frequency);
	period = 64.E6/frequency - 1;
	usbTimerPeriodW_USB2600(udev, timer, period);
	usbTimerPulseWidthW_USB2600(udev, timer, period / 2);
	usbTimerCountW_USB2600(udev, timer, 0);
	usbTimerDelayW_USB2600(udev, timer, 0);
	usbTimerControlW_USB2600(udev, timer, 0x1);
	toContinue();
	usbTimerControlW_USB2600(udev, timer, 0x0);
        break;
      case 'T':
        usbTemperature_USB2600(udev, &temperature);
	printf("Temperature = %.2f deg C  or  %.2f deg F \n", temperature, 9.0/5.0*temperature + 32.);
	break;
      case 'v':
	version = 0xbeef;
        usbFPGAVersion_USB2600(udev, &version);
	printf("FPGA version %02x.%02x\n", version >> 0x8, version & 0xff);
	break;
    default:
        break;
    }
  }
}

