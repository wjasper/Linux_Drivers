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
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include <unistd.h>

#include "pmd.h"
#include "usb-1608G.h"

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
  struct tm calDate;
  
  double voltage;
  double frequency, duty_cycle;
  float temperature;
  float table_AIN[NGAINS_1608G][2];
  float table_AO[NCHAN_AO_1608GX][2];
  ScanList list[NCHAN_1608G];  // scan list used to configure the A/D channels.

  int ch;
  int i, j, m, k, nchan, repeats;
  int nread;
  int nScans = 0;
  uint8_t input;
  int temp, ret;
  uint8_t options;
  char serial[9];
  uint32_t period;
  uint16_t version;
  uint16_t status;
  int usb1608GX_2AO = FALSE;
  int flag;
  int transferred;            // number of bytes transferred
  uint16_t value, data;
  uint16_t *sdataIn;          // holds 16 bit unsigned analog input data
  uint16_t sdataOut[512];     // holds 16 bit unsigned analog output data

  uint8_t mode, gain, channel;

  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("libusb_init: Failed to initialize libusb");
    exit(1);
  }

  // There are 2 firmware versions of the board.  See MCC for details.
  if ((udev = usb_device_find_USB_MCC(USB1608G_V2_PID, NULL))) {
    printf("Success, found a USB 1608G!\n");
    usbInit_1608G(udev, 2);
  } else if ((udev = usb_device_find_USB_MCC(USB1608GX_V2_PID, NULL))) {
    printf("Success, found a USB 1608GX!\n");
    usbInit_1608G(udev, 2);
  } else if ((udev = usb_device_find_USB_MCC(USB1608GX_2AO_V2_PID, NULL))) {
    printf("Success, found a USB 1608GX_2AO!\n");
    usb1608GX_2AO = TRUE;
    usbInit_1608G(udev, 2);
  } else if ((udev = usb_device_find_USB_MCC(USB1608G_PID, NULL))) {
    printf("Success, found a USB 1608G!\n");
    usbInit_1608G(udev, 1);
  } else if ((udev = usb_device_find_USB_MCC(USB1608GX_PID, NULL))) {
    printf("Success, found a USB 1608GX!\n");
    usbInit_1608G(udev, 1);
  } else if ((udev = usb_device_find_USB_MCC(USB1608GX_2AO_PID, NULL))) {
    printf("Success, found a USB 1608GX_2AO!\n");
    usb1608GX_2AO = TRUE;
    usbInit_1608G(udev, 1);
  } else {
    printf("Failure, did not find a USB 1608G series device!\n");
    return 0;
  }

  //print out the wMaxPacketSize.  Should be 512
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  usbBuildGainTable_USB1608G(udev, table_AIN);
  for (i = 0; i < NGAINS_1608G; i++) {
    printf("Gain: %d   Slope = %f   Offset = %f\n", i, table_AIN[i][0], table_AIN[i][1]);
  }

  if (usb1608GX_2AO) {
    usbBuildGainTable_USB1608GX_2AO(udev, table_AO);
    printf("\n");
    for (i = 0; i < NCHAN_AO_1608GX; i++) {
      printf("VDAC%d:    Slope = %f    Offset = %f\n", i, table_AO[i][0], table_AO[i][1]);
    }
  }

  usbCalDate_USB1608G(udev, &calDate);
  printf("\n");
  printf("MFG Calibration date = %s\n", asctime(&calDate));

  while(1) {
    printf("\nUSB 1608G/1608GX/1608GX_2AO Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'C' to test continuous sampling at 1000 Hz.\n");
    printf("Hit 'd' to test digital IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'o' to test Analog Output\n");
    printf("Hit 'O' to test Analog Output Scan\n");
    printf("Hit 'p' to test Pulse Width Modulation\n");
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
        usbBlink_USB1608G(udev, options);
	break;
      case 'c':
        usbCounterInit_USB1608G(udev, COUNTER0);
        printf("Connect DIO0 to CTR0\n");
	usbDTristateW_USB1608G(udev, 0xf0);
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDLatchW_USB1608G(udev, 0x0);
	  usbDLatchW_USB1608G(udev, 0x1);
        }
        printf("Count = %d.  Should read 100.\n", usbCounter_USB1608G(udev, COUNTER0));
        break;      
      case 'd':
        printf("\nTesting Digital I/O...\n");
	printf("connect pins DIO[0-3] <--> DIO[4-7]\n");
	usbDTristateW_USB1608G(udev,0xf0);
	printf("Digital Port Tristate Register = %#x\n", usbDTristateR_USB1608G(udev));
	do {
          printf("Enter a byte number [0-0xf] : " );
          scanf("%x", &temp);
	  temp &= 0xf;
          usbDLatchW_USB1608G(udev, (uint16_t)temp);
	  temp = usbDLatchR_USB1608G(udev);
          input = usbDPort_USB1608G(udev) >> 4;
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, temp);
	  for (i = 0; i < 4; i++) {
	    printf("Bit %d = %d\n", i, (temp>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'p':
        printf("Enter desired frequency: ");
        scanf("%lf", &frequency);
        printf("Enter duty cycle: ");
        scanf("%lf", &duty_cycle);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
	  usbDLatchW_USB1608G(udev, (uint16_t) 0x1);
          usleep((useconds_t) 1000000./frequency*duty_cycle);
	  usbDLatchW_USB1608G(udev, (uint16_t) 0x0);
          usleep((useconds_t) 1000000./frequency*(1. - duty_cycle));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
        usbDLatchW_USB1608G(udev, (uint16_t) 0x0);
        break;
      case 'e':
	usbDLatchW_USB1608G(udev, 0x0);                  // zero out the DIO
	if (usb1608GX_2AO) {
	  usbAOutScanStop_USB1608GX_2AO(udev);
	  usbAOut_USB1608GX_2AO(udev, 0, 0x0, table_AO);
	  usbAOut_USB1608GX_2AO(udev, 1, 0x0, table_AO);
	}
        cleanup_USB1608G(udev);
        return 0;
      case 'i':
	printf("Enter 0 for Single Ended, 1 for Differential: ");
	scanf("%hhd", &mode);
	if (mode == SINGLE_ENDED) {
  	  mode = SINGLE_ENDED;
	  printf("Input channel [0-15]: ");
	  scanf("%hhd", &channel);
	} else {
	  mode = DIFFERENTIAL;
	  printf("Input channel [0-7]: ");
	  scanf("%hhd", &channel);
	}	  
	printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = +/- 2V  4 = +/- 1V: ",channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_10V; break;
  	  case '2': gain = BP_5V; break;
	  case '3': gain = BP_2V; break;
	  case  '4': gain = BP_1V; break;
	  default:  gain = BP_10V; break;
	}
	mode |= LAST_CHANNEL ; 
	list[0].range = gain;
        list[0].mode = mode;
	list[0].channel = channel;
	usbAInConfig_USB1608G(udev, list);
	for (i = 0; i < 20; i++) {
	  value = usbAIn_USB1608G(udev, channel);
	  if (value >=  0xfffd) {
	    printf("DAC is saturated at +FS\n");
	  } else if  (value <= 0x60) {
	    printf("DAC is saturated at -FS\n");
	  } else {
	    value = rint(value*table_AIN[gain][0] + table_AIN[gain][1]);
	  }
	  printf("Channel %d  Mode = %#x  Gain = %d Sample[%d] = %#x Volts = %lf\n",
		 list[0].channel, list[0].mode, list[0].range, i, value, volts_USB1608G(gain, value));
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing USB-1608G Multi-Channel Analog Input Scan.\n");
	usbAInScanStop_USB1608G(udev);
	usbAInScanClearFIFO_USB1608G(udev);
	printf("Enter 0 for Single Ended, 1 for Differential: ");
	scanf("%hhd", &mode);
	if (mode == SINGLE_ENDED) {
  	  mode = SINGLE_ENDED;
	  printf("enter number of channels (1-16) :");
	  scanf("%d", &nchan);
	  if (nchan > 16) break;
	} else {
	  mode = DIFFERENTIAL;
	  printf("enter number of channels (1-8) :");
	  scanf("%d", &nchan);
	  if (nchan > 8) break;
	}	  
        printf("Enter number of scans: ");
        scanf("%d", &nScans);
        printf("Enter number of repeats: ");
        scanf("%d", &repeats);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
        for (channel = 0; channel < nchan; channel++) {
	  printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = +/- 2V  4 = +/- 1V: ",channel);
	  while((ch = getchar()) == '\0' || ch == '\n');
	  switch(ch) {
	    case '1': gain = BP_10V; break;
	    case '2': gain = BP_5V; break;
	    case '3': gain = BP_2V; break;
	    case '4': gain = BP_1V; break;
	    default:  gain = BP_10V; break;
	  }
	  list[channel].range = gain;  
	  list[channel].mode = mode;
	  list[channel].channel = channel;
	}
        list[nchan-1].mode |= LAST_CHANNEL;
	usbAInConfig_USB1608G(udev, list);
        if ((sdataIn = malloc(2*nchan*nScans)) == NULL) {
	  perror("Can not allocate memory for sdataIn");
	  break;
	}
        for (m = 0; m < repeats; m++) {
	  printf("\n\n---------------------------------------");
	  printf("\nrepeat: %d\n", m);
	  usbAInScanStart_USB1608G(udev, nScans, 0, frequency, 0x0);
	  ret = usbAInScanRead_USB1608G(udev, nScans, nchan, sdataIn, 20000, 0);
	  printf("Number bytes read = %d  (should be %d)\n", ret, 2*nchan*nScans);
	  for (i = 0; i < nScans; i++) {
	    printf("%6d", i);
	    for (j = 0; j < nchan; j++) {
              gain = list[j].range;
	      k = i*nchan + j;
	      if (sdataIn[k] >= 0xfffd) {
		printf("DAC is saturated at +FS\n");
	      } else if (sdataIn[k] <= 0x60) {
		printf("DAC is saturated at -FS\n");
	      } else {
		data = rint(sdataIn[k]*table_AIN[gain][0] + table_AIN[gain][1]);
	      }
	      printf(", %8.4lf", volts_USB1608G(gain, data));
	    }
	    printf("\n");
	  }
	}
	printf("\n\n---------------------------------------");
	free(sdataIn);
        break;
      case 'C':
      	printf("Testing USB-1608G Analog Input Scan in continuous mode 16 channels\n");
        printf("Hit any key to exit\n");
	printf("Enter desired sampling frequency (greater than 1000): ");
	scanf("%lf", &frequency);
	usbAInScanStop_USB1608G(udev);
        nScans = 0;         // for continuous mode
        nchan = 16;         // 16 channels
	gain = BP_10V;
	// mode = DIFFERENTIAL;
	mode = SINGLE_ENDED;

        for (channel = 0; channel < nchan; channel++) {
	  list[channel].range = gain;
	  list[channel].mode = mode;
	  list[channel].channel = channel;
	}
	list[nchan-1].mode |= LAST_CHANNEL;
	usbAInConfig_USB1608G(udev, list);

	nread = 256;
	if ((sdataIn = malloc(2*nchan*nread)) == NULL) {
	  perror("Can not allocate memory for sdataIn");
	  break;
	}
	sleep(1);
        i = 0;
        usbAInScanStart_USB1608G(udev, nScans, 0, frequency, 0x1);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
	  usbAInScanRead_USB1608G(udev, nread, nchan, sdataIn, 2000, CONTINUOUS);
          if (i%100 == 0) {
            printf("Scan = %d\n", i);
	  }
          i++;
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
        usbAInScanStop_USB1608G(udev);
	usbAInScanClearFIFO_USB1608G(udev);
	free(sdataIn);
        sleep(2); // let things settle down.
        break;
      case 'o':
        if (!(usb1608GX_2AO)) {
	  printf("Analog output only on the USB-1608GX_2AO model.\n");
  	  break;
        }
        printf("Enter voltage: ");
	scanf("%lf", &voltage);
	usbAOut_USB1608GX_2AO(udev, 0, voltage, table_AO);
	usbAOutR_USB1608GX_2AO(udev, 0, &voltage, table_AO);
	printf("Analog Output Voltage = %f V\n", voltage);
        break;
      case 'O':
	if (!(usb1608GX_2AO)) {
	  printf("Analog output only on the USB-1608GX_2AO model.\n");
	  break;
	}
	channel = 0;
	printf("Test of Analog Output Scan.\n");
	printf("Hook scope up to VDAC 0\n");
	printf("Enter desired frequency of sine wave [Hz]: ");
	scanf("%lf", &frequency);
        frequency *= 512.;
	for (i = 0; i < 512; i++) {
	  voltage = 10.*sin(2.*M_PI*i/512.);
          voltage = (voltage/10.*32768. + 32768.);
	  sdataOut[i] = voltage*table_AO[channel][0] + table_AO[channel][1];
	}
        usbAOutScanStop_USB1608GX_2AO(udev);
	usbAOutScanStart_USB1608GX_2AO(udev, 0, 0, frequency,  AO_CHAN0);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT|2, (unsigned char *) sdataOut, sizeof(sdataOut), &transferred, 400);
	  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT|2, (unsigned char *) sdataOut, 0, &transferred, 400);
	  // printf("ret = %d  status = %#x\n", ret, usbStatus_USB1608G(udev));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbAOutScanStop_USB1608GX_2AO(udev);
	break;
      case 'r':
	usbReset_USB1608G(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB1608G(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
	status = usbStatus_USB1608G(udev);
        printf("Status = %#x\n", status);
	break;
      case 't':
        printf("Enter frequency of timer: ");
        scanf("%lf", &frequency);
	period = 64.E6/frequency - 1;
	usbTimerPeriodW_USB1608G(udev, period);
	usbTimerPulseWidthW_USB1608G(udev, period / 2);
	usbTimerCountW_USB1608G(udev, 0);
	usbTimerDelayW_USB1608G(udev, 0);
	usbTimerControlW_USB1608G(udev, 0x1);
	toContinue();
	usbTimerControlW_USB1608G(udev, 0x0);
        break;
      case 'T':
        usbTemperature_USB1608G(udev, &temperature);
	printf("Temperature = %.2f deg C  or  %.2f deg F \n", temperature, 9.0/5.0*temperature + 32.);
	break;
      case 'v':
	version = 0xbeef;
        usbFPGAVersion_USB1608G(udev, &version);
	printf("FPGA version %02x.%02x\n", version >> 0x8, version & 0xff);
	break;
      default:
        break;
    }
  }
}

