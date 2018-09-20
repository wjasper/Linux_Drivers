/*
 *
 *  Copyright (c) 2017 Warren J. Jasper <wjasper@ncsu.edu>
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
#include "usb-1808.h"

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
  Calibration_AIN table_AIN[NCHAN_1808][NGAINS_1808];
  Calibration_AOUT table_AO[NCHAN_AO_1808];
  ScanList list[NCHAN_1808];  // scan list used to configure the A/D channels.
  uint8_t scanQueueAOut[3];
  uint8_t scanQueueAIn[13];

  int ch;
  int i, j, m, k, nchan, repeats;
  int nread;
  int nScans = 0;
  uint8_t input;
  int temp, ret;
  uint8_t options;
  uint8_t timer;
  char serial[9];
  uint32_t count, period;
  uint16_t version;
  uint16_t status;
  int flag;
  int transferred;            // number of bytes transferred
  uint32_t value[8], data;
  uint32_t *sdataIn;          // holds 18 bit unsigned analog input data
  uint16_t sdataOut[512];     // holds 16 bit unsigned analog output data

  uint8_t mode, gain, channel;

  udev = NULL;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("libusb_init: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB1808_PID, NULL))) {
    printf("Success, found a USB 1808!\n");
    ret = usbInit_1808(udev);
    if (ret < 0) {
      exit(1);
    }
  } else if ((udev = usb_device_find_USB_MCC(USB1808X_PID, NULL))) {
    printf("Success, found a USB 1808X!\n");
    usbInit_1808(udev);
  } else {
    printf("Failure, did not find a USB 1808 series device!\n");
    return 0;
  }

  //print out the wMaxPacketSize.  Should be 512
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  usbBuildGainTableAI_USB1808(udev, table_AIN);
  for (i = 0; i < NCHAN_1808; i++) {
    for (j = 0; j < NGAINS_1808; j++) {
      printf("Channel: %d  Gain: %d   Slope = %f   Offset = %f\n", i, j, table_AIN[i][j].slope, table_AIN[i][j].offset);
    }
  }

  usbBuildGainTableAO_USB1808(udev, table_AO);
  printf("\n");
  for (i = 0; i < NCHAN_AO_1808; i++) {
    printf("VDAC%d:    Slope = %f    Offset = %f\n", i, table_AO[i].slope, table_AO[i].offset);
  }

  usbCalDate_USB1808(udev, &calDate);
  printf("\n");
  printf("MFG Calibration date = %s\n", asctime(&calDate));

  for (i = 0; i < 8; i++) {
    list[i].range = BP_10V;
    list[i].mode = DIFFERENTIAL;
  }

  usbADCSetupW_USB1808(udev, list);
  
  while(1) {
    printf("\nUSB 1808/1808X Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'C' to test continuous sampling greater than 1000 Hz.\n");
    printf("Hit 'd' to test digital IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'o' to test Analog Output\n");
    printf("Hit 'O' to test Analog Output Scan\n");
    printf("Hit 'p' to test timer and counter frequency.\n");
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
        usbBlink_USB1808(udev, options);
	break;
      case 'd':
        printf("\nTesting Digital I/O...\n");
	usbDTristateW_USB1808(udev,0x0);                 // set pins to output
	printf("Digital Port Tristate Register = %#x\n", usbDTristateR_USB1808(udev));
	do {
          printf("Enter a nibble number [0-0xf] : " );
          scanf("%x", &temp);
	  temp &= 0xf;
          usbDLatchW_USB1808(udev, (uint16_t)temp);
	  input = usbDLatchR_USB1808(udev);
          printf("The number you entered = %#x   Latched value = %#x\n\n",temp, input);
	  for (i = 0; i < 4; i++) {
	    printf("Bit %d = %d\n", i, (temp>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'c':
	usbCounterOptionsW_USB1808(udev, COUNTER0, 0x0);
	usbCounterModeW_USB1808(udev, COUNTER0, 0x0);
	usbCounterW_USB1808(udev, COUNTER0, 0x0);
        printf("Connect DIO0 to CTR0\n");
	usbDTristateW_USB1808(udev, 0x0);
        toContinue();
        for (i = 0; i < 100; i++) {
	  usbDLatchW_USB1808(udev, 0x0);
	  usbDLatchW_USB1808(udev, 0x1);
        }
	usbCounterR_USB1808(udev, COUNTER0, &count);
        printf("Count = %d.  Should read 100.\n", count);
        break;
      case 'p':
	printf("Test counter and timer.\n");
	printf("Connect Timer0 to Counter0\n");
	printf("Enter desired frequency: ");
	scanf("%lf", &frequency);
	duty_cycle = 0.5;
	usbTimerControlW_USB1808(udev, TIMER0, 0x0);  // stop timer0
	usbTimerParametersW_USB1808(udev, TIMER0, frequency, duty_cycle, 0, 0);
	usbTimerControlW_USB1808(udev, TIMER0, TIMER_ENABLE);  // enable timer0
	usbCounterParametersW_USB1808(udev, COUNTER0, COUNTER_PERIOD | PERIOD_MODE_10X, 0x0);
	sleep(1);
	usbCounterR_USB1808(udev, COUNTER0, &period);
	frequency = 100.E6/(period + 1)*5.0;
	usbTimerControlW_USB1808(udev, TIMER0, 0x0);  // stop timer0
	printf("frequency = %f\n", frequency);
	break;
      case 'i':
	printf("Input channel [0-7]: ");
	scanf("%hhd", &channel);
	printf("Enter 0 for Differential and 1 for Single Ended: ");
	scanf("%hhd", &mode);
	if (mode == 0) {
	  mode = DIFFERENTIAL;
	} else {
	  mode = SINGLE_ENDED;
	}
	printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = 0-10V  4 = 0-5V: ", channel);
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case '1': gain = BP_10V; break;
  	  case '2': gain = BP_5V; break;
	  case '3': gain = UP_10V; break;
	  case '4': gain = UP_5V; break;
	  default:  gain = BP_10V; break;
	}

	for (i = 0; i < 8; i++) {
	  list[i].range = gain;
          list[i].mode = mode;
	}
	usbADCSetupW_USB1808(udev, list);
	usbAIn_USB1808(udev, value, table_AIN, list);  // read all 8 channels
	for (i = 0; i < 8; i++) {
	  printf("Channel %d  Mode = %#x  Gain = %d value[%d] = %#x Volts = %lf\n",
	     i, list[i].mode, list[i].range, i, value[i], volts_USB1808(gain, value[i]));
	}
        break;
      case 'I':
	printf("Testing USB-1808 Multi-Channel Analog Input Scan.\n");
	usbAInScanStop_USB1808(udev);
	usbAInBulkFlush_USB1808(udev, 5);
	usbAInScanClearFIFO_USB1808(udev);
	printf("Enter 0 for Differential, 1 for Single Ended: ");
	scanf("%hhd", &mode);
	if (mode == 1) {
  	  mode = SINGLE_ENDED;
	} else {
	  mode = DIFFERENTIAL;
	}	  
	printf("enter number of channels (1-8) : ");
	scanf("%d", &nchan);
	if (nchan > 8) break;
        printf("Enter number of scans: ");
        scanf("%d", &nScans);
        printf("Enter number of repeats: ");
        scanf("%d", &repeats);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
        for (channel = 0; channel < nchan; channel++) {
	  printf("Gain Range for channel %d: 1 = +/-10V  2 = +/- 5V  3 = 0-10V  4 = 0-5V: ",channel);
	  while((ch = getchar()) == '\0' || ch == '\n');
	  switch(ch) {
	    case '1': gain = BP_10V; break;
	    case '2': gain = BP_5V; break;
	    case '3': gain = UP_10V; break;
	    case '4': gain = UP_5V; break;
	    default:  gain = BP_10V; break;
	  }
	  list[channel].range = gain;  
	  list[channel].mode = mode;
	  scanQueueAIn[channel] = channel;
	}
	usbADCSetupW_USB1808(udev, list);
	usbAInScanConfigW_USB1808(udev, scanQueueAIn, nchan);
        if ((sdataIn = malloc(4*nchan*nScans)) == NULL) {
	  perror("Can not allocate memory for sdataIn");
	  break;
	}
        for (m = 0; m < repeats; m++) {
	  printf("\n\n---------------------------------------");
	  printf("\nrepeat: %d\n", m);
	  usbAInScanStart_USB1808(udev, nScans, 0, frequency, 0x0);
	  ret = usbAInScanRead_USB1808(udev, nScans, nchan, sdataIn, 2000, 0);
	  printf("Number bytes read = %d  (should be %d)\n", ret, 4*nchan*nScans);
	  for (i = 0; i < nScans; i++) {
	    printf("%6d", i);
	    for (j = 0; j < nchan; j++) {
              gain = list[j].range;
	      k = i*nchan + j;
	      data = rint(sdataIn[k]*table_AIN[j][gain].slope + table_AIN[j][gain].offset);
	      printf(", %#x, %#x, %8.4lf", sdataIn[k], data, volts_USB1808(gain, data));
	    }
	    printf("\n");
	  }
	}
	printf("\n\n---------------------------------------");
	free(sdataIn);
        break;
    case 'C':
      	printf("Testing USB-1808 Analog Input Scan in continuous mode 8 channels\n");
	printf("Hit any key to exit\n");
	printf("Enter desired sampling frequency (greater than 1000): ");
	scanf("%lf", &frequency);
	usbAInScanStop_USB1808(udev);
        nScans = 0;         // for continuous mode
        nchan = 8;          // 8 channels
	gain = BP_10V;
	// mode = DIFFERENTIAL;
	mode = SINGLE_ENDED;

        for (channel = 0; channel < nchan; channel++) {
	  list[channel].range = gain;
	  list[channel].mode = mode;
	  scanQueueAIn[channel] = channel;
	}
	usbADCSetupW_USB1808(udev, list);
	usbAInScanConfigW_USB1808(udev, scanQueueAIn, nchan);

	nread = 128;
	if ((sdataIn = malloc(4*nchan*nread)) == NULL) {
	  perror("Can not allocate memory for sdataIn");
	  break;
	}
	sleep(1);
        i = 0;
        usbAInScanStart_USB1808(udev, nScans, 0, frequency, 0x0);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
	  usbAInScanRead_USB1808(udev, nread, nchan, sdataIn, 2000, CONTINUOUS);
          if (i%100 == 0) {
            printf("Scan = %d\n", i);
	  }
          i++;
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
        usbAInScanStop_USB1808(udev);
	usbAInScanClearFIFO_USB1808(udev);
	usbAInBulkFlush_USB1808(udev, 5);
	free(sdataIn);
        break;
    case 'o':
        printf("Enter voltage for channel 0: ");
	scanf("%lf", &voltage);
	usbAOut_USB1808(udev, 0, voltage, table_AO);
        break;
      case 'O':
	channel = 0;
	printf("Test of Analog Output Scan.\n");
	printf("Hook scope up to VDAC 0\n");
	printf("Enter desired frequency of sine wave [Hz] (1-40Hz): ");
	scanf("%lf", &frequency);
        frequency *= 128.;
	for (i = 0; i < 512; i ++) {
	  voltage = 10.*sin(2.*M_PI*i/128.);
          voltage = (voltage/10.*32768. + 32768.);
	  sdataOut[i] = voltage*table_AO[channel].slope + table_AO[channel].offset;
	}
        usbAOutScanStop_USB1808(udev);
	usbAOutScanClearFIFO_USB1808(udev);
	scanQueueAOut[0] = 0;
	scanQueueAOut[1] = 0;
	scanQueueAOut[2] = 0;
	usbAOutScanConfigW_USB1808(udev, scanQueueAOut, 1);
	usbAOutScanStart_USB1808(udev, 0, 0, frequency,  0x0);
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT|2, (unsigned char *) sdataOut, sizeof(sdataOut), &transferred, 400);
	  if (ret < 0) {
	    perror("error in analog output scan");
	  }
	  ret = libusb_bulk_transfer(udev, LIBUSB_ENDPOINT_OUT|2, (unsigned char *) sdataOut, 0, &transferred, 400);
	  if (ret < 0) {
	    perror("error in analog output scan 2");
	  }
          // printf("ret = %d  status = %#x\n", ret, usbStatus_USB1808(udev));
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbAOutScanStop_USB1808(udev);
	break;
      case 'e':
	// usbDLatchW_USB1808(udev, 0x0);                  // zero out the DIO
	usbAOutScanStop_USB1808(udev);
	usbAOut_USB1808(udev, 0, 0x0, table_AO);
	usbAOut_USB1808(udev, 1, 0x0, table_AO);
        cleanup_USB1808(udev);
        return 0;
      case 'r':
	usbReset_USB1808(udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB1808(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
	status = usbStatus_USB1808(udev);
        printf("Status = %#x\n", status);
	if (status & AIN_SCAN_RUNNING)   printf("    AIN scan running. \n");
	if (status & AIN_SCAN_OVERRUN)   printf("    AIN scan overrun. \n");
	if (status & AOUT_SCAN_RUNNING)  printf("    AOUT scan running. \n");
	if (status & AOUT_SCAN_UNDERRUN) printf("    AOUT scan underrun.\n");
	if (status & AIN_SCAN_DONE)      printf("    AIN scan done.\n");
	if (status & AOUT_SCAN_DONE)     printf("    AOUT scan done.\n");
	if (status & FPGA_CONFIGURED)    printf("    FPGA configured.\n");
	if (status & FPGA_CONFIG_MODE)   printf("    in FPGA configuration mode.\n");
	break;
      case 't':
	printf("Test timers.\n");
	printf("Enter desired frequency: ");
	scanf("%lf", &frequency);
	if (frequency == 0.0) {
	  usbTimerControlW_USB1808(udev, timer, 0x0);  // stop timer
	  break;
	}
	printf("Enter desired duty cycle (0-1.0): ");
	scanf("%lf", &duty_cycle);
	printf("Enter timer (0-1): ");
	scanf("%hhd", &timer);
	usbTimerControlW_USB1808(udev, timer, 0x0);  // stop timer
	usbTimerParametersW_USB1808(udev, timer, frequency, duty_cycle, 0, 0);
	usbTimerControlW_USB1808(udev, timer, TIMER_ENABLE);  // enable timer
	break;
      case 'v':
	version = 0xbeef;
        usbFPGAVersion_USB1808(udev, &version);
	printf("FPGA version %02x.%02x\n", version >> 0x8, version & 0xff);
	break;
      default:
        break;
    }
  }
}
