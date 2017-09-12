/*
 *
 *  Copyright (c) 2014 Warren J. Jasper <wjasper@ncsu.edu>
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
#include <sys/types.h>
#include <asm/types.h>

#include "pmd.h"
#include "usb-2020.h"

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
  usbDevice2020 device;
  struct tm calDate;
  double frequency;
  float temperature;
  int ch;
  int i;
  int flag;
  int nSamples = 0;
  uint8_t input;
  int temp, ret;
  uint8_t options;
  char serial[9];
  uint16_t version;
  uint16_t status;
  uint16_t value;
  uint16_t dataAIn[512*20];  // holds 16 bit unsigned analog input data, must be multiple of 256.
  uint16_t *dataAInBurst;    // pointer for BURSTIO data
  uint8_t mode, gain, channel;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((device.udev = usb_device_find_USB_MCC(USB2020_PID, NULL))) {
    printf("Success, found a USB-2020.\n");
  } else {
    printf("Failure, did not find a USB-2020!\n");
    return 0;
  }
  // some initialization
  usbInit_USB2020(device.udev);

  //print out the wMaxPacketSize.  Should be 512
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(device.udev,0));

  usbBuildGainTable_USB2020(device.udev, device.table_AIn);
  for (i = 0; i < NGAINS_2020; i++) {
    printf("Gain: %d   Slope = %f   Offset = %f\n", i, device.table_AIn[i][0], device.table_AIn[i][1]);
  }

  usbCalDate_USB2020(device.udev, &calDate);
  printf("\n");
  printf("MFG Calibration date = %s\n", asctime(&calDate));

  while(1) {
    printf("\nUSB 2020 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'B' for BURSTIO unsing onboard DDR RAM\n");
    printf("Hit 'd' to test digital IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'C' to test Continuous AIn Scan @ 1000 Hz.\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 'T' to get temperature\n");
    printf("Hit 'v' to get version numbers\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
        usbBlink_USB2020(device.udev, options);
	break;
      case 'd':
        printf("\nTesting Digital I/O...\n");
	printf("connect pins  [0-3] <--> [4-7]\n");
	usbDTristateW_USB2020(device.udev,0xf0);
	printf("Digital Port 1 Tristate Register = %#x\n", usbDTristateR_USB2020(device.udev));
	do {
          printf("Enter a  number [0-0xf]: " );
          scanf("%x", &temp);
          temp = temp;
          usbDLatchW_USB2020(device.udev, (uint16_t)temp);
	  temp = usbDLatchR_USB2020(device.udev);
          input = (usbDPort_USB2020(device.udev) >> 4) & 0xf;
          input = ~input & 0xf;
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, temp);
	  for (i = 0; i < 4; i++) {
	    printf("Bit %d = %d\n", i, (temp>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'e':
        cleanup_USB2020(device.udev);
        return 0;
      case 'i':
	printf("Input channel [0-1]: ");
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
	mode = (LAST_CHANNEL| SINGLE_ENDED);
	device.list[0].range = gain;
        device.list[0].mode = mode;
	device.list[0].channel = channel;
	usbAInConfig_USB2020(device.udev, device.list);
	for (i = 0; i < 20; i++) {
	  value = usbAIn_USB2020(device.udev, channel);
	  value = rint(value*device.table_AIn[gain][0] + device.table_AIn[gain][1]);
	  printf("Channel %d  Mode = %#x  Gain = %d Sample[%d] = %#x Volts = %lf\n",
		 device.list[0].channel, device.list[0].mode, device.list[0].range, i, value, volts_USB2020(gain, value));
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing USB-2020 Analog Input Scan.\n");
	usbAInScanStop_USB2020(device.udev);
	usbAInScanClearFIFO_USB2020(device.udev);
        printf("Enter number of samples (less than 5000): ");
        scanf("%d", &nSamples);
	printf("Input channel [0-1]: ");
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
        device.list[0].range = gain;
        device.list[0].mode = mode;
        device.list[0].channel = channel;
	usbAInConfig_USB2020(device.udev, device.list);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
        options = 0x0;
        for (i = 0; i < nSamples; i++) {
          dataAIn[i] = 0xbeef;
	}
	usbAInScanStart_USB2020(device.udev, nSamples, 0, frequency, nSamples-1, options);
	ret = usbAInScanRead_USB2020(device.udev, nSamples, 1, &dataAIn[0], 2000, 0);
	printf("Number samples read = %d\n", ret/2);
	for (i = 0; i < nSamples; i++) {
          dataAIn[i] &= 0xfff;
	  dataAIn[i] = rint(dataAIn[i]*device.table_AIn[gain][0] + device.table_AIn[gain][1]);
	  printf("Channel %d  Mode = %d  Gain = %d Sample[%d] = %#x Volts = %lf\n", channel,
		 mode, gain, i, dataAIn[i], volts_USB2020(gain, dataAIn[i]));
	}
	usbAInScanStop_USB2020(device.udev);
	usbAInScanClearFIFO_USB2020(device.udev);
        break;
      case 'B':
	printf("Testing USB-2020 Analog Input Scan BURSTIO mode.\n");
	usbAInScanStop_USB2020(device.udev);
	usbAInScanClearFIFO_USB2020(device.udev);
        printf("Enter number of samples (greater than or equal to 256, less than 64 MB and a multiple of 256): ");
        scanf("%d", &nSamples);
	dataAInBurst = malloc(2*nSamples);
	printf("Input channel [0-1]: ");
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
        device.list[0].range = gain;
        device.list[0].mode = mode;
        device.list[0].channel = channel;
	usbAInConfig_USB2020(device.udev, device.list);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
        options = DDR_RAM;
        for (i = 0; i < nSamples; i++) {
          dataAInBurst[i] = 0xbeef;
	}
	usbAInScanStart_USB2020(device.udev, nSamples, 0, frequency, nSamples-1, options);
	ret = usbAInScanRead_USB2020(device.udev, nSamples, 1, &dataAInBurst[0], 2000, options);
	printf("Number samples read = %d\n", ret/2);
	for (i = 0; i < nSamples; i++) {
          dataAInBurst[i] &= 0xfff;
	  dataAInBurst[i] = rint(dataAInBurst[i]*device.table_AIn[gain][0] + device.table_AIn[gain][1]);
	  printf("Channel %d  Mode = %d  Gain = %d Sample[%d] = %#x Volts = %lf\n", channel,
		 mode, gain, i, dataAInBurst[i], volts_USB2020(gain, dataAInBurst[i]));
	}
	usbAInScanStop_USB2020(device.udev);
	usbAInScanClearFIFO_USB2020(device.udev);
	free(dataAInBurst);
        break;
      case 'C':
	printf("Testing USB-2020 Analog Input Scan in continuous mode.\n");
        printf("Hit any key to exit\n");
   	usbAInScanStop_USB2020(device.udev);
	usbAInScanClearFIFO_USB2020(device.udev);
	printf("Enter desired sampling frequency (greater than 1000): ");
	scanf("%lf", &frequency);
        nSamples = 0;       // put in continuous mode
        channel = 0;        // use channel 0
        gain = BP_10V;      // set gain to +/- 10 V
	mode = (LAST_CHANNEL | SINGLE_ENDED);
        device.list[0].range = gain;
        device.list[0].mode = mode;
        device.list[0].channel = channel;
	usbAInConfig_USB2020(device.udev, device.list);
        options = 0x0;
	usbAInScanStart_USB2020(device.udev, nSamples, 0, frequency, nSamples-1, options);
        i = 0;
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
	  i++;
	  usbAInScanRead_USB2020(device.udev, 1024, 1, &dataAIn[0], 2000, CONTINUOUS);
          if (i%((int)frequency/1024) == 0) {
	    printf("Scan = %d.  Samples read = %d\n", i, 1024*i);
	  }
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	usbAInScanStop_USB2020(device.udev);
        break;
      case 'r':
	usbReset_USB2020(device.udev);
	return 0;
	break;
      case 's':
        usbGetSerialNumber_USB2020(device.udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
	status = usbStatus_USB2020(device.udev);
        printf("Status = %#x\n", status);
	if (status & AIN_SCAN_RUNNING) printf("AIn pacer running.\n");
	if (status & AIN_SCAN_OVERRUN) printf("AIn scan overrun.\n");
	if (status & AIN_SCAN_DONE)    printf("AIn scan done.\n");
	if (status & FPGA_CONFIGURED)  printf("FPGA is configured.\n");
	if (status & FPGA_CONFIG_MODE) printf("In FPGA config mode.\n");
	break;
      case 'T':
        usbTemperature_USB2020(device.udev, &temperature);
	printf("Temperature = %.2f deg C  or  %.2f deg F \n", temperature, 9.0/5.0*temperature + 32.);
	break;
      case 'v':
	version = 0xbeef;
        usbFPGAVersion_USB2020(device.udev, &version);
	printf("FPGA version %02x.%02x\n", version >> 0x8, version & 0xff);
	break;
    default:
        break;
    }
  }
}

