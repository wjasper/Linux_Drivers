/*
 *
 *  Copyright (c) 2004-2007 Warren Jasper <wjasper@ncsu.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>

#include "pmd.h"
#include "minilab-1008.h"

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
  signed short svalue;
  int temp, i, j;
  int ch;
  int rate;
  int16_t *sdata;
  uint16_t value;
  uint16_t count;
  uint8_t gains[8];
  uint8_t options;
  uint8_t input, pin = 0, channel, gain;
  int ret;

  hid_device*  hid = 0x0;

  // Debug information.  Delete when not needed    
  //  hid_set_debug(HID_DEBUG_ALL);
  //  hid_set_debug_stream(stderr);
  //  hid_set_usb_debug(2);

start:
  
  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, MINILAB1008_PID, NULL)) > 0) {
    printf("miniLAB 1008 Device is found!\n");
  } else {
    fprintf(stderr, "miniLAB 1008 not found.\n");
    exit(1);
  }

  /* config mask 0x01 means all inputs */
  usbDConfigPort_miniLAB1008(hid, DIO_PORTB, DIO_DIR_IN);
  usbDConfigPort_miniLAB1008(hid, DIO_PORTA, DIO_DIR_OUT);
  usbDConfigPort_miniLAB1008(hid, DIO_PORTCL, DIO_DIR_IN);
  usbDConfigPort_miniLAB1008(hid, DIO_PORTCH, DIO_DIR_OUT);

  /* Note:  I believe the direction setting for the Auxport (DIO0-DIO3)
     is opposite than for PORTA-PORTC.  This is not documented.
  */
  usbDConfigPort_miniLAB1008(hid, DIO_AUXPORT, 0xf); // all 4 bits to output
  usbDOut_miniLAB1008(hid, DIO_PORTA, 0x0);
  
  while(1) {
    printf("\nminiLAB 1008 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 's' to set user id\n");
    printf("Hit 'g' to get user id\n");
    printf("Hit 'c' to test counter \n");
    printf("Hit 'd' to test digital I/O \n");
    printf("Hit 't' to test digital bit I/O\n");
    printf("Hit 'o' to test analog output\n");
    printf("Hit 'i' to test analog input (differential)\n");
    printf("Hit 'I' to test analog input (single ended)\n");
    printf("Hin 'n' to test analog input scan\n");
    printf("Hit 'r' to reset the device.\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    
    switch(ch) {
    case 'b': /* test to see if led blinks */
      usbBlink_miniLAB1008(hid);
      break;
    case 's':
      printf("enter a user id :");
      ret = scanf("%d",&temp);
      usbSetID_miniLAB1008(hid, temp);
      printf("User ID is set to %d\n", usbGetID_miniLAB1008(hid));      
      break;
    case 'g':
      printf("User ID = %d\n", usbGetID_miniLAB1008(hid));      
      break;
    case 'o': /* test the analog output */
      printf("Testing the analog output...\n");
      printf("Enter channel [0-1] => (pin 13-14):");
      ret = scanf("%d", &temp);
      if (ret < 0) {
	perror("Error in scanf.");
      }
      channel = (uint8_t) temp;
      for ( j = 0; j < 5; j++ ) {
        for ( value = 0; value < 0x3ff; value++ ) {
	  usbAOut_miniLAB1008(hid, channel, value);
	}
      }
      break;
    case 'c':
      printf("connect DIO and CTR\n");
      usbInitCounter_miniLAB1008(hid);
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
	sleep(1);
        usbDOut_miniLAB1008(hid, DIO_AUXPORT, 0x1);
        usbDOut_miniLAB1008(hid, DIO_AUXPORT, 0x0);
	printf("Counter = %d\n",usbReadCounter_miniLAB1008(hid));
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    case 'd':
      printf("\nTesting Digital I/O....\n");
      printf("On the DB37 Connector Pin-out: Connect pins 30 through 37 <=>  3 through 10.\n");
      printf("i.e.: A0 <=> B0, A1 <=> B1, ... A7 <=> B7 \n");
      do {
        printf("Enter a byte number [0-0xff]: " );
        ret = scanf("%x", &temp);
	if (ret < 0) {
  	  perror("Error in scanf.");
	}
        usbDOut_miniLAB1008(hid, DIO_PORTA, (uint8_t)temp);
        usbDIn_miniLAB1008(hid, DIO_PORTB, &input);
        usbDOut_miniLAB1008(hid, DIO_AUXPORT, (uint8_t)temp);
        printf("The number you entered = %#x\n",input);
      } while (toContinue());
      break;
    case 't':
      //reset the pin values
      usbDOut_miniLAB1008(hid, DIO_PORTA, 0x0);
      printf("\nTesting Bit  I/O....\n");
      printf("Enter a bit value for output (0 | 1) : ");
      ret = scanf("%d", &temp);
      if (ret < 0) {
	perror("Error in scanf.");
      }
      input = (uint8_t) temp;
      printf("Select the Pin in port A [0-7] :");
      ret = scanf("%d", &temp);
      if (ret < 0) {
	perror("Error in scanf.");
      }
      pin = (uint8_t) temp;
      usbDBitOut_miniLAB1008(hid, DIO_PORTA, pin, input);
      usbDIn_miniLAB1008(hid, DIO_PORTB, &input);
      printf("The number you entered 2^%d = %d \n",temp, input);
      break;
    case 'n':
      printf("\t\t1. +/- 20.V\n");
      printf("\t\t2. +/- 10.V\n");
      printf("\t\t3. +/- 5.V\n");
      printf("\t\t4. +/- 4.V\n");
      printf("\t\t5. +/- 2.5V\n");
      printf("\t\t6. +/- 2.0V\n");
      printf("\t\t7. +/- 1.25V\n");
      printf("\t\t8. +/- 1.0V\n");
      printf("Select gain: [1-8]\n");
      ret = scanf("%d", &temp);
      if (ret < 0) {
	perror("Error in scanf.");
      }
      switch(temp) {
        case 1: gains[0] = BP_20_00V;
                break;
        case 2: gains[0] = BP_10_00V;
                break;
        case 3: gains[0] = BP_5_00V;
                break;
        case 4: gains[0] = BP_4_00V;
                break;
        case 5: gains[0] = BP_2_50V;
                break;
        case 6: gains[0] = BP_2_00V;
                break;
        case 7: gains[0] = BP_1_25V;
                break;
        case 8: gains[0] = BP_1_00V;
                break;
        default:
                break;
      }
      for (i = 1; i < 8; i++) {
	gains[i] = gains[0];
      }
      //      printf("Select sampling rate [Hz]: ");
      //      ret = scanf("%d", &rate);
      printf("Select number of samples: ");
      scanf("%hd", &count);
      rate = 100;
      options = AIN_EXECUTION;
      sdata = (short*) malloc(2048);
      usbAInScan_miniLAB1008(hid, count, rate, 0, 3, options, sdata, gains);
      for ( i = 0; i < count/4; i++ ) {
	printf("scan %d: ", i);
	for (j = 0; j < 4; j++) {
	  printf("%.2fV ", volts_LS(gain, sdata[4*i+j]));
	}
	printf("\n");
      }
      free(sdata);
      break;
    case 'i':
      printf("Select channel [0-3]: ");
      ret = scanf("%d", &temp);
      if (ret < 0) {
	perror("Error in scanf.");
      }
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
      ret = scanf("%d", &temp);
      if (ret < 0) {
	perror("Error in scanf.");
      }
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
	        gain = BP_10_00V;
                break;
      }
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
	svalue = usbAIn_miniLAB1008(hid, channel, gain);
	sleep(1);
	printf("Channel: %d: value = %#hx, %.2fV\n",
	       channel, svalue, volts_LS(gain, svalue));
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    case 'r':
      usbReset_miniLAB1008(hid);
      sleep(2);
      hid_close(hid);
      goto start;       
    case 'e':
      hid_close(hid);
      hid_exit();
      exit(0);
      break;
    case 'I':
      printf("Single Ended test.\n");
      printf("Select channel [0-7]: ");
      ret = scanf("%d", &temp);
      if (ret < 0) {
	perror("Error in scanf.");
      }
      if ( temp < 0 || temp > 7 ) break;
      channel = (uint8_t) temp;
      gain = SE_10_00V;
      printf("Range +/- 10V only for single ended\n");
      flag = fcntl(fileno(stdin), F_GETFL);
      fcntl(0, F_SETFL, flag | O_NONBLOCK);
      do {
	sleep(1);
	svalue = usbAIn_miniLAB1008(hid, channel, gain);
	printf("Channel: %d: value = %#hx, %.2fV\n",
	       channel, svalue, volts_LS(gain, svalue));
      } while (!isalpha(getchar()));
      fcntl(fileno(stdin), F_SETFL, flag);
      break;
    default:
      break;
    }
  }
}
  



