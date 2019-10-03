/*
 * Copyright (C) 2007  Warren Jasper
 * All rights reserved.
 */


/***************************************************************************
 *
 *  test-das1000.c
 *
 *  This program is used to test the PCI-DAS1000 Analog to Digital
 *  Linux loadable module.
 *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
//#include <asm/page.h>
#include <sys/mman.h>
#include "pci-das1000.h"


/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/das1000/ad0_0";
int  ADC_Gain     = UP_10_00V;
int  ADC_Mode = ADC_SOFT_CONVERT;
int  Count    = 1;
int  NoStop   = 0;
int  NoBlock  = 0;
int  Print    = 1;
int  freq_A2D = 5000;
int  Status;

int fdADC;                   /* A/D file descriptors */
int fdDIOA, fdDIOB, fdDIOC;  /* DIO file descriptors */

void Usage( void )
{
  fprintf(stderr, "\n");
  fprintf(stderr, "Usage: adcread \'options\'\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr,"    [-dev /dev/das1000/ad0_##]  - Specify device file.\n");
  fprintf(stderr,"    [-range (0-8)]              - Voltage input range (see manual)\n");
  fprintf(stderr,"    [-ct ##]                    - Number of samples to read\n");
  fprintf(stderr,"    [-soft]                     - Use software triggers\n");
  fprintf(stderr,"    [-extern]                   - Use extnernal clock triggers\n");
  fprintf(stderr,"    [-pacer]                    - Use pacer clock triggers\n");
  fprintf(stderr,"    [-noblock]                  - Open in non blocking mode\n");
  fprintf(stderr,"    [-noprint]                  - Don't print samples\n");
  fprintf(stderr,"    [-nostop]                   - Sample forever\n");
  fprintf(stderr,"    [-A2Dfreq ##]               - Sample A/D Frequency used with pacer triggers\n");
  fprintf(stderr,"\n");
  exit(1);
}

void DoCommandLine(int argc, char **argv)
{
  int i = 1;

  while (i < argc) {
    if (strcmp(argv[i], "-dev") == 0) {
      i++;
      if (i == argc)
        Usage();
      else 
        DevName = argv[i];
    } else if (strcmp(argv[i], "-range") == 0) {
      i++;
      if (i == argc)
        Usage();
      else 
        ADC_Gain = atoi(argv[i]);
    } else if (strcmp(argv[i], "-ct") == 0) {
      i++;
      if (i == argc)
        Usage();
      else 
        Count = atoi(argv[i]);
    } else if (strcmp(argv[i], "-soft") == 0) {
      ADC_Mode = ADC_SOFT_CONVERT;
    } else if (strcmp(argv[i], "-pacer") == 0) {
      ADC_Mode = ADC_PACER_CLOCK;
    } else if (strcmp(argv[i], "-noblock") == 0) {
      NoBlock = 1;
    } else if (strcmp(argv[i], "-noprint") == 0) {
      Print = 0;
    } else if (strcmp(argv[i], "-nostop") == 0) {
      NoStop = 1;
    } else if (strcmp(argv[i], "-A2Dfreq") == 0) {
      i++;
      if (i == argc)
        Usage();
      else
        freq_A2D = atoi(argv[i]);
    } else {
      Usage();
    }
    i++;
  }
}

void DoOpenDevices()
{
  char str[80];

  if ( NoBlock ) {
    if (( fdADC = open(DevName, ADC_Mode | O_NONBLOCK)) < 0 ) {
      perror(str);
      printf("error opening device %s in non blocking mode.\n", DevName);
      exit(2);
    }
  } else {
    if (( fdADC = open(DevName, ADC_Mode)) < 0 ) {
      perror(str);
      printf("error opening device %s\n", DevName);
      exit(2);
    }
  }

  strcpy(str, "/dev/das1000/dio0_0A");
  if (( fdDIOA = open(str, O_RDWR)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/das1000/dio0_0B");
  if (( fdDIOB = open(str, O_RDWR)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/das1000/dio0_0C");
  if (( fdDIOC = open(str, O_RDWR)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  ioctl(fdADC, ADC_SET_GAINS, UP_10_00V);
}

void testDIO()
{
  unsigned short value;
  unsigned char bReg;
  char str[80];

  printf("***********************************************************\n");
  printf("* The program writes values on port A and reads them on   *\n");
  printf("* port B (8 bits), and writes on the low nibble of port C *\n");
  printf("* (4 bits) and reads them on the high nibble of port C.   *\n");
  printf("***********************************************************\n\n");

  ioctl(fdDIOA, DIO_SET_DIRECTION, PORT_OUTPUT);
  ioctl(fdDIOB, DIO_SET_DIRECTION, PORT_INPUT);
  ioctl(fdDIOC, DIO_SET_DIRECTION, LOW_PORT_OUTPUT);
  ioctl(fdDIOC, DIO_SET_DIRECTION, HIGH_PORT_INPUT);

  while (1) {
      printf("Enter value for port A in hex: ");
      scanf("%hx", &value);
      write(fdDIOA, &value, 1);
      read(fdDIOB, &bReg, 1);
      printf("Port B value = %#hx\n", (short int) bReg);
      printf("Enter value for port C in hex: ");
      scanf("%hx", &value);
      value &= 0x0f;          /* mask off high nibble */
      write(fdDIOC, &value, 1);
      read(fdDIOC, &bReg, 1);
      bReg >>= 4;
      printf("Port C value = %#hx\n", (short int) bReg);

      printf("\n Continue? ");
      scanf("%s", str);
      if (str[0] == 'n' || str[0] == 'N') return;
  }
}

float volts( int gain, unsigned short value )
{
  float volt;
  
  switch( gain ) {
    case BP_10_00V:
      volt = (20.0/4096.)*(value - 2048);
      break;
    case BP_5_00V:
      volt = (10.0/4096.)*(value - 2048);
      break;
    case UP_10_00V:
      volt = (10.0/4096.)*value;
      break;
    case UP_5_00V:
      volt = (5.0/4096.)*value;
      break;
  }
  return volt;
}

void testADC()
{
  unsigned short value[16384];
  unsigned short *readBuff;
  char str[80];
  int i, j;
  int bytesRead;
  double mean, sd;
  unsigned short max, min;

  ioctl(fdADC, ADC_SET_PACER_FREQ, freq_A2D);

  while (1) {
    if (NoBlock) {
      if((readBuff = mmap(0, Count*2, PROT_READ, MAP_PRIVATE, fdADC, 0*getpagesize()))
       == (unsigned short *) MAP_FAILED) {
        printf("Test Failed: Mmap call failed \n");	
        sleep(3);
        return;
      } else{
        printf("Test Passed: Succesfully mmaped %d bytes\n", Count*2); 
      }

      bytesRead = read(fdADC, value, Count);
      printf("noblock set: looping on ADC_NBIO_COMPLETE.\n");

      i = 0;
      for ( j = 0; j < 5; j++ ) {
	ioctl(fdADC, ADC_NBIO_COMPLETE, &i);
	if ( i == 1 ) {
	  printf("blocking read complete j = %d !\n", j);
	  break;
	} else {
	  sleep(1);
	}
      }
      bytesRead = read(fdADC, value, Count);

      for ( i = 0; i < Count; i++ ) {
        printf("Value number %d is %#x  %#x  Volts = %f\n", i+1, readBuff[i], value[i], volts(ADC_Gain,value[i]));
      }

      printf("%d samples read\n", bytesRead);
      printf("\n*******  Statistics ******\n");
      mean = 0.0;
      max = 0x0;
      min = 0xffff;

      for ( i = 0; i < Count; i++ ) {
	mean += readBuff[i];
	if ( readBuff[i] > max ) max = readBuff[i];
	if ( readBuff[i] < min ) min = readBuff[i];
      }
      mean /= Count;
      sd = 0.0;
      for ( i = 0; i < Count; i++ ) {
	sd += (value[i] - mean)*(value[i] - mean);
      }
      sd = sqrt( sd / (Count - 1));
      printf("Mean = %f,  Standard deviation = %f.\n", mean, sd);
      printf("Max = %#x,  Min = %#x,  Avg = %#x\n", max, min, (unsigned short) (mean+0.5));
    } else {
      bytesRead = read(fdADC, value, Count);

      if ( bytesRead != Count ) {
	printf("testADC: Error on read() \n");
      }

      for ( i = 0; i < Count; i++ ) {
        printf("Value number %d is %#x  Volts = %f\n", i+1, value[i], volts(ADC_Gain,value[i]));
      }

      printf("%d samples read\n", bytesRead);
      printf("\n*******  Statistics ******\n");
      mean = 0.0;
      max = 0x0;
      min = 0xffff;

      for ( i = 0; i < Count; i++ ) {
	mean += value[i];
	if ( value[i] > max ) max = value[i];
	if ( value[i] < min ) min = value[i];
      }
      mean /= Count;
      sd = 0.0;
      for ( i = 0; i < Count; i++ ) {
	sd += (value[i] - mean)*(value[i] - mean);
      }
      sd = sqrt( sd / (Count - 1));
      printf("Mean = %f,  Standard deviation = %f.\n", mean, sd);
      printf("Max = %#x,  Min = %#x,  Avg = %#x\n", max, min, (unsigned short) (mean+0.5));
    }

    printf("\n\n Continue? ");
    scanf("%s", str);
    if (str[0] == 'n' || str[0] == 'N') return;
  }
}

void ChangeADCGains()
{
    int choice;

    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("  1.   Bipolar +/- 10.00V.\n");
    printf("  2.   Bipolar +/-  5.00V.\n");
    printf("  3.   Bipolar +/-  2.50V.\n");
    printf("  4.   Bipolar +/-  1.25V.\n");
    printf("  5.   Unipolar     0-10.00V.\n");
    printf("  6.   Unipolar     0-5.00V.\n");
    printf("  7.   Unipolar     0-2.50V.\n");
    printf("  8.   Unipolar     0-1.25V\n");
    printf("\nOption: ");
    scanf("%d", &choice);
    switch(choice){
    case 1:
      ioctl(fdADC, ADC_SET_GAINS, BP_10_00V);
      ADC_Gain = BP_10_00V;
      break;
    case 2:
      ioctl(fdADC, ADC_SET_GAINS, BP_5_00V);
      ADC_Gain = BP_5_00V;
      break;
    case 3:
      ioctl(fdADC, ADC_SET_GAINS, BP_2_50V);
      ADC_Gain = BP_2_50V;
      break;
    case 4:
      ioctl(fdADC, ADC_SET_GAINS, BP_1_25V);
      ADC_Gain = BP_1_25V;
      break;
    case 5:
      ioctl(fdADC, ADC_SET_GAINS, UP_10_00V);
      ADC_Gain = UP_10_00V;
      break;
    case 6:
      ioctl(fdADC, ADC_SET_GAINS, UP_5_00V);
      ADC_Gain = UP_5_00V;
      break;
    case 7:
      ioctl(fdADC, ADC_SET_GAINS, UP_2_50V);
      ADC_Gain = UP_2_50V;
      break;
    case 8:
      ioctl(fdADC, ADC_SET_GAINS, UP_1_25V);
      ADC_Gain = UP_1_25V;
      break;
    default:
      printf("Illegal option.\n");
    }
}


void SetADPacer()
{
  printf("***********************************************************\n");
  printf("* This program sets the A/D Pacer Frequency which can be  *\n");
  printf("* seen with a scope on Pin 95                             *\n");
  printf("***********************************************************\n\n");
  
  printf("Input the desired AD Pacer Frequency (Hz): ");
  scanf("%d", &freq_A2D);
  ioctl(fdADC, ADC_STOP_PACER, 0);
  ioctl(fdADC, ADC_SET_PACER_FREQ, freq_A2D);
  ioctl(fdADC, ADC_GET_PACER_FREQ, &freq_A2D);
  ioctl(fdADC, ADC_START_PACER, 0);
  printf("\nThe AD Pacer Frequency is set to %d\n", freq_A2D);
  sleep(3);
}


void Domenu()
{
  int choice;

  while(1) {
    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("    1. Test Digital I/O.\n");
    printf("    2. Test Analog to Digital Converter.\n");
    printf("    3. Change A/D Gains.\n");
    printf("    4. Set A/D Internal Pacer.\n");
    printf("    5. Exit.\n");
    printf("\nOption: ");
   
    scanf("%d", &choice);
    switch(choice){
      case 1:  
        testDIO();
        break;
      case 2:  
        testADC();
        break;
      case 3:
        ChangeADCGains();
        break;
      case 4:
        SetADPacer();
        break;
      case 5:  
        return;
        break;
      default:
        break;
    }
  }    
}

int main(int argc, char **argv)
{
  DoCommandLine(argc, argv);
  DoOpenDevices();
  Domenu();
  close(fdDIOA);
  close(fdDIOB);
  close(fdDIOC);
  close(fdADC);
  return(1);
}

