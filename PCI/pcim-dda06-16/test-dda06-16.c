/*
 * Copyright (C) 2020   Warren J. Jasper
 * All rights reserved.
 *
 */

/***************************************************************************
 *
 *  test-dda06_16.c
 *
 *  This program is used to test the PCIM-DDA06-16 
 *  Linux loadable module(dda06_16).
 *
 ***************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include "pcim-dda06-16.h"

#define FS  (65536)

/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/dda06-16/da0_0";
char DevNameIO[80];
int  Count    = 1;
int  NoStop   = 0;
int  Mode     = 0;
int  Status;

static int fd_A, fd_B, fd_C;
static int fd_dac[6];    // we are only going to use 2 for this test.

int  DAC_Gain[6]  = {BP_5_0V, BP_5_0V, BP_5_0V, BP_5_0V, BP_5_0V, BP_5_0V};  // HW selectable onboard only

/***************************************************************************
 *
 *  Show user how the command line is used.
 *
 ***************************************************************************/

void Usage( void )
{
  fprintf(stderr, "\n");
  fprintf(stderr, "Usage: test-dda06X-16 \'options\'\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "    [-dev /dev/dda06_16/dio0_{A,B,C}]  - Specify device file.\n");
  fprintf(stderr, "    [-ct ##]                           - Number of samples to write\n");
  fprintf(stderr, "    [-nostop]                          - Write forever\n");
  fprintf(stderr, "\n");
  exit(1);
}

/***************************************************************************
 *
 *  Process command line switches
 *
 ***************************************************************************/

void DoCommandLine(int argc, char **argv)
{
  int i;
  i = 1;

  while (i < argc) {
    if (strcmp(argv[i], "-dev") == 0) {
      i++;
      if (i == argc)
	Usage();
      else 
	DevName = argv[i];
    } else if (strcmp(argv[i], "-ct") == 0) {
      i++;
      if (i == argc)
	Usage();
      else 
	Count = atoi(argv[i]);
    } else if (strcmp(argv[i], "-nostop") == 0) {
      NoStop = 1;
    } else {
      Usage();
    }
    i++;
  }
}

void DoOpenDevices()
{
  strcpy(DevNameIO, "/dev/dda06-16/da0_0");
  if ((fd_dac[0] = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }

  strcpy(DevNameIO, "/dev/dda06-16/da0_1");
  if ((fd_dac[1] = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }

  /* open the dio */
  strcpy(DevNameIO, "/dev/dda06-16/dio0_A");
  if ((fd_A = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dda06-16/dio0_B");
  if ((fd_B = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dda06-16/dio0_C");
  if ((fd_C = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
}

void testDIO()
{
  unsigned char  bReg;
  unsigned short value;
  char str[80];

  printf("***********************************************************\n");
  printf("* The program writes values on port A and reads them on   *\n");
  printf("* port B (8 bits), and writes on the low nibble of port C *\n");
  printf("* (4 bits) and reads them on the high nibble of port C.   *\n");
  printf("***********************************************************\n\n");

  while(1) {
    if (ioctl(fd_A, DIO_SET_DIRECTION, PORT_OUTPUT) < 0)
      perror("ioctl  Port A failed");
    if (ioctl(fd_B, DIO_SET_DIRECTION, PORT_INPUT) < 0)
      perror("ioctl Port B failed");
    if (ioctl(fd_C, DIO_SET_DIRECTION, LOW_PORT_OUTPUT) < 0)
      perror("ioctl Port CL failed");
    if (ioctl(fd_C, DIO_SET_DIRECTION, HIGH_PORT_INPUT) < 0)
      perror("ioctl Port CH failed");

    printf("Enter value for port A in hex: ");
    scanf("%hx", &value);
    write(fd_A, &value, 1);
    read(fd_A, &bReg, 1);
    printf("Port A value = %#x\n", bReg);
    read(fd_B, &bReg, 1);
    printf("Port B value = %#hx\n", bReg);

    // Now reverse the process
    if (ioctl(fd_A, DIO_SET_DIRECTION, PORT_INPUT) < 0)
      perror("ioctl Port A failed");
    if (ioctl(fd_B, DIO_SET_DIRECTION, PORT_OUTPUT) < 0)
      perror("ioctl Port B failed");
    write(fd_B, &value, 1);
    read(fd_B, &bReg, 1);
    printf("Port B value = %#x\n", bReg);
    read(fd_A, &bReg, 1);
    printf("Port A value = %#hx\n\n", bReg);

    printf("Enter value for port C in hex: ");
    scanf("%hx", &value);
    value &= 0x0f;          /* mask off high nibble */
    write(fd_C, &value, 1);
    read(fd_C, &bReg, 1);
    bReg >>= 4;
    printf("Port C value = %#hx\n", bReg);
    // Now reverse the process
    if (ioctl(fd_C, DIO_SET_DIRECTION, LOW_PORT_INPUT) < 0)
      perror("ioctl Port CL failed");
    if (ioctl(fd_C, DIO_SET_DIRECTION, HIGH_PORT_OUTPUT) < 0)
      perror("ioctl Port CH failed");
    value <<= 4;
    write(fd_C, &value, 1);
    read(fd_C, &bReg, 1);
    bReg &= 0xf;
    printf("Port C value = %#hx\n\n", bReg);

    printf("\n Continue? ");
    scanf("%s", str);
    if (str[0] == 'n' || str[0] == 'N') return;
  }
}

float volts( int gain, unsigned short value )
{
  float volt;
  
  switch( gain ) {
    case BP_5_0V:
      printf("gain set to BP_5_0V\n");
      volt = (5.0/FS)*(value - FS/2);
      break;
    case BP_10_0V:
      printf("gain set to BP_10_0V\n");
      volt = (10.0/FS)*(value - FS/2);
      break;
    case UP_5_0V:
      printf("gain set to UP_5_0V\n");
      volt = (5.0/FS)*(value);
      break;
    case UP_10_0V:
      printf("gain set to UP_10_0V\n");
      volt = (10.0/FS)*value;
      break;
  }
  return volt;
}

void ChangeDACGains()
{
  int choice;
  int channel = 0;
  printf("Gains can only be change via DIP switches located on the board.  The default is Bipolar +/- 5V\n");
  printf("Use this function to tell the driver the default settings for each setting\n");
  printf("Enter desired channel: ");
  scanf("%d", &channel);
  printf("Select from the following choices:\n");
  printf("  1.   Bipolar  +/- 10.00V.\n");
  printf("  2.   Bipolar  +/-  5.00V.\n");
  printf("  3.   Unipolar 0 - 10.00V.\n");
  printf("  4.   Unipolar 0 -  5.00V.\n");

  printf("\nOption: ");
  scanf("%d", &choice);
  switch(choice) {
  case 1:
    ioctl(fd_dac[channel], DAC_SET_GAINS, BP_10_0V);
    DAC_Gain[channel] = BP_10_0V;
    break;
  case 2:
    ioctl(fd_dac[channel], DAC_SET_GAINS, BP_5_0V);
    DAC_Gain[channel] = BP_5_0V;
    break;
  case 3:
    ioctl(fd_dac[channel], DAC_SET_GAINS, UP_10_0V);
    DAC_Gain[channel] = UP_10_0V;
    break;
  case 4:
    ioctl(fd_dac[channel], DAC_SET_GAINS, UP_5_0V);
    DAC_Gain[channel] = UP_5_0V;
    break;
  default:
    printf("Illegal option.\n");
  }
  ioctl(fd_dac[channel], DAC_SET_GAINS, DAC_Gain[channel]);
  printf("Gain Set to %#x\n\n", DAC_Gain[channel]);
  sleep(2);
  return;
}

void testDAC2()
{
  int i, j;
  double x;
  unsigned short value = 0;

  for (j = 0; j < 120; j++) {
    for (i = 0; i < FS; i++) {
      //      x = 5 + 2.5*cos(2*M_PI*i/FS);
      //      value = x*FS/10.;
      //      write(fd_dac[1], &value, 1);  // sine wave
      //value = 0xffff;
      //write(fd_dac[0], &value, 1);    // square wave
      //usleep(1000);
      //value = 0x0;
      value = (unsigned short) i*16;   // triangular wave
      write(fd_dac[0], &value, 1);    
      usleep(1000);
    }
  }
}

void testDAC()
{
  unsigned short value;
  char str[80];

  while (1) {
    printf("Enter value for DAC0 [0-0xffff]: ");
    scanf("%hx", &value);
    write(fd_dac[0], &value, 1);
    printf("Output should show %f volts\n", volts(DAC_Gain[0], value));
    printf("Enter value for DAC1 [0-0xffff]: ");
    scanf("%hx", &value);
    write(fd_dac[1], &value, 1);
    printf("Output should show %f volts\n", volts(DAC_Gain[1], value));
    printf("\n Continue? ");
    scanf("%s", str);
    if (str[0] == 'n' || str[0] == 'N') return;
  }
}

void Domenu()
{
  int choice;

  while(1) {
    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("    1. Test Digital I/O.\n");
    printf("    2. Test Digital to Analog Converter.\n");
    printf("    3. Change D/A Gains.\n");
    printf("    4. Test simultanoues output on Chan 0 and 1\n");
    printf("    5. Exit.\n");
    printf("\nOption: ");
   
    scanf("%d", &choice);
    switch(choice){
      case 1:  
        testDIO();
        break;
      case 2:  
        testDAC();
        break;
      case 3:
        ChangeDACGains();
        break;
      case 4:
        testDAC2();
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
  close(fd_dac[0]);
  close(fd_dac[1]);
  close(fd_A);
  close(fd_B);
  close(fd_C);

  return(1);
}


