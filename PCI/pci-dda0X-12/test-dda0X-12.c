/*
 * Copyright (C) 2003   Warren J. Jasper
 * All rights reserved.
 *
 */

/***************************************************************************
 *
 *  test-dda0X_12.c
 *
 *  This program is used to test the PCI-DDA0X-12 
 *  Linux loadable module(pci-dda0X_12).
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
#include "pci-dda0X-12.h"

#define FS  (4096)

/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/dda0x-12/da0_0";
char DevNameIO[20];
int  Count    = 1;
int  NoStop   = 0;
int  Mode     = 0;
int  Status;

static int fd_0A, fd_0B, fd_0C;
static int fd_1A, fd_1B, fd_1C;
static int fd_dac[8];    // we are only going to use 2 for this test.

int  DAC_Gain[4]  = {UP_10_0V, UP_10_0V, UP_10_0V, UP_10_0V};

/***************************************************************************
 *
 *  Show user how the command line is used.
 *
 ***************************************************************************/

void Usage( void )
{
  fprintf(stderr, "\n");
  fprintf(stderr, "Usage: test-dda0X_12 \'options\'\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "    [-dev /dev/dda0x_12/dio0_{A,B,C}]  - Specify device file.\n");
  fprintf(stderr, "    [-ct ##]                     - Number of samples to write\n");
  fprintf(stderr, "    [-nostop]                    - Write forever\n");
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
  strcpy(DevNameIO, "/dev/dda0x-12/da0_0");
  if ((fd_dac[0] = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  ioctl(fd_dac[0], DAC_SET_GAINS, UP_10_0V);

  strcpy(DevNameIO, "/dev/dda0x-12/da0_1");
  if ((fd_dac[1] = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  ioctl(fd_dac[1], DAC_SET_GAINS, UP_10_0V);

  /* open the dio */
  strcpy(DevNameIO, "/dev/dda0x-12/dio0_0A");
  if ((fd_0A = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dda0x-12/dio0_0B");
  if ((fd_0B = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dda0x-16/dio0_0C");
  if ((fd_0C = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dda0x-12/dio0_1A");
  if ((fd_1A = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dda0x-12/dio0_1B");
  if ((fd_1B = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dda0x-12/dio0_1C");
  if ((fd_1C = open(DevNameIO, O_RDWR )) < 0) {
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
    if (ioctl(fd_0A, DIO_SET_DIRECTION, PORT_OUTPUT) < 0)
      perror("ioctl Group 0 Port A failed");
    if (ioctl(fd_0B, DIO_SET_DIRECTION, PORT_INPUT) < 0)
      perror("ioctl Group 0 Port B failed");
    if (ioctl(fd_0C, DIO_SET_DIRECTION, LOW_PORT_OUTPUT) < 0)
      perror("ioctl Group 0 Port CL failed");
    if (ioctl(fd_0C, DIO_SET_DIRECTION, HIGH_PORT_INPUT) < 0)
      perror("ioctl Group 0 Port CH failed");
    if (ioctl(fd_1A, DIO_SET_DIRECTION, PORT_OUTPUT) <0)
      perror("ioctl Group 1 Port A failed");
    if (ioctl(fd_1B, DIO_SET_DIRECTION, PORT_INPUT) < 0)
      perror("ioctl Group 1 Port B failed");
    if (ioctl(fd_1C, DIO_SET_DIRECTION, LOW_PORT_OUTPUT) < 0)
      perror("ioctl Group 1 Port CL failed");
    if (ioctl(fd_1C, DIO_SET_DIRECTION, HIGH_PORT_INPUT) < 0)
      perror("ioctl Group 1 Port CH failed");

    printf("Enter value for port 0A in hex: ");
    scanf("%hx", &value);
    write(fd_0A, &value, 1);
    read(fd_0A, &bReg, 1);
    printf("Port 0A value = %#x\n", bReg);
    read(fd_0B, &bReg, 1);
    printf("Port 0B value = %#hx\n", bReg);

    // Now reverse the process
    if (ioctl(fd_0A, DIO_SET_DIRECTION, PORT_INPUT) < 0)
      perror("ioctl Group 0 Port A failed");
    if (ioctl(fd_0B, DIO_SET_DIRECTION, PORT_OUTPUT) < 0)
      perror("ioctl Group 0 Port B failed");
    write(fd_0B, &value, 1);
    read(fd_0B, &bReg, 1);
    printf("Port 0B value = %#x\n", bReg);
    read(fd_0A, &bReg, 1);
    printf("Port 0A value = %#hx\n\n", bReg);

    printf("Enter value for port 0C in hex: ");
    scanf("%hx", &value);
    value &= 0x0f;          /* mask off high nibble */
    write(fd_0C, &value, 1);
    read(fd_0C, &bReg, 1);
    bReg >>= 4;
    printf("Port 0C value = %#hx\n", bReg);
    // Now reverse the process
    if (ioctl(fd_0C, DIO_SET_DIRECTION, LOW_PORT_INPUT) < 0)
      perror("ioctl Group 0 Port CL failed");
    if (ioctl(fd_0C, DIO_SET_DIRECTION, HIGH_PORT_OUTPUT) < 0)
      perror("ioctl Group 0 Port CH failed");
    value <<= 4;
    write(fd_0C, &value, 1);
    read(fd_0C, &bReg, 1);
    bReg &= 0xf;
    printf("Port 0C value = %#hx\n\n", bReg);

    printf("Enter value for port 1A in hex: ");
    scanf("%hx", &value);
    write(fd_1A, &value, 1);
    read(fd_1A, &bReg, 1);
    printf("Port 1A value = %#x\n", bReg);
    read(fd_1B, &bReg, 1);
    printf("Port 1B value = %#hx\n", bReg);
    // Now reverse the process
    if (ioctl(fd_1A, DIO_SET_DIRECTION, PORT_INPUT) < 0)
      perror("ioctl Group 1 Port A failed");
    if (ioctl(fd_1B, DIO_SET_DIRECTION, PORT_OUTPUT) < 0)
      perror("ioctl Group 1 Port B failed");
    write(fd_1B, &value, 1);
    read(fd_1B, &bReg, 1);
    printf("Port 1B value = %#x\n", bReg);
    read(fd_1A, &bReg, 1);
    printf("Port 1A value = %#hx\n\n", bReg);

    printf("Enter value for port 1C in hex: ");
    scanf("%hx", &value);
    value &= 0x0f;          /* mask off high nibble */
    write(fd_1C, &value, 1);
    read(fd_1C, &bReg, 1);
    bReg >>= 4;
    printf("Port 1C value = %#hx\n", bReg);
    // Now reverse the process
    if (ioctl(fd_1C, DIO_SET_DIRECTION, LOW_PORT_INPUT) < 0)
      perror("ioctl Group 1 Port CL failed");
    if (ioctl(fd_1C, DIO_SET_DIRECTION, HIGH_PORT_OUTPUT) < 0)
      perror("ioctl Group 1 Port CH failed");
    value <<= 4;
    write(fd_1C, &value, 1);
    read(fd_1C, &bReg, 1);
    bReg &= 0xf;
    printf("Port 1C value = %#hx\n\n", bReg);
    
    printf("\n Continue? ");
    scanf("%s", str);
    if (str[0] == 'n' || str[0] == 'N') return;
  }
}

float volts( int gain, unsigned short value )
{
  float volt;
  
  switch( gain ) {
    case BP_2_5V:
      volt = (2.5/FS)*(value - FS/2);
      break;
    case BP_5_0V:
      volt = (5.0/FS)*(value - FS/2);
      break;
    case BP_10_0V:
      volt = (10.0/FS)*(value - FS/2);
      break;
    case UP_2_5V:
      volt = (2.5/FS)*(value);
      break;
    case UP_5_0V:
      volt = (5.0/FS)*(value);
      break;
    case UP_10_0V:
      volt = (10.0/FS)*value;
      break;
  }
  return volt;
}

void ChangeDACGains()
{
  int choice;
  int channel = 0;
  long gain;

  printf("Enter desired channel: ");
  scanf("%d", &channel);
  printf("Select from the following choices:\n");
  printf("  1.   Bipolar  +/- 10.00V.\n");
  printf("  2.   Bipolar  +/-  5.00V.\n");
  printf("  3.   Bipolar  +/-  2.50V.\n");
  printf("  4.   Unipolar +/- 10.00V.\n");
  printf("  5.   Unipolar +/-  5.00V.\n");
  printf("  6.   Unipolar +/-  2.50V.\n");

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
    ioctl(fd_dac[channel], DAC_SET_GAINS, BP_2_5V);
    DAC_Gain[channel] = BP_2_5V;
    break;
  case 4:
    ioctl(fd_dac[channel], DAC_SET_GAINS, UP_10_0V);
    DAC_Gain[channel] = UP_10_0V;
    break;
  case 5:
    ioctl(fd_dac[channel], DAC_SET_GAINS, UP_5_0V);
    DAC_Gain[channel] = UP_5_0V;
    break;
  case 6:
    ioctl(fd_dac[channel], DAC_SET_GAINS, UP_2_5V);
    DAC_Gain[channel] = UP_2_5V;
    break;
  default:
    printf("Illegal option.\n");
  }
  ioctl(fd_dac[channel], DAC_GET_GAINS, &gain);
  printf("Gain Set to %#lx\n\n", gain);
  sleep(2);
  return;
}
void testDAC2()
{
/* a sine wave for Mike Zhu :) */

  int i, j;
  double x;
  unsigned short value = 0;

  ioctl(fd_dac[0], DAC_SET_GAINS, UP_10_0V);
  ioctl(fd_dac[0], DAC_SET_SIMULT, 1);
  ioctl(fd_dac[1], DAC_SET_GAINS, BP_10_0V);
  ioctl(fd_dac[1], DAC_SET_SIMULT, 1);
  for (j = 0; j < 120; j++) {
    for (i = 0; i < FS; i++) {
      x = 5.0 + 5.*sin(2*M_PI*i/FS);
      value = x*FS/10.;
      write(fd_dac[0], &value, 1);  // sine wave
      x = 5 + 2.5*cos(2*M_PI*i/FS);
      value = x*FS/10.;
      write(fd_dac[1], &value, 1);  // sine wave
      ioctl(fd_dac[0], DAC_SIMULT_UPDATE, 0);
      //write(fd_dac[0], &i, 1);    // triangular wave
    }
  }
}

void testDAC()
{
  unsigned short value;
  char str[80];

  ioctl(fd_dac[0], DAC_SET_SIMULT, 0);
  ioctl(fd_dac[1], DAC_SET_SIMULT, 0);
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
  close(fd_0A);
  close(fd_0B);
  close(fd_0C);
  close(fd_1A);
  close(fd_1B);
  close(fd_1C);

  return(1);
}


