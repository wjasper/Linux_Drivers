/*
 * Copyright (C) 2007   Warren Jasper
 * All rights reserved.
 *
 */

/***************************************************************************
 *
 *  test-dio.c
 *
 *  This program is used to test the PCI-DIO24 Digital I/O
 *  Linux loadable module(pci-dio24).
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
#include "pci-dio24.h"

/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/dio24/dio0_0A";
char DevNameIO[20];
int  Board    = 0;
int  Count    = 1;
int  NoStop   = 0;
int  Mode     = 0;
int  Status;

/***************************************************************************
 *
 *  Show user how the command line is used.
 *
 ***************************************************************************/

void Usage( void )
{
  fprintf(stderr, "\n");
  fprintf(stderr, "Usage: test-dio24 \'options\'\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "    [-dev /dev/dio24/dio0_{A,B,C}] - Specify device file.\n");
  fprintf(stderr, "    [-board #]                     - Specify board number.\n");  
  fprintf(stderr, "    [-ct ##]                       - Number of samples to write\n");
  fprintf(stderr, "    [-nostop]                      - Write forever\n");
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
      if (i == argc) {
	Usage();
      } else {
	DevName = argv[i];
      }
    } else if (strcmp(argv[i], "-board") == 0) {
      i++;
      if (i == argc) {
	Usage();
      } else {
	Board = atoi(argv[i]);
      }
    } else if (strcmp(argv[i], "-ct") == 0) {
      i++;
      if (i == argc) {
	Usage();
      } else {
	Count = atoi(argv[i]);
      }
    } else if (strcmp(argv[i], "-nostop") == 0) {
      NoStop = 1;
    } else {
      Usage();
    }
    i++;
  }	
}

/***************************************************************************
 *
 *  Main
 *
 ***************************************************************************/

int main(int argc, char **argv)
{
  int fd_A, fd_B, fd_C;  
  unsigned char  bReg;
  unsigned short value;

  DoCommandLine(argc, argv);
	
  printf("Running in mode %d\n", Mode);

  sprintf(DevNameIO, "/dev/dio24/dio%1d_0A", Board);
  if ((fd_A = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  sprintf(DevNameIO, "/dev/dio24/dio%1d_0B", Board);
  if ((fd_B = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  sprintf(DevNameIO, "/dev/dio24/dio%1d_0C", Board);
  if ((fd_C = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }

  printf("***********************************************************\n");
  printf("* The program writes values on port A and reads them on   *\n");
  printf("* port B (8 bits), and writes on the low nibble of port C *\n");
  printf("* (4 bits) and reads them on the high nibble of port C.   *\n");
  printf("***********************************************************\n\n");

  ioctl(fd_A, DIO_SET_DIRECTION, PORT_OUTPUT);
  ioctl(fd_B, DIO_SET_DIRECTION, PORT_INPUT);
  ioctl(fd_C, DIO_SET_DIRECTION, LOW_PORT_OUTPUT);
  ioctl(fd_C, DIO_SET_DIRECTION, HIGH_PORT_INPUT);

  do {
      printf("Enter value for port A in hex: ");
      scanf("%hx", &value);
      write(fd_A, &value, 1);
      read(fd_A, &bReg, 1);
      printf("Port A value = %#x\n", bReg);

      read(fd_B, &bReg, 1);
      printf("Port B value = %#hx\n", bReg);

      printf("Enter value for port C in hex: ");
      scanf("%hx", &value);
      value &= 0x0f;          /* mask off high nibble */
      write(fd_C, &value, 1);
      read(fd_C, &bReg, 1);
      bReg >>= 4;
      printf("Port C value = %#hx\n", bReg);
  } while (NoStop);

  close(fd_A);
  close(fd_B);
  close(fd_C);

  return 0;
}
	
	



