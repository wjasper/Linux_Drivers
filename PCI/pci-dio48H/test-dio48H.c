/*
 * Copyright (C) 2007   Warren J. Jasper
 * All rights reserved.
 *
 */

/***************************************************************************
 *
 *  test-dio48H.c
 *
 *  This program is used to test the PCI-DIO48H Digital I/O
 *  Linux loadable module(pci-dio48H).
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
#include "pci-dio48H.h"


/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/dio48H/dio0_0A";
char DevNameIO[20];
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
  fprintf(stderr, "Usage: test-dio48H \'options\'\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "    [-dev /dev/dio48H/dio0_0{A,B,C}]  - Specify device file.\n");
  fprintf(stderr, "    [-ct ##]                          - Number of samples to write\n");
  fprintf(stderr, "    [-nostop]                         - Write forever\n");
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
  int fd_0A, fd_0B, fd_0C;
  int fd_1A, fd_1B, fd_1C;  
  unsigned char  bReg;
  unsigned short value;

  DoCommandLine(argc, argv);
	
  printf("Running in mode %d\n", Mode);

  strcpy(DevNameIO, "/dev/dio48H/dio0_0A");
  if ((fd_0A = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio48H/dio0_0B");
  if ((fd_0B = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio48H/dio0_0C");
  if ((fd_0C = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio48H/dio0_1A");
  if ((fd_1A = open(DevNameIO, O_RDWR )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio48H/dio0_1B");
  if ((fd_1B = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio48H/dio0_1C");
  if ((fd_1C = open(DevNameIO, O_RDWR )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }

  printf("***********************************************************\n");
  printf("* The program writes values on port A and reads them on   *\n");
  printf("* port B (8 bits), and writes on the low nibble of port C *\n");
  printf("* (4 bits) and reads them on the high nibble of port C.   *\n");
  printf("***********************************************************\n\n");

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

  do {
      printf("Enter value for port 0A in hex: ");
      scanf("%hx", &value);
      write(fd_0A, &value, 1);
      read(fd_0A, &bReg, 1);
      printf("Port 0A value = %#x\n", bReg);

      read(fd_0B, &bReg, 1);
      printf("Port 0B value = %#hx\n", bReg);

      printf("Enter value for port 0C in hex: ");
      scanf("%hx", &value);
      value &= 0x0f;          /* mask off high nibble */
      write(fd_0C, &value, 1);
      read(fd_0C, &bReg, 1);
      bReg >>= 4;
      printf("Port 0C value = %#hx\n", bReg);

       printf("Enter value for port 1A in hex: ");
      scanf("%hx", &value);
      write(fd_1A, &value, 1);
      read(fd_1A, &bReg, 1);
      printf("Port 1A value = %#x\n", bReg);

      read(fd_1B, &bReg, 1);
      printf("Port 1B value = %#hx\n", bReg);

      printf("Enter value for port 1C in hex: ");
      scanf("%hx", &value);
      value &= 0x0f;          /* mask off high nibble */
      write(fd_1C, &value, 1);
      read(fd_1C, &bReg, 1);
      bReg >>= 4;
      printf("Port 1C value = %#hx\n", bReg);
} while (NoStop);

  close(fd_0A);
  close(fd_0B);
  close(fd_0C);
  close(fd_1A);
  close(fd_1B);
  close(fd_1C);

  return 0;
}
	
	



