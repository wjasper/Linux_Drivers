/***************************************************************************
 Copyright (C) 2010  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-QUAD04, is free software; you can redistribute it
 and/or modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation; either version 2 of the
 License, or (at your option) any later version, provided that this
 copyright notice is preserved on all copies.

 ANY RIGHTS GRANTED HEREUNDER ARE GRANTED WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, AND FURTHER,
 THERE SHALL BE NO WARRANTY AS TO CONFORMITY WITH ANY USER MANUALS OR
 OTHER LITERATURE PROVIDED WITH SOFTWARE OR THAM MY BE ISSUED FROM TIME
 TO TIME. IT IS PROVIDED SOLELY "AS IS".

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
***************************************************************************/

/***************************************************************************
 *
 *  test-quad04.c
 *
 *  This program is used to test the PCI-QUAD04 Quadrature Encoder Board
 *  Linux loadable module.
 *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include "pci-quad04.h"

char *DevName = "/dev/quad04/channel0_1";
int fd_channel1 = -1;
int fd_channel2 = -1;


void DoOpenDevices()
{
  char str[80];

  strcpy(str, "/dev/quad04/channel0_1");
  if ((fd_channel1 = open(str, O_RDWR)) < 0) {
    perror(str);
    printf("error opening device %s\n",str);
    exit(2);
  }
  strcpy(str, "/dev/quad04/channel0_2");
  if ((fd_channel2 = open(str, O_RDWR)) < 0) {
    perror(str);
    printf("error opening device %s\n",str);
    exit(2);
  }
}

void Domenu()
{
  int ret;
  int count = 0;
  unsigned int value;
  int flag = 0;
  

  printf("\n\n Connect the following pins: \n");
  printf("          Encoder 1                   Encoder 2   \n");
  printf("      A1+  --> Pin 20              A2+ --> Pin 34\n");
  printf("      B1+  --> Pin 21              B2+ --> Pin 35\n");
  printf("      GND  --> Pin 36              GND --> Pin 36\n");
  printf("      +5V  --> Pin 18              +5V --> Pin  2\n");
  printf("    Pin 23 --> Pin 8            Pin 37 --> Pin  4\n\n");

  while (1) {
    count++;
    if ((count % 20) == 0) {
      value = 0x0;
      write(fd_channel1, &value, 3);
      ioctl(fd_channel1, LOAD_CMD_REG, TRAN_PR_CNTR);
      printf("\nZeroing out the counter\n");
    }
    /* read the first encoder */
    value = 0x0;
    ret = read(fd_channel1, &value,  3);
    if (ret != 3) {
      printf("Error in reading channel 1.\n");
    } else {
      ioctl(fd_channel1, GET_CMD_REG, &flag);
      printf("Encoder 1 = %#6x\t", value);
      sleep(1);
    }
    /* read the second encoder */
    value = 0x0;
    if (fd_channel2 > 0) {
      ret = read(fd_channel2, &value,  3);
      if (ret != 3) {
        printf("Error in reading channel 2.\n");
      } else {
        printf("Encoder 2: = %#6x\tFlag = %x\n", value, flag);
      }
    }
  }
}

int main(int argc, char **argv)
{
  DoOpenDevices();
  Domenu();
  close(fd_channel1);
  return(0);
}

