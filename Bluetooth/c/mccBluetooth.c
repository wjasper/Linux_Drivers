/*
 *  Copyright (c) 2016 Warren J. Jasper <wjasper@ncsu.edu>
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

#include <stdio.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include "mccBluetooth.h"


static int recvfromTimeOut(int sock, struct timeval* timeout)
{
  fd_set fds;

  FD_ZERO(&fds);
  FD_SET(sock, &fds);
  // -1: error occurred
  // 0: timed out
  // >0: data ready to be read
  return select(sock+1, &fds, 0, 0, timeout);
}

int receiveMessage(int sock, void *message, int maxLength, unsigned long timeout)
{
  // timeout is in ms.
  struct timeval tv;
  int bytesReceived = 0;
  int status;

  if (sock < 0) {  // invalid socket number.
    return -1;
  }

  //  bytesReceived = recv(sock, message, maxLength, 0);
  //  return bytesReceived;
  
  tv.tv_sec = timeout/1000;
  tv.tv_usec = (timeout - (tv.tv_sec*1000)) * 1000;

  // set a receive timeout
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *) &tv, sizeof(tv));

  status = recvfromTimeOut(sock, &tv);

  switch (status) {
    case 0:   // timed out
    case -1:  // error 
     return -1;
      break;
    default:  // got a reply
      //bytesReceived = recv(sock, message, maxLength, MSG_DONTWAIT);
      bytesReceived = recv(sock, message, maxLength, 0);
      break;
  }
  return bytesReceived;
}

unsigned char calcChecksum(void *buffer, int length)
{
  int i;
  unsigned char checksum = 0;

  for (i = 0; i < length; i++) {
    checksum += ((unsigned char*) buffer)[i];
  }
  return checksum;
}

int discoverDevice(BluetoothDeviceInfo *device, char *name)
{
  FILE *fp;
  char var[80];
  int len;

  fp = popen("/usr/bin/bt-device --list", "r");
  len = sizeof(name);

  while (fgets(var, sizeof(var), fp) != NULL) {
    if (strncmp(var, name, len) == 0) {  // match!!
      strtok(var, "(");
      strncpy(device->baddr, strtok(NULL, "("), 17);
      device->baddr[17] = '\0';
      pclose(fp);
      return 0;
    }
  }
  return -1;
}

int openDevice(BluetoothDeviceInfo *device)
{
  int sock, status;
  struct sockaddr_rc addr = {0};

  // allocate a socket
  sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

  // set the connection parameters
  addr.rc_family = AF_BLUETOOTH;
  addr.rc_channel = 1;
  str2ba(device->baddr, &addr.rc_bdaddr);

  // connect to server
  status = connect(sock, (struct sockaddr *) &addr, sizeof(addr));
  if (status < 0) {
    perror("could not connect to bluetooth device");
    close(sock);
    return -1;
  }
  device->sock = sock;

  return 0;
}
