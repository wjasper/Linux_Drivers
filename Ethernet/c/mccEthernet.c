/*
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@ncsu.edu>
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
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include "mccEthernet.h"

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

void printDeviceInfo(EthernetDeviceInfo *device_info)
{
  printf("   Found device: %s\n", device_info->NetBIOS_Name);
  printf("   MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
	 device_info->MAC[0], device_info->MAC[1], device_info->MAC[2],
	 device_info->MAC[3], device_info->MAC[4], device_info->MAC[5]);
  printf("   IP: %s\n", inet_ntoa(device_info->Address.sin_addr));
  printf("   Product ID: 0x%04X\n", device_info->ProductID);
  printf("   FW version: %02X.%02X\n", 
	 (unsigned char)(device_info->FirmwareVersion >> 8),
	 (unsigned char)(device_info->FirmwareVersion));
  printf("   Boot version: %02X.%02X\n",
	 (unsigned char)(device_info->BootloaderVersion >> 8),
	 (unsigned char)(device_info->BootloaderVersion));
  printf("   Command Port: %d\n", device_info->CommandPort);
  printf("   Status: %s\n", (device_info->Status == 0) ? "Available" : "In Use");
  printf("   Remote host: %s\n\n", inet_ntoa(device_info->RemoteHost.sin_addr));
}

int discoverDevice(EthernetDeviceInfo *device_info, uint16_t productID)
{
  EthernetDeviceInfo device;  // a MCC device that responded
  struct sockaddr_in sendaddr;
  struct sockaddr_in recvaddr;
  struct sockaddr_in remoteaddr;
  struct timeval tv;
  
  int sock;
  int broadcast;
  unsigned char msg[64];
  bool finished = false;   
  int nfound = 0;            // number of matched devices found
  socklen_t remoteaddrSize;
  int BytesReceived;

  // create the socket
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("discoverDevice: Error in creating socket");
    return -1;
  }
  // set the broadcast option
  broadcast = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
    perror("discoverDevice: Error setting socket options");
    close(sock);
    return -1;
  }
  // set up the send and receive addresses
  memset(&sendaddr, 0, sizeof(sendaddr));
  sendaddr.sin_family = AF_INET;
  sendaddr.sin_addr.s_addr = INADDR_BROADCAST;
  sendaddr.sin_port = htons(DISCOVER_PORT);
  
  memset(&recvaddr, 0, sizeof(recvaddr));
  recvaddr.sin_family = AF_INET;
  recvaddr.sin_addr.s_addr = INADDR_ANY;
  recvaddr.sin_port = htons(DISCOVER_PORT);

  // bind the socket for receive
  if (bind(sock, (struct sockaddr*)&recvaddr, sizeof(recvaddr)) < 0) {
    perror("discoverDevice: Error binding port");
    close(sock);
    return -1;
  }

  // send a broadcast discover datagram
  msg[0] = 'D';
  if (sendto(sock, msg, 1, 0, (struct sockaddr*)&sendaddr, sizeof(sendaddr)) != 1) {
    perror("discoverDevice: sendto failed");
    close(sock);
    return -1;
  }

  tv.tv_sec = 1;
  tv.tv_usec = 0;

  // look for replies (including the original broadcast)
  while (!finished) {
    switch (recvfromTimeOut(sock, &tv)) {
      case 0:
        // timed out
        finished = true;
        break;
      case -1:
        // error
        printf("Error from recvfromTimeOut\n");
        finished = true;
        close(sock);
        return -1;
        break;
      default:
       // got a reply
        remoteaddrSize = sizeof(remoteaddr);
        BytesReceived = recvfrom(sock, msg, 64, 0, (struct sockaddr*)&remoteaddr, &remoteaddrSize);
        if ((BytesReceived == 64) && (msg[0] == 'D')) {
	  memcpy(device.MAC, &msg[1], 6);
	  memcpy(&device.ProductID, &msg[7], 2);
	  memcpy(&device.FirmwareVersion, &msg[9], 2);
	  memcpy(device.NetBIOS_Name, &msg[11], 16);
	  memcpy(&device.CommandPort, &msg[27], 2);
	  memcpy(&device.Status, &msg[33], 2);
	  memcpy(&device.RemoteHost.sin_addr, &msg[35], 4);
	  memcpy(&device.BootloaderVersion, &msg[39], 2);
	  memcpy(&device.Address, &remoteaddr, sizeof(remoteaddr));
          if (device.ProductID == productID) {  // check for match
	    memcpy(device_info, &device, sizeof(EthernetDeviceInfo));
	    nfound++;
	  }
	}
	break;
    }
  }
  close(sock);
  return nfound;
}

int discoverDevices(EthernetDeviceInfo *devices_info[], uint16_t productID, int maxDevices)
{
  EthernetDeviceInfo device;  // a MCC device that responded
  struct sockaddr_in sendaddr;
  struct sockaddr_in recvaddr;
  struct sockaddr_in remoteaddr;
  struct timeval tv;
  int sock;
  int broadcast;
  unsigned char msg[64];
  bool finished = false;   
  int nfound = 0;            // number of matched devices found
  socklen_t remoteaddrSize;
  int BytesReceived;

  // create the socket
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("discoverDevices: Error in creating socket");
    return -1;
  }

  // set the broadcast option
  broadcast = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
    perror("discoverDevices: Error setting socket options");
    close(sock);
    return -1;
  }

  // set up the send and receive addresses
  memset(&sendaddr, 0, sizeof(sendaddr));
  sendaddr.sin_family = AF_INET;
  sendaddr.sin_addr.s_addr = INADDR_BROADCAST;
  sendaddr.sin_port = htons(DISCOVER_PORT);
  
  memset(&recvaddr, 0, sizeof(recvaddr));
  recvaddr.sin_family = AF_INET;
  recvaddr.sin_addr.s_addr = INADDR_ANY;
  recvaddr.sin_port = htons(DISCOVER_PORT);

  // bind the socket for receive
  if (bind(sock, (struct sockaddr*)&recvaddr, sizeof(recvaddr)) < 0) {
    perror("discoverDevices: Error binding port");
    close(sock);
    return -1;
  }

  // send a broadcast discover datagram
  msg[0] = 'D';
  if (sendto(sock, msg, 1, 0, (struct sockaddr*)&sendaddr, sizeof(sendaddr)) != 1) {
    perror("discoverDevices: sendto failed");
    close(sock);
    return -1;
  }

  tv.tv_sec = 1;
  tv.tv_usec = 0;

  // look for replies (including the original broadcast)
  while (!finished) {
    switch (recvfromTimeOut(sock, &tv)) {
      case 0:
        // timed out
        finished = true;
        break;
      case -1:
        // error
        printf("Error from recvfromTimeOut\n");
        finished = true;
        close(sock);
        return -1;
        break;
      default:
       // got a reply
        remoteaddrSize = sizeof(remoteaddr);
        BytesReceived = recvfrom(sock, msg, 64, 0, (struct sockaddr*)&remoteaddr, &remoteaddrSize);
        if ((BytesReceived == 64) && (msg[0] == 'D')) {
	  memcpy(device.MAC, &msg[1], 6);
	  memcpy(&device.ProductID, &msg[7], 2);
	  memcpy(&device.FirmwareVersion, &msg[9], 2);
	  memcpy(device.NetBIOS_Name, &msg[11], 16);
	  memcpy(&device.CommandPort, &msg[27], 2);
	  memcpy(&device.Status, &msg[33], 2);
	  memcpy(&device.RemoteHost.sin_addr, &msg[35], 4);
	  memcpy(&device.BootloaderVersion, &msg[39], 2);
	  memcpy(&device.Address, &remoteaddr, sizeof(remoteaddr));
          if (device.ProductID == productID) {  // check for match
	    memcpy(devices_info[nfound], &device, sizeof(EthernetDeviceInfo));
	    devices_info[nfound]->connectCode = 0x0;  // default connect code
	    devices_info[nfound]->frameID = 0x0;      // initialize frameID
	    nfound++;
	    if (nfound > maxDevices) {
	      close(sock);
	      return nfound;
	    }
	  }
	}
	break;
    }
  }
  close(sock);
  return nfound;
}

int openDevice(uint32_t addr, uint32_t connectCode)
{
  int sock;
  struct sockaddr_in sendaddr;
  struct timeval tv;
  unsigned char msg[64];
  int bytesReceived;

  // open the UDP socket
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("openDevice: Error creating socket.");
      return -1;
  }

  // set up the send address
  memset(&sendaddr, 0, sizeof(sendaddr));
  sendaddr.sin_family = AF_INET;
  sendaddr.sin_addr.s_addr = addr;
  sendaddr.sin_port = htons(DISCOVER_PORT);

  if (connect(sock, (const struct sockaddr*) &sendaddr, sizeof(sendaddr)) < 0) {
    perror("openDevice: error in connect.");
    close(sock);
    return -1;
  }
 
  // send the connect message
  msg[0] = 'C';
  memcpy(&msg[1], &connectCode, 4);
  if (send(sock, msg, 5, 0) < 0) {
    perror("openDevice: Error in send.");
    close(sock);
    return -1;
  }

  tv.tv_sec = 1;
  tv.tv_usec = 0;

  // look for a reply
  switch (recvfromTimeOut(sock, &tv)) {
    case 0:  // timed out
    case -1: //  error    
      close (sock);
      return -1;
      break;
    default:  // got a reply
      bytesReceived = recv(sock, msg, 64, 0);
      if ((bytesReceived == 2) && (msg[0] = 'C') && (msg[1] == 0)) {
	break;
      } else {
	close(sock);
	return -1;
      }
      break;
  }
  close(sock);   // finished with the UDP portion

  if ((sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
    perror("openDevice: error creating TCP socket.");
    return -1;
  }

  // set up the send address
  memset(&sendaddr, 0, sizeof(sendaddr));
  sendaddr.sin_family = AF_INET;
  sendaddr.sin_addr.s_addr = addr;
  sendaddr.sin_port = htons(COMMAND_PORT);

  // create a tcp connection
  if ((connect(sock, (const struct sockaddr*) &sendaddr, sizeof(sendaddr))) < 0) {
      perror("openDevice: can not connect to device.");
      close(sock);
      return -1;
  }
    return sock;
}     

int flushInput(int sock)
{
  int numRecv, numTotal=0;
  char cbuf[512];

  while (1) {
      numRecv = recv(sock, cbuf, sizeof cbuf, MSG_DONTWAIT);
      if (numRecv <= 0) break;
      numTotal += numRecv;
  }
  if (numTotal > 0) fprintf(stderr, "ethernet::flushInput flushed %d bytes\n", numTotal);
  return numTotal;
}

int sendMessage(int sock, void *message, int length, int flags)
{
  flushInput(sock);
  return send(sock, message, length, flags);
}

int receiveMessage(int sock, void *message, int maxLength, unsigned long timeout)
{
  struct timeval tv;
  int bytesReceived = 0;

  if (sock < 0) {  // invalid socket number.
    return -1;
  }

  // set a receive timeout
  tv.tv_sec = timeout/1000;
  tv.tv_usec = (timeout - (tv.tv_sec*1000)) * 1000;

  // set a receive timeout  
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *) &tv, sizeof(tv));

  switch (recvfromTimeOut(sock, &tv)) {
    case 0:   // timed out
    case -1:  // error
      return -1;
      break;
    default:  // got a reply
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
