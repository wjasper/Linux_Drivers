/*
 *
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

#ifndef ETHERNET_H

#define ETHERNET_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>


#define DISCOVER_PORT   54211
#define COMMAND_PORT    54211
#define SCAN_PORT       54212

#define MSG_SUCCESS         0   // Command succeeded
#define MSG_ERROR_PROTOCOL  1   // Command failed due to improper protocol
                                // (number of expected data bytes did not match protocol definition)
#define MSG_ERROR_PARAMETER 2   // Command failed due to invalid parameters
                                // (the data contents were incorrect)
#define MSG_ERROR_BUSY      3   // Command failed because resource was busy
#define MSG_ERROR_READY     4   // Command failed because the resource was not ready
#define MSG_ERROR_TIMEOUT   5   // Command failed due to a resource timeout
#define MSG_ERROR_OTHER     6   // Command failed due to some other error

#define MSG_HEADER_SIZE     6
#define MSG_CHECKSUM_SIZE   1

#define MSG_INDEX_START      0
#define MSG_INDEX_COMMAND    1
#define MSG_INDEX_FRAME      2
#define MSG_INDEX_STATUS     3
#define MSG_INDEX_COUNT_LOW  4  // The maximum value for count is 1024
#define MSG_INDEX_COUNT_HIGH 5
#define MSG_INDEX_DATA       6

#define MSG_REPLY            (0x80)
#define MSG_START            (0xDB)


/* Structures for Temperature */
//*******************************************************************
// NIST Thermocouple coefficients
//
// The following types are supported:
//
//	J, K, R, S, T, N, E, B

/* Define the types of Thermocouples supported */

#define CHAN_DISABLE 0
#define TC_TYPE_J    1  // Type J thermocouple
#define TC_TYPE_K    2  // Type K thermocouple
#define TC_TYPE_T    3  // Type T thermocouple
#define TC_TYPE_E    4  // Type E thermocouple
#define TC_TYPE_R    5  // Type R thermocouple
#define TC_TYPE_S    6  // Type S thermocouple
#define TC_TYPE_B    7  // Type B thermocouple
#define TC_TYPE_N    8  // Type N thermocouple

struct networkDeviceInfo_t {
  uint32_t ip_address;      // current device IP address
  uint32_t subnet_mask;     // current device subnet mask
  uint32_t gateway_address; // current gatewayaddress
};

typedef struct EthernetDeviceInfo_t {
  struct sockaddr_in Address;                        // internet address of device
  struct sockaddr_in RemoteHost;                     // internet address of remote host
  int sock;                                          // TCP command socket id 
  int scan_sock;                                     // TCP scan data socket id 
  uint32_t connectCode;                              // connent code (default 0x0)
  unsigned short ProductID;                          // Product ID 
  unsigned short FirmwareVersion;                    // Firmware version 1.01
  unsigned short BootloaderVersion;                  // Bootloader firmware version
  unsigned short CommandPort;                        // TCP command port (54211)
  unsigned short Status;                             // Status of response from device
  char NetBIOS_Name[16];                             // NetBIOS name: E-1608-19ADFS
  unsigned char MAC[6];                              // MAC address of device
  uint8_t frameID;                                   // current frame id
} EthernetDeviceInfo;


// global functions;
int flushInput(int sock);
int sendMessage(int sock, void *message, int length, int flags);
int receiveMessage(int sock, void *message, int maxLength, unsigned long timeout);
void printDeviceInfo(EthernetDeviceInfo *device_info);
unsigned char calcChecksum(void *buffer, int length);
int discoverDevice(EthernetDeviceInfo *device_info, uint16_t productID);
int discoverDevices(EthernetDeviceInfo *devices_info[], uint16_t productID, int maxDevices);
int openDevice(uint32_t addr, uint32_t connectCode);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif 
