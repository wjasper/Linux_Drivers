/*
 *
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

#ifndef BLUETOOTH_H

#define BLUETOOTH_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#define MSG_SUCCESS         0   // Command succeeded
#define MSG_ERROR_PROTOCOL  1   // Command failed due to improper protocol
                                // (number of expected data bytes did not match protocol definition)
#define MSG_ERROR_PARAMETER 2   // Command failed due to invalid parameters
                                // (the data contents were incorrect)
#define MSG_ERROR_BUSY      3   // Command failed because resource was busy
#define MSG_ERROR_READY     4   // Command failed due to FIFO overrun

#define MSG_INDEX_START     0
#define MSG_INDEX_COMMAND   1
#define MSG_INDEX_FRAME     2
#define MSG_INDEX_STATUS    3
#define MSG_INDEX_COUNT     4  // The maximum value for count is 0x256
#define MSG_INDEX_DATA      5

#define MSG_HEADER_SIZE     5
#define MSG_CHECKSUM_SIZE   1

#define MSG_REPLY           (0x80)
#define MSG_START           (0xDB)

typedef struct BluetoothDeviceInfo_t {
  struct sockaddr_rc address;                // baddr address of device
  char baddr[18];                            // string of baddr 00:06:66:71:50:E3
  int sock;                                  // bluetooth socket
  uint8_t frameID;                           // current frame id
} BluetoothDeviceInfo;

// global functions;
int receiveMessage(int sock, void *message, int maxLength, unsigned long timeout);
unsigned char calcChecksum(void *buffer, int length);
int openDevice(BluetoothDeviceInfo *device);
int discoverDevice(BluetoothDeviceInfo *device, char *name);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // BLUETOOTH_H
