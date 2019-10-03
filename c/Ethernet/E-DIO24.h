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

#ifndef E_DIO24_H

#define E_DIO24_H
#ifdef __cplusplus
extern "C" { 
#endif

#include "mccEthernet.h"

#define EDIO24_PID       0x0137

// Digital I/O Commands
#define CMD_DIN_R            (0x00)  // Read DIO pins
#define CMD_DOUT_R           (0x02)  // Read DIO latch value
#define CMD_DOUT_W           (0x03)  // Write DIO latch value
#define CMD_DCONF_R          (0x04)  // Read DIO configuration value
#define CMD_DCONF_W          (0x05)  // Write DIO Configuration value
// Counter Commands
#define CMD_COUNTER_R        (0x30)  // Read event counter
#define CMD_COUNTER_W        (0x31)  // Reset event counter
// Memory Commands
#define CMD_CONF_MEM_R       (0x40)  // Read configuration memeory
#define CMD_CONF_MEM_W       (0x41)  // Write configuration memory
#define CMD_USR_MEM_R        (0x42)  // Read user memory  
#define CMD_USR_MEM_W        (0x43)  // Write user memory
#define CMD_SET_MEM_R        (0x44)  // Read settings memory
#define CMD_SET_MEM_W        (0x45)  // Write settings memory
#define CMD_BOOT_MEM_R       (0x46)  // Read bootloader memory
#define CMD_BOOT_MEM_W       (0x47)  // Write bootloader memory
// Miscellaneous Commands
#define CMD_BLINKLED         (0x50)  // Blink the LED
#define CMD_RESET            (0x51)  // Reset the device
#define CMD_STATUS           (0x52)  // Read the device status
#define CMD_NETWORK_CONF     (0x54)  // Read device network configuration
#define CMD_FIRMWARE         (0x60)  // Enter bootloader for firmware upgrade

/* function prototypes for the E-DIO24 */
bool DIn_DIO24(EthernetDeviceInfo *device_info, uint32_t *value);
bool DOutR_DIO24(EthernetDeviceInfo *device_info, uint32_t *value);
bool DOut_DIO24(EthernetDeviceInfo *device_info, uint32_t mask, uint32_t value);
bool DConfigR_DIO24(EthernetDeviceInfo *device_info, uint32_t *value);
bool DConfigW_DIO24(EthernetDeviceInfo *device_info, uint32_t mask, uint32_t value);
bool CounterR_DIO24(EthernetDeviceInfo *device_info, uint32_t *count);
bool CounterW_DIO24(EthernetDeviceInfo *device_info);
bool BlinkLED_DIO24(EthernetDeviceInfo *device_info, unsigned char count);
bool Reset_DIO24(EthernetDeviceInfo *device_info);
bool Status_DIO24(EthernetDeviceInfo *device_info, uint16_t *status);
bool NetworkConfig_DIO24(EthernetDeviceInfo *device_info, struct in_addr newtork[3]);
bool FirmwareUpgrade_DIO24(EthernetDeviceInfo *device_info);
bool ConfigMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool ConfigMemoryW_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool UserMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool UserMemoryW_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool SettingsMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool SettingsMemoryW_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool BootloaderMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool BootloaderMemoryW_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // E_1608_H

/*
    Configuration memory map
|=================================================================|
|    Address   |        Value                                     |
|=================================================================|
| 0x00 - 0x07  | Serial number (not used by firmware)             |
|-----------------------------------------------------------------|
| 0x08 - 0x09  | Reserved                                         |
|-----------------------------------------------------------------|
| 0x0A - 0x0F  | MAC address                                      |
|              |  If all 6 bytes are 0xFF then the firmware will  |
|              |  use the Microchip unique MAC address that       |
|              |  is programmed into the micro.                   |
|=================================================================|


    Settings memory map
|=====================================================================================|
|    Address    |        Value                                        | Default value |
|=====================================================================================|
| 0x000 - 0x001 | Network options:                                    | 0x0000        |
|               |   Bit 0: 0 = DHCP enabled     1 = DHCP disabled     |               |
|               |   Bit 1: 0 = Auto IP enabled  1 = Auto IP disabled  |               |
|               |   Bits 2-15 reserved                                |               |
|-------------------------------------------------------------------------------------|
| 0x002 - 0x005 | Default IP address                                  | 192.168.0.101 |
|-------------------------------------------------------------------------------------|
| 0x006 - 0x009 | Default subnet mask                                 | 255.255.255.0 |
|-------------------------------------------------------------------------------------|
| 0x00A - 0x00D | Default gateway address                             | 192.168.0.1   |
|-------------------------------------------------------------------------------------|
| 0x00E - 0x011 | Reserved                                            |               |
|-------------------------------------------------------------------------------------|
| 0x012 - 0x015 | Connection code, 4 bytes                            | 0x00000000    |
|-------------------------------------------------------------------------------------|
| 0x016         | DOut connection mode.  This determines the DOut     | 0             |
|               | value when the connection status changes.           |               |
|               |   0 = no change                                     |               |
|               |   1 = apply specified tristate / latch values       |               |              
|-------------------------------------------------------------------------------------|
| 0x017         | Reserved                                            |               |  
|-------------------------------------------------------------------------------------|
| 0x018         | DOut port 0 tristate mask for connection /          | 0xFF          |            
|               | disconnection (bits set to 0 are outputs, bits set  |               |
|               | to 1 are no change)                                 |               |
|-------------------------------------------------------------------------------------|
| 0x019         | DOut port 1 tristate mask for connection /          | 0xFF          |            
|               | disconnection (bits set to 0 are outputs, bits set  |               |
|               | to 1 are no change)                                 |               |
|-------------------------------------------------------------------------------------|
| 0x01A         | DOut port 2 tristate mask for connection /          | 0xFF          |            
|               | disconnection (bits set to 0 are outputs, bits set  |               |
|               | to 1 are no change)                                 |               |
|-------------------------------------------------------------------------------------|
| 0x01B         | Reserved                                            |               |
|-------------------------------------------------------------------------------------|
| 0x01C         | DOut port0 latch value when host is connected       | 0x00          |
|-------------------------------------------------------------------------------------|
| 0x01D         | DOut port1 latch value when host is connected       | 0x00          |
|-------------------------------------------------------------------------------------|
| 0x01D         | DOut port2 latch value when host is connected       | 0x00          |
|-------------------------------------------------------------------------------------|
| 0x01F         | Reserved                                            |               |
|-------------------------------------------------------------------------------------|
| 0x020         | DOut port0 latch value when host is disconnected    | 0x00          |
|-------------------------------------------------------------------------------------|
| 0x021         | DOut port1 latch value when host is disconnected    | 0x00          |
|-------------------------------------------------------------------------------------|
| 0x022         | DOut port2 latch value when host is disconnected    | 0x00          |
|-------------------------------------------------------------------------------------|
| 0x023 - 0x0FF | Reserved                                            |               |
|=====================================================================================|

Note: The settings do not take effect until after device is reset or power cycled.

    User memory map
|=================================================================|
|    Address     |        Value                                   |
|=================================================================|
| 0x000 - 0xEEF  | Available for UL use                           |
|=================================================================|

*/
