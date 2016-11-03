/*
 *
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@tx.ncsu.edu>
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

#include "ethernet.h"

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
bool SettingsMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool BootloaderMemoryR_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);
bool BootloaderMemoryW_DIO24(EthernetDeviceInfo *device_info, uint16_t address, uint16_t count, uint8_t *data);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // E_1608_H
