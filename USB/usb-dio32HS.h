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

#ifndef USB_DIO32HS_H
#define USB_DIO32HS_H

#ifdef __cplusplus
extern "C" { 
#endif 

#define USBDIO32HS_PID  (0x0133)

#define DIO_PORTA  (0x00)
#define DIO_PORTB  (0x01)
#define DIO_PORTS  (0x02)  // Ports A & B
#define PORT0      (0x1)   // Port A for channel_map
#define PORT1      (0x2)   // Port B  for channel_map

#define DIO_DIR_IN  (0x01)
#define DIO_DIR_OUT (0x00)

/* Commands and Codes for USB-DIO32HS reports */
  // DIO Commands
#define DTRISTATE            (0x00)  // Read/Write Tristate register
#define DPORT                (0x01)  // Read digital port pins
#define DLATCH               (0x02)  // Read/Write Digital port output latch register
  // Register Commands
#define READ_REG             (0x10)  // Read the specified register
#define WRITE_REG            (0x11)  // Write the specified register
  // Acquisition Commands
#define IN_SCAN_START        (0x20)  // Start input scan
#define IN_SCAN_STOP         (0x21)  // Stop input scan
#define IN_SCAN_CLEAR_FIFO   (0x22)  // Clear data in the input FIFO
#define IN_BULK_FLUSH        (0x23)  // Flush the input Bulk pipe
#define OUT_SCAN_START       (0x24)  // Start output scan
#define OUT_SCAN_STOP        (0x25)  // Stop output scan
#define OUT_SCAN_CLEAR_FIFO  (0x26)  // Clear data in the output FIFO
  // Memory Commands
#define MEMORY               (0x30)  // Read/Write EEPROM
#define MEM_ADDRESS          (0x31)  // EEPROM read/write address value
#define MEM_WRITE_ENABLE     (0x32)  // Enable writes to firmware area
  //Miscellaneojus Commands
#define STATUS               (0x40)  // Device status
#define BLINK_LED            (0x41)  // Blink the LED
#define RESET                (0x42)  // Reset the device
#define TRIGGER_CONFIG       (0x43)  // External trigger configuration
#define PATTERN_DETECT       (0x44)  // Pattern Detection trigger configuration
#define SERIAL               (0x48)  // Read/write USB serial number
  // FPGA Commands
#define FPGA_CONFIG          (0x50)  // Start FPGA configuration
#define FPGA_DATA            (0x51)  // Write FPGA configuration data
#define FPGA_VERSION         (0x52)  // Read FPGA version

/* Status bit values */
#define IN_SCAN_RUNNING      (0x1 << 1)
#define IN_SCAN_OVERRUN      (0x1 << 2)
#define OUT_SCAN_RUNNING     (0x1 << 3)
#define OUT_SCAN_UNDERRUN    (0x1 << 4)
#define IN_SCAN_DONE         (0x1 << 5)
#define OUT_SCAN_DONE        (0x1 << 6)
#define FPGA_CONFIGURED      (0x1 << 8)
#define FPGA_CONFIG_MODE     (0x1 << 9)

/* function prototypes for the USB-DIO32HS */
void usbInit_DIO32HS(libusb_device_handle *udev);
uint16_t  usbDTristateR_USBDIO32HS(libusb_device_handle *udev, uint8_t port);
void usbDTristateW_USBDIO32HS(libusb_device_handle *udev, uint16_t value, uint8_t port);
uint32_t usbDPort_USBDIO32HS(libusb_device_handle *udev);
uint32_t usbDLatchR_USBDIO32HS(libusb_device_handle *udev, uint8_t port);
void usbDLatchW_USBDIO32HS(libusb_device_handle *udev, uint32_t data, uint8_t port);
void usbReadReg_USBDIO32HS(libusb_device_handle *udev, uint8_t address, uint8_t *value);
void usbWriteReg_USBDIO32HS(libusb_device_handle *udev, uint8_t address, uint8_t value);
void usbInScanStart_USBDIO32HS(libusb_device_handle *udev, uint8_t channel_map, uint32_t count, uint32_t retrig_count, double frequency, uint8_t packet_size, uint8_t options);
int usbInScanRead_USBDIO32HS(libusb_device_handle *udev, int count, uint16_t *data);
void usbInScanStop_USBDIO32HS(libusb_device_handle *udev);
void usbInScanClearFIFO_USBDIO32HS(libusb_device_handle *udev);
void usbInScanBulkFlush_USBDIO32HS(libusb_device_handle *udev, uint8_t count);
void usbOutScanStart_USBDIO32HS(libusb_device_handle *udev, uint8_t channel_map, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options);
int usbOutScanWrite_USBDIO32HS(libusb_device_handle *udev, int count, uint16_t *data);
void usbOutScanStop_USBDIO32HS(libusb_device_handle *udev);
void usbOutScanClearFIFO_USBDIO32HS(libusb_device_handle *udev);
void usbMemoryR_USBDIO32HS(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemoryW_USBDIO32HS(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemAddressR_USBDIO23HS(libusb_device_handle *udev, uint16_t address);
void usbMemAddressW_USBDIO3208HS(libusb_device_handle *udev, uint16_t address);
void usbMemWriteEnable_USBDIO32HS(libusb_device_handle *udev);
uint16_t usbStatus_USBDIO32HS(libusb_device_handle *udev);
void usbBlink_USBDIO32HS(libusb_device_handle *udev, uint8_t count);
void usbReset_USBDIO32HS(libusb_device_handle *udev);
void usbTriggerConfig_USBDIO32HS(libusb_device_handle *udev, uint8_t options);
void usbTriggerConfigR_USBDIO32HS(libusb_device_handle *udev, uint8_t *options);
void usbPatternDetectConfig(libusb_device_handle *udev, uint16_t value, uint16_t mask, uint8_t options);
void usbGetSerialNumber_USBDIO32HS(libusb_device_handle *udev, char serial[9]);
void usbFPGAConfig_USBDIO32HS(libusb_device_handle *udev);
void usbFPGAData_USBDIO32HS(libusb_device_handle *udev, uint8_t *data, uint8_t length);
void usbFPGAVersion_USBDIO32HS(libusb_device_handle *udev, uint16_t *version);
  
#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_DIO32HS_H
