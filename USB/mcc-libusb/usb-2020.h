/*
 *
 *  Copyright (c) 2014 Warren J. Jasper <wjasper@ncsu.edu>
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

#ifndef USB_2020_H

#define USB_2020_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB2020_PID (0x011c)

/* Aanalog Input */
#define CALIBRATION    1
#define LAST_CHANNEL  (0x8)
#define PACKET_SIZE    512    // max bulk transfer size in bytes
#define USB_CONTINUOUS_READOUT (0x1)
#define USB_SINGLEIO           (0x2)
#define USB_FORCE_PACKET_SIZE  (0x4)
  
/* Ranges */
#define BP_10V 0x0    // +/- 10 V
#define BP_5V  0x1    // +/- 5V
#define BP_2V  0x2    // +/- 2V
#define BP_1V  0x3    // +/- 1V

/* Options for AInScan */
#define TRIGGER            (0x1 << 3) // 1 = use trigger or gate
#define PACER_OUT          (0x1 << 5) // 1 = External Pacer Output, 0 = External Pacer Input
#define RETRIGGER          (0x1 << 6) // 1 = retrigger mode, 0 = normal trigger
#define DDR_RAM            (0x1 << 7) // 1 = Use DDR RAM as storage (BURSTIO) , 0 = Stream via USB
  
/* Status bit values */
#define AIN_SCAN_RUNNING   (0x1 << 1)
#define AIN_SCAN_OVERRUN   (0x1 << 2)
#define AIN_SCAN_DONE      (0x1 << 5)
#define FPGA_CONFIGURED    (0x1 << 8)
#define FPGA_CONFIG_MODE   (0x1 << 9)

#define NCHAN_2020            2 // max number of A/D channels in the device
#define NGAINS_2020           4 // max number of gain levels
#define MAX_PACKET_SIZE_HS  512 // max packet size for HS device
#define MAX_PACKET_SIZE_FS   64 // max packet size for FS device

typedef struct Calibration_AIN_t {
  float slope;
  float offset;
} Calibration_AIN;

typedef struct ScanList_t {
  uint8_t channel;      // 0 = channel 0,    1 = channel 1
  uint8_t range;
  uint8_t last_channel; // 1 = last channel,   0 = not last channel
  uint8_t mode;         // 1 = Calibration Mode,  0 = Cahnnel input is tied to ACD
} ScanList;

typedef struct TriggerConfig_t {
  uint8_t options;
  uint8_t triggerChannel;
  uint8_t lowThreshold[2];
  uint8_t highThreshold[2];
} TriggerConfig;

typedef struct usbDevice2020_t {
  libusb_device_handle *udev;                       // libusb 1.0 handle
  Calibration_AIN table[NCHAN_2020][NGAINS_2020];   // calibration coefficients ADC
  ScanList scanList[NCHAN_2020];                    // scan list used to configure the A/D channels.
  int lastElement;                                  // last element of the scan list
  uint32_t count;
  uint32_t retrig_count;
  uint8_t options;
  uint8_t nChannels;
  double frequency;                    // frequency of the scan  (0 for external clock)
  uint16_t packet_size;                // number of samples to return from FIFO
  uint16_t status;                     // status word of the device
  int bytesToRead;                     // number of bytes left to read in the scan
  uint8_t mode;                        /* mode bits:
                                        bit 0:   0 = counting mode,  1 = CONTINUOUS_READOUT
                                        bit 1:   1 = SINGLEIO
                                        bit 2:   1 = use packet size passed scanData->packet_size
                                       */

} usbDevice2020;

/* function prototypes for the USB-2020 */
void usbDTristateW_USB2020(libusb_device_handle *udev, uint16_t value);
uint16_t usbDTristateR_USB2020(libusb_device_handle *udev);
uint16_t usbDPort_USB2020(libusb_device_handle *udev);
void usbDLatchW_USB2020(libusb_device_handle *udev, uint16_t value);
uint16_t usbDLatchR_USB2020(libusb_device_handle *udev);
void usbBlink_USB2020(libusb_device_handle *udev, uint8_t count);
void cleanup_USB2020( libusb_device_handle *udev);
void usbTemperature_USB2020(libusb_device_handle *udev, float *temperature);
void usbGetSerialNumber_USB2020(libusb_device_handle *udev, char serial[9]);
void usbReset_USB2020(libusb_device_handle *udev);
void usbCalConfig_USB2020(libusb_device_handle *udev, uint8_t voltage);
void usbFPGAConfig_USB2020(libusb_device_handle *udev);
void usbFPGAData_USB2020(libusb_device_handle *udev, uint8_t *data, uint8_t length);
void usbFPGAVersion_USB2020(libusb_device_handle *udev, uint16_t *version);
uint16_t usbStatus_USB2020(libusb_device_handle *udev);
void usbInit_USB2020(libusb_device_handle *udev);
void usbMemoryR_USB2020(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemoryW_USB2020(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemAddressR_USB2020(libusb_device_handle *udev, uint16_t address);
void usbMemAddressW_USB2020(libusb_device_handle *udev, uint16_t address);
void usbMemWriteEnable_USB2020(libusb_device_handle *udev);
void usbTriggerConfig_USB2020(libusb_device_handle *udev, TriggerConfig *triggerConfig);
void usbTriggerConfigR_USB2020(libusb_device_handle *udev, TriggerConfig *triggerConfig);
void usbTemperature_USB2020(libusb_device_handle *udev, float *temperature);
void usbGetSerialNumber_USB2020(libusb_device_handle *udev, char serial[9]);
uint16_t usbAIn_USB2020(libusb_device_handle *udev, uint16_t channel);
void usbAInScanStart_USB2020(libusb_device_handle *udev, usbDevice2020 *usb2020);
void usbAInScanStop_USB2020(libusb_device_handle *udev);
int usbAInScanRead_USB2020(libusb_device_handle *udev, usbDevice2020 *usb2020, uint16_t *data);
void usbAInConfig_USB2020(libusb_device_handle *udev, usbDevice2020 *usb2020);
void usbAInConfigR_USB2020(libusb_device_handle *udev, usbDevice2020 *usb2020);
void usbAInScanClearFIFO_USB2020(libusb_device_handle *udev);
void usbBuildGainTable_USB2020(libusb_device_handle *udev, Calibration_AIN table[NCHAN_2020][NGAINS_2020]);
void usbCalDate_USB2020(libusb_device_handle *udev, struct tm *date);
double volts_USB2020(const uint8_t gain, uint16_t value);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif //USB_2020_H
