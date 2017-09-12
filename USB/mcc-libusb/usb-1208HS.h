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

#ifndef USB_1208HS_H

#define USB_1208HS_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB1208HS_PID     (0x00c4)
#define USB1208HS_2AO_PID (0x00c5)
#define USB1208HS_4AO_PID (0x00c6)

/* Status bit values */
#define AIN_SCAN_RUNNING   (0x1 << 1)
#define AIN_SCAN_OVERRUN   (0x1 << 2)
#define AOUT_SCAN_RUNNING  (0x1 << 3)
#define AOUT_SCAN_UNDERRUN (0x1 << 4)
#define AIN_SCAN_DONE      (0x1 << 5)
#define AOUT_SCAN_DONE     (0x1 << 6)
#define FPGA_CONFIGURED    (0x1 << 8)
#define FPGA_CONFIG_MODE   (0x1 << 9)

/* Counter Timer */
#define COUNTER0         0x0      //  Counter 0
#define COUNTER1         0x1      //  Counter 1

#define NCHAN_1208HS          8   // max number of A/D channels in the device
#define NGAINS_1208HS         4   // max number of gain levels (analog input)
#define NMODE                 4   // max number of configuration modes
#define NCHAN_AO_1208HS       4   // number of analog output channels
#define MAX_PACKET_SIZE_HS  512   // max packet size for HS device
#define MAX_PACKET_SIZE_FS   64   // max packet size for FS device


/* Analog Input Scan and Modes */
#define SINGLE_ENDED        0     // 8 single-ended inputs
#define PSEUDO_DIFFERENTIAL 1     // 4 pseudo differential inputs
#define DIFFERENTIAL        2     // 4 true differential inputs
#define PSEUDO_DIFFERENTIAL_UP    // 7 pseudo differential inputs
#define PACKET_SIZE         512   // max bulk transfer size in bytes

/* Analog Input Scan Options */
#define CHAN0  (0x1 << 0)
#define CHAN1  (0x1 << 1)
#define CHAN2  (0x1 << 2)
#define CHAN3  (0x1 << 3)
#define CHAN4  (0x1 << 4)
#define CHAN5  (0x1 << 5)
#define CHAN6  (0x1 << 6) 
#define CHAN7  (0x1 << 7)

#define BURST_MODE   0x1
#define CONTINUOUS   0x2 
#define TRIGGER_MODE 0x8
#define DEBUG_MODE   0x20
#define RETRIG_MODE  0x40

#define BP_10V   0
#define BP_5V    1
#define BP_2_5V  2
#define UP_10V   3

#define BP_20V_DE 0
#define BP_10V_DE 1
#define BP_5V_DE  2

/* Ananlog Output Scan Options */
#define AO_CHAN0       0x1   // Include Channel 0 in output scan
#define AO_CHAN1       0x2   // Include Channel 1 in output scan
#define AO_CHAN2       0x4   // Include Channel 2 in output scan
#define AO_CHAN3       0x8   // Include Channel 3 in output scan
#define AO_TRIG        0x10  // Use Trigger
#define AO_RETRIG_MODE 0x20  // Retrigger Mode

typedef struct timerParams_t {
  uint32_t period;
  uint32_t pulseWidth;
  uint32_t count;
  uint32_t delay;
} timerParams;

/* function prototypes for the USB-1208HS */
uint16_t  usbDTristateR_USB1208HS(libusb_device_handle *udev);
void usbDTristateW_USB1208HS(libusb_device_handle *udev, uint16_t value);
uint16_t usbDPort_USB1208HS(libusb_device_handle *udev);
uint16_t usbDLatchR_USB1208HS(libusb_device_handle *udev);
void usbDLatchW_USB1208HS(libusb_device_handle *udev, uint16_t data);
void cleanup_USB1208HS(libusb_device_handle *udev);
void usbBlink_USB1208HS(libusb_device_handle *udev, uint8_t count);
void usbTemperature_USB1208HS(libusb_device_handle *udev, float *temperature);
void usbGetSerialNumber_USB1208HS(libusb_device_handle *udev, char serial[9]);
void usbReset_USB1208HS(libusb_device_handle *udev);
void usbFPGAConfig_USB1208HS(libusb_device_handle *udev);
void usbFPGAData_USB1208HS(libusb_device_handle *udev, uint8_t *data, uint8_t length);
void usbFPGAVersion_USB1208HS(libusb_device_handle *udev, uint16_t *version);
uint16_t usbStatus_USB1208HS(libusb_device_handle *udev);
void usbMemoryR_USB1208HS(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemoryW_USB1208HS(libusb_device_handle *udev, uint8_t *data, uint16_t length);
void usbMemAddressR_USB1208HS(libusb_device_handle *udev, uint16_t address);
void usbMemAddressW_USB1208HS(libusb_device_handle *udev, uint16_t address);
void usbMemWriteEnable_USB1208HS(libusb_device_handle *udev);
void usbTriggerConfig_USB1208HS(libusb_device_handle *udev, uint8_t options);
void usbTriggerConfigR_USB1208HS(libusb_device_handle *udev, uint8_t *options);
void usbInit_1208HS(libusb_device_handle *udev);
void usbCounterInit_USB1208HS(libusb_device_handle *udev, uint8_t counter);
uint32_t usbCounter_USB1208HS(libusb_device_handle *udev, uint8_t counter);
void usbTimerControlR_USB1208HS(libusb_device_handle *udev, uint8_t *control);
void usbTimerControlW_USB1208HS(libusb_device_handle *udev, uint8_t control);
void usbTimerPeriodR_USB1208HS(libusb_device_handle *udev, uint32_t *period);
void usbTimerPeriodW_USB1208HS(libusb_device_handle *udev, uint32_t period);
void usbTimerPulseWidthR_USB1208HS(libusb_device_handle *udev, uint32_t *pulseWidth);
void usbTimerPulseWidthW_USB1208HS(libusb_device_handle *udev, uint32_t pulseWidth);
void usbTimerCountR_USB1208HS(libusb_device_handle *udev, uint32_t *count);
void usbTimerCountW_USB1208HS(libusb_device_handle *udev, uint32_t count);
void usbTimerDelayR_USB1208HS(libusb_device_handle *udev, uint32_t *delay);
void usbTimerDelayW_USB1208HS(libusb_device_handle *udev, uint32_t delay);
void usbTimerParamsR_USB1208HS(libusb_device_handle *udev, timerParams *params);
void usbTimerParamsW_USB1208HS(libusb_device_handle *udev, timerParams *params);
uint16_t usbAIn_USB1208HS(libusb_device_handle *udev, uint8_t channel);
void usbAInConfig_USB1208HS(libusb_device_handle *udev, uint8_t mode, uint8_t range[NCHAN_1208HS]);
void usbAInConfigR_USB1208HS(libusb_device_handle *udev, uint8_t *mode, uint8_t range[NCHAN_1208HS]);
void usbAInScanStop_USB1208HS(libusb_device_handle *udev);
void usbAInScanStart_USB1208HS(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t channels, uint8_t packet_size, uint8_t options);
int usbAInScanRead_USB1208HS(libusb_device_handle *udev, int nScan, int nChan,  uint16_t *data, int options);
void usbAOut_USB1208HS(libusb_device_handle *udev, uint8_t channel, double voltage, float table_AO[NCHAN_AO_1208HS][2]);
void usbAOutR_USB1208HS(libusb_device_handle *udev, uint8_t channel, double *voltage, float table_AO[NCHAN_AO_1208HS][2]);
void usbAOutScanStop_USB1208HS(libusb_device_handle *udev);
void usbAOutScanClearFIFO_USB1208HS(libusb_device_handle *udev);
void usbAOutScanStart_USB1208HS(libusb_device_handle *udev, uint32_t count, uint32_t retrig_count, double frequency, uint8_t options);
int usbAOutScanWrite_USB1208HS(libusb_device_handle *udev, uint32_t count, uint16_t *sdata);
void usbBuildGainTable_USB1208HS(libusb_device_handle *udev, float table[NMODE][NGAINS_1208HS][2]);
void usbBuildGainTable_USB1208HS_4AO(libusb_device_handle *udev, float table_AO[NCHAN_AO_1208HS][2]);
uint16_t voltsTou12_USB1208HS_AO(double volts, int channel, float table_AO[NCHAN_AO_1208HS][2]);
double volts_USB1208HS(const uint8_t mode, const uint8_t gain, uint16_t value);  

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // USB_1208HS_H
