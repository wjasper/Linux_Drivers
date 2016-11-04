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

#ifndef USB_7204_H

#define USB_7204_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB7204_PID    (0x00f0)

/* Commands and USB Report ID for USB-7204 */

/* MBD Control Transfers */
#define STRING_MESSAGE (0x80)  // Send string messages to the device
#define RAW_DATA       (0x81)  // Return RAW data from the device

/* UL Control TRansfers */
    // Digital I/O Commands
#define DCONFIG_PORT    (0x0)   // Configure digital port
#define DPORT           (0x2)   // Read/Write digital port
    // Analog Input Commands
#define AIN             (0x10)  // Read analog input channel
#define AIN_SCAN        (0x11)  // Scan analog input channels
#define AIN_STOP        (0x12)  // Stop ananlog input scan
#define ALOAD_QUEUE     (0x13)  // Load the channel/gain queue
    // Analog Output Commands
#define AOUT            (0x14)  // Write analog output channel
#define AOUT_SCAN       (0x15)  // Scan analog output
#define AOUT_STOP       (0x16)  // Stop output scan
#define AOUT_RESET      (0x17)  // Clear USB Buffer, FIFO and Status/Error Flags
    // Counter Commands
#define COUNTER         (0x20)  // Read/initialize counter
    // Memory Commands
#define MEMORY          (0x30)  // Read/Write memory
#define MEMORY_ADDR     (0x31)  // Read/Write memory address
    // Miscellaneous Commands
#define BLINK_LED       (0x40)  // Cause LED to blink
#define RESET           (0x41)  // Force a device reset
#define TRIGGER_CONFIG  (0x42)  // Configure external trigger
#define SYNC_CONFIG     (0x43)  // Configure sync input/output
#define STATUS          (0x44)  // Retrieve device status
#define CAL_CONFIG      (0x45)  // Control CAL MUX
#define SERIAL          (0x48)  // Read/Write serial number
      // Code Update Commands
#define UPDATE_MODE     (0x50)  // Put device into code update mode
#define UPDATE_ADDR     (0x51)  // Set code download address
#define UPDATE_DATA     (0x52)  // Download code image
#define UPDATE_CHECKSUM (0x53)  // Read/download checksum
#define UPDATE_VERSION  (0x5a)  // Read the downloader version

#define NCHAN_USB7204     8  // max number of ADC channels
#define NGAINS_USB7204    8  // max number of gain levels

#define DIO_DIR_IN  (0x01)
#define DIO_DIR_OUT (0x00)

// DIO Ports
#define PORT0   0
#define PORT1   1

// MODE
#define NMODE 2    // number of modes
#define DF 0       // Differential
#define SE 1       // Single Ended

// Gain Ranges
#define BP_20_00V  (0x0)     // Single Ended +/- 20.0 V only
#define BP_10_00V  (0x1)     // +/- 10.00 V
#define BP_5_00V   (0x2)     // +/- 5.00 V
#define BP_4_00V   (0x3)     // +/- 4.00 V
#define BP_2_50V   (0x4)     // +/- 2.50 V
#define BP_2_00V   (0x5)     // +/- 2.00 V
#define BP_1_25V   (0x6)     // +/- 1.25 V
#define BP_1_00V   (0x7)     // +/- 1.00 V

typedef struct Calibration_AIN_t {
  float slope;
  float intercept;
} Calibration_AIN;

typedef struct Calibration_AOUT_t {
  float slope;
  float intercept;
} Calibration_AOUT;

typedef struct LoadQueue_t {
  uint8_t num;     // the number of channel/gain pairs to follow (max 16)
  uint8_t chan_0;  // the channel number of the first queue element (0-15)
  uint8_t gain_0;  // the gain range of the first queue element (0-7)
  uint8_t chan_1;  // the channel number of the first queue element (0-15)
  uint8_t gain_1;  // the gain range of the first queue element (0-7)
  uint8_t chan_2;  // the channel number of the first queue element (0-15)
  uint8_t gain_2;  // the gain range of the first queue element (0-7)
  uint8_t chan_3;  // the channel number of the first queue element (0-15)
  uint8_t gain_3;  // the gain range of the first queue element (0-7)
  uint8_t chan_4;  // the channel number of the first queue element (0-15)
  uint8_t gain_4;  // the gain range of the first queue element (0-7)
  uint8_t chan_5;  // the channel number of the first queue element (0-15)
  uint8_t gain_5;  // the gain range of the first queue element (0-7)
  uint8_t chan_6;  // the channel number of the first queue element (0-15)
  uint8_t gain_6;  // the gain range of the first queue element (0-7)
  uint8_t chan_7;  // the channel number of the first queue element (0-15)
  uint8_t gain_7;  // the gain range of the first queue element (0-7)
  uint8_t chan_8;  // the channel number of the first queue element (0-15)
  uint8_t gain_8;  // the gain range of the first queue element (0-7)
  uint8_t chan_9;  // the channel number of the first queue element (0-15)
  uint8_t gain_9;  // the gain range of the first queue element (0-7)
  uint8_t chan_10;  // the channel number of the first queue element (0-15)
  uint8_t gain_10;  // the gain range of the first queue element (0-7)
  uint8_t chan_11;  // the channel number of the first queue element (0-15)
  uint8_t gain_11;  // the gain range of the first queue element (0-7)
  uint8_t chan_12;  // the channel number of the first queue element (0-15)
  uint8_t gain_12;  // the gain range of the first queue element (0-7)
  uint8_t chan_13;  // the channel number of the first queue element (0-15)
  uint8_t gain_13;  // the gain range of the first queue element (0-7)
  uint8_t chan_14;  // the channel number of the first queue element (0-15)
  uint8_t gain_14;  // the gain range of the first queue element (0-7)
  uint8_t chan_15;  // the channel number of the first queue element (0-15)
  uint8_t gain_15;  // the gain range of the first queue element (0-7)
} LoadQueue;

// options for AInScan
#define AIN_EXECUTION      0x1   // 1 = single execution,        0 = continuous execution
#define AIN_BURST_MODE     0x2   // 1 = immediate transfer mode, 0 = block transfer mode
#define AIN_EXTERN_TRIGGER 0x4   // 1 = Use External Trigger
#define AIN_GAIN_QUEUE     0x10  // 1 = use channel gain queue,  0 = use channel parameters specified
#define AIN_RETRIGGER      0x20  // 1 = retrigger mode,          0 = normal mode
#define AIN_DEGBUG_MODE    0x40  // 1 = debug mode
#define AIN_STALL          0x80  // 1 = stall

/* Status bit values */
#define SYNC_SLAVE            (0x1)     
#define SYNC_MASTER           (0x0)
#define EXT_TRIG_FAILING_EDGE (0)
#define EXT_TRIG_RAISING_EDGE (1)
#define AIN_SCAN_OVERRUN      (0x1 << 2)    // overrun error during AInScan, cleared with starting new scan
#define GATE                  (0x1 << 3)    // Gated sync (only used when Sync = slave[0])
#define UNDERRUN              (0x1 << 4)    // underrun error during AOutScan, cleared when starting new scan

/* function prototypes for the USB-7204 */
void usbDConfigPortR_USB7204(libusb_device_handle *udev, uint8_t port, uint8_t *direction);
void usbDConfigPort_USB7204(libusb_device_handle *udev, uint8_t port, uint8_t direction);
uint8_t usbDPortR_USB7204(libusb_device_handle *udev, uint8_t port);
void usbDPortW_USB7204(libusb_device_handle *udev, uint8_t port, uint8_t value);
uint16_t usbAIn_USB7204(libusb_device_handle *udev, uint8_t mode, uint8_t channel, uint8_t range);
void usbAInScan_USB7204(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count, float *frequency, uint8_t options);
int usbAInScanRead_USB7204(libusb_device_handle *udev, int nScan, int nChan, uint16_t *data);
void usbAInStop_USB7204(libusb_device_handle *udev);
void usbAInLoadQueue_USB7204(libusb_device_handle *udev, LoadQueue *gainArray);
void usbAOut_USB7204(libusb_device_handle *udev, uint8_t channel, float vout);
int usbAOutScan_USB7204(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count, float *frequency, uint16_t data[], uint8_t options);
void usbAOutScanStop_USB7204(libusb_device_handle *udev);
void usbAOutScanReset_USB7204(libusb_device_handle *udev);
void usbInitCounter_USB7204(libusb_device_handle *udev);
uint32_t usbReadCounter_USB7204(libusb_device_handle *udev);
void usbReadMemory_USB7204(libusb_device_handle *udev, uint8_t count, uint8_t* data);
void usbWriteMemory_USB7204(libusb_device_handle *udev, uint8_t count, uint8_t* data);
uint16_t usbReadMemoryAddress_USB7204(libusb_device_handle *udev);
void usbWriteMemoryAddress_USB7204(libusb_device_handle *udev, uint16_t address);
void usbBlinkLED_USB7204(libusb_device_handle *udev, uint8_t count);
void usbReset_USB7204(libusb_device_handle *udev, uint8_t type);
void usbTriggerConfig_USB7204(libusb_device_handle *udev, uint8_t type);
void usbSyncConfig_USB7204(libusb_device_handle *udev, uint8_t type);
uint16_t usbStatus_USB7204(libusb_device_handle *udev);
void usbCalConfig_USB7204(libusb_device_handle *udev, uint8_t setting);
void usbGetSerialNumber_USB7204(libusb_device_handle *udev, char serial[9]);
void usbSetSerialNumber_USB7204(libusb_device_handle *udev, char serial[9]);
void usbUpdateMode_USB7204(libusb_device_handle *udev);
void usbUpdateAddress_USB7204(libusb_device_handle *udev, uint8_t address[3]);
void usbUpdateData_USB7204(libusb_device_handle *udev, uint8_t count, uint8_t *data);
void usbUpdateChecksum_USB7204(libusb_device_handle *udev, uint16_t *checksum);
void usbBuildGainTable_USB7204(libusb_device_handle *udev, Calibration_AIN table[NMODE][NGAINS_USB7204][NCHAN_USB7204]);
double volts_USB7204(uint16_t value, uint8_t range);
void getMFGCAL_USB7204(libusb_device_handle *udev, struct tm *date);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // USB_7204_H
