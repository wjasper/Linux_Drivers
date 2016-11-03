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

#ifndef USB_2408_H

#define USB_2408_H
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>

#define USB2408_PID      (0x00fd)
#define USB2408_2AO_PID  (0x00fe)

/* Commands and HID Report ID for USB 2408  */
/* Digital I/O Commands */
#define DIN              (0x00)     // Read digital port
#define DOUT             (0x01)     // Read/Write digital port drive register

/* Analog Input Commands */
#define AIN              (0x10)     // Read analog input channel
#define AIN_SCAN_START   (0x11)     // Start analog input scan
#define AIN_SCAN_STOP    (0x12)     // Stop analog input scan
#define AIN_SCAN_STATUS  (0x13)     // Read analog input scan status
#define AIN_SCAN_QUEUE   (0x14)     // Read/Write analog input channel gain queue
#define SETTLING_SCAN    (0x16)     // Front end settling test

/* Analog Output Commands */
#define AOUT             (0x18)     // Write analog output channel
#define AOUT_SCAN_START  (0x19)     // Analog output scan start
#define AOUT_SCAN_STOP   (0x1A)     // Analog output scan stop
#define AOUT_SCAN_STATUS (0x1B)     // Analog output scan status

/* Counter Commands */
#define COUNTER          (0x20)     // Read/reset event counter

/* Memory Commands */
#define MEMORY           (0x30)     // Read/Write EEPROM

/* Miscellaneous Commands */
#define RESET            (0x40)     // Reset the device
#define BLINK_LED        (0x41)     // Causes LED to blink
#define CJC              (0x42)     // Read CJC sensor values
#define CAL_CONFIG       (0x43)     // Configure calibration source
#define GET_STATUS       (0x44)     // Read device status
#define ADCAL            (0x45)     // Perform A/D self-calibration
#define TC_CAL           (0x46)     // Measure TC calibration source
#define SERIAL           (0x48)     // Read/Write USB Serial Number
#define VERSION          (0x49)     // Read micro firmware versions

/* Firmware Update */
#define UPDATE_MODE      (0x50)     // Put device into firmware update mode
#define UPDATE_ADDRESS   (0x51)     // Read/Write the firmware address
#define UPDATE_DATA      (0x52)     // Read/Write the firmware data
#define UPDATE_VERSION   (0x53)     // Read the firmware update code version
#define UPDATE_ERASE     (0x54)     // Erase the firmware
#define MBD_COMMAND      (0x80)     // Text-baseed MBD command
#define MBD_RAW          (0x81)     // Raw MBD response

//  Modes
#define DIFFERENTIAL     (0x0)      // Voltage - differential
#define SE_HIGH          (0x1)      // Voltage - single-ended high channel
#define SE_LOW           (0x2)      // Voltage - single-ended low channel
#define DAC_READBACK     (0x3)      // Voltage - D/A readback
#define THERMOCOUPLE     (0x4)      // Thermocouple
#define CAL_OFFSET       (0x5)      // AIn Offset Calibration
#define CAL_GAIN         (0x6)      // AIn Gain Calibration
#define TC_OFFSET        (0x7)      // TC Offset Calibration
#define TC_GAIN_POS      (0x8)      // TC Gain Calibration Positive
#define TC_GAIN_NEG      (0x9)      // TC Gain Calibration Negative
#define BURNOUT          (0xa)      // Thermocouple without burnout detect

// Gain Ranges
//#define BP_20V           (0x0)     // +/- 20V
#define BP_10V           (0x1)     // +/- 10V
#define BP_5V            (0x2)     // +/- 5V
#define BP_2_5V          (0x3)     // +/- 2.5V
#define BP_1_25V         (0x4)     // +/- 1.25V
#define BP_625V          (0x5)     // +/- 0.625V
#define BP_312V          (0x6)     // +/- 0.3125V
#define BP_156V          (0x7)     // +/- 0.15625V
#define BP_078V          (0x8)     // +/- 0.078125V Voltage (all), Thermocouple

// Rates
#define HZ30000          (0)       // 30,000 S/s
#define HZ15000          (1)       // 15,000 S/s
#define HZ7500           (2)       //  7,500 S/s
#define HZ3750           (3)       //  3,750 S/s
#define HZ2000           (4)       //  2,000 S/s
#define HZ1000           (5)       //  1,000 S/s
#define HZ500            (6)       //    500 S/s
#define HZ100            (7)       //    100 S/s
#define HZ60             (8)       //     60 S/s
#define HZ50             (9)       //     50 S/s
#define HZ30             (10)      //     30 S/s
#define HZ25             (11)      //     25 S/s
#define HZ15             (12)      //     15 S/s
#define HZ10             (13)      //     10 S/s
#define HZ5              (14)      //      5 S/s
#define HZ2_5            (15)      //    2.5 S/s

#define COUNTER0         0x0       //  Counter 0
#define COUNTER1         0x1       //  Counter 1
  
#define NCHAN_2408      16       // max number of A/D channels in the device (8 Differential)
#define nCJCGrad_2408    8       // max number of CJC Gradient array elecments
#define NGAINS_2408      9       // max number of gain levels (analog input)
#define MAX_QUEUE_SIZE  64       // max number of entries in the AIN scan queue
#define NCHAN_AO_2408    2       // number of analog output channels

typedef struct AInScanQueue_t {
  uint8_t count;
  struct queue_t {
    uint8_t channel;
    uint8_t mode;
    uint8_t range;
    uint8_t rate;
  } queue[MAX_QUEUE_SIZE];
} AInScanQueue;

#define INPUT_SCAN_RUNNING (0x1)
#define INPUT_FIFO_FULL (0x2)
#define INPUT_PACER_SHORT (0x4)

#define OUTPUT_SCAN_RUNNING (0x1)
#define OUTPUT_SCAN_UNDERRUN (0x2)

void cleanup_USB2408(libusb_device_handle *udev);
void usbBlink_USB2408(libusb_device_handle *udev,  uint8_t count);
uint8_t usbDIn_USB2408(libusb_device_handle *udev, uint8_t port);
void usbDOut_USB2408(libusb_device_handle *udev, uint8_t value, uint8_t port);
uint8_t usbDOutR_USB2408(libusb_device_handle *udev, uint8_t port);
int  usbAIn_USB2408(libusb_device_handle *udev, uint8_t channel, uint8_t mode, uint8_t range, uint8_t rate, uint8_t *flags);
void usbAInScanStop_USB2408(libusb_device_handle *udev);
uint8_t usbAInScanStatus_USB2408(libusb_device_handle *udev, uint16_t *depth);
void usbAInScanQueueWrite_USB2408(libusb_device_handle *udev, AInScanQueue *queue);
void usbAInScanQueueRead_USB2408(libusb_device_handle *udev, AInScanQueue *queue);
void usbAInScanStart_USB2408(libusb_device_handle *udev, double frequency, uint16_t count, uint8_t packet_size);
int usbAInScanRead_USB2408(libusb_device_handle *udev, uint16_t count, uint8_t nChan, int32_t *data);
void usbGetSerialNumber_USB2408(libusb_device_handle *udev, char serial[9]);
void usbSetSerialNumber_USB2408(libusb_device_handle *udev, char serial[9]);
void usbGetVersion_USB2408(libusb_device_handle *udev, uint16_t version[4]);
void usbReset_USB2408(libusb_device_handle *udev);
void usbCounterInit_USB2408(libusb_device_handle *udev, uint8_t counter);
uint32_t usbCounter_USB2408(libusb_device_handle *udev, uint8_t counter);
uint8_t usbStatus_USB2408(libusb_device_handle *udev);
void usbCJC_USB2408(libusb_device_handle *udev, float temp[8]);  
void usbReadMemory_USB2408(libusb_device_handle *udev, uint16_t length,  uint16_t address, uint8_t *data);
void usbWriteMemory_USB2408(libusb_device_handle *udev, uint16_t length,  uint16_t address, uint8_t *data);
void usbCalConfig_USB2408(libusb_device_handle *udev, uint8_t value);
uint8_t usbADCal_USB2408(libusb_device_handle *udev);
void usbTCCalMeasure_USB2408(libusb_device_handle *udev, uint8_t value);
double volts_USB2408(const int gain, const int value);
void usbBuildGainTable_USB2408(libusb_device_handle *udev, double table[NGAINS_2408][2]);
void usbBuildCJCGradientTable_USB2408(libusb_device_handle *udev, float table_CJCGrad[nCJCGrad_2408]);
void usbUpdateMode_USB2408(libusb_device_handle *udev, uint8_t micro);

/* USB 2408_2AO specific */
void usbBuildGainTable_USB2408_2AO(libusb_device_handle *udev, double table_AO[NCHAN_AO_2408][2]);
void usbCalDate_USB2408(libusb_device_handle *udev, struct tm *date);
void usbAOutScanStop_USB2408_2AO(libusb_device_handle *udev);
void usbAOutScanStart_USB2408_2AO(libusb_device_handle *udev, double frequency, uint16_t scans, uint8_t options);
uint8_t  usbAOutScanStatus_USB2408_2AO(libusb_device_handle *udev, uint16_t *depth);
void usbAOut_USB2408_2AO(libusb_device_handle *udev, int channel, double voltage, double table_A0[NCHAN_AO_2408][2]);
void voltsTos16_USB2408_2AO(double *voltage, int16_t *data, int nSamples, double table_AO[]);
double usbAInMinPacerPeriod_USB2408(libusb_device_handle *udev);
int  int24ToInt(int int24val);
double tc_temperature_USB2408(libusb_device_handle *udev, int tc_type, uint8_t channel);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif
#endif // USB_2408_H
