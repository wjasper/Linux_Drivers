/*
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

#ifndef USB_TC_H
#define USB_TC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define USB_TC_PID (0x0090)

#define DIO_DIR_IN  (0x01)
#define DIO_DIR_OUT (0x00)

/* Commands and HID Report ID for USB TC  */

// Digital I/O Commands
#define DCONFIG     (0x01)     // Configure digital port
#define DCONFIG_BIT (0x02)     // Configure individual digital port bits
#define DIN         (0x03)     // Read digital port
#define DOUT        (0x04)     // Write digital port
#define DBIT_IN     (0x05)     // Read digital port bit
#define DBIT_OUT    (0x06)     // Write digital port bit

#define TIN         (0x18)     // Read input channel
#define TIN_SCAN    (0x19)     // Read multiple input channels

// Memory Commands
#define MEM_READ    (0x30)     // Read Memory
#define MEM_WRITE   (0x31)     // Write Memory

// Miscellaneous Commands
#define BLINK_LED          (0x40) // Causes LED to blink
#define RESET              (0x41) // Reset USB interface
#define GET_STATUS         (0x44) // Get device status
#define SET_ITEM           (0x49) // Set a configuration item
#define GET_ITEM           (0x4A) // Get a configuration item
#define CALIBRATE          (0x4B) // Perform a channel calibration
#define GET_BURNOUT_STATUS (0x4C) // Get thermocouple burnout detection status

// Code Update Commands
#define PREPARE_DOWNLOAD   (0x50) // Prepare for program memory download
#define WRITE_CODE         (0x51) // Write program memory
#define WRITE_SERIAL       (0x53) // Write a new serial number to device
#define READ_CODE          (0x55) // Read program memory

// Channels
#define CH0  (0x0)  // Channel 0
#define CH1  (0x1)  // Channel 1
#define CH2  (0x2)  // Channel 2
#define CH3  (0x3)  // Channel 3
#define CH4  (0x4)  // Channel 4
#define CH5  (0x5)  // Channel 5
#define CH6  (0x6)  // Channel 6
#define CH7  (0x7)  // Channel 7
#define CJC0 (0x80) // Cold Junction Compensator 0
#define CJC1 (0x81) // Cold Junction Compensator 1

// Configuration Items
#define ADC_0 (0x0)  // Setting for ADC 0
#define ADC_1 (0x1)  // Setting for ADC 1
#define ADC_2 (0x2)  // Setting for ADC 2
#define ADC_3 (0x3)  // Setting for ADC 3

// Sub Items
#define SENSOR_TYPE     (0x00) // Sensor type  Read Only
#define CONNECTION_TYPE (0x01) // Connection type - RTD & Thermistor
#define FILTER_RATE     (0x02) // Filter update rate
#define EXCITATION      (0x03) // Currect excitation
#define VREF            (0x04) // Measured Vref value
#define I_value_0       (0x05) // Measured I value @ 10uA
#define I_value_1       (0x06) // Measured I value @ 210uA
#define I_value_2       (0x07) // Measured I value @ 10uA (3 wire connection)
#define V_value_0       (0x08) // Measured V value @ 10uA
#define V_value_1       (0x09) // Measured V value @ 210uA
#define V_value_2       (0x0a) // Measured V value @ 210uA (3 wire connection)
#define CH_0_TC         (0x10) // Thermocouple type for channel 0
#define CH_1_TC         (0x11) // Thermocouple type for channel 1
#define CH_0_GAIN       (0x12) // Channel 0 gain value
#define CH_1_GAIN       (0x13) // Channel 1 gain value
#define CH_0_COEF_0     (0x14) // Coefficient 0
#define CH_1_COEF_0     (0x15) // Coefficient 0
#define CH_0_COEF_1     (0x16) // Coefficient 1
#define CH_1_COEF_1     (0x17) // Coefficient 1
#define CH_0_COEF_2     (0x18) // Coefficient 2
#define CH_1_COEF_2     (0x19) // Coefficient 2
#define CH_0_COEF_3     (0x1a) // Coefficient 3
#define CH_1_COEF_3     (0x1b) // Coefficient 3

// Possible Values
#define RTD           (0x0)
#define THERMISTOR    (0x1)
#define THERMOCOUPLE  (0x2)
#define SEMICONDUCTOR (0x3)
#define DISABLED      (0x4)

#define FREQ_500_HZ   (0x1)
#define FREQ_250_HZ   (0x2)
#define FREQ_125_HZ   (0x3)
#define FREQ_62_5_HZ  (0x4)
#define FREQ_50_HZ    (0x5)
#define FREQ_39_2_HZ  (0x6)
#define FREQ_33_3_HZ  (0x7)
#define FREQ_19_6_HZ  (0x8)
#define FREQ_16_7_HZ  (0x9)
//#define FREQ_16_7_HZ  (0xa)
#define FREQ_12_5_HZ  (0xb)
#define FREQ_10_HZ    (0xc)
#define FREQ_8_33_HZ  (0xd)
#define FREQ_6_25_HZ  (0xe)
#define FREQ_4_17_HZ  (0xf)

#define GAIN_1X       (0x0)
#define GAIN_2X       (0x1)
#define GAIN_4X       (0x2)
#define GAIN_8X       (0x3)
#define GAIN_16X      (0x4)
#define GAIN_32X      (0x5)
#define GAIN_64X      (0x6)
#define GAIN_128X     (0x7)

/* For connection types RTD & thermistor */
#define TWO_WIRE_ONE_SENSOR (0x0)
#define TWO_WIRE_TWO_SENSOR (0x1)
#define THREE_WIRE          (0x2)
#define FOUR_WIRE           (0x3)

/* Current excitation values */
#define EXCITATION_OFF       (0x0)
#define MU_A_10              (0x1)  //  10 micro Amps
#define MU_A_210             (0x2)  // 210 micro Amps

/* function prototypes for the USB-TC */
void usbDConfigPort_USBTC(hid_device *hid, uint8_t direction);
void usbDConfigBit_USBTC(hid_device *hid, uint8_t bit_num, uint8_t direction);
void usbDIn_USBTC(hid_device *hid, uint8_t* value);
void usbDInBit_USBTC(hid_device *hid, uint8_t bit_num, uint8_t* value);
void usbDOut_USBTC(hid_device *hid, uint8_t value);
void usbDOutBit_USBTC(hid_device *hid, uint8_t bit_num, uint8_t value);
void usbTin_USBTC(hid_device *hid, uint8_t channel, uint8_t units, float *value);
void usbTinScan_USBTC(hid_device *hid, uint8_t start_chan, uint8_t end_chan, uint8_t units, float value[]);

void usbReadMemory_USBTC(hid_device *hid, uint16_t address, uint8_t type, uint8_t count, uint8_t memory[]);
int usbWriteMemory_USBTC(hid_device *hid, uint16_t address, uint8_t type, uint8_t count, uint8_t data[]);
void usbBlink_USBTC(hid_device *hid);
int usbReset_USBTC(hid_device *hid);
uint8_t usbGetStatus_USBTC(hid_device *hid);
int usbGetItem_USBTC(hid_device *hid, uint8_t item, uint8_t subitem, void *value);
void usbSetItem_USBTC(hid_device *hid, uint8_t item, uint8_t subitem, uint32_t value);
void usbCalibrate_USBTC(hid_device *hid);
uint8_t  usbGetBurnoutStatus_USBTC(hid_device *hid, uint8_t mask);
void usbPrepareDownload_USBTC(hid_device *hid, uint8_t micro);
void usbWriteCode_USBTC(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);
int usbReadCode_USBTC(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);
void usbWriteSerial_USBTC(hid_device *hid, uint8_t serial[8]);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_TC_H
