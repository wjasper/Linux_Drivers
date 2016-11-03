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

#ifndef USB_1408FS_H
#define USB_1408FS_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB1408FS_PID (0x00a1)

#define DIO_PORTA (0x00)
#define DIO_PORTB (0x01)

#define DIO_DIR_IN  (0x01)
#define DIO_DIR_OUT (0x00)

/* Commands and HID Report ID for USB 1408FS  */
#define DCONFIG     (0x01)     // Configure digital port
#define DIN         (0x03)     // Read digital port
#define DOUT        (0x04)     // Write digital port

#define AIN         (0x10)     // Read analog input channel
#define AIN_SCAN    (0x11)     // Scan analog channels
#define AIN_STOP    (0x12)     // Stop input scan
#define ALOAD_QUEUE (0x13)     // Load the channel/gain queue

#define AOUT        (0x14)     // Write analog output channel
#define AOUT_SCAN   (0x15)     // Output values to a range of output channels
#define AOUT_STOP   (0x16)     // Stop output scan

#define CINIT       (0x20)     // Initialize counter
#define CIN         (0x21)     // Read Counter

#define MEM_READ    (0x30)     // Read Memory
#define MEM_WRITE   (0x31)     // Write Memory

#define BLINK_LED   (0x40)     // Causes LED to blink
#define RESET       (0x41)     // Reset USB interface
#define SET_TRIGGER (0x42)     // Configure external trigger
#define SET_SYNC    (0x43)     // Configure sync input/output
#define GET_STATUS  (0x44)     // Get device status
#define SET_CAL     (0x45)     // Set calibaration output
#define GET_ALL     (0x46)     // Get all analog and digital input values

#define PREPARE_DOWNLOAD (0x50) // Prepare for program memory download
#define WRITE_CODE       (0x51) // Write program memory
#define WRITE_SERIAL     (0x53) // Write a new serial number to device
#define READ_CODE        (0x55) // Read program memory

// #define OFFSET_ADJUSTMENT  (0x1F00)   // Offset Adjustment for the A/D        0x1F00 - 0x1F4F
// #define SE_GAIN_ADJUSTMENT (0x1F50)   // Single Ended Gain Adjustment for A/D 0x1F50 - 0x1F5F
// #define DE_GAIN_ADJUSTMENT (0x1F60)   // Differential Gain Adjustment for A/D 0x1F60 - 0x1F67
// #define CAL_PIN_VOLTAGE    (0x1FA0)   // Calibration pin voltage              0x1FA0 - 0x1FA3

#define EXT_TRIG_FAILING_EDGE 0;
#define EXT_TRIG_RAISING_EDGE 1;
#define SYNC_MASTER 0
#define SYNC_SLAVE  1

// Gain Ranges
#define SE_10_00V  (0x9)           // Single Ended 0-10.0 V

#define BP_20_00V  (0x0)           // Differential +/- 20.0 V
#define BP_10_00V  (0x1)           // Differential +/- 10.0 V
#define BP_5_00V   (0x2)           // Differential +/- 5.00 V
#define BP_4_00V   (0x3)           // Differential +/- 4.00 V
#define BP_2_50V   (0x4)           // Differential +/- 2.50 V
#define BP_2_00V   (0x5)           // Differential +/- 2.00 V
#define BP_1_25V   (0x6)           // Differential +/- 1.25 V
#define BP_1_00V   (0x7)           // Differential +/- 1.00 V

// Option values for AInScan
#define AIN_EXECUTION     0x1  // 1 = single execution, 0 = continuous execution
#define AIN_TRANSFER_MODE 0x2  // 1 = Immediate Transfer mode  0 = block transfer mode
#define AIN_TRIGGER       0x4  // 1 = Use External Trigger
#define AIN_DEBUG         0x8  // 1 = debug mode.
#define AIN_GAIN_QUEUE    0x10 // 1 = Use Channel Gain Queue, 0 = Use channnel parameters


typedef struct t_getAllValues {
  int16_t ref_Low[4];   // Differential Inputs with reference set to the low channel
  int16_t ref_High[4];  // Differential Inputs with reference set to the low channel
  int16_t se[8];        // Single Ended inputs
  uint8_t dio_portA;    // Digital Port A
  uint8_t dio_portB;    // Digital Port B
} getAllValues;

/* function prototypes for the USB-1408FS */
void usbDConfigPort_USB1408FS(libusb_device_handle *udev, uint8_t port, uint8_t direction);
void usbDIn_USB1408FS(libusb_device_handle *udev, uint8_t port, uint8_t* din_value);
void usbDOut_USB1408FS(libusb_device_handle *udev, uint8_t port, uint8_t value);

signed short usbAIn_USB1408FS(libusb_device_handle *udev, uint8_t channel, uint8_t range);
void usbAOut_USB1408FS(libusb_device_handle *udev, uint8_t channel, uint16_t value);
int usbAOutScan_USB1408FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel,
			  uint32_t count, float *frequency, uint16_t data[], uint8_t options);
void usbAOutStop_USB1408FS(libusb_device_handle *udev);
void usbAInStop_USB1408FS(libusb_device_handle *udev);
int usbAInScan_USB1408FS(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count,
			 float *frequency, uint8_t options, int16_t data[]);
int usbAInScan_USB1408FS_SE(libusb_device_handle *udev, uint8_t lowchannel, uint8_t highchannel, uint32_t count,
			    float *frequency, uint8_t options, int16_t sdata[]);
void usbALoadQueue_USB1408FS(libusb_device_handle *udev, uint8_t num, uint8_t chan[], uint8_t gains[]);
void usbInitCounter_USB1408FS(libusb_device_handle *udev);
uint32_t usbReadCounter_USB1408FS(libusb_device_handle *udev);

void usbReadMemory_USB1408FS( libusb_device_handle *udev, uint16_t address, uint8_t count, uint8_t memory[]);
int usbWriteMemory_USB1408FS(libusb_device_handle *udev, uint16_t address,  uint8_t count, uint8_t data[]);
void usbBlink_USB1408FS(libusb_device_handle *udev);
int usbReset_USB1408FS(libusb_device_handle *udev);
uint16_t usbGetStatus_USB1408FS(libusb_device_handle *udev);
void usbSetTrigger_USB1408FS(libusb_device_handle *udev, uint8_t type);
void usbSetSync_USB1408FS(libusb_device_handle *udev, uint8_t type);
void usbGetAll_USB1408FS(libusb_device_handle *udev, getAllValues* value);
float volts_1408FS(const int gain, const signed short num);
float volts_1408FS_SE(const signed short num);
void init_USB1408FS(libusb_device_handle *udev);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_1408FS_H
