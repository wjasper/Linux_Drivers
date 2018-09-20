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

#ifndef USB_1208LS_H
#define USB_1208LS_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB1208LS_PID (0x007A)

#define DIO_PORTA     (0x01)
#define DIO_PORTB     (0x04)
#define DIO_AUXPORT   (0x10)

#define DIO_DIR_IN  (0x01)
#define DIO_DIR_OUT (0x00)

#define MINILAB_CLOCK (6000000L)  // 6 MHz clock

#define OFFSET_ADJUSTMENT  (0x1F00)   // Offset Adjustment for the A/D        0x1F00 - 0x1F4F
#define SE_GAIN_ADJUSTMENT (0x1F50)   // Single Ended Gain Adjustment for A/D 0x1F50 - 0x1F5F
#define DE_GAIN_ADJUSTMENT (0x1F60)   // Differential Gain Adjustment for A/D 0x1F60 - 0x1F67
#define CAL_PIN_VOLTAGE    (0x1FA0)   // Calibration pin voltage              0x1FA0 - 0x1FA3

#define EXT_TRIG_FAILING_EDGE 0
#define EXT_TRIG_RAISING_EDGE 1

// Gain Ranges
#define SE_10_00V  (0x8)           // Single Ended +/- 10.0 V

#define BP_20_00V  (0x00)           // Differential +/- 20.0 V
#define BP_10_00V  (0x10)           // Differential +/- 10.0 V
#define BP_5_00V   (0x20)           // Differential +/- 5.00 V
#define BP_4_00V   (0x30)           // Differential +/- 4.00 V
#define BP_2_50V   (0x40)           // Differential +/- 2.50 V
#define BP_2_00V   (0x50)           // Differential +/- 2.00 V
#define BP_1_25V   (0x60)           // Differential +/- 1.25 V
#define BP_1_00V   (0x70)           // Differential +/- 1.00 V

// Option values for AInScan
#define AIN_EXECUTION     0x1  // 1 = single execution, 0 = continuous execution
#define AIN_BURST_MODE    0x2  // 1 = Burst Mode
#define AIN_TRANSFER      0x4  // 1 = Block Transfer Mode
#define AIN_TRIGGER       0x8  // 1 = Use External Trigger

/* function prototypes for the USB-1208LS */
void usbDConfigPort_USB1208LS(hid_device *hid, uint8_t port, uint8_t direction);
void usbDIn_USB1208LS(hid_device *hid, uint8_t port, uint8_t* din_value);
void usbDOut_USB1208LS(hid_device *hid, uint8_t port, uint8_t value);
uint8_t usbDBitIn_USB1208LS(hid_device *hid, uint8_t port, uint8_t bit);
void usbDBitOut_USB1208LS(hid_device *hid, uint8_t port, uint8_t bit, uint8_t value);
signed short usbAIn_USB1208LS(hid_device *hid, uint8_t channel, uint8_t range);
void usbAInScan_USB1208LS(hid_device *hid, uint16_t count, int rate, uint8_t low_channel, uint8_t high_channel, uint8_t options, int16_t value[], uint8_t gainLoadQueue[]);
void usbAInLoadQueue_USB1208LS(hid_device *hid, uint8_t chanCount, uint8_t chanLoadQueue[], uint8_t gainLoadQueue[]);
void usbAOut_USB1208LS(hid_device *hid, uint8_t channel, uint16_t value);
void usbAInStop_USB1208LS(hid_device *hid);
void usbInitCounter_USB1208LS(hid_device *hid);
uint32_t usbReadCounter_USB1208LS(hid_device *hid);
void usbReadMemory_USB1208LS(hid_device *hid, uint16_t address, uint8_t *data, uint8_t count);
void usbBlink_USB1208LS(hid_device *hid);
void usbReset_USB1208LS(hid_device *hid);
void usbSetTrigger_USB1208LS(hid_device *hid, uint8_t type, uint8_t chan);
uint8_t usbGetID_USB1208LS(hid_device *hid);
void usbSetID_USB1208LS(hid_device *hid, uint8_t id);
float volts_LS(const int gain, const signed short num);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_1208LS_H
