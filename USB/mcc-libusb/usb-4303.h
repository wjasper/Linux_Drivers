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

#ifndef USB_4303_H
#define USB_4303_H

#ifdef __cplusplus
extern "C" { 
#endif 

#define USB4301_PID  (0x00ae)
#define USB4303_PID  (0x00b9)

/* Commands and Codes for USB 4303 HID reports */
#define DIN              (0x03) // Read digital port
#define DOUT             (0x04) // Write digital port
#define DBIT_IN          (0x05) // Read Digital port bit
#define DBIT_OUT         (0x06) // Write Digital port bit

/* Counter Commands */
#define LOAD             (0x20) // Load the selected counter(s)
#define SAVE             (0x21) // Save the selected counter(s)
#define ARM              (0x22) // Arm the selected counter(s)
#define DISARM           (0x23) // Disarm the selected counter(s)
#define READ             (0x24) // Read counter value
#define SET_TOGGLE       (0x25) // Set or Clear counter toggle output
#define STEP             (0x26) // Step counter up or down

/* 9513 Commands */
#define SET9513CONFIG    (0x27) // Set 9513 configuration
#define GET9513CONFIG    (0x28) // Get 9513 configuration
#define SETOSCOUT        (0x29) // Enable/disable oscillator output
#define SETREGISTER      (0x2a) // Set register value
#define GETREGISTER      (0x2b) // Get register value
#define RESET9513        (0x2c) // Reset 9513 chip to default settings
#define SELECT9513CLOCK  (0x2e) // Select the counter chip's internal clock
#define READFREQ         (0x2f) // Measure frequency

/* Memory Commands */
#define MEM_READ         (0x30) // Read Memory
#define MEM_WRITE        (0x31) // Write Memory

/* Miscellaneous Commands */
#define BLINK_LED        (0x40) // Causes LED to blink
#define RESET            (0x41) // Reset USB interface
#define GET_STATUS       (0x44) // Retrieve device status
#define INTERRUPT_CONFIG (0x45) // Configure external interrupt pin

/* Code Update Commands */
#define PREPARE_DOWNLOAD (0x50) // Prepare for program memory download
#define WRITE_CODE       (0x51) // Write program memory
#define WRITE_SERIAL     (0x53) // Write new serial number to device
#define READ_CODE        (0x55) // Read program memory

/* 9513 Direct Port Access Commands */
#define WRITE_COMMAND    (0x60) // Write 9513 command port
#define READ_COMMAND     (0x61) // Read 9513 command port
#define WRITE_DATA       (0x62) // Write 9513 data port
#define READ_DATA        (0x63) // Read 9513 data port

#define CHIP_1            (1)
#define CHIP_2            (2)
#define COUNTER_1      (0x01)
#define COUNTER_2      (0x02)
#define COUNTER_3      (0x04)
#define COUNTER_4      (0x08)
#define COUNTER_5      (0x10)

#define CNT_1_MODE_REG (0x01)
#define CNT_2_MODE_REG (0x02)
#define CNT_3_MODE_REG (0x03)
#define CNT_4_MODE_REG (0x04)
#define CNT_5_MODE_REG (0x05)
#define ALARM_1_REG    (0x07)
#define CNT_1_LOAD_REG (0x09)
#define CNT_2_LOAD_REG (0x0a)
#define CNT_3_LOAD_REG (0x0b)
#define CNT_4_LOAD_REG (0x0c)
#define CNT_5_LOAD_REG (0x0d)
#define ALARM_2_REG    (0x0f)
#define CNT_1_HOLD_REG (0x11)
#define CNT_2_HOLD_REG (0x12)
#define CNT_3_HOLD_REG (0x13)
#define CNT_4_HOLD_REG (0x14)
#define CNT_5_HOLD_REG (0x15)

/****************************************************************
 *              Counter Mode Register Bit Assignments           *
 ****************************************************************/

/* Ouput control */
#define INACTIVE_LOW   (0x0)  /* Inactive, Output Low */
#define ACTIVE_HIGH    (0x1)  /* Active High Terminal Count Pulse */
#define TC_Toggled     (0x2)  /* TC Toggled */
#define INACTIVE_HIGH  (0x4)  /* Inactive, Output High Impedance */
#define LOW_ON_TC      (0x5)  /* Active Low on Terminal Count Pulse */

/* Count Direction */
#define COUNTDOWN      (0x0)  /* Count Down */
#define COUNTUP        (0x8)  /* Count Up */

/* BCD Mode */
#define BINARY         (0x0)  /* Binary Count */
#define BCD            (0x10) /* BCD Count */

/* Recycle Mode */
#define ONETIME        (0x0)  /* Count Once */
#define RECYCLE        (0x20) /* Count Repetitively */

/* RELOAD */
#define LOADREG        (0x0)   /* Reload from Load */
#define LOADANDHOLDREG (0x40)  /* Reload from Load or Hold except in 
                                  Mode X which reloads only from Load */
/* Special Gate */
#define SPECIALGATEOFF (0x0)   /* Disable Special Gate */
#define SPECIALGATE    (0x80)  /* Enable Special Gate */

/* Count Source Selection */
#define TCN1           (0x000) /* Terminal count of previous counter */
#define SRC1           (0x100) /* Counter Input 1 */
#define SRC2           (0x200) /* Counter Input 2 */
#define SRC3           (0x300) /* Counter Input 3 */
#define SRC4           (0x400) /* Counter Input 4 */
#define SRC5           (0x500) /* Counter Input 5 */
#define GATE1          (0x600) /* Gate Input 1 */
#define GATE2          (0x700) /* Gate Input 2 */
#define GATE3          (0x800) /* Gate Input 3 */
#define GATE4          (0x900) /* Gate Input 4 */
#define GATE5          (0xa00) /* Gate Input 5 */
#define FREQ1          (0xb00) /* Internal frequency from oscillator */
#define FREQ2          (0xc00) /* Internal frequency from oscillator */
#define FREQ3          (0xd00) /* Internal frequency from oscillator */
#define FREQ4          (0xe00) /* Internal frequency from oscillator */
#define FREQ5          (0xf00) /* Internal frequency from oscillator */

/* Source Edge */
#define POSITIVEEDGE   (0x0)    /* Count on Rising Edge */
#define NEGATIVEEDGE   (0x1000) /* Count on Falling Edge */

/* Gating Control */
#define NOGATE         (0x0000) /* No Gating                  */
#define AHLTCPREVCTR   (0x2000) /* Active High TCN-1          */
#define AHLNEXTGATE    (0x4000) /* Active High Level GATE N+1 */
#define AHLPREVGATE    (0x6000) /* Active High Level GATE N-1 */
#define AHLGATE        (0x8000) /* Active High Level GATE N   */
#define ALLGATE        (0xa000) /* Active Low Level GATE N    */
#define AHEGATE        (0xc000) /* Active High Edge GATE N    */
#define ALEGATE        (0xe000) /* Active Low Edge GATE N     */

/* function prototypes for the USB-4303 */
uint8_t usbDIn_USB4303(hid_device *hid);
void usbDOut_USB4303(hid_device *hid, uint8_t value);
uint8_t usbDBitIn_USB4303(hid_device *hid, uint8_t bit);
void usbDBitOut_USB4303(hid_device *hid, uint8_t bit, uint8_t value);

void usbLoad_USB4303(hid_device *hid, uint8_t chip, uint8_t counters);
void usbSave_USB4303(hid_device *hid, uint8_t chip, uint8_t counters);
void usbArm_USB4303(hid_device *hid, uint8_t chip, uint8_t counters);
void usbDisarm_USB4303(hid_device *hid, uint8_t chip, uint8_t counters);
uint16_t usbRead_USB4303(hid_device *hid, uint8_t chip, uint8_t counter);
void usbSetToggle_USB4303(hid_device *hid, uint8_t chip, uint8_t counter, uint8_t set);
void usbStep_USB4303(hid_device *hid, uint8_t chip, uint8_t counter);

void usbSet9513Config_USB4303(hid_device *hid, uint8_t chip, uint16_t settings);
uint16_t usbGet9513Config_USB4303(hid_device *hid, uint8_t chip);
void usbSetOscOut_USB4303(hid_device *hid, uint8_t chip, uint8_t enable);
void usbSetRegister_USB4303(hid_device *hid, uint8_t chip, uint8_t reg, uint16_t value);
uint16_t usbGetRegister_USB4303(hid_device *hid, uint8_t chip, uint8_t reg);
void usbReset9513_USB4303(hid_device *hid, uint8_t chip);
void usbSelect9513Clock_USB4303(hid_device *hid, uint8_t clock);
uint16_t usbReadFreq_USB4303(hid_device *hid, uint8_t chip, uint8_t source, uint16_t interval_counts);

void usbReadMemory_USB4303(hid_device *hid, uint16_t address, uint8_t count, uint8_t* memory);
int usbWriteMemory_USB4303(hid_device *hid, uint16_t address, uint8_t count, uint8_t* data);
int usbBlink_USB4303(hid_device *hid, uint8_t count);
int usbReset_USB4303(hid_device *hid);
uint32_t usbGetStatus_USB4303(hid_device *hid);
void usbInterruptConfig_USB4303(hid_device *hid, uint16_t config, uint16_t data[10]);
void usbPrepareDownload_USB4303(hid_device *hid);
void usbWriteCode_USB4303(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);
void usbWriteSerial_USB4303(hid_device *hid, uint8_t serial[8]);
int usbReadCode_USB4303(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);
void usbWrite9513Command_USB4303(hid_device *hid, uint8_t chip, uint8_t command);
uint8_t usbRead9513Command_USB4303(hid_device *hid, uint8_t chip);
void usbWrite9513Data_USB4303(hid_device *hid, uint8_t chip, uint8_t data);
uint8_t usbRead9513Data_USB4303(hid_device *hid, uint8_t chip);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_4303_H

