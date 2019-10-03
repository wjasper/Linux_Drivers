/*
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@ncsu.edu>
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

#ifndef USB_5200_H
#define USB_5200_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB5201_OLD_PID (0x0098)  // legacy number

#define USB5201_PID (0x00af)
#define USB5203_PID (0x00b0)

#define DIO_DIR_IN  (0x01)
#define DIO_DIR_OUT (0x00)

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

/* For connection types Semiconductor */
#define SINGLE_ENDED        (0x00)
#define DIFFERENTIAL        (0x01)  

/* Current excitation values */
#define EXCITATION_OFF       (0x0)
#define MU_A_10              (0x1)  //  10 micro Amps
#define MU_A_210             (0x2)  // 210 micro Amps

/* Data Structures */
// Time should always be represented in GMT

typedef struct deviceTime_t {
  uint8_t seconds;   // seconds in BCD      range 0-59 (eg 0x25 is 25 seconds)
  uint8_t minutes;   // minutes in BCD,     range 0-59
  uint8_t hours;     // hours in BCD,       range 0-23 (eg 0x22 is 2200 or 10 pm)
  uint8_t day;       // day of month in BCD range 1-31
  uint8_t month;     // month in BCD        range 1-12
  uint8_t year;      // last 2 digits of year since 2000 in BCD range 0-99  (represents 2000-2099).
  int8_t time_zone;  // time zone correction to GMT for local time.
} deviceTime;

// 32-byte structure for FAT16 directory entry structure
typedef struct dir_entry_t {
  char DIR_Name[11];        // Short name.
  uint8_t DIR_Attr;         // File attributes
  uint8_t DIR_NTRes;        // Reserved for use by Windows NT.  Set value to 0 when
                            // file is created and never modify it afterwards.
  uint8_t DIR_CrtTimeTenth; // Millisecond stamp at file creation time.  This field actually
                            // contains a count of tenths of a second.  The granularity of the
                            // second part of DIR_CtrTime is 2 seconds so this field is a
                            // count of tenths of a second and its valid range is 0-199 inclusive.
  uint16_t DIR_CtrTime;     // Time file was created.
  uint16_t DIR_CrtDate;     // Date file was created.
  uint16_t DIR_FstClusHI;   // High word of this entry's first cluster number (always 0)
  uint16_t DIR_WrtTime;     // Time of last write. Note that file creation is considered a write.
  uint16_t DIRDrtDate;      // Date of lst write. Note that file creation is considered a write.
  uint16_t DIRFstClusLO;    // Low word of this entry's first cluster number.
  uint32_t DIR_FileSize;    // 32-bit DWORD holding this file's size in bytes.
} dir_entry;

typedef struct disk_info_t {
  uint8_t PMD_format;       // Volume formatted for USB-TEMP/TC use
  uint8_t Volume_size[4];   // Size of volume in bytes
  uint8_t Free_size[4];     // Amount of free space in bytes;
} disk_info;

typedef struct file_header_t {
  uint8_t identifier;     // MCC file identifier, 0xDB
  uint8_t version;        // Data file version
  uint8_t options;        // Logging options
  uint8_t channels;       // Channels logged bit mask
  uint8_t units;          // Data units bit mask
  uint8_t seconds[4];     // Number of seconds between entries
  uint8_t start_time[6];  // Time logging started of type deviceTime
} file_header;

/* Status Bits */
#define PERFORMING_CALIBRATION 0x1  // Performing Channel Calibration
#define DAUGHTERBOARD_PRESENT  0x2  // Data logging daughter board present
#define MEMORYCARD_PRESENT     0x4  // Memory card present (can only become set if daughter board is present).
#define READFILE_IN_PROGRESS   0x8  // ReadFile in progress (can be cancelled with ReadFileAbort).

/* Logging Configuration Options */
#define DISABLE      0x0  // disable logging
#define POWER_UP     0x1  // start logging on powerup
#define START_BUTTON 0x2  // start logging on button press
#define START_TIME   0x3  // start logging at specified time
#define LOG_CJC      0x4  // log CJC temperatures
#define LOG_TIME     0x8  // log timestamp on each entry

/* function prototypes for the USB-5200 */
void usbDConfigPort_USB5200(hid_device *hid, uint8_t direction);
void usbDConfigBit_USB5200(hid_device *hid, uint8_t bit_num, uint8_t direction);
void usbDIn_USB5200(hid_device *hid, uint8_t* value);
void usbDInBit_USB5200(hid_device *hid, uint8_t bit_num, uint8_t* value);
void usbDOut_USB5200(hid_device *hid, uint8_t value);
void usbDOutBit_USB5200(hid_device *hid, uint8_t bit_num, uint8_t value);
void usbTin_USB5200(hid_device *hid, uint8_t channel, uint8_t units, float *value);
void usbTinScan_USB5200(hid_device *hid, uint8_t start_chan, uint8_t end_chan, uint8_t units, float value[]);

void usbReadMemory_USB5200(hid_device *hid, uint16_t address, uint8_t type, uint8_t count, uint8_t memory[]);
int usbWriteMemory_USB5200(hid_device *hid, uint16_t address, uint8_t type, uint8_t count, uint8_t data[]);
void usbBlink_USB5200(hid_device *hid);
int usbReset_USB5200(hid_device *hid);
uint8_t usbGetStatus_USB5200(hid_device *hid);
void usbSetItem_USB5201(hid_device *hid, uint8_t item, uint8_t subitem, uint32_t value);
void usbSetItem_USB5203(hid_device *hid, uint8_t item, uint8_t subitem, float fValue);
int usbGetItem_USB5200(hid_device *hid, uint8_t item, uint8_t subitem, void* value);
void usbCalibrate_USB5200(hid_device *hid);
uint8_t  usbGetBurnoutStatus_USB5200(hid_device *hid, uint8_t mask);
void usbPrepareDownload_USB5200(hid_device *hid, uint8_t micro);
void usbWriteCode_USB5200(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);
int usbReadCode_USB5200(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[]);
void usbWriteSerial_USB5200(hid_device *hid, uint8_t serial[8]);
void usbGetDeviceTime_USB5200(hid_device *hid, deviceTime *date);
void usbSetDeviceTime_USB5200(hid_device *hid, deviceTime *date);
void usbFormatCard_USB5200(hid_device *hid);
void usbGetFirstFile_USB5200(hid_device *hid, dir_entry *dirEntry);
void usbGetNextFile_USB5200(hid_device *hid, dir_entry *dirEntry);
void usbGetDiskInfo_USB5200(hid_device *hid, disk_info *diskInfo);
void usbGetFileInfo_USB5200(hid_device *hid, char *filename, dir_entry *dirEntry);
void usbReadFile_USB5200(hid_device *hid, uint8_t ack_count, char *filename);
void usbDeleteFile_USB5200(hid_device *hid, char *filename);
void usbConfigureLogging_USB5200(hid_device *hid, uint8_t options, uint8_t channels, uint8_t units, uint32_t seconds,
				 uint16_t filenumber, deviceTime starttime);
void usbGetLoggingConfig_USB5200(hid_device *hid, uint8_t *options, uint8_t *channels, uint8_t *units, uint32_t *seconds,
				 uint16_t *filenumber, deviceTime *starttime);
void usbGetFileHeader_USB5200(hid_device *hid, char *filename, file_header *header);
void usbReadFileAck_USB5200(hid_device *hid);
void usbReadFileAbort_USB5200(hid_device *hid);
void usbConfigureAlarm_USB5200(hid_device *hid, uint8_t number, uint8_t in_options, uint8_t out_options, float value_1, float value_2);
void usbGetAlarmConfig_USB5200(hid_device *hid, uint8_t number, uint8_t *in_options, uint8_t *out_options, float *value_1, float *value_2);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif //USB_5200_H
