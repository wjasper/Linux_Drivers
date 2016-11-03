/*
 *  Copyright (c) 2016 Warren J. Jasper <wjasper@tx.ncsu.edu>
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "pmd.h"
#include "usb-tc-32.h"

#define FS_DELAY 10000

static int nBits(uint32_t num)
{
  int count = 0;
  int i;
  
  // counts the number of bits in a number
  for (i = 0; i < 32; i++) {
    if ((num & 0x1) == 0x1) count++;
    num = (num >> 0x1);
  }
  return count;
}

/***********************************************
 *            Digital I/O                      *
 ***********************************************/

/* reads digital port  */
void usbDIn_TC32(libusb_device_handle *udev, uint8_t value[2])
{
  /* This command reads the current state of the digital input pins.
     A 0 in a bit position indicates the corresponding pin is readig a
     low state, a 1 indicates a high state.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DIN, 0x0, 0x0, (unsigned char *) value, 2, FS_DELAY) < 0) {
    perror("usbDIn_TC32: error in libusb_control_transfer().");
  }
  return;
}

/* writes digital port */
void usbDOut_TC32(libusb_device_handle *udev, uint8_t index, uint32_t values)
{
  /* The command reads or writes the DIO output latch value.  The
     power on default is all 1 (pins are floating) but this could be
     different if alarms are configured.  A 0 in a bit position
     indicates the corresponding pin driver is low, a 1 indicates it
     is floating and can be pulled high.

     index:  the values to write: bit 0: Base unit,  Bit 1 EXP
     value:  the new latch values  
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[5];
  
  data[0] = index;
  memcpy(&data[1], &values, 4);

  if (libusb_control_transfer(udev, requesttype, DOUT, 0x0, 0x0, (unsigned char *) data, sizeof(data), FS_DELAY) < 0) {
    perror("usbDOut_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbDOutR_TC32(libusb_device_handle *udev, uint32_t value[2])
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, DOUT, 0x0, 0x0, (unsigned char *) value, 8, FS_DELAY) < 0) {
    perror("usbDOutR_TC32: error in libusb_control_transfer().");
  }
  return;
}
  
/***********************************************
 *      Temperature Input Commands             *
 ***********************************************/

float usbTin_TC32(libusb_device_handle *udev, uint8_t channel, uint8_t units, uint8_t wait)
{
  /* This command returns the most recent value of a single thermocouple channel.  The
     control pipe will stall if an invalid channel number is specified.  There are some 
     special return values:
     -777.0: Input voltage outside valid common-mode voltage range
     -888.0: Open thermocouple detected
     -999.0: Channel disabled (also returned if EXP channels are specified but no EXP is connected.)

     channel: the channel to read (0-63)
     units:   0 - Celsius
              1 - Voltage
	      2 - ADC code (uncalibraded)
     wait:    0 - return current value
              1 - wait for new reading before returning
  */

  float value;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t wIndex = (wait << 0x8) | units;

  if (channel > 63) return -999.0;

  if (libusb_control_transfer(udev, requesttype, TIN, channel, wIndex, (unsigned char *) &value, sizeof(value), FS_DELAY) < 0) {
    perror("usbDIN_TC32: error in libusb_control_transfer().");
  }
  return value;
}

float usbCJC_TC32(libusb_device_handle *udev, uint8_t channel)
{
  /* This command returns the most recent value of a single CJC sensor
     in Celsius.  The control pipe will stall if an invalid channel
     number is specified.  The value -999 will be returned if an EXP
     sensor is specified but no EXP is connected.
  */

  float value;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (channel > 63) return -999.0;

  if (libusb_control_transfer(udev, requesttype, CJC, channel, 0x0, (unsigned char *) &value, sizeof(value), FS_DELAY) < 0) {
    perror("usbCJC_TC32: error in libusb_control_transfer().");
  }
  return value;
}

void usbTinMultiple_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* Ths command reads the value of multiple thermocouple channels or
     writes the configuration for subsequent reads.  The channels to
     be read are passed as a bitmap when writing the command.  The
     data will be returned in the order low channel number to high
     channel number.  The number of floating point values returned
     will be equal to the number of channels specified (max 64).  The
     special return values listed in the TIn command also apply.  The
     correct length (number of channels * 4 bytes per reading) needs
     to be specified with performing a read.

     The output arguments are used because the values in a setup
     packet are not large engouh to store the channel masks.  The
     output arguments may be skipped and all channel values returned
     if all_channels is set to 1 in the input argurmnets.

     all_channels: 0 - use channel mask for the output arguments
                   1 - ignore output arguments and return values for all 64 channels
     units:        0 - Celsius
                   1 - Voltage
	           2 - ADC code (uncalibraded)
     wait:         0 - return current value
                   1 - wait for new reading before returning
  */

  uint16_t nChan;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t all_channels = cdev->all_channels;
  uint16_t wIndex = ((cdev->wait << 0x8) | cdev->units);

  if (all_channels == 0) {
    nChan = nBits(cdev->channel_mask[0]) + nBits(cdev->channel_mask[1]);
    requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
    if (libusb_control_transfer(udev, requesttype, TIN_MULTIPLE, 0x0, 0x0, (unsigned char *) cdev->channel_mask, 8, FS_DELAY) < 0) {
      perror("usbTInMultiple_TC32: error in libusb_control_transfer().");
    }
  } else {
    nChan = 64;
  }

  requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (libusb_control_transfer(udev, requesttype, TIN_MULTIPLE, all_channels, wIndex, (unsigned char *) cdev->Tin_values, 4*nChan, FS_DELAY) < 0) {
    perror("usbTInMultiple_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbCJCMultiple_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads the value of multiple CJC sensors or writes
     the configuration for subsequent reads.  The sensors to be read
     are passed as a bitmap when writing the command.  The data will
     be returned in the order low channel number to high channel
     number.  The number of floating point values returned will be
     equal to the number of channels specified (max 64).  The correct
     length (number of channels * 4 bytes per reading) needs to be
     specified when performing a read.  The value -999 will be
     returned if an EXP sensor is specified but no EXP is connected.

     The output arguments are used because the values in a setup
     packet are not large enough to store the channel masks.  The
     output arguments may be skipped and all the channel values
     returned if all_channels is set to 1 in the input arguments.

     all_channels: 0 - use channel mask for the output arguments
                   1 - ignore output arguments and return values for all 64 channels
  */

  uint16_t nChan;
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t all_channels = cdev->all_channels;

  if (all_channels == 0) {
    nChan = nBits(cdev->cjc_mask[0]) + nBits(cdev->cjc_mask[1]);
    requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
    if (libusb_control_transfer(udev, requesttype, CJC_MULTIPLE, 0x0, 0x0, (unsigned char *) cdev->channel_mask, 8, FS_DELAY) < 0) {
      perror("usbCJCMultiple_TC32: error in libusb_control_transfer().");
    }
  } else {
    nChan = 64;
  }

  requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  if (libusb_control_transfer(udev, requesttype, CJC_MULTIPLE, all_channels, 0x0, (unsigned char *) cdev->CJC_values, 4*nChan, FS_DELAY) < 0) {
    perror("usbCJCMultiple_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbTinConfig_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads or writes the thermocouple channel
     configuration values.  Each configuration is a uint8_t with the
     following possible values:

     0 - channel disabled
     1 - TC type J
     2 - TC type K
     3 - TC type T
     4 - TC type E
     5 - TC type R
     6 - TC type S
     7 - TC type B
     8 - TC type N
  */
  
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, TIN_CONFIG, 0x0, 0x0, (unsigned char *) cdev->config_values, 64, FS_DELAY) < 0) {
    perror("usbTinConfig_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbTinConfigR_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, TIN_CONFIG, 0x0, 0x0, (unsigned char *) cdev->config_values, 64, FS_DELAY) < 0) {
    perror("usbTinConfig_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbTinStatus_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{

  /*  This command reads the status of the temperature readings. If a
      bit is set the corresponding channel has a new reading that has
      not been read with either the TIn or TInMultiple command.
  */
  
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, TIN_STATUS, 0x0, 0x0, (unsigned char *) cdev->Tin_status, 64, FS_DELAY) < 0) {
    perror("usbTinStatus_TC32: error in libusb_control_transfer().");
  }
  return;
}
  
void usbOTDStatus_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads the status of the open thermocouple detect. If
     a bit is set an open thermocouple is currently detected on the
     corresponding channel. The LED on the front of the device is on
     if any bits are set in this value.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, OTD_STATUS, 0x0, 0x0, (unsigned char *) cdev->OTD_status, 64, FS_DELAY) < 0) {
    perror("usbOTDStatus_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbMeasureConfigR_T32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads or writes the measurement configuration. The
     measurement micro stores the configuration in EEPROM and restores
     it at power on.

     bit 0: 0 - OTD enable,    1 - OTD disabled
     bit 1: 0 - notch @ 60 Hz, 1 - notch @ 50 Hz
     bit 2: 0 - factory coef.  1 - field coef.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, MEASURE_CONFIG, 0x0, 0x0, (unsigned char *) cdev->config_measure, 2, FS_DELAY) < 0) {
    perror("usbMeasureConfigR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbMeasureConfigW_T32(libusb_device_handle *udev, tc32_channel *cdev)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, MEASURE_CONFIG, 0x0, 0x0, (unsigned char *) cdev->config_measure, 2, FS_DELAY) < 0) {
    perror("usbMeasureConfigW_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbMeasureModeR_T32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads or writes the measurement mode. The power on
     value is mode 0 (normal mode). The other modes are used for
     calibration or testing.

     mode_measure: the measurement mode for the unit:
             0 = Normal mode, the measurement loop converts all of the configured channels in sequence
             1 = Test mode: the muxes are fixed on channel 0 and 16 and the ADCs continuously convert those channels.
             2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                 on that input. Value is stored in channel 0 and 16.
             3 = Gain measure mode: gain cal circuit is connected to cal mux and all conversions are performed on 
                 that input. Value is stored in channel 0 and 16.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, MEASURE_MODE, 0x0, 0x0, (unsigned char *) cdev->mode_measure, 2, FS_DELAY) < 0) {
    perror("usbMeasureModeR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbMeasureModeW_T32(libusb_device_handle *udev, tc32_channel *cdev)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, MEASURE_CONFIG, 0x0, 0x0, (unsigned char *) cdev->mode_measure, 2, FS_DELAY) < 0) {
    perror("usbMeasureModeW_TC32: error in libusb_control_transfer().");
  }
  return;
}

/*****************************************
 *         Alarm  Commands               *
 *****************************************/

void usbAlarmConfigR_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads or writes the temperature alarm
     configurations. There are configuration values and two threshold
     values for each of the 32/64 thermocouple channels. The
     measurement microcontrollers store the configurations in EEPROM.

    alarm_config: the alarm configuration
      bit 0: Alarm enable
             0 - alarm disabled, associated bit is controlled by DOut
             1 - alarm enabled,  associated bit is controlled by status
      bit 1: Alarm invert
             0 - normal polarity   (output is low when in alarm condition)
             1 - inverted polarity (output is high when in alarm condition)
      bits 2-3: Alarm type
             0 - High level: alarm when reading >= threshold 1, reset when reading < threshold 2
             1 - Low level: alarm when reading <= threshold 1, reset when reading > threshold 2
             2 - Outside window: alarm when reading < threshold 1 or > threshold 2
      bit 4: Alarm latch
             0 - no latch, alarm output status indicates current state of alarm
             1 - latch, alarm output is active if an alarm condition is detected 
                 and remains active until cleared with AlarmStatus command
      bits 5-6: Error alarms
            00 - Alarm can only be set by valid temperature reading
            01 - An open thermocouple or common-mode voltage error will also set the alarm
            10 - Only an open thermocouple or common-mode voltage will set the alarm,
                 termperature is ignored.
            11 - invalid.
      bit 7: reserveed.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[576];

  if (libusb_control_transfer(udev, requesttype, ALARM_CONFIG, 0x0, 0x0, (unsigned char *) data, 576, FS_DELAY) < 0) {
    perror("usbAlarmConfigR_TC32: error in libusb_control_transfer().");
  }

  memcpy(cdev->alarm_config, &data[0], 32);               // config_base
  memcpy(cdev->alarm_threshold1, &data[32], 128);         // threshold1_base
  memcpy(cdev->alarm_threshold2, &data[160], 128);        // threshold2_base
  memcpy(&(cdev->alarm_config[32]), &data[288], 32);      // config_exp
  memcpy(&(cdev->alarm_threshold1[32]), &data[320], 128); // threshold1_exp
  memcpy(&(cdev->alarm_threshold2[32]), &data[448], 128); // threshold2_exp

  return;
}

void usbAlarmConfigW_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[576];

  memcpy(&data[0],   cdev->alarm_config,             32);  // config_base
  memcpy(&data[32],  cdev->alarm_threshold1,        128);  // threshold1_base
  memcpy(&data[160], cdev->alarm_threshold2,        128);  // threshold2_base
  memcpy(&data[288], &(cdev->alarm_config[32]),      32);  // config_exp
  memcpy(&data[320], &(cdev->alarm_threshold1[32]), 128);  // threshold1_exp
  memcpy(&data[448], &(cdev->alarm_threshold2[32]), 128);  // threshold2_exp

  if (libusb_control_transfer(udev, requesttype, ALARM_CONFIG, 0x0, 0x0, (unsigned char *) data, 576, FS_DELAY) < 0) {
    perror("usbAlarmConfigW_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbAlarmStatusR_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads or clears the status of the temperature
     alarms. If a bit is set an alarm condition exists or is latched
     on the corresponding channel. If the alarm is configured for
     latching then the status will not clear when the alarm condition
     is no longer present. It must be cleared by writing a 1 to the
     corresponding bit. The LED on the front of the device is on if
     any bits are set in this value.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, ALARM_STATUS, 0x0, 0x0, (unsigned char *) cdev->alarm_status, 8, FS_DELAY) < 0) {
    perror("usbAlarmStatusR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbAlarmStatusW_TC32(libusb_device_handle *udev, uint8_t index, uint32_t clear_mask)
{
  // index: 0 - base unit, 1 - EXP
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[5];

  data[0] = index;
  memcpy(&data[1], &clear_mask, 4);
  
  if (libusb_control_transfer(udev, requesttype, ALARM_STATUS, 0x0, 0x0, (unsigned char *) data, sizeof(data), FS_DELAY) < 0) {
    perror("usbAlarmStatusW_TC32: error in libusb_control_transfer().");
  }
  return;
}

/*****************************************
 *         Memory  Commands              *
 *****************************************/

void usbUserMemoryR_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads or writes the nonvolatile user memory. The
     user memory is spread among 3 EEPROM parts

      Address                   Value
    --------------------------------------------
    0x0000 - 0x0EFF     Comms micro memory
    0x1000 - 0x1DFF     Measurement micro memory
    0x2000 - 0x2DFF     EXP micro memory

  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbUserMemoryR: count exceeds 1024.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbUserMemroyR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbUserMemoryW_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 1024) {
    printf("usbUserMemoryW: count exceeds 1024.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, USER_MEMORY, address, 0x0, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbUserMemoryW_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbSettingsMemoryR_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads or writes the nonvolatile network settings
     memory. The settings memory is 32 bytes (address 0 – 0x1F). The
     settings will be implemented after a device reset.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 32) {
    printf("usbSettingsMemoryR: count exceeds 32.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, SETTINGS_MEMORY, address, 0x0, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbSettingsMemroyR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbSettingsMemoryW_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 32) {
    printf("usbSettingsMemoryW: count exceeds 32.\n");
    return;
  }
  if (libusb_control_transfer(udev, requesttype, SETTINGS_MEMORY, address, 0x0, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbSettingsMemoryW_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbConfigMemoryR_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data)
{
  /* This command reads or writes the nonvolatile device configuration
     memory. The configuration memory is 16 bytes (address 0 –
     0x0F). The configuration will be implemented after a device
     reset. The configuration should only be written during factory
     setup.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 16) {
    printf("usbConfigMemoryR: count exceeds 16.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, CONFIG_MEMORY, address, 0x0, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbConfigMemroyR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbConfigMemoryW_TC32(libusb_device_handle *udev, uint16_t address, uint16_t count, uint8_t *data)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (count > 16) {
    printf("usbConfigMemoryW: count exceeds 16.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, CONFIG_MEMORY, address, 0x0, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbConfigMemoryW_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbFactoryCoefficientsR_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads or writes the factory calibration
     coefficients. Each coefficient is a float. The firmware applies
     the coefficients when calculating the voltage and temperature
     values for each channel. The coefficients are applied immediately
     and stored in EEPROM. These should only be written during
     manufacturing setup.
  */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, FACTORY_COEF, 0x0, 0x0, (unsigned char *) &cdev->calCoeffFactory,
			      sizeof(calCoeff), FS_DELAY) < 0) {
    perror("usbFactoryCoefficientsR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbFieldCoefficientsR_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads or writes the field calibration
     coefficients. Each coefficient is a float. The firmware applies
     the coefficients when calculating the voltage and temperature
     values for each channel. The coefficients are applied immediately
     and stored in EEPROM.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, FIELD_COEF, 0x0, 0x0, (unsigned char *) &cdev->calCoeffField,
			      sizeof(calCoeff), FS_DELAY) < 0) {
    perror("usbFieldCoefficientsR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbFieldCoefficientsW_TC32(libusb_device_handle *udev, uint8_t index, calCoeff *coeff)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[33];

  data[0] = index;
  memcpy(&data[1], coeff, 32);

  if (libusb_control_transfer(udev, requesttype, CONFIG_MEMORY, 0x0, 0x0, (unsigned char *) data, sizeof(data), FS_DELAY) < 0) {
    perror("usbFieldCoefficientsW_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbCalDate_TC32(libusb_device_handle *udev, struct tm *date_base, struct tm *date_exp)
{

  /* This command reads or writes the factory calibration date(s). */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint8_t data[12];
  time_t time;

  if (libusb_control_transfer(udev, requesttype, CAL_DATE, 0x0, 0x0, (unsigned char *) data, sizeof(data), FS_DELAY) < 0) {
    perror("usbCalDate_TC32: error in libusb_control_transfer().");
  }

  date_base->tm_year = data[0] + 100;
  date_base->tm_mon = data[1] - 1;
  date_base->tm_mday = data[2];
  date_base->tm_hour = data[3];
  date_base->tm_min = data[4];
  date_base->tm_sec = data[5];
  time = mktime(date_base);
  date_base = localtime(&time);

  date_exp->tm_year = data[6] + 100;
  date_exp->tm_mon = data[7] - 1;
  date_exp->tm_mday = data[8];
  date_exp->tm_hour = data[9];
  date_exp->tm_min = data[10];
  date_exp->tm_sec = data[11];
  time = mktime(date_exp);
  date_exp = localtime(&time);

  return;
}

void usbGainVoltages_TC32(libusb_device_handle *udev, gainVoltages *gain)
{
  /* This command reads or writes the gain voltage reference
     values. These values should only be written during manufacturing
     setup.
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  
  if (libusb_control_transfer(udev, requesttype, GAIN_VOLTAGE, 0x0, 0x0, (unsigned char *) gain, sizeof(gainVoltages), FS_DELAY) < 0) {
    perror("usbGainVoltages_TC32: error in libusb_control_transfer().");
  }
  return;
}

/*****************************************
 *       Miscellaneous  Commands         *
 *****************************************/

void usbBlink_TC32(libusb_device_handle *udev, uint8_t count)
{
  /*  This command will blink the device LED "count" number of times  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, BLINK_LED, 0x0, 0x0, (unsigned char *) &count, sizeof(count), FS_DELAY) < 0) {
    perror("usbBlink_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbReset_TC32(libusb_device_handle *udev)
{
  /* This command resets the device */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, RESET, 0x0, 0x0, (unsigned char *) NULL, 0, FS_DELAY) < 0) {
    perror("usbRESET_TC32: error in libusb_control_transfer().");
  }
  return;
}
  
void usbStatus_TC32(libusb_device_handle *udev, tc32_channel *cdev)
{
  /* This command reads the device status
     status: bit 0 - no EXP detected, 1 - EXP detected
  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, STATUS, 0x0, 0x0, (unsigned char *) &cdev->status, 2, FS_DELAY) < 0) {
    perror("usbStatus_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbVersion_TC32(libusb_device_handle *udev, struct version_t *version)
{
  /* This command reads the firmware versions. */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, VERSION, 0x0, 0x0, (unsigned char *) version, 12, FS_DELAY) < 0) {
    perror("usbVersion_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbADCal_TC32(libusb_device_handle *udev)
{
  /* This command causes the measurement loop to pause and an A/D
     system offset calibration to run. The calibration requires
     approximately 50ms to complete then the measurement loop
     automatically returns to the current mode.
  */
  
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, AD_CAL, 0x0, 0x0, (unsigned char *) NULL, 0, FS_DELAY) < 0) {
    perror("usbADCal_TC32: error in libusb_control_transfer().");
  }

  sleep(1);
  return;
}

void usbNetworkConfig_TC32(libusb_device_handle *udev, struct networkDeviceInfo_t *device)
{
  /* This command reads the current device network configuration. */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, NETWORK_CONFIG, 0x0, 0x0, (unsigned char *) device, sizeof(device), FS_DELAY) < 0) {
    perror("usbNetworkConfig_TC32: error in libusb_control_transfer().");
  }

  return;
}

void usbLastResult(libusb_device_handle *udev, uint8_t *result)
{
  /* This command reads the status value for the most recent
     command. It is useful for determining why a command failed when
     getting a bus stall.

     result: 0: Success
             1: Bad protocol: an unrecognized command code or incorrect number 
                of parameters was sent to the device
             2: Bad parameter: a parameter was sent that had an incorrect value
             3: Busy: a resource needed by the command was busy with another task
             4: Not ready: a resource needed by the command was not yet ready or initialized
             5: Timeout: a resource needed by the command timed out before completion
             6: Other: some other error occurred
  */
  
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);

  if (libusb_control_transfer(udev, requesttype, LAST_RESULT, 0x0, 0x0, (unsigned char *) result, sizeof(result), FS_DELAY) < 0) {
    perror("usbLastResult_TC32: error in libusb_control_transfer().");
  }
  return;
}

/*******************************************
 *       Firmware Upgrade Commands         *
 *******************************************/

void usbFirmwareUpgradeMode(libusb_device_handle *udev)
{
  /*
    This command places the device in firmware upgrade mode. The
    firmware cannot be upgraded unless the communications micro is in
    upgrade mode executing from a small bootloader program in a
    different region of FLASH.  

    When the communications micro is put into upgrade mode it must
    reset and operate in USB-only mode with a HID interface (protocol
    defined in a separate document.)

    The communications micro can upgrade the measurement micros while
    in upgrade mode.  

    The user can cancel upgrade mode by power cycling the board before
    writing any new data to program memory.  

    The bootloaders can be overwritten with the various BLMemory
    commands. FirmwareUpgradeMode is not used prior to calling those
    commands.
  */

  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t key = 0xadad;

  if (libusb_control_transfer(udev, requesttype, FIRMWARE_UPGRADE, 0x0, 0x0, (unsigned char *) &key, sizeof(key), FS_DELAY) < 0) {
    perror("usbFirmwareUpgradeMode_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbBLMemoryCommsR(libusb_device_handle *udev, uint32_t address, uint16_t count, uint8_t *data)
{
  /* This command reads or writes the communications micro bootloader
     stored in nonvolatile FLASH memory. The bootloader is located in
     program FLASH memory in two physical address ranges: 0x1D000000 –
     0x1D007FFF for bootloader code and 0x1FC00000 – 0x1FC01FFF for C
     startup code and interrupts. Reads may be performed at any
     time. Writes outside these ranges are ignored. The bootloader
     memory is write protected and must be unlocked in order to write
     the memory. The unlock procedure is to write the unlock code
     0xAA55 to address 0xFFFFFFFE. Writes to the entire memory range
     are then possible. Write any other value to address 0xFFFFFFFE to
     lock the memory after writing.

     The FLASH memory must be erased prior to programming. A bulk
     erase is performed by writing 0xAA55 to address 0x80000000 after
     unlocking the memory for write. The bulk erase will require

     approximately 150 ms to complete. Once the erase is complete the
     memory may be written; however, the device will not be able to
     boot unless it has a valid bootloader so the device should not be
     reset until the bootloader is completely written and verified
     using this command to read back the memory.

    The writes are performed on 4-byte boundaries internally and it is
    recommended that the output data be sent in the same manner.

  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t wValue = address & 0xffff;
  uint16_t wIndex = address >> 16;

  if (count > 1024) {
    printf("usbBLMemoryCommsR_TC32: count exceeds 1024.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, BLMEMORY_COMMS, wValue, wIndex, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbBLMemoryCommsR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbBLMemoryCommsW(libusb_device_handle *udev, uint32_t address, uint16_t count, uint8_t *data)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t wValue = address & 0xffff;
  uint16_t wIndex = address >> 16;

  if (count > 1024) {
    printf("usbBLMemoryCommsW_TC32: count exceeds 1024.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, BLMEMORY_COMMS, wValue, wIndex, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbBLMemoryCommsW_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbBLMemoryEXPR(libusb_device_handle *udev, uint32_t address, uint16_t count, uint8_t *data)
{
  /* This command reads or writes the EXP measurement micro bootloader
     stored in nonvolatile FLASH memory. The bootloader is located in
     program FLASH memory in two physical address ranges: 0x1D000000 –
     0x1D007FFF for bootloader code and 0x1FC00000 – 0x1FC01FFF for C
     startup code and interrupts. Reads may be performed at any
     time. Writes outside these ranges are ignored. The bootloader
     memory is write protected and must be unlocked in order to write
     the memory. The unlock procedure is to write the unlock code
     0xAA55 to address 0xFFFFFFFE. Writes to the entire memory range
     are then possible. Write any other value to address 0xFFFFFFFE to
     lock the memory after writing.

     The FLASH memory must be erased prior to programming. A bulk
     erase is performed by writing 0xAA55 to address 0x80000000 after
     unlocking the memory for write. The bulk erase will require
     approximately 150 ms to complete. Once the erase is complete the
     memory may be written; however, the device will not be able to
     boot unless it has a valid bootloader so the device should not be
     reset until the bootloader is completely written and verified
     using this command to read back the memory.

     The writes are performed on 4-byte boundaries internally and it
     is recommended that the output data be sent in the same manner.

  */

  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t wValue = address & 0xffff;
  uint16_t wIndex = address >> 16;

  if (count > 1024) {
    printf("usbBLMemoryEXPR_TC32: count exceeds 1024.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, BLMEMORY_EXP, wValue, wIndex, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbBLMemoryEXPR_TC32: error in libusb_control_transfer().");
  }
  return;
}

void usbBLMemoryEXPW(libusb_device_handle *udev, uint32_t address, uint16_t count, uint8_t *data)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  uint16_t wValue = address & 0xffff;
  uint16_t wIndex = address >> 16;

  if (count > 1024) {
    printf("usbBLMemoryEXPW_TC32: count exceeds 1024.\n");
    return;
  }

  if (libusb_control_transfer(udev, requesttype, BLMEMORY_EXP, wValue, wIndex, (unsigned char *) data, count, FS_DELAY) < 0) {
    perror("usbBLMemoryEXPW_TC32: error in libusb_control_transfer().");
  }
  return;
}
