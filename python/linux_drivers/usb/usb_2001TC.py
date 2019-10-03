#! /usr/bin/python3
#
# Copyright (c) 2019 Warren J. Jasper <wjasper@ncsu.edu>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

import libusb1
import usb1
import time
import sys

from thermocouple import *
from struct import *
from datetime import datetime
from mccUSB import *

class usb_2001TC(mccUSB):
  # UL Control Transfers
  STRING_MESSAGE = 0x80     # Send string messages to the device
  RAW_DATA       = 0x81     # Return RAW data from the device

  AIN            = 0x10     # Read analog input channel
  GET_ALL        = 0x46     # Read Status, CJC and analog input channel
  MEMORY         = 0x30     # Read/Write memory
  MEMADDRESS     = 0x31     # Read/Write memory address
  BLINK_LED      = 0x40     # Cause LED to blink
  RESET          = 0x41     # Force a device reset
  SERIAL         = 0x48     # Read/Write serial number
  UPDATE_MODE    = 0xb0     # Put device into code update mode

  NGAIN          = 8        # max number of gain levels (0-7)
  MAX_MESSAGE_LENGTH  = 64  # max length of MBD Packet in bytes

#typedef struct TC_data_t {
#  uint8_t  status;     // 0x00 - Ready,  0x01 - Busy,  0xff - Error
#  int16_t  CJC;        // CJC Value in Counts
#  uint32_t ADC_Value;  // ADC Value in Counts
# } TC_data;

  # Gain Ranges
  BP_1_17V      =  0  # +/- 1.17V
  BP_585_mV     =  1  # +/- 585mV
  BP_292_5_mV   =  2  # +/- 292.5mV
  BP_146_25mV   =  3  # +/- 146.25mV
  BP_73_125mV   =  4  # +/- 73.125mV
  BP_36_5625    =  5  # +/- 36.5625mV
  BP_18_28125mV =  6  # +/- 18.28125mV
  BP_9_140625mV =  7  # +/- 9.140625mV

  # Status values
  READY    = 0
  ERROR    = 1
  BUSY     = 2
  UNKNOWN  = 3

  status = 0

  def __init__(self, serial=None):
    self.productID = 0x00f9
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-2001TC not found")
      # need to get wMaxPacketSize
    self.wMaxPacketSize = self.getMaxPacketSize()
    self.status = self.READY

  def sendStringRequest(self, message):
    """
    This command reads/writes the string message components of the
    device.  The device stores the various comonents and parameters to
    provide read back.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.STRING_MESSAGE
    wValue = 0
    wIndex = 0
    barray = bytearray(64)
    for i in range(len(message)):
      barray[i] = ord(message[i])
    barray[63] = ord('\0')
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, barray, timeout = 100)

  def getStringReturn(self):
    # Return 64 byte message
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.STRING_MESSAGE
    wValue = 0
    wIndex = 0
    message = self.udev.controlRead(request_type, request, wValue, wIndex, 64, timeout = 100)
    return message.decode()

  def AIn(self):
    """
    This command reads the latest ADC value.  Note, the reading of bad/stale data is possible
    if the device is BUSY, or ERROR, refer to the STATUS of the device for further details or
    use GetAll.  Apply CJC, CJC Gradient, ADC Slope & Offset and account for the ADC resolution
    in to the real temperature calculation.  For RAW voltages, use only the resolution and range 
    used.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN
    wValue = 0
    wIndex = 0
    data, = unpack('=L',self.udev.controlRead(request_type, request, wValue, wIndex, 4, timeout = 500))
    return data

  def GetAll(self):
    """
    This command reads all of the sensor parameters required to make a temperature measurement or
    to ensure a valid RAW reading is performed.  The CJC value does not apply the CJC Gradient or
    convert it to degrees C.  The following conversions are required:
    
    CJC Temperature = (CJC Value / 2^15) * 128 - (CJC Gradient)
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.GET_ALL
    wValue = 0
    wIndex = 0
    message = self.udev.controlRead(request_type, request, wValue, wIndex, 7, timeout = 500)
    (status, CJC, ADC_Value) = unpack('=BhL', message)
    print(status, CJC, ADC_Value)
    return 
    
  #################################
  #     Miscellaneous Commands    #
  #################################

  def Blink(self, count):

    """
    This command causes the LED to blink.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.BLINK_LED
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [count], timeout = 100)

  def Reset(self, type=1):
    """
    This command causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.
    type:  0 = DAQ Defaults - Parameters reset, currently A/D Ranges only
           1 = Device reset with disconnect from USB
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.RESET
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [type], timeout = 100)

  def setSerialNumber(serial):
    """
    This command writes the device serial number.  The serial number consists of 8 bytes, in
    ASCII numeric or hexadecimal digits (ie. '00000001'). The new serial number will be programmed 
    but not used until hardware reset.
    """
    
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.SERIAL
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [serial], timeout = 100)

  ##############################################################################################
  def setVoltageRange(self, volt_range):
    if volt_range == 3:
      self.sendStringRequest("AI{0}:RANGE=BIP146.25E-3V")
    elif volt_range == 4:
      self.sendStringRequest("AI{0}:RANGE=BIP73.125E-3V")
    else :
      self.sendStringRequest("AI{0}:RANGE=BIP146.25E-3V")

  def getVoltageRange(self):
    self.sendStringRequest("?AI{0}:RANGE")
    message = self.getStringReturn()
    if message == "AI{0}:RANGE=BIP146.25E-3V":
      return 3
    elif message == "AI{0}:RANGE=BIP73.125E-3V":
      return 4
    else:
      print("getVoltRange: invalid range", message)
      return -1

  def sendSensorType(self, sensor_type):
    message = "AI{0}:SENSOR=TC/" + sensor_type
    self.sendStringRequest(message)

  def getSensorType(self):
    self.sendStringRequest("?AI{0}:SENSOR")
    return self.getStringReturn()[16:-1]

  def getStatus(self):
    self.sendStringRequest("?AI{0}:STATUS")
    status = self.getStringReturn()[13:-1]
    if status == "READY":
      self.status = self.READY
    elif status == "BUSY":
      self.status = self.BUSY
    elif status == "ERROR":
      self.status = self.ERROR
    else:
      self.status = self.UNKNOWN
    return status
  
  def getFirmwareVersion(self):
    self.sendStringRequest("?DEV:FWV")
    return self.getStringReturn()[8:-1]

  def getMFGCAL(self):
    year = self.getMFGCALYear()
    month = self.getMFGCALMonth()
    day = self.getMFGCALDay()
    hour = self.getMFGCALHour()
    minute = self.getMFGCALMinute()
    second = self.getMFGCALSecond()
    mdate = datetime(year, month, day, hour, minute, second)
    return mdate

  def getMFGCALYear(self):
    self.sendStringRequest("?DEV:MFGCAL{YEAR}")
    year = self.getStringReturn()
    return int(year[17:-1])

  def getMFGCALMonth(self):
    self.sendStringRequest("?DEV:MFGCAL{MONTH}")
    month = self.getStringReturn()
    return int(month[18:-1])

  def getMFGCALDay(self):
    self.sendStringRequest("?DEV:MFGCAL{DAY}")
    day =  self.getStringReturn()
    return int(day[16:-1])

  def getMFGCALHour(self):
    self.sendStringRequest("?DEV:MFGCAL{HOUR}")
    hour = self.getStringReturn()
    return int(hour[17:-1])

  def getMFGCALMinute(self):
    self.sendStringRequest("?DEV:MFGCAL{MINUTE}")
    minute = self.getStringReturn()
    return int(minute[19:-1])

  def getMFGCALSecond(self):
    self.sendStringRequest("?DEV:MFGCAL{SECOND}")
    second = self.getStringReturn()
    return int(second[19:-1])

  def getCJCDegC(self):
    self.sendStringRequest("?AI{0}:CJC/DEGC")
    CJtemp = self.getStringReturn()
    return float(CJtemp[15:-1])

  def getCJCDegF(self):
    self.sendStringRequest("?AI{0}:CJC/DEGF")
    CJtemp = self.getStringReturn()
    return float(CJtemp[15:-1])

  def getCJCDegKelvin(self):
    self.sendStringRequest("?AI{0}:CJC/KELVIN")
    CJtemp = self.getStringReturn()
    return float(CJtemp[17:-1])

  def getSlope(self):
    self.sendStringRequest("?AI{0}:SLOPE")
    slope = self.getStringReturn()
    return float(slope[12:-1])

  def getOffset(self):
    self.sendStringRequest("?AI{0}:OFFSET")
    offset = self.getStringReturn()
    return float(offset[13:-1])

  def getValue(self):
    self.sendStringRequest("?AI{0}:VALUE")
    value = self.getStringReturn()
    return int(value[12:-1])
    
  def tc_temperature(self, tc_type):
    tc = Thermocouple()
    # Set the voltage range (Mode = 4, Range = +/- .078125V)
    # self.setVoltageRange(4)

    # Get Slope and Offset
    slope = self.getSlope()
    offset = self.getOffset()

    # Apply calibration slope and offset 
    value = float(self.getValue())*slope + offset

    # Calculate the TC voltage (in mV) from the corrected values
    tc_voltage = ((value - 524288.)/524288.) * 73.125

    # Read the CJC value in Celsius
    CJC_temp = self.getCJCDegC()

    # Calculate the CJC voltage using the NIST polynomials and add to tc_voltage in millivolts
    tc_mv = tc.temp_to_mv(tc_type, CJC_temp) + tc_voltage

    # Calcualate actual temperature using reverse NIST polynomial.
    return tc.mv_to_temp(tc_type, tc_mv)

  
