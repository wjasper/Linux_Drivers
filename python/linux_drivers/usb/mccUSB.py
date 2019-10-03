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
from datetime import datetime

class Error(Exception):
  ''' Base class for other exceptions.'''
  pass

class OverrunError(Error):
  ''' Raised when overrun on AInScan'''
  pass

class UnderrunError(Error):
  ''' Raised when underrun on AOutScan'''
  pass

class SaturationError(Error):
  ''' Raised when DAC is saturated '''
  pass


# Description of the requestType byte
# Data transfer direction D7
HOST_TO_DEVICE = 0x0 << 7
DEVICE_TO_HOST = 0x1 << 7
# Type D5-6D
STANDARD_TYPE = 0x0 << 5
CLASS_TYPE    = 0x1 << 5
VENDOR_TYPE   = 0x2 << 5
RESERVED_TYPE = 0x3 << 5
# Recipient D0 - D4
DEVICE_RECIPIENT    = 0x0
INTERFACE_RECIPIENT = 0x1
ENDPOINT_RECIPIENT  = 0x2
OTHER_RECIPIENT     = 0x3
RESERVED_RECIPIENT  = 0x4 

# Base class for lookup tables of calibration coefficients (slope and intercept)
class table:
  def __init__(self):
    self.slope = 0.0
    self.intercept = 0.0

class mccUSB:
  def __init__(self):
    pass

  def openByVendorIDAndProductID(self, vendor_id, product_id, serial):
    self.context = usb1.USBContext()
    for device in self.context.getDeviceIterator(skip_on_error=False):
      if device.getVendorID() == vendor_id and device.getProductID() == product_id:
        if serial == None:
          return device.open()
        else:
          if device.getSerialNumber() == serial:
            return device.open()
    return None      

  def getSerialNumber(self):
    with usb1.USBContext() as context:
      for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == 0x9db and device.getProductID() == self.productID:
          return(device.getSerialNumber())

  def getProduct(self):
    with usb1.USBContext() as context:
      for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == 0x9db and device.getProductID() == self.productID:
          return(device.getProduct())

  def getManufacturer(self):
    with usb1.USBContext() as context:
      for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == 0x9db and device.getProductID() == self.productID:
          return(device.getManufacturer())

  def getMaxPacketSize(self, endpoint=0):
    with usb1.USBContext() as context:
      for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == 0x9db and device.getProductID() == self.productID:
          try:
            maxPacketSize = device.getMaxPacketSize(endpoint)
            return maxPacketSize
          except:
            maxPacketSize = device.getMaxPacketSize0()
            return maxPacketSize


