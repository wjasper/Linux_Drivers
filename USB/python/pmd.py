#! /usr/bin/python3
#
# Copyright (c) 2018 Warren J. Jasper <wjasper@ncsu.edu>
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

# Error Handling
class Error(Exception):
  ''' Base class for other exceptions.'''
  pass

class ResultError(Error):
  ''' Raised when return TCP packet fails'''
  pass

class mccHID:
  # These definitions are used to build the request type in usb_control_msg */
  MCC_VID             =   0x09db                 # Vendor ID for Measurement Computing
  CTRL_IN             =   (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
  CTRL_OUT            =    (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
  INTR_LENGTH         =    64

  INPUT_REPORT        = (1 << 8)
  OUTPUT_REPORT       = (2 << 8)

  # Description of the requestType byte */
  # Data transfer direction D7
 HOST_TO_DEVICE       = (0x0 << 7)
 DEVICE_TO_HOST       = (0x1 << 7)

 # Type D5-D6
  STANDARD_TYPE       = (0x0 << 5)
  CLASS_TYPE          = (0x1 << 5)
  VENDOR_TYPE         = (0x2 << 5)
  RESERVED_TYPE       = (0x3 << 5)

  # Recipient D0 - D4
  DEVICE_RECIPIENT    = (0x0)
  INTERFACE_RECIPIENT = (0x1)
  ENDPOINT_RECIPIENT  = (0x2)
  OTHER_RECIPIENT     = (0x3)
  RESERVED_RECIPIENT  = (0x4) 

  def  __init__(self, productID=None):
    hid = hid_open(MCC_VID, USB1208LS_PID, NULL)
