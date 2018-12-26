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

import hid
import time
from struct import *

class usb_dioLS:   # HID LS dio devices
    
  DIO_PORTA     = 0x01
  DIO_PORTB     = 0x04
  DIO_PORTC_LOW = 0x08
  DIO_PORTC_HI  = 0x02

  DIO_DIR_IN    = 0x01 
  DIO_DIR_OUT   = 0x00

  portC         = 0x0  # current value of port C

  # Commands and Codes for USB 1024-LS HID reports
  # Digital I/O Commands
  DCONFIG     = 0x0D     # Configure digital port
  DIN         = 0x00     # Read digital port
  DOUT        = 0x01     # Write digital port
  DBIT_IN     = 0x02     # Read Digital port bit
  DBIT_OUT    = 0x03     # Write Digital port bit

  # Counter Commands
  CINIT       = 0x05     # Initialize counter
  CIN         = 0x04     # Read Counter

  # Memory Commands
  MEM_READ    = 0x09     # Read Memory
  MEM_WRITE   = 0x0A     # Write Memory

  # Miscellaneous Commands
  BLINK_LED   = 0x0B     # Causes LED to blink
  RESET       = 0x11     # Reset USB interface
  SET_ID      = 0x0C     # Set the user ID
  GET_ID      = 0x0F     # Get the user ID

  productID   = 0

  def __init__(self):
    try:
      self.h = hid.device()
    except:
      print('Error in creating hid device')
      return
    
  #################################
  #     Digital I/O  Commands     #
  #################################

  def DConfig(self, port, direction):
    """
    This command sets the direction of the digital ports A, B and C
    """
    self.h.write([self.DCONFIG, port, direction, 0, 0, 0, 0, 0])

  def DIn(self, port):
    """
    This command reads the digital port
    Bug: need leading 0 in command string
    """
    self.h.write([0, self.DIN, port, 0, 0, 0, 0, 0, 0])
    try:
      value = self.h.read(8,100)
    except:
      print('DIn: error in reading.')

    if port == self.DIO_PORTC_HI :
      value[0] >>= 4    # value of uppper nibble

    if port == self.DIO_PORTC_LOW :
      value[0] &= 0xf    # value of lower nibble

    return value[0]

  def DOut(self, port, value):
    """
    This command writes data to the DIO port.
    """
    if port == self.DIO_PORTC_LOW :
      self.portC &= 0xf0
      self.portC |= (value & 0xf)
      self.h.write([self.DOUT, port, self.portC, 0, 0, 0, 0, 0])
    elif  port == self.DIO_PORTC_HI :
      self.portC &= 0x0f
      self.portC |= (value << 4)
      self.h.write([self.DOUT, port, self.portC, 0, 0, 0, 0, 0])
    else:
      self.h.write([self.DOUT, port, value, 0, 0, 0, 0, 0])

  def DBitIn(self, port, bit):
    """
    Reads digital port bit
    """
    self.h.write([0, self.DBIT_IN, port, bit, 0, 0, 0, 0, 0])
    try:
      value = self.h.read(8,100)
    except:
      print('DBitIn: error in reading.')
    return value[0]

  def DBitOut(self, port, pin, value):
    """
    rites digital port bit
    """
    try:
      self.h.write([self.DBIT_OUT, port, pin, value, 0, 0, 0, 0])
    except:
      print('Error in DBitOut. port =',port,' bit =',bit,' value = ',value)

  #################################
  #     Counter  Commands         #
  #################################

  def CIn(self):
    """
    This function reads the 32-bit event counter on the device.  This
    counter tallies the transitions of an external input attached to
    the CTR pin (pin 20) on the screw terminal of the device.
    """
    self.h.write([self.CIN, 0, 0, 0, 0, 0, 0, 0])
    try:
      value = self.h.read(8,100)
    except:
      print('Error in CIn.')
      return 0
    return (value[0] | (value[1]<<8) | (value[2]<<16) | (value[3]<<24))

  def CInit(self):
    """
    This command initializes the event counter and resets the count to zero
    """
    self.h.write([self.CINIT, 0, 0, 0, 0, 0, 0, 0])


  #################################
  #     Memory  Commands          #
  #################################

  def MemRead(self, address, count):
    """
    This command reads data from the configuration memeory (EEPROM).
    """
    if (count > 8):
      rasie ValueError('MemRead: max count is 8')
      return
    self.h.write([self.MEM_READ, address, count, 0, 0, 0, 0, 0])
    try:
      value = self.h.read(8, 100)
    except:
      print('Error in reading memory, value =', value)
    return(value[0:count])

  def MemWrite(self, address, count, data):
    """
    This command writes data to the non-volatile EEPROM memory on the device.
    The non-volatile memory is used to store calibration coefficients, system
    information and user data.
       count: the number of bytes to read (maximum 4)
       data:  the data to be written (4 bytes max)
    """

    if (count > 4):
      raise ValueError('MemWrite: max count is 4')
      return
    self.h.write([self.MEM_Write, address, count, data[0:count]] + [0]*(5-count))

  #################################
  #     Miscellaneous Commands    #
  #################################

  def Blink(self):
    """
    This commands causes the LED to flash several times.
    """
    self.h.write([self.BLINK_LED, 0, 0, 0, 0, 0, 0, 0])

  def Reset(self):
    """
    The command causes the device to perform a soft reset. The device
    simulates a disconnect from the USB bus which in turn causes the
    host computer to re-enumerate the device.
    """
    self.h.write([self.RESET, 0, 0, 0, 0, 0, 0, 0])

  def SetID(self, id):
    """
    This command stores an identifier on the unit.  Values of 0-255 are valid.  Note that 0
    is used to flag uninitialized units.
    
    id: user id number (0-255)
    """

    if id < 0 or id > 255:
      raise ValueError('SetID: id out of range')
      return
    try:
      self.h.write([self.SET_ID, id, 0, 0, 0, 0, 0, 0])
    except:
      print('Error in writing id')
      
  def GetID(self):
    """
    This function retrieves the id number stored on the device.  Note that a value of 0
    is used to flag an uninitialized device.
    """
    try:
      self.h.write([self.GET_ID, 0, 0, 0, 0, 0, 0, 0])
    except:
      print('Error in reading id')
    try:
      value = self.h.read(8, 100)
    except:
      print('Error in reading id, value =', value)
    return(value[0])

#############################################################################################

class usb_1024LS(usb_dioLS):
  def __init__(self, serial=None):
    self.productID = 0x0076                            # MCC USB-1024LS
    usb_dioLS.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)           
    except:
      print('Can not open USB-1024LS')
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)
    
    self.DConfig(self.DIO_PORTA, self.DIO_DIR_OUT)     # Port A output
    self.DConfig(self.DIO_PORTB, self.DIO_DIR_IN)      # Port B input
    self.DOut(self.DIO_PORTA, 0x0)                     # zero out the port
    self.DConfig(self.DIO_PORTC_LOW, self.DIO_DIR_OUT) # Port C Low output
    self.DConfig(self.DIO_PORTC_HI, self.DIO_DIR_IN)   # Port C Hi  input

class usb_1024HLS(usb_dioLS):
  def __init__(self, serial=None):
    self.productID = 0x007f                            # MCC USB-1024HLS
    usb_dioLS.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)           
    except:
      print('Can not open USB-1024HLS')
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)
    
    self.DConfig(self.DIO_PORTA, self.DIO_DIR_OUT)     # Port A output
    self.DConfig(self.DIO_PORTB, self.DIO_DIR_IN)      # Port B input
    self.DOut(self.DIO_PORTA, 0x0)                     # zero out the port
    self.DConfig(self.DIO_PORTC_LOW, self.DIO_DIR_OUT) # Port C Low output
    self.DConfig(self.DIO_PORTC_HI, self.DIO_DIR_IN)   # Port C Hi  input

class usb_DIO24(usb_dioLS):
  def __init__(self, serial=None):                      # MCC USB-DIO24
    self.productID = 0x0093
    usb_dioLS.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)           
    except:
      print('Can not open USB-DIO24')
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)
    
    self.DConfig(self.DIO_PORTA, self.DIO_DIR_OUT)     # Port A output
    self.DConfig(self.DIO_PORTB, self.DIO_DIR_IN)      # Port B input
    self.DOut(self.DIO_PORTA, 0x0)                     # zero out the port
    self.DConfig(self.DIO_PORTC_LOW, self.DIO_DIR_OUT) # Port C Low output
    self.DConfig(self.DIO_PORTC_HI, self.DIO_DIR_IN)   # Port C Hi  input

class usb_DIO24H(usb_dioLS):
  def __init__(self, serial=None):
    self.productID = 0x0094                            # MCC USB-DIO24H
    usb_dioLS.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)           
    except:
      print('Can not open USB-DIO24H')
      return
      
    # enable non-blocking mode
    self.h.set_nonblocking(1)
    
    self.DConfig(self.DIO_PORTA, self.DIO_DIR_OUT)     # Port A output
    self.DConfig(self.DIO_PORTB, self.DIO_DIR_IN)      # Port B input
    self.DOut(self.DIO_PORTA, 0x0)                     # zero out the port
    self.DConfig(self.DIO_PORTC_LOW, self.DIO_DIR_OUT) # Port C Low output
    self.DConfig(self.DIO_PORTC_HI, self.DIO_DIR_IN)   # Port C Hi  input
