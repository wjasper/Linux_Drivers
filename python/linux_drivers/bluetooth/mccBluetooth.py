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

from bluetooth import *
import subprocess
import sys

# Base class for lookup tables of calibration coefficients (slope and offset)
class table:
  def __init__(self):
    self.slope = 0.0
    self.intercept = 0.0

# Error Handling
class Error(Exception):
  ''' Base class for other exceptions.'''
  pass

class ResultError(Error):
  ''' Raised when return TCP packet fails'''
  pass
    

# Global constants
MSG_SUCCESS =         0  # Command succeeded
MSG_ERROR_PROTOCOL =  1  # Command failed due to improper protocol (number of expected data bytes did not match protocol definition)
MSG_ERROR_PARAMETER = 2  # Command failed due to invalid parameters (the data contents were incorrect)
MSG_ERROR_BUSY =      3  # Command failed because resource was busy
MSG_ERROR_READY =     4  # Command failed due to FIFO overrun

MSG_INDEX_START =      0
MSG_INDEX_COMMAND =    1
MSG_INDEX_FRAME =      2
MSG_INDEX_STATUS =     3
MSG_INDEX_COUNT =      4  # The maximum value for count is 0x256
MSG_INDEX_DATA =       5

MSG_HEADER_SIZE =     5
MSG_CHECKSUM_SIZE =   1

MSG_REPLY =           0x80
MSG_START =           0xDB

# Definitions for basic MCC Bluetooth Device class    
class mccBluetoothDevice:

  frameID = 0      # current frame id
  sock = 0         # Bluetooth socket
  address = None   # address

  def __init__(self, device_address=None):
    self.address = device_address
    return

  def receiveMessage(self, length):
    data = self.sock.recv(length)
    return data

  def sendMessage(self, message, flush=True):
    if (flush):
      self.flushInput()
    self.sock.send(bytes(message))

  def calcChecksum(self, buf, length):
    checksum = 0
    for i in range(length):
      checksum +=  buf[i]
    return (checksum & 0xff)

  def flushInput(self):
    # Flush input buffers.
    numTotal = 0
    while True:
      try:
        cbuf = self.sock.recv(512, self.sock.MSG_DONTWAIT)
        numTotal += len(cbuf)
      except:
        break
    if numTotal > 0:
      print('flushInput flushed', numTotal, 'bytes')
    return numTotal

  def openDevice(self):
    port = 1
    self.sock = BluetoothSocket(RFCOMM)

    # connect to server
    self.sock.connect((self.address,port))


####################################################################
      
def discoverDevice(target_name):
  # return the address of target_name
  p = subprocess.Popen('/usr/bin/bt-device --list', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
  for line in p.stdout.readlines():
    line = line.decode()
    if target_name in line:
      offset = line.find('(')
      address = str(line[offset+1:-2])
      retval = p.wait()
      return address
  # No device found
  retval = p.wait()
  return


