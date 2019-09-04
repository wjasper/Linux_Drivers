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

import time
import sys
import fcntl
import os
from usb_20x import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb20x = usb_201()
    print("USB-201 device found.")
  except:
    try:
      usb20x = usb_202()
      print("USB-202 device found.")
    except:
      try:
        usb20x = usb_204()
        print("USB-204 device found.")
      except:
        try:
          usb20x = usb_205()
          print("USB-205 device found.")
        except:
          print('No USB-200 series device found.')
          return

  # print out the calibration tables:
  for chan in range(usb20x.NCHAN):
    print('Calibration Table (Single Ended): Chan = ', chan, 
          'Slope = ',format(usb20x.table_AIn[chan].slope,'.5f'),\
          'Intercept = ',format(usb20x.table_AIn[chan].intercept,'5f'))

  # print last known calibration date:
  mdate = usb20x.CalDate()
  print('\nMFG Calibration date: ', mdate)

  print("wMaxPacketSize = ", usb20x.wMaxPacketSize)

  while True:
    print("\nUSB-200 Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter")
    print("Hit 'd' to test digital I/O")
    print("Hit 'i' to test Analog Input")
    print("Hit 'I' to test Analog Input Scan")
    print("Hit 'C' to test continuous sampling.")
    print("Hit 'o' to test Analog Output (202/205 only).")
    print("Hit 'r' to reset the device")
    print("Hit 's' to get serial number")
    print("Hit 'S' to get Status")
    print("Hit 'e' to exit")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb20x.BlinkLED(count)
    elif ch == 'c':
      usb20x.ResetCounter()
      usb20x.DTristateW(0xf0)
      print('Connect DIO0 to CTR0')
      usb20x.DLatchW(0x0)
      toContinue()
      for i in range(100):
        usb20x.DLatchW(0x1)
        usb20x.DLatchW(0x0)
      count = usb20x.Counter()
      print("Count = ", count, "    Should read 100.")
    elif ch == 'd':
      print("Testing Digital I/O ...")
      print("connect pins DIO[0-3] <--> DIO[4-7]")
      usb20x.DTristateW(0xf0)
      print("Digital Port Tristate Register = ", hex(usb20x.DTristateR()))
      while True:
        value = int(input('Enter a byte number [0-0xf]: '),16) & 0xf
        usb20x.DLatchW(value)
        value2 = usb20x.DLatchR()
        value3 = usb20x.DPort() >> 4
        print("The number you entered: ", hex(value3), "  Latched value: ", hex(value2))
        if toContinue() != True:
          break
    elif ch == 'i':
      channel = int(input("Input channel DE [0-7]: "))
      for i in range(20):
        value = usb20x.AIn(channel)
        print("Channel = {0:d},  Sample[{1:d}] = 0x{2:x}    Volts = {3:.4f}"\
              .format(channel, i, value, usb20x.volts(value)))
    elif ch == 'I':
      print('Testing USB-20X Analog Input Scan')
      count = int(input('Enter number of scans: '))
      nchan = int(input('Enter number of channels [1-8]: '))
      frequency = float(input('Enter sampling frequency: '))
      if frequency < 100:
        options = usb20x.IMMEDIATE_TRANSFER_MODE
      else:
        options = 0x0
      usb20x.AInScanStop()
      usb20x.AInScanClearFIFO()
      channels = 0
      for i in range(nchan):
        channels |= (0x1 << i)
      usb20x.AInScanStart(count, frequency, channels, options, 0, 0)
      dataAIn = usb20x.AInScanRead(count)
      for scan in range(count):
        for channel in range(nchan):
          ii = scan*nchan + channel
          dataAIn[ii] = round(dataAIn[ii]*usb20x.table_AIn[channel].slope + usb20x.table_AIn[channel].intercept)
          print("Channel {0:d}  Sample[{1:d}] = ".format(channel, ii), hex(dataAIn[ii])," Volts = {0:7.4f}".format(usb20x.volts(dataAIn[ii])))
    elif ch == 'C':
      print('Testing USB-20X Analog Input Scan in Continuous mode')
      nchan = int(input('Enter number of channels [1-8]: '))
      frequency = float(input('Enter sampling frequency [Hz]: '))
      print('Hit any key to exit')
      if frequency < 100:
        options = usb20x.IMMEDIATE_TRANSFER_MODE
      else:
        options = 0x0
      for i in range(nchan):
        channels |= (0x1 << i)
      usb20x.AInScanStop()
      usb20x.AInScanClearFIFO()
      usb20x.AInScanStart(0, frequency, channels, options, 0, 0)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      j = 0
      while True:
        raw_data = usb20x.AInScanRead(128)
        print('Scan =', j, 'samples returned =', len(raw_data))
        j += 1
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb20x.AInScanStop()
    elif ch == 'e':
      usb20x.udev.close()
      exit(0)
    elif ch == 'o':
      # supported on models USB-202 and USB-205
      channel = int(input('Enter output channel [0-1]: '))
      value = int(input('Enter value [0-4095]: '))
      usb20x.AOut(channel, value)
    elif ch == 's':
      print('Serial Number: ', usb20x.getSerialNumber())
    elif ch == 'S':
      status = usb20x.Status()
      print("Status:", hex(status))
        
if __name__ == "__main__":
  main()
