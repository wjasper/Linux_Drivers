#! /usr/bin/python3
#
# Copyright (c) 2020 Warren J. Jasper <wjasper@ncsu.edu>
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
import math
from usb_1808 import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb1808 = usb_1808()
    print("USB-1808 device found.")
  except:
    try:
      usb1808 = usb_1808X()
      print("USB-1808X device found.")
    except:
      print('No USB-1808 device found.')
      return

  # print out the calibration tables
  print('Calibration Analog Input Table:')
  for channel in range(usb1808.NCHAN):
    for gain in range(usb1808.NGAIN):
      print('  Channel = ', channel, 'Range =',gain, \
            'Slope =',format(usb1808.table_AIn[channel][gain].slope,'.5f'),\
            'Intercept =',format(usb1808.table_AIn[channel][gain].intercept,'5f'))

  print('\nCalibration Analog Output Table:')
  for channel in range(usb1808.NCHAN_AO):
    print('  Channel =',channel, \
          'Slope =',format(usb1808.table_AOut[channel].slope,'.5f'),\
          'Intercept =',format(usb1808.table_AOut[channel].intercept,'5f'))

  # print last known calibration date:
  mdate = usb1808.CalDate()
  print('\nMFG Calibration date: ', mdate)

  print("wMaxPacketSize = ", usb1808.wMaxPacketSize)

  while True:
    print("\nUSB-1808/1808X Testing")
    print("------------------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' to test continous sampling greater tahn 1000 Hz")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' to test analog input scan.")
    print("Hit 'o' to test Analog Output")
    print("Hit 'O' to test Analog Output Scan")
    print("Hit 'M' for information.")
    print("Hit 't' to test timers")
    print("Hit 'T' to test timer and counter frequency")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")
    print("Hit 'v' to get version numbers")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb1808.BlinkLED(count)
    elif ch == 'c':
      counter = int(input('Enter counter [0-1]: '))
      if counter == 0:                    
        usb1808.CounterInit(usb1808.COUNTER0)
        print("Connect DIO0 to CTR0")
      else:
        usb1808.CounterInit(usb1808.COUNTER1)
        print("Connect DIO0 to CTR1")
      usb1808.DTristateW(0xc)
      toContinue()
      for i in range(100):
        usb1808.DLatchW(0x0)
        usb1808.DLatchW(0x1)
      print("Count = %d.  Should read 100" % (usb1808.Counter(counter)))
    elif ch == 'e':
      usb1808.udev.close()
      exit(0)
    elif ch == 'M':
      print("Manufacturer: %s" % usb1808.getManufacturer())
      print("Product: %s" % usb1808.getProduct())
      print("Serial No: %s" % usb1808.getSerialNumber())
    elif ch == 'r':
      usb1808.Reset()
    elif ch == 'S':
      print('Status =', hex(usb1808.Status()))
      usb1808.printStatus()
    elif ch == 's':
      print("Serial No: %s" % usb1808.getSerialNumber())
    elif ch == 'v':
      print("FPGA version %s" % (usb1808.FPGAVersion()))

if __name__ == "__main__":
  main()