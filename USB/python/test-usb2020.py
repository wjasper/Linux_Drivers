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
from usb_2020 import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb2020 = usb_2020()
    print("USB-2020 device found.")
  except:
    print("No USB-2020 device found.")
    return

  # print out the calibration tables
  print('\nCalibration Analog Input Table:')
  for channel in range(usb2020.NCHAN):
    for gain in range(usb2020.NGAIN):
      print('  Channel =', channel, '   Range = ',gain, \
            'Slope =',format(usb2020.table_AIn[channel][gain].slope,'.5f'),\
            'Intercept =',format(usb2020.table_AIn[channel][gain].intercept,'5f'))

  # print last known calibration date:
  mdate = usb2020.CalDate()
  print('\nMFG Calibration date: ', mdate)

  print("wMaxPacketSize = ", usb2020.wMaxPacketSize)

  while True:
    print("\nUSB-2020 Testing")
    print("------------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'C' to test continous sampling")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' to test analog input scan.")
    print("Hit 'M' for information.")
    print("Hit 'T' to get temperature")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")
    print("Hit 'v' to get version numbers")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb2020.BlinkLED(count)
    elif ch == 'e':
      usb2020.udev.close()
      exit(0)
    elif ch == 'd':
      print("Testing Digital I/O ...")
      print("connect pins DIO[0-3] <--> DIO[4-7]")
      usb2020.DTristateW(0xf0)
      print("Digital Port Tristate Register = ", hex(usb2020.DTristateR()))
      while True:
        value = int(input('Enter a byte number [0-0xf]: '),16) & 0xf
        usb2020.DLatchW(value)
        value2 = usb2020.DLatchR()
        value3 = usb2020.DPort() >> 4
        print("The number you entered: ", hex(value3), "  Latched value: ", hex(value2))
        if toContinue() != True:
          break
    elif ch == 'M':
      print("Manufacturer: %s" % usb2020.getManufacturer())
      print("Product: %s" % usb2020.getProduct())
      print("Serial No: %s" % usb2020.getSerialNumber())
    elif ch == 'e':
      usb2020.udev.close()
      exit(0)
    elif ch == 'r':
      usb2020.Reset()
    elif ch == 'S':
      print('Status =', hex(usb2020.Status()))
      usb2020.printStatus()
    elif ch == 's':
      print("Serial No: %s" % usb2020.getSerialNumber())
    elif ch == 'T':
      print("Internal temperature = %.2f deg C or %.2f deg " % (usb2020.Temperature(), usb2020.Temperature()*9./5. + 32.))
    elif ch == 'v':
      print("FPGA version %s" % (usb2020.FPGAVersion()))

if __name__ == "__main__":
  main()
