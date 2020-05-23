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
from usb_1608G import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb1608G = usb_1608G()
    print("USB-1608G device found.")
  except:
    print('No USB-1608G device found.')
    return

# print out the calibration tables
  for gain in range(usb1608G.NGAIN):
    print('Calibration Table (Differential):  Range = ',gain, \
          'Slope = ',format(usb1608G.table_AIn[gain].slope,'.5f'),\
          'Intercept = ',format(usb1608G.table_AIn[gain].intercept,'5f'))

  # print last known calibration date:
  # mdate = usb1608G.CalDate()
  # print('\nMFG Calibration date: ', mdate)

  print("wMaxPacketSize = ", usb1608G.wMaxPacketSize)

  while True:
    print("\nUSB-1608G Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' for continous sampling")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' to test analog input scan.")
    print("Hit 'M' for information.")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb1608G.BlinkLED(count)
    elif ch == 'M':
      print("Manufacturer: %s" % usb1608G.getManufacturer())
      print("Product: %s" % usb1608G.getProduct())
      print("Serial No: %s" % usb1608G.getSerialNumber())
    elif ch == 'e':
      usb1608G.udev.close()
      exit(0)
    elif ch == 'r':
      usb1608G.Reset()
    elif ch == 'S':
      print('Status =', hex(usb1608G.Status()))
      usb1608G.printStatus()
    elif ch == 's':
      print("Serial No: %s" % usb1608G.getSerialNumber())


if __name__ == "__main__":
  main()
