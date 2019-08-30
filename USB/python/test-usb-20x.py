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

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb20x.BlinkLED(count)
    elif ch == 'e':
      usb20x.udev.close()
      exit(0)
        
if __name__ == "__main__":
  main()
