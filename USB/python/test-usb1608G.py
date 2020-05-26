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
    try:
      usb1608G =usb_1608GX()
      print("USB-1608GX device found.")
    except:
      try:
        usb1608G = usb_1608GX_2AO()
        print("USB-1608GX-2AO device found.")
      except:
        print('No USB-1608G device found.')
        return

# print out the calibration tables
  print('Calibration Analog Input Table:')
  for gain in range(usb1608G.NGAIN):
    print('  Range =',gain, \
          'Slope =',format(usb1608G.table_AIn[gain].slope,'.5f'),\
          'Intercept =',format(usb1608G.table_AIn[gain].intercept,'5f'))

  if usb1608G.productID == usb1608G.USB_1608GX_2AO_PID:
    print('Calibration Analog Output Table:')
    for channel in range(usb1608G.NCHAN_AO):
      print('  Channel =',channel, \
          'Slope =',format(usb1608G.table_AIn[channel].slope,'.5f'),\
          'Intercept =',format(usb1608G.table_AIn[channel].intercept,'5f'))

  # print last known calibration date:
  # mdate = usb1608G.CalDate()
  # print('\nMFG Calibration date: ', mdate)

  print("wMaxPacketSize = ", usb1608G.wMaxPacketSize)

  while True:
    print("\nUSB-1608G/1608GX/1608GX_2AO Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' to test continous sampling")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' to test analog input scan.")
    if usb1608G.productID == usb1608G.USB_1608GX_2AO_PID:
      print("Hit 'o'to test Analog Output")
      print("Hit 'O' to test Analog Output Scan")
    print("Hit 'M' for information.")
    print("Hit 'p' to test Pulse Width Modulation")
    print("Hit 't' to test timers")
    print("Hit 'T' to get temperature")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")
    print("Hit 'v' to get version numbers")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb1608G.BlinkLED(count)
    elif ch == 'c':
      counter = int(input('Enter counter [0-1]: '))
      if counter == 0:                    
        usb1608G.CounterInit(usb1608G.COUNTER0)
        print("Connect DIO0 to CTR0")
      else:
        usb1608G.CounterInit(usb1608G.COUNTER1)
        print("Connect DIO0 to CTR1")
      usb1608G.DTristateW(0xf0)
      toContinue()
      for i in range(100):
        usb1608G.DLatchW(0x0)
        usb1608G.DLatchW(0x1)
      print("Count = %d.  Should read 100" % (usb1608G.Counter(counter)))
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
    elif ch == 'T':
      print("Internal temperature = %.2f deg C or %.2f deg " % (usb1608G.Temperature(), usb1608G.Temperature()*9./5. + 32.))
    elif ch == 'v':
      print("FPGA version %s" % (usb1608G.FPGAVersion()))
    elif ch == 't':
      frequency = int(input('Enter frequency of timer: '))
      period = 64.E6/frequency  - 1.
      usb1608G.TimerPeriodW(period)
      usb1608G.TimerPulseWidthW(period / 2)
      usb1608G.TimerCountW(0)
      usb1608G.TimerStartDelayW(0)
      usb1608G.TimerControlW(0x1)
      toContinue()
      usb1608G.TimerControlW(0x0)
#      usb1608G.TimerParamsR()
      print("Timer:", usb1608G.timerParameters.timer, \
            "  Control Reg:",hex(usb1608G.TimerControlR()), \
            "\tPeriod Reg:",hex(usb1608G.TimerPeriodR()), \
            "\tPulse Width Reg:",hex(usb1608G.TimerPulseWidthR()), \
            "    \tCount Reg:",hex(usb1608G.TimerCountR()), \
            "    \tDelay Reg:",hex(usb1608G.TimerStartDelayR()))

if __name__ == "__main__":
  main()
