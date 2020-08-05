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
    print("Hit 'B' to test BURSTIO.")
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
    elif ch == 'i':
      channel = int(input('Input channel [0-1]: '))
      gain = int(input('Enter gain.  1 = +/-10V  2 = +/- 5V  3 = +/- 2V  4 = +/- 1V: '))
      if gain == 1:
        gain = usb2020.BP_10V
      elif gain == 2:
        gain = usb2020.BP_5V
      elif gain == 3:
        gain = usb2020.BP_2V
      elif gain == 4:
        gain = usb2020.BP_1V
      else:
        print('Unknown gain choice.')
        break
      usb2020.AInConfigW(0, channel, gain, True)
      while True:
        try:
          value = usb2020.AIn(channel, gain)
        except ValueError as e:
          print(e)
          break
        print("AIn: %#x  volt = %f" % (value, usb2020.volts(gain, value)))
        if toContinue() != True:
          break
    elif ch == 'I':
      print('Testing USB-2020 Analog Input Scan.')
      usb2020.AInScanStop()
      usb2020.AInScanClearFIFO()
      count = int(input('Enter total number of scans: '))
      nRepeat = int(input('Enter number of repeats: '))
      gain = int(input('Enter gain.  1 = +/-10V  2 = +/- 5V  3 = +/- 2V  4 = +/- 1V: '))
      frequency = float(input('Enter sampling frequency [Hz]: '))
      nChan = int(input('Enter number of channels [1-2]: '))
      for channel in range(nChan):
        if gain == 1:
          gain = usb2020.BP_10V
        elif gain == 2:
          gain = usb2020.BP_5V
        elif gain == 3:
          gain = usb2020.BP_2V
        elif gain == 4:
          gain = usb2020.BP_1V
        else:
          print('Unknown gain choice.')
          break
        usb2020.AInConfigW(channel, channel, gain)
      usb2020.AInConfigW(nChan-1, nChan-1, gain, True)
      for repeat in range(nRepeat):
        print('\n\n---------------------------------------')
        print('repeat: %d' % (repeat))
#        mode = usb2020.VOLTAGE
        mode = 0
        options = 0
        usb2020.AInScanStart(count, 0, frequency, options, mode)
        data = usb2020.AInScanRead()
        print('Number of samples read = %d (should be %d)' % (len(data), count*nChan))
        for i in range(count):
          print("%6d" % (i), end ='')
          for j in range(nChan):
            k = i*nChan + j
            if mode & usb2020.VOLTAGE:   # data returned in volts
              print(", %8.4lf V" % data[k], end='')
            else:
              if data[k] >= 0xffd:
                print("DAC is saturated at +FS")
              elif data[k] <= 0x30:
                print("DAC is saturated at -FS")
              else:
                data[k] = int(round(data[k]*usb2020.table_AIn[j][gain].slope + usb2020.table_AIn[j][gain].intercept))
              print(", %8.4lf V" % usb2020.volts(gain, data[k]), end='')
          print("")
      print("\n---------------------------------------\n")
      usb2020.AInScanStop()
      usb2020.AInScanClearFIFO()
    elif ch == 'C':
      print("Testing USB-2020 Analog Input Scan in continuous mode 2 channels")
      print("Hit any key to exit")
      frequency = float(input("Enter desired sampling frequency (greater than 1000): "))
      usb2020.AInScanStop()
      nScans = 0  # for conitnuous mode
      nChan = 2   # 2 channels
      gain = usb2020.BP_10V
      for channel in range(nChan):
        usb2020.AInConfigW(channel, channel, gain)
      usb2020.AInConfigW(nChan-1, nChan-1, gain, lastElement=True)        
      time.sleep(1)
      mode = usb2020.CONTINUOUS_READOUT
      options = 0
      usb2020.AInScanStart(nScans, 0, frequency, options, mode)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      i = 0
      while True:
        raw_data = usb2020.AInScanRead()
        if i%100 == 0:
          print('Scan =', i, 'samples returned =', len(raw_data))
        i += 1
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb2020.AInScanStop()
      usb2020.AInScanClearFIFO()
    elif ch == 'B':
      print('Testing USB-2020 Analog Input Scan BURSTIO mode')
      usb2020.AInScanStop()
      usb2020.AInScanClearFIFO()
      nSamples = int(input('Enter number of samples (greater than or equal to 256, less than 64 MB and a multiple of 256): '))
      channel = int(input('Input channel [0-1]: '))
      frequency = float(input("Enter desired sampling frequency (greater than 1000): "))
      gain = int(input('Enter gain.  1 = +/-10V  2 = +/- 5V  3 = +/- 2V  4 = +/- 1V: '))
      if gain == 1:
        gain = usb2020.BP_10V
      elif gain == 2:
        gain = usb2020.BP_5V
      elif gain == 3:
        gain = usb2020.BP_2V
      elif gain == 4:
        gain = usb2020.BP_1V
      else:
        print('Unknown gain choice.')
        break
      usb2020.AInConfigW(0, channel, gain, lastElement=True)
      options = usb2020.DDR_RAM
#      options = (0x1 << 7)
      print('options = ', options)
      mode = 0x0
      usb2020.AInScanStart(nSamples, 0, frequency, options, mode)
      data = usb2020.AInScanRead()
      print('Number of samples read = %d (should be %d)' % (len(data), nSamples))

      usb2020.AInScanStop()
      usb2020.AInScanClearFIFO()
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
