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
from usb_2600 import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb2600 = usb_2633()
    print("USB-2633 device found.")
  except:
    try:
      usb2600 = usb_2637()
      print("USB-2637 device found.")
    except:
      try:
        usb2600 = usb_2623()
        print("USB-usb-2623 device found.")
      except:
        try:
          usb2600 = usb_2627()
        except:
          print("No USB-2600 device found.")
          return

  # print out the calibration tables
  print('\nCalibration Analog Input Table:')
  for gain in range(usb2600.NGAIN):
    print('  Range =',gain, \
          'Slope =',format(usb2600.table_AIn[gain].slope,'.5f'),\
          'Intercept =',format(usb2600.table_AIn[gain].intercept,'5f'))

  # print last known calibration date:
  mdate = usb2600.CalDate()
  print('\nAnalog Input MFG Calibration date: ', mdate)

  if usb2600.productID == usb2600.USB_2637_PID:
    print('\nCalibration Analog Output Table:')
    for channel in range(usb2600.NCHAN_AO):
      print('  Channel =',channel, \
          'Slope =',format(usb2600.table_AOut[channel].slope,'.5f'),\
          'Intercept =',format(usb2600.table_AOut[channel].intercept,'5f'))
    # print last known calibration date:
    mdate = usb2600.CalDate()
    print('\nAnalog Output MFG Calibration date: ', mdate)

    
  print("\nwMaxPacketSize = ", usb2600.wMaxPacketSize)

  while True:
    print("\nUSB-2633/2637 Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' to test continous sampling")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' to test analog input scan.")
    if usb2600.productID == usb2600.USB_2637_PID:
      print("Hit 'o' to test Analog Output")
      print("Hit 'O' to test Analog Output Scan")
    print("Hit 'M' for information.")
    print("Hit 't' to test timers")
    print("Hit 'T' to get temperature")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")
    print("Hit 'v' to get version numbers")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb2600.BlinkLED(count)
    elif ch == 'c':
      counter = int(input('Enter counter [0-3]: '))
      print('Connect A0 to counter input')
      usb2600.CounterInit(counter)
      usb2600.DTristateW(usb2600.DIO_A,0xf0)
      toContinue()
      for i in range(100):
        usb2600.DLatchW(usb2600.DIO_A, 0x0)
        usb2600.DLatchW(usb2600.DIO_A,0x1)
      print("Count = %d.  Should read 100" % (usb2600.Counter(counter)))
    elif ch == 'd':
      print("Testing Digital I/O ...")
      print("connect pins Port A <--> Port B")
      usb2600.DTristateW(usb2600.DIO_A, 0x00)
      usb2600.DTristateW(usb2600.DIO_B, 0xff)
      
      print("Digital Port Tristate Register A = ", hex(usb2600.DTristateR(usb2600.DIO_A))) # outupt Port A
      print("Digital Port Tristate Register B = ", hex(usb2600.DTristateR(usb2600.DIO_B))) # input Port B
      while True:
        value = int(input('Enter a hex number [0-0xff]: '),16) & 0xff 
        usb2600.DLatchW(usb2600.DIO_A, value)
        value2 = usb2600.DLatchR(usb2600.DIO_A)
        value3 = usb2600.DPort(usb2600.DIO_B)
        print("The number you entered: ", hex(value2), "  Latched value: ", hex(value3))
        if toContinue() != True:
          break
    elif ch == 'i':
      channel = int(input('Enter channel number (0-63): '))
      gain = int(input('Enter Gain: 0 = +/- 10V, 1 = +/- 5V, 2 = +/- 2V, 3 = +/- 1V: '))
      usb2600.AInConfigW(0, channel, gain, lastElement=True)
      while True:
        try:
          value = usb2600.AIn(channel)
        except ValueError as e:
          print(e)
          break
        print("AIn: %#x  volt = %f" % (value, usb2600.volts(gain, value)))
        if toContinue() != True:
          break
    elif ch == 'I':
      print('Testing USB-1608G Multi-Channel Analog Input Scan.')
      usb2600.AInScanStop()
      usb2600.AInScanClearFIFO()
      nChan = int(input('Enter number of channels 1-64: '))
      nScans = int(input('Enter number of scans: '))
      nRepeat = int(input('Enter nuber of repeats: '))
      frequency = float(input('Enter sampling frequency [Hz]: '))
      for channel in range(nChan):
        gain = int(input('Enter Gain: 0 = +/- 10V, 1 = +/- 5V, 2 = +/- 2V, 3 = +/- 1V: '))
        usb2600.AInConfigW(channel, channel, gain)
      usb2600.AInConfigW(nChan-1, nChan-1, gain, lastElement=True)

      for repeat in range(nRepeat):
        print('\n\n---------------------------------------')
        print('repeat: %d' % (repeat))
#        mode = usb2600.VOLTAGE
        mode = 0
        options = 0
        usb2600.AInScanStart(nScans, 0, frequency, options, mode)
        data = usb2600.AInScanRead()
        print('Number of samples read = %d (should be %d)' % (len(data), nChan*nScans))
        for i in range(nScans):
          print("%6d" % (i), end ='')
          for j in range(nChan):
            k = i*nChan + j
            if mode & usb2600.VOLTAGE:   # data returned in volts
              print(", %8.4lf V" % data[k], end='')
            else:
              if data[k] >= 0xfffd:
                print("DAC is saturated at +FS")
              elif data[k] <= 0x60:
                print("DAC is saturated at -FS")
              else:
                data[k] = int(round(data[k]*usb2600.table_AIn[gain].slope + usb2600.table_AIn[gain].intercept))
              print(", %8.4lf V" % usb2600.volts(gain, data[k]), end='')
          print("")
      print("\n\n---------------------------------------")
    elif ch == 'C':
      print("Testing USB-2600 Analog Input Scan in continuous mode 16 channels")
      print("Hit any key to exit")
      frequency = float(input("Enter desired sampling frequency (greater than 1000): "))
      usb2600.AInScanStop()
      nScans = 0  # for conitnuous mode
      nChan = 64  # 64 channels
      gain = usb2600.BP_10V
      for channel in range(nChan):
        usb2600.AInConfigW(channel, channel, gain)
      usb2600.AInConfigW(nChan-1, nChan-1, gain, lastElement=True)
      time.sleep(1)
      mode = usb2600.CONTINUOUS_READOUT
      options = 0
      usb2600.AInScanStart(nScans, 0, frequency, options, mode)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      i = 0
      total_samples_returned = 0
      while True:
        i += 1
        raw_data = usb2600.AInScanRead()
        total_samples_returned += len(raw_data)
        if i%1000 == 0:
          print('Scan =', i, 'total samples returned =', total_samples_returned)
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb2600.AInScanStop()
      usb2600.AInScanClearFIFO()
    elif ch == 't':
      timer = int(input('Enter timer [0-3]: '))
      frequency = float(input('Enter frequency of timer: '))
      period = 1000./frequency     # period in ms 
      usb2600.TimerPeriodW(timer, period)
      usb2600.TimerPulseWidthW(timer, period/2)
      usb2600.TimerCountW(timer, 0)
      usb2600.TimerStartDelayW(timer, period/10)
      usb2600.TimerControlW(timer, 0x1)
      toContinue()
      usb2600.TimerControlW(timer, 0x0)
      print("Timer:", usb2600.timerParameters[timer].timer, \
            "  Control Reg:",hex(usb2600.TimerControlR(timer)), \
            "\tPeriod:", usb2600.TimerPeriodR(timer),"ms" \
            "\tPulse Width:", usb2600.TimerPulseWidthR(timer),"ms"\
            "    \tCount Reg:",usb2600.TimerCountR(timer), \
            "    \tDelay:", usb2600.TimerStartDelayR(timer),"ms")
    elif ch == 'M':
      print("Manufacturer: %s" % usb2600.getManufacturer())
      print("Product: %s" % usb2600.getProduct())
      print("Serial No: %s" % usb2600.getSerialNumber())
    elif ch == 'e':
      usb2600.udev.close()
      exit(0)
    elif ch == 'r':
      usb2600.Reset()
    elif ch == 'S':
      print('Status =', hex(usb2600.Status()))
      usb2600.printStatus()
    elif ch == 's':
      print("Serial No: %s" % usb2600.getSerialNumber())
    elif ch == 'T':
      print("Internal temperature = %.2f deg C or %.2f deg " % (usb2600.Temperature(), usb2600.Temperature()*9./5. + 32.))
    elif ch == 'v':
      print("FPGA version %s" % (usb2600.FPGAVersion()))
    elif ch == 'o':
      voltage = float(input('Enter voltage: '))
      usb2600.AOut(0, voltage)
      voltage = usb2600.AOutR(0)
      print('Analog Output Voltage = %f V' % (voltage))
    elif ch == 'O':
      print('Test of Analog Output Scan.')
      print('Hook scope up to VDAC 0')
      frequency = float(input('Enter desired frequency of sine wave [ 1-40 Hz]: '))
      frequency *= 512
      data = [0]*512
      for i in range(512):
        voltage = 10*math.sin(2.* math.pi * i / 512.)
        voltage = voltage / 10. * 32768. + 32768.
        data[i] = voltage * usb2600.table_AOut[channel].slope + usb2600.table_AOut[channel].intercept
        if data[i] > 0xffff:
          data[i] = 0xffff
        elif data[i] < 0:
          data[i] = 0
        else:
          data[i] = int(data[i])
      usb2600.AOutScanStop()
      usb2600.AOutScanStart(0,0,frequency,usb2600.AO_CHAN0)
      print("Hit 's <CR>' to stop")
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        try:
          ret = usb2600.AOutScanWrite(data)
        except:
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb2600.AOutScanStop()
          break
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb2600.AOutScanStop()
      usb2600.AOutScanClearFIFO


if __name__ == "__main__":
  main()
