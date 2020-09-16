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
  print('\nCalibration Analog Input Table:')
  for gain in range(usb1608G.NGAIN):
    print('  Range =',gain, \
          'Slope =',format(usb1608G.table_AIn[gain].slope,'.5f'),\
          'Intercept =',format(usb1608G.table_AIn[gain].intercept,'5f'))

  if usb1608G.productID == usb1608G.USB_1608GX_2AO_PID:
    print('\nCalibration Analog Output Table:')
    for channel in range(usb1608G.NCHAN_AO):
      print('  Channel =',channel, \
          'Slope =',format(usb1608G.table_AOut[channel].slope,'.5f'),\
          'Intercept =',format(usb1608G.table_AOut[channel].intercept,'5f'))

  # print last known calibration date:
  mdate = usb1608G.CalDate()
  print('\nMFG Calibration date: ', mdate)

  print("wMaxPacketSize = ", usb1608G.wMaxPacketSize)

  while True:
    print("\nUSB-1608G/1608GX/1608GX_2AO Testing")
    print("-------------------------------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' to test continous sampling")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' to test analog input scan.")
    if usb1608G.productID == usb1608G.USB_1608GX_2AO_PID:
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
    elif ch == 'd':
      print("Testing Digital I/O ...")
      print("connect pins DIO[0-3] <--> DIO[4-7]")
      usb1608G.DTristateW(0xf0)
      print("Digital Port Tristate Register = ", hex(usb1608G.DTristateR()))
      while True:
        value = int(input('Enter a byte number [0-0xf]: '),16) & 0xf
        usb1608G.DLatchW(value)
        value2 = usb1608G.DLatchR()
        value3 = usb1608G.DPort() >> 4
        print("The number you entered: ", hex(value3), "  Latched value: ", hex(value2))
        if toContinue() != True:
          break
    elif ch == 'i':
      channel = int(input('Enter channel number: '))
      mode = int(input('Enter mode: 0 = Differential, 1 = Single Ended: '))
      gain = int(input('Enter Gain: 0 = +/- 10V, 1 = +/- 5V, 2 = +/- 2V, 3 = +/- 1V: '))
      usb1608G.AInConfigW(0, channel, gain, mode, lastElement=True)
      while True:
        try:
          value = usb1608G.AIn(channel, gain)
        except ValueError as e:
          print(e)
          break
        print("AIn: %#x  volt = %f" % (value, usb1608G.volts(gain, value)))
        if toContinue() != True:
          break
    elif ch == 'I':
      print('Testing USB-1608G Multi-Channel Analog Input Scan.')
      usb1608G.AInScanStop()
      usb1608G.AInScanClearFIFO()
      mode = int(input('Enter mode: 0 = Differential, 1 = Single Ended: '))
      if mode == usb1608G.SINGLE_ENDED:
        nChan = int(input('Enter number of channels 1-16: '))
      else:
        nChan = int(input('Enter number of channels 1-8: '))
      nScans = int(input('Enter number of scans: '))
      nRepeat = int(input('Enter nuber of repeats: '))
      frequency = float(input('Enter sampling frequency [Hz]: '))
      for channel in range(nChan):
        gain = int(input('Enter Gain: 0 = +/- 10V, 1 = +/- 5V, 2 = +/- 2V, 3 = +/- 1V: '))
        usb1608G.AInConfigW(channel, channel, gain, mode)
      usb1608G.AInConfigW(nChan-1, nChan-1, gain, mode, lastElement=True)

      for repeat in range(nRepeat):
        print('\n\n---------------------------------------')
        print('repeat: %d' % (repeat))
        mode = usb1608G.VOLTAGE
#        mode = 0
        options = 0
        usb1608G.AInScanStart(nScans, 0, frequency, options, mode)
        data = usb1608G.AInScanRead()
        print('Number of samples read = %d (should be %d)' % (len(data), nChan*nScans))
        for i in range(nScans):
          print("%6d" % (i), end ='')
          for j in range(nChan):
            k = i*nChan + j
            if mode & usb1608G.VOLTAGE:   # data returned in volts
              print(", %8.4lf V" % data[k], end='')
            else:
              if data[k] >= 0xfffd:
                print("DAC is saturated at +FS")
              elif data[k] <= 0x60:
                print("DAC is saturated at -FS")
              else:
                data[k] = int(round(data[k]*usb1608G.table_AIn[gain].slope + usb1608G.table_AIn[gain].intercept))
              print(", %8.4lf V" % usb1608G.volts(gain, data[k]), end='')
          print("")
      print("\n\n---------------------------------------")
    elif ch == 'C':
      print("Testing USB-1608G Analog Input Scan in continuous mode 16 channels")
      print("Hit any key to exit")
      frequency = float(input("Enter desired sampling frequency (greater than 1000): "))
      usb1608G.AInScanStop()
      nScans = 0  # for conitnuous mode
      nChan = 16  # 16 channels
      gain = usb1608G.BP_10V
      mode = usb1608G.SINGLE_ENDED
      for channel in range(nChan):
        usb1608G.AInConfigW(channel, channel, gain, mode)
      usb1608G.AInConfigW(nChan-1, nChan-1, gain, mode, lastElement=True)
      time.sleep(1)
      mode = usb1608G.CONTINUOUS_READOUT
      options = 0
      usb1608G.AInScanStart(nScans, 0, frequency, options, mode)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      i = 0
      while True:
        raw_data = usb1608G.AInScanRead()
        if i%100 == 0:
          print('Scan =', i, 'samples returned =', len(raw_data))
        i += 1
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1608G.AInScanStop()
      usb1608G.AInScanClearFIFO()
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
      frequency = float(input('Enter frequency of timer: '))
      period = 1000/frequency  # period in ms
      usb1608G.TimerPeriodW(period)
      usb1608G.TimerPulseWidthW(period/2)
      usb1608G.TimerCountW(1000)
      usb1608G.TimerStartDelayW(period/10)
      usb1608G.TimerControlW(0x1)
      toContinue()
      usb1608G.TimerControlW(0x0)
      print("Timer:", usb1608G.timerParameters.timer, \
            "  Control Reg:",hex(usb1608G.TimerControlR()), \
            "\tPeriod:", usb1608G.TimerPeriodR(),"ms" \
            "\tPulse Width:", usb1608G.TimerPulseWidthR(),"ms" \
            "    \tCount Reg:", hex(usb1608G.TimerCountR()), \
            "    \tDelay:", usb1608G.TimerStartDelayR(),"ms")
    elif ch == 'o':
      voltage = float(input('Enter voltage: '))
      usb1608G.AOut(0, voltage)
      voltage = usb1608G.AOutR(0)
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
        data[i] = voltage * usb1608G.table_AOut[channel].slope + usb1608G.table_AOut[channel].intercept
        if data[i] > 0xffff:
          data[i] = 0xffff
        elif data[i] < 0:
          data[i] = 0
        else:
          data[i] = int(data[i])
      usb1608G.AOutScanStop()
      usb1608G.AOutScanStart(0,0,frequency,usb1608G.AO_CHAN0)
      print("Hit 's <CR>' to stop")
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        try:
          ret = usb1608G.AOutScanWrite(data)
        except:
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb1608G.AOutScanStop()
          break
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1608G.AOutScanStop()
      usb1608G.AOutScanClearFIFO
      
if __name__ == "__main__":
  main()
