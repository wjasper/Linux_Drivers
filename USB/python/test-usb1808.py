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
    print("Hit 'C' to test continous sampling greater than 1000 Hz")
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
        print("Connect DIO0 to CTR0")
      else:
        print("Connect DIO0 to CTR1")
      usb1808.CounterOptionsW(counter, 0x0)
      usb1808.CounterModeW(counter, 0x0)
      usb1808.CounterSet(counter, 0x0)
      usb1808.DTristateW(0xc)
      toContinue()
      for i in range(100):
        usb1808.DLatchW(0x0)
        usb1808.DLatchW(0x1)
      print("Count = %d.  Should read 100" % (usb1808.Counter(counter)))
    elif ch == 'd':
      print('Testing Digital I/O ...')
      usb1808.DTristateW(0x0)
      print("Digital Port Tristate Register = %#x" % usb1808.DTristateR())
      while True:
        value = int(input('Enter a nubble number [0-0xf]: '),16) & 0xf
        usb1808.DLatchW(value)
        value2 = usb1808.DLatchR()
        print("The number you entered = %#x   Latched value = %#x\n\n" %(value, value2))
        for i in range(4):
          print('Bit %d = %d' % (i, (value2>>i) & 0x1))
        if toContinue() == False:
          break
    elif ch == 'i':
      print('Test Analog Input for all 8 channels')
      mode = int(input('Enter 0 for Differential and 1 for Single Ended: '))
      if mode == 0:
        mode = usb1808.DIFFERENTIAL
      else:
        mode = usb1808.SINGLE_ENDED
      gain = int(input('Eneter Gain: 1 = +/-10V  2 = +/- 5V  3 = 0-10V  4 = 0-5V: '))
      if gain == 1:
        gain = usb1808.BP_10V
      elif gain == 2:
        gain = usb1808.BP_5V
      elif gain == 3:
        gain = usb1808.UP_10V
      elif gain == 4:
        gain = usb1808.UP_5V
      else:
        gain = usb1808.BP_10V
      for chan in range(8):
        usb1808.ADCSetup(chan, gain, mode)  # set them all the same for now
      value = usb1808.AIn(voltage = False)  # return raw 18 bit values, not voltages
      for i in range(8):
        gain = usb1808.AInConfig[i] & 0x3
        mode = (usb1808.AInConfig[i] >> 2) & 0x3
        print('Channel %d Mode = %#x  Gain = %d value[%d] = %#x Volts = %lf' %
              (i, mode, gain, i, value[i], usb1808.volts(gain, value[i])))
    elif ch == 'I':
      print('Testing USB-1808 Multi-Channel Analog Input Scan')
      usb1808.AInScanStop()
      usb1808.AInScanClearFIFO()
      usb1808.AInBulkFlush(5)
      mode = int(input('Enter 0 for Differential and 1 for Single Ended: '))
      if mode == 0:
        mode = usb1808.DIFFERENTIAL
      else:
        mode = usb1808.SINGLE_ENDED
      nchan = int (input('Enter number of channels (1-8): '))
      if nchan > 8:
        print('Max number is channels is 8')
        continue
      nScans = int(input('Enter number of scans: '))
      repeats = int(input('Enter number of repeats: '))
      frequency = float(input('Enter sampling frequency [Hz]: '))
      for channel in range(nchan):
        gain = int(input('Eneter Gain: 1 = +/-10V  2 = +/- 5V  3 = 0-10V  4 = 0-5V: '))
        if gain == 1:
          gain = usb1808.BP_10V
        elif gain == 2:
          gain = usb1808.BP_5V
        elif gain == 3:
          gain = usb1808.UP_10V
        elif gain == 4:
          gain = usb1808.UP_5V
        else:
          gain = usb1808.BP_10V
        usb1808.ADCSetup(channel, gain, mode)
        usb1808.AInScanConfigW(channel, channel)
      usb1808.AInScanConfigW(nchan-1, nchan-1, True)  # Set last entry

      for m in range(repeats):
        print("\n---------------------------------------")
        print('repeat: %d' % m)
        usb1808.AInScanStart(nScans, 0, frequency, 0x0)
        data = usb1808.AInScanRead()
        print('Number of samples read = %d (should be %d)' % (len(data), nScans*nchan))
        for i in range(nScans):
          print("%6d" % (i), end ='')
          for j in range(nchan):
            k = i*nchan + j
            if mode & usb1808.VOLTAGE:   # data returned in volts
              print(", %8.4lf V" % data[k], end='')
            else:
              data[k] = int(round(data[k]*usb1808.table_AIn[j][gain].slope + usb1808.table_AIn[j][gain].intercept))
              if data[k] >= 0x3ffff:
                print(" DAC is saturated at +FS")
                data[k] = 0x3ffff
              elif data[k] < 0x0:
                print(" DaC is saturated at -FS")
                data[k] = 0x0
              print(", %8.4lf V" % usb1808.volts(gain, data[k]), end='')
          print("")
      print("\n\n---------------------------------------")
    elif ch == 'C':
      print('Testing USB-1808 Analog Input Scan in continuous mode 8 channels')
      print('Hit any key to exit')
      frequency = float(input('Enter desired sampling frequency (great than 1000 Hz): '))
      usb1808.AInScanStop()
      usb1808.AInScanClearFIFO()
      usb1808.AInBulkFlush(5)
      nScans = 0                    # for continuous mode
      nchan = 8                     # 8 channels
      gain = usb1808.BP_10V         # +/- 10V
      mode = usb1808.SINGLE_ENDED   # single ended
      for channel in range(nchan):
        usb1808.ADCSetup(channel, gain, mode)
        usb1808.AInScanConfigW(channel, channel)
      usb1808.AInScanConfigW(channel, channel, True) # Set last entry
      nread = 128
      i = 0
#      usb1808.AInScanStart(nScans, 0, frequency, 0, usb1808.CONTINUOUS_READOUT | usb1808.VOLTAGE)
      usb1808.AInScanStart(nScans, 0, frequency, 0, usb1808.CONTINUOUS_READOUT)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        data = usb1808.AInScanRead()
        if i%100 == 0:
          print('Scan = %d' % i)
        i += 1
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1808.AOutScanStop()
      usb1808.AOutScanClearFIFO
    elif ch == 'o':
      channel = int(input('Enter analog output channel [0-1]: '))
      voltage = float(input('Enter voltage [+/- 10V]: '))
      usb1808.AOut(channel, voltage)
    elif ch == 'O':
      print('Test of Analog Output Scan.')
      print('Hook scope up to VDAC 0')
      channel = 0
      frequency = float(input('Enter desired frequency of sine wave [1-40 Hz]: '))
      frequency *= 512
      data = [0]*512
      for i in range(512):
        voltage = 10*math.sin(2.* math.pi * i / 512.)
        voltage = voltage / 10. * 32768. + 32768.
        data[i] = voltage * usb1808.table_AOut[channel].slope + usb1808.table_AOut[channel].intercept
        if data[i] > 0xffff:
          data[i] = 0xffff
        elif data[i] < 0:
          data[i] = 0
        else:
          data[i] = int(data[i])
      usb1808.AOutScanStop()
      usb1808.AOutScanConfig(0, 0, 0)
      usb1808.AOutScanStart(0, 0, frequency, 0)
      print("Hit 's <CR>' to stop")
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        try:
          ret = usb1808.AOutScanWrite(data)
        except:
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb1808.AOutScanStop()
          break
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1808.AOutScanStop()
      usb1808.AOutScanClearFIFO
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
    elif ch == 't':
      print('Test timers.')
      timer = int(input('Enter timer [0-1]: '))
      frequency = float(input('Enter desired frequency: '))
      count = 0
      delay = 0
      usb1808.TimerControlW(timer, 0x0)  # stop timer
      if frequency == 0.0:
        break
      duty_cycle = float(input('Enter desired duty cycle [0.0 - 1.0]: '))
      usb1808.TimerControlW(timer, 0x0)  # stop timer
      usb1808.TimerParametersW(timer, frequency, duty_cycle, count, delay)
      usb1808.TimerControlW(timer, usb1808.TIMER_ENABLE) # enable timer
      [frequency, duty_cycle, count, delay] = usb1808.TimerParametersR(timer)
      print("timer = %d    period = %d     pulseWidth = %d     count = %d     delay = %d" %
            (timer, usb1808.timerParameters[timer].period, usb1808.timerParameters[timer].pulseWidth,
             usb1808.timerParameters[timer].count, usb1808.timerParameters[timer].delay))
      print("timer = %d    frequency = %.3f   duty cycle = %.1f%%   delay = %.3f ms" %
            (timer, frequency, duty_cycle*100, delay))
    elif ch == 'T':
      print('Testing counter and timer.')
      print('Connect Timer 0 to Counter 0')
      frequency = float(input('Enter desired frequency: '))
      duty_cycle = 0.5
      count = 0
      delay = 0
      usb1808.TimerControlW(usb1808.TIMER0, 0x0)  # stop timer
      usb1808.TimerParametersW(usb1808.TIMER0, frequency, duty_cycle, count, delay)
      usb1808.TimerControlW(usb1808.TIMER0, usb1808.TIMER_ENABLE) # enable timer
      usb1808.CounterParametersW(usb1808.COUNTER0, usb1808.COUNTER_PERIOD | usb1808.PERIOD_MODE_10X, 0x0)
      time.sleep(1)
      period = usb1808.Counter(usb1808.COUNTER0)
      frequency = 100.E6/(period + 1)*5.0
      usb1808.TimerControlW(usb1808.TIMER0, 0x0)  # stop timer
      print('frequency = %f' % frequency)
    elif ch == 'v':
      print("FPGA version %s" % (usb1808.FPGAVersion()))

if __name__ == "__main__":
  main()
