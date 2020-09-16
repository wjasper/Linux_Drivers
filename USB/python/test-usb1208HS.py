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
from usb_1208HS import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb1208HS = usb_1208HS()
    print("USB-1208HS device found.")
  except:
    try:
      usb1208HS = usb_1208HS_2AO()
      print("USB-1208HS-2AO device found.")
    except:
      try:
        usb1208HS = usb_1208HS_4AO()
        print("USB-1208HS-4AO device found.")
      except:
        print("No USB-1208HS device found.")
        return

# print out the calibration tables
  print('\nCalibration Analog Input Table:')
  for mode in range(usb1208HS.NMODE):
    for gain in range(usb1208HS.NGAIN):
      print('  Range =',gain, \
            'Slope =',format(usb1208HS.table_AIn[mode][gain].slope,'.5f'),\
            'Intercept =',format(usb1208HS.table_AIn[mode][gain].intercept,'5f'))

  if usb1208HS.productID == usb1208HS.USB_1208HS_2AO_PID or usb1208HS.productID == usb1208HS.USB_1208HS_4AO_PID:
    print('\nCalibration Analog Output Table:')
    for channel in range(usb1208HS.NCHAN_AO):
      print('  Channel =',channel, \
          'Slope =',format(usb1208HS.table_AOut[channel].slope,'.5f'),\
          'Intercept =',format(usb1208HS.table_AOut[channel].intercept,'5f'))

  print("\nwMaxPacketSize = ", usb1208HS.wMaxPacketSize)

  while True:
    print("\nUSB-1208HS/1208HS_2AO/1208HS_4AO Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' to test continous sampling")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' to test analog input scan.")
    if usb1208HS.productID == usb1208HS.USB_1208HS_2AO_PID or usb1208HS.productID == usb1208HS.USB_1208HS_4AO_PID:
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
      usb1208HS.BlinkLED(count)
    elif ch == 'c':
      counter = int(input('Enter counter [0-1]: '))
      if counter == 0:                    
        usb1208HS.CounterInit(usb1208HS.COUNTER0)
        print("Connect DIO0 to CTR0")
      else:
        usb1208HS.CounterInit(usb1208HS.COUNTER1)
        print("Connect DIO0 to CTR1")
      usb1208HS.DTristateW(0xf0)
      toContinue()
      for i in range(100):
        usb1208HS.DLatchW(0x0)
        usb1208HS.DLatchW(0x1)
      print("Count = %d.  Should read 100" % (usb1208HS.Counter(counter)))
    elif ch == 'd':
      print("Testing Digital I/O ...")
      print("connect pins DIO[0-3] <--> DIO[4-7]")
      usb1208HS.DTristateW(0xf0)
      print("Digital Port Tristate Register = ", hex(usb1208HS.DTristateR()))
      while True:
        value = int(input('Enter a byte number [0-0xf]: '),16) & 0xf
        usb1208HS.DLatchW(value)
        value2 = usb1208HS.DLatchR()
        value3 = usb1208HS.DPort() >> 4
        print("The number you entered: ", hex(value3), "  Latched value: ", hex(value2))
        if toContinue() != True:
          break
    elif ch == 'i':
      mode = int(input('Enter mode: 0 = single_ended, 1 = pseudo differential, 2 = true differential, 3 pseudo differential: '))
      if mode == 0 or mode == 3:
          gain = int(input('Enter Gain: 0 = +/- 10V, 1 = +/- 5V, 2 = +/- 2.5V, 3 = 0-10V: '))
      else :
          gain = int(input('Enter Gain: 0 = +/- 20V, 1 = +/- 10V, 2 = +/- 5V: '))
      channel = int(input('Enter channel number: '))
      usb1208HS.AInConfigW(channel, mode, gain)
      while True:
        try:
          value = usb1208HS.AIn(channel)
        except ValueError as e:
          print(e)
          break
        print("AIn: mode = %d  gain = %d  channel = %d  value = %#x  volt = %f" % (mode, gain, channel, value, usb1208HS.volts(mode, gain, value)))
        if toContinue() != True:
          break
    elif ch == 'I':
      print('Testing USB-1208HS Multi-Channel Analog Input Scan.')
      usb1208HS.AInScanStop()
      modeAIn = int(input('Enter mode: 0 = Differential, 1 = Single Ended: '))
      if modeAIn == usb1208HS.SINGLE_ENDED:
        nChan = int(input('Enter number of channels 1-8: '))
      else:
        nChan = int(input('Enter number of channels 1-4: '))
      nScans = int(input('Enter number of scans: '))
      nRepeat = int(input('Enter nuber of repeats: '))
      frequency = float(input('Enter sampling frequency [Hz]: '))
      for channel in range(nChan):
        gain = int(input('Enter Gain: 0 = +/- 10V, 1 = +/- 5V, 2 = +/- 2.5V, 3 = 0-10V: '))
        usb1208HS.AInConfigW(channel, modeAIn, gain)
      channels = 0
      for i in range(nChan):
        channels |= (0x1 << i)

      for repeat in range(nRepeat):
        print('\n\n---------------------------------------')
        print('repeat: %d' % (repeat))
#        mode = usb1208HS.VOLTAGE
        mode = 0
        options = 0
        usb1208HS.AInScanStart(nScans, 0, frequency, channels, options, 0x0)
        data = usb1208HS.AInScanRead()
        print('Number of samples read = %d (should be %d)' % (len(data), nChan*nScans))
        for i in range(nScans):
          print("%6d" % (i), end ='')
          for j in range(nChan):
            k = i*nChan + j
            if mode & usb1208HS.VOLTAGE:   # data returned in volts
              print(", %8.4lf V" % data[k], end='')
            else:
              if data[k] >= 0x1fff:
                print("DAC is saturated at +FS")
              elif data[k] <= 0x0:
                print("DAC is saturated at -FS")
              else:
                data[k] = int(round(data[k]*usb1208HS.table_AIn[modeAIn][gain].slope + usb1208HS.table_AIn[modeAIn][gain].intercept))
              print(", %8.4lf V" % usb1208HS.volts(modeAIn, gain, data[k]), end='')
          print("")
      print("\n\n---------------------------------------")
    elif ch == 'C':
      print("Testing USB-1608G Analog Input Scan in continuous mode 8 channels")
      print("Hit any key to exit")
      frequency = float(input("Enter desired sampling frequency (greater than 1000): "))
      usb1208HS.AInScanStop()
      nScans = 0  # for conitnuous mode
      nChan = 8   # 8 channels
      gain = usb1208HS.BP_10V
      modeAIn = usb1208HS.SINGLE_ENDED 
      channels = 0xff
      for channel in range(nChan):
        usb1208HS.AInConfigW(channel, modeAIn, gain)
      time.sleep(1)
      options = 0
      usb1208HS.AInScanStart(nScans, 0, frequency, channels, options, usb1208HS.CONTINUOUS_READOUT | usb1208HS.VOLTAGE)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      i = 0
      while True:
        raw_data = usb1208HS.AInScanRead()
        if i%100 == 0:
          print('Scan =', i, 'samples returned =', len(raw_data))
        i += 1
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1208HS.AInScanStop()
    elif ch == 'M':
      print("Manufacturer: %s" % usb1208HS.getManufacturer())
      print("Product: %s" % usb1208HS.getProduct())
      print("Serial No: %s" % usb1208HS.getSerialNumber())
    elif ch == 'e':
      usb1208HS.udev.close()
      exit(0)
    elif ch == 'r':
      usb1208HS.Reset()
    elif ch == 'S':
      print('Status =', hex(usb1208HS.Status()))
      usb1208HS.printStatus()
    elif ch == 's':
      print("Serial No: %s" % usb1208HS.getSerialNumber())
    elif ch == 'T':
      print("Internal temperature = %.2f deg C or %.2f deg " % (usb1208HS.Temperature(), usb1208HS.Temperature()*9./5. + 32.))
    elif ch == 'v':
      print("FPGA version %s" % (usb1208HS.FPGAVersion()))
    elif ch == 't':
      frequency = float(input('Enter frequency of timer: '))
      period = 1000./frequency  # period in ms
      usb1208HS.TimerPeriodW(period)
      usb1208HS.TimerPulseWidthW(period/2)
      usb1208HS.TimerCountW(0)
      usb1208HS.TimerStartDelayW(period/10)
      usb1208HS.TimerControlW(0x1)
      toContinue()
      usb1208HS.TimerControlW(0x0)
      print(usb1208HS.TimerParamsR())
      print("Timer:", usb1208HS.timerParameters.timer, \
            "  Control Reg:",hex(usb1208HS.TimerControlR()), \
            "\tPeriod:",usb1208HS.TimerPeriodR(),"ms" \
            "\tPulse Width:",usb1208HS.TimerPulseWidthR(),"ms" \
            "\tCount Reg:",hex(usb1208HS.TimerCountR()), \
            "\tDelay:",usb1208HS.TimerStartDelayR(),"ms")
    elif ch == 'o':
      voltage = float(input('Enter voltage: '))
      usb1208HS.AOut(0, voltage)
      voltage = usb1208HS.AOutR(0)
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
        data[i] = voltage * usb1208HS.table_AOut[channel].slope + usb1208HS.table_AOut[channel].intercept
        if data[i] > 0xffff:
          data[i] = 0xffff
        elif data[i] < 0:
          data[i] = 0
        else:
          data[i] = int(data[i])
      usb1208HS.AOutScanStop()
      usb1208HS.AOutScanStart(0,0,frequency,usb1208HS.AO_CHAN0)
      print("Hit 's <CR>' to stop")
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        try:
          ret = usb1208HS.AOutScanWrite(data)
        except:
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb1208HS.AOutScanStop()
          break
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1208HS.AOutScanStop()
      usb1208HS.AOutScanClearFIFO
      
if __name__ == "__main__":
  main()
