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
from bth_1208LS import *
import time
import sys
import fcntl
import os

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    bth1208LS = bth_1208LS()
    print("BTH-1208LS device found.\n")
  except:
    print('No BTH-1208LS device found.')
    return

  # print out the calibration tables:
  for chan in range(bth1208LS.NCHAN_DE):
    for gain in range(bth1208LS.NGAINS):
      print('Calibration Table (Differential): Chan =',chan,' Range = ',gain, \
            'Slope = ',format(bth1208LS.table_AInDE[chan][gain].slope,'.5f'),\
            'Intercept = ',format(bth1208LS.table_AInDE[chan][gain].intercept,'5f'))

  print("")
  for chan in range(bth1208LS.NCHAN_SE):
    print('Calibration Table (Single Ended): Chan = ', chan, 
          'Slope = ',format(bth1208LS.table_AInSE[chan].slope,'.5f'),\
          'Intercept = ',format(bth1208LS.table_AInSE[chan].intercept,'5f'))

  # print last known calibration date:
  mdate = bth1208LS.CalDate()
  print('\nMFG Calibration date: ', mdate)


  while True:
    print("\nBTH-1208LS Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter.")
    print("Hit 'C' for continuous sampling.")
    print("Hit 'd' to read digital IO.")
    print("Hit 'D' to write digital IO.")
    print("Hit 'i' to test Analog Input.")
    print("Hit 'I' to test Analog Input Scan.")
    print("Hit 'o' to test Analog Output.")
    print("Hit 'x' to test Analog Input Scan (Multi-channel).")
    print("Hit 'r' to reset the device.")
    print("Hit 's' to get serial number.")
    print("Hit 'S' to get Status.")
    print("Hit 'e' to exit.")
    
    ch = input('\n')
    
    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      bth1208LS.BlinkLED(count)
    elif ch == 'e':
      bth1208LS.udev.close()
      exit(0)
    elif ch == 'c':
      bth1208LS.ResetCounter()
      print("Connect AO 0 to CTR.")
      toContinue()
      for i in range(100):
        bth1208LS.AOut(0, 4095)
        bth1208LS.AOut(0, 0)
      count = bth1208LS.Counter()
      print("Count = ", count, "    Should read 100.")
    elif ch == 'd':
      value = bth1208LS.DIn()
      print("Digital IO pins: ", hex(value))
    elif ch == 'D':
      value = int(input('Input digital value [0-ff]: '),base=16)
      bth1208LS.DOut(value)
      value = bth1208LS.DOutR()
      print("The value you entered: ", hex(value))
    elif ch == 'i':
      channel = int(input("Input channel DE [0-3]: "))
      gain = int(input("Input range [0-7]: "))
      mode = bth1208LS.DIFFERENTIAL
      for i in range(20):
        value = bth1208LS.AIn(channel, mode, gain)
        print("Range = {0:d},  Channel = {1:d},  Sample[{2:d}] = {3:x}    Volts = {4:f}"\
              .format(gain, channel, i, value, bth1208LS.volts(value, gain)))
    elif ch == 'I':
      print("Testing BTH-1208LS Analog Input Scan.")
      count = int(input("Enter number of scans: "))
      channel = int(input("Input channel DE [0-3]: "))
      gain = int(input("Input range [0-7]: "))
      frequency = float(input("Enter sampling frequency [Hz]: "))
      ranges = [0x0, 0x0, 0x0, 0x0]
      ranges[channel] = gain
      options = bth1208LS.DIFFERENTIAL_MODE
      bth1208LS.AInScanStop()
      bth1208LS.AInScanClearFIFO()
      bth1208LS.AInConfigW(ranges)
      ranges = bth1208LS.AInConfigR()
      print('ranges = ', ranges)
      bth1208LS.AInScanStart(count, 0x0, frequency, (0x1 << channel), options)
      dataAIn = bth1208LS.AInScanRead(count)
      for i in range(count):
        dataAIn[i] = round(dataAIn[i]*bth1208LS.table_AInDE[channel][gain].slope + bth1208LS.table_AInDE[channel][gain].intercept)
        print("Range {0:d}  Channel {1:d}  Sample[{2:d}] = ".format(gain, channel, i), hex(dataAIn[i])," Volts = {0:7.4f}".format(bth1208LS.volts(dataAIn[i], gain)))
    elif ch == 'x':
      print("Testing BTH-1208LS Multi-Channel Analog Input Scan.")
      nChan = int(input("Enter number of channels (1-4): "))
      nScan = int(input("Enter number of scans: "))
      repeats = int(input("Enter number of repeats: "))
      frequency = float(input("Enter sampling frequency [Hz]: "))
      # build bitmap for the first nchan in channels
      channels = 0
      for i in range(nChan):
        channels |= (0x1 << i)
      # Always use BP_20V to make it easy (BP_20V is 0...)
      gain = bth1208LS.BP_20V
      ranges = [0x0, 0x0, 0x0, 0x0]
      for i in range(4):
        ranges[i] = gain
      options = bth1208LS.DIFFERENTIAL_MODE
      bth1208LS.AInScanStop()
      bth1208LS.AInScanClearFIFO()
      bth1208LS.AInConfigW(ranges)
      time.sleep(1)
      for m in range(repeats):
        print("\n\n---------------------------------------");
        print("\nrepeat: ", m);
        bth1208LS.AInScanStop()
        bth1208LS.AInScanClearFIFO()
        bth1208LS.AInScanStart(nScan*nChan, 0x0, frequency, channels, options)
        data = bth1208LS.AInScanRead(nScan)
        for i in range(int(nScan)):
          print("{0:6d}".format(i), end="")
          for j in range(nChan):
            k = i*nChan + j
            value = round(data[k]*bth1208LS.table_AInDE[j][gain].slope + bth1208LS.table_AInDE[j][gain].intercept)
            print(" ,{0:8.4f}".format(bth1208LS.volts(value, ranges[0])), end="")
          print("")
    elif ch == 'C':
      print('BTH-1208LS Continuous Samping')
      print('Hit space <CR> to exit.')
      frequency = float(input('Enter desired frequency [Hz]: '))
      options = bth1208LS.DIFFERENTIAL_MODE  # Single Ended mode
      count = 0                              # continuous execution
      channels = 0x01                        # read all 8 channels
      # Always use BP_20V to make it easy (BP_20V is 0...)
      ranges = [0x0, 0x0, 0x0, 0x0]
      bth1208LS.AInConfigW(ranges)
      bth1208LS.AInScanStop()
      bth1208LS.AInScanClearFIFO()
      bth1208LS.AInScanStart(count, 0x0, frequency, channels, options)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      j = 0
      while True:
        raw_data = bth1208LS.AInScanRead(128)
        print('Scan =', j, 'samples returned =', len(raw_data))
        j += 1
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      bth1208LS.AInScanStop()
    elif ch == 'o':
      print("Test Analog Output")
      channel = int(input("Enter Channel [0-1]: "))
      voltage = float(input("Enter voltage [0-2.5V]: "))
      value = int(voltage * 4095 / 2.5)
      bth1208LS.AOut(channel, value)
      print("Analog Output Voltage = ", bth1208LS.AOutR(channel)*2.5/4095)
    elif ch == 's':
      print('Serial Number: ', bth1208LS.getSerialNumber())
    elif ch == 'S':
      status = bth1208LS.Status()
      pin = bth1208LS.BluetoothPinR()
      radioVersion = bth1208LS.RadioFirmwareVersion()
      print("Status:", hex(status), "    Bluetooth PIN:", pin,"    Radio FirmwareVersion: {0:1x}.{1:2x}"\
            .format((radioVersion >> 8) & 0xff, (radioVersion & 0xff)))

if __name__ == "__main__":
  main()
