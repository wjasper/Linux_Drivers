#! /usr/bin/python3
#
# Copyright (c) 2018 Warren J. Jasper <wjasper@ncsu.edu>
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

from usb_2400 import *
import time
import sys
import fcntl
import os
import math

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False
  
def main():
  # initalize the class
  try:
    usb2416 = usb_2416()
  except:
    try:
      usb2416 = usb_2416_4AO()
    except:
      print('No device found')
      return

  print('wMaxPacketSize =', usb2416.wMaxPacketSize)
  for gain in range(0,10):
    print('Calibration Table: Range =', gain, \
          'Slope = ',format(usb2416.Cal[gain].slope,'.5f'),\
          'Intercept = ',format(usb2416.Cal[gain].intercept,'5f'))

  print('\n')
  for chan in range(32):
    print('CJC Gradient: Chan =', chan, \
          ' CJC Gradient =',format(usb2416.CJCGradient[chan], '.5f'))

  if getattr(usb2416,'AOut',False) == False:
    pass
  else:
    # Only for USB-2416-4A0
    print('\n')
    for chan in range(0,4):
      print('Calibration Table: Channel =', chan, \
            'Slope = ',format(usb2416.Cal_AO[chan].slope,'.5f'),\
          'Intercept = ',format(usb2416.Cal_AO[chan].intercept,'5f'))
  
  while True:
    print("\nUSB-2416 and USB-2416-2AO Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter")
    print("Hit 'C' to test continuous sampling at 1000 Hz.")
    print("Hit 'd' to read the digital I/O.")
    print("Hit 'D' to write the digital I/O.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test Analog Input")
    print("Hit 'I' to test Analog Input Scan")
    print("Hit 'j' read CJC sensors.")
    if usb2416.productID == 0x00d1:  # Only for USB-2416-4A0
      print("Hit 'o' for Analog Output")
      print("Hit 'O' for Analog Output Scan")
    print("Hit 'r' to reset the device.")
    print("Hit 's' to get serial number.")
    print("Hit 'S' to get Status.")
    print("Hit 't' to get TC temperature")
    print("Hit 'v' to get version numbers.")
    print("Hit 'x' for self calibration.")

    ch = input('\n')
    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb2416.Blink(count)
    elif ch == 'c':
      usb2416.CounterInit(usb2416.COUNTER0)
      print('Connect DIO0 to CTR0')
      toContinue()
      for i in range(100) :
        usb2416.DOut(0x1)
        usb2416.DOut(0x0)
      print('Count = ',usb2416.Counter(usb2416.COUNTER0),'   Should read 100')
    elif ch == 'e':
      usb2416.udev.close()
      exit(0)
    elif ch == 'd':
      print('Testing Read Digital I/O ...')
      data = usb2416.DIn()
      print('port 0: value = ', hex(data))
    elif ch == 'D':
      print('Testing Write Digital I/O ...')
      value = int(input('Enter a hex number [0-0xff]: '), 16)
      usb2416.DOut(value)
      data = usb2416.DOutR()
      print('The number you entered = ', hex(data))
    elif ch == 'i':
      channel = int(input('Input channel [0-7]: '))
      ch = int(input('Input gain range: 1 = 10V  2 = 5V  3 = 2.5V Differential: '))
      if ch == 1:
        gain = usb2416.BP_10_00V
      elif ch == 2:
        gain = usb2416.BP_5_00V
      elif ch == 3:
        gain = usb2416.BP_2_50V
      else:
        print('Unknown value')
        continue
      rate = usb2416.HZ1000
      mode = usb2416.DIFFERENTIAL
      for i in range(20):
        data, flags = usb2416.AIn(channel, mode, gain, rate)
        data = int(data*usb2416.Cal[gain].slope + usb2416.Cal[gain].intercept)
        print('Channel %2i Sample[%1d] = %#x  Volts = %lf' %(channel,i,data,usb2416.volts(gain,data)))
        time.sleep(1.0)
    elif ch == 'I':
      print('Testing USB-2416 Analog Input Scan')
      usb2416.AInScanStop()
      channel = int(input('Input channel [0-7]: '))
      nScan = int(input('Input the number of scans [1-512]: '))
      ch = int(input('Input gain range: 1 = 10V  2 = 5V  3 = 2.5V Differential: '))
      if ch == 1:
        gain = usb2416.BP_10_00V
      elif ch == 2:
        gain = usb2416.BP_5_00V
      elif ch == 3:
        gain = usb2416.BP_2_50V
      else:
        print('Unknown value')
        gain = usb2416.BP_10_00V
      rate = usb2416.HZ1000;
      mode = usb2416.DIFFERENTIAL;

      usb2416.Queue[0] = 3              # set queue depth, in this case 3 entries per scan
      usb2416.Queue[1].channel = channel
      usb2416.Queue[1].mode = mode      
      usb2416.Queue[1].gain = gain
      usb2416.Queue[1].rate = rate

      usb2416.Queue[2].channel = channel
      usb2416.Queue[2].mode = mode      
      usb2416.Queue[2].gain = gain
      usb2416.Queue[2].rate = rate

      usb2416.Queue[3].channel = channel
      usb2416.Queue[3].mode = mode      
      usb2416.Queue[3].gain = gain
      usb2416.Queue[3].rate = rate

      usb2416.AInScanQueue()                # load the gain queue
      usb2416.AInScanStart(900., nScan, 15)  # sample at 900 Hz
      options = 0
      data = usb2416.AInScanRead(nScan, options)
      for i in range(nScan*usb2416.Queue[0]):
        queue_index = (data[i] >> 24) + 1                  # MSB of data contains the queue index
        gain = usb2416.Queue[queue_index].gain
        channel = usb2416.Queue[queue_index].channel
        data[i] = usb2416.int24ToInt(data[i])        # convert raw 24 bit signed to 32 bit int
        value = int(data[i]*usb2416.Cal[gain].slope + usb2416.Cal[gain].intercept)
        print('Sample %i Index %i Channel %i  gain = %i raw = %#x voltage = %f' \
              %(i, queue_index, channel, gain, value, usb2416.volts(gain,value)))
    elif ch == 'C':
      print('Testing USB-2416 Analog Input Scan in Continuous mode 8 channels')
      print('Hit any key to exit')
      usb2416.AInScanStop()
      count = 0   # for continuous mode
      rate = usb2416.HZ1000
      mode = usb2416.DIFFERENTIAL
      gain = usb2416.BP_10_00V
      usb2416.Queue[0] = 8
      for i in range(1,9):
        usb2416.Queue[i].channel = i-1
        usb2416.Queue[i].mode = mode
        usb2416.Queue[i].gain = gain
        usb2416.Queue[i].rate = rate
      usb2416.AInScanQueue()
      usb2416.AInScanStart(100, count, 15)
      j = 0
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        data = usb2416.AInScanRead(64,usb2416.CONTINUOUS)
#        for i in range(512):
#          queue_index = (data[i] >> 24) + 1                  # MSB of data contains the queue index
#          gain = usb2416.Queue[queue_index].gain
#          channel = usb2416.Queue[queue_index].channel
#          data[i] = usb2416.int24ToInt(data[i])        # convert raw 24 bit signed to 32 bit int
#          value = int(data[i]*usb2416.Cal[gain].slope + usb2416.Cal[gain].intercept)
#          print('Sample %i Index %i Channel %i  gain = %i raw = %#x voltage = %f' \
#                %(i, queue_index, channel, gain, value, usb2416.volts(gain,value)))
        j += 1
        if j%10 == 0:
          print('j = ',j)
        c = sys.stdin.readlines()
        if (len(c) != 0):
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          break
      usb2416.AInScanStop()
    elif ch == 'j':
      cjc = usb2416.CJC()
      for i in range(8):
        print('CJC sensor[%d] = %.2f' %(i, cjc[i]), 'degree C.')
    elif ch == 'r':
      usb2416.Reset()
    elif ch == 's':
      print("Serial No: %s" % usb2416.getSerialNumber())
    elif ch == 'S':
      if usb2416.Status() & 0x1:
        print('Isolated micro ready for commands.')
      else:
        print('Isolated micro not ready for commands.')
      if usb2416.Status() & 0x2:
        print('EXP detected')
      else:
        print('EXP not detected')
    elif ch == 'v':
      print("Manufacturer: %s" % usb2416.getManufacturer())
      print("Product: %s" % usb2416.getProduct())
      print("Serial No: %s" % usb2416.getSerialNumber())
      version = usb2416.Version()
      print('USB micro firmware version = ', version[0])
      print('USB update firmware version = ', version[1])
      print('USB isolated micro firmware version = ', version[2])
      print('USB isolated update firmware version = ', version[3])
    elif ch == 't':
      channel = int(input('Input channel [0-7]: '))
      tc_type = input('Input Thermocouple type [B,E,J,K,R,S,T,N]: ')
      for i in range(10):
        temperature = usb2416.Temperature(tc_type, channel)
        tempF = temperature*9./5 + 32.
        print('Temp = %.3f C   %.3f F' %(temperature, tempF))
        time.sleep(1)
    elif ch == 'x':
      usb2416.ADCal()
    elif ch == 'o':
      channel = int(input('Input Channel [0-3]: '))
      while True:
        voltage = float(input('Enter output voltage [-10 to 10]: '))
        try:
          usb2416.AOut(channel, voltage)
        except AttributeError:
          print('AttributeError: USB-2416-4AO only')
          break;
        if toContinue() == False:
          break
    elif ch == 'O':
      try:
        usb2416.AOutScanStop()
      except AttributeError:
        print('AttributeError: USB-2416-4AO only')
        continue
      channel = int(input('Input Channel [0-3]: '))
      options = 0x1 << channel
      print('Test of Analog Ouput Scan')
      print('Hook up scope to VDAC Channel', channel)
      frequency = float(input('Enter desired frequency of sine wave [Hz]: '))
      sine = [0]*512
      data = [0]*512
      for i in range(512):
        sine[i] = 10*math.sin(2*math.pi*i/128.)
        data[i] = sine[i]*(1<<15)/10.
        data[i] = data[i]*usb2416.Cal_AO[channel].slope + usb2416.Cal_AO[channel].intercept
        if data[i] > 32767:
          data [i] = 0x7fff
        elif data[i] < -32768:
          data[i] = 0x8000
        else:
          data[i] = int(data[i])

      usb2416.AOutScanStart(128*frequency, 0, options)
      print("Hit 's <CR>' to stop")
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        try:
          usb2416.AOutScanWrite(data)
        except:
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb2416.AOutScanStop()
          break
        c = sys.stdin.readlines()
        if (len(c) != 0):
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb2416.AOutScanStop()
          break
            
if __name__ == "__main__":
  main()
