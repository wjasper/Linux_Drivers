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
    usb2408 = usb_2408()
  except:
    try:
      usb2408 = usb_2408_2AO()
    except:
      print('No device found')
      return

  print('wMaxPacketSize =', usb2408.wMaxPacketSize)
  for gain in range(1,10):
    print('Calibration Table: Range =', gain, \
          'Slope = ',format(usb2408.Cal[gain].slope,'.5f'),\
          'Intercept = ',format(usb2408.Cal[gain].intercept,'5f'))

  if usb2408.productID == 0x00fe:    
    print('\nAnalog Out Calibration')
    for chan in range(0,2):
      print('Calibration Table: Channel =', chan, \
            'Slope = ',format(usb2408.Cal_AO[chan].slope,'.5f'),\
          'Intercept = ',format(usb2408.Cal_AO[chan].intercept,'5f'))
  print('')
  for chan in range(8):
    print('CJC Gradient: Chan =', chan, \
          ' CJC Gradient =',format(usb2408.CJCGradient[chan], '.5f'))

  print('\nMFG Calibration date: ', usb2408.getMFGCAL())
  
  while True:
    print("\nUSB-2408 and USB-2408-2AO Testing")
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
    if usb2408.productID == 0x00fe: # Only for USB-2408-2A0
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
      usb2408.Blink(count)
    elif ch == 'c':
      usb2408.CounterInit(usb2408.COUNTER0)
      print('Connect DIO0 to CTR0')
      toContinue()
      for i in range(100) :
        usb2408.DOut(0x1)
        usb2408.DOut(0x0)
      print('Count = ',usb2408.Counter(usb2408.COUNTER0),'   Should read 100')
    elif ch == 'e':
      usb2408.udev.close()
      exit(0)
    elif ch == 'd':
      print('Testing Read Digital I/O ...')
      data = usb2408.DIn()
      print('port 0: value = ', hex(data))
    elif ch == 'D':
      print('Testing Write Digital I/O ...')
      value = int(input('Enter a hex number [0-0xff]: '), 16)
      usb2408.DOut(value)
      data = usb2408.DOutR()
      print('The number you entered = ', hex(data))
    elif ch == 'i':
      ch = int(input('Input mode: 1 = Differential, 2 = Single-Ended: '))
      if ch == 1:
        mode = usb2408.DIFFERENTIAL
      elif ch == 2:
        mode = usb2408.SINGLE_ENDED
      else:
        print('Unknown mode')
        continue
      if mode == usb2408.DIFFERENTIAL:
        channel = int(input('Input channel [0-7]: '))
      else:
        channel = int(input('Input channel [0-15]: '))
      ch = int(input('Input gain range: 1 = 10V  2 = 5V  3 = 2.5V: '))
      if ch == 1:
        gain = usb2408.BP_10_00V
      elif ch == 2:
        gain = usb2408.BP_5_00V
      elif ch == 3:
        gain = usb2408.BP_2_50V
      else:
        print('Unknown gain range')
        continue
      rate = usb2408.HZ1000
      for i in range(20):
        data, flags = usb2408.AIn(channel, mode, gain, rate)
        data = int(data*usb2408.Cal[gain].slope + usb2408.Cal[gain].intercept)
        print('Channel %2i Sample[%1d] = %#x  Volts = %lf' %(channel,i,data,usb2408.volts(gain,data)))
        time.sleep(1.0)
    elif ch == 'I':
      print('Testing USB-2408 Analog Input Scan')
      usb2408.AInScanStop()
      channel = int(input('Input channel [0-7]: '))
      nScan = int(input('Input the number of scans [1-512]: '))
      ch = int(input('Input gain range: 1 = 10V  2 = 5V  3 = 2.5V Differential: '))
      if ch == 1:
        gain = usb2408.BP_10_00V
      elif ch == 2:
        gain = usb2408.BP_5_00V
      elif ch == 3:
        gain = usb2408.BP_2_50V
      else:
        print('Unknown value')
        gain = usb2408.BP_10_00V
      rate = usb2408.HZ1000;
      mode = usb2408.DIFFERENTIAL;

      usb2408.Queue[0] = 3              # set queue depth, in this case 3 entries per scan
      usb2408.Queue[1].channel = channel
      usb2408.Queue[1].mode = mode      
      usb2408.Queue[1].gain = gain
      usb2408.Queue[1].rate = rate

      usb2408.Queue[2].channel = channel
      usb2408.Queue[2].mode = mode      
      usb2408.Queue[2].gain = gain
      usb2408.Queue[2].rate = rate

      usb2408.Queue[3].channel = channel
      usb2408.Queue[3].mode = mode      
      usb2408.Queue[3].gain = gain
      usb2408.Queue[3].rate = rate

      usb2408.AInScanQueue()                # load the gain queue
      usb2408.AInScanStart(900., nScan, 15)  # sample at 900 Hz
      options = 0
      data = usb2408.AInScanRead(nScan, options)
      for i in range(nScan*usb2408.Queue[0]):
        queue_index = (data[i] >> 24) + 1                  # MSB of data contains the queue index
        gain = usb2408.Queue[queue_index].gain
        channel = usb2408.Queue[queue_index].channel
        data[i] = usb2408.int24ToInt(data[i])        # convert raw 24 bit signed to 32 bit int
        value = int(data[i]*usb2408.Cal[gain].slope + usb2408.Cal[gain].intercept)
        print('Sample %i Index %i Channel %i  gain = %i raw = %#x voltage = %f' \
              %(i, queue_index, channel, gain, value, usb2408.volts(gain,value)))
    elif ch == 'C':
      print('Testing USB-2408 Analog Input Scan in Continuous mode 8 channels')
      print('Hit any key to exit')
      usb2408.AInScanStop()
      count = 0   # for continuous mode
      rate = usb2408.HZ1000
      mode = usb2408.DIFFERENTIAL
      gain = usb2408.BP_10_00V
      usb2408.Queue[0] = 8
      for i in range(1,9):
        usb2408.Queue[i].channel = i-1
        usb2408.Queue[i].mode = mode
        usb2408.Queue[i].gain = gain
        usb2408.Queue[i].rate = rate
      usb2408.AInScanQueue()
      usb2408.AInScanStart(100, count, 15)
      j = 0
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        data = usb2408.AInScanRead(64,usb2408.CONTINUOUS)
#        for i in range(512):
#          queue_index = (data[i] >> 24) + 1                  # MSB of data contains the queue index
#          gain = usb2408.Queue[queue_index].gain
#          channel = usb2408.Queue[queue_index].channel
#          data[i] = usb2408.int24ToInt(data[i])        # convert raw 24 bit signed to 32 bit int
#          value = int(data[i]*usb2408.Cal[gain].slope + usb2408.Cal[gain].intercept)
#          print('Sample %i Index %i Channel %i  gain = %i raw = %#x voltage = %f' \
#                %(i, queue_index, channel, gain, value, usb2408.volts(gain,value)))
        j += 1
        if j%10 == 0:
          print('j = ',j)
        c = sys.stdin.readlines()
        if (len(c) != 0):
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          break
      usb2408.AInScanStop()
    elif ch == 'j':
      cjc = usb2408.CJC()
      for i in range(2):
        print('CJC sensor[%d] = %.2f' %(i, cjc[i]), 'degree C.')
    elif ch == 'r':
      usb2408.Reset()
    elif ch == 's':
      print("Serial No: %s" % usb2408.getSerialNumber())
    elif ch == 'S':
      if usb2408.Status() == 1:
        print('Isolated micro ready for commands.')
      else:
        print('Isolated micro not ready for commands.')
    elif ch == 'v':
      print("Manufacturer: %s" % usb2408.getManufacturer())
      print("Product: %s" % usb2408.getProduct())
      print("Serial No: %s" % usb2408.getSerialNumber())
      version = usb2408.Version()
      print('USB micro firmware version = ', version[0])
      print('USB update firmware version = ', version[1])
      print('USB isolated micro firmware version = ', version[2])
      print('USB isolated update firmware version = ', version[3])
    elif ch == 't':
      channel = int(input('Input channel [0-7]: '))
      tc_type = input('Input Thermocouple type [B,E,J,K,R,S,T,N]: ')
      for i in range(10):
        temperature = usb2408.Temperature(tc_type, channel)
        tempF = temperature*9./5 + 32.
        print('Temp = %.3f C   %.3f F' %(temperature, tempF))
        time.sleep(1)
    elif ch == 'x':
      usb2408.ADCal()
    elif ch == 'o':
      channel = int(input('Input Channel [0-1]: '))
      while True:
        voltage = float(input('Enter output voltage [-10 to 10]: '))
        try:
          usb2408.AOut(channel, voltage)
        except AttributeError:
          print('AttributeError: USB-2408-2AO only')
          break;
        if toContinue() == False:
          break
    elif ch == 'O':
      try:
        usb2408.AOutScanStop()
      except AttributeError:
        print('AttributeError: USB-2408-2AO only')
        continue
      channel = int(input('Input Channel [0-1]: '))
      options = 0x1 << channel
      print('Test of Analog Ouput Scan')
      print('Hook up scope to VDAC Channel', channel)
      frequency = float(input('Enter desired frequency of sine wave [Hz]: '))
      sine = [0]*512
      data = [0]*512
      for i in range(512):
        sine[i] = 10*math.sin(2*math.pi*i/64.)
        data[i] = sine[i]*32768./10. + 32768.
        data[i] = data[i]*usb2408.Cal_AO[channel].slope + usb2408.Cal_AO[channel].intercept
        if data[i] > 65535.:
          data [i] = 0xffff
        elif data[i] < 0:
          data[i] = 0x0
        else:
          data[i] = int(data[i]) 
      usb2408.AOutScanStart(frequency, 0, options)
      print("Hit 's <CR>' to stop")
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        try:
          usb2408.AOutScanWrite(data)
        except:
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb2408.AOutScanStop()
          break
        c = sys.stdin.readlines()
        if (len(c) != 0):
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb2408.AOutScanStop()
          break
            
if __name__ == "__main__":
  main()
