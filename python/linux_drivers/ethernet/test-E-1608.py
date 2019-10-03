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

from E_1608 import *
import time
import sys

def main():
  if len(sys.argv) == 2:
    device = []
    device.append(mccEthernetDevice(E1608_PID,sys.argv[1]))
  else:
    # Find a E-1608 device on the subnet
    device = mccDiscover(E1608_PID)

  print('Number of devices found = ', len(device))
  if (len(device) > 0):
    for i in range(len(device)):
      device[i].printDeviceInfo()
  else:
    exit(0)

  # Open the device
  device[0].mccOpenDevice()

  # initalize the class
  e1608 = E_1608(device[0])

  # print out the calibration tables
  for i in range(NGAINS):
    print('Calibration Table (Differential): Range = ', i, ' Slope = ', e1608.table_AInDF[i].slope, \
          ' Intercept = ', e1608.table_AInDF[i].intercept)
  for i in range(NGAINS):
    print('Calibration Table (Single Ended): Range = ', i, ' Slope = ', e1608.table_AInSE[i].slope, \
          ' Intercept = ', e1608.table_AInSE[i].intercept)
  for i in range(NCHAN_AOUT):
    print('Calibration Table (Analog Output): Channel = ', i, ' Slope = ', e1608.table_AOut[i].slope, \
          ' Intercept = ', e1608.table_AOut[i].intercept)
  
  while True:
    print("\nE-1608 Testing")
    print("----------------")
    print("Hit 'b' to blink.")
    print("Hit 'c' to test counter.")
    print("Hit 'd' to test digital IO.")
    print("Hit 'i' to test Analog Input.")
    print("Hit 'I' to tesr Analog Input Scan.")
    print("Hit 'o' to test Analog Output.")
    print("Hit 'r' to reset the device.")
    print("Hit 'n' to get networking information.")
    print("Hit 's' to get device status.")
    print("Hit 'e' to exit")

    ch = input('\n')
    if ch == 'b':
      count = input('Enter number of times to blink: ')
      e1608.Blink(int(count))
    elif ch == 'c':
      print('Connect DIO0 to CTR')
      e1608.DConfig_W(0xf0)
      e1608.ResetCounter()
      for i in range(20):
        e1608.DOut(0x1)
        time.sleep(0.1)
        e1608.DOut(0x0)
        time.sleep(0.1)
        count = e1608.Counter()
        print('Counter = ', count)
    elif ch == 'd':
      print("\n Testing Digital I/O ...")
      print("connect pins DIO[0-3] <---> DIO[4-7]")
      e1608.DConfig_W(0xf0)  # write to pins 0-3
      value = e1608.DConfig_R()
      print('Digital Port Tristate Register = ',hex(value))
      value = int(input('Enter a number [0-0xf]: '),16)
      e1608.DOut(value)
      value1 = (e1608.DIn()>>4) & 0xf
      value2 = e1608.DOut_R()
      print('The number you entered = ',hex(value1), '  Latched value = ', hex(value2))
    elif ch == 'i':
      chan = int(input('Input channel [0-11]: '))
      gain = int(input('Input range [0-3]: '))
      for i in range(20):
        value = e1608.AIn(chan,gain)
        print('Channel = ', chan, '  Range = ',gain, ' Sample[',i,'] = ',hex(value), '  Volts = ',\
              format(e1608.volts(value, gain),'.4f'))
    elif ch == 'e':
      e1608.device.sock.close()
      exit(0)
    elif ch == 'I':
      e1608.AInScanStop()
      frequency = float(input('Enter sampling frequency [Hz]: '))
      count = int(input('Enter number of scans (less than 512): '))
      nchan = int(input('Enter number of channels per scan [1-8]: '))
      if (nchan < 1 or nchan > 8):
        print('Number of channels must be between 1 and 8.')
        break;
      # set up the gain queue
      e1608.queue[0] = nchan
      for i in range(nchan):
        chan = int(input('Enter channel in gain queue[0-11]: '))
        e1608.queue[2*i+1] = chan & 0xff
        gain = int(input('Enter range [0-3]: '))
        e1608.queue[2*i+2] = gain & 0xff
      e1608.AInQueue_W()
      e1608.AInQueue_R()
      for i in range(e1608.queue[0]):
        print(i+1, 'channel = ',e1608.queue[2*i+1],'  range = ',e1608.queue[2*i+2])
      options = 0x0
      e1608.AInScanStart(count, frequency, options)
      data = e1608.AInScanRead(count, nchan)
      e1608.device.scan_sock.close()
      for i in range(count):
        for j in range(nchan):
          k = i*nchan + j                   # sample number
          channel = e1608.queue[2*j+1]      # channel
          gain = e1608.queue[2*j+2]         # range value
          print('Scan ',i,'Range ',gain,' channel ', j, ' Sample[',k,'] = ', hex(data[k]),\
                ' Volts = ', format(e1608.volts(data[k], gain),'.4f'))
    elif ch == 'o':
      chan = int(input('Enter AOut channel [0-1]: '))
      volts = float(input('Enter desired output voltage [-10V to 10V]: '))
      value = e1608.valueAOut(volts)
      print('AOut: channel = ',chan, ' value = ', hex(value))
      e1608.AOut(chan, value)
    elif ch == 'r':
      e1608.Reset()
    elif ch == 's':
      # Print the calibration data and status
      status = e1608.Status()
      print('status = ', hex(status))
      mdate = e1608.getMFGCAL()
      print('Last Calibration date: ', mdate)
    elif ch == 'n':
      value = e1608.NetworkConfig()
      print('Network configuration values: ')
      print('  IP address = ', value[0])
      print('  subnet mask = ', value[1])
      print('  gateway = ', value[2])
      print('  MAC:', \
            hex(e1608.device.MAC>>40&0xff)[2:].zfill(2)+':'+\
            hex(e1608.device.MAC>>32&0xff)[2:].zfill(2)+':'+\
            hex(e1608.device.MAC>>24&0xff)[2:].zfill(2)+':'+\
            hex(e1608.device.MAC>>16&0xff)[2:].zfill(2)+':'+\
            hex(e1608.device.MAC>>8&0xff)[2:].zfill(2)+':'+\
            hex(e1608.device.MAC&0xff)[2:].zfill(2))
    else:
      pass

if __name__ == "__main__":
  main()





        
                  
