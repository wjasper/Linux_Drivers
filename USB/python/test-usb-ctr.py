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
from usb_ctr import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    ctr = usb_ctr08()
    print("USB-CTR08 device found.")
  except:
    try:
      ctr = usb_ctr04()
      print("USB-CTR04 device found.")
    except:      
     print('No USB-CTR08 or USB-CTR04 device found.')
     return

  print("wMaxPacketSize = ", ctr.wMaxPacketSize)

  while True:
    print("\nUSB-CTR Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("hit 'd' to test digital I/O")
    print("Hit 's' to get serial number")
    print("Hit 'S' to get the status")
    print("Hit 'e' to exit.")
    print("Hit 'c' to test counter")
    print("Hit 'P' to read the counter parameters")
    print("Hit 'i' for scan input")
    print("Hit 't' to test the timers")
    print("Hit 'T' to read timer parameters")
    print("Hit 'L' to read the scan list")
    print("Hit 'M' to read memory")
    print("Hit 'v' for version.")

    ch = input('\n')
    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      ctr.BlinkLED(count)
    elif ch == 'd':
      print("Testing Digital I/O ...")
      print("connect pins DIO[0-3] <--> DIO[4-7]")
      ctr.DTristateW(0xf0)
      print("Digital Port Tristate Register = ", hex(ctr.DTristateR()))
      while True:
        value = int(input('Enter a byte number [0-0xf]: '),16) & 0xf
        ctr.DLatchW(value)
        value2 = ctr.DLatchR()
        value3 = ctr.DPort() >> 4
        print("The number you entered: ", hex(value3), "  Latched value: ", hex(value2))
        if toContinue() != True:
          break
    elif ch == 's':
      print('Serial Number: ', ctr.GetSerialNumber())
    elif ch == 'S':
      status = ctr.Status()
      print('Status = ', hex(status))
      if status & ctr.PACER_RUNNING:
        print("USB-CTR: Pacer running.")
      if status & ctr.SCAN_OVERRUN:
        print("USB-CTR: Scan overrun.")
      if status & ctr.SCAN_DONE:
        print("USB-CTR: Scan done.")
      if status & ctr.FPGA_CONFIGURED:
        print("USB-CTR: FPGA configured.")
      if status & ctr.FPGA_CONFIG_MODE:
        print("USB-CTR: FPGA config mode.")
    elif ch == 'e':
      ctr.udev.close()
      exit(0)
    elif ch == 'v':
      version = ctr.FPGAVersion()
      print("FPGA version:", version)
    elif ch == 'c':
      if ctr.NCOUNTER == 8:
        counter = int(input('Enter counter [0-7]: '))
      else:
        counter = int(input('Enter counter [0-3]: '))
      ctr.CounterSet(counter, 0x0)        # set counter to 0
      ctr.CounterModeW(counter, 0x0)
      ctr.CounterOptionsW(counter, 0)     # count on rising edge
      ctr.CounterGateConfigW(counter, 0)  # disable gate
      ctr.CounterOutConfigW(counter, 0)   # output off
      print("Connect DIO0 to CTR#")
      ctr.DTristateW(0xf0)
      toContinue()
      for i in range(100):
        ctr.DLatchW(0x0)
        ctr.DLatchW(0x1)
      print("Count =", ctr.Counter(counter), "Should read 100.")
    elif ch == 'i':
      print("Testing scan input")
      print("Connect Timer 1 to Counter 1")
      count = 100                # total number of scans to perform
      frequency = 1000           # scan rate at 1000 Hz

      # Set up the scan list (use 4 counters 0-3)
      for counter in  range(4):  # use the first 4 counters
        for bank in range(4):    # each counter has 4 banks of 16-bit registers to be scanned
          ctr.scanList[4*counter + bank] = (counter & 0x7) | (bank & 0x3) << 3 | (0x2 << 5)
      ctr.lastElement = 15        # depth of scan list [0-32]
      ctr.ScanConfigW()
      ctr.ScanConfigR()
      
      # set up the counters
      for counter in  range(4):  # use the first 4 counters
        ctr.CounterSet(counter, 0x0)        # set counter to 0
        ctr.CounterModeW(counter, 0x0)
        ctr.CounterOptionsW(counter, 0)     # count on rising edge
        ctr.CounterGateConfigW(counter, 0)  # disable gate
        ctr.CounterOutConfigW(counter, 0)   # output off

      # set up the timer to generate some pulses
      timer_frequency = 4000   # 4 pulses per scan
      period = int(96.E6/timer_frequency - 1)
      timer = 1
      ctr.TimerPeriodW(timer, period)
      ctr.TimerPulseWidthW(timer, int(period/2))
      ctr.TimerCountW(timer, 0)
      ctr.TimerStartDelayW(timer,0)
      ctr.TimerControlW(timer, 0x1)

      ctr.ScanStart(count, 0, frequency, 0)
      data = ctr.ScanRead(count)
      counter_data = [0, 0, 0, 0]
      for scan in range(count):          # total number of scans
        for counter in range(4):         # number of counters
          offset = scan*16 + counter*4   # there are 4 banks of 16-bit values per counter
          counter_data[counter] = data[offset] & 0xffff
          counter_data[counter] += (data[offset+1] & 0xffff) << 16
          counter_data[counter] += (data[offset+2] & 0xffff) << 32
          counter_data[counter] += (data[offset+3] & 0xffff) << 48
        print("Scan:", scan, "   ", counter_data[0], counter_data[1], counter_data[2], counter_data[3])
      ctr.TimerControlW(timer, 0x0)
    elif ch == 'P':
      for counter in range(ctr.NCOUNTER):
        ctr.CounterParamsR(counter)
        print("Counter:", ctr.counterParameters[counter].counter,                       \
              "\tMode Options:", hex(ctr.counterParameters[counter].modeOptions),       \
              "\tCounter Options:", hex(ctr.counterParameters[counter].counterOptions), \
              "\tGate Options:", hex(ctr.counterParameters[counter].gateOptions),       \
              "\tOutput Options:", hex(ctr.counterParameters[counter].outputOptions),   \
              "\tdebounce:", hex(ctr.counterParameters[counter].debounce))
        print("Counter Out Values Min:",hex(ctr.CounterOutValuesR(counter,0)))
        print("Counter Out Values Max:",hex(ctr.CounterOutValuesR(counter,1)))
        print("Counter Limit Values Min:",hex(ctr.CounterLimitValuesR(counter,0)))
        print("Counter Limit Values Max:",hex(ctr.CounterLimitValuesR(counter,1)))
    elif ch == 't':
      frequency = int(input('Enter frequency of timer: '))
      timer     = int(input('Enter timer [0-3]: '))
      period = int(96.E6/frequency - 1)
      ctr.TimerPeriodW(timer, period)
      ctr.TimerPulseWidthW(timer, int(period/2))
      ctr.TimerCountW(timer, 0)
      ctr.TimerStartDelayW(timer,0)
      ctr.TimerControlW(timer, 0x1)
      toContinue()
      ctr.TimerControlW(timer, 0x0)
    elif ch == 'T':
      for timer in range(ctr.NTIMER):
        print("Timer:", ctr.timerParameters[timer].timer, \
              "  Control Reg:",hex(ctr.TimerControlR(timer)), \
              "\tPeriod Reg:",hex(ctr.TimerPeriodR(timer)), \
              "\tPulse Width Reg:",hex(ctr.TimerPulseWidthR(timer)), \
              "    \tCount Reg:",hex(ctr.TimerCountR(timer)), \
              "    \tDelay Reg:",hex(ctr.TimerStartDelayR(timer)))
    elif ch == 'L':
      ctr.ScanConfigR()
      print("Scan list: ", end='')
      for i in range(33):
        print(hex(ctr.scanList[i]),"", end='')
      print(" ")
    elif ch == 'M':
      address = int(input("Enter memory address: "),16)
      length = int(input("Enter number of bytes to read: "))
      ctr.MemAddressW(address)
      print("address = ", hex(ctr.MemAddressR()))
      print("data: ", ctr.MemoryR(length))
      
            
if __name__ == "__main__":
  main()


  
