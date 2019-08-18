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
    ctr08 = usb_ctr08()
    print("USB-CTR08 device found.")
  except:
    print('No USB-CTR08 device found.')
    return

  print("wMaxPacketSize = ", ctr08.wMaxPacketSize)

  while True:
    print("\nUSB-CTR08 Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("hit 'd' to test digital I/O")
    print("Hit 's' to get serial number")
    print("Hit 'S' to get the status")
    print("Hit 'e' to exit.")
    print("Hit 'c' to test counter")
    print("Hit 'P' to read the counter parameters")
    print("Hit 't' to test the timers")
    print("Hit 'T' to read timer parameters")
    print("Hit 'L' to read the scan list")
    print("Hit 'M' to read memory")
    print("Hit 'v' for version.")

    ch = input('\n')
    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      ctr08.BlinkLED(count)
    elif ch == 'd':
      print("Testing Digital I/O ...")
      print("connect pins DIO[0-3] <--> DIO[4-7]")
      ctr08.DTristateW(0xf0)
      print("Digital Port Tristate Register = ", hex(ctr08.DTristateR()))
      while True:
        value = int(input('Enter a byte number [0-0xf]: '),16) & 0xf
        ctr08.DLatchW(value)
        value2 = ctr08.DLatchR()
        value3 = ctr08.DPort() >> 4
        print("The number you entered: ", hex(value3), "  Latched value: ", hex(value2))
        if toContinue() != True:
          break
    elif ch == 's':
      print('Serial Number: ', ctr08.GetSerialNumber())
    elif ch == 'S':
      status = ctr08.Status()
      print('Status = ', hex(status))
      if status & ctr08.PACER_RUNNING:
        print("USB-CTR08: Pacer running.")
      if status & ctr08.SCAN_OVERRUN:
        print("USB-CTR08: Scan overrun.")
      if status & ctr08.SCAN_DONE:
        print("USB-CTR08: Scan done.")
      if status & ctr08.FPGA_CONFIGURED:
        print("USB-CTR08: FPGA configured.")
      if status & ctr08.FPGA_CONFIG_MODE:
        print("USB-CTR08: FPGA config mode.")
    elif ch == 'e':
      ctr08.udev.close()
      exit(0)
    elif ch == 'v':
      version = ctr08.FPGAVersion()
      print("FPGA version:", version)
    elif ch == 'c':
      counter = int(input('Enter counter [0-7]: '))
      ctr08.CounterSet(counter, 0x0)        # set counter to 0
      ctr08.CounterModeW(counter, 0x0)
      ctr08.CounterOptionsW(counter, 0)     # count on rising edge
      ctr08.CounterGateConfigW(counter, 0)  # disable gate
      ctr08.CounterOutConfigW(counter, 0)   # output off
      print("Connect DIO0 to CTR#")
      ctr08.DTristateW(0xf0)
      toContinue()
      for i in range(100):
        ctr08.DLatchW(0x0)
        ctr08.DLatchW(0x1)
      print("Count =", ctr08.Counter(counter), "Should read 100.")
    elif ch == 'P':
      for counter in range(ctr08.NCOUNTER):
        ctr08.CounterParamsR(counter)
        print("Counter:", ctr08.counterParameters[counter].counter,                       \
              "\tMode Options:", hex(ctr08.counterParameters[counter].modeOptions),       \
              "\tCounter Options:", hex(ctr08.counterParameters[counter].counterOptions), \
              "\tGate Options:", hex(ctr08.counterParameters[counter].gateOptions),       \
              "\tOutput Options:", hex(ctr08.counterParameters[counter].outputOptions),   \
              "\tdebounce:", hex(ctr08.counterParameters[counter].debounce))
        print("Counter Out Values Min:",hex(ctr08.CounterOutValuesR(counter,0)))
        print("Counter Out Values Max:",hex(ctr08.CounterOutValuesR(counter,1)))
        print("Counter Limit Values Min:",hex(ctr08.CounterLimitValuesR(counter,0)))
        print("Counter Limit Values Max:",hex(ctr08.CounterLimitValuesR(counter,1)))
    elif ch == 't':
      frequency = int(input('Enter frequency of timer: '))
      timer     = int(input('Enter timer [0-3]: '))
      period = int(96.E6/frequency - 1)
      ctr08.TimerPeriodW(timer, frequency)
      ctr08.TimerPulseWidthW(timer, int(period/2))
      ctr08.TimerCountW(timer, 0)
      ctr08.TimerStartDelayW(timer,0)
      ctr08.TimerControlW(timer, 0x1)
      toContinue()
      ctr08.TimerControlW(timer, 0x0)
    elif ch == 'T':
      for timer in range(ctr08.NTIMER):
        ctr08.TimerParamsR(timer)
        print("Timer:", ctr08.timerParameters[timer].timer, \
              "  Control Reg:",hex(ctr08.TimerControlR(timer)), \
              "\tPeriod Reg:",hex(ctr08.timerParameters[timer].period), \
              "\tPulse Width Reg:",hex(ctr08.timerParameters[timer].pulseWidth), \
              "  Count Reg:",hex(ctr08.timerParameters[timer].count), \
              "  Delay Reg:",hex(ctr08.timerParameters[timer].delay))
    elif ch == 'L':
      ctr08.ScanConfigR()
      print("Scan list: ", ctr08.scanList) 
    elif ch == 'M':
      address = int(input("Enter memory address: "),16)
      length = int(input("Enter number of bytes to read: "))
      ctr08.MemAddressW(address)
      print("address = ", hex(ctr08.MemAddressR()))
      print("data: ", ctr08.MemoryR(length))
      
            
if __name__ == "__main__":
  main()


  
