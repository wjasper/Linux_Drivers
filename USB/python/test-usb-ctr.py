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
    print("Hit 'I' for for scan input with continuous readout")
    print("Hit 'C' for continuous scan")
    print("Hit 'f' for frequency count")
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
      print("Manufacturer: %s" % ctr.getManufacturer())
      print("Product: %s" % ctr.getProduct())
    elif ch == 'S':
      ctr.printStatus()      
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
          ctr.ScanConfigW(4*counter+bank, counter, bank, zero_fill=True, lastElement=True)
      ctr.ScanConfigR()
      
      # set up the counters
      for counter in  range(4):  # use the first 4 counters
        ctr.CounterSet(counter, 0x0)        # set counter to 0
        ctr.CounterModeW(counter, 0x0)
        ctr.CounterOptionsW(counter, 0)     # count on rising edge
        ctr.CounterGateConfigW(counter, 0)  # disable gate
        ctr.CounterOutConfigW(counter, 0)   # output off

      # set up the timer to generate some pulses
      timer_frequency = 100000      # 100 pulses per scan
      period = 1000/timer_frequency # period in ms
      timer = 1
      ctr.TimerPeriodW(timer, period)
      ctr.TimerPulseWidthW(timer, period/2)
      ctr.TimerCountW(timer, 0)
      ctr.TimerStartDelayW(timer,0)
      ctr.TimerControlW(timer, 0x1)

      ctr.ScanStart(count, 0, frequency, 0, 0)
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
    elif ch == 'I':
      print("Testing scan input with continuous readout")
      print("Connect Timer 1 to Counter 1")
      count = 100                # total number of scans to perform
      frequency = 1000           # scan rate at 1000 Hz
      numCounters = 4            # 4 counters 

      # Set up the scan list (use 4 counters 0-3)
      for counter in  range(numCounters):            # use the first 4 counters
        for bank in range(ctr.NBANK):                 # each counter has 4 banks of 16-bit registers to be scanned
          ctr.ScanConfigW(ctr.NBANK*counter+bank, counter, bank, zero_fill=True, lastElement=True)
      ctr.ScanConfigR()
      
      # set up the counters
      for counter in range(numCounters):  # use the first 4 counters
        ctr.CounterSet(counter, 0x0)        # set counter to 0
        ctr.CounterModeW(counter, 0x0)
        ctr.CounterOptionsW(counter, 0)     # count on rising edge
        ctr.CounterGateConfigW(counter, 0)  # disable gate
        ctr.CounterOutConfigW(counter, 0)   # output off

      # set up the timer to generate some pulses
      timer_frequency = 100000   # 100 pulses per scan
      period = 1000/timer_frequency
      timer = 1
      ctr.TimerPeriodW(timer, period)
      ctr.TimerPulseWidthW(timer, period/2)
      ctr.TimerCountW(timer, 0)
      ctr.TimerStartDelayW(timer,0)
      ctr.TimerControlW(timer, 0x1)

      # Make sure the FIFO is cleared
      ctr.ScanStop()
      ctr.ScanClearFIFO()
      ctr.BulkFlush(5)

      ctr.ScanStart(count, 0, frequency, 0, ctr.CONTINUOUS_READOUT)
      print("lastElement =", ctr.lastElement, "   packet_size =", ctr.packet_size, "  options =", ctr.scan_options, "mode = ", ctr.scan_mode)
      counter_data = [0, 0, 0, 0, 0, 0, 0, 0]
      currentCounter = 0
      while (currentCounter < count):
        data = ctr.ScanRead(count)
        numSamplesRead = len(data)
        numScansRead = numSamplesRead//(numCounters * ctr.NBANK)
#       print('number of samples read =', numSamplesRead, 'number of Scans Read =', numScansRead)
        for scan in range(numScansRead):
          for counter in range(numCounters):
            offset = scan*numCounters*ctr.NBANK + counter*ctr.NBANK   # there are 4 banks of 16-bit values per counter
            counter_data[counter] = 0
            for bank in range(ctr.NBANK):
              counter_data[counter] += (data[offset+bank] & 0xffff) << (16*bank)
          print("Scan:", currentCounter, end="")
          for counter in range(numCounters):
            print("   ",counter_data[counter], " ", end="")
          print("")
          currentCounter += 1
          if currentCounter > count:
            break
      ctr.TimerControlW(timer, 0x0)
    elif ch == 'C':
      print("Testing Continuous scan input")
      print("Connect Timer 1 to Counter 1")
      count = 0                  # for continuous scan
      frequency = 1000           # scan rate at 1000 Hz

      # Set up the scan list (use 4 counters 0-3)
      for counter in  range(4):  # use the first 4 counters
        for bank in range(4):    # each counter has 4 banks of 16-bit registers to be scanned
          ctr.ScanConfigW(4*counter+bank, counter, bank, zero_fill=True, lastElement=True)
      ctr.ScanConfigR()
      
      # set up the counters
      for counter in  range(4):  # use the first 4 counters
        ctr.CounterSet(counter, 0x0)        # set counter to 0
        ctr.CounterModeW(counter, 0x0)
        ctr.CounterOptionsW(counter, 0)     # count on rising edge
        ctr.CounterGateConfigW(counter, 0)  # disable gate
        ctr.CounterOutConfigW(counter, 0)   # output off

      # set up the timer to generate some pulses
      timer_frequency = 100000          
      period = 1000/timer_frequency
      timer = 1
      ctr.TimerPeriodW(timer, period)
      ctr.TimerPulseWidthW(timer, period/2)
      ctr.TimerCountW(timer, 0)
      ctr.TimerStartDelayW(timer,0)
      ctr.TimerControlW(timer, 0x1)

      ctr.ScanStart(count, 0, frequency, 0)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      scan = 0
      counter_data = [0, 0, 0, 0]
      while True:
        data = ctr.ScanRead(count)
        scan += 1
        if scan == 100:
          break
        print("Scan =", scan, "counter values returned =", len(data)//4, end='')
        # print out the first four values
        offset = 0
        for counter in range(4):
          offset = counter*4   # there are 4 banks of 16-bit values per counter
          counter_data[counter] = data[offset] & 0xffff
          counter_data[counter] += (data[offset+1] & 0xffff) << 16
          counter_data[counter] += (data[offset+2] & 0xffff) << 32
          counter_data[counter] += (data[offset+3] & 0xffff) << 48
        print("   ", counter_data[0], counter_data[1], counter_data[2], counter_data[3])
        c = sys.stdin.readlines()
        if (len(c) != 0):
          ctr.ScanStop()
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)        
      ctr.TimerControlW(timer, 0x0)
      ctr.ScanClearFIFO()
      ctr.BulkFlush(5)
    elif ch == 'f':
      print("Testing frequency count")
      print("Connect Timer 1 to Counter 1")

      # set up the timer to generate some pulses
      timer_frequency = 1000         # 1000 Hz
      period = 1000/timer_frequency  # period in ms
      timer = 1
      ctr.TimerPeriodW(timer, period)
      ctr.TimerPulseWidthW(timer, period/2)
      ctr.TimerCountW(timer, 0)
      ctr.TimerStartDelayW(timer,0)
      ctr.TimerControlW(timer, 0x1)

      # configure counter 1
      counter = 1
      ctr.CounterSet(counter, 0x0)        # set counter to 0
      ctr.CounterModeW(counter, ctr.PERIOD | ctr.PERIOD_MODE_1000X)
      ctr.CounterOptionsW(counter, 0)     # count on rising edge
      ctr.CounterGateConfigW(counter, 0)  # disable gate
      ctr.CounterOutConfigW(counter, 0)   # output off

      time.sleep(2.0)                     
      count = ctr.Counter(counter)
      if count == 0:
        print('Error: count = 0')
        break
      else:
        period = count*20.83E-9/1000.
        frequency = 1/period
        print("count =", count, "   period =", period, "   frequency = {0:.1f} Hz".format(frequency), \
            "   timer frequency =", timer_frequency, "Hz")
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
      timer     = int(input('Enter timer [0-3]: '))
      frequency = float(input('Enter frequency of timer: '))
      period = 1000/frequency       # period in ms
      ctr.TimerPeriodW(timer, period)
      ctr.TimerPulseWidthW(timer, period/2)
      ctr.TimerCountW(timer, 0)
      ctr.TimerStartDelayW(timer, period/10)
      ctr.TimerControlW(timer, 0x1)
      toContinue()
      ctr.TimerControlW(timer, 0x0)
#      print(ctr.TimerParamsR(timer))
      print("Timer:", ctr.timerParameters[timer].timer, \
            "  Control Reg:",hex(ctr.TimerControlR(timer)), \
            "\tPeriod:", ctr.TimerPeriodR(timer),"ms" \
            "\tPulse Width:", ctr.TimerPulseWidthR(timer),"ms"\
            "    \tCount Reg:",ctr.TimerCountR(timer), \
            "    \tDelay:", ctr.TimerStartDelayR(timer),"ms")
    elif ch == 'T':
      for timer in range(ctr.NTIMER):
        print("Timer:", ctr.timerParameters[timer].timer, \
              "  Control Reg:",hex(ctr.TimerControlR(timer)), \
              "\tPeriod:",ctr.TimerPeriodR(timer),"ms" \
              "\tPulse Width:",ctr.TimerPulseWidthR(timer),"ms" \
              "    \tCount Reg:",hex(ctr.TimerCountR(timer)), \
              "    \tDelay Reg:",ctr.TimerStartDelayR(timer),"ms")
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


  
