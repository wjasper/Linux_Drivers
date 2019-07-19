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

from bth_1208LS import *
import time
import sys

def main():
  if len(sys.argv) == 2:
    device = []
    device.append(mccBluetoothDevice(sys.argv[1]))
  else:
    # Discover a BTH-1208LS device
    device = discoverDevice("BTH-1208LS-6833")

  if (len(device) > 0):
      print('Number of devices found = ', len(device))
  else:
    exit(0)

  # Open the device
  device[0].openDevice()

  # initalize the class
  bth1208LS = BTH_1208LS(device[0])

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

  while True:
    print("\nBTH-1208LS Testing")
    print("----------------")
    print("Hit 'b' to blink.")
    print("Hit 'c' to test counter.")
    print("Hit 'd' to test digitial IO.")
    print("Hit 'e' to exit.")
    print("Hit 's' to get serial number.")
    print("Hit 'S' to get Status.")
    print("Hit 'v' to get the battery voltage in mV.")

    ch = input('\n')
    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      bth1208LS.BlinkLED(count)
    elif ch == 'e':
      bth1208LS.device.sock.close()
      exit(0)
    elif ch == 's':
      print('Serial Number:',bth1208LS.GetSerialNumber())
    elif ch == 'S':
      status = bth1208LS.Status()
      version = bth1208LS.FirmwareVersion()
      radioVersion = bth1208LS.RadioFirmwareVersion()
      print("Status:", hex(status), "    Firmware Version: {0:1x}.{1:2x}    Radio FirmwareVersion: {2:1x}.{3:2x}"\
            .format ((version >> 8) & 0xff, (version & 0xff), (radioVersion >> 8) & 0xff, (radioVersion & 0xff)))
    elif ch == 'r':
      bth1208LS.Reset()
    elif ch == 'v':
      voltage = bth1208LS.BatteryVoltage()
      print("Battery Voltage {0:d} mV".format(voltage))


if __name__ == "__main__":
  main()
