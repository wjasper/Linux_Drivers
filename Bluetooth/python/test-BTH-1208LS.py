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

if __name__ == "__main__":
  main()
