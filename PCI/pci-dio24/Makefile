#############################################################################
#	
#	Makefile for building:
#
#		pci-dio24.o:  PCI-DIO24 adapater Linux loadable module.
#		test-dio24:   Program to test dio24 module
#
#               Copyright (C) 2004 - 2015
#               Written by:  Warren J. Jasper <wjasper@tx.ncsu.edu>
#                            North Carolina State Univerisity
#
#
# This program, PCI-DIO24, is free software; you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version, provided that this
# copyright notice is preserved on all copies.
#
# ANY RIGHTS GRANTED HEREUNDER ARE GRANTED WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, AND FURTHER,
# THERE SHALL BE NO WARRANTY AS TO CONFORMITY WITH ANY USER MANUALS OR
# OTHER LITERATURE PROVIDED WITH SOFTWARE OR THAM MY BE ISSUED FROM TIME
# TO TIME. IT IS PROVIDED SOLELY "AS IS".
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
###########################################################################

#  Current Version of the driver
VERSION=2.16

#  Number of PCI-DIO24 boards on your system: (1-4)
#  MUST ALSO CHANGE MAX_BOARDS in dio.h
NUM_BOARDS=1

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

# Major Number of device
#MAJOR_DEV=249

ID=PCI-DIO24
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= dio24.o
#pci-dio24-objs := dio24.o

# 2.4 kernel compatibility
modules: pci-dio24.o

pci-dio24.o: dio24.o
	$(LD) -r -o $@ dio24.o

dio24.o: dio24.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-dio24.o test-dio24 dio24.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {dio24.c,dio24_2_4.c,dio24_2_6.c,dio24_2_6_29.c,dio24_3_3_7.c,dio24_3_10_11.c,dio24_4_0_8.c,test-dio24.c,dio24.h,pci-dio24.h,Makefile,README,ModList,License,PCI-DIO24x.pdf,RegMapPCI-DIO24X.pdf,PCIe-DIO24.pdf}

all: default test-dio24

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-dio24:	test-dio24.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-dio24.*.cmd pci-dio24.mod.c .dio24.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-dio24/$(DIST_FILES);

install: 
	-/sbin/rmmod dio24
	-/bin/cp ./pci-dio24.h /usr/local/include/pci-dio24.h
	-/bin/chmod 644 /usr/local/include/pci-dio24.h 
	install -d $(MODULE_DIR)
	if [ -f ./dio24.ko ]; then \
	/sbin/insmod  dio24.ko; \
	install -c ./dio24.ko $(MODULE_DIR); \
	else \
	/sbin/insmod  pci-dio24.o; \
	install -c ./pci-dio24.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod dio24
	-/bin/rm -f /dev/dio24*
	if [ -f $(MODULE_DIR)/dio24.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/dio24.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-dio24.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-dio24.o; \
	fi

devices:
	-/bin/rm -f /dev/dio24*
	-/bin/mknod /dev/dio24_0A  c $(MAJOR_DEV) 0
	-/bin/mknod /dev/dio24_0B  c $(MAJOR_DEV) 1
	-/bin/mknod /dev/dio24_0C  c $(MAJOR_DEV) 2

	-if [ $(NUM_BOARDS) -ge 2 ];  then \
	/bin/mknod /dev/dio24_1A c $(MAJOR_DEV) 16; \
	/bin/mknod /dev/dio24_1B c $(MAJOR_DEV) 17; \
	/bin/mknod /dev/dio24_1C c $(MAJOR_DEV) 18; \
	fi

	-if [ $(NUM_BOARDS) -ge 3 ];  then \
	/bin/mknod /dev/dio24_2A  c $(MAJOR_DEV) 32; \
	/bin/mknod /dev/dio24_2B  c $(MAJOR_DEV) 33; \
	/bin/mknod /dev/dio24_2C  c $(MAJOR_DEV) 34; \
	fi

	-if [ $(NUM_BOARDS) -ge 4 ];  then \
	/bin/mknod /dev/dio24_3A  c $(MAJOR_DEV) 48; \
	/bin/mknod /dev/dio24_3B  c $(MAJOR_DEV) 49; \
	/bin/mknod /dev/dio24_3C  c $(MAJOR_DEV) 50; \
	fi

	-/bin/chmod 666 /dev/dio24*
endif
