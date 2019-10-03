#############################################################################
#	
#	Makefile for building:
#
#		pci-dio24H.o:  PCI-DIO24H adapater Linux loadable module.
#		test-dio24H:   Program to test dio24H module
#
#               Copyright (C) September 7, 1999
#               Written by:  Warren J. Jasper
#                            North Carolina State Univerisity
#
#
# This program, PCI-DIO24H, is free software; you can redistribute it
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
VERSION=1.11

#  Number of PCI-DIO24H boards on your system:
#  MUST ALSO CHANGE MAX_BOARDS in dio24H.h
NUM_BOARDS=1

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

# Major Number of device
#MAJOR_DEV=249

ID=PCI-DIO24H
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= dio24H.o
#pci-dio24H-objs := dio24H.o

# 2.4 kernel compatibility
modules: pci-dio24H.o

pci-dio24H.o: dio24H.o
	$(LD) -r -o $@ dio24H.o

dio24H.o: dio24H.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-dio24H.o test-dio24H dio24H.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {dio24H.c,dio24H_2_4.c,dio24H_2_6.c,dio24H_2_6_29.c,dio24H_3_3_7.c,dio24H_3_10_11.c,dio24H_4_0_8.c,test-dio24H.c,dio24H.h,pci-dio24H.h,Makefile,README,ModList,License,PCI-DIO24x.pdf,RegMapPCI-DIO24X.pdf}

all: default test-dio24H

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-dio24H:	test-dio24H.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-dio24H.*.cmd pci-dio24H.mod.c .dio24H.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-dio24H/$(DIST_FILES);

install: 
	-/sbin/rmmod dio24H
	-/bin/cp ./pci-dio24H.h /usr/local/include/pci-dio24H.h
	-/bin/chmod 644 /usr/local/include/pci-dio24H.h 
	-install -d $(MODULE_DIR)
	if [ -f ./dio24H.ko ]; then \
	/sbin/insmod  dio24H.ko; \
	install -c ./dio24H.ko $(MODULE_DIR); \
	else \
	/sbin/insmod  pci-dio24H.o; \
	install -c ./pci-dio24H.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod dio24H
	-/bin/rm -f /dev/dio24H*
	if [ -f $(MODULE_DIR)/dio24H.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/dio24H.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-dio24H.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-dio24H.o; \
	fi

devices:
	-/bin/rm -f /dev/dio24H*
	-/bin/mknod /dev/dio24H_0A  c $(MAJOR_DEV) 0
	-/bin/mknod /dev/dio24H_0B  c $(MAJOR_DEV) 1
	-/bin/mknod /dev/dio24H_0C  c $(MAJOR_DEV) 2

	if [ $(NUM_BOARDS) -ge 2 ];  then \
	/bin/mknod /dev/dio24H_1A c $(MAJOR_DEV) 16; \
	/bin/mknod /dev/dio24H_1B c $(MAJOR_DEV) 17; \
	/bin/mknod /dev/dio24H_1C c $(MAJOR_DEV) 18; \
	fi

	if [ $(NUM_BOARDS) -ge 3 ];  then \
	/bin/mknod /dev/dio24H_2A  c $(MAJOR_DEV) 32; \
	/bin/mknod /dev/dio24H_2B  c $(MAJOR_DEV) 33; \
	/bin/mknod /dev/dio24H_2C  c $(MAJOR_DEV) 34; \
	fi

	if [ $(NUM_BOARDS) -ge 4 ];  then \
	/bin/mknod /dev/dio24H_3A  c $(MAJOR_DEV) 48; \
	/bin/mknod /dev/dio24H_3B  c $(MAJOR_DEV) 49; \
	/bin/mknod /dev/dio24H_3C  c $(MAJOR_DEV) 50; \
	fi

	-/bin/chmod 666 /dev/dio24H*

endif
