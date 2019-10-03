#############################################################################
#	
#	Makefile for building:
#
#		pci-dio48H.o:  PCI-DIO48H adapater Linux loadable module.
#		test-dio48H:   Program to test dio48H module
#
#               Copyright (C) September 7, 1999
#               Written by:  Warren J. Jasper
#                            North Carolina State Univerisity
#
#
# This program, PCI-DIO48H, is free software; you can redistribute it
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
VERSION=1.10

#  MUST ALSO CHANGE MAX_BOARDS in dio48H.h
#  Number of PCI-DIO48H boards on your system:
NUM_BOARDS=2

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

# Major Number of device
#MAJOR_DEV=249

ID=PCI-DIO48H
DIST_NAME=$(ID).$(VERSION).tgz
ifneq ($(KERNELRELEASE),)
obj-m	:= dio48H.o
#pci-dio48H-objs := dio48H.o

# 2.4 kernel compatibility
modules: pci-dio48H.o

pci-dio48H.o: dio48H.o
	$(LD) -r -o $@ dio48H.o

dio48H.o: dio48H.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-dio48H.o test-dio48H dio48.ko

MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {dio48H.c,dio48H_2_4.c,dio48H_2_6.c,dio48H_2_6_29.c,dio48H_3_3_7.c,dio48H_3_10_11.c,dio48H_4_0_8.c,test-dio48H.c,dio48H.h,pci-dio48H.h,Makefile,README,ModList,License,PCI-DIOxxH.pdf,RegMapPCI-DIOxxH.pdf}

all: default test-dio48H

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-dio48H:	test-dio48H.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-dio48H.*.cmd pci-dio48H.mod.c .dio48H.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-dio48H/$(DIST_FILES);

install: 
	-/sbin/rmmod dio48H
	-/bin/cp ./pci-dio48H.h /usr/local/include/pci-dio48H.h
	-/bin/chmod 644 /usr/local/include/pci-dio48H.h 
	-install -d $(MODULE_DIR)
	if [ -f ./dio48H.ko ]; then \
	/sbin/insmod  ./dio48H.ko; \
	install -c ./dio48H.ko $(MODULE_DIR); \
	else \
	/sbin/insmod  ./pci-dio48H.o; \
	install -c ./pci-dio48H.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod pci-dio48H
	-/bin/rm -f /dev/dio48H*
	if [ -f $(MODULE_DIR)/dio48H.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/dio48H.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-dio48H.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-dio48H.o; \
	fi

devices:
	-/bin/rm -f /dev/dio48H*
	-/bin/mknod /dev/dio48H_0A  c $(MAJOR_DEV) 0
	-/bin/mknod /dev/dio48H_0B  c $(MAJOR_DEV) 1
	-/bin/mknod /dev/dio48H_0C  c $(MAJOR_DEV) 2
	-/bin/mknod /dev/dio48H_1A  c $(MAJOR_DEV) 3
	-/bin/mknod /dev/dio48H_1B  c $(MAJOR_DEV) 4
	-/bin/mknod /dev/dio48H_1C  c $(MAJOR_DEV) 5

	-if [ $(NUM_BOARDS) -ge 2 ];  then \
	/bin/mknod /dev/dio48H_2A c $(MAJOR_DEV) 16; \
	/bin/mknod /dev/dio48H_2B c $(MAJOR_DEV) 17; \
	/bin/mknod /dev/dio48H_2C c $(MAJOR_DEV) 18; \
	/bin/mknod /dev/dio48H_3A c $(MAJOR_DEV) 19; \
	/bin/mknod /dev/dio48H_3B c $(MAJOR_DEV) 20; \
	/bin/mknod /dev/dio48H_3C c $(MAJOR_DEV) 21; \
	fi

	-if [ $(NUM_BOARDS) -ge 3 ];  then \
	/bin/mknod /dev/dio48H_4A  c $(MAJOR_DEV) 32; \
	/bin/mknod /dev/dio48H_4B  c $(MAJOR_DEV) 33; \
	/bin/mknod /dev/dio48H_4C  c $(MAJOR_DEV) 34; \
	/bin/mknod /dev/dio48H_5A  c $(MAJOR_DEV) 35; \
	/bin/mknod /dev/dio48H_5B  c $(MAJOR_DEV) 36; \
	/bin/mknod /dev/dio48H_5C  c $(MAJOR_DEV) 37; \
	fi

	-/bin/chmod 666 /dev/dio48H*

endif
