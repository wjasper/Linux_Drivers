#############################################################################
#	
#	Makefile for building:
#
#		pci-dio96.o:  PCI-DIO96 adapater Linux loadable module.
#		test-dio96:   Program to test dio96 module
#
#               Copyright (C) May 20, 2005
#               Written by:  Warren J. Jasper
#                            North Carolina State Univerisity
#
#
# This program, PCI-DIO96, is free software; you can redistribute it
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
VERSION=1.7

#  Number of PCI-DIO96 boards on your system: (1-4)
#  MUST ALSO CHANGE MAX_BOARDS in dio.h
NUM_BOARDS=1

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

# Major Number of device
#MAJOR_DEV=249

ID=PCI-DIO96
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= dio96.o
#pci-dio96-objs := dio96.o

# 2.4 kernel compatibility
modules: pci-dio96.o

pci-dio96.o: dio96.o
	$(LD) -r -o $@ dio96.o

dio96.o: dio96.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-dio96.o test-dio96 dio96.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {dio96.c,dio96_2_4.c,dio96_2_6.c,dio96_2_6_29.c,dio96_3_3_7.c,dio96_3_10_11.c,dio96_4_0_8.c,test-dio96.c,dio96.h,pci-dio96.h,Makefile,README,ModList,License,pci-dio96.pdf,RegMapPCI-DIO96.pdf,CTS8254.pdf}

all: default test-dio96

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-dio96:	test-dio96.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-dio96.*.cmd pci-dio96.mod.c .dio96.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-dio96/$(DIST_FILES);

install: 
	-/sbin/rmmod dio96
	-/bin/cp ./pci-dio96.h /usr/local/include/pci-dio96.h
	-/bin/chmod 644 /usr/local/include/pci-dio96.h 
	install -d $(MODULE_DIR)
	if [ -f ./dio96.ko ]; then \
	/sbin/insmod  ./dio96.ko; \
	install -c ./dio96.ko $(MODULE_DIR); \
	else \
	/sbin/insmod ./pci-dio96.o; \
	install -c ./pci-dio96.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod dio96
	-/bin/rm -f /dev/dio96*
	if [ -f $(MODULE_DIR)/dio96.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/dio96.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-dio96.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-dio96.o; \
	fi


endif
