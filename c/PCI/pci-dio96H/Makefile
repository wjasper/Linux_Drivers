#############################################################################
#	
#	Makefile for building:
#
#		pci-dio96H.o:  PCI-DIO96H adapater Linux loadable module.
#		test-dio96H:   Program to test dio96H module
#
#               Copyright (C) May 21, 2005
#               Written by:  Warren J. Jasper
#                            North Carolina State Univerisity
#
#
# This program, PCI-DIO96H, is free software; you can redistribute it
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
VERSION=1.9

#  Number of PCI-DIO96H boards on your system: (1-4)
#  MUST ALSO CHANGE MAX_BOARDS in dio.h
NUM_BOARDS=1

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

# Major Number of device
#MAJOR_DEV=254

ID=PCI-DIO96H
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= dio96H.o
#pci-dio96H-objs := dio96H.o

# 2.4 kernel compatibility
modules: pci-dio96H.o

pci-dio96H.o: dio96H.o
	$(LD) -r -o $@ dio96H.o

dio96H.o: dio96H.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-dio96H.o test-dio96H dio96H.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {dio96H.c,dio96H_2_4.c,dio96H_2_6_20.c,dio96H_2_6_26.c,dio96H_2_6_29.c,dio96H_3_3_7.c,dio96H_3_10_11.c,dio96H_4_0_8.c,test-dio96H.c,dio96H.h,pci-dio96H.h,Makefile,README,ModList,License,pci-dio96h.pdf,RegMapPCI-DIO96H.pdf,PCIe-DIO96H.pdf}

all: default test-dio96H

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-dio96H:	test-dio96H.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-dio96H.*.cmd pci-dio96H.mod.c .dio96H.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-dio96H/$(DIST_FILES);

install: 
	-/sbin/rmmod dio96H
	-/bin/cp ./pci-dio96H.h /usr/local/include/pci-dio96H.h
	-/bin/chmod 644 /usr/local/include/pci-dio96H.h 
	install -d $(MODULE_DIR)
	if [ -f ./dio96H.ko ]; then \
	/sbin/insmod  ./dio96H.ko; \
	install -c ./dio96H.ko $(MODULE_DIR); \
	else \
	/sbin/insmod ./pci-dio96H.o; \
	install -c ./pci-dio96H.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod dio96H
	-/bin/rm -f /dev/dio96H*
	if [ -f $(MODULE_DIR)/dio96H.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/dio96H.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-dio96H.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-dio96H.o; \
	fi

endif
