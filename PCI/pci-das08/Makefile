##############################################################################
#	
#	Makefile for building:
#
#		pci-das08.o: PCI-DAS08 A/D adapater Linux loadable module.
#		test-das08:  Program to test das08.o
#
#               Copyright (C) October 4, 1999
#               Written by:  Warren J. Jasper
#                            North Carolina State Univerisity
#
#
#
# This program, PCI-DAS08, is free software; you can redistribute it
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
VERSION=2.11

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

#  Number of PCI-DAS08 boards on your system:
#  MUST ALSO CHANGE MAX_BOARDS in a2dc.h
NUM_BOARDS=1

#MAJOR_DEV=247

ID=PCI-DAS08
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= das08.o
das08-objs := a2dc.o

# 2.4 kernel compatibility
modules: pci-das08.o

pci-das08.o: a2dc.o
	$(LD) -r -o $@ a2dc.o

a2dc.o: a2dc.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-das08.o pci-das08.ko test-das08
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {a2dc.c,a2dc_2_4.c,a2dc_2_6.c,a2dc_2_6_29.c,a2dc_3_3_7.c,a2dc_3_10_11.c,a2dc_4_0_8.c,test-das08.c,pci-das08.h,a2dc.h,Makefile,README,ModList,License,pci-das08.pdf,RegMapPCI-DAS08.pdf}

all: default test-das08

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-das08:	test-das08.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-das08.*.cmd pci-das08.mod.c .a2dc.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-das08/$(DIST_FILES);

install: 
	-/sbin/rmmod das08
	-/bin/cp ./pci-das08.h /usr/local/include/pci-das08.h
	-/bin/chmod 644 /usr/local/include/pci-das08.h 
	-install -d $(MODULE_DIR)
	if [ -f ./das08.ko ]; then \
	/sbin/insmod  ./das08.ko; \
	install -c ./das08.ko $(MODULE_DIR) ; \
	else \
	/sbin/insmod ./pci-das08.o ; \
	install -c ./pci-das08.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod das08
	-/bin/rm -f /dev/das08*
	if [ -f $(MODULE_DIR)/das08.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/das08.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-das08.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-das08.o; \
	fi

devices:
	-/bin/rm /dev/pci-das08_*
	-/bin/mknod /dev/pci-das08_adc00 c $(MAJOR_DEV) 0
	-/bin/mknod /dev/pci-das08_adc01 c $(MAJOR_DEV) 1
	-/bin/mknod /dev/pci-das08_adc02 c $(MAJOR_DEV) 2
	-/bin/mknod /dev/pci-das08_adc03 c $(MAJOR_DEV) 3
	-/bin/mknod /dev/pci-das08_adc04 c $(MAJOR_DEV) 4
	-/bin/mknod /dev/pci-das08_adc05 c $(MAJOR_DEV) 5
	-/bin/mknod /dev/pci-das08_adc06 c $(MAJOR_DEV) 6
	-/bin/mknod /dev/pci-das08_adc07 c $(MAJOR_DEV) 7
	-/bin/chmod 666 /dev/pci-das08*

	if [ $(NUM_BOARDS) -ge 2 ];  then \
	/bin/mknod /dev/pci-das08_adc10  c $(MAJOR_DEV) 16; \
	/bin/mknod /dev/pci-das08_adc11  c $(MAJOR_DEV) 17; \
	/bin/mknod /dev/pci-das08_adc12  c $(MAJOR_DEV) 18; \
	/bin/mknod /dev/pci-das08_adc13  c $(MAJOR_DEV) 19; \
	/bin/mknod /dev/pci-das08_adc14  c $(MAJOR_DEV) 20; \
	/bin/mknod /dev/pci-das08_adc15  c $(MAJOR_DEV) 21; \
	/bin/mknod /dev/pci-das08_adc16  c $(MAJOR_DEV) 22; \
	/bin/mknod /dev/pci-das08_adc17  c $(MAJOR_DEV) 23; \
	fi

	if [ $(NUM_BOARDS) -ge 3 ];  then \
	/bin/mknod /dev/pci-das08_adc20  c $(MAJOR_DEV) 32; \
	/bin/mknod /dev/pci-das08_adc21  c $(MAJOR_DEV) 33; \
	/bin/mknod /dev/pci-das08_adc22  c $(MAJOR_DEV) 34; \
	/bin/mknod /dev/pci-das08_adc23  c $(MAJOR_DEV) 35; \
	/bin/mknod /dev/pci-das08_adc24  c $(MAJOR_DEV) 36; \
	/bin/mknod /dev/pci-das08_adc25  c $(MAJOR_DEV) 37; \
	/bin/mknod /dev/pci-das08_adc26  c $(MAJOR_DEV) 38; \
	/bin/mknod /dev/pci-das08_adc27  c $(MAJOR_DEV) 39; \
	fi

	-/bin/chmod 666 /dev/pci-das08_adc*
endif
