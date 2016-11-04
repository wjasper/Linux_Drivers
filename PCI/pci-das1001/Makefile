##############################################################################
#	Makefile for building:
#
#		adc.o:    PCI-DAS1001 A/D adapater Linux loadable module.
#		test-das1602:   Program to test adc module
#
#               Copyright (C) 2003-2012
#               Written by:  Warren J. Jasper <wjasper@tx.ncsu.edu>
#                            North Carolina State Univerisity
#
#
#
#
# This program, PCI-DAS1001, is free software; you can redistribute it
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

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

#
#  Number of PCI-DAS1001 boards on your system:
#  MUST ALSO CHANGE MAX_BOARDS in a2dc.h
NUM_BOARDS=1

ID=PCI-DAS1001
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= das1001.o
das1001-objs := a2dc.o

# 2.4 kernel compatibility
modules: pci-das1001.o

pci-das1001.o: a2dc.o
	$(LD) -r -o $@ a2dc.o

a2dc.o: a2dc.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-das1001.o pci-das1001.ko test-das1001
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {a2dc.c,a2dc_2_4.c,a2dc_2_6.c,a2dc_2_6_29.c,a2dc_3_3_7.c,a2dc_3_10_11.c,a2dc_4_0_8.c,test-das1001.c,pci-das1001.h,a2dc.h,Makefile,README,ModList,License,RegMapSTC1000.pdf,cal100x.doc,AD7376_0.pdf,dac8800.pdf,PCI-DAS100x.pdf}

all: default test-das1001

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules


test-das1001:  test-das1001.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-das1001.*.cmd pci-das1001.mod.c .a2dc.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-das1001/$(DIST_FILES);

install: 
	-/sbin/rmmod das1001
	-/bin/cp ./pci-das1001.h /usr/local/include/pci-das1001.h
	-/bin/chmod 644 /usr/local/include/pci-das100*.h 
	if [ -f ./das1001.ko ]; then \
	/sbin/insmod  ./das1001.ko; \
	install -c ./das1001.ko $(MODULE_DIR) ; \
	else \
	/sbin/insmod ./pci-das1001.o ; \
	install -c ./pci-das1001.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod das1001
	-/bin/rm -rf /dev/das1001*
	if [ -f $(MODULE_DIR)/das1001.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/das1001.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-das1001.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-das1001.o; \
	fi

devices:
	-/bin/rm /dev/das1001_*
	-/bin/mknod /dev/das1001_adc00 c $(MAJOR_DEV) 0
	-/bin/mknod /dev/das1001_adc01 c $(MAJOR_DEV) 1
	-/bin/mknod /dev/das1001_adc02 c $(MAJOR_DEV) 2
	-/bin/mknod /dev/das1001_adc03 c $(MAJOR_DEV) 3
	-/bin/mknod /dev/das1001_adc04 c $(MAJOR_DEV) 4
	-/bin/mknod /dev/das1001_adc05 c $(MAJOR_DEV) 5
	-/bin/mknod /dev/das1001_adc06 c $(MAJOR_DEV) 6
	-/bin/mknod /dev/das1001_adc07 c $(MAJOR_DEV) 7
	-/bin/mknod /dev/das1001_adc08 c $(MAJOR_DEV) 8
	-/bin/mknod /dev/das1001_adc09 c $(MAJOR_DEV) 9
	-/bin/mknod /dev/das1001_adc0a c $(MAJOR_DEV) 10
	-/bin/mknod /dev/das1001_adc0b c $(MAJOR_DEV) 11
	-/bin/mknod /dev/das1001_adc0c c $(MAJOR_DEV) 12
	-/bin/mknod /dev/das1001_adc0d c $(MAJOR_DEV) 13
	-/bin/mknod /dev/das1001_adc0e c $(MAJOR_DEV) 14
	-/bin/mknod /dev/das1001_adc0f c $(MAJOR_DEV) 15
	-/bin/mknod /dev/das1001_dio0A c $(MAJOR_DEV) 16
	-/bin/mknod /dev/das1001_dio0B c $(MAJOR_DEV) 17
	-/bin/mknod /dev/das1001_dio0C c $(MAJOR_DEV) 18
	-/bin/mknod /dev/das1001_dac00 c $(MAJOR_DEV) 19
	-/bin/mknod /dev/das1001_dac01 c $(MAJOR_DEV) 20

	-if [ $(NUM_BOARDS) -ge 2 ];  then \
	-/bin/mknod /dev/das1001_adc10 c $(MAJOR_DEV) 32; \
	-/bin/mknod /dev/das1001_adc11 c $(MAJOR_DEV) 33; \
	-/bin/mknod /dev/das1001_adc12 c $(MAJOR_DEV) 34; \
	-/bin/mknod /dev/das1001_adc13 c $(MAJOR_DEV) 35; \
	-/bin/mknod /dev/das1001_adc14 c $(MAJOR_DEV) 36; \
	-/bin/mknod /dev/das1001_adc15 c $(MAJOR_DEV) 37; \
	-/bin/mknod /dev/das1001_adc16 c $(MAJOR_DEV) 38; \
	-/bin/mknod /dev/das1001_adc17 c $(MAJOR_DEV) 39; \
	-/bin/mknod /dev/das1001_adc18 c $(MAJOR_DEV) 40; \
	-/bin/mknod /dev/das1001_adc19 c $(MAJOR_DEV) 41; \
	-/bin/mknod /dev/das1001_adc1a c $(MAJOR_DEV) 42; \
	-/bin/mknod /dev/das1001_adc1b c $(MAJOR_DEV) 43; \
	-/bin/mknod /dev/das1001_adc1c c $(MAJOR_DEV) 44; \
	-/bin/mknod /dev/das1001_adc1d c $(MAJOR_DEV) 45; \
	-/bin/mknod /dev/das1001_adc1e c $(MAJOR_DEV) 46; \
	-/bin/mknod /dev/das1001_adc1f c $(MAJOR_DEV) 47; \
	-/bin/mknod /dev/das1001_dio1A c $(MAJOR_DEV) 48; \
	-/bin/mknod /dev/das1001_dio1B c $(MAJOR_DEV) 49; \
	-/bin/mknod /dev/das1001_dio1C c $(MAJOR_DEV) 50; \
	-/bin/mknod /dev/das1001_dac10 c $(MAJOR_DEV) 51; \
	-/bin/mknod /dev/das1001_dac11 c $(MAJOR_DEV) 52; \
	fi

	-/bin/chmod 666 /dev/das1001_adc0*
	-/bin/chmod 666 /dev/das1001_dio*
	-/bin/chmod 666 /dev/das1001_dac*

endif
