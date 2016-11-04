##############################################################################
#	Makefile for building:
#
#		adc.o:    PCI-DAS1602-16 A/D adapater Linux loadable module.
#		test-das1602:   Program to test adc module
#
#               Copyright (C) 1997 - 2012
#               Written by:  Warren J. Jasper <wjasper@tx.ncsu.edu>
#                            North Carolina State Univerisity
#
# This program, PCI-DAS1602-16, is free software; you can redistribute it
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

# Current Version of the driver
VERSION=2.13

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

ID=PCI-DAS1602-16
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= das1602-16.o
das1602-16-objs := a2dc.o

# 2.4 kernel compatibility
modules: pci-das1602-16.o

pci-das1602-16.o: a2dc.o
	$(LD) -r -o $@ a2dc.o

a2dc.o: a2dc.c

devices:
	-/bin/mknod /dev/pci-das1602-16_adc00 c $(MAJOR_DEV) 0
	-/bin/mknod /dev/pci-das1602-16_adc01 c $(MAJOR_DEV) 1
	-/bin/mknod /dev/pci-das1602-16_adc02 c $(MAJOR_DEV) 2
	-/bin/mknod /dev/pci-das1602-16_adc03 c $(MAJOR_DEV) 3
	-/bin/mknod /dev/pci-das1602-16_adc04 c $(MAJOR_DEV) 4
	-/bin/mknod /dev/pci-das1602-16_adc05 c $(MAJOR_DEV) 5
	-/bin/mknod /dev/pci-das1602-16_adc06 c $(MAJOR_DEV) 6
	-/bin/mknod /dev/pci-das1602-16_adc07 c $(MAJOR_DEV) 7
	-/bin/mknod /dev/pci-das1602-16_adc08 c $(MAJOR_DEV) 8
	-/bin/mknod /dev/pci-das1602-16_adc09 c $(MAJOR_DEV) 9
	-/bin/mknod /dev/pci-das1602-16_adc0a c $(MAJOR_DEV) 10
	-/bin/mknod /dev/pci-das1602-16_adc0b c $(MAJOR_DEV) 11
	-/bin/mknod /dev/pci-das1602-16_adc0c c $(MAJOR_DEV) 12
	-/bin/mknod /dev/pci-das1602-16_adc0d c $(MAJOR_DEV) 13
	-/bin/mknod /dev/pci-das1602-16_adc0e c $(MAJOR_DEV) 14
	-/bin/mknod /dev/pci-das1602-16_adc0f c $(MAJOR_DEV) 15
	-/bin/mknod /dev/pci-das1602-16_dio_A c $(MAJOR_DEV) 16
	-/bin/mknod /dev/pci-das1602-16_dio_B c $(MAJOR_DEV) 17
	-/bin/mknod /dev/pci-das1602-16_dio_C c $(MAJOR_DEV) 18
	-/bin/mknod /dev/pci-das1602-16_dac0  c $(MAJOR_DEV) 19
	-/bin/mknod /dev/pci-das1602-16_dac1  c $(MAJOR_DEV) 20
	-/bin/chmod 666 /dev/pci-das1602-16_adc0*
	-/bin/chmod 666 /dev/pci-das1602-16_dio_*
	-/bin/chmod 666 /dev/pci-das1602-16_dac*

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-das1602-16.o pci-das1602-16.ko test-das1602
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {a2dc.c,a2dc_2_4.c,a2dc_2_6.c,a2dc_2_6_29.c,a2dc_3_3_7.c,a2dc_3_10_11.c,a2dc_4_0_8.c,test-das1602.c,pci-das1602-16.h,a2dc.h,Makefile,README,ModList,License,contrib,pciprod.pdf,pci-das1602_16.pdf}

all: default test-das1602

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-das1602:  test-das1602.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-das1602-16.*.cmd pci-das1602-16.mod.c .a2dc.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-das1602-16/$(DIST_FILES);

install: 
	-/sbin/rmmod das1602-16
	-/bin/cp ./pci-das1602-16.h /usr/local/include/pci-das1602-16.h
	-/bin/chmod 644 /usr/local/include/pci-das1602-16.h 
	if [ -f ./das1602-16.ko ]; then \
	/sbin/insmod  ./das1602-16.ko; \
	install -c ./das1602-16.ko $(MODULE_DIR) ; \
	else \
	/sbin/insmod ./pci-das1602-16.o ; \
	install -c ./pci-das1602-16.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod das1602-16
	-rm -f $(MODULE_DIR)/das1602-16.o
	-/bin/rm -f /dev/das1602-16*
	if [ -f $(MODULE_DIR)/das1602-16.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/das1602-16.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-das1602-16.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-das1602-16.o; \
	fi

endif
