##############################################################################
#	Makefile for building:
#
#	pci-ctr10.o: PCI-CTR10 Counter/Timer module
#	test-ctr10:     Program to test adc module
#
#        Copyright (C) 2007
#        Written by:  Warren J. Jasper <wjasper@tx.ncsu.edu>
#                     North Carolina State Univerisity
#
#
#
#
# This program, PCI-CTR10, is free software; you can redistribute it
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

#  Number of PCI-CTR10 boards on your system:
#  MUST ALSO CHANGE MAX_BOARDS in c9513.h
NUM_BOARDS=1

# Major Number of device
#MAJOR_DEV=241

ID=PCI-CTR10
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= ctr10.o
ctr10-objs := c9513.o

# 2.4 kernel compatibility
modules: pci-ctr10.o

pci-ctr10.o: c9513.o
	$(LD) -r -o $@ c9513.o

c9513.o: c9513.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-ctr10.o test-ctr10 pci-ctr10.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {c9513.c,c9513_2_4.c,c9513_2_6.c,c9513_2_6_29.c,c9513_3_3_7.c,c9513_3_10_11.c,c9513_4_0_8.c,test-ctr10.c,c9513.h,pci-ctr10.h,Makefile,README,ModList,License,pci-ctr10.pdf,RegMapPCI-CTR10.pdf,9513A.pdf,9052db-102.pdf}

all: default test-ctr10

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-ctr10:	test-ctr10.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-ctr10.*.cmd pci-ctr10.mod.c .c9513.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-ctr10/$(DIST_FILES);

install: 
	-/sbin/rmmod ctr10
	-/bin/cp ./pci-ctr10.h /usr/local/include/pci-ctr10.h
	-/bin/chmod 644 /usr/local/include/pci-ctr10.h 
	-install -d $(MODULE_DIR)
	if [ -f ./ctr10.ko ]; then \
	/sbin/insmod  ctr10.ko; \
	install -c ./ctr10.ko $(MODULE_DIR); \
	else \
	/sbin/insmod  pci-ctr10.o; \
	install -c ./pci-ctr10.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod ctr10
	-/bin/rm -f /dev/ctr10*
	if [ -f $(MODULE_DIR)/ctr10.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/ctr10.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-ctr10.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-ctr10.o; \
	fi

devices:
	-/bin/rm /dev/pci-ctr10_*
	-/bin/mknod /dev/pci-ctr10_DIO0A c $(MAJOR_DEV) 0
	-/bin/mknod /dev/pci-ctr10_CTR01 c $(MAJOR_DEV) 1
	-/bin/mknod /dev/pci-ctr10_CTR02 c $(MAJOR_DEV) 2
	-/bin/mknod /dev/pci-ctr10_CTR03 c $(MAJOR_DEV) 3
	-/bin/mknod /dev/pci-ctr10_CTR04 c $(MAJOR_DEV) 4
	-/bin/mknod /dev/pci-ctr10_CTR05 c $(MAJOR_DEV) 5
	-/bin/mknod /dev/pci-ctr10_DIO0B c $(MAJOR_DEV) 8
	-/bin/mknod /dev/pci-ctr10_CTR06 c $(MAJOR_DEV) 9
	-/bin/mknod /dev/pci-ctr10_CTR07 c $(MAJOR_DEV) 10
	-/bin/mknod /dev/pci-ctr10_CTR08 c $(MAJOR_DEV) 11
	-/bin/mknod /dev/pci-ctr10_CTR09 c $(MAJOR_DEV) 12
	-/bin/mknod /dev/pci-ctr10_CTR0a c $(MAJOR_DEV) 13

	-if [ $(NUM_BOARDS) -ge 2 ];  then \
	-/bin/mknod /dev/pci-ctr10_DIO1A c $(MAJOR_DEV) 256; \
	-/bin/mknod /dev/pci-ctr10_CTR11 c $(MAJOR_DEV) 257; \
	-/bin/mknod /dev/pci-ctr10_CTR12 c $(MAJOR_DEV) 258; \
	-/bin/mknod /dev/pci-ctr10_CTR13 c $(MAJOR_DEV) 259; \
	-/bin/mknod /dev/pci-ctr10_CTR14 c $(MAJOR_DEV) 260; \
	-/bin/mknod /dev/pci-ctr10_CTR15 c $(MAJOR_DEV) 261; \
	-/bin/mknod /dev/pci-ctr10_DIO1B c $(MAJOR_DEV) 264; \
	-/bin/mknod /dev/pci-ctr10_CTR16 c $(MAJOR_DEV) 265; \
	-/bin/mknod /dev/pci-ctr10_CTR17 c $(MAJOR_DEV) 266; \
	-/bin/mknod /dev/pci-ctr10_CTR18 c $(MAJOR_DEV) 267; \
	-/bin/mknod /dev/pci-ctr10_CTR19 c $(MAJOR_DEV) 268; \
	-/bin/mknod /dev/pci-ctr10_CTR1a c $(MAJOR_DEV) 269; \
	fi

	-if [ $(NUM_BOARDS) -ge 3 ];  then \
	-/bin/mknod /dev/pci-ctr10_DIO2A c $(MAJOR_DEV) 512; \
	-/bin/mknod /dev/pci-ctr10_CTR21 c $(MAJOR_DEV) 513; \
	-/bin/mknod /dev/pci-ctr10_CTR22 c $(MAJOR_DEV) 514; \
	-/bin/mknod /dev/pci-ctr10_CTR23 c $(MAJOR_DEV) 515; \
	-/bin/mknod /dev/pci-ctr10_CTR24 c $(MAJOR_DEV) 516; \
	-/bin/mknod /dev/pci-ctr10_CTR25 c $(MAJOR_DEV) 517; \
	-/bin/mknod /dev/pci-ctr10_DIO2B c $(MAJOR_DEV) 520; \
	-/bin/mknod /dev/pci-ctr10_CTR26 c $(MAJOR_DEV) 521; \
	-/bin/mknod /dev/pci-ctr10_CTR27 c $(MAJOR_DEV) 522; \
	-/bin/mknod /dev/pci-ctr10_CTR28 c $(MAJOR_DEV) 523; \
	-/bin/mknod /dev/pci-ctr10_CTR29 c $(MAJOR_DEV) 524; \
	-/bin/mknod /dev/pci-ctr10_CTR2a c $(MAJOR_DEV) 525; \
	fi

	-/bin/chmod 666 /dev/pci-ctr10*

endif
