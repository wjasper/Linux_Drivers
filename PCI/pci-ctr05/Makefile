##############################################################################
#	Makefile for building:
#
#	pci-ctr05.o: PCI-CTR05 Counter/Timer module
#	test-ctr05:     Program to test adc module
#
#        Copyright (C) 2003 - 2012
#        Written by:  Warren J. Jasper <wjasper@tx.ncsu.edu>
#                     North Carolina State Univerisity
#
#
#
#
# This program, PCI-CTR05, is free software; you can redistribute it
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

#  Number of PCI-CTR05 boards on your system:
#  MUST ALSO CHANGE MAX_BOARDS in c9513.h
NUM_BOARDS=1

# Major Number of device
#MAJOR_DEV=242

ID=PCI-CTR05
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= ctr05.o
ctr05-objs := c9513.o

# 2.4 kernel compatibility
modules: pci-ctr05.o

pci-ctr05.o: c9513.o
	$(LD) -r -o $@ c9513.o

c9513.o: c9513.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-ctr05.o test-ctr05 pci-ctr05.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {c9513.c,c9513_2_4.c,c9513_2_6.c,c9513_2_6_29.c,c9513_3_3_7.c,c9513_3_10_11.c,c9513_4_0_8.c,test-ctr05.c,c9513.h,pci-ctr05.h,Makefile,README,ModList,License,pci-ctr05.pdf,RegMapPCI-CTR05.pdf,9513A.pdf,9052db-102.pdf}

all: default test-ctr05

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-ctr05:	test-ctr05.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-ctr05.*.cmd pci-ctr05.mod.c .c9513.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-ctr05/$(DIST_FILES);

install: 
	-/sbin/rmmod ctr05
	-/bin/cp ./pci-ctr05.h /usr/local/include/pci-ctr05.h
	-/bin/chmod 644 /usr/local/include/pci-ctr05.h 
	-install -d $(MODULE_DIR)
	if [ -f ./ctr05.ko ]; then \
	/sbin/insmod  ctr05.ko; \
	install -c ./ctr05.ko $(MODULE_DIR); \
	else \
	/sbin/insmod  pci-ctr05.o; \
	install -c ./pci-ctr05.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod ctr05
	-/bin/rm -f /dev/ctr05*
	if [ -f $(MODULE_DIR)/ctr05.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/ctr05.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-ctr05.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-ctr05.o; \
	fi

devices:
	-/bin/rm /dev/pci-ctr05_*
	-/bin/mknod /dev/pci-ctr05_DIO0A c $(MAJOR_DEV) 0
	-/bin/mknod /dev/pci-ctr05_CTR01 c $(MAJOR_DEV) 1
	-/bin/mknod /dev/pci-ctr05_CTR02 c $(MAJOR_DEV) 2
	-/bin/mknod /dev/pci-ctr05_CTR03 c $(MAJOR_DEV) 3
	-/bin/mknod /dev/pci-ctr05_CTR04 c $(MAJOR_DEV) 4
	-/bin/mknod /dev/pci-ctr05_CTR05 c $(MAJOR_DEV) 5

	-if [ $(NUM_BOARDS) -ge 2 ];  then \
	-/bin/mknod /dev/pci-ctr05_DIO1A c $(MAJOR_DEV) 256; \
	-/bin/mknod /dev/pci-ctr05_CTR11 c $(MAJOR_DEV) 257; \
	-/bin/mknod /dev/pci-ctr05_CTR12 c $(MAJOR_DEV) 258; \
	-/bin/mknod /dev/pci-ctr05_CTR13 c $(MAJOR_DEV) 259; \
	-/bin/mknod /dev/pci-ctr05_CTR14 c $(MAJOR_DEV) 260; \
	-/bin/mknod /dev/pci-ctr05_CTR15 c $(MAJOR_DEV) 261; \
	fi

	-if [ $(NUM_BOARDS) -ge 3 ];  then \
	-/bin/mknod /dev/pci-ctr05_DIO2A c $(MAJOR_DEV) 512; \
	-/bin/mknod /dev/pci-ctr05_CTR21 c $(MAJOR_DEV) 513; \
	-/bin/mknod /dev/pci-ctr05_CTR22 c $(MAJOR_DEV) 514; \
	-/bin/mknod /dev/pci-ctr05_CTR23 c $(MAJOR_DEV) 515; \
	-/bin/mknod /dev/pci-ctr05_CTR24 c $(MAJOR_DEV) 516; \
	-/bin/mknod /dev/pci-ctr05_CTR25 c $(MAJOR_DEV) 517; \
	fi

	-/bin/chmod 666 /dev/pci-ctr05*

endif
