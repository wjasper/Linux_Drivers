##############################################################################
#	Makefile for building:
#
#	pci-ctr20HD.o:  PCI-CTR20HD Counter/Timer module
#	test-ctr20HD:   Program to test adc module
#
#        Copyright (C) 2013
#        Written by:  Warren J. Jasper <wjasper@tx.ncsu.edu>
#                     North Carolina State Univerisity
#
#
#
#
# This program, PCI-CTR20HD, is free software; you can redistribute it
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
VERSION=1.5

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

#  Number of PCI-CTR20HD boards on your system:
#  MUST ALSO CHANGE MAX_BOARDS in c9513.h
NUM_BOARDS=1

# Major Number of device
#MAJOR_DEV=241

ID=PCI-CTR20HD
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= ctr20HD.o
ctr20HD-objs := c9513.o

# 2.4 kernel compatibility
modules: pci-ctr20HD.o

pci-ctr20HD.o: c9513.o
	$(LD) -r -o $@ c9513.o

c9513.o: c9513.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-ctr20HD.o test-ctr20HD pci-ctr20HD.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {c9513.c,c9513_2_6.c,c9513_2_6_29.c,c9513_3_3_7.c,c9513_3_10_11.c,c9513_4_0_8.c,test-ctr20HD.c,c9513.h,pci-ctr20HD.h,Makefile,README,ModList,License,PCI-CTR20HD.pdf,RegMapPCI-CTR20HD.pdf,9513A.pdf}

all: default test-ctr20HD

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-ctr20HD:	test-ctr20HD.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-ctr20HD.*.cmd pci-ctr20HD.mod.c .c9513.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-ctr20HD/$(DIST_FILES);

install: 
	-/sbin/rmmod ctr20HD
	-/bin/cp ./pci-ctr20HD.h /usr/local/include/pci-ctr20HD.h
	-/bin/chmod 644 /usr/local/include/pci-ctr20HD.h 
	-install -d $(MODULE_DIR)
	if [ -f ./ctr20HD.ko ]; then \
	/sbin/insmod  ctr20HD.ko; \
	install -c ./ctr20HD.ko $(MODULE_DIR); \
	else \
	/sbin/insmod  pci-ctr20HD.o; \
	install -c ./pci-ctr20HD.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod ctr20HD
	-/bin/rm -f /dev/ctr20HD*
	if [ -f $(MODULE_DIR)/ctr20HD.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/ctr20HD.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-ctr20HD.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-ctr20HD.o; \
	fi

endif
