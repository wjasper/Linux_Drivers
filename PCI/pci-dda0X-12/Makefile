#############################################################################
#	
#	Makefile for building:
#
#		pci-dda0X-12.o:  PCI-DDA0X/12 adapater Linux loadable module.
#		test-dda0X-12:   Program to test PCI-DDA0X/12 module
#
#               Copyright (C) August 9, 2007
#               Written by:  Warren J. Jasper
#                            North Carolina State Univerisity
#
#
# This program, PCI-DDA0X/12, is free software; you can redistribute it
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
VERSION=0.7

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

#
#  Number of PCI-DDA0X/12 boards on your system: (1-4)
#  MUST ALSO CHANGE MAX_BOARDS in dda0X_12.h
NUM_BOARDS=1

ID=PCI-DDA0X-12
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= dda0X-12.o

# 2.4 kernel compatibility
modules: pci-dda0X-12.o

pci-dda0X-12.o: dda0X-12.o
	$(LD) -r -o $@ dda0X-12.o

dda0X-12.o: dda0X-12.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-dda0X-12.o test-dda0X-12 dda0X-12.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {dda0X-12.c,dda0X-12_2_6.c,dda0X-12_2_6_29.c,dda0X-12_3_3_7.c,dda0X-12_3_10_11.c,dda0X-12_4_0_8.c,test-dda0X-12.c,dda0X-12.h,pci-dda0X-12.h,Makefile,README,ModList,License,RegMapPCI-DDA0x-12.pdf}

all: default test-dda0X-12

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-dda0X-12:	test-dda0X-12.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-dda0X-12.*.cmd pci-dda0X-12.mod.c .dda0X-12.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) modules

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-dda0X-12/$(DIST_FILES);

install: 
	-/sbin/rmmod dda0X-12
	-/bin/cp ./pci-dda0X-12.h /usr/local/include/pci-dda0X-12.h
	-/bin/chmod 644 /usr/local/include/pci-dda0X-12.h 
	install -d $(MODULE_DIR)
	if [ -f ./dda0X-12.ko ]; then \
	/sbin/insmod  dda0X-12.ko; \
	install -c ./dda0X-12.ko $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod dda0X-12
	-/bin/rm -rf /dev/dda0X-12*
	if [ -f $(MODULE_DIR)/dda0X-12.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/dda0X-12.ko; \
	fi

endif
