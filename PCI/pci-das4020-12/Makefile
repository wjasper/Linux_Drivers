##############################################################################
#	Makefile for building:
#
#	pci-das4020-12.o: PCI-DAS4020-12 A/D adapater Linux loadable module.
#	test-das4020:     Program to test adc module
#
#        Copyright (C) 2004
#        Written by:  Warren J. Jasper <wjasper@tx.ncsu.edu>
#                     North Carolina State Univerisity
#
#
#
#
# This program, PCI-DAS4020-12, is free software; you can redistribute it
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
VERSION=1.26

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

#  Number of PCI-DAS4020/12 boards on your system:
#  MUST ALSO CHANGE MAX_BOARDS in a2dc.h
NUM_BOARDS=1

ID=PCI-DAS4020-12
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= das4020-12.o
das4020-12-objs := a2dc.o

# 2.4 kernel compatibility
modules: pci-das4020-12.o

das4020-12.o: a2dc.o
	$(LD) -r -o $@ a2dc.o

a2dc.o: a2dc.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-das4020-12.o pci-das4020-12.ko test-das4020
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
MAJOR_DEV=250
DIST_FILES = {a2dc.c,a2dc_2_4.c,a2dc_2_6_20.c,a2dc_2_6_26.c,a2dc_2_6_29.c,a2dc_3_3_7.c,a2dc_3_10_11.c,a2dc_4_0_8.c,test-das4020.c,pci-das4020.h,a2dc.h,plx9080.h,Makefile,README,ModList,License,plx9080.pdf,9080-3errata-14.pdf,9080-3dn-13.pdf,Calibration.ps}

all: default test-das4020

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-das4020: test-das4020.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean: 
	rm -f *.o *~ \#* .pci-das4020-12.*.cmd pci-das4020-12.mod.c .a2dc.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-das4020-12/$(DIST_FILES);

install:
	-/sbin/rmmod das4020-12
	-/bin/cp ./pci-das4020.h /usr/local/include/pci-das4020.h
	-/bin/chmod 644 /usr/local/include/pci-das4020.h
	-install -d $(MODULE_DIR)
	if [ -f ./das4020-12.ko ]; then \
	/sbin/insmod  ./das4020-12.ko; \
	install -c ./das4020-12.ko $(MODULE_DIR) ; \
	else \
	/sbin/insmod ./pci-das4020-12.o ; \
	install -c ./pci-das4020-12.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod das4020-12
	-/bin/rm -rf /dev/das4020-12*
	if [ -f $(MODULE_DIR)/das4020-12.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/das4020-12.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-das4020-12.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-das4020-12.o; \
	fi

endif

