##############################################################################
#	Makefile for building:
#
#	pci-quad04.o: PCI-QUAD04 Four-Channel Quadrature Encoder
#	test-quad04:  Program to test quadrature encoder
#
#        Copyright (C) 2012
#        Written by:  Warren J. Jasper <wjasper@tx.ncsu.edu>
#                     North Carolina State Univerisity
#
#
# This program, PCI-QUAD04, is free software; you can redistribute it
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
VERSION=1.4

#  Number of PCI-QUAD04 boards on your system: 
#  MUST ALSO CHANGE MAX_BOARDS in quad04.h
NUM_BOARDS=1

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

# Major Number of device
#MAJOR_DEV=242

ID=PCI-QUAD04
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= quad04.o
quad04-objs := LS7266R1.o

# 2.4 kernel compatibility
modules: pci-quad04.o

pci-quad04.o: LS7266R1.o
	$(LD) -r -o $@ LS7266R1.o

LS7266R1.o: LS7266R1.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pci-quad04.o test-quad04 pci-quad04.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {LS7266R1.c,LS7266R1_2_6_30.c,LS7266R1_3_3_7.c,LS7266R1_3_10_11.c,LS7266R1_4_0_8.c,test-quad04.c,quad04.h,pci-quad04.h,Makefile,README,ModList,License,LS7266R1.pdf,pci-quad04.pdf,RegMapPCI-QUAD04.pdf,8259.pdf}

all: default test-quad04

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-quad04:	test-quad04.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pci-quad04.*.cmd pci-quad04.mod.c  $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pci-quad04/$(DIST_FILES);

install: 
	-/sbin/rmmod quad04
	-/bin/cp ./pci-quad04.h /usr/local/include/pci-quad04.h
	-/bin/chmod 644 /usr/local/include/pci-quad04.h 
	-install -d $(MODULE_DIR)
	if [ -f ./quad04.ko ]; then \
	/sbin/insmod  quad04.ko; \
	install -c ./quad04.ko $(MODULE_DIR); \
	else \
	/sbin/insmod  pci-quad04.o; \
	install -c ./pci-quad04.o $(MODULE_DIR); \
	fi

uninstall:
	-/sbin/rmmod quad04
	-/bin/rm -f /dev/quad04*
	if [ -f $(MODULE_DIR)/quad04.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/quad04.ko; \
	fi
	if [ -f $(MODULE_DIR)/pci-quad04.o ]; then \
	/bin/rm -f $(MODULE_DIR)/pci-quad04.o; \
	fi
endif
