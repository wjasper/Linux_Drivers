##############################################################################
#	Makefile for building:
#
#		adc.o:    PCIM-1602/16 A/D adapater Linux loadable module.
#		test-das1602:   Program to test adc module
#
#               Copyright (C) 2011
#               Written by:  Warren J. Jasper <wjasper@ncsu.edu>
#                            North Carolina State Univerisity
#
#
#
#
# This program, PCIM-DAS1602/16, is free software; you can redistribute it
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
VERSION=1.7

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

#
#  Number of PCIM-DAS1602/16 boards on your system: 
#  MUST ALSO CHANGE MAX_BOARDS in a2dc.h
NUM_BOARDS=1

ID=PCIM-DAS1602-16
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= pcim_das1602_16.o
pcim_das1602_16-objs := a2dc.o
modules: pcim_das1602_16.o

pcim-das1602-16.o: a2dc.o
	$(LD) -r -o $@ a2dc.o

a2dc.o: a2dc.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pcim-das1602_16.ko test-pcim-das1602-16
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {a2dc.c,a2dc_2_6_29.c,a2dc_3_3_7.c,a2dc_3_10_11.c,a2dc_4_0_8.c,test-pcim-das1602-16.c,pcim-das1602-16.h,a2dc.h,Makefile,README,ModList,License,RegMapPCIM-DAS1602-16.pdf}

all: default test-pcim-das1602-16

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-pcim-das1602-16:  test-pcim-das1602-16.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pcim-das1602-16.*.cmd pcim-das1602-16.mod.c .a2dc.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pcim-das1602-16/$(DIST_FILES);

install: 
	-/sbin/rmmod pcim_das1602_16
	-/bin/cp ./pcim-das1602-16.h /usr/local/include/pcim-das1602-16.h
	-/bin/chmod 644 /usr/local/include/pcim-das1602*.h 
	-/sbin/insmod  ./pcim_das1602_16.ko; \
	install -c ./pcim_das1602_16.ko $(MODULE_DIR) ; \

uninstall:
	-/sbin/rmmod pcim_das1602_16
	-/bin/rm -rf /dev/pcim-das1602*
	if [ -f $(MODULE_DIR)/pcim_das1602_16.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/pcim_das1602_16.ko; \
	fi

devices:
	-/usr/bin/mkdir -p /dev/pcim-das1602-16
	mv /dev/ad0* /dev/pcim-das1602-16
	mv /dev/da0* /dev/pcim-das1602-16
	mv /dev/dio0* /dev/pcim-das1602-16
endif
