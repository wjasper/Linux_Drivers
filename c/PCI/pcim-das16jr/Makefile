##############################################################################
#	Makefile for building:
#
#		adc.o:    PCIM-16JR/16 A/D adapater Linux loadable module.
#		test-das16jr:   Program to test adc module
#
#               Copyright (C) 2011
#               Written by:  Warren J. Jasper <wjasper@ncsu.edu>
#                            North Carolina State Univerisity
#
#
#
#
# This program, PCIM-DAS16JR/16, is free software; you can redistribute it
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

#
#  Current Version of the driver
#
VERSION=1.3

#  Number of PCIM-DAS16JR/16 boards on your system: 
#  MUST ALSO CHANGE MAX_BOARDS in a2dc.h
NUM_BOARDS=1

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

ID=PCIM-DAS16JR
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= pcim_das16jr.o
pcim_das16jr-objs := a2dc.o
modules: pcim_das16jr.o

pcim-das16jr.o: a2dc.o
	$(LD) -r -o $@ a2dc.o

a2dc.o: a2dc.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pcim-das16jr.ko test-pcim-das16jr
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {a2dc.c,a2dc_2_6_29.c,a2dc_3_3_7.c,a2dc_3_10_11.c,a2dc_4_0_8.c,test-pcim-das16jr.c,pcim-das16jr.h,a2dc.h,Makefile,README,ModList,License,RegmapPCIM-DAS16JR-16.pdf}

all: default test-pcim-das16jr

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-pcim-das16jr:  test-pcim-das16jr.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pcim-das16jr.*.cmd pcim-das16jr.mod.c .a2dc.*.cmd $(TARGETS)
	rm -rf .tmp_versions
	$(MAKE) -C $(KDIR) M=$(PWD) clean

dist:
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pcim-das16jr/$(DIST_FILES);

install: 
	-/sbin/rmmod pcim_das16jr
	-/bin/cp ./pcim-das16jr.h /usr/local/include/pcim-das16jr.h
	-/bin/chmod 644 /usr/local/include/pcim-das16jr*.h 
	-/sbin/insmod  ./pcim_das16jr.ko; \
	install -c ./pcim_das16jr.ko $(MODULE_DIR) ; \

uninstall:
	-/sbin/rmmod pcim_das16jr
	-/bin/rm -rf /dev/pcim-das16jr*
	if [ -f $(MODULE_DIR)/pcim_das16jr.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/pcim_das16jr.ko; \
	fi
endif
