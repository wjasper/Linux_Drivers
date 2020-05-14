#############################################################################
#	
#	Makefile for building:
#
#		pcim-dda06-16.o: PCIM-DDA06/16 adapater Linux loadable module.
#		test-dda06-16:   Program to test PCIM-DDA06/16 module
#
#               Copyright (C) March 13, 2020
#               Written by:   Warren J. Jasper
#                             North Carolina State University
#
#
# This program, PCIM-DDA06/16, is free software; you can redistribute it
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
VERSION=0.1

# To let the kernel know is this not a signed copy.
CONFIG_MODULE_SIG=n
CONFIG_MODULE_SIG_ALL=n

#  Number of PCIM-DDA06/16 boards on your system: (1-4)
#  MUST ALSO CHANGE MAX_BOARDS in dda06_16.h
NUM_BOARDS=1

ID=PCIM-DDA06-16
DIST_NAME=$(ID).$(VERSION).tgz

ifneq ($(KERNELRELEASE),)
obj-m	:= pcim-dda06-16.o
pcim-dda06-16-objs := dda06-16.o

# 2.4 kernel compatibility
modules: pcim-dda06-16.o

pcim-dda06-16.o: dda06-16.o
	$(LD) -r -o $@ dda06-16.o

dda06-16.o: dda06-16.c

else

KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
TARGETS=pcim-dda06-16.o test-dda06-16 dda06-16.ko
MODULE_DIR=/lib/modules/`uname -r`/kernel/drivers/char
DIST_FILES = {dda06-16.c,dda06-16_4_0_8.c,test-dda06-16.c,dda06-16.h,pcim-dda06-16.h,Makefile,README,ModList,License,RegMapPCIM-DDA06-16.pdf}

all: default test-dda06-16

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test-dda06-16:	test-dda06-16.c
	$(CC) -Wall -g -o $@ $@.c -lm

clean:
	rm -f *.o *~ \#* .pcim-dda06-16.*.cmd pcim-dda06-16.mod.c .dda06-16.*.cmd $(TARGETS)
	rm -rf .tmp_versions
dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) pcim-dda06-16/$(DIST_FILES);

install: 
	-/sbin/rmmod pcim-dda06-16
	-/bin/cp ./pcim-dda06-16.h /usr/local/include/pcim-dda06-16.h
	-/bin/chmod 644 /usr/local/include/pcim-dda06-16.h
	-/sbin/insmod  ./pcim-dda06-16.ko; \
	install -c ./pcim-dda06-16.ko $(MODULE_DIR) ; \

uninstall:
	-/sbin/rmmod pcim-dda06-16
	-/bin/rm -rf /dev/pcim-dda06*
	if [ -f $(MODULE_DIR)/pcim-das06-16.ko ]; then \
	/bin/rm -f $(MODULE_DIR)/pcim-dda06-16.ko; \
	fi

endif
