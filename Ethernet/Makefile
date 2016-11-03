#############################################################################
#	                                                                    #
#	Makefile for building:                                              #
#                                                                           #
#		libmcceth.so:        Library for Ethernet series            #
#                                                                           #
#                                                                           #
#                                                                           #
#               Copyright (C)  2015                                         #
#               Written by:  Warren J. Jasper                               #
#                            North Carolina State Univerisity               #
#                                                                           #
#############################################################################
#  Current Version of the driver
VERSION=1.04
SRCS =	ethernet.c E-1608.c E-DIO24.c E-TC32.c E-TC.c

HEADERS = ethernet.h E-1608.h E-DIO24.h E-TC32.h E-TC.h

OBJS = $(SRCS:.c=.o)   # same list as SRCS with extension changed
CC=gcc
CFLAGS+= -g -Wall -fPIC -O 

ifeq ($(shell uname), Darwin)
	SONAME_FLAGS = -install_name
	SHARED_EXT = dylib
else
	SONAME_FLAGS = -soname
	SHARED_EXT = so
endif 
TARGETS=libmcceth.$(SHARED_EXT) libmcceth.a test-E-1608 test-E-DIO24 test-E-TC32 test-E-TC

ID=MCCLIBETH
DIST_NAME=$(ID).$(VERSION).tgz
DIST_FILES={README,Makefile,ethernet.h,ethernet.c,E-1608.h,E-1608.c,test-E-1608.c,E-DIO24.h,E-DIO24.c,test-E-DIO24.c,E-TC32.h,E-TC32.c,test-E-TC32.c,E-TC.h,E-TC.c,test-E-TC.c}
###### RULES
all: $(TARGETS)

%.d: %.c
	set -e; $(CC) -I. -M $(CPPFLAGS) $< \
	| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
	[ -s $@ ] || rm -f $@
ifneq ($(MAKECMDGOALS),clean)
include $(SRCS:.c=.d)
endif

libmcceth.$(SHARED_EXT): $(OBJS)
#	$(CC) -O -shared -Wall $(OBJS) -o $@
	$(CC) -shared -Wl,$(SONAME_FLAGS),$@ -o $@ $(OBJS) -lc -lm $(CFLAGS)

libmcceth.a: $(OBJS)
	ar -r libmcceth.a $(OBJS)
	ranlib libmcceth.a
#
#
#
test-E-1608: test-E-1608.c E-1608.o libmcceth.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmcceth -lm 

test-E-DIO24: test-E-DIO24.c E-DIO24.o libmcceth.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmcceth -lm 

test-E-TC32: test-E-TC32.c E-TC32.o libmcceth.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmcceth -lm 

test-E-TC: test-E-TC.c E-TC.o libmcceth.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmcceth -lm 

clean:
	rm -rf *.d *.o *~ *.a *.so *.dylib *.dll *.lib *.dSYM $(TARGETS)

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) mcc-ethernet/$(DIST_FILES);

install:
	-install -d /usr/local/lib
	-install -c --mode=0755 ./libmcceth.a libmcceth.$(SHARED_EXT) /usr/local/lib
	-/bin/ln -s /usr/local/lib/libmcceth.$(SHARED_EXT) /usr/lib/libmcceth.$(SHARED_EXT)
	-/bin/ln -s /usr/local/lib/libmcceth.a /usr/lib/libmcceth.a
	-install -d /usr/local/include/libeth
	-install -c --mode=0644 ${HEADERS} /usr/local/include/libeth/

uninstall:
	-rm -f /usr/local/lib/libmcceth*
	-rm -f /usr/lib/libmcceth*
	-rm -rf /usr/local/include/libeth

