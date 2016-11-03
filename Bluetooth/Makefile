#############################################################################
#	                                                                    #
#	Makefile for building:                                              #
#                                                                           #
#		libmccbth.so:        Library for Bluetooth series           #
#                                                                           #
#                                                                           #
#                                                                           #
#               Copyright (C)  2016                                         #
#               Written by:  Warren J. Jasper                               #
#                            North Carolina State Univerisity               #
#                                                                           #
#############################################################################
#  Current Version of the driver

VERSION=1.0

SRCS =	bluetooth.c bth-1208LS.c

HEADERS = bluetooth.h bth-1208LS.h

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

TARGETS=libmccbth.$(SHARED_EXT) libmccbth.a test-bth1208LS

ID=MCCLIBBTH
DIST_NAME=$(ID).$(VERSION).tgz
DIST_FILES={README,Makefile,bluetooth.h,bluetooth.c,bth-1208LS.h,bth-1208LS.c,test-bth1208LS.c}
###### RULES
all: $(TARGETS)

%.d: %.c
	set -e; $(CC) -I. -M $(CPPFLAGS) $< \
	| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
	[ -s $@ ] || rm -f $@
ifneq ($(MAKECMDGOALS),clean)
include $(SRCS:.c=.d)
endif

libmccbth.$(SHARED_EXT): $(OBJS)
#	$(CC) -O -shared -Wall $(OBJS) -o $@
	$(CC) -shared -Wl,$(SONAME_FLAGS),$@ -o $@ $(OBJS) -lc -lm $(CFLAGS)

libmccbth.a: $(OBJS)
	ar -r libmccbth.a $(OBJS)
	ranlib libmccbth.a
#
#
#
test-bth1208LS: test-bth1208LS.c bth-1208LS.o libmccbth.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccbth -lm -lbluetooth

clean:
	rm -rf *.d *.o *~ *.a *.so *.dylib *.dll *.lib *.dSYM $(TARGETS)

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) mcc-bluetooth/$(DIST_FILES);
install:
	-install -d /usr/local/lib
	-install -c --mode=0755 ./libmccbth.a libmccbth.$(SHARED_EXT) /usr/local/lib
	-/bin/ln -s /usr/local/lib/libmccbth.$(SHARED_EXT) /usr/lib/libmccbth.$(SHARED_EXT)
	-/bin/ln -s /usr/local/lib/libmccbth.a /usr/lib/libmccbth.a
	-install -d /usr/local/include/libbth
	-install -c --mode=0644 ${HEADERS} /usr/local/include/libbth/

uninstall:
	-rm -f /usr/local/lib/libmccbth*
	-rm -f /usr/lib/libmccbth*
	-rm -rf /usr/local/include/libbth
