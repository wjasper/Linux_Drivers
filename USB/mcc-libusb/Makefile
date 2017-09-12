#############################################################################
#	                                                                    #
#	Makefile for building:                                              #
#                                                                           #
#		libmccusb.so:        Library for USB series                 #
#                                                                           #
#                                                                           #
#                                                                           #
#               Copyright (C)  2014-2016                                    #
#               Written by:  Warren J. Jasper                               #
#                            wjasper@ncsu.edu                               #
#                            North Carolina State Univerisity               #
#                                                                           #
#############################################################################

#  Current Version of the driver
VERSION=1.18

SRCS =    pmd.c  nist.c   usb-1608G.c usb-20X.c usb-1208FS-Plus.c usb-1608FS-Plus.c usb-2020.c  \
          usb-ctr.c usb-2600.c usb-2408.c usb-2416.c usb-1608HS.c usb-1208HS.c usb-2001-tc.c    \
          usb-1024LS.c usb-1208LS.c usb-1608FS.c usb-7202.c usb-tc.c usb-dio24.c usb-dio96H.c   \
          usb-5200.c usb-temp.c usb-7204.c usb-1208FS.c usb-ssr.c usb-erb.c usb-pdiso8.c        \
          usb-1408FS.c usb-1616FS.c usb-3100.c usb-4303.c usb-tc-ai.c usb-dio32HS.c usb-tc-32.c \
          bth-1208LS.c minilab-1008.c usb-1808.c
HEADERS = pmd.h usb-500.h usb-1608G.h usb-20X.h usb-1208FS-Plus.h usb-1608FS-Plus.h usb-2020.h  \
          usb-ctr.h usb-2600.h usb-2408.h usb-2416.h usb-1608HS.h usb-1208HS.h usb-2001-tc.h    \
          usb-1024LS.h usb-1208LS.h usb-1608FS.h usb-7202.h usb-tc.h usb-dio24.h usb-dio96H.h   \
          usb-5200.h usb-temp.h usb-7204.h usb-1208FS.h usb-ssr.h usb-erb.h usb-pdiso8.c        \
          usb-1408FS.h usb-1616FS.h usb-3100.h usb-4303.h usb-tc-ai.h usb-dio32HS.h usb-tc-32.h \
          bth-1208LS.h minilab-1008.h usb-1808.h
OBJS = $(SRCS:.c=.o)   # same list as SRCS with extension changed
CC=gcc
CFLAGS+= -g -Wall -fPIC -O -I/usr/local/include/libusb-1.0 -L/usr/local/lib -lusb-1.0
ifeq ($(shell uname), Darwin)
	SONAME_FLAGS = -install_name
	SHARED_EXT = dylib
else
	SONAME_FLAGS = -soname
	SHARED_EXT = so
endif 
TARGETS=libmccusb.$(SHARED_EXT) libmccusb.a test-usb1608G test-usb20X test-usb500 test-usb1208FS-Plus test-usb1608FS-Plus \
        test-usb2020 test-usb-ctr test-usb2600 test-usb2408 test-usb2416 test-usb1608HS test-usb1208HS         \
        test-usb2001tc test-usb1024LS test-usb1208LS test-usb1208FS test-usb1408FS test-usb1608FS test-usb7202 \
        test-usb7204 test-usb-tc test-usb-dio24 test-usb-dio96H test-usb5201 test-usb5203 test-usb-temp        \
        test-usb-ssr test-usb-erb test-usb-pdiso8 test-usb1616FS test-usb3100 test-usb4300 test-usb-tc-ai      \
        test-usb-temp-ai test-usb-dio32HS test-usb-tc32 test-bth1208LS test-minilab1008 test-usb1808
ID=MCCLIBUSB
DIST_NAME=$(ID).$(VERSION).tgz
DIST_FILES={README,Makefile,nist.c,pmd.c,pmd.h,usb-1608G.h,usb-1608G.rbf,usb-1608G-2.rbf,usb-1608G.c,test-usb1608G.c,usb-20X.h,usb-20X.c,test-usb20X.c,usb-500.h,test-usb500.c,usb-1608FS-Plus.h,usb-1608FS-Plus.c,test-usb1608FS-Plus.c,usb-2020.h,usb-2020.rbf,usb-2020.c,test-usb2020.c,usb-1208FS-Plus.h,usb-1208FS-Plus.c,test-usb1208FS-Plus.c,usb-ctr.h,usb-ctr.rbf,usb-ctr.c,test-usb-ctr.c,usb-2600.h,usb-26xx.rbf,usb-2600.c,test-usb2600.c,usb-2416.h,usb-2416.c,test-usb2416.c,usb-1608HS.h,usb-1608HS.c,test-usb1608HS.c,usb-1208HS.rbf,usb-1208HS.h,usb-1208HS.c,test-usb1208HS.c,usb-2001-tc.h,usb-2001-tc.c,test-usb2001tc.c,usb-2408.h,usb-2408.c,test-usb2408.c,usb-2001-tc.h,usb-2001-tc.c,test-usb2001tc.c,usb-1024LS.h,usb-1024LS.c,test-usb1024LS.c,usb-1208LS.h,usb-1208LS.c,test-usb1208LS.c,usb-1608FS.h,usb-1608FS.c,test-usb1608FS.c,usb-7202.h,usb-7202.c,test-usb7202.c,usb-tc.h,usb-tc.c,test-usb-tc.c,usb-dio24.h,usb-dio24.c,test-usb-dio24.c,usb-dio96H.h,usb-dio96H.c,test-usb-dio96H.c,usb-5200.h,usb-5200.c,test-usb5201.c,test-usb5203.c,usb-temp.h,usb-temp.c,test-usb-temp.c,usb-7204.h,usb-7204.c,test-usb7204.c,usb-1208FS.h,usb-1208FS.c,test-usb1208FS.c,usb-ssr.h,usb-ssr.c,test-usb-ssr.c,usb-erb.h,usb-erb.c,test-usb-erb.c,usb-pdiso8.h,usb-pdiso8.c,test-usb-pdiso8.c,usb-1408FS.h,usb-1408FS.c,test-usb1408FS.c,usb-1616FS.h,usb-1616FS.c,test-usb1616FS.c,usb-3100.h,usb-3100.c,test-usb3100.c,usb-4303.h,usb-4303.c,test-usb4300.c,usb-tc-ai.h,usb-tc-ai.c,test-usb-tc-ai.c,test-usb-temp-ai.c,usb-dio32HS.h,usb-dio32HS.c,usb-dio32HS.rbf,test-usb-dio32HS.c,usb-tc-32.h,usb-tc-32.c,test-usb-tc32.c,bth-1208LS.h,bth-1208LS.c,test-bth1208LS.c,minilab-1008.h,minilab-1008.c,test-minilab1008.c,usb-1808.h,usb-1808.c,test-usb1808.c}

###### RULES
all: $(TARGETS)

%.d: %.c
	set -e; $(CC) -I. -M $(CPPFLAGS) $< \
	| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
	[ -s $@ ] || rm -f $@
ifneq ($(MAKECMDGOALS),clean)
include $(SRCS:.c=.d)
endif

libmccusb.$(SHARED_EXT): $(OBJS)
#	$(CC) -O -shared -Wall $(OBJS) -o $@
	$(CC) -shared -Wl,$(SONAME_FLAGS),$@ -o $@ $(OBJS) -lc -lm $(CFLAGS)

libmccusb.a: $(OBJS)
	ar -r libmccusb.a $(OBJS)
	ranlib libmccusb.a

#
# libusb-1.0
#
test-usb500:  test-usb500.c
	$(CC) -g -Wall -I. -o $@ $@.c -lm -lusb-1.0

test-usb1608G:	test-usb1608G.c usb-1608G.o libmccusb.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb1808:	test-usb1808.c usb-1808.o libmccusb.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb2020:	test-usb2020.c usb-2020.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb1208FS-Plus:	test-usb1208FS-Plus.c usb-1208FS-Plus.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb1608FS-Plus:	test-usb1608FS-Plus.c usb-1608FS-Plus.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0

test-usb1208HS:	test-usb1208HS.c usb-1208HS.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb1608HS:	test-usb1608HS.c usb-1608HS.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb20X:	test-usb20X.c usb-20X.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb2600:	test-usb2600.c usb-2600.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb2408:	test-usb2408.c usb-2408.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb2416:	test-usb2416.c usb-2416.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb-ctr:	test-usb-ctr.c usb-ctr.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb2001tc: test-usb2001tc.c usb-2001-tc.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb7202: test-usb7202.c usb-7202.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb7204: test-usb7204.c usb-7204.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0

test-usb-dio32HS: test-usb-dio32HS.c usb-dio32HS.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0

test-usb-tc32: test-usb-tc32.c usb-tc-32.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0

test-bth1208LS: test-bth1208LS.c bth-1208LS.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0


################ HID devices ##########################
test-minilab1008: test-minilab1008.c minilab-1008.o libmccusb.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0

test-usb1024LS:	test-usb1024LS.c usb-1024LS.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb-dio24:	test-usb-dio24.c usb-dio24.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb-dio96H: test-usb-dio96H.c usb-dio96H.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb1208LS:	test-usb1208LS.c usb-1208LS.o libmccusb.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb1208FS:	test-usb1208FS.c usb-1208FS.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb1408FS:	test-usb1408FS.c usb-1408FS.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb1608FS:	test-usb1608FS.c usb-1608FS.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb1616FS:	test-usb1616FS.c usb-1616FS.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0

test-usb3100:	test-usb3100.c usb-3100.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0

test-usb4300:	test-usb4300.c usb-4303.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0

test-usb5201: test-usb5201.c usb-5200.o libmccusb.a
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib -lhidapi-libusb -lusb-1.0 

test-usb5203: test-usb5203.c usb-5200.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib  -lhidapi-libusb -lusb-1.0 

test-usb-tc: test-usb-tc.c usb-tc.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib  -lhidapi-libusb -lusb-1.0 

test-usb-tc-ai: test-usb-tc-ai.c usb-tc-ai.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib  -lhidapi-libusb -lusb-1.0 

test-usb-temp: test-usb-temp.c usb-temp.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib  -lhidapi-libusb -lusb-1.0 

test-usb-temp-ai: test-usb-temp-ai.c usb-tc-ai.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib  -lhidapi-libusb -lusb-1.0 

test-usb-ssr: test-usb-ssr.c usb-ssr.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib  -lhidapi-libusb -lusb-1.0 

test-usb-erb: test-usb-erb.c usb-erb.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib  -lhidapi-libusb -lusb-1.0 

test-usb-pdiso8: test-usb-pdiso8.c usb-pdiso8.o libmccusb.a 
	$(CC) -g -Wall -I. -o $@ $@.c -L. -lmccusb  -lm -L/usr/local/lib  -lhidapi-libusb -lusb-1.0 

####################################################################################################

clean:
	rm -rf \#*\# *.d *.o *~ *.a *.so *.dylib *.dll *.lib *.dSYM $(TARGETS)

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) mcc-libusb/$(DIST_FILES);

install:
	-install -d /usr/local/lib
	-install -c --mode=0755 ./libmccusb.a libmccusb.$(SHARED_EXT) /usr/local/lib
	-/bin/ln -s /usr/local/lib/libmccusb.$(SHARED_EXT) /usr/lib/libmccusb.$(SHARED_EXT)
	-/bin/ln -s /usr/local/lib/libmccusb.a /usr/lib/libmccusb.a
	-install -d /usr/local/include/libusb
	-install -c --mode=0644 ${HEADERS} /usr/local/include/libusb/

uninstall:
	-rm -f /usr/local/lib/libmccusb*
	-rm -f /usr/lib/libmccusb*
	-rm -rf /usr/local/include/libusb
