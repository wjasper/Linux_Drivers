/***************************************************************************
 Copyright (C) 2007-2015  Warren J. Jasper   <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-DDA0X-12, is free software; you can redistribute it
 and/or modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation; either version 2 of the
 License, or (at your option) any later version, provided that this
 copyright notice is preserved on all copies.

 ANY RIGHTS GRANTED HEREUNDER ARE GRANTED WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, AND FURTHER,
 THERE SHALL BE NO WARRANTY AS TO CONFORMITY WITH ANY USER MANUALS OR
 OTHER LITERATURE PROVIDED WITH SOFTWARE OR THAM MY BE ISSUED FROM TIME
 TO TIME. IT IS PROVIDED SOLELY "AS IS".

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
***************************************************************************/
#if defined(CONFIG_MODVERSIONS) && !defined(MODVERSIONS)
#  define MODVERSIONS
#endif

#ifndef LINUX_VERSION_CODE
#  include <linux/version.h>
#endif

#ifndef VERSION_CODE
#  define VERSION_CODE(vers,rel,seq) ( ((vers)<<16) | ((rel)<<8) | (seq) )
#endif

#if LINUX_VERSION_CODE < VERSION_CODE(2,6,0) /*  < 2.6 */
#  error "This kernel is too old: not supported by this file"
#endif

#if LINUX_VERSION_CODE >= VERSION_CODE(2,6,0) && LINUX_VERSION_CODE < VERSION_CODE(2,6,27)
#include "dda0X-12_2_6.c"
#endif

#if LINUX_VERSION_CODE >= VERSION_CODE(2,6,29) && LINUX_VERSION_CODE < VERSION_CODE(2,6,38)
#include "dda0X-12_2_6_29.c"
#endif

#if LINUX_VERSION_CODE >= VERSION_CODE(3,0,0) && LINUX_VERSION_CODE < VERSION_CODE(3,7,0)
#include "dda0X-12_3_3_7.c"
#endif

#if LINUX_VERSION_CODE >= VERSION_CODE(3,7,0) && LINUX_VERSION_CODE < VERSION_CODE(3,20,0)
#include "dda0X-12_3_10_11.c"
#endif

#if LINUX_VERSION_CODE >= VERSION_CODE(4,0,0) && LINUX_VERSION_CODE < VERSION_CODE(4,20,0)
#include "dda0X-12_4_0_8.c"
#endif

#if LINUX_VERSION_CODE >= VERSION_CODE(4,20,0)
#error "Your kernel is too new for the current driver.  It may work but is untested."
#endif

