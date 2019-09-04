# Measurement Computing (MCC) Linux Drivers

[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](http://www.gnu.org/licenses/gpl-3.0)
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](http://www.gnu.org/licenses/lgpl-3.0)

:hear_no_evil: :see_no_evil: :speak_no_evil:

This repository contains Linux drivers for data acquisition boards from
Measurement Computing Corp., MCC (aka ComputerBoards).  All these drivers are
maintained by Warren J. Jasper (wjasper@ncsu.edu).  Please send any
comments, suggestions, or questions to me.  All the PCI drivers are licensed
under the Gnu Public License while the USB, Ethernet, and Bluetooth drivers are
licensed under the Lesser Gnu Public License or LGPL.  Most PCI drivers
will run under the Linux 2.6, 3.X and 4.X kernels.

For more information on these cards, go to 
http://www.measurementcomputing.com

If you don't see a driver for a card you want or have, or if there is
a feature that you want but are having problems implementing, please
write me (wjasper@ncsu.edu) about it.

There have been many requests for Python drivers, which I have put off
for many reasons.  However, I am beginning the process starting with
the E-1608.py (Ethernet devices) and am now writing them for USB HID
devices. Please send me comments and feedback.

**Note:** There have been many changes to the 2.6 kernel API.  All the
drivers have been tested with the 2.6.22 kernel.  There are backward
compatibility issues, so it may or may not work under earlier versions
of 2.6 (pre 2.6.22).  Please send email and I'll try to work them out,
or upgrade to the 2.6.22 or later kernel.  Around 2.6.29, the kernel
API changed enough that I started a new version of the modules. I do
not have a new version of the modules for each version of the kernel.
If you notice problems that are kernel version related, use a later
version or email me.  The pci drivers should also work on the 3.X and
4.X kernels.

I have divided the drivers into categories by bus type.  Thus, all the
USB drivers are in the USB directory, etc. 

I have written drivers for the following boards:

**PCI Cards**
1. PCI-DAS08                              
2. PCI-DAS1602/16                        
3. PCI-DAS4020/12                         
4. PCI-DIO24,PCIe, DIO24/LP, DIO24/S      
5. PCI-DIO24H                            
6. PCI-DIO48H                             
7. PCI-DIO96                              
8. PCI-DIO96H                             
9. PCI-DAS1000                            
10. PCI-DAS1001                            
11. PCI-DAS1002                            
12. PCI-CTR05                              
13. PCI-CTR10                             
14. PCI-CTR20HD                            
15. PCI-DDA0x-12                           
16. PCI-DDA0x-16                           
17. PCI-QUAD04
18. PCIM-DAS1602/16
19. PCIM-DAS16Jr/16

**Ethernet**
1. E-1608   (C/Python)
2. E-DIO24  (C/Python)
3. E-TC     (C/Python)
4. E-TC32   (C/Python)

**New**:heavy_exclamation_mark: Python drivers for E-1608, E-DIO24, E-TC, E-TC32

**Bluetooth**
1. BTH-1208LS

**USB**
*Uses the libusb.1.0/HIDAPI*   
   USB-1208FS, USB-1024LS, USB-1024HLS, USB-1608FS, USB-1208LS
   USB-TC, USB-TEMP, USB-1096HFS, USB-1616FS miniLAB-1008, USB-5201, USB-5203
   USB-SSR24, USB-SSR08, USB-3101,USB-3102, USB-3103,USB-3104, USB-3105, 
   USB-3106, USB-3110, USB-3112, USB-3114, 
   USB-501, USB-502, USB-503, USB-504, USB-ERB, USB-DIO96H, USB-DIO96H/50, 
   USB-4301, USB-4303, USB-DIO24, USB-DIO24H, USB-1408FS, USB-TC-AI, USB-TEMP-AI
   USB-1608HS, USB-1608HS-2AO, USB-2416, USB-1208HS, USB-1608G, USB-1608GX, UB-1608GX-2AO
   USB-1608FS-Plus, USB-2633, USB-2637, USB-201, USB-204, USB-205, USB-CTR8, USB-1208FS-Plus,
   USB-1408FS-Plus, USB-2020 USB-2001TC USB-2408 USB-7202 USB-7204 USB-DIO32HS USB-1808

**Python**
  USB-1208LS, USB-miniLAB1008, USB-1024LS, USB-1024HLS, USB-DIO24, USB-DIO24H, USB-SSR24, 
  USB-SSR08, USB-ERB24, USB-ERB08, USB-PDISO8 USB-1208FS, USB-1408FS, USB-1608FS,
  USB-2408, USB-2408-2AO, USB-3101, USB-3101, USB-3102, USB-3103, USB-3104, USB-3105,
  USB-3106, USB-3110, USB-3112, USB-3114, USB-2001TC

**New Python**:heavy_exclamation_mark: USB-1208LS.py, USB-1024LS.py (also works for USB-1024HLS, 
      USB-DIO24 and USB-DIO1024H), USB-SSR24, USB-SSR08, USB-ERB24, USB-ERB08, USB-PDISO8, USB-31XX,
      USB-2001TC

====================================================================
## FAQ:  Here are some questions that I sometimes get that might help.

1. Q: Should Plug-And-Play be set in the BIOS  
   A: You should disable/Turn off Plug-And-Play in the BIOS.

2. Q: The newer Makefiles don't work under 2.4.X kernel.  
   A: Depending on your distribution, esp. Red-Hat, the new Makefile won't
      work for the 2.4.X kernel out of the box.  Please do the following:
       
       1. install the kernel-source (or kernel-sourcecode on Fedora) rpm. 
       2. cd /usr/src/linux-*
       3. type "make xconfig"
       4. click "Save and Exit"
       5. IMPORTANT:  do not do anything else, esp make dep, make modules.
          You either need to recompile the entire kernel or none at all.
          You should see a .config file in /usr/src/linux-*
       Now the Makefile will work.  This should not be an issue under 2.6.X.

3. Q: The newer Makefiles don't work under 2.6.X kernel.  
   A: If the driver is not compiling, chances are you forgot to include the
      kernel-include pacakge for your disribution.  They go by different names depending
      upon your distribution, but a good place to look is in /lib/modules/`uname -r`/build/include.
      This directory should not be empty.

4. Q: How do I know if I have a hardware problem with the board?  
   A: Measurement Computing tests each and every board and QC's it before
   shipping.  It is rare, but possible that you have a bad board. Run InstaCal (under Windows). If InstaCal fails, you should
   contact Measurement Computing, as it is likely you have a hardware problem.
      
5. Q: The driver fails to load.  
   A: There are many reasons why the driver may fail to load.  Check that  
        a) Modules are enabled in the kernel.  
        b) Check that the board is installed.  You can not install the driver
           if the boards are not installed.  

    Under the new 2.6 improved drivers, the modules should autoload if
    you have pci-coldplug installed.  After typeing

    ```bash
    $ make
    $ make install
    $/sbin/depmod -a
    ```
    
    This should put the correct entry in /lib/modules/`uname -r`/modules.alias file and 
    /lib/modules/`uname -r`/modules.pcimap
    and your system will load the driver (kernel module) on bootup.  If you update
    your kernel, you will need to repeat the process (make, make install, depmod -a).
    If this fails, put the following line in rc.local:
        
      ```bash
       if [ -x /sbin/modprobe ]; then
         /sbin/modprobe driver_name
       fi
      ```

6. Q: The driver loads, but the test program does not run correctly.  
   A: Type "dmesg" as see if there is a message at the end about the driver
      loading correctly. If the driver loaded correctly, but you are not getting
      what you think is the correct answer, check the cable (either it is loose or
      you put in in backwards), or your wiring.  It is possible that the board is bad
      (See Question 4).

7.  Q: How do I access the drivers as a non-root user.  
    A: for the 2.6 kernels: copy the file 60-mcc.rules to /etc/udev/rules.d and restart udev (or reboot)  
     
     ```bash
     $ cp 60-mcc.rules /etc/udev/rules.d  
     $ /sbin/udevadm control --reload-rules
     ```

     for the 3.X and 4.X kernels: copy the file 61-mcc.rules to /etc/udev/rules.d and restart udev (or reboot)
     
     ```bash
     $ cp 61-mcc.rules /etc/udev/rules.d  
     $ /sbin/udevadm control --reload 
     ```

8. Q: Under Ubuntu, the drivers compile correctly, but do not load.  I can not run the test-pci* program.  
   A: Recent versions of Ubuntu are now shipping with the COMEDI drivers.  These are a set of
      data acquistion drivers for a variety of boards that conflict with my drivers.  The drivers
      in question may look like cb_* or 8255_pci.  Type lsmod -vv | grep cb  or lspci -vv to 
      see the names of the driver associated with your board.  Then go to /etc/modprobe.d/blacklist.conf
      and add the line
      
      ```bash
      blacklist 8255_pci (or whatever the name of the kernel module is)
      update-initramfs -u
      ```
      
      Then run 
     
     ```bash
      $ depmod -a 
     ```
     
     and reboot.  After reboot, use lsmod is see if the drivers are gone. If not, seach the web blacklisting
     kernel modules.

9. Q: Under Raspian on the Raspberry Pi, I can not run the test program except as root.  
   A: Go to /etc/udev/rules.d and rename the file 61-mcc.rules to 99-mcc.rules and reboot.
   
