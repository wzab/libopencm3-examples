#!/usr/bin/env python
# -*- encoding=iso-8859-2 -*-
# Written by Wojciech M. Zabołotny <wzab01@gmail.com>
# Copyleft 2022 W.M. Zabołotny
# This is a PUBLIC DOMAIN code
#
# The code is somehow based on:
# https://stackoverflow.com/questions/44290837/how-to-interact-with-usb-device-using-pyusb

import usb.core
import usb.util
import threading
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy

global eps
# find our device
dev = usb.core.find(idVendor=0xabba, idProduct=0x5301)

# was it found?
if dev is None:
    raise ValueError('Device not found')

# free up the device from the kernel
for cfg in dev:
    for intf in cfg:
        if dev.is_kernel_driver_active(intf.bInterfaceNumber):
            try:
                dev.detach_kernel_driver(intf.bInterfaceNumber)
            except usb.core.USBError as e:
                sys.exit("Could not detach kernel driver from interface({0}): {1}".format(intf.bInterfaceNumber, str(e)))

# try default conf
print("setting configuration")
dev.set_configuration()
print("config set")

print("trying to claim device")
try:
    usb.util.claim_interface(dev, 0)
    print("claimed device")
except usb.core.USBError as e:
    print("Error occurred claiming " + str(e))
    sys.exit("Error occurred on claiming")
print("device claimed")
cfg=dev.get_active_configuration()
intf=cfg.interfaces()
eps=intf[0].endpoints()

def prstr(s):
   l=""
   for i in s:
     #print i
     if i<0:
       l+=chr(i+256)
     else:
       l+=chr(i)
   print (l)

def cmd(c):
   eps[0].write(c)
   rsp = eps[1].read(64)
   print(rsp)
   prstr(rsp)

def setsmp(presc,per):
   c = struct.pack("<BLL",2,presc,per)
   cmd(c)
   
def setchans(chans):
   fmt = "<"+str(2+len(chans))+"B"
   c = struct.pack(fmt,1,len(chans),*chans)
   print(fmt)
   print(c)
   cmd(c)

def start():
   c = b"\x03"
   cmd(c)
   
def stop():
   c = b"\x00"
   cmd(c)


class receiver(threading.Thread):
   def run(self):
     global eps
     global nchans
     global data
     while True:
       try:
         rsp = eps[2].read(64)
         # Maybe we should do something better with the received values...
         # Now we just print them out.
         if rsp[0] == ord("D"):
            lout=""
            print(rsp)
            # This are the sample data
            data = numpy.roll(data,-1,0)
            for i in range(nchans):
              data[plotlen-1,i] = 256*rsp[2*i+1]+rsp[2*i+2]
         else:
            # This is the start or end message
            print("".join([chr(i) for i in rsp]))
       except Exception as e:
         print(str(e))
         pass      

plotlen = 200
xs = numpy.array([i for i in range(plotlen)])
chans = (0,1,2,3,4)
nchans = len(chans)
data = numpy.zeros((plotlen,nchans),dtype=int)        
setsmp(1000,1000)
setchans(chans)

fig = plt.figure()
#creating a subplot 
ax1 = fig.add_subplot(1,1,1)
lines = plt.plot(xs,data)

t=receiver()
t.start()
start()
while True:
   for i in range(nchans):
      lines[i].set_ydata(data[:,i])
   plt.gca().relim()
   plt.gca().autoscale_view()
   fig.show()
   plt.pause(0.01) 
        
