#!/usr/bin/env python
# -*- encoding=iso-8859-2 -*-
# Written by Wojciech M. Zabołotny <wzab01@gmail.com>
# Copyritght 2022 W.M. Zaabołotny
# The enumeration of usb devices
# Copyright 2005 Wander Lairson Costa

import usb
import struct


busses = usb.busses()

for bus in busses:
	devices = bus.devices
	for dev in devices:
		if dev.idVendor==0xabba and dev.idProduct==0x5301:
		   print("To jest nasze urzzenie!")
		   mydev=dev
		print("Device:", dev.filename)
		print("  Device class:",dev.deviceClass)
		print("  Device sub class:",dev.deviceSubClass)
		print("  Device protocol:",dev.deviceProtocol)
		print("  Max packet size:",dev.maxPacketSize)
		print("  idVendor:",dev.idVendor)
		print("  idProduct:",dev.idProduct)
		print("  Device Version:",dev.deviceVersion)
		for config in dev.configurations:
			print ("  Configuration:", config.value)
			print ("    Total length:", config.totalLength)
			print ("    selfPowered:", config.selfPowered)
			print ("    remoteWakeup:", config.remoteWakeup)
			print ("    maxPower:", config.maxPower)
			for intf in config.interfaces:
				print ("    Interface:",intf[0].interfaceNumber)
				for alt in intf:
					print ("    Alternate Setting:",alt.alternateSetting)
					print ("      Interface class:",alt.interfaceClass)
					print ("      Interface sub class:",alt.interfaceSubClass)
					print ("      Interface protocol:",alt.interfaceProtocol)
					for ep in alt.endpoints:
						print ("      Endpoint:",hex(ep.address))
						print ("        Type:",ep.type)
						print ("        Max packet size:",ep.maxPacketSize)
						print ("        Interval:",ep.interval)

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
   f.bulkWrite(1,c)
   rsp = f.bulkRead(0x82,64,500)
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
  

f=mydev.open()
f.setConfiguration(mydev.configurations[0])
f.claimInterface(0)
# Program the frequency

setsmp(128,1000)
setchans((0,4,3,1))
start()


