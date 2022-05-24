#!/usr/bin/env python
# -*- encoding=iso-8859-2 -*-
# Enumerate usb devices
#
#Copyright 2005 Wander Lairson Costa

import usb

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

def readint():
   f.bulkWrite(1,"\004")
   print(f.bulkRead(2,30,500))
def id():
   f.bulkWrite(1,"\000")
   print(f.bulkRead(2,30,500))

def stop():
   f.bulkWrite(1,"\003")
   print(f.bulkRead(2,30,500))


def prstr(s):
   l=""
   for i in s:
     #print i
     if i<0:
       l+=chr(i+256)
     else:
       l+=chr(i)
   print (l)

f=mydev.open()
f.setConfiguration(mydev.configurations[0])
f.claimInterface(0)
f.bulkWrite(1,"To ja\000")
s=f.bulkRead(0x82,30,500)
print(s)
prstr(s)
s=f.bulkRead(0x82,30,500)
print(s)
prstr(s)
#f.bulkWrite(1,(3,0))
f.bulkWrite(1,"No, ale co ja moge")
s=f.bulkRead(0x82,30,500)
print(s)
prstr(s)
s=f.bulkRead(0x82,30,500)
print(s)
prstr(s)

if False:
  adc_cfg=chr(1)
  adc_cfg += chr(1)+chr(127)+chr(0) #Frequency
  adc_cfg += chr(0)+chr(1)+chr(2)+chr(33)+chr(34)+chr(5)+chr(6)+chr(7)+chr(0)
  #adc_cfg = (1,0,128,128,1,2,3,3,255,5,6,7)
  f.bulkWrite(1,adc_cfg)
  s=f.bulkRead(2,30,500)
  prstr(s)
  print(s)
  f.bulkWrite(1,"\005")
  s=f.bulkRead(2,30,500)
  print(s)
  print ("stop")
  f.bulkWrite(1,"\003")
  s=f.bulkRead(2,30,500)
  print(s)
  print("start")
  f.bulkWrite(1,"\002")
  s=f.bulkRead(2,30,5000)
  print(s)
  p=0
  while True:
     s=f.bulkRead(2,3000)
     p+=1
     print ([ "%2.2x" % (i+256*(i<0))  for i in s[0:len(s)]])
     #prstr(s)
    
