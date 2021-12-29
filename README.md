Spp Bridge
==========

This firmware is designed to use QGroundControl on an iPad with a drone that has
a bluetooth connection. As Apple devices can't directly use bluetooth SPP devices 
i made this as a workaround. The firmware tries to connect to a bluetooth SPP
device specified in the config and sets up bi-directional communication on TCP 
port 5760. Devices like an iPad can connect to the WiFi access point and connect to
the TCP port to interact with the drone.

Known Issues
============
* When the bluetooth connection is not established the firmware continuously stays  
  in disovery mode to find the configured device. During the discovery the wifi
  performance is limited and this might result in not being able to connect to
  the WiFi network. 

Test setup
==========

Drone
-----
* Firmware: ArduPilot 4.1.3-rc1
* Bluetooth dongle: JDY-30 (connected to TELEM 1)
* Serial1 configured for 115200 baud and MavLink2 protocol

ESP32
-----
* Hardware: DevkitC
* Firmware: SppBridge
* Bluetooth configured to match JDY-30 MAC and PIN
* ESSID configured with passphrase

iPad
----
* Hardware: iPad Air 2
* App: QGroundControl 4.2.0 (custom build for iPad)
* Commlink configured to TCP 192.168.4.1/5760

Links
=====
* QGroundControl: http://qgroundcontrol.com
* ArduPilot: https://ardupilot.org