#!/bin/bash
 
# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash
#  or
# sudo ./install.bash usb0
#
# where usb0 is whatever network interface you want to set the robot
# up for.  wlan0 is the default.

echo "Installing opentribot upstart scripts."
 
cat opentribot-start > /usr/sbin/opentribot-start
chmod a+x /usr/sbin/opentribot-start
cat opentribot-stop > /usr/sbin/opentribot-stop
chmod a+x /usr/sbin/opentribot-stop
cat opentribot.conf > /etc/init/opentribot.conf
