#!/bin/sh

testForModule () {
    res=`lsmod | grep "$1   " | awk '{print $1}'`

    if [ "$res" == "$1" ] 
	then
	echo "(OK)     Found module [$1]"
    else
	echo "(FAILED) Module [$1] not loaded."
    fi
}

testForDevice () {
    if ( test -c $1 ) then
	echo "(OK)     Found device [$1]"
    else
	echo "(FAILED) Device [$1] not present."
    fi
    
    if ( test -r $1 ) then
	echo "(OK)     Device [$1] readable"
    else
	echo "(FAILED) Device [$1] not readable for user."
    fi
    
    if ( test -w $1 ) then
	echo "(OK)     Device [$1] writeable"
    else
	echo "(FAILED) Device [$1] not writeable for user."
    fi
}


modules=""
devices=""

if [ "$1" == "desktop" ]
    then
    modules=""
    devices=""
elif [ "$1" == "canrobot" ]
    then
    modules="video1394 raw1394 ohci1394 ieee1394 pcan"
    devices="/dev/raw1394 /dev/video1394/1 /dev/pcan32"
elif [ "$1" == "serrobot" ]
    then
    modules="video1394 raw1394 ohci1394 ieee1394"
    devices="/dev/raw1394 /dev/video1394/1 /dev/ttyUSB0"
else
    echo "Usage: $0 < desktop | canrobot | serrobot >"
    exit -1
fi



for module in `echo $modules`
do
  testForModule "$module"
done



for device in  `echo $devices`
do
  testForDevice "$device"
done
