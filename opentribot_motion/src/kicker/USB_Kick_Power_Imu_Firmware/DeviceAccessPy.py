#    Purpose: Interact with USB device firmware
#    Created: 2008-08-15 by Opendous Inc.
#    Last Edit: 2008-09-16 by Opendous Inc.
#    Released under the MIT License
import usb

# all the critical information regarding the device and the interface and endpoints you plan to use.
vendorid    = 0x03EB
productid   = 0x206C
confignum   = 1
interfacenum= 0
timeout     = 1500


# Control Message bmRequestType Masks
REQDIR_HOSTTODEVICE = (0 << 7)
REQDIR_DEVICETOHOST = (1 << 7)
REQTYPE_STANDARD    = (0 << 5)
REQTYPE_CLASS       = (1 << 5)
REQTYPE_VENDOR      = (2 << 5)
REQREC_DEVICE       = (0 << 0)
REQREC_INTERFACE    = (1 << 0)
REQREC_ENDPOINT     = (2 << 0)
REQREC_OTHER        = (3 << 0)



# loop over all busses and all devices and find the one with the correct vendor and product IDs
# Note that if you have several of the same devices connected, it will select the last
print "STARTED DeviceAccessPy"
busses = usb.busses()
for bus in busses:
    devices = bus.devices
    for dev in devices:
        if (dev.idVendor == vendorid) & (dev.idProduct == productid):
            founddev = dev
            foundbus = bus

# the device has been found, otherwise exit with error
bus = foundbus
dev = founddev

# open the device for communication
handle = dev.open()

# choose which of the device's configurations to use
handle.setConfiguration(confignum)

# choose which interface to interact with
handle.claimInterface(interfacenum)


#TODO: this while() loop is where you should place your device interaction code
buffer = () # initialize the read/write buffer
maxlen = 8 # size of buffer
i = 0
while (i < 10):
    # read data from the device and then send it right back, where the first byte should get incremented from 1 to 2
    bmRTmask = (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQTYPE_STANDARD)
    buffer = handle.controlMsg(requestType = bmRTmask, request = 1, value = 0,
        index = 0, buffer = maxlen, timeout = timeout)
    print buffer, "TCNT1=", (buffer[6] | (buffer[7] << 8)), "MainTask_Calls:", buffer[4]

    # alter the first byte of the received tuple -- need to recreate buffer as tuples are immutable
    buffer = (1, buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7])

    bmRTmask = (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQTYPE_STANDARD)
    handle.controlMsg(requestType = bmRTmask, request = 2, value = 0,
        index = 0, buffer = buffer, timeout = timeout)
    i = i+1

# NOTE: For control msg reads, buffer is the maximum length of the returned
#    array, whereas for control writes it is the actual data buffer to be sent

# release the device (also called automatically on __del__)
handle.releaseInterface()

