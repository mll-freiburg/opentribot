/*	Purpose: Interact with USB device firmware using Control Endpoint
	Created: 2008-08-15 by Opendous Inc.
	Last Edit: 2008-09-16 by Opendous Inc.
	Released under the MIT License
*/

#include <stdio.h>
#include <usb.h>
#include "stopwatch.h"
#include <string.h>
#include <iostream>


using namespace std;


/* all the critical information regarding the device and the interface and endpoints you plan to use */
#define VENDORID		0x03a3
#define PRODUCTID		0x2b49
#define CNTRL_BUF_SIZE		64
#define CONFIGNUM		1
#define INTERFACENUM		0
#define TIMEOUT			1500


/* Control Message bmRequestType Masks */
#define REQDIR_HOSTTODEVICE        (0 << 7)
#define REQDIR_DEVICETOHOST        (1 << 7)
#define REQTYPE_STANDARD           (0 << 5)
#define REQTYPE_CLASS              (1 << 5)
#define REQTYPE_VENDOR             (2 << 5)
#define REQREC_DEVICE              (0 << 0)
#define REQREC_INTERFACE           (1 << 0)
#define REQREC_ENDPOINT            (2 << 0)
#define REQREC_OTHER               (3 << 0)

int main(int argc,char**argv)
{
	int value=0;
	char buffer[CNTRL_BUF_SIZE];
	char buffer2[CNTRL_BUF_SIZE];
	/* initialize buffer to a sane value */
	for (int i = 0; i < CNTRL_BUF_SIZE ; i++) {
		buffer[i] = 0;
		buffer2[i] = 0;
	}

       StopWatch stopwatch;

	
	
        if (argc>1){

	if (strcmp(argv[1],"kick")<1){
	if (argc<4){printf("Syntax for kick :   kick whichkicker ms");
		exit(0);
		}
	printf ("Trying to kick");
	buffer[0]=1;
  	buffer[1]=atoi(argv[2]);
	buffer[2]=atoi(argv[3])>>8;
	buffer[3]=atoi(argv[3]);
	}
	
	if (strcmp(argv[1],"off")==0){
	printf ("Shuting off");
	buffer[0]=2;
	}
	if (strcmp(argv[1],"on")==0){
	printf ("Shuting on");
	buffer[0]=3;
	}

}
else 
{
printf("\nTribots Kicker Tool\nUsage: kick whichkicker ms,  on , off\n\n");

exit(0);

}

;


	struct usb_bus *bus;
	struct usb_device *dev;
	usb_dev_handle *udev;
	unsigned char bmRTmask;
	int ret, i;


	printf("STARTED DeviceAccessC\n");

	/* let libusb print all debug messages */
//	usb_set_debug(255);

	usb_init();
	usb_find_busses();
	usb_find_devices();

	/* loop over all busses and all devices and communicate with devices with the correct vendor and product IDs */
	/* Note that if you have several of the same devices connected, it will communicate with each one in turn */
	for (bus = usb_get_busses(); bus; bus = bus->next) {
		for (dev=bus->devices;dev;dev=dev->next) {

			if ((dev->descriptor.idVendor==VENDORID) && (dev->descriptor.idProduct==PRODUCTID)) {

				udev = usb_open(dev);

				/* tell libusb to use the CONFIGNUM configuration of the device */
				usb_set_configuration(udev, CONFIGNUM);

				/* if your device has an alternate setting for this interface, you can change to it */
				//usb_set_altinterface(udev, INTERFACENUM_OTHER);

				#ifdef LIBUSB_HAS_DETACH_KERNEL_DRIVER_NP
				/* detach the automatically assigned kernel driver from the current interface */
				usb_detach_kernel_driver_np(udev, INTERFACENUM);
				#endif

				/* claim the interface for use by this program */
				usb_claim_interface(udev, INTERFACENUM);
				cout <<"Sending and Receiving"<<endl;

				/* TODO: this for() loop is where you should place your device interaction code */
				for (i = 0; i < 100; i++) {
					/* read data from the device and then send it right back, where the first byte should get incremented from 1 to 2 */
					bmRTmask = (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQTYPE_STANDARD);
stopwatch.reset();			
    		ret = usb_control_msg(udev, bmRTmask, 1, 0, 0, buffer2, CNTRL_BUF_SIZE, TIMEOUT);
printf("usb read:%i us ",stopwatch.get_usecs() );
					printf("\n");
					for(int i=0;i<16;i++){
						if(i%8==0)printf("|");
						printf(" %3u",(unsigned char) buffer2[i]);
					}
/*					printf("\tTCNT1=%5d, MainTask_Calls:%3d)\n",
							((unsigned char)buffer[6] | ((unsigned char)buffer[7] << 8)),
							(unsigned char)buffer[4]);
*/
					bmRTmask = (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQTYPE_STANDARD);
					stopwatch.reset();			
					ret = usb_control_msg(udev, bmRTmask, 2, 0, 0, buffer, CNTRL_BUF_SIZE, TIMEOUT);
//printf("\nusb write:%i us ",stopwatch.get_usecs() );
stopwatch.reset();			
					printf("\n");
					for(int i=0;i<16;i++){
						if(i%8==0)printf("|");
						printf(" %3u",(unsigned char) buffer[i]);
					}
					printf("[Write] returned: %d bytes successfully written\n", ret);
				}
					usleep(5000);


				/* make sure other programs can still access this device */
				/* release the interface and close the device */
				usb_release_interface(udev, INTERFACENUM);
				usb_close(udev);

			}
		}
	}
    return 0;
}
