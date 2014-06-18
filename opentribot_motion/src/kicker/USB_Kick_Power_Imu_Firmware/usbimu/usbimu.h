/*	Purpose: Interact with USB device firmware using Control Endpoint
	Created: 2008-08-15 by Opendous Inc.
	Last Edit: 2008-09-16 by Opendous Inc.
	Released under the MIT License
*/

#include <stdio.h>
#include <usb.h>
#include "stopwatch.h"
#include <iostream>





/* all the critical information regarding the device and the interface and endpoints you plan to use */
#define VENDORID		0x03A3
#define PRODUCTID		0x2B49
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


struct USBImu{
	struct usb_bus *bus;
	struct usb_device *dev;
	usb_dev_handle *udev;
	char buffer[CNTRL_BUF_SIZE];
	unsigned char bmRTmask;
	int ret, i;
       StopWatch stopwatch;
	int value;
	double acc[3];
	double gyro[3];
   	double joy[2];
        double but[2];
        int slow[3];


USBImu(){
	value=0;
	for (i = 0; i < CNTRL_BUF_SIZE ; i++) {
		buffer[i] = 0;
	}
}

void init(){





	usb_set_debug(255);

	usb_init();
	usb_find_busses();
	usb_find_devices();

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
				return ;

			}}}





}



void close(){

				/* make sure other programs can still access this device */
				/* release the interface and close the device */
				usb_release_interface(udev, INTERFACENUM);
				usb_close(udev);


}
void get(int value){




/* read data from the device and then send it right back, where the first byte should get incremented from 1 to 2 */
bmRTmask = (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQTYPE_STANDARD);
stopwatch.reset();			
   		ret = usb_control_msg(udev, bmRTmask, 1, 0, 0, buffer, CNTRL_BUF_SIZE, TIMEOUT);
//printf("\nusb read:%i us ",stopwatch.get_usecs() );
		printf("\n  -- ");
		for(int i=0;i<20;i++){
			if(i%8==0)printf(" | ");
//			printf(" %3u",(unsigned char) buffer[i]);
		}
// COnvert values to Wii values
  // Reihenfolge x y z   
  acc[0] = (buffer[2] << 2) + ((buffer[5] >> 3) & 2) ;
  acc[1] = (buffer[3] << 2) + ((buffer[5] >> 4) & 2);
  acc[2] = (buffer[4] << 2) + ((buffer[5] >> 5) & 6) ;
  but[0]=(buffer[5]>>2) &1;
  but[1]=(buffer[5]>>3) &1;
  joy[0]=buffer[0]*1.0f/128;
  joy[1]=buffer[0]*1.0f/128;




printf ("\na1 %f  a2 %f  a3 %f", acc[0],acc[1],acc[2] );

  // yaw pitch roll
  gyro[0] = (((buffer[6+4]&0xFC)<<6) + buffer[6+1]);
  gyro[1] = (((buffer[6+5]&0xFC)<<6) + buffer[6+0]);
  gyro[2] = (((buffer[6+3]&0xFC)<<6) + buffer[6+2]);


  slow[0]=(buffer[6+3]&0x02)>>1;  
  slow[1]=(buffer[6+4]&0x02)>>1;  
  slow[2]=(buffer[6+3]&0x01)>>0;  








printf (" g1 %f  g2 %f  g3 %f", gyro[0],gyro[1],gyro[2] );



/*					printf("\tTCNT1=%5d, MainTask_Calls:%3d)\n",
							((unsigned char)buffer[6] | ((unsigned char)buffer[7] << 8)),
							(unsigned char)buffer[4]);
*/
		buffer[3] = 33; // just to check if data is being sent
		bmRTmask = (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQTYPE_STANDARD);
			buffer[0]=value;
stopwatch.reset();			
		ret = usb_control_msg(udev, bmRTmask, 2, 0, 0, buffer, CNTRL_BUF_SIZE, TIMEOUT);
//printf("\nusb write:%i us ",stopwatch.get_usecs() );
stopwatch.reset();			
//					printf("[Write] returned: %d bytes successfully written\n", ret);

				







}





void loop(int value){




				/* TODO: this for() loop is where you should place your device interaction code */
				for (int i = 0; i < 1000; i++) {
					get(value);
					
					usleep(5000);
				}









}

};


