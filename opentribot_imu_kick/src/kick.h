//compile:  gcc -lusb kick.c -o kick
#ifndef ___USBIMUKICKER
#define ___USBIMUKICKER




#include <stdio.h>
#include <usb.h>
#include <unistd.h>
#include <math.h>
#include "stopwatch.h"
#include <string.h>
#include <imu.h>





#define VENDORID                0x03a3
#define PRODUCTID               0x2b49
#define CNTRL_BUF_SIZE          64
#define CONFIGNUM               1
#define INTERFACENUM            0
#define TIMEOUT                 1500

/* Control Message bmRequestType Masks */
#define REQDIR_HOSTTODEVICE        (0 << 7)
#define REQDIR_DEVICETOHOST        (1 << 7)
#define REQTYPE_STANDARD           (0 << 5)
#define REQTYPE_CLASS              (1 << 5)
#define REQTYPE_VENDOR             (2 << 5)
#define REQREC_DEVICE              (0 << 0)
#define REQREC_INTERFACE           (1 << 0)
#define REQREC_ENDPOINT            (2 << 0)
//#define REQREC_OTHER    





struct USBKickerImu 
{
struct usb_bus *bus;
struct usb_device *dev;
usb_dev_handle *udev;
unsigned char bmRTmask;
int ret, i;

USBKickerImu(){


}

double acc[3];
double gyro[3];
double joy[2];
double but[2];
int slow[3];











unsigned char buffer_send[CNTRL_BUF_SIZE];
unsigned char buffer_receive[CNTRL_BUF_SIZE];


void clear_buffers(){
	int i;

 	for(i=0;i<CNTRL_BUF_SIZE;i++)buffer_send[i]=0;
	for(i=0;i<CNTRL_BUF_SIZE;i++)buffer_receive[i]=0;




}
void openUSBDevice(void)
{
	clear_buffers();

	usb_init();
	usb_find_busses();
	usb_find_devices();
	
	for (bus=usb_get_busses();bus;bus=bus->next){
	 	for (dev=bus->devices;dev;dev=dev->next){
			if ((dev->descriptor.idVendor==VENDORID)&&(dev->descriptor.idProduct==PRODUCTID)) {
		   udev= usb_open(dev);
		   usb_set_configuration(udev, CONFIGNUM);
                   #ifdef LIBUSB_HAS_DETACH_KERNEL_DRIVER_NP
                   usb_detach_kernel_driver_np(udev, INTERFACENUM);
		   #endif
		   usb_claim_interface(udev, INTERFACENUM);
		   return ;
		}}}

	printf("DEvice not found\n");
	exit(0);
}

void closeUSBDevice(void)
{
	if (usb_release_interface(udev, 0) <0)
	{
		printf("Error - usb_release_interface\n");
		exit(0);
	}

	if (usb_close(udev) <0)
	{
		printf("Error - usb_close\n");
		exit(0);
	}
}


int init(){


	openUSBDevice();



}

int  get(int val){

	int ret;	
	//printf("Read Data");
       	buffer_send[0]=0;
	StopWatch sw;
	sw.reset();
   	bmRTmask = (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQTYPE_STANDARD);
        ret = usb_control_msg(udev, bmRTmask, 1, 0, 0, (char*)buffer_receive, CNTRL_BUF_SIZE, TIMEOUT);


	if (ret != 64)
	{
		printf("usb_controlmsg_read returned %d instead of %d\n",ret,64);
		//usleep(1000);
	} else {

	  int i= 0;
//	  printf("\n%i us elapsed in getting kicker data.",sw.get_usecs());
//	 for(i=0;i<16;i++){printf(" %3u",(unsigned char) buffer_receive[i]);}
//	printf("\n");


//acc[0] = (buffer[2] << 2) + ((buffer[5] >> 2) & 0x03);
acc[1] = (buffer_receive[3] << 2) + ((buffer_receive[5] >> 4) & 0x03);
acc[2] = (buffer_receive[4] << 2) + ((buffer_receive[5] >> 6) & 0x03);
acc[0] = (buffer_receive[2] << 2) + ((buffer_receive[5] >> 3) & 2);
acc[1] = (buffer_receive[3] << 2) + ((buffer_receive[5] >> 4) & 2);
acc[2] = (buffer_receive[4] << 2) + ((buffer_receive[5] >> 5) & 2);
but[0] = (buffer_receive[5] >> 2) & 1;
      but[1] = (buffer_receive[5] >> 3) & 1;
      joy[0] = buffer_receive[0] * 1.0f / 128;
      joy[1] = buffer_receive[0] * 1.0f / 128;
        //  printf ("\na1 %f  a2 %f  a3 %f", acc[0], acc[1], acc[2]);
        
              // y p r
                    gyro[0] = (((buffer_receive[6 + 3]>>2) << 8) + buffer_receive[6 + 0]);
                    gyro[1] = (((buffer_receive[6 + 5] >>2)<<8)  + buffer_receive[6 + 2]);
                    gyro[2] = (((buffer_receive[6 + 4] >>2) << 8) + buffer_receive[6 + 1]);
		    slow[0] = (buffer_receive[6 + 3] & 0x02) ;
                    slow[1] = (buffer_receive[6 + 3] & 0x01) ;
                    slow[2] = (buffer_receive[6 + 4] & 0x02) ;

	}





}

int power_up(){

	int ret;	
	printf("Power Up Device!");
       	buffer_send[0]=3;

   	bmRTmask = (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQTYPE_STANDARD);
        ret = usb_control_msg(udev, bmRTmask, 2, 0, 0,(char*) buffer_send, CNTRL_BUF_SIZE, TIMEOUT);


	if (ret != 64)
	{
		printf("usb_interrupt_write returned %d instead of %d\n",ret,64);
		//usleep(1000);
	} else {
		//printf("%X %X %X %X\n",rxdata[0],rxdata[1],rxdata[2],rxdata[3]);
	}
}

int power_down(){

	int ret;
	printf("Power Down Device!");
       	buffer_send[0]=2;

   	bmRTmask = (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQTYPE_STANDARD);
        ret = usb_control_msg(udev, bmRTmask, 2, 0, 0,(char*) buffer_send, CNTRL_BUF_SIZE, TIMEOUT);


	if (ret != 64)
	{
		printf("usb_interrupt_write returned %d instead of %d\n",ret,64);
		//usleep(1000);
	} else {
		//printf("%X %X %X %X\n",rxdata[0],rxdata[1],rxdata[2],rxdata[3]);
	}

//	printf ("kicker s : %i %i \n",kicker.Select,kicker.Time);



}


int kick(unsigned int which, unsigned int time_ms ){


      unsigned int time = (unsigned int) (((double)time_ms) *(1000.0f/64.0)); // 1kickeinheit sind 64 mikrosekunden

       buffer_send[0]=1;
       buffer_send[1]=which;
       buffer_send[2]=time_ms>>8;
       buffer_send[3]=time_ms;

	
      int ret;

        bmRTmask = (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQTYPE_STANDARD);
        ret = usb_control_msg(udev, bmRTmask, 2, 0, 0,(char*) buffer_send, CNTRL_BUF_SIZE, TIMEOUT);


	if (ret != 64)
	{
		printf("usb write returned %d instead of %d\n",ret,64);
		//usleep(1000);
	} else {
		//printf("%X %X %X %X\n",rxdata[0],rxdata[1],rxdata[2],rxdata[3]);
	}

	
	printf ("\nkicker s : %i %i \n",ret,time_ms);



}

int close(){


	closeUSBDevice();



}





};


#endif
