//compile:  gcc -lusb kick.c -o kick


#include <stdio.h>
#include <usb.h>
#include <unistd.h>
#include <math.h>



struct usb_bus *USBbus;
struct usb_device *USBdevice;
struct usb_device *current_device;
usb_dev_handle *current_handle;
int in_ep=3;
int out_ep=4;


struct __attribute__ ((packed)) Kicker_t{
	unsigned short Select; 
	unsigned short Time;
};



void openUSBDevice(void)
{
	usb_init();
	usb_find_busses();
	usb_find_devices();
	
	USBbus=usb_get_busses();
	current_device=NULL;
	while(USBbus!=NULL)
		{USBdevice=USBbus->devices;
		while(USBdevice!=NULL)
			{if ((USBdevice->descriptor.idVendor==0x03a3)&&(USBdevice->descriptor.idProduct==0x2b49))
				current_device=USBdevice;
			USBdevice=USBdevice->next;}
		USBbus=USBbus->next;}
	if (current_device==NULL)
		{printf("\n\nCould not find device\n\n");exit(0);}
	fflush(stdout);
	current_handle=usb_open(current_device);

	if (usb_claim_interface(current_handle, 0) <0)
	{
		printf("Error - usb_claim_interface\n");
		exit(0);
	}

	if (usb_clear_halt(current_handle,out_ep) <0)
	{
		printf("Error - usb_clear_halt EP %d\n",out_ep);
		exit(0);
	}
}

void closeUSBDevice(void)
{
	if (usb_release_interface(current_handle, 0) <0)
	{
		printf("Error - usb_release_interface\n");
		exit(0);
	}

	if (usb_close(current_handle) <0)
	{
		printf("Error - usb_close\n");
		exit(0);
	}
}


int initKicker(){


	openUSBDevice();



}

int power_up(){

	
      struct Kicker_t kicker;
	int ret;

	kicker.Select = 65535;
	kicker.Time = 65535;


      ret=usb_interrupt_write(current_handle,out_ep,(char *) &kicker,sizeof(struct Kicker_t),10);

	if (ret != sizeof(struct Kicker_t))
	{
		printf("usb_interrupt_write returned %d instead of %d\n",ret,sizeof(struct Kicker_t));
		//usleep(1000);
	} else {
		//printf("%X %X %X %X\n",rxdata[0],rxdata[1],rxdata[2],rxdata[3]);
	}


	printf ("kicker s : %i %i \n",kicker.Select,kicker.Time);


}

int power_down(){

	
      struct Kicker_t kicker;
	int ret;



	kicker.Select = 61440;
	kicker.Time = 61440;


      ret=usb_interrupt_write(current_handle,out_ep,(char *) &kicker,sizeof(struct Kicker_t),10);

	if (ret != sizeof(struct Kicker_t))
	{
		printf("usb_interrupt_write returned %d instead of %d\n",ret,sizeof(struct Kicker_t));
		//usleep(1000);
	} else {
		//printf("%X %X %X %X\n",rxdata[0],rxdata[1],rxdata[2],rxdata[3]);
	}

	printf ("kicker s : %i %i \n",kicker.Select,kicker.Time);



}


int kick(unsigned int which, unsigned int time_ms ){


      unsigned int time = (unsigned int) (((double)time_ms) *(1000.0f/64.0)); // 1kickeinheit sind 64 mikrosekunden

	
      struct Kicker_t kicker;
      int ret;
      kicker.Select = which;
      kicker.Time = time;


      ret=usb_interrupt_write(current_handle,out_ep,(char *) &kicker,sizeof(struct Kicker_t),10);

	if (ret != sizeof(struct Kicker_t))
	{
		printf("usb_interrupt_write returned %d instead of %d\n",ret,sizeof(struct Kicker_t));
		//usleep(1000);
	} else {
		//printf("%X %X %X %X\n",rxdata[0],rxdata[1],rxdata[2],rxdata[3]);
	}

	printf ("kicker s : %i %i \n",kicker.Select,kicker.Time);



}

int deinitKicker(){


	closeUSBDevice();



}




