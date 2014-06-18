#include "imu.h"



int main(int argc, char *argv[])
{


	USBKickerImu   usbdevice;
	int which;
	int time;

	Imu imu;
	imu.usbdevice=&usbdevice;
	imu.loadcalibration();

	if ( argc  <2 )
	{
		printf("Usage:     %s which duration\n",argv[0]);
		printf("Power on:  %s off\n",argv[0]);
		printf("Power off: %s on\n",argv[0]);
		printf("Get  Data: %s get\n",argv[0]);
		return(3);
	}

	printf("Init Board");
	usbdevice.init();



	if (argc==3)	
	{
		which = atoi(argv[1]);
		time = atoi(argv[2]);
		usbdevice.kick(which,time);
	}
	if (argc==2)
		{
		printf("Argv = %s \n",argv[1]);
		if (strcmp(argv[1],"on")==0) usbdevice.power_up();
		if (strcmp(argv[1],"off")==0) usbdevice.power_down();
		if (strcmp(argv[1],"get")==0) while(1==1){
			
			imu.imu_get_values();

			printf("Yaw: %f" ,imu.yaw);
			usleep(30000); };
		}



	usbdevice.close();



	return 0;
}
