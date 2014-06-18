/*	Purpose: Interact with USB device firmware using Control Endpoint
	Created: 2008-08-15 by Opendous Inc.
	Last Edit: 2008-09-16 by Opendous Inc.
	Released under the MIT License
*/

#include "usbimu.h"


int main(int argc,char**argv)
{
int value=0;


        if (argc>1)value=atoi(argv[1]);
printf ("Setting value %i",value);

USBImu imu;
imu.init();
imu.loop(value);
imu.close();




			
    return 0;
}
