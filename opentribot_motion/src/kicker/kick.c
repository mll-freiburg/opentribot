

#include "kick.h"






int main(int argc, char *argv[])
{


	int which;
	int time;

	if ( argc  <2 )
	{
		printf("Usage: %s <KickerNumber> <Duration>\n",argv[0]);
		printf("            0xF000         0xF000    -> Master OFF  (default)\n");
		printf("            0xFFFF	   0xFFFF    -> Master ON\n");
		return(3);
	}


	initKicker();



	if (argc==3)	
	{
		which = atoi(argv[1]);
		time = atoi(argv[2]);
		kick(which,time);
	}
	if (argc==2)
		{
		printf("Argv = %s \n",argv[1]);
		if (strcmp(argv[1],"on")==0) power_up();
		if (strcmp(argv[1],"off")==0) power_down();
		}



	deinitKicker();



	return 0;
}
