#include <iostream>
#include <fstream>
#include <cstring>
//#include <joystick.h>
#include "../../../Fundamental/Joystick.h"
#include "time.h"
#include <sys/vfs.h>
#include "../../../Communication/UDPSocket.h"
#include "../../../Structures/DriveVector.h"
//#include <vector>
#define RECORD_MAX 90
#include <cstring>
using namespace std;
using namespace Tribots;

#include <stdlib.h>
#include <sstream>




int
main (int argc, char *argv[])
{
  	



if (argc<3){
	cout << "usage: "<<argv[0]<< " TargetIP"<<"  max_speed"<<endl;


	return(0) ;



}



  UDPSocket sock;
//  sock.init_socket_fd (6011);

//  sock.init_serv_addr ("127.0.0.1", 51111);
  sock.init_as_client (argv[1], 51111);
//sock.init_serv_addr ("192.168.2.187", 51111);

  //struct statfs fs_stat;   //Free Space Test
  //statfs(".",&fs_stat);
  //cout << 1.0f*(fs_stat.f_blocks-fs_stat.f_bfree)/fs_stat.f_blocks<<endl;


  double a0, a1, a2, a3;
  char buffer[128];

  Joystick*js;
  JoystickState jstate;
  Joystick j1("/dev/input/js0");


  DriveVector dv;

  js=&j1;
  

  
  //polling
  fd_set rfds;
  char packet[1024];
  int counter;
  cout << "starting joy loop "<<endl;
  while(1) {
  counter++;

    FD_ZERO(&rfds);
    FD_SET(js->file_descriptor, &rfds);
    int retval = select(js->file_descriptor+1, &rfds, NULL, NULL, NULL);
    if (retval > 0 && FD_ISSET(js->file_descriptor,&rfds) ) {
    js->update();


    float max_speed=atof(argv[2]);
    if (max_speed>5.0f ) {
	max_speed=5;
	cout << "Achtung , lieber joystick nicht schneller als 3 m/s fahren !!!"<<endl;
	}

    dv.vtrans.x=js->axis[0]*max_speed;
    dv.vtrans.y=-js->axis[1]*max_speed;
    dv.vrot=-js->axis[2]*max_speed*2;


cout <<" X:"<<dv.vtrans.x<< " Y:"<<dv.vtrans.y<< " VROT: "<< dv.vrot<< endl;
    if (js->button[0]==true)
     dv.kick=1;
	else dv.kick=0;


//    	memcpy ((void *)&jstate,packet,sizeof(JoystickState));


	//sock.send ((char *) &dv, sizeof(DriveVector));

  	ostringstream outstr;	outstr<<dv;
	sock.send ((char *) outstr.str().c_str(), outstr.str().size());


    

//       cout << "\nread " <<  " ev 0:"<< a0 << " 1:" << a1 << " 2:" << a2 << " 3:" << a3;
      usleep (100000);

    }
    else {
     cerr<< "\nstrange select event occurred";
    }
  }









  return 0;




}




