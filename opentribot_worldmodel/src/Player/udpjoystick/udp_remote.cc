#include <iostream>
#include <fstream>
#include <cstring>
//#include <joystick.h>
#include "../../Fundamental/Joystick.h"
#include "time.h"
#include <sys/vfs.h>
#include "../../Communication/UDPSocket.h"
//#include <vector>
#define RECORD_MAX 90
#include <cstring>
using namespace std;
using namespace Tribots;
int
main (int argc, char *argv[])
{

  UDPSocket sock;
//  sock.init_socket_fd (6011);

//  sock.init_serv_addr ("127.0.0.1", 51111);
  sock.init_as_client ("172.16.38.9", 51111);
//sock.init_serv_addr ("192.168.2.187", 51111);

  //struct statfs fs_stat;   //Free Space Test
  //statfs(".",&fs_stat);
  //cout << 1.0f*(fs_stat.f_blocks-fs_stat.f_bfree)/fs_stat.f_blocks<<endl;


  double a0, a1, a2, a3;
  char buffer[128];

  Joystick*js;
  JoystickState jstate;
  Joystick j1("/dev/input/js0");


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
	for (int i=0;i<js->axis.size();i++)
	{	jstate.axis[i]=js->axis[i];
	cout << "i: "<<i<< ":"<<js->axis[i];
}
cout << endl;

	for (int i=0;i<js->button.size();i++)
		jstate.buttons[i]=js->button[i];



//    	memcpy ((void *)&jstate,packet,sizeof(JoystickState));

	sock.send ((char *) &jstate, sizeof(JoystickState));
    

//       cout << "\nread " <<  " ev 0:"<< a0 << " 1:" << a1 << " 2:" << a2 << " 3:" << a3;
      usleep (100000);

    }
    else {
     cerr<< "\nstrange select event occurred";
    }
  }









  return 0;




}
