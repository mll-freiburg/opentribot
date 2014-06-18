#include <iostream>
#include <fstream>
#include <cstring>
//#include <joystick.h>
#include "time.h"
#include <sys/vfs.h>
#include "UDPSocket.h"
//#include <vector>
#define RECORD_MAX 90
#include <cstring>
using namespace std;
using namespace Tribots;

#include <stdlib.h>
#include <sstream>

#include "usbimu/imu.h"


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
  sock.init_as_client (argv[1], 58585);
//sock.init_serv_addr ("192.168.2.187", 51111);

  //struct statfs fs_stat;   //Free Space Test
  //statfs(".",&fs_stat);
  //cout << 1.0f*(fs_stat.f_blocks-fs_stat.f_bfree)/fs_stat.f_blocks<<endl;


  double a0, a1, a2, a3;
  char buffer[128];

  Imu imu;

  imu.loadcalibration();  

  double imudata[100];

  
  //polling
  fd_set rfds;
  char packet[1024];
  int counter;
  while(1) {
  counter++;

   if ( imu.imu_get_values()>1){
        for (int i=0;i<16;i++)imudata[i]=imu.filter.mat4x4[i];
	imudata[16]=imu.filter.think_x;
        imudata[17]=imu.filter.think_y;
        imudata[18]=imu.filter.think_z;	
        imudata[19]=imu.yaw;	
	imudata[20]=imu.axisrotate.x;
	imudata[21]=imu.axisrotate.y;
	imudata[22]=imu.axisrotate.z;
//	cout <<"ok sending values!!!"<<endl;

        sock.send ((char *) imudata, sizeof(double)*30);
//        printf("\nCOUNT: r%f p%f y%f, ax%.1f,ay%.1f ,th %.1f",imu.filter.kroll,imu.filter.kpitch,imu.filter.rkyaw,imu.filter.accelAngleX,imu.filter.accelAngleY,imu.filter.theta);

//       cout << "\nread " <<  " ev 0:"<< a0 << " 1:" << a1 << " 2:" << a2 << " 3:" << a3;
//      usleep (100000);
}
    }


  return 0;




}




