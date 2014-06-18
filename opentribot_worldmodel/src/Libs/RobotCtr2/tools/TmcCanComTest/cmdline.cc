
#include "TmcCanCom.h"
#include <fstream>
#include <iostream>

using namespace std;
using namespace RobotCtr2;

#define usage() std::cerr << "Usage: " << argv[0] << " [-f] \n"

int main(int argc, char* argv[]) {
  unsigned int confec;
  CanCyclicDataFrame frame;
  int  paramoffset = 0;
  bool force = false;
  int motor  = 1;

  if (argc > 1 && argv[1][0] == '-' && argv[1][1] == 'f') {
    paramoffset++;
    force = true;
  }

  if (!force) {
    char c;
    std::cout << "\nThis program will !! move all motors !! and write data in file data.log."
	      << "\nDo you realy want to move wheels? (y,n)\n";
    std::cin >> c;
    if (c != 'y') exit(0);
  }

  TmcCanCom* com = 0;
  
  ofstream f("log.dat");  

  try {
    com = new TmcCanCom();    
  }
  catch (TmcCanException& e) {
    std::cout << "We have an exception: " << e.what() << "\n";
    exit(0);
  }

#if 1
   confec = 0;

   confec = com->setPIDParams(MOTOR1, 10, 80, 0);
   confec = com->setPIDParams(MOTOR2, 10, 80, 0);
   confec = com->setPIDParams(MOTOR3, 10, 80, 0);
   
   if (confec) std::cout << "pid " << com->resolveCONFEC(confec) << "\n";

   //com->startCyclicMessageColection();
  
   confec = com->setCyclicSendMode(CSM_ALL);
 
   if (confec) std::cout << "cycmode " << com->resolveCONFEC(confec) << "\n";

   //   sleep(2);

  for (int i=0; i<300; i++) 
    {
      //usleep(20000);
      //com->getLastCyclicFrame( frame );
      com->getNextCyclicFrame( frame );

      f << i << "  " 
	<< (int) frame.vel.CycleId << "  " 
	<< frame.vel.motorVel1 << "  " 
	<< frame.current.motorC1  << "  " 
	<< frame.vel.vcc  << "  "
	<< frame.output.motorO1 << "  "
	<< (int) frame.cycInfo.CycleIdle_ms << "  "
	<<"\n";
      
      //DOUT(5, "Received: " << frame.sprint());


      if ( i > 10 && i < 200)
	com->sendMotorVel(100, 100, 100 , MOTOR_CTR_MODE_PID);
      else 
	com->sendMotorVel(0,   0, 0 , MOTOR_CTR_MODE_PID);

      //if (i==10 ) com->sendMotorVel(0,  100, 0 , MOTOR_CTR_MODE_PID);
      //if (i==100)  com->sendMotorVel(0,  200, 0 , MOTOR_CTR_MODE_PID); 
      //if (i==200) com->sendMotorVel(0,    0, 0 , MOTOR_CTR_MODE_PID);
    }
  
   confec = com->setCyclicSendMode(CSM_NOTHING);
 
   //com->stopCyclicMessageColection();

   com->sendMotorVel(0, 0, 0 , MOTOR_CTR_MODE_PWM);

#else
   sleep(2);
#endif
  f.close();
  delete com;
}
