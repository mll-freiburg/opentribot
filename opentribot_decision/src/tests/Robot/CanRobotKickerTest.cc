#include "TmcCanCom.h"
#include <libpcan.h>
#include <iostream>
#include <sstream>
#include <fcntl.h>
#include <sys/time.h>
#include "../../Fundamental/ConfigReader.h"

using namespace std;
using namespace RobotCtr2;


TmcCanCom* com;


bool OpenTmcCanCom( TmcCanCom_Params p ) 
{
  bool res = true;
  std::string errstr;
  
  try {
    com = new TmcCanCom(p);    
  }
  catch (RobotCtr2::TmcCanException& e) {
    //std::cout << "We have an exception: " << e.what() << "\n";
    //exit(0);
    errstr = e.what();
    res = false;
  }
  
  return res;
}


bool kick( unsigned char _duration ) {
  unsigned int confec = 0;
  CanCyclicDataFrame candata;
  bool res = true;

  unsigned char duration = _duration;

  confec |= com->setCyclicSendMode(CSM_ALL);
  
  int i=0;
  do {
    com->sendMotorVel( 0 , 0 , 0 , MOTOR_CTR_MODE_PWM, duration, 0);
    if (i>2) duration = 0;
    res &= com->getNextCyclicFrame(candata, 20000);
    i++;
  } while (i<200 && res);
  
  confec |= com->setCyclicSendMode(CSM_NOTHING);
  
  com->sendMotorVel( 0 , 0, 0, MOTOR_CTR_MODE_PWM, 0, 0);
  
  return  true;
}


int main(int argc, char** argv) {
  unsigned char duration = 0;

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <duration (0-255) >\n";
    exit(1);
  }

  int dum;
  dum = atoi(argv[1]);
  
  if (dum <0) duration = 0;
  else {
    if (dum > 255) duration = 255;
    else duration = (unsigned char) dum;
  }
  
  std::cout << " ---- Testing Kicker (" << (int) duration << ") ---- \n";
  
  TmcCanCom_Params p;
  p.DevNodeStr = "/dev/pcan32";
  p.canbaud    = CAN_BAUD_250K;

  if (!OpenTmcCanCom( p )) {
    std::cerr << "Error on opening tmcCanCom\n";
    exit(1);
  }
  
  kick(duration);
  
  if (com!= 0) delete com;
}
