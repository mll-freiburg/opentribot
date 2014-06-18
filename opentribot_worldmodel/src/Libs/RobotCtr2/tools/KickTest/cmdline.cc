
#include "TmcCanCtr.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace RobotCtr2;

int main(int argc, char* argv[]) {
 
  TmcCanCom_Params com_params;
  TmcCanCtr_Params ctr_params;

  ofstream file("log.dat");
  
  TmcCanCtr* ctr = 0;

  


  com_params.DevNodeStr = "/dev/pcan32";
  com_params.canbaud    = CAN_BAUD_250K;

  ctr_params.setMaxMotorVel(6000);
  ctr_params.setGearFactor(5.4);
  ctr_params.setPIDParams(15,80,0);

  try {
    ctr = new TmcCanCtr(ctr_params, com_params);
  }
  catch (RobotCtrException & e)
    {
      std::cout << "We have an ";
      if (!e.isRecoverable()) std::cout << "bad ";
      std::cout << "exception: " << e.what() << "\n";
      exit(0);
    }


  TmcDataFrame f;
  unsigned char k1;
  

  for (int i=0; i<250; i++)
    {
      try 
	{
	  f = ctr->getNextData(20000);
	}
      catch (RobotCtrException &e)
	{
	  if (e.isRecoverable()) {
	    std::cerr << "Get not data: " << e.what() << "\n";
	    continue;
	  }
	  else {
	    std::cerr << "Exiting: " << e.what() << "\n";
	    exit(0);
	  }
	}
      
      file << f.sprint() << "\n";

      if (i==100) k1= (unsigned int) atoi(argv[1]);
      else k1=0;

      ctr->setWheelSpeed( 0,  0, 0, k1, k1);
      //ctr->setMotorPWM( 0.5 , -0.5 , 0, k1, k1);

    }

  file.close();

  if (ctr != 0)
    delete ctr;
}
