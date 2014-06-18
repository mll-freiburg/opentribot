#include "TmcCanCom.h"
#include "TmcCanCtr.h"
#include <libpcan.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <fcntl.h>
#include <sys/time.h>
#include "../../Fundamental/ConfigReader.h"

using namespace std;
using namespace RobotCtr2;

#define REPORTTEST( __ok__, __out__, __test_name__, __error_desc__, __error_hint__,__fatal__) ReportTest(__ok__,__out__, __test_name__, __error_desc__, __error_hint__); if(__fatal__ && !__ok__) return false;

#define SIGN(x) (x<0?-1:1)


TmcCanCom* com;




void ReportTest(bool ok,
		std::ostream& out,
		std::string test_name,
		std::string error_desc,
		std::string error_hint)
{
  if (ok) 
    out << "(OK)    \t"; 
  else 
    out << "(FAILED)\t";

  out << test_name;

  if (!ok) {
    std::cout << "\n\t ErrDesc: " << error_desc;
    std::cout << "\n\t Hint: " << error_hint;
  }

  out << "\n" << std::flush;
}


bool TestHandle(TmcCanCom_Params p) {
  
  HANDLE h;
  
  char chbuf[strlen(p.DevNodeStr.c_str())+1];
  sprintf(chbuf, p.DevNodeStr.c_str(), strlen(p.DevNodeStr.c_str())); 
  h= LINUX_CAN_Open ( chbuf, O_RDWR );

  REPORTTEST(h,
	     std::cout, 
	     "Opening can handle [" + p.DevNodeStr + "]",
	     "\n\t- unable to get a pcan handle",
	     "\n\t- wrong device node; \n\t- pcan kernel module not loaded (modprobe pcan); \n\t- device not present; \n\t- device not readable;", true);
  
  int err;
  err = CAN_Close(h);

  REPORTTEST(!err,
	     std::cout, 
	     "Closing can handle [" + p.DevNodeStr + "]",
	     "\n\tUnknown.",
	     "\n\tUnknown.", true);
  
  return true;
}


bool TestOpenTmcCanCom( TmcCanCom_Params p ) 
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
  REPORTTEST(res,
	     std::cout, 
	     "Opening TmcCanCom",
	     "\n\t." + errstr,
	     "\n\t- Tmc not connected;\n\t- Tmc not switched on;\n\t- Wrong device node; \n\t- ...", true);

  std::cout << "TmcCanCom fw: " 
	    << (int) com->fw_mayer_version << "." 
	    << (int) com->fw_lower_version << "\n";

  return res;
}


bool TestPing() 
{
  bool res=true;
  std::string version;

  if (com==0) std::cerr << "Null pointer!\n";

  res = com->ping(&version);

  REPORTTEST(res,
 	     std::cout, 
 	     "Pinging [" + version + "]",
 	     "\n\t Unknown",
 	     "\n\t Unknown", true);
  
  return res;
}

bool  TestRoundtripTime()
{
  bool res = true;
  int  i   = 0;

  timeval t1, t2;

  gettimeofday(&t1, 0);
  do {
    res = com->ping();
    i++;
  } while (res == true && i < 100);
  gettimeofday(&t2, 0);
  
  double diff = (t2.tv_sec - t1.tv_sec) * 1000.0 +  (t2.tv_usec - t1.tv_usec) / 1000.0;

  std::stringstream msg;
  msg << "Stresstest average time for ping " << diff/100.0 << " ms";

  REPORTTEST(res,
 	     std::cout, 
 	     "Roundtrip time measurement: " + msg.str(),
 	     "\n\t Unknown",
 	     "\n\t Unknown", true);
  
  return res;
}



bool  TestMotorPolarity(int motorid)
{
  unsigned int confec = 0;
  CanCyclicDataFrame candata;
  bool res = true;

  if (motorid < 0 || motorid > 2) {
    std::cerr << "# test called with wrong parameter: motorid (0,1,2): " << motorid << "\n";
  }

  int motorPWM[3];
  int motorVel[3];

  for (int i=0; i<3; i++) {
    if (i==motorid) motorPWM[i]=300;
    else motorPWM[i] = 0;
  }

  confec |= com->setCyclicSendMode(CSM_ALL);
  
  int i=0;
  do {
    com->sendMotorVel( motorPWM[0] , motorPWM[1], motorPWM[2], MOTOR_CTR_MODE_PWM, 0, 0);
    res &= com->getNextCyclicFrame(candata, 20000);
    i++;
  } while (i<100 && res);
  
  confec |= com->setCyclicSendMode(CSM_NOTHING);
  
  com->sendMotorVel( 0 , 0, 0, MOTOR_CTR_MODE_PWM, 0, 0);
  
  motorVel[0] = candata.vel.motorVel0;
  motorVel[1] = candata.vel.motorVel1;
  motorVel[2] = candata.vel.motorVel2;
 
  bool turncheck = (abs(motorVel[motorid]) > 20);

  bool polaritycheck = SIGN(motorVel[motorid]) == SIGN(motorPWM[motorid]);

  std::stringstream msg;
  msg << "Motor [" << motorid << "] turncheck/polarity check:";

  // communication ok?
  if (!((confec==0) && res)) {
    REPORTTEST( (confec==0) && res,
		std::cout, 
		msg.str(),
		"\n\t " + com->resolveCONFEC(confec),
		"\n\t Unknown", true);
  } else {
    if (!turncheck) {
      REPORTTEST( turncheck,
		  std::cout, 
		  msg.str(),
		  "\n\t The motor is not turning!!!",
		  "\n\t- Encoders not connected properly;\n\t- Motors not turning;", true);
    } else {
      REPORTTEST( polaritycheck,
		  std::cout,
		  msg.str(),
		  "\n\t The motor is not correct wired to the tmc!!!",
		  "\n\t Switch the wires at the motor port of tmc.", true);
    }
  }
  
  
  return  true;
}


bool TestKicker( int _duration , int _kickerid = 1) {
  unsigned int confec = 0;
  CanCyclicDataFrame candata;
  bool res = true;
  int kickerid;

  if (_kickerid < 1 || _kickerid > 2) {
    std::cerr << "# test called with wrong parameter: kicker (1,2): " << _kickerid << "\n";
  }

  kickerid = _kickerid -1;
  int duration[2];

  for (int i=0; i<2; i++) {
    if (i==kickerid) duration[i]=_duration;
    else duration[i] = 0;
  }

  confec |= com->setCyclicSendMode(CSM_ALL);
  
  int i=0;
  do {
    com->sendMotorVel( 0 , 0 , 0 , MOTOR_CTR_MODE_PWM, duration[0], duration[1]);
    res &= com->getNextCyclicFrame(candata, 20000);
    i++;
  } while (i<200 && res);
  
  confec |= com->setCyclicSendMode(CSM_NOTHING);
  
  com->sendMotorVel( 0 , 0, 0, MOTOR_CTR_MODE_PWM, 0, 0);
 
  std::stringstream msg;
  msg << "Kicker [" << _kickerid << "] test:";

  // communication ok?
  REPORTTEST( (confec==0) && res,
	      std::cout, 
	      msg.str(),
	      "\n\t " + com->resolveCONFEC(confec),
	      "\n\t Unknown", true);
  
  return  true;
}


bool TestPIDControl (TmcCanCtr* ctr) {
  bool res;
  std::vector< RobotCtr2::TmcDataFrame > newdat;

  std::cout << "PIDControl test, file PIDTest.prot will be written\n";

  std::ofstream prot("PIDTest.prot");

  float target[3]={0,0,0};

  //ctr->setMotorPWM(target[0], target[1], target[2] , 0 );
  ctr->setWheelSpeed(target[0], target[1], target[2] , 0 , 0 ); 
  
  try {
    for (int i=0; i<500; i++) {
      usleep(25000);
      newdat.clear();
      ctr->getNewData(newdat);
      
      ctr->setWheelSpeed(target[0], target[1], target[2] , 0 , 0);
      //ctr->setMotorPWM(target[0], target[1], target[2] , 0 );
      
      if (i==20) { target[0] = 0; target[1] = 30.0; target[2] = -30.0; }
      
      for (unsigned int h=0; h<newdat.size(); h++) {
	prot << i 
	     << " " << newdat[h].idle 	
	     << " " << target[0] << " " << target[1] << " " << target[2]
	     << " " << newdat[h].V[0] << " " << newdat[h].V[1] << " " << newdat[h].V[2] 
	     << " " << newdat[h].O[0] << " " << newdat[h].O[1] << " " << newdat[h].O[2] 
	     << " " << newdat[h].C[0] << " " << newdat[h].C[1] << " " << newdat[h].C[2]
	     << "\n"; 
      } 
    }
  }  
  catch (RobotCtr2::RobotCtrException & e) {
    res = false;
    std::cerr << "Test failed : " << e.what() << "\n";
  }
  prot.close();
  return res;
}


bool YesNoQuestion (std::string question, std::string explanation)
{
  char c=' ';
  std::cout << "# " << explanation << "\n" << "# " <<question << " [Y,N] : ";
  do {
    std::cin >> c;
  } while (!(c=='n' || c=='N' || c=='y' || c=='Y'));
  return (c == 'y' || c == 'Y');
}


int main(int argc, char** argv) {
  
  std::cout << " ---- Testing CanRobot Hardware ---- \n";

  TmcCanCom_Params p;
  p.DevNodeStr = "/dev/pcan32";
  p.canbaud    = CAN_BAUD_1M;
  TmcCanCtr_Params p_ctr;
  p_ctr.setMaxMotorVel(6000);
  p_ctr.setGearFactor(5.4);
  p_ctr.setPIDParams(15,30,0);

  // read in config files for device node path
  if (argc<2) {
    std::cout << "# No config file given using standards.\n";
  } else {
    std::cout << "# Reading config file: " << argv[1] << "\n";
    Tribots::ConfigReader cr;
    string sdum;
    double ddum;
    int    idum;
    std::vector< int > vidum;

    cr.append_from_file(argv[1], true);
    
    if (!cr.get("RobotCommunication::device", p.DevNodeStr)) {
      std::cerr << "# Can't read [RobotCommunication::device] from config file [" << argv[1] << "]\n";
    }

    if (cr.get("RobotHardware::MaxMotorVel", idum)) p_ctr.setMaxMotorVel(idum);
    if (cr.get("RobotHardware::GearFactor", ddum)) p_ctr.setGearFactor(ddum);
    if (cr.get("RobotHardware::PIDFactors", vidum)) {
      if (vidum.size() != 3) std::cerr << "!!! Wrong number of paprams for pid\n";
      else p_ctr.setPIDParams(vidum[0],vidum[1],vidum[2]); 
    }
  }


  // Pretest of basic configuration (port, devices, readable, ... )
  if (!TestHandle(p)) exit(0);

  // Ok, now open a TmcCanCom instance for further testing
  if (!TestOpenTmcCanCom(p)) exit(0);
  
  // At this stage we have a connection to a switched on tmc with can software 
  // so do some more testing
  TestPing();
  TestRoundtripTime();

  if (YesNoQuestion ("Do you want to test Motor Polarity?", "Attention: motors will move!!"))
    {
      TestMotorPolarity(0);
      TestMotorPolarity(1);
      TestMotorPolarity(2);
    }
  
  

  if( YesNoQuestion("Do you want to test kicker?", "Attention!! HANDS OFF KICKER, care for your hands and face !"))
    {
      TestKicker( 100 );
    }

  if (com!= 0) delete com;


  if (YesNoQuestion("Do you want to test high level robot things (TmcCanCtr)?","")) {
    TmcCanCtr* ctr;
    
    try {
       ctr = new TmcCanCtr(p_ctr, p);
    }
    catch (RobotCtr2::RobotCtrException & e) {
      REPORTTEST( 0 ,
		std::cout, 
		"Init Robot :",
		"\n\t " + e.what(),
		"\n\t Unknown", true);
    }
    REPORTTEST( 1 ,
		std::cout, 
		"Init Robot :",
		"\n\t " ,
		"\n\t Unknown", true);
    
    
    TestPIDControl(ctr);
    

    if (ctr!=0) delete ctr;
  }
  
}
