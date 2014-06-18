#include "TmcCanCtr.h"
#include <sstream>
#include <iostream>
#include <math.h>

using namespace RobotCtr2;

#define TMCLOGGING

#ifdef TMCLOGGING
#define LOG( __x__ ) *logstream << __x__ << "\n"
#else
#define LOG( __x__ )
#endif

RobotCtrException::RobotCtrException( const std::string & user_message, bool _recoverable) throw()
  : message( user_message ), recoverable( _recoverable )
{
  ;
}

RobotCtrException::~RobotCtrException() throw()
{
  ;
}


TmcCanCtr_Params::TmcCanCtr_Params()
{
  for (int i=0; i<3; i++) {
    maxMotorVel[i] = 6000;
    gearFactor[i]  = 4.8;
    PID_Kp[i]      = 10;
    PID_Tn[i]      = 100;
    PID_Tv[i]      = 0;

    dir            = -1;
  }
}

void TmcCanCtr_Params::setMaxMotorVel(unsigned int v)
{
  for (int i=0; i<3; i++) {
    maxMotorVel[i] = v;
  }
}

void TmcCanCtr_Params::setGearFactor(float g)
{
  for (int i=0; i<3; i++) {
    gearFactor[i]  = fabs(g);
  }
}

void TmcCanCtr_Params::setPIDParams(unsigned int Kp, unsigned int Tn, unsigned int Tv)
{
   for (int i=0; i<3; i++) {
     PID_Kp[i] = Kp;
     PID_Tn[i] = Tn;
     PID_Tv[i] = Tv;
   }
}

std::string TmcCanCtr_Params::sprint()
{
  std::stringstream res;

  res << "Params for TmcCanCtr:"
      << "\n maxMotorVel [U/min]: ";
  for (int i=0; i<3; i++) res << "  " << maxMotorVel[i];
  res << "\n GearFactor: ";
  for (int i=0; i<3; i++) res << "  " << gearFactor[i];
  res << "\n PID Params (Kp, Tn, Tv): ";
  for (int i=0; i<3; i++) res << "(" <<  PID_Kp[i] << "," << PID_Tn[i] << "," << PID_Tv[i] << ")";

  res << "\n";
  return res.str();
}


std::string TmcDataFrame::sprint()
{
  std::stringstream s;
  
  s << "ID: " << (int)CycId 
    << " VCC: " << vcc 
    << " V: " << V[0] << " " << V[1] << "  " << V[2]
    << " C: " << C[0] << " " << C[1] << "  " << C[2]
    << " O: " << O[0] << " " << O[1] << "  " << O[2]
    << " HERR: " << HERR[0] << "  " << HERR[1] << "  " << HERR[2]
    ;
  
  return s.str();
}

void TmcDataFrame::print(std::ostream& out)
{
  out << timestamp << "  "
      << V[0] << " " << V[1] << "  " << V[2] << "  "
      << C[0] << " " << C[1] << "  " << C[2] << "  "
      << O[0] << " " << O[1] << "  " << O[2] << "  "
      << HERR[0] << "  " << HERR[1] << "  " << HERR[2];
}

//---TmcCanCtr -------------------------------------------------------------------------
TmcCanCtr::TmcCanCtr(const TmcCanCtr_Params& _ctr_params,
		     const TmcCanCom_Params& _com_params) throw ( RobotCtrException )
{
#ifdef TMCLOGGING
  logstream = new std::ofstream("tmc.log");
#endif
  ctr_params        = _ctr_params;
  std::cout << ctr_params.sprint();
  com_params        = _com_params;
  reinitCounterFlag = 0;
  com = 0;
  reinitCounter     = 0;
  init();
}

TmcCanCtr::~TmcCanCtr() throw()
{
  try {
    deinit();
  }
  catch (RobotCtrException& e) {
    if (com!=0) delete com;
  }
#ifdef TMCLOGGING
  logstream->close();
  delete logstream;
#endif
}


void TmcCanCtr::init() throw ( RobotCtrException )
{
  unsigned int confec = 0;

  if (com == 0) {
    try {
      com = new TmcCanCom( com_params );
    }
    catch (TmcCanException &e) {
      LOG("Error on init of TMC (TmcCanCom) : " << e.what());
      throw RobotCtrException(e.what(), false);
    }
  }

  confec |= com->setPIDParams(MOTOR1, ctr_params.PID_Kp[MOTOR1], ctr_params.PID_Tn[MOTOR1], ctr_params.PID_Tv[MOTOR1]);
  confec |= com->setPIDParams(MOTOR2, ctr_params.PID_Kp[MOTOR2], ctr_params.PID_Tn[MOTOR2], ctr_params.PID_Tv[MOTOR2]);
  confec |= com->setPIDParams(MOTOR3, ctr_params.PID_Kp[MOTOR3], ctr_params.PID_Tn[MOTOR3], ctr_params.PID_Tv[MOTOR3]);

  // TODO: klären warum das mit alter FW nicht geht
  //if (com->fw_mayer_version > 0 || com->fw_lower_version>3) {
  //  confec |= com->setMaxMotorVel(ctr_params.maxMotorVel[MOTOR1] , ctr_params.maxMotorVel[MOTOR2] , ctr_params.maxMotorVel[MOTOR3] );
  //}

  confec |= com->setCyclicSendMode(CSM_ALL);
  
  if (confec) {
    deinit(true);
    LOG("Error in init of tmc (TmcCanCom) : " << com->resolveCONFEC(confec));
    throw RobotCtrException( "Error in init of tmc: " + com->resolveCONFEC(confec) ,false);
  }

}

void TmcCanCtr::reinit() throw ( RobotCtrException )
{
  reinitCounter++;
  LOG("Reinit (" << reinitCounter << ") of tmc.");
  deinit(true);
  
  try {
    init();
  }
  catch (RobotCtrException &e) {
    reinitCounterFlag++;
    if (reinitCounterFlag < 100)
      throw RobotCtrException( "Reinit not successful ... still trying. " + e.what() , true );
    else {
      LOG("Reinit not successful, giving up after 100 tries.");
      throw RobotCtrException( "Reinit not successful ... stop trying. " + e.what() , false );
    }
  }
  // LOG(logf , "Successful after: " << reinitCounterFlag << " tries");
  reinitCounterFlag = 0;
}

void TmcCanCtr::check_for_reinit() throw ( RobotCtrException )
{
  if (reinitCounterFlag > 5) reinit();
}

void TmcCanCtr::deinit(bool emergency) throw ( RobotCtrException )
{
  unsigned int confec;
  if (com!=0) 
    {
      confec = com->setCyclicSendMode(CSM_NOTHING);
      
      com->sendMotorVel(0, 0, 0 , MOTOR_CTR_MODE_PWM);
      
      delete com;
      com = 0;
    }
}


void TmcCanCtr::setWheelSpeed(float v1_radps, float v2_radps, float v3_radps, 
			      unsigned char kicker1, unsigned char kicker2) throw ( RobotCtrException )
{
  check_for_reinit();

  float v_radps[3] = { ctr_params.dir * v1_radps, ctr_params.dir * v2_radps, ctr_params.dir * v3_radps };
  int   v_ipofMax[3];
  
  for (int i=0; i<3; i++) {
    float tmp =  (((float) ctr_params.maxMotorVel[i])  /  ( 60.0 * ctr_params.gearFactor[i] )) * 2 * M_PI ; // max wheel speed in rad/s
    tmp = (v_radps[i] / tmp) * 1023;
    if (tmp >  1023) tmp = 1023;
    if (tmp < -1023) tmp = -1023;
    v_ipofMax[i] = (int) tmp;
  }
  com->sendMotorVel(v_ipofMax[0] , v_ipofMax[1] , v_ipofMax[2] , MOTOR_CTR_MODE_PID, kicker1, kicker2);
}

void TmcCanCtr::setMotorPWM(float pwm1, float pwm2, float pwm3, 
			    unsigned char kicker1, unsigned char kicker2) throw ( RobotCtrException )
{
  check_for_reinit();
  float fpwm[3] = { ctr_params.dir * pwm1, ctr_params.dir * pwm2, ctr_params.dir * pwm3};
  int   ipwm[3];

  for (int i=0; i<3; i++) {
    if (fpwm[i] >  1.0) fpwm[i] =  1.0;
    if (fpwm[i] < -1.0) fpwm[i] = -1.0;

    ipwm[i] = (int) (fpwm[i] * 1023);
  }

  com->sendMotorVel(ipwm[0] , ipwm[1] , ipwm[2] , MOTOR_CTR_MODE_PWM, kicker1, kicker2);
}


unsigned int TmcCanCtr::resetPIDParams(unsigned int gain, unsigned int Tn, unsigned int Tv) {
  unsigned int confec = 0;
 
  confec |= com->setPIDParams(MOTOR1, gain , Tn, Tv );
  confec |= com->setPIDParams(MOTOR2, gain , Tn, Tv );
  confec |= com->setPIDParams(MOTOR3, gain , Tn, Tv );

  return confec;
}


TmcDataFrame TmcCanCtr::getNextData(unsigned long usec_patience) throw ( RobotCtrException )
{
  check_for_reinit();

  CanCyclicDataFrame candata;
  bool res;

  res = com->getNextCyclicFrame(candata, usec_patience);

  if (!res) {
    reinitCounterFlag++;
    // LOG(logf , "No data received.");
    throw RobotCtrException("No new Data received.", true);
  } else {
    reinitCounterFlag=0;
  }
  
  return computeTmcDataFrame(candata);
}


bool TmcCanCtr::getNewData(std::vector< TmcDataFrame >& res )
{
  check_for_reinit();
  res.clear();

  std::vector< CanCyclicDataFrame > candata;
  bool d;
  
  d = com->getNewCyclicFrames(candata);
  
  if (!d) {
    reinitCounterFlag++;
    // LOG(logf , "No data received.");
    throw RobotCtrException("No new Data received.", true);
  } else {
    reinitCounterFlag=0;
  }


  for (unsigned int i=0; i<candata.size(); i++) {
    res.push_back(computeTmcDataFrame(candata[i]));
  }
  
  return d;
}


TmcDataFrame TmcCanCtr::computeTmcDataFrame(const CanCyclicDataFrame& candata)
{
  TmcDataFrame res;

  res.CycId = candata.cycInfo.CycleId;
  res.idle  = candata.cycInfo.CycleIdle_ms;

  res.timestamp = candata.vel.CanTimeStamp;

  res.vcc   = ((1.0/25) * candata.vel.vcc) - 0.73 ;

  int imotorpofmax[3]={candata.vel.motorVel0, candata.vel.motorVel1, candata.vel.motorVel2};
  for (int i=0; i<3; i++)
    res.V[i] = ctr_params.dir * (((imotorpofmax[i] / 1023.0) * ctr_params.maxMotorVel[i]) / (60.0 * ctr_params.gearFactor [i])) * 2 * M_PI;
  
  int iC[3] = { candata.current.motorC0, candata.current.motorC1, candata.current.motorC2 };
  for (int i=0; i<3; i++)
    res.C[i] = (iC[i] / 1023.0) * 5.0;

  int iO[3]={candata.output.motorO0, candata.output.motorO1, candata.output.motorO2};
  for (int i=0; i<3; i++)
    res.O[i] = ctr_params.dir * (iO[i] / 1023.0);

  res.HERR[0] = candata.vel.herr0;
  res.HERR[1] = candata.vel.herr1;
  res.HERR[2] = candata.vel.herr2;

  return res;
}
//--- TmcCanCtr <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
