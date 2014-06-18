#ifndef _TMC_CAN_CTR_H_
#define _TMC_CAN_CTR_H_

#include <exception>
#include <string>
#include <fstream>

#include "TmcCanCom.h"

#define TMCLOGGING

namespace RobotCtr2 {

  class RobotCtrException : public std::exception {
  public:
    RobotCtrException( const std::string & user_message, bool _recoverable = false)   throw();
    virtual ~RobotCtrException() throw();
    
    const std::string& what()          throw() {return message;};
    bool               isRecoverable() throw() {return recoverable;};
    
  protected:
    bool        recoverable;
    std::string message;  
  };
  

  struct TmcCanCtr_Params {
    TmcCanCtr_Params();
    
    void setMaxMotorVel(unsigned int v);
    void setGearFactor(float g);
    void setPIDParams(unsigned int Kp, unsigned int Tn, unsigned int Tv);

    std::string sprint();
    
    unsigned int PID_Kp[3];
    unsigned int PID_Tn[3];
    unsigned int PID_Tv[3];

    unsigned int maxMotorVel[3]; // [U / min]
    float        gearFactor [3];

    int          dir; // direction reverse -1 or 1
  };

  struct TmcDataFrame {

    unsigned char CycId;
    int   idle; // idle time [ms] in TMC negative values are bad    
    float V[3]; // velocity of wheels [rad/sec]
    float O[3]; // %pwm of motor output [-1,1]
    float C[3]; // motor current [A]
    bool  HERR[3]; // Motor H bridge error (0 OK, 1 ERR)

    float vcc;

    long  timestamp;

    std::string sprint();
    void        print(std::ostream& out);
  };
  

  class TmcCanCtr {
  public:
    TmcCanCtr(const TmcCanCtr_Params& _ctr_params = TmcCanCtr_Params(),
	      const TmcCanCom_Params& _com_params = TmcCanCom_Params() ) throw ( RobotCtrException );
    ~TmcCanCtr() throw();

    // set the motor speed in rad/sec (PID Control)
    void setWheelSpeed(float v1_radps, float v2_radps, float v3_radps, 
		       unsigned char kicker1=0, unsigned char kicker2=0) throw ( RobotCtrException );

    // set the motor voltage output in % of vcc (pwm duty cycle) [-1.0, 1.0]
    void setMotorPWM(float pwm1, float pwm2, float pwm3,
		     unsigned char kicker1=0, unsigned char kicker2=0) throw ( RobotCtrException );


    // reset the pid params for all motors
    unsigned int resetPIDParams(unsigned int gain, unsigned int Tn, unsigned int Tv);

    // get the data from tmc
    TmcDataFrame getNextData(unsigned long usec_patience = 1000) throw ( RobotCtrException );

    // get all data from tmc that was received till the last call 
    // (caution: underlying cyclic buffer can have a overrun if called with low frequency!!)
    bool getNewData(std::vector< TmcDataFrame >& res );

    void get_fw_version(unsigned char& h, unsigned char& l) {
      if (com==0) return;
      h = com->fw_mayer_version;
      l = com->fw_lower_version;
    };

  protected:
    void init() throw ( RobotCtrException );
    void deinit(bool emergency = false) throw ( RobotCtrException );
    void reinit() throw ( RobotCtrException );
    void check_for_reinit() throw ( RobotCtrException );
    
    TmcDataFrame computeTmcDataFrame(const CanCyclicDataFrame& candata);

    TmcCanCtr_Params ctr_params;
    TmcCanCom_Params com_params;
    
    int reinitCounterFlag; // if this flag is set we need a reinit because some communication was errornous
    long reinitCounter;

    TmcCanCom *com;

#ifdef TMCLOGGING
    std::ofstream* logstream;
#endif
  };

}
#endif
