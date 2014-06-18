#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

#include <limits>
#include <iostream>

namespace Tribots
{
  
  /** realizes a simple linear PID-Controller that is able to control
   *  one independent value. */
  class PIDController {
  public:
    PIDController(double kp, double ki, double kd, 
                  double maxout=std::numeric_limits<double>::max(), 
                  double minout=std::numeric_limits<double>::min());
    ~PIDController();
   
    void reset();
    private:
    double getAction(double target, double state, double deltaT_ms);
    public:
    double getAction(double target, double state, double deltaT_ms, std::ostream&); // letztes Argument: debug-stream

    void set_new_parameters(double kp_,double ki_, double kd_,double max,double min){
	kp=kp_;
	ki=ki_;
   	kd=kd_;
	maxout=max;
	minout=min;

    }

  protected:
    double kp, ki, kd;
    double maxout, minout;
    double Paction, Iaction, Daction;
    double lastOut, lastError;   
  };

};

#endif //_PIDCONTROLLER_H_
