#include "PIDController.h"

namespace Tribots
{
  
  using namespace std;
  
  PIDController::PIDController(double kp, double ki, double kd, 
                               double maxout, double minout)
    : kp(kp), ki(ki), kd(kd), maxout(maxout), minout(minout)
  {
    reset(); 
  }

  PIDController::~PIDController()
  {}

  void 
  PIDController::reset() 
  {
    Paction    = 0;
    Iaction    = 0;
    Daction    = 0;
    lastOut    = 0;
    lastError  = 0;
  }

  double 
  PIDController::getAction(double target, double state, double deltaT_ms)
  {
    double error = state-target;
    Paction = -error * kp;

    if (lastOut<maxout && -error>0 || lastOut>minout && -error<0) {
      Iaction += ki * -error * (deltaT_ms / 1000.0);
    }

    Daction = kd * -(lastError-error) / (deltaT_ms / 1000.0);
    double out = Paction+Iaction+Daction;
  
    if (out > maxout) out = maxout;
    if (out < minout) out = minout;

    lastOut   = out;
    lastError = error;
    
    return out;
  }
  
  double PIDController::getAction(double target, double state, double deltaT_ms, ostream& debug) {
    debug << "PIDController target: " << target << " state: " << state
        << " error: " << state-target;
    double out = getAction (target, state, deltaT_ms);
    debug << " p: " << Paction << " i: " << Iaction
        << " d: " << Daction << " final output: " << out << endl;    
    return out;
  }
  
};
