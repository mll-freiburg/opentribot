
#ifndef Tribots_BGameStopped_h
#define Tribots_BGameStopped_h

#include "../../Behavior.h"

namespace Tribots {

  /** Haelt den Roboter beim Refereestate stopRobot an. */
  class BGameStopped : public Behavior {
  public:
    BGameStopped () throw ();
    bool checkInvocationCondition(const Time&) throw();
    bool checkCommitmentCondition(const Time&) throw();
    DriveVector getCmd(const Time&) throw();
    void gainControl(const Time&) throw();
    
    bool stopped;
    int cycle;
    double initial_speed;
  };

}

#endif
