
#ifndef _Tribots_BLeaveGoal_h_
#define _Tribots_BLeaveGoal_h_

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"

namespace Tribots {

  /** Verhalten, das den Spieler aus dem Tor heraus faehrt und dabei Hindernisse umfaehrt */
  class BLeaveGoal : public Behavior {
  public:
    BLeaveGoal();
    ~BLeaveGoal() throw() {;}

    bool checkCommitmentCondition(const Time&) throw();
    bool checkInvocationCondition(const Time&) throw();
    DriveVector getCmd(const Time&) throw(TribotsException);

  protected:
    SPhysGotoPosAvoidObstacles goto_skill;
  };

}

#endif 
