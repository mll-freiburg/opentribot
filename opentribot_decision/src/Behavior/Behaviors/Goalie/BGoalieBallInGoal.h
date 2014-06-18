
#ifndef Tribots_BGoalieBallInGoal_h
#define Tribots_BGoalieBallInGoal_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"

namespace Tribots {

  /** Haelt an, wenn der Ball im eigenen Tor ist */
  class BGoalieBallInGoal : public Behavior {
  public:
    BGoalieBallInGoal () throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();

  private:
    Vec goal_post_left;
    Vec goal_post_right;
    Vec pa;
    SPhysGotoPosAvoidObstacles goto_obs_skill;
   };
}

#endif 
