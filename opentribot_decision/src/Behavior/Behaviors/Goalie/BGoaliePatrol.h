
#ifndef Tribots_BGoaliePatrol_h
#define Tribots_BGoaliePatrol_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots {

  /** Faehrt die Homeposition des Goalie an und laesst den Goalie sich leicht hin- und herdrehen */
  class BGoaliePatrol : public Behavior {
  public:
    /** Konstruktor, arg1: Homeposition des Torwarts, arg2: zu verwendendes Anfahrtskill */
    BGoaliePatrol (Vec, SPhysGotoPos* = NULL) throw ();
    bool checkInvocationCondition(const Time&) throw() { return true; }
    bool checkCommitmentCondition(const Time&) throw() { return true; }
    DriveVector getCmd(const Time&) throw();
    void gainControl (const Time&) throw(TribotsException);

  private:
    SPhysGotoPos own_goto_pos_skill;
    SPhysGotoPos* goto_pos_skill;
    SPhysGotoPosAvoidObstacles goto_obs_skill;
    Vec home_pos;
    Circle patrol_area;
  };

}

#endif
