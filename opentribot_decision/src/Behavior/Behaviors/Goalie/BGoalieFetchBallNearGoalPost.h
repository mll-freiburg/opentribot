
#ifndef Tribots_BGoalieFetchBallNearGoalPost_h
#define Tribots_BGoalieFetchBallNearGoalPost_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoBallCarefully.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots {

  /** Faehrt hinter den Ball, um ihn aus dem Tor herauszuholen,
   wenn der Ball in der Naehe der Torpfosten liegt, der Roboter also
   nicht vollstaendig hinter ihn fahren kann */
  class BGoalieFetchBallNearGoalPost : public Behavior {
  public:
    BGoalieFetchBallNearGoalPost () throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();

  private:
    Vec fa1, fa2;  // Grenze Fetching area bezogen auf den rechten Pfosten
    Vec pa1, pa2;
    Vec goal_post_right;
    Vec goal_post_left;
    double robot_half_width;
    SPhysGotoBallCarefully goto_ball_skill;
  };

}

#endif 
