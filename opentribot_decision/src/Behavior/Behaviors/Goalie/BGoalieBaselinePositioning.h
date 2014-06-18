
#ifndef Tribots_BGoalieBaselinePositioning_h
#define Tribots_BGoalieBaselinePositioning_h

#include "../../../Behavior/Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"

namespace Tribots {

  class BGoalieBaselinePositioning : public Behavior {
  public:

    BGoalieBaselinePositioning (bool) throw ();
    ~BGoalieBaselinePositioning () throw ();
    void gainControl (const Time&) throw (TribotsException);
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();

  private:
    Arc positioning_arc;
    Vec left_end, right_end;
    Vec pa1, pa2;  // Grenzen des Strafraums als Arbeitsbereich des Goalies fuer dieses Verhalten
    Vec goal_post_right;
    Vec goal_post_left;
    Angle max_angle;
    bool use_comm_ball;
    SPhysGotoPos* skill;
    Time timeout;
  };
}

#endif 
