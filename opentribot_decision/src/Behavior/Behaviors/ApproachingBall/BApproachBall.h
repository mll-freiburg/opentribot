
#ifndef _Tribots_BApproachBall_h_
#define _Tribots_BApproachBall_h_

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoBallAvoidObstacles.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots {

  /** Allgemeines Anfahrverhalten, IC und CC sind immer wahr, wenn die Ballposition bekannt ist (known oder communicated) */
  class BApproachBall : public Tribots::Behavior {
  public:
    BApproachBall () throw();
    ~BApproachBall () throw();
    DriveVector getCmd (const Time&) throw(TribotsException);
    void loseControl (const Time&) throw(TribotsException);
    bool checkInvocationCondition (const Time&) throw();
    bool checkCommitmentCondition (const Time&) throw();

  protected:  
    SPhysGotoBallAvoidObstacles* skill_ball;
    SPhysGotoPosAvoidObstacles* skill_pos;
    
    Area* area_approach_pointing_to_middle;
    Area* area_approach_pointing_away_own_goal;
    Area* area_approach_directly;
    Area* area_near_boundary;
    Area* area_field;

    Vec target_pointing_to_middle;
    Vec center_own_goal;
    Vec center_opponent_goal;
    
    bool ball_rolling;
    bool intercept_prefered;
  };

}

#endif
