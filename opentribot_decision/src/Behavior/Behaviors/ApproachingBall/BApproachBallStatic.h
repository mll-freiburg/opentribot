
#ifndef _Tribots_BApproachBallStatic_h_
#define _Tribots_BApproachBallStatic_h_

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoBallAvoidObstacles.h"
#include "../../../Fundamental/geometry.h"
#include "../../Skills/ApproachingBall/SApproachMovingBall.h"
namespace Tribots {

  /** Allgemeines Anfahrverhalten, IC und CC sind immer wahr, wenn die Ballposition bekannt ist (known oder communicated) */
  class BApproachBallStatic : public Tribots::Behavior {
  public:
    BApproachBallStatic () throw();
    ~BApproachBallStatic () throw();
    DriveVector getCmd (const Time&) throw();
    void gainControl (const Time&) throw();
    void loseControl (const Time&) throw();
    bool checkInvocationCondition (const Time&) throw();
    bool checkCommitmentCondition (const Time&) throw();

  protected:  
    //SPhysGotoBallAvoidObstacles* skill_ball;
    SApproachMovingBall* skill;
};

}

#endif
