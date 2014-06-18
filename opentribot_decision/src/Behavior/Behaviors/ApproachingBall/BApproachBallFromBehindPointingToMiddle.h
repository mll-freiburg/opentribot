#ifndef _TRIBOTS_BAPPROACHBALLFROMBEHINDPOINTINGTOMIDDLE_H_
#define _TRIBOTS_BAPPROACHBALLFROMBEHINDPOINTINGTOMIDDLE_H_

#include "../../Behavior.h"
#include "../../Skills/ApproachingBall/SApproachMovingBall.h"

namespace Tribots {


/**
 * Approaches the ball using the direct (shortest) path. Points the kicking
 * device always towards the goal to goal line. Always applicable.
 * \TODO : Applicable always when robot does not owns the ball.
 */
class BApproachBallFromBehindPointingToMiddle : public Tribots::Behavior
{
public:
  BApproachBallFromBehindPointingToMiddle(Vec targetPosRelToOppGoal 
                                            = Vec(0., -4000)) throw();
  virtual ~BApproachBallFromBehindPointingToMiddle() throw();

  virtual DriveVector getCmd(const Time&) throw(TribotsException);

  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);
  virtual void updateTactics (const TacticsBoard&) throw ();  
protected:  
  SApproachMovingBall* skill; 
  Vec targetPos;
  float transVelFactor;
  bool lookAtBall;
};

}

#endif //_TRIBOTS_BAPPROACHBALLFROMBEHINDPOINTINGTOMIDDLE_H_
