#ifndef _TRIBOTS_BAPPROACHBALLDIRECTLY_H_
#define _TRIBOTS_BAPPROACHBALLDIRECTLY_H_

#include "../../Behavior.h"
#include "../../Skills/ApproachingBall/SApproachMovingBall.h"

namespace Tribots {


/**
 * Approaches the ball using the direct (shortest) path. Points the kicking
 * device always towards the ball. Always applicable.
 * \TODO : Applicable always when robot owns the ball.
 */
class BApproachBallDirectly : public Tribots::Behavior
{
public:
  BApproachBallDirectly(const std::string& name = "BApproachBallDirectly") throw();
  virtual ~BApproachBallDirectly() throw();
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);
  virtual bool checkInvocationCondition (const Time& t) throw();
  virtual bool checkCommitmentCondition (const Time& t) throw();
  virtual void updateTactics (const TacticsBoard&) throw ();
protected:
  SApproachMovingBall* skill;
  float transVelFactor;
};

}

#endif //_TRIBOTS_BAPPROACHBALLDIRECTLY_H_
