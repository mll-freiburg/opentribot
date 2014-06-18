#ifndef _BAPPROACHBALLFROMBEHINDPOINTINGTOGOAL_H_
#define _BAPPROACHBALLFROMBEHINDPOINTINGTOGOAL_H_

#include "../../Behavior.h"
#include "../../Skills/ApproachingBall/SApproachMovingBall.h"

namespace Tribots
{
  
/**
 * Approaches the ball from behind, pointing the robots kicking device always
 * towards the opponent goal. Always applicable. 
 *  \TODO : Applicable always, when robot owns ball.
 */
class BApproachBallFromBehindPointingToGoal : public Tribots::Behavior
{
public:
	BApproachBallFromBehindPointingToGoal() throw();
	virtual ~BApproachBallFromBehindPointingToGoal() throw ();
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);  
  virtual void updateTactics (const TacticsBoard&) throw ();    
protected:
  SApproachMovingBall* skill;  
  float transVelFactor;
  bool lookAtBall;
  bool saveApproach;
};

};

#endif //_BAPPROACHBALLFROMBEHINDPOINTINGTOGOAL_H_
