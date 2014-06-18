#ifndef _BAPPROACHBALLFROMBEHINDPOINTINGAWAYOWNGOAL_H_
#define _BAPPROACHBALLFROMBEHINDPOINTINGAWAYOWNGOAL_H_

#include "../../Behavior.h"
#include "../../Skills/ApproachingBall/SApproachMovingBall.h"

namespace Tribots
{
  
/**
 * Approaches the ball from behind, pointing the robots kicking device always
 * towards the opponent goal. Always applicable. 
 *  \TODO : Applicable always, when robot owns ball.
 */
class BApproachBallFromBehindPointingAwayOwnGoal : public Tribots::Behavior
{
public:
	BApproachBallFromBehindPointingAwayOwnGoal() throw();
	virtual ~BApproachBallFromBehindPointingAwayOwnGoal() throw ();
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);
  virtual void updateTactics (const TacticsBoard&) throw ();    
protected:
  SApproachMovingBall* skill;  
  float transVelFactor;
  bool lookAtBall;
};

};

#endif //_BAPPROACHBALLFROMBEHINDPOINTINGAWAYOWNGOAL_H_
