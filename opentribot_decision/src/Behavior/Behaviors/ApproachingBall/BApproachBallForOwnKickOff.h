#ifndef _BAPPROACHBALLFOROWNKICKOFF_H_
#define _BAPPROACHBALLFOROWNKICKOFF_H_

#include "../../Behavior.h"
#include "../../Skills/ApproachingBall/SApproachParkedBall.h"

namespace Tribots
{
  
/**
 *  Should used to touches the ball on indirekt Standard Situations for Attacker 
 */
class BApproachBallForOwnKickOff : public Tribots::Behavior
{
public:
	BApproachBallForOwnKickOff() throw();
	virtual ~BApproachBallForOwnKickOff() throw ();
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);  
  
protected:
  SApproachParkedBall* skill;  
};

};

#endif //_BAPPROACHBALLFOROWNKICKOFF_H_
