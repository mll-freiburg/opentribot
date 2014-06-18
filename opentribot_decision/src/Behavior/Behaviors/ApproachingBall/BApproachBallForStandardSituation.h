#ifndef _BAPPROACHBALLFORSTANDARDSITUATION_H_
#define _BAPPROACHBALLFORSTANDARDSITUATION_H_

#include "../../Behavior.h"
#include "../../Skills/ApproachingBall/SApproachParkedBall.h"

namespace Tribots
{
  
/**
 *  Should used to touches the ball on indirekt Standard Situations for Attacker 
 */
class BApproachBallForStandardSituation : public Tribots::Behavior
{
public:
	BApproachBallForStandardSituation() throw();
	virtual ~BApproachBallForStandardSituation() throw ();
  virtual DriveVector getCmd(const Time&, Vec target) throw(TribotsException);
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);  
  
protected:
  SApproachParkedBall* skill;  
};

};

#endif //_BAPPROACHBALLFORSTANDARDSITUATION_H_
