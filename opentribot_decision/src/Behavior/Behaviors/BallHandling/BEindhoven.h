#ifndef _Tribots_BEindhoven_H_
#define _Tribots_BEindhoven_H_

#include "../../Behavior.h"
#include "../../Skills/BallHandling/SDribbleBallToPosInterface.h"

namespace Tribots
{

/** 
 * Dribbelt den Ball zum Tor und weicht dabei Hindernissen aus. Neigt zu
 * groﬂz¸gigen Schlenkern (siehe auch SDribbleBallToPos).
 */
class BEindhoven : public Tribots::Behavior
{
public:
	BEindhoven(double transVel = 2.0);
	virtual ~BEindhoven() throw();
  
  /** Roboter hat den Ball */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** Roboter hat den Ball */  
  virtual bool checkInvocationCondition(const Time&) throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);   
  virtual void updateTactics (const TacticsBoard&) throw ();  
  
protected:
  bool activated;
  int cycles_on_target;
  double transVel;
  SDribbleBallToPosInterface* skill;
};

};

#endif //_Tribots_BEindhoven_H_
