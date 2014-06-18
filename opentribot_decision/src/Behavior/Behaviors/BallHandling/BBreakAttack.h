#ifndef _Tribots_BBreakAttack_H_
#define _Tribots_BBreakAttack_H_

#include "../../Behavior.h"
#include "../../Skills/BallHandling/SDribbleBallToPosInterface.h"

namespace Tribots
{

/** 
 * Dribbelt den Ball zum Tor und weicht dabei Hindernissen aus. Neigt zu
 * groﬂz¸gigen Schlenkern (siehe auch SDribbleBallToPos).
 */
class BBreakAttack : public Tribots::Behavior
{
public:
	BBreakAttack(double transVel = 2.0);
	virtual ~BBreakAttack() throw();
  
  /** Roboter hat den Ball */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** Roboter hat den Ball */  
  virtual bool checkInvocationCondition(const Time&) throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);   
  virtual void updateTactics (const TacticsBoard&) throw ();  
  
protected:
  double transVel;
  SDribbleBallToPosInterface* skill;
  bool swing;
  int swingDir;
  enum {SWING_LEFT, SWING_RIGHT};
};

};

#endif //_Tribots_BBreakAttack_H_
