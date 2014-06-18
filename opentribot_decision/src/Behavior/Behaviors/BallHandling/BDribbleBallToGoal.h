#ifndef _Tribots_BDribbleBallToGoal_H_
#define _Tribots_BDribbleBallToGoal_H_

#include "../../Behavior.h"
#include "../../Skills/BallHandling/SDribbleBallToPosInterface.h"

namespace Tribots
{

/** 
 * Dribbelt den Ball zum Tor und weicht dabei Hindernissen aus. Neigt zu
 * großzügigen Schlenkern (siehe auch SDribbleBallToPos).
 */
class BDribbleBallToGoal : public Tribots::Behavior
{
public:
	BDribbleBallToGoal(double transVel = 2.0);
	virtual ~BDribbleBallToGoal() throw();
  
  /** Roboter hat den Ball */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** Roboter hat den Ball */  
  virtual bool checkInvocationCondition(const Time&) throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);   
  virtual void updateTactics (const TacticsBoard&) throw ();  
  
protected:
  bool doDribbleToMiddleFirst;
  int state;
  double transVel;
  SDribbleBallToPosInterface* skill;
  bool swing;
  int swingDir;
  enum {SWING_LEFT, SWING_RIGHT};
};

};

#endif //_Tribots_BDribbleBallToGoal_H_
