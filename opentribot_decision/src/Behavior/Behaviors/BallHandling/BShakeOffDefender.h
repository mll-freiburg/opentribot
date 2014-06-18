#ifndef _Tribots_BShakeOffDefender_H_
#define _Tribots_BShakeOffDefender_H_

#include "../../Behavior.h"
#include "../../Skills/BallHandling/SDribbleBallToPosInterface.h"

namespace Tribots
{

/** 
 * Dribbelt den Ball zum Tor und weicht dabei Hindernissen aus. Neigt zu
 * groﬂz¸gigen Schlenkern (siehe auch SDribbleBallToPos).
 */
class BShakeOffDefender : public Tribots::Behavior
{
public:
	BShakeOffDefender(double transVel = 2.0);
	virtual ~BShakeOffDefender() throw();
  
  /** Roboter hat den Ball */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** Roboter hat den Ball */  
  virtual bool checkInvocationCondition(const Time&) throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);   
  virtual void updateTactics (const TacticsBoard&) throw ();  
  
  virtual void cycleCallBack(const Time& t) throw();
  
protected:
  bool activated;
  double transVel;
  SDribbleBallToPosInterface* skill;
//  bool swing;
  int swingDir;
  bool swingReverse;
  enum {SWING_LEFT, SWING_RIGHT};
  int state;
  enum {SOD_OFF, SOD_TURN, SOD_DRIVE, SOD_TURN_BACK, SOD_DONE};
  Vec tmpPos;
  Vec lastObstPos;
  bool haveObst;
  bool doNotActivateForHighSpeeds;

};

};

#endif //_Tribots_BShakeOffDefender_H_
