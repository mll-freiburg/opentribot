#ifndef _Tribots_BRetreatDribble_H_
#define _Tribots_BRetreatDribble_H_

#include "../../Behavior.h"
#include "../../Skills/BallHandling/SDribbleBallToPosInterface.h"
#include <vector>

namespace Tribots
{

/** 
 * Dribbelt den Ball zum Tor und weicht dabei Hindernissen aus. Neigt zu
 * groﬂz¸gigen Schlenkern (siehe auch SDribbleBallToPos).
 */
class BRetreatDribble : public Tribots::Behavior
{
public:
	BRetreatDribble(double transVel = 2.0);
	virtual ~BRetreatDribble() throw();
  
  /** Roboter hat den Ball */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** Roboter hat den Ball */  
  virtual bool checkInvocationCondition(const Time&) throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);   
  virtual void updateTactics (const TacticsBoard&) throw ();  
  virtual void cycleCallBack(const Time& t) throw ();
  
protected:
  bool activationLevel;
  enum { ON, ONLY_IN_PENALTY_AREA, OFF };
  double transVel;
  SDribbleBallToPosInterface* skill;
  bool swing;
  int swingDir;
  enum {SWING_LEFT, SWING_RIGHT};
  
  std::vector<Vec> positions;
  Vec target;
  Vec startPos;
  
  bool reverseTurn;
  
  bool mayBecomeActive;
  bool m_bDecisionMade;
  int m_iLostBallLoops;
  
};

};

#endif //_Tribots_BRetreatDribble_H_
