#ifndef _BPass_h_
#define _BPass_h_

/*
 *  BPass.h
 *  robotcontrol
 */

#include "../../Behavior.h"
#include "../../../Fundamental/Time.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"

namespace Tribots
{
  
class BPass : public Behavior
{
public:
  BPass(double passProbability = 1.);
  virtual ~BPass() throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual bool checkCommitmentCondition(const Time&) throw();
  virtual bool checkInvocationCondition(const Time&) throw();
	
	virtual void updateTactics(const TacticsBoard& tb) throw();
  
  virtual void cycleCallBack(const Time&) throw();
  virtual void loseControl(const Time&) throw(TribotsException);
  
protected:
  Vec target;
	//** robot position on activation of behavior */
	Vec startPos;
	//** id of the target robot */
  unsigned int targetId;
  bool kicked;
	double passProbability;

	SDribbleBallToPos* skill;

};

  
}

#endif
