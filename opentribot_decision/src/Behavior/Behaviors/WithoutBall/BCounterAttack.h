#ifndef _BCounterAttack_h_
#define _BCounterAttack_h_

#include "../../Behavior.h"
#include "../../../Fundamental/Time.h"
#include "../../Skills/BasicMovements/SBoostToPos.h"
#include "../../Skills/BasicMovements/SGoToPosEvadeObstaclesOld.h"

namespace Tribots
{
  
class BCounterAttack : public Behavior
{
public:
  BCounterAttack(int maxDuration = 2000);
  virtual ~BCounterAttack() throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual bool checkCommitmentCondition(const Time&) throw();
  virtual bool checkInvocationCondition(const Time&) throw();
  
  virtual void gainControl(const Time&) throw();
  virtual void loseControl(const Time&) throw();
	
	virtual void updateTactics(const TacticsBoard& tb) throw();
    
protected:
  Time activated;
  int maxDuration;
  double driveVel;
  SBoostToPos* dash;
  SGoToPosEvadeObstaclesOld* gotopos;
  Skill* intention;
};

  
}

#endif
