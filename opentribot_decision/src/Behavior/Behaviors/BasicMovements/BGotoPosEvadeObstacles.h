#ifndef _BGOTOPOSEVADEOBSTACLES_H_
#define _BGOTOPOSEVADEOBSTACLES_H_

#include "../../Behavior.h"
#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
//#include "../../../Fundamental/PIDController.h"

namespace Tribots {

class BGotoPosEvadeObstacles : public Behavior
{
public:
  BGotoPosEvadeObstacles(Vec targetPos, Vec targetHeading, double driveVel = 1.3);
  virtual ~BGotoPosEvadeObstacles() throw();
  virtual void updateTactics (const TacticsBoard&) throw ();  
  virtual bool checkInvocationCondition(const Time& t) throw();
  virtual bool checkCommitmentCondition(const Time& t) throw();
  virtual void cycleCallBack(const Time& t) throw();
	virtual DriveVector getCmd(const Time&) throw(TribotsException);

  
protected:
	SGoToPosEvadeObstacles* skill;
//  PIDController headingController;

	Vec targetPos;
};
  
}

#endif
