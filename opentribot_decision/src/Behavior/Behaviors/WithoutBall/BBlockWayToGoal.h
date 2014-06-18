#ifndef _BBlockWayToGoal_H_
#define _BBlockWayToGoal_H_

#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../../Skills/BasicMovements/SGoToPosEvadeObstaclesOld.h"
#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Fundamental/PIDController.h"

#define SKILL SGoToPosEvadeObstaclesOld

namespace Tribots
{
  class BBlockWayToGoal : public Behavior
  {
  public:
    BBlockWayToGoal(double maxDistance, bool obeyPenaltyAreaRules=true);
    virtual ~BBlockWayToGoal() throw ();
    
    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();
    
    virtual void updateTactics (const TacticsBoard&) throw ();  
         
  protected:
    SKILL* skill;;
    PIDController headingController;
    double maxDistance;
    bool obeyPenaltyAreaRules;
    float transVel;
  };
  
}
#endif
