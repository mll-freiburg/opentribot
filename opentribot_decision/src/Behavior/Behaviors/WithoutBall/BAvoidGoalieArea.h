#ifndef _BAvoidGoalieArea_H_
#define _BAvoidGoalieArea_H_

#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots
{
  class BAvoidGoalieArea : public Behavior
  {
  public:
    BAvoidGoalieArea();
    virtual ~BAvoidGoalieArea() throw ();
    
    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual void updateTactics (const TacticsBoard&) throw ();  
    
//    virtual void gainControl(const Time&) throw(TribotsException);
         
  protected:
    SGoToPosEvadeObstacles* skill;
    int blockPosition;
    float transVel;
  };
  
}
#endif
