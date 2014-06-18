#ifndef _BBlockWayToOpponentGoal_H_
#define _BBlockWayToOpponentGoal_H_

#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots
{
  class BBlockWayToOpponentGoal : public Behavior
  {
  public:
    BBlockWayToOpponentGoal();
    virtual ~BBlockWayToOpponentGoal() throw ();
    
    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual void gainControl(const Time&) throw(TribotsException);
    
    virtual void updateTactics (const TacticsBoard&) throw ();  
         
  protected:
    SGoToPosEvadeObstacles* skill;
    enum { BLOCK_LEFT=0, BLOCK_RIGHT };
    int blockPosition;
    float transVel;
  };
  
}
#endif
