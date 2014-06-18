#ifndef _BBlockWayToMiddle_H_
#define _BBlockWayToMiddle_H_

#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots
{
  class BBlockWayToMiddle : public Behavior
  {
  public:
    BBlockWayToMiddle();
    virtual ~BBlockWayToMiddle() throw ();
    
    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual void gainControl(const Time&) throw(TribotsException);
    
    virtual void updateTactics (const TacticsBoard&) throw ();  
    
    void cycleCallBack(const Time& t) throw();
         
  protected:
    SGoToPosEvadeObstacles* skill;
    enum { BLOCK_LEFT=0, BLOCK_RIGHT };
    int blockPosition;
    float transVel;
    Time lastTimeBefreiungsschlag;
  };
  
}
#endif
