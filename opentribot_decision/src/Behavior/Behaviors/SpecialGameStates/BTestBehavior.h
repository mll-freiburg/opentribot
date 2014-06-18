#ifndef _TRIBOTS_BTESTBEHAVIOR_H_
#define _TRIBOTS_BTESTBEHAVIOR_H_

#include "../../Behavior.h"
#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../ApproachingBall/BApproachBallForStandardSituation.h"
#include "../ApproachingBall/BApproachBallForOwnKickOff.h"
#include "../BasicMovements/BEmergencyStop.h"
#include "../../../Fundamental/geometry.h"
#include <vector>
#include "../../Skills/ApproachingBall/SApproachMovingBall.h"
#include "../BallHandling/BStuckOwnsBall.h"


namespace Tribots {

  class BTestBehavior : public Behavior {
  public:

    BTestBehavior();
    ~BTestBehavior() throw();

    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual bool checkInvocationCondition(const Time&) throw();
    
    virtual void gainControl(const Time& t) throw();
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
    

    protected:

  private:
    std::vector<XYRectangle> areas;
    Time start;

  };

}

#endif 
