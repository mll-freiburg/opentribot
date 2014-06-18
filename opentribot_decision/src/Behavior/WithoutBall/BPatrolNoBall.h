#ifndef _TRIBOTS_BPATROLNOBALL_H_
#define _TRIBOTS_BPATROLNOBALL_H_

#include "../../Behavior.h"
#include "../../Skills/WithoutBall/SPatrol.h"

namespace Tribots {

  class BPatrolNoBall : public Behavior {
  public:

    /**
     * Constructor of BPatrolNoBall. Expects as arguments two points the robot
     * should use as its waypoints of its patrol movement.
     */
    BPatrolNoBall(const Vec& pos1, const Vec& pos2);
    ~BPatrolNoBall() throw();

    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual bool checkInvocationCondition(const Time&) throw();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);

  protected:
    SPatrol* patrol;
  };

}

#endif 
