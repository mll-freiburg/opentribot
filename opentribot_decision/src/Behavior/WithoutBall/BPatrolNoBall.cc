#include "BPatrolNoBall.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Structures/Journal.h"
#include <vector>


namespace Tribots {
  
  using namespace std;

  BPatrolNoBall::BPatrolNoBall(const Vec& pos1, const Vec& pos2)
    : Behavior("BPatrolNoBall"), patrol(new SPatrol())
  {
    patrol->setPatrolPositions(pos1, pos2);
  }

  BPatrolNoBall::~BPatrolNoBall() throw ()
  {
    delete patrol;
  }

  bool 
  BPatrolNoBall::checkCommitmentCondition(const Time& t) 
    throw()
  {
    return BPatrolNoBall::checkInvocationCondition(t);
  }

  bool 
  BPatrolNoBall::checkInvocationCondition(const Time& t) 
    throw()
  {
    const BallLocation& ball = MWM.get_ball_location(t);
    return (ball.pos_known == BallLocation::unknown);
  }

  DriveVector 
  BPatrolNoBall::getCmd(const Time& t) throw(TribotsException)
  {
    return patrol->getCmd(t);
  }
}
