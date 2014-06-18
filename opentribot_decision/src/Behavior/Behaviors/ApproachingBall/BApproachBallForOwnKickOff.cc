#include "BApproachBallForOwnKickOff.h"
#include "../../../WorldModel/WorldModel.h"

namespace Tribots
{

BApproachBallForOwnKickOff::BApproachBallForOwnKickOff()
  throw()
  : Behavior("BApproachBallForOwnKickOff"), 
    skill(new SApproachParkedBall())
{}

BApproachBallForOwnKickOff::~BApproachBallForOwnKickOff()
  throw()
{
  delete skill;
}

DriveVector 
BApproachBallForOwnKickOff::getCmd(const Time& t) 
  throw(TribotsException)
{
  const BallLocation& ballLocation =
    MWM.get_ball_location(t);
  const RobotLocation& robot = MWM.get_robot_location(t);

  Vec target(-500, 288);
  Vec twodball(ballLocation.pos.x, ballLocation.pos.y);
  Vec vecBallTarget = twodball - target;
 
  skill->setParameters(target-robot.pos, 3.0);
  return skill->getCmd(t);
}

void 
BApproachBallForOwnKickOff::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
}
void 
BApproachBallForOwnKickOff::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
}

};
