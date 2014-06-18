#include "BApproachBallForStandardSituation.h"
#include "../../../WorldModel/WorldModel.h"

namespace Tribots
{

BApproachBallForStandardSituation::BApproachBallForStandardSituation()
  throw()
  : Behavior("BApproachBallForStandardSituation"), 
    skill(new SApproachParkedBall())
{}

BApproachBallForStandardSituation::~BApproachBallForStandardSituation()
  throw()
{
  delete skill;
}

DriveVector
BApproachBallForStandardSituation::getCmd(const Time& t, Vec target)
  throw(TribotsException){
   const RobotLocation& robot = MWM.get_robot_location(t);
  Vec targetDir =target-robot.pos;    // Zielausrichtung

  skill->setParameters(targetDir, 3.0);
  return skill->getCmd(t);
}


DriveVector 
BApproachBallForStandardSituation::getCmd(const Time& t) 
  throw(TribotsException)
{
  const BallLocation& ballLocation =
    MWM.get_ball_location(t);
  const FieldGeometry& field =
    MWM.get_field_geometry();
   const RobotLocation& robot = MWM.get_robot_location(t);
   

  
  Vec goal(0, field.field_length / 2);
  Vec twodball(ballLocation.pos.x, ballLocation.pos.y);
  Vec vecBallTor = twodball - goal;
  Vec rotate = goal + (ballLocation.pos.x > 0. ? vecBallTor.rotate_twelvth() : vecBallTor.rotate_eleven_twelvth());  //drehung um die Achse BAll Tor um ein zwölftel
  Vec targetDir =robot.pos-rotate;    // Zielausrichtung
 
  skill->setParameters(targetDir, 3.0);
  return skill->getCmd(t);
}

void 
BApproachBallForStandardSituation::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
}
void 
BApproachBallForStandardSituation::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
}

};
