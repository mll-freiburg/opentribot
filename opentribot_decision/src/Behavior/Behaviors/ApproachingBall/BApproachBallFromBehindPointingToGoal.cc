#include "BApproachBallFromBehindPointingToGoal.h"
#include "../../../WorldModel/WorldModel.h"

using namespace std;
namespace Tribots
{

BApproachBallFromBehindPointingToGoal::BApproachBallFromBehindPointingToGoal()
  throw()
  : Behavior("BApproachBallFromBehindPointingToGoal"), 
    skill(new SApproachMovingBall()), lookAtBall(true), saveApproach(true)
{
  transVelFactor=1.0;
}

BApproachBallFromBehindPointingToGoal::~BApproachBallFromBehindPointingToGoal()
  throw()
{
  delete skill;
}

void BApproachBallFromBehindPointingToGoal::updateTactics (const TacticsBoard& tb) 
    throw () 
{
  // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
        transVelFactor=0.8;
  } else	  
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
       transVelFactor=0.5;
  } else
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
       transVelFactor=1.3;
  } else { // =normal
       transVelFactor=1.0;
  }
  if (tb[string("3DFeldspielerVerhalten")] == "aus") {
    lookAtBall = false;
  } else {
    lookAtBall = true;
  }
  if (tb[string("KampfUmBall")] == "BallFreispielen") {
    saveApproach = false;
  } else {
    saveApproach = true;
  }
}


DriveVector 
BApproachBallFromBehindPointingToGoal::getCmd(const Time& t) 
  throw(TribotsException)
{
  const BallLocation& ballLocation =
    MWM.get_ball_location(t);
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& field =
    MWM.get_field_geometry();

  Vec oppGoalPos(0., 500+field.field_length / 2.);
  Vec targetDir = oppGoalPos - ballLocation.pos.toVec();    // Zielausrichtung
  if (ballLocation.pos.y < field.field_length/6. &&
      saveApproach) { // Ball in eigener Haelfte-> Parallel zur Seitenlinie, um nicht die Linie aufzumachen
    targetDir = Vec(0.,1.); // Einfach parallel zur Seitenlinie anfahren
  }
  Vec targetHeading = lookAtBall ? (ballLocation.pos.toVec()-robot.pos).normalize() : targetDir;
 
  skill->setParameters(targetDir, targetHeading, 2.2*transVelFactor);
  return skill->getCmd(t);
}

void 
BApproachBallFromBehindPointingToGoal::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
}
void 
BApproachBallFromBehindPointingToGoal::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
}

};
