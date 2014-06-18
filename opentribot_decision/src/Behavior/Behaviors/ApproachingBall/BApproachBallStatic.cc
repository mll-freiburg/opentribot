#include <stdexcept>
#include "../../../Structures/Journal.h"
#include "BApproachBallStatic.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <math.h>

using namespace Tribots;
using namespace std;


BApproachBallStatic::BApproachBallStatic () throw() : Behavior ("BApproachBallStatic"), skill(new SApproachMovingBall()) {
//  skill_ball = new SPhysGotoBallAvoidObstacles();
//  skill_ball->set_dynamics (2.5, 5.0);
}


BApproachBallStatic::~BApproachBallStatic () throw() {
//  delete skill_ball;
  delete skill;
}

bool BApproachBallStatic::checkInvocationCondition (const Time& texec) throw () {
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));

  if (WBOARD->doPossessBall(texec)) // target reached !
      return false;
  if (ball_exec.pos_known == BallLocation::unknown) {
    return false;
  }

  if (fabs( (robot_exec.pos-ball_exec.pos.toVec()).length()) > 1000)
  {
      LOUT << "Ball is too far away";
      return false;
  }
  Angle robot_ball = (ball_exec.pos.toVec()-robot_exec.pos).angle()-
                     (robot_exec.heading+Angle::quarter);
  Vec relBall = WBOARD->getAbs2RelFrame(texec) * ball_exec.pos.toVec();
  if (fabs(robot_ball.get_deg_180()) > 30 && 
      !(fabs(relBall.x) < 500. && relBall.y > 0 && relBall.y < 700.)) {
    LOUT << "BApproachBallStatic: not approaching, angle to big and not very close to robot:" << robot_ball.get_deg_180() << endl;
    return false;
  }
  // Es ist ok, wenn sich der ball langsam auf den Roboter zubewgt. Bei hoeheren Geschwindigkeiten muss er sich aber vom ROboter
  // entfernen (abgeprallt sein).
  if(fabs(((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180()) > 90 &&
     ball_exec.velocity.length() > .7) {
    LOUT << "BApproachBallStatic not invoced, because the ball is moving towards the robot\n";
    LOUT << "angle         : "<< fabs(((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180()) <<"\n";
    return false;
  }
  return true;
}


bool BApproachBallStatic::checkCommitmentCondition (const Time& texec) throw() {
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));  
  if (WBOARD->doPossessBall(texec)) { // target reached !
    LOUT << "BApproachBallStatic: possessing Ball. Stop.";
    return false;
  }
  
  if (ball_exec.pos_known == BallLocation::unknown) {
    LOUT << "BApproachBallStatic: ball_exec.pos_known == BallLocation::unknown. Stop.";
    return false;
  }
  
  if (fabs( (robot_exec.pos-ball_exec.pos.toVec()).length()) > 1100.)
  {
    LOUT << "BApproachBallStatic: Ball is too far away. Stop.";
    return false;
  }
  Angle robot_ball = (ball_exec.pos.toVec()-robot_exec.pos).angle()-
  (robot_exec.heading+Angle::quarter);
  Vec relBall = WBOARD->getAbs2RelFrame(texec) * ball_exec.pos.toVec();
  if (fabs(robot_ball.get_deg_180()) > 35 && 
      !(fabs(relBall.x) < 550. && relBall.y > 0 && relBall.y < 750.)) {
    LOUT << "BApproachBallStatic: not approaching, angle to big and not very close to robot:" << robot_ball.get_deg_180() << endl;
    return false;
  }
  if(fabs(((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180()) > 90 &&
     ball_exec.velocity.length() > .8) {
    LOUT << "BApproachBallStatic aborted, because the ball is moving towards the robot\n";
    LOUT << "angle         : "<< fabs(((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180()) <<"\n";
    return false;
  }
  return true;
}

DriveVector BApproachBallStatic::getCmd (const Time& t) throw() {
   //Vec ball = MWM.get_ball_location (t).pos.toVec();
  const BallLocation& ballLocation =
    MWM.get_ball_location(t);
  const RobotLocation& robotLocation =
    MWM.get_robot_location(t);
   DriveVector dv;
   dv.vrot=0;
   dv.vtrans=Vec(0,0);
   dv.kick=false;
   const double desiredVel = 1.5;
   if (MWM.get_ball_location(t).pos_known != BallLocation::raised) {
     try { 
        LOUT << "BApproachBallStatic\n";
        //Angle heading_tolerance = Angle::deg_angle (15);
        //double max_v=2.5;
        //skill_ball->init (ball, max_v, heading_tolerance);
        //dv = skill_ball->getCmdNonMovingBall (t);

        skill->setParameters(ballLocation.pos.toVec() - 
                             robotLocation.pos, 
                             desiredVel);
        dv = skill->getCmd(t);
     } catch(exception& e) {
        JERROR("exception in BApproachBallStatic: ");
        JERROR(e.what());
     }
   }
   return dv;
}

void BApproachBallStatic::gainControl (const Time& t) throw() {
   try {
     skill->gainControl(t);
   } catch(exception& e) {
      JERROR("exception in BApproachBallStatic:gainControl: ");
      JERROR(e.what());
   }
}

void BApproachBallStatic::loseControl (const Time& t) throw() {
   try {
     skill->loseControl(t);
   } catch(exception& e) {
      JERROR("exception in BApproachBallStatic:loseControl: ");
      JERROR(e.what());
   }
}

