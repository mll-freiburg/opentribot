#include "BOpposeBall.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <cmath>

using namespace Tribots;
using namespace std;

const float MIN_TURN_DIST = 900.0f;


BOpposeBall::BOpposeBall (bool hearToSignal) throw () 
  : Behavior ("BOpposeBall"), 
    headingController(4., 0.0000 , 0.00, 1 , -1),
    hearToSignal(hearToSignal)
{
}

bool BOpposeBall::checkInvocationCondition (const Time& texec) throw () {
   const BallLocation& ball_exec (MWM.get_ball_location (texec));
  
  WBOARD->checkMessageBoard();
  string msg = MWM.get_message_board().scan_for_prefix("pass:");
  bool rp = false;
  if (msg != "") {
    istringstream str(msg);
    string tmp;
    unsigned int number;
    str >> tmp >> number;
    if (str && MWM.get_robot_id() == number) {              // passempfaenger wurde kommuniziert
      LOUT << "Auch im BOpposeBall gefunden." << endl;
      rp = true;
    }
  }
  // LOUT << "BallPassingReceiver: BOppose: check inv cond - rp=" << rp << endl;
  
  if ( !WBOARD->detectPassedBall(texec) && (hearToSignal && !(WBOARD->receivePass() || rp)) ) {
    return false;
  }
  LOUT << "BallPassingReceiver: pass for me!" << endl;
  
   if (WBOARD->doPossessBall(texec)) {
      LOUT << "BOpposeBall: possessing ball -> no oppose\n";
	   return false;
   }
   if (ball_exec.pos_known == BallLocation::unknown)
   {
      LOUT << "BOpposeBall: ball unknown -> no oppose\n";
	   return false;
   }
   if (hearToSignal && MWM.get_message_board().scan_for_prefix ("passbreak").length()>0) {
    return false;
   }
   return true;
}
void BOpposeBall::gainControl(const Time& t) throw(TribotsException)
{
  tActivated.update();
}



bool BOpposeBall::checkCommitmentCondition (const Time& texec) throw () {
  const BallLocation& ball_exec = MWM.get_ball_location(texec);
  const RobotLocation& robot_exec = MWM.get_robot_location(texec);

  if (WBOARD->doPossessBall(texec)) {
    LOUT << "BOpposeBall: possessing ball -> no oppose\n";
    return false;
  }
  if (ball_exec.pos_known == BallLocation::unknown)
  {
    LOUT << "BOpposeBall: ball unknown -> no oppose\n";
    return false;
  }
  if(fabs((robot_exec.pos-ball_exec.pos.toVec()).length())<MIN_TURN_DIST) {
    LOUT << "BOpposeBall: not turning, since distance to the ball below MIN_TURN_DIST.\n ";
    return false;
  }
  if (hearToSignal && MWM.get_message_board().scan_for_prefix ("passbreak").length()>0) {
    LOUT << "BOpposeBallheard passbreak" << endl;
    return false;
  }
  if (tActivated.elapsed_sec() >= 3) {
    if ( (fabs( (robot_exec.pos-ball_exec.pos.toVec()).angle(ball_exec.velocity.toVec()).get_deg_180())
          > 90) 
        && (ball_exec.velocity.toVec().length() > .7))
    {
      LOUT << "BOpposeBall aborted, because the ball moves away from us\n";
      return false;
    }
  }
  return true;
}


DriveVector BOpposeBall::getCmd(const Time& texec) throw () {
	DriveVector dest;
	const RobotLocation& robot_exec (MWM.get_robot_location (texec));
	const BallLocation& ball_exec (MWM.get_ball_location (texec));

	//dest.vtrans = robot_exec.vtrans; // do not change current movement
	dest.vtrans = Vec(0,0);
	dest.vrot = 0.;

	double hdiff=(robot_exec.heading - ((ball_exec.pos.toVec() - robot_exec.pos).angle() - Angle::quarter)).get_rad_pi()/M_PI;

	//test different speeds
	double max_rotate_speed=5.5;

	PiecewiseLinearFunction plf;
	plf.addVertex(0.1, 0.2);	//test on the robot :erst steil, dann flach
	plf.addVertex(0.3, 0.4);      // schneller Anstieg
	plf.addVertex(1.0, 1.0);      // flacherer Anstieg
	dest.vrot = -(fabs(hdiff)/hdiff)*max_rotate_speed*plf.getValue(fabs(hdiff));

	dest.kick=false; // since the robot does not have the ball yet, kicking makes no sense
	return dest;
}


