
#include "BGoalieBaselinePositioning.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoalieBaselinePositioning::BGoalieBaselinePositioning (bool cb) throw () : Behavior ("BGoalieBaselinePositioning") {
  skill = new SPhysGotoPos();

  use_comm_ball = cb;
  const FieldGeometry& fg (MWM.get_field_geometry());
  goal_post_right.x=0.5*fg.goal_width;
  goal_post_right.y=-0.5*fg.field_length;
  goal_post_left.x=-0.5*fg.goal_width;
  goal_post_left.y=-0.5*fg.field_length;
  pa1.x = -0.5*fg.penalty_area_width;
  pa1.y = -0.5*fg.field_length-200;
  pa2.x = -pa1.x;
  pa2.y = -0.5*fg.field_length+fg.penalty_area_length;

  timeout.update();
}

BGoalieBaselinePositioning::~BGoalieBaselinePositioning () throw () {
  if (skill) {
    delete skill;
  }
}

void BGoalieBaselinePositioning::gainControl (const Time&) throw(TribotsException) {
  timeout.update();
  skill->set_dynamics (2.0, 2.0, 5.0, 8.0);
}

bool BGoalieBaselinePositioning::checkInvocationCondition (const Time& texec) throw () {
  const RobotLocation& robot (MWM.get_robot_location (texec));
  const BallLocation& ball (MWM.get_ball_location (texec));
  
  if (ball.pos_known != BallLocation::known) {
    return false;
  }
  
  if (!(pa1.x<=robot.pos.x && pa1.y<=robot.pos.y && robot.pos.x<=pa2.x && robot.pos.y<=pa2.y)) {
    return false;
  }


  if (ball.velocity.toVec().length() < 0.5) {
    return false;
  }


  if (!ball.velocity.toVec().angle().in_between (Angle::five_eighth, Angle::seven_eighth)) {
    return false;
  }
  return true;
}

bool BGoalieBaselinePositioning::checkCommitmentCondition (const Time& texec) throw () {
  //wait for some time
  if (timeout.elapsed_sec() > 10) {
      LOUT << "BGoalieBaselinePositioning: timeout" << endl;
      return false;
  }
  
//  return true;
  return checkInvocationCondition(texec);
}

DriveVector BGoalieBaselinePositioning::getCmd(const Time& texec) throw () {
  DriveVector dest;
  const RobotLocation& robot (MWM.get_robot_location (texec));
  BallLocation ball (MWM.get_ball_location (texec));
  const FieldGeometry& fg (MWM.get_field_geometry());
  
  if (ball.velocity.y==0) {
    ball.velocity.y += 1;
  }


  Line dyna_pos_line(ball.pos.toVec(), ball.pos.toVec()+ball.velocity.toVec());
  Line goal_line(goal_post_left, goal_post_right);

  dest.vtrans = Vec(0.,0.);
  dest.vrot = 0.;
  
  Vec inter;  
  try{
    inter = intersect(goal_line, dyna_pos_line);
  } catch(invalid_argument&) {
    // rollt parallel zur Torauslinie
    inter = Vec(0,-fg.field_length/2);
  }
  
  int baseline = -fg.field_length/2;
  int dontTouchDistance = 300;

  //erstmal mitte annehmen
  Vec target = Vec(0, baseline);
  //mšglicherweise nach links
  if (inter.x < (goal_post_left.x/3)) {
    target = Vec(goal_post_left.x/1.5 + dontTouchDistance, baseline);
    LOUT << "BGoalieBaselinePositioning: left position: " << target.x << ", " << target.y << endl;
  }
  //oder rechts
  if (inter.x > (goal_post_right.x/3)) {
    target = Vec(goal_post_right.x/1.5 - dontTouchDistance, baseline);
    LOUT << "BGoalieBaselinePositioning: right position: " << target.x << ", " << target.y << endl;
  }

  //ball fliegt auf mich zu
  if (ball.velocity.y < 0.) {
    LOUT << "BGoalieBaselinePositioning: ball kommt" << endl;
    //close to target, drive normally and correct orientation
    if (fabs(target.x - robot.pos.x) < 100 ) {
      LOUT << "BGoalieBaselinePositioning: bin schon nahe am ziel" << endl;
      skill->init (target, Vec(0.0, 1.0), true);
      dest = skill->getCmd (texec);
    
    } else {
      LOUT << "BGoalieBaselinePositioning: weit vom ziel weg" << endl;
      //far from target, rush
      bool left = target.x < robot.pos.x;
      /*
      dest.vtrans = 
        left ? Vec(-2.5, 0.) * -robot.heading :
        Vec( 2.5, 0.) * -robot.heading;
      */
      
      dest.mode = MOTORVOLTAGE;
      if (left) {
        dest.vaux[0] = 12;
        dest.vaux[1] = 12;
        dest.vaux[2] = -24;
      } else {
        dest.vaux[0] = -12;
        dest.vaux[1] = -12;
        dest.vaux[2] = 24;
      }
    }
    
    LOUT << "% blue solid circle  " << Circle(target, 200.) << endl;
  } else {
    LOUT << "% red solid circle  " << Circle(target, 200.) << endl;
    //stand still, correct orientation
    
    //double hdiff=(robot.heading - ((ball.pos.toVec() - robot.pos).angle() - Angle::quarter)).get_rad_pi()/M_PI;
    double hdiff=(robot.heading - (Vec(0.0, 1.0).angle() - Angle::quarter)).get_rad_pi()/M_PI;
    double maxRot=5.5;
    
    PiecewiseLinearFunction plf;
    plf.addVertex(0.1, 0.2);
    plf.addVertex(0.3, 0.4);
    plf.addVertex(1.0, 1.0);
    
    dest.vrot = -(fabs(hdiff)/hdiff) * maxRot * plf.getValue(fabs(hdiff));
  }
  

  dest.kick=false;
  return dest;
}
