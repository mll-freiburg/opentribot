
#include "BGoalieGoalKick.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/random.h"
#include <cmath>

using namespace Tribots;
using namespace std;

namespace {

  inline int signum (const double& x) { return (x>0 ? 1 : (x<0 ? -1 : 0)); }

}


BGoalieGoalKick::BGoalieGoalKick (Angle ma, bool hw, bool kp) throw () : Behavior ("BGoalieGoalKick") {
  const FieldGeometry& fg (MWM.get_field_geometry());
  const RobotProperties& rp (MWM.get_robot_properties());
  pa1.x = -0.5*fg.penalty_area_width;
  pa1.y = -0.5*fg.field_length;
  pa2.x = -pa1.x;
  pa2.y = -0.5*fg.field_length+fg.penalty_area_length;
  was_active=false;
  was_moving_around_goal_post=false;
  goal_post_right.x = 0.5*fg.goal_width;
  goal_post_right.y = -0.5*fg.field_length;
  goal_post_left.x = -0.5*fg.goal_width;
  goal_post_left.y = -0.5*fg.field_length;
  target_direction_known=false;
  kick_permission = kp;
  kicker_half_width = 0.5*rp.kicker_width;
  kick_distance = rp.kicker_distance+0.5*fg.ball_diameter;
  turn_radius = rp.kicker_distance+0.5*fg.ball_diameter+100;
  robotball_radius = rp.max_robot_radius+0.5*fg.ball_diameter+200;
  dont_care_direction_area = Triangle (Vec (0, -0.5*fg.field_length-fg.goal_length), Vec(-0.5*fg.field_length+fg.penalty_area_length, -0.5*(fg.penalty_area_length+fg.goal_length)), Vec(-0.5*fg.field_length+fg.penalty_area_length, +0.5*(fg.penalty_area_length+fg.goal_length)));
  near_goal_post_inner_area = XYRectangle (Vec (0.5*fg.goal_width-0.5*rp.robot_width-200,-0.5*fg.field_length-fg.goal_length), Vec (0.5*fg.goal_width, -0.5*fg.field_length+600));
  near_goal_post_inner_area_dil = XYRectangle (Vec (0.5*fg.goal_width-0.5*rp.robot_width-200,-0.5*fg.field_length-fg.goal_length), Vec (0.5*fg.goal_width+300, -0.5*fg.field_length+600));
  near_goal_post_outer_area = XYRectangle (Vec (0.5*fg.goal_width+0.5*rp.robot_width+500,-0.5*fg.field_length-fg.goal_length), Vec (0.5*fg.goal_width, -0.5*fg.field_length+600));
  near_goal_post_outer_area_dil = XYRectangle (Vec (0.5*fg.goal_width+0.5*rp.robot_width+500,-0.5*fg.field_length-fg.goal_length), Vec (0.5*fg.goal_width-300, -0.5*fg.field_length+600));
}

bool BGoalieGoalKick::checkInvocationCondition (const Time& t) throw () {
  const GameState& gs (MWM.get_game_state ());
  const BallLocation& ball = MWM.get_ball_location (t);
  return (gs.refstate == preOwnGoalKick || (gs.refstate == preOwnFreeKick && ball.pos_known==BallLocation::known && pa1.x < ball.pos.x && ball.pos.x < pa2.x && pa1.y < ball.pos.y && ball.pos.y < pa2.y));
}

bool BGoalieGoalKick::checkCommitmentCondition (const Time& t) throw () {
  const GameState& gs (MWM.get_game_state ());
  return checkInvocationCondition(t) || (was_active && gs.refstate == freePlay);
}

void BGoalieGoalKick::gainControl(const Time& t) throw() {
  ballpos=Vec::zero_vector;
  wait_rel_to_goal=0;
  dir=0;
}

void BGoalieGoalKick::loseControl(const Time& t) throw() {
  was_active=false;
  target_direction_known=false;
  goto_via_skill.loseControl(t);
}

DriveVector BGoalieGoalKick::getCmd(const Time& t) throw () {
  DriveVector dest;
  
  const BallLocation& ball = MWM.get_ball_location (t);
  const RobotLocation& robot = MWM.get_robot_location (t);
  const GameState& gs = MWM.get_game_state ();
  was_active=true;
  dest.kick=false;

  if (ball.pos_known != BallLocation::known) {
    LOUT << "unbekannte Ballposition\n";
    goto_pos_skill.init (Vec(0,0), Angle::zero, true);
    dest = goto_pos_skill.getCmd (t);
  } else if (gs.refstate != freePlay) {
    // Wenn die Ballposition sich markant verschoben hat, dem Ball folgen
    if ((ballpos-ball.pos.toVec()).length()>200)
      ballpos = ball.pos.toVec();

    Vec waitpos = ballpos;

    if (dont_care_direction_area.is_inside (ballpos)) {
      if (dir==0) dir = (ballpos.x<robot.pos.x ? -1 : +1);
    } else {
      dir = (waitpos.x>0 ? +1 : -1);
    }

    Vec absballpos (abs(ballpos.x), ballpos.y);
    bool area1d = near_goal_post_inner_area_dil.is_inside (absballpos);
    bool area2d = near_goal_post_outer_area_dil.is_inside (absballpos);
    bool area1 = near_goal_post_inner_area.is_inside (absballpos);
    //    bool area2 = near_goal_post_outer_area.is_inside (absballpos);
    if (area1d || area2d) {
      if ((!area1d) || (area2d && wait_rel_to_goal==-1) || (!area1 && wait_rel_to_goal==0)) {
	waitpos.x = signum (ballpos.x)*(goal_post_right.x+500);
	dir = -signum (ballpos.x);
        wait_rel_to_goal=-1;
	LOUT << "Warteposition ausserhalb des Tores in Tornaehe " << dir << "\n";
      } else {
	waitpos.x = signum (ballpos.x)*(goal_post_right.x-500);
	dir = signum (ballpos.x);
        wait_rel_to_goal=+1;
	LOUT << "Warteposition innerhalb des Tores in Tornaehe " << dir << "\n";
      }
    } else
      wait_rel_to_goal=0;
    LOUT << "\% dark_green cross " << waitpos << '\n';
    
    // Fahre zur Warteposition ohne den Ball zu beruehren
    if (waitpos.y<goal_post_right.y+600 && (was_moving_around_goal_post || robot.pos.y<goal_post_right.y+600)) {
      if (waitpos.x<goal_post_right.x && robot.pos.x>goal_post_right.x) {
	goto_via_skill.init (goal_post_right+Vec(-600,300), Angle::zero, goal_post_right+Vec(0,600), 400, true);
  dest = goto_via_skill.getCmd (t);
	was_moving_around_goal_post=true;
	LOUT << "fahre um rechten Torpfosten herum\n";
	return dest;
      }
      if (waitpos.x>goal_post_right.x && robot.pos.x<goal_post_right.x) {
	goto_via_skill.init (goal_post_right+Vec(+800,300), Angle::zero, goal_post_right+Vec(0,600), 400, true);
	dest = goto_via_skill.getCmd (t);
	was_moving_around_goal_post=true;
	LOUT << "fahre um rechten Torpfosten herum\n";
	return dest;
      }	
      if (waitpos.x<goal_post_left.x && robot.pos.x>goal_post_left.x) {
	goto_via_skill.init (goal_post_left+Vec(-800,300), Angle::zero, goal_post_left+Vec(0,600), 400, true);
	dest = goto_via_skill.getCmd (t);
	was_moving_around_goal_post=true;
	LOUT << "fahre um linken Torpfosten herum\n";
	return dest;
      }
      if (waitpos.x>goal_post_left.x && robot.pos.x<goal_post_left.x) {
	goto_via_skill.init (goal_post_left+Vec(+600,300), Angle::zero, goal_post_left+Vec(0,600), 400, true);
	dest = goto_via_skill.getCmd (t);
	was_moving_around_goal_post=true;
	LOUT << "fahre um linken Torpfosten herum\n";
	return dest;
      }	
    }

    if (was_moving_around_goal_post) {
      goto_via_skill.loseControl(t);
      was_moving_around_goal_post=false;
    }
    goto_ball_skill.init (Angle::zero, dir, true);
    dest = goto_ball_skill.getCmd (t, waitpos);
    LOUT << "fahre auf Warteposition\n";
    return dest;

  } else {

    // den Abstoss/Freistoss ausfuehren
    if (!target_direction_known) {
      target_direction_known=true;
      // Zielrichtung berechnen
      target_direction = (brandom (0.5) ? +1 : -1)*Angle::deg_angle (30);  // nicht Null, da sonst die simple Gegnertaktik "stehe direkt vor den Ball" aufgeht
      // fuer jedes Hindernis den Winkel der rechten und linken Hindernisspitze berechnen
      vector<Angle> leftangle (obstacles.size());
      vector<Angle> rightangle (obstacles.size());
      vector<ObstacleDescriptor>::const_iterator oit=obstacles.begin();
      vector<ObstacleDescriptor>::const_iterator oitend=obstacles.end();
      vector<Angle>::iterator lait = leftangle.begin();
      vector<Angle>::iterator rait = rightangle.begin();
      while (oit<oitend) {
	double dist = (oit->pos-ball.pos.toVec()).length();
	Angle baseangle = (oit->pos-ball.pos.toVec()).angle();
	if (dist<3000 && oit->width>200) {
	  Angle addangle = Angle::rad_angle (atan2 (0.5*oit->width, dist));
	  (*lait) = baseangle+addangle;
	  (*rait) = baseangle-addangle;
	} else {
	  (*lait) = Angle::zero;
	  (*rait) = Angle::half;
	}
	oit++;
	lait++;
	rait++;
      }
      for (unsigned int i=0; i<leftangle.size(); i++) {
      	LOUT << "\% black solid thin line " << ball.pos << ' ' << ball.pos+1500*Vec::unit_vector_x.rotate (rightangle[i]) << " dark_blue line " << ball.pos << ' ' << ball.pos+(1500)*Vec::unit_vector_x.rotate (leftangle[i]) << " arc " << ball.pos << ' ' << 500+20*i << ' '  << rightangle[i].get_deg() << ' ' << leftangle[i].get_deg() << '\n';
      }

      // verschiedene Richtungen auf Eignung hin pruefen
      double best_free_angle=0;  // groesster bisher gefundener freier Winkelbreich in rad
      vector<Angle>::iterator raitend = rightangle.end();
      const Angle fuenfgrad (Angle::deg_angle(5));
      const Angle zwanziggrad (Angle::deg_angle(20));
      for (int i=0; i<25; i++) {
	if (i==19 && best_free_angle>0.16)
	  break;  // wenn eine gute Richtung nach vorne gefunden wurde, dann los; ansonsten in den Randbereichen weitersuchen
	Angle cangle = Angle::quarter+(i%2==0 ? i/2 : -(i/2+1))*fuenfgrad;
	Angle leftmost = cangle+zwanziggrad;
	Angle rightmost = cangle-zwanziggrad;
	lait = leftangle.begin();
	rait = rightangle.begin();
	while (rait<raitend) {
	  if (cangle.in_between (*rait, *lait)) {
	    leftmost=rightmost;
	    break;  // keine gute Richtung, da Hindernis im Weg
	  }
	  if (lait->in_between (rightmost, cangle))
	    rightmost = *lait;  // Hindernis schraenkt den Bewegungsspielraum nach rechts ein
	  if (rait->in_between (cangle, leftmost))
	    leftmost = *rait;  // Hindernis schraenkt den Bewegungsspielraum nach links ein
	  lait++;
	  rait++;
	}
	double free_angle = (leftmost-rightmost).get_rad();
	if (free_angle>best_free_angle) {
	  double ave_angle = rightmost.get_rad() + 0.5*free_angle;
	  double sea_angle = cangle.get_rad();
	  double w = sea_angle-0.5*M_PI;
	  double rho = exp(-0.5*w*w);
	  target_direction = Angle::rad_angle (rho*ave_angle+(1-rho)*sea_angle);
	  best_free_angle=free_angle;
	}
      }
    }

    // in target_heading steht nun die Zielausrichtung, Bewegung berechnen
    LOUT << "\% orange cross " << ball.pos << " line " << ball.pos << ' ' << ball.pos+1000*Vec::unit_vector (target_direction) << '\n';
    Angle target_heading = target_direction-Angle::quarter;
    if (abs(((ball.pos-robot.pos).angle()-target_direction).get_rad_pi())<0.2) {
      goto_ball_skill.init (target_heading, false);
      dest = goto_ball_skill.getCmd (t);
      LOUT << "GotoBall\n";
    } else {
      turn_skill.init (ball.pos.toVec(), target_heading, turn_radius);
      dest = turn_skill.getCmd (t);
      LOUT << "TurnAroundBall\n";
    }
    if (kick_permission) {
      Time tkick=t;
      tkick.add_msec(-200);
      RobotLocation rkick = MWM.get_robot_location (tkick);
      BallLocation bkick = MWM.get_ball_location (tkick);
      try{
	Line robot_mid (rkick.pos, rkick.pos+Vec::unit_vector_y.rotate (rkick.heading));
	Vec pp = robot_mid.perpendicular_point (bkick.pos.toVec());
	dest.kick = (pp-bkick.pos.toVec()).squared_length()<kicker_half_width*kicker_half_width
	  && (pp-rkick.pos).angle().in_between (rkick.heading, rkick.heading+Angle::half)
	  && (pp-rkick.pos).length()<kick_distance 
	  && bkick.velocity.length()>0.5;
      }catch(std::invalid_argument&){;} // Ball- und Roboterposition fallen aufeinander
    }
  }
  return dest;
}
