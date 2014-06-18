
#include "BApproachBall.h"
#include "../../WorldModel/WorldModel.h"
#include <math.h>

using namespace Tribots;
using namespace std;


BApproachBall::BApproachBall () throw() : Behavior ("BApproachBall") {
  const FieldGeometry& fg = MWM.get_field_geometry();
  target_pointing_to_middle = Vec(0, 0.5*fg.field_length-2000);
  center_own_goal = Vec (0, -0.5*fg.field_length);
  center_opponent_goal = Vec (0,0.5*fg.field_length);

  skill_ball = new SPhysGotoBallAvoidObstacles;
  skill_pos = new SPhysGotoPosAvoidObstacles;
  area_approach_directly = new XYRectangle (Vec (-0.5*fg.field_width+1500, 0.5*fg.field_length-1700), Vec (0.5*fg.field_width-1500, -0.5*fg.field_length+3500));
  area_approach_pointing_away_own_goal = new XYRectangle (Vec (-0.5*fg.field_width+1000, -0.5*fg.field_length+3500), Vec (0.5*fg.field_width-1000, -0.5*fg.field_length));

  area_field = new XYRectangle (Vec(-0.5*fg.field_width-500, 0.5*fg.field_length+500), Vec(0.5*fg.field_width+500, -0.5*fg.field_length-500));
  
  UnionArea* ua = new UnionArea;
  ua->add (XYRectangle (Vec (-0.5*fg.field_width, 0.5*fg.field_length), Vec (-0.5*fg.goal_width, 0.5*fg.field_length-300)));
  ua->add (XYRectangle (Vec (0.5*fg.field_width, 0.5*fg.field_length), Vec (0.5*fg.goal_width, 0.5*fg.field_length-300)));
  ua->add (XYRectangle (Vec (-0.5*fg.field_width, 0.5*fg.field_length), Vec (-0.5*fg.field_width+1000, 0.5*fg.field_length-3500)));
  ua->add (XYRectangle (Vec (0.5*fg.field_width, 0.5*fg.field_length), Vec (0.5*fg.field_width-1000, 0.5*fg.field_length-3500)));
  area_approach_pointing_to_middle = ua;
  
  ua = new UnionArea;
  ua->add (XYRectangle (Vec(-0.5*fg.field_width-500, 0.5*fg.field_length+500), Vec(-0.5*fg.field_width+1000, -0.5*fg.field_length-500)));
  ua->add (XYRectangle (Vec(0.5*fg.field_width+500, 0.5*fg.field_length+500), Vec(0.5*fg.field_width-1000, -0.5*fg.field_length-500)));
  ua->add (XYRectangle (Vec(-0.5*fg.field_width-500, 0.5*fg.field_length+500), Vec(0.5*fg.field_width+500, 0.5*fg.field_length-1000)));
  ua->add (XYRectangle (Vec(-0.5*fg.field_width-500, -0.5*fg.field_length-500), Vec(0.5*fg.field_width+500, -0.5*fg.field_length+1000)));
  area_near_boundary = ua;

  skill_ball->set_dynamics (2.5, 5.0);
  skill_pos->set_dynamics (2.5, 5.0);
}


BApproachBall::~BApproachBall () throw() {
  delete skill_ball;
  delete skill_pos;
  delete area_approach_pointing_to_middle;
  delete area_approach_pointing_away_own_goal;
  delete area_approach_directly;
  delete area_near_boundary;
  delete area_field;
}

DriveVector BApproachBall::getCmd (const Time& t) throw(TribotsException) {
  Vec ball = ballPos;
  Vec ballvel = ballVel;
  
//  const FieldGeometry& fg = MWM.get_field_geometry();
  
  ball_rolling = (ball_rolling && ballvel.length()>0.4) || (ballvel.length()>0.6);
  
  if (!ball_rolling) {
    // nicht-rollenden Ball anfahren; Heading je nach Anfahrtsbereich bestimmen
    intercept_prefered = false;
    Vec heading = center_opponent_goal-ball;
    double max_v = 2.0;
    Angle heading_tolerance = Angle::deg_angle (15);
    if (area_approach_pointing_to_middle->is_inside (ball)) {
      heading = target_pointing_to_middle-ball;
    } else if (area_approach_pointing_away_own_goal->is_inside (ball)) {
      heading = ball-center_own_goal;
      heading_tolerance = Angle::deg_angle (40);
    } else if (area_approach_directly->is_inside (ball)) {
      heading = ball-robotPos;
      heading_tolerance = Angle::deg_angle (30);
    }
    
    Vec opgoal = center_opponent_goal-ball;
    max_v *= 0.5*(1.0+cos ((opgoal-heading).angle().get_rad()));  // Anfahrtsgeschwindigkeit maximal bei Anfahrt auf gegnerisches Tor, minimal bei Anfahrt in Gegenrichtung
    if (max_v>2.5) max_v=2.5;
    skill_ball->init (heading, max_v, heading_tolerance);
    return skill_ball->getCmdNonMovingBall (t);
  } else {
    // rollenden Ball anfahren
    Line ballmovement (ball, ball+ballvel);
    Vec pp = ballmovement.perpendicular_point (robotPos);
    Vec contact_point = skill_ball->determineContactPoint (t);
    bool robot_near_ball = (ball-robotPos).length()<1000;  // Roboter nahe am Ball?
    bool prefer_intercept = ((robotPos-contact_point)*ballvel>=0) || ((pp-contact_point).length()<800);  // eher intercepten oder folgen? (je nach Lage des theoretischen Kontaktpunktes
    bool contact_point_in_back = area_approach_pointing_away_own_goal->is_inside (contact_point);  // Kontaktpunkt vor dem eigenen Tor?
//    bool contact_point_near_boundary = area_near_boundary->is_inside (contact_point);  // Kontaktpunkt am Rand?    
//    bool ballvel_pointing_outside = area_field->is_inside (contact_point+500*ballvel);  // bleibt der Ball im Feld nach Kontakt?
    bool ballvel_towards_goal = (ballvel.y<-0.3);  // Ball rollt in Richtung eigene Haelfte

    // stark vereinfacht. TODO: Spezialfaelle unterscheiden
    if (contact_point_in_back && ballvel_towards_goal) {
      if ((contact_point-robotPos)*ballvel<0) {
        intercept_prefered = true;
        skill_ball->init (Angle::zero, 2.5);  // Heading wird nicht benoetigt
        return skill_ball->getCmdInterceptBall (t);
      } else {
        intercept_prefered = false;
        skill_ball->init (contact_point-robotPos, 2.0);
        return skill_ball->getCmdNonMovingBall (t);
      }
    }

    if ((robot_near_ball && intercept_prefered) || (!(!intercept_prefered && robot_near_ball) && prefer_intercept)) {
      intercept_prefered = true;
      skill_ball->init (Angle::zero, 2.5);  // Heading wird nicht benoetigt
      return skill_ball->getCmdInterceptBall (t);
    } else {
      intercept_prefered = false;
      skill_ball->init (Angle::zero, 2.5);  // Heading wird nicht benoetigt
      return skill_ball->getCmdFollowBall (t);
    }
  } 
} 

void BApproachBall::loseControl (const Time& t) throw(TribotsException) {
  skill_ball->loseControl(t);
  skill_pos->loseControl(t);
  ball_rolling = false;
  intercept_prefered = false;
}

bool BApproachBall::checkInvocationCondition (const Time& t) throw() {
  return (MWM.get_ball_location(t).pos_known==BallLocation::known) || (MWM.get_ball_location(t).pos_known==BallLocation::communicated);
}

bool BApproachBall::checkCommitmentCondition (const Time& t) throw() {
  return checkInvocationCondition(t);
}
