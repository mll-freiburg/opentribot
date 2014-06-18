
#include "BGoaliePositioning.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoaliePositioning::BGoaliePositioning (Vec home, Vec corner, Angle ma, bool cb, SPhysGotoPos* sp) throw () : Behavior ("BGoaliePositioning") {
  if (sp)
    goto_pos_skill = sp;
  else
    goto_pos_skill = &own_goto_pos_skill;
  use_comm_ball = cb;
  max_angle = ma;
  if (corner.x>0) {
    right_end = corner;
    left_end = Vec (-corner.x, corner.y);
  } else {
    left_end = corner;
    right_end = Vec (-corner.x, corner.y);
  }
  Vec center (home.x, ((corner.x-home.x)*(corner.x-home.x)+corner.y*corner.y-home.y*home.y)/(2*(corner.y-home.y)));
  double radius = (right_end-center).length();
  Angle a1 = (right_end-center).angle();
  Angle a2 = (left_end-center).angle();
  positioning_arc = Arc (center, radius, a1, a2);
  const FieldGeometry& fg (MWM.get_field_geometry());
  pa1.x = -0.5*fg.penalty_area_width;
  pa1.y = -0.5*fg.field_length-200;
  pa2.x = -pa1.x;
  pa2.y = -0.5*fg.field_length+fg.penalty_area_length;
  goal_post_right.x=0.5*fg.goal_width;
  goal_post_right.y=-0.5*fg.field_length;
  goal_post_left.x=-0.5*fg.goal_width;
  goal_post_left.y=-0.5*fg.field_length;
}

void BGoaliePositioning::gainControl (const Time&) throw(TribotsException) {
  goto_pos_skill->set_dynamics (2.0, 2.0, 5.0, 8.0);
}

bool BGoaliePositioning::checkInvocationCondition (const Time& texec) throw () {
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));
  return (ball_exec.pos_known == BallLocation::known || (use_comm_ball && ball_exec.pos_known == BallLocation::communicated)) && (pa1.x<=robot_exec.pos.x && pa1.y<=robot_exec.pos.y && robot_exec.pos.x<=pa2.x && robot_exec.pos.y<=pa2.y);
}

bool BGoaliePositioning::checkCommitmentCondition (const Time& texec) throw () {
  return checkInvocationCondition (texec);
}

DriveVector BGoaliePositioning::getCmd(const Time& texec) throw () {
  DriveVector dest;
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));

  // die statische Position berechnen:
  Vec stat_pos;
  Vec ball_lpost = goal_post_left-ball_exec.pos.toVec();
  Vec ball_rpost = goal_post_right-ball_exec.pos.toVec();
  Line stat_pos_line (ball_exec.pos.toVec(), ball_exec.pos.toVec()+ball_lpost.normalize()+ball_rpost.normalize());
  vector<Vec> p = intersect (stat_pos_line, positioning_arc);
  if (p.size()==0) {
    if (ball_exec.pos.x<0)
      stat_pos=left_end;
    else
      stat_pos=right_end;
  } else if (p.size()==1) {
    if (ball_exec.pos.y<p[0].y) {
      // Problem: Ball liegt hinter dem Schnittpunkt
      if (ball_exec.pos.x<0)
        stat_pos=left_end;
      else
        stat_pos=right_end;
    } else
      stat_pos=p[0];
  } else {
    // Problem: zwei Schnittpunkt: waehle der dem Ball naechstgelegenen
    if (ball_exec.pos.x*p[0].x<0)
      stat_pos = p[1];
    else
      stat_pos = p[0];
  }

  // die dynamische Position berechnen:
  Vec dyna_pos=stat_pos;
  if (ball_exec.velocity.length()>0.4 && ball_exec.velocity.y<0) {
    Line dyna_pos_line (ball_exec.pos.toVec(), ball_exec.pos.toVec()+ball_exec.velocity.toVec());
    Line goal_line (goal_post_left, goal_post_right);
    try{
      Vec q = intersect (goal_line, dyna_pos_line);
      bool dangerous_ball = (goal_post_left.x-2000<q.x && q.x<goal_post_right.x+2000);
      if (dangerous_ball) {
        int leftright = (q.x<0.3*goal_post_left.x ? -1 : (q.x>0.3*goal_post_right.x ? +1 : 0));  // Ball auf linke Eck, rechte Eck oder Mitte
        int kurzlang = (leftright*ball_exec.pos.x>0.01 ? +1 : (leftright*ball_exec.pos.x<-0.01 ? -1 : 0));  // Ball ins kurze oder lange Eck
        if (kurzlang==-1) {
          // langes Eck: versuche dem Ball den Weg abzuschneiden
          dyna_pos = dyna_pos_line.perpendicular_point (stat_pos);
        } else {
          // kurzes Eck oder Mitte: positionieren
          p = intersect (dyna_pos_line, positioning_arc);
          if (p.size()==0) {
            if (leftright==1)
              dyna_pos = right_end;
            else
              dyna_pos = left_end;
          } else if (p.size()==1)
            dyna_pos = p[0];
          else if (ball_exec.pos.x*p[0].x<0)
            dyna_pos = p[1];
          else
            dyna_pos = p[0];
        }
      }
    }catch(invalid_argument&){
      // Ball rollt parallel zu Torauslinie
      if (ball_exec.pos.y-goal_post_left.y<700)
        if (ball_exec.pos.x<0)
          dyna_pos=left_end;
        else
          dyna_pos=right_end;
    }
  }

  // Die Zielposition aus dyna_pos und stat_pos berechnen:
  double rho = 1.0/(1.0+exp(6-3.5*ball_exec.velocity.length()));
  double dy = (0.5*MWM.get_field_geometry().field_length+ball_exec.pos.y);
  double tau = 1.0-1.0/(1.0+exp(5-0.0025*dy));
  Vec target_pos = tau*rho*dyna_pos+(1-tau*rho)*stat_pos;
  LOUT << "% dark_blue cross " << dyna_pos << ' ' << stat_pos << ' ' << target_pos << '\n';
  LOUT << "% dark_blue word " << dyna_pos << " Dyn word " << stat_pos << " Stat word " << target_pos << " Tgt\n";
  Angle target_heading = (ball_exec.pos-target_pos).angle()-Angle::quarter;
  if (target_heading.in_between (max_angle, Angle::half))
    target_heading=max_angle;
  else if (target_heading.in_between (Angle::half, -max_angle))
    target_heading=-max_angle;

  if (ball_exec.pos.y<robot_exec.pos.y && abs(ball_exec.pos.x)<goal_post_right.x)  // wenn Ball hinter Roboter, stehen bleiben
    goto_pos_skill->init (robot_exec.pos, robot_exec.heading, true);
  else  
    goto_pos_skill->init (target_pos, target_heading, true);
  dest = goto_pos_skill->getCmd (texec);
  dest.kick=false;
  return dest;
}

BGoaliePositioningFarBall::BGoaliePositioningFarBall (Vec home, Vec corner, Angle ma, bool cb, SPhysGotoPos* sp) throw () :
  BGoaliePositioning (home, corner, ma, cb, sp) {
  name="BGoaliePositioningFarBall";
  double fwh = 0.5*MWM.get_field_geometry().field_width;
  double flh = 0.5*MWM.get_field_geometry().field_length;
  ballFarArea = XYRectangle (Vec(-fwh-1000, -flh+4500), Vec(fwh+1000, flh+1000));
  ballFarSeen.add_sec(-100);  
}

bool BGoaliePositioningFarBall::checkInvocationCondition (const Time& t) throw () {
  return BGoaliePositioning::checkInvocationCondition (t) && t.diff_msec (ballFarSeen)<1000;
}

bool BGoaliePositioningFarBall::checkCommitmentCondition (const Time& t) throw () {
  return BGoaliePositioning::checkCommitmentCondition (t) && t.diff_msec (ballFarSeen)<1000;
}

void BGoaliePositioningFarBall::cycleCallBack (const Time& t) throw () {
  if (ballFarArea.is_inside (MWM.get_ball_location(t).pos.toVec()) && 
    (MWM.get_ball_location(t).pos_known==BallLocation::known || MWM.get_ball_location(t).pos_known==BallLocation::communicated))
    ballFarSeen=t;  // beobachten, ob der Ball in der entfernten Bereich gesehen wurde
}
