
#include "SPhysGotoBallCarefully.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;


SPhysGotoBallCarefully::SPhysGotoBallCarefully () throw () : Skill ("SPhysGotoBallCarefully") {
  const RobotProperties& rp (MWM.get_robot_properties());
  const FieldGeometry& fg (MWM.get_field_geometry());
  robot_half_width = 0.5*rp.robot_width;
  robot_half_length = 0.5*rp.robot_length;
  ball_radius = 0.5*fg.ball_diameter;
  goto_pos_skill.set_dynamics (2.0, 3.0, rp.max_acceleration, rp.max_rotational_acceleration);
}

void SPhysGotoBallCarefully::init (Vec th, int d, bool w) throw () {
  init (th.angle()-Angle::quarter, d, w);
}

void SPhysGotoBallCarefully::init (Angle ta, int d, bool w) throw () {
  target_heading=ta;
  dir = d;
  wait = w;
}

void SPhysGotoBallCarefully::init (Vec th, bool w) throw () {
  init (th.angle()-Angle::quarter, w);
}

void SPhysGotoBallCarefully::init (Angle ta, bool w) throw () {
  init (ta, 0, w);
}

void SPhysGotoBallCarefully::set_dynamics (double v1, double v2, double a1, double a2) throw () {
  goto_pos_skill.set_dynamics (v1, v2, a1, a2);
}

void SPhysGotoBallCarefully::set_dynamics (double v1, double v2) throw () {
  goto_pos_skill.set_dynamics (v1, v2);
}

void SPhysGotoBallCarefully::set_dynamics (double v1) throw () {
  goto_pos_skill.set_dynamics (v1);
}

DriveVector SPhysGotoBallCarefully::getCmd(const Time& texec) throw() {
  BallLocation ball = MWM.get_ball_location (texec);
  return getCmd (texec, ball.pos.toVec());
}

DriveVector SPhysGotoBallCarefully::getCmd(const Time& texec, Vec ballpos) throw() {
  DriveVector dest;
  RobotLocation robot = MWM.get_robot_location (texec);
  if (dir==0) {
    // zuerst noch die Richtung der Anfahrt bestimmen
    dir = ((robot.pos-ballpos).angle()-robot.heading).in_between (Angle::three_quarters, Angle::quarter) ? -1 : +1;    
  }
 
  // Anfahrt ueber Zwischenziele:
  bool tolerance = false;
  Vec stopover_pos;
  Angle stopover_heading = target_heading;

  Vec unit_parallel = Vec::unit_vector_y*target_heading;
  Vec unit_ortho = unit_parallel*dir*Angle::quarter;

  double dist_front = ball_radius+(robot_half_length>robot_half_width ? robot_half_length : robot_half_width)+400;
  double dist_back = ball_radius+robot_half_length+200;
  double dist_side = ball_radius+robot_half_width+200;

  double u = unit_ortho*(robot.pos-ballpos);
  double v = unit_parallel*(robot.pos-ballpos);

  LOUT << "Hundekurve " << (dir>0 ? "links" : "rechts") << '\n';

  Vec sop;
  if ((u<=-0.5*dist_side && (v>=0 || u-v<=-0.5*dist_side)) || (u<=0 && v>=-0.3*dist_side && v<ball_radius+robot_half_length+100)) {
    LOUT << "(P1) vorne\n";
    sop = Vec (0, dist_front);
  } else if ((u<=0.7*dist_side && v>=0) || (v>=dist_front+200)) {
    LOUT << "(P2) seitlich vorne\n";
    sop = Vec (dist_side, ball_radius+robot_half_length);
  } else if (v>=0.4*dist_front && u<=2*dist_side) {
    LOUT << "(P3) seitlich leicht vorne\n";
    sop = Vec (dist_side, 0.3*dist_front);
  } else if (v>=-0.3*dist_back && u<=2*dist_side) {
    if (u<=dist_side) {
      LOUT << "(P4*) nach aussen verschieben\n";
      sop = Vec (dist_side, v);
    } else {
      LOUT << "(P4) seitlich leicht hinter\n";
      sop = Vec (dist_side, -0.5*dist_back);
    }
  } else if (u+v>=0) {
    LOUT << "(P5) seitlich hinter\n";
    sop = Vec (0.5*dist_side, -dist_back);
  } else if (!wait && 2*abs(u)+v<=0) {
    LOUT << "(P7) Ball\n";
    sop = Vec (0,0);
  } else {
    LOUT << "(P6) hinter\n";
    sop = Vec (0,-dist_back);
    if (wait) tolerance = true;
  }

  stopover_pos = ballpos+sop.x*unit_ortho+sop.y*unit_parallel;
  
  goto_pos_skill.init (stopover_pos, stopover_heading, true, tolerance, tolerance);
  
  return goto_pos_skill.getCmd (texec);
}
