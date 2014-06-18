
#include "SPhysVolley.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include <cmath>

using namespace Tribots;
using namespace std;

namespace {
  const bool debug = true;
}

namespace {
  // Hilfsfunktionen:
  // Abschaetzen der Zeit in ms, die der Roboter braucht, um einen Punkt target zu erreichen mit Anfangspunkt start und Anfangsgeschwindigkeit vel
  double expected_time_to_pos (Vec start, Vec target, Vec vel, double vmax=2) {
    Vec delta = target-start;
    double ds = 1e-3*delta.length();  // in m
    const double robot_acc = 2;  // Annahme: Roboterbeschleunigung max 2 m/s^2
    double v0=vel*delta.normalize();
    double bw = (vmax*vmax)-(v0*v0)/(2*robot_acc);
    if (v0>vmax)
      return 1e3*ds/vmax;
    else if (ds<bw)
      return 1e3*(sqrt(v0*v0+2*robot_acc*ds)-v0)/robot_acc;
    else
      return 1e3*(vmax-v0)/robot_acc+(ds-bw)/vmax;
  }

  // Abschaetzen der Ballposition (mit Bremseffekt) starte von pos mit Geschwindigkeit vel fuer dt Millisekunden
  Vec expected_ballpos (Vec pos, Vec vel, double dt) {
    const double ball_dec = 0.3;  // Annahme: Bremsverzoegerung des Balls
    double maxt = 1e3*vel.length()/ball_dec;
    double tdt = (maxt>dt ? dt : maxt);
    return pos+tdt*vel-0.5e-3*ball_dec*tdt*tdt*vel.normalize();
  }

}


SPhysVolley::SPhysVolley () throw () {
  Time now;
  loseControl(now);  // zureucksetzen
}

SPhysVolley::~SPhysVolley () throw () {;}

void SPhysVolley::init (Vec td, double v) throw () {
  target_velocity = v;
  target_directed=td;
}

void SPhysVolley::set_dynamics (double vl, double va, double al, double aa) throw () {
  goto_pos_skill.set_dynamics (vl, va, al, aa);
}

void SPhysVolley::set_dynamics (double vl, double va) throw () {
  goto_pos_skill.set_dynamics (vl, va);
}

void SPhysVolley::set_dynamics (double vl) throw () {
  goto_pos_skill.set_dynamics (vl);
}

Vec SPhysVolley::getInterceptPoint (const Time& t) throw () {
  const RobotLocation& rloc = MWM.get_robot_location (t);
  const BallLocation& bloc = MWM.get_ball_location (t);
  Vec ballpos = bloc.pos.toVec();
  Vec ballvel = bloc.velocity.toVec();
  Vec robot_ball = ballpos-rloc.pos;
  goto_pos_skill.set_ball_as_obstacle (false);  // da Ball sowieso in Bewegung
  goto_pos_skill.set_dynamics (3.0);
  
  Vec stop_over_point = ballpos;
  bool needwait = false;
  try{
    // Attempt: go straight into target direction, maybe wait until ball passes (if possible)
    Line ballmovementline (ballpos, ballpos+ballvel);
    Line robottargetline (rloc.pos, target_directed);
    Vec pp = intersect (ballmovementline, robottargetline);
    double expected_time_to_pp = expected_time_to_pos (rloc.pos, pp, rloc.vtrans);
    Vec expected_ballpos1 = expected_ballpos (ballpos, ballvel, expected_time_to_pp);
    Angle robot_cball = (ballpos-rloc.pos).angle();
    Angle robot_pp = (pp-rloc.pos).angle();
    Angle robot_expected = (expected_ballpos1-rloc.pos).angle();
    bool ball_did_not_reach_pp = ((robot_cball-robot_pp).get_rad()>(robot_pp-robot_cball).get_rad() ? robot_expected.in_between (robot_cball, robot_pp) : robot_expected.in_between (robot_pp, robot_cball));
    if (ball_did_not_reach_pp) {
      stop_over_point = pp+500*(pp-target_directed).normalize();
      needwait=true;
      double vmax=2;
      do {
        double vmax2 = vmax-0.1;
        double expected_time_to_pp = expected_time_to_pos (rloc.pos, pp, rloc.vtrans);
        Vec expected_ballpos1 = expected_ballpos (ballpos, ballvel, expected_time_to_pp);
        Angle robot_cball = (ballpos-rloc.pos).angle();
        Angle robot_pp = (pp-rloc.pos).angle();
        Angle robot_expected = (expected_ballpos1-rloc.pos).angle();
        bool ball_did_not_reach_pp = ((robot_cball-robot_pp).get_rad()>(robot_pp-robot_cball).get_rad() ? robot_expected.in_between (robot_cball, robot_pp) : robot_expected.in_between (robot_pp, robot_cball));
        if (ball_did_not_reach_pp) {
          vmax=vmax2;
        } else {
          break;
        }
      }while (vmax>0.8);
      goto_pos_skill.set_dynamics (1.3*vmax);
    }
  }catch(exception&) {
    needwait=false;
  }
      
  if (!needwait) {
    // Attempt: calculate position where robot can meet ball
    Vec ballposfut = ballpos;
    // ungefaehre Zeit bis zum Eintreffen des Roboters an der Ballposition berechnen (Berechnung in m,s,m/s,m/s^2), Endergebnis in ms:
    double expected_time_to_ball=0;
    for (unsigned int j=0; j<5; j++) {  // mehrere Iterationen durchfuehren, um Loesung zu approximieren
      expected_time_to_ball = expected_time_to_pos (rloc.pos, ballposfut-300*(ballposfut-rloc.pos).normalize(), rloc.vtrans);
      ballposfut = expected_ballpos (ballpos, ballvel, expected_time_to_ball);
    }
  
    stop_over_point = ballposfut+300*(ballposfut-target_directed).normalize();
  }
  
  // Attempt: if ball is near in front of robot, go to ball
  Vec ballrel = (ballpos-rloc.pos).rotate (-rloc.heading);
  if ( ballrel.y>0 && ballrel.y<600 && abs(ballrel.x)<300) {
    stop_over_point = ballpos;
  }

  return stop_over_point;
}  


DriveVector SPhysVolley::getCmd(const Time& t) throw() {
  Vec stop_over_point = getInterceptPoint (t);
    
  goto_pos_skill.init (stop_over_point, target_directed-stop_over_point, target_velocity);
  
  return goto_pos_skill.getCmd (t);
}
