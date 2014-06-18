
#include "SPhysFollowBall.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace std;
using namespace Tribots;

namespace {
  const bool debug = false;
}

namespace {
  // Hilfsfunktionen:
  // Abschaetzen der Zeit in ms, die der Roboter braucht, um einen Punkt target zu erreichen mit Anfangspunkt start und Anfangsgeschwindigkeit vel
  double expected_time_to_pos (Vec start, Vec target, Vec vel) {
    Vec delta = target-start;
    double ds = 1e-3*delta.length();  // in m
    const double vmax = 2;  // Annahme: Roboter-Geschwindigkeit max. 2 m/s
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

  Vec determineContactPoint (const Time& t) throw () {
    // ungefaehre Zeit bis zum Eintreffen des Roboters an der Ballposition berechnen (Berechnung in m,s,m/s,m/s^2), Endergebnis in ms:
    const RobotLocation& rloc = MWM.get_robot_location (t);
    const BallLocation& bloc = MWM.get_ball_location (t);
    double expected_time_to_ball=0;
    Vec ballposfut = bloc.pos.toVec();
    for (unsigned int j=0; j<5; j++) {  // mehrere Iterationen durchfuehren, um Loesung zu approximieren
      expected_time_to_ball = expected_time_to_pos (rloc.pos, ballposfut-300*(ballposfut-rloc.pos).normalize(), rloc.vtrans);
      ballposfut = expected_ballpos (bloc.pos.toVec(), bloc.velocity.toVec(), expected_time_to_ball);
    }
    if ((ballposfut-rloc.pos).length()>2000) {
      ballposfut+=300*bloc.velocity.toVec(); // bei grossen Entfernungen eine gewisse Unsicherheit beruecksichtigen
    }
    return ballposfut;
  }

}


SPhysFollowBall::SPhysFollowBall () throw () {
  double d3, d4, d5, d6;
  goto_pos_skill.get_dynamics (maxvtrans,maxvrot,d3,d4,d5,d6);
  maxvtrans-=0.5;
  goto_pos_skill.set_dynamics (maxvtrans,maxvrot);
}

SPhysFollowBall::~SPhysFollowBall () throw () {;}

void SPhysFollowBall::set_dynamics (double vl, double va, double al, double aa) throw () {
  goto_pos_skill.set_dynamics (vl, va, al, aa);
}

void SPhysFollowBall::set_dynamics (double vl, double va) throw () {
  goto_pos_skill.set_dynamics (vl, va);
}

void SPhysFollowBall::set_dynamics (double vl) throw () {
  goto_pos_skill.set_dynamics (vl);
}


DriveVector SPhysFollowBall::getCmd (const Time& t) throw () {
  goto_pos_skill.set_ball_as_obstacle (false);  // da Ball sowieso in Bewegung
  const RobotLocation& rloc = MWM.get_robot_location (t);
  const BallLocation& bloc = MWM.get_ball_location (t);
  const FieldGeometry& fg = MWM.get_field_geometry();
  const RobotProperties& rp = MWM.get_robot_properties();
  Vec ballpos = bloc.pos.toVec();
  Vec ballvel = bloc.velocity.toVec();
  Vec robot_ball = ballpos-rloc.pos;
  Vec norm = ballvel.normalize();
  Vec ortho = norm.rotate_quarter();
  Vec ballposfut = determineContactPoint (t);
  Line ballmovement (ballpos, ballpos+ballvel);
  aveballvel = (latest_aveballvel.elapsed_msec()>200 ? ballvel : 0.9*aveballvel+0.1*ballvel);

  Vec ball_rel = (bloc.pos.toVec()-rloc.pos)/rloc.heading;
//  bool push_by_fingers = (ball_rel.y<rp.kicker_distance+200) && (abs(ball_rel.x)<0.5*rp.robot_width+150) && (abs(ball_rel.x)>0.5*rp.kicker_width+100);
//  if (push_by_fingers) {
//    if (debug) LOUT << "Ball durch Dribblehoernchen weggedrueckt\n";
//    DriveVector nv (Vec(0,0), 0, false);
//    return nv;
//  }

  Angle tg_heading_dyn = ballvel.angle()-Angle::quarter;
  Angle heading_towards_ball = (ballposfut-rloc.pos).angle()-Angle::quarter;
  Angle heading_towards_ball_mod = heading_towards_ball;
  double dda = (heading_towards_ball-tg_heading_dyn).get_deg_180();
  if (dda>15) heading_towards_ball_mod = tg_heading_dyn+Angle::deg_angle(15);
  if (dda<-15) heading_towards_ball_mod = tg_heading_dyn-Angle::deg_angle(15);
  Angle diff_head = (ballposfut-rloc.pos).angle (ballvel);

  if (abs(diff_head.get_deg_180())<20 && ballmovement.distance (rloc.pos)<200) {
    // Roboter bereits weitgehend hinter dem Ball => fahre direkt auf den Ball zu
    goto_pos_skill.init (ballposfut, heading_towards_ball, ballvel.length()+0.5);
    if (debug) LOUT << "Ballanfahrt hinten/Ballposition anfahren\n";
  } else if ((ballpos-rloc.pos)*norm>300 && abs((ballpos-rloc.pos)*ortho)<700) {
    // Roboter noch etwas seitlich hinter dem Ball => fahre etwas hinter die Ballposition,
    // um den Ball nicht mit den Hoernchen wegzudruecken
    goto_pos_skill.init (ballposfut-200*norm, heading_towards_ball, ballvel.length()+1.0);
    if (debug) LOUT << "Ballanfahrt hinten/etwas hinter Ballposition anfahren\n";
  } else {
    // Iterativ Zielposition berechnen
    bool conflict=false;  // wuerde die Bewegung einen Konflikt mit dem Ball erzeugen?
    bool wait=false;  // muss der Roboter noch warten, bis der Ball passiert hat?
    Vec shift = -(0.5*rp.robot_width+0.5*fg.ball_diameter+100)*ballmovement.side (rloc.pos)*ortho;
    Line conflict_line (ballpos+shift, ballpos+ballvel+shift);  // die Linie, hinter der es zu Konflikten mit dem Ball kommen kann
    Vec tg = ballposfut-600*norm;
    do {
      if ((rloc.pos-tg)*ballvel>0) {  // nicht nach hinten fahren, sondern im Zweifel neben der Schussrichtung warten, bis Ball ueberholt hat
        tg = ballmovement.perpendicular_point (rloc.pos)+shift;
        conflict=false;
        wait=true;
        break;
      }
      Line robotmovement (rloc.pos, tg);
      try{
        Vec pis = intersect (conflict_line, robotmovement);
        double dt = expected_time_to_pos (rloc.pos, pis, rloc.vtrans);
        Vec bp = expected_ballpos (ballpos, ballvel, dt);
        double vsp = (bp-pis)*norm;
        conflict = (vsp<rp.max_robot_radius+0.5*fg.ball_diameter);
      }catch(invalid_argument&) {
        conflict=false;  // Fall sollte nicht auftreten, wenn Roboter nicht hinter Ball
      }
      if (conflict)
        tg-=500*norm;  // Anfahrposition zurueckverlegen, um nicht in Konflikt mit dem Ball zu kommen
    } while (conflict);
    if (debug) LOUT << "Ballanfahrt hinten/Hinter Ball einschwenken\n";
    goto_pos_skill.init (tg, heading_towards_ball_mod, wait);
    if (ballvel.length()>1.0)
      goto_pos_skill.init_barrier (Line(tg, tg+ballvel));
  }
  // maximale Drehgeschwindigkeit:
//  double ttc = expected_time_to_pos (ballposfut, rloc.pos, rloc.vtrans);
//  double diffrad = (rloc.heading+Angle::quarter-ballvel.angle()).get_rad_pi();
//  double maxvrot2 = (abs(diffrad)+1.0)/ttc*1e3;
//  double truemaxvrot = maxvrot>maxvrot2 ? (maxvrot2>1.0 ? maxvrot2 : 1.0) : maxvrot;
//  goto_pos_skill.set_dynamics (maxvtrans, truemaxvrot);

  DriveVector dv = goto_pos_skill.getCmd (t);
  if ((robot_ball.length()<3000) && (ballvel.length()>1.0)) {
    if (debug) LOUT << "Neben Ball herbeschleunigen\n";
    dv.vtrans+=(1.5-1.5/(pow(ballvel.length(),3)+1))*aveballvel.normalize();  // magische Formel, asymptotisch 1.5, monoton steigend
  }
  return dv;
}
