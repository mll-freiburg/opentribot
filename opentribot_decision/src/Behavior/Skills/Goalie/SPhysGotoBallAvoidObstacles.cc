
#include "SPhysGotoBallAvoidObstacles.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

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

}


SPhysGotoBallAvoidObstacles::SPhysGotoBallAvoidObstacles () throw () {
  Time now;
  loseControl(now);  // zureucksetzen
}

SPhysGotoBallAvoidObstacles::~SPhysGotoBallAvoidObstacles () throw () {;}

void SPhysGotoBallAvoidObstacles::init (Vec h, double v, Angle th) throw () {
  init (h.angle()-Angle::quarter, v, th);
}

void SPhysGotoBallAvoidObstacles::init (Vec h, double v) throw () {
  init (h.angle()-Angle::quarter, v, Angle::deg_angle (15));
}

void SPhysGotoBallAvoidObstacles::init (Angle h, double v) throw () {
  init (h, v, Angle::deg_angle (15));
}

void SPhysGotoBallAvoidObstacles::init (Angle h, double v, Angle th) throw () {
  target_heading = h;
  target_velocity = v;
  approach_directly=false;
  heading_tolerance = th;
}

void SPhysGotoBallAvoidObstacles::init (double v) throw () {
  target_velocity = v;
  approach_directly=true;
}

void SPhysGotoBallAvoidObstacles::set_dynamics (double vl, double va, double al, double aa) throw () {
  goto_pos_skill.set_dynamics (vl, va, al, aa);
}

void SPhysGotoBallAvoidObstacles::set_dynamics (double vl, double va) throw () {
  goto_pos_skill.set_dynamics (vl, va);
}

void SPhysGotoBallAvoidObstacles::set_dynamics (double vl) throw () {
  goto_pos_skill.set_dynamics (vl);
}

void SPhysGotoBallAvoidObstacles::loseControl(const Time&) throw(TribotsException) {
  ball_was_rolling=false;
  remember_approach_dir=false;
  gotoBall=false;
}

Vec SPhysGotoBallAvoidObstacles::determineContactPoint (const Time& t) throw () {
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

DriveVector SPhysGotoBallAvoidObstacles::getCmd(const Time& t) throw() {
  // Fallunterscheidungen: wenn hinter dem Ball, dann Ballanfahrt, ansonsten fahre zu einem Punkt hinter dem Ball
  const RobotLocation& rloc = MWM.get_robot_location (t);
  const BallLocation& bloc = MWM.get_ball_location (t);
  const FieldGeometry& fg = MWM.get_field_geometry();
  const RobotProperties& rp = MWM.get_robot_properties();
  const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
  
  Vec ballpos = bloc.pos.toVec();
  Vec ballvel = bloc.velocity.toVec();
  Vec robot_ball = ballpos-rloc.pos;
  if (approach_directly)
    target_heading = robot_ball.angle()-Angle::quarter;
  bool rolling_ball = (robot_ball.length()<3000 ? (bloc.velocity.length()>(ball_was_rolling ? 0.3 : 0.5)) : (bloc.velocity.length()>(ball_was_rolling ? 0.6 : 0.8)));  // mit Hysterese
  ball_was_rolling = rolling_ball;
  if (rolling_ball) {
    // Fall eines rollenden Balls
    goto_pos_skill.set_ball_as_obstacle (false);  // da Ball sowieso in Bewegung
    Vec norm = ballvel.normalize();
    Vec ortho = norm.rotate_quarter();
    Vec ballposfut = ballpos;
    // ungefaehre Zeit bis zum Eintreffen des Roboters an der Ballposition berechnen (Berechnung in m,s,m/s,m/s^2), Endergebnis in ms:
    double expected_time_to_ball=0;
    for (unsigned int j=0; j<5; j++) {  // mehrere Iterationen durchfuehren, um Loesung zu approximieren
      expected_time_to_ball = expected_time_to_pos (rloc.pos, ballposfut-300*(ballposfut-rloc.pos).normalize(), rloc.vtrans);
      ballposfut = expected_ballpos (ballpos, ballvel, expected_time_to_ball);
    }
    if ((ballposfut-rloc.pos).length()>2000) {
      ballposfut+=300*ballvel; // bei grossen Entfernungen eine gewisse Unsicherheit beruecksichtigen
    }

    Line ballmovement (ballpos, ballpos+ballvel);
    // 4 Faelle unterscheiden
    ApproachDir dir = previous_approach_dir;
    double da = (ballvel.angle()-target_heading).get_deg();
    if (!remember_approach_dir) {
      if (da<=145 && da>=35)
        dir = hinten;
      if (da<215 && da>145)
        dir = links;
      if (da<=325 && da>=215)
        dir = vorne;
      if (da<35 || da>325)
        dir = rechts;
    } else if (robot_ball.length()>700) {
      // nur dann Anfahrtsentscheidung neu berechnen, wenn nicht nahe Ball
      // Hysterese beachten
      if (da<=135 && da>=45)
        dir = hinten;
      if (da<205 && da>155)
        dir = links;
      if (da<=315 && da>=225)
        dir = vorne;
      if (da<25 || da>335)
        dir = rechts;
    }
    remember_approach_dir=true;
    previous_approach_dir=dir;
    if (debug) LOUT << "% dark_red thick solid circle " << ballposfut << " 110\n";
    if (dir==hinten) {
      // Ball von hinten anfahren
      Vec ball_rel = (bloc.pos.toVec()-rloc.pos)/rloc.heading;
      bool push_by_fingers = (ball_rel.y<rp.kicker_distance+200) && (abs(ball_rel.x)<0.5*rp.robot_width+150) && (abs(ball_rel.x)>0.5*rp.kicker_width+100);
      if (push_by_fingers) {
        if (debug) LOUT << "Ball durch Dribblehoernchen weggedrueckt\n";
        DriveVector nv (Vec(0,0), 0, false);
        return nv;
      }

      Angle tg_heading_dyn = ballvel.angle()-Angle::quarter;
      Angle heading_towards_ball = (ballposfut-rloc.pos).angle()-Angle::quarter;
      double dda = (heading_towards_ball-tg_heading_dyn).get_deg_180();
      if (dda>15) heading_towards_ball = tg_heading_dyn+Angle::deg_angle(15);
      if (dda<-15) heading_towards_ball = tg_heading_dyn-Angle::deg_angle(15);
      Angle diff_head = (ballposfut-rloc.pos).angle (ballvel);
      if (abs(diff_head.get_deg_180())<15 && ballmovement.distance (rloc.pos)<150) {
        goto_pos_skill.init (ballposfut, heading_towards_ball, ballvel.length());
        if (debug) LOUT << "Anfahrt bewegter Ball von hinten, bereits hinter dem Ball\n";
      } else {
        // Iterativ Zielposition berechnen
        bool conflict=false;
        bool wait=false;
        Vec shift = -(0.5*rp.robot_width+0.5*fg.ball_diameter+100)*ballmovement.side (rloc.pos)*ortho;
        Line conflict_line (ballpos+shift, ballpos+ballvel+shift);  // die Linie, hinter der es zu Konflikten mit dem Ball kommen kann
        Vec tg = ballposfut-600*norm;
        do {
          if ((rloc.pos-tg)*ballvel>0) {  // nicht nach hinten fahren, sondern im Zweifel neben der Schussrichtung warten, bis Ball ueberholt hat
            tg = ballmovement.perpendicular_point (rloc.pos)+shift;
            conflict=false;
            wait=true;
            if (debug) LOUT << "Anfahrt bewegter Ball von hinten, noch vor dem Ball\n";
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
          else {
            if (debug) LOUT << "Anfahrt bewegter Ball von hinten, schraeg hinter dem Ball\n";
          }
        } while (conflict);
        goto_pos_skill.init (tg, heading_towards_ball, (wait ? 0.0 : ballvel.length()+1.0));
      }
      return goto_pos_skill.getCmd (t);
    }
    if (dir==vorne) {
      // Ball von vorne anfahren (intercepten)
      Vec pp = ballmovement.perpendicular_point (rloc.pos);
      if (ballvel*robot_ball<0) {
        goto_pos_skill.init (pp, ballvel.angle()+Angle::quarter, 0.0);
        if (debug) LOUT << "Anfahrt bewegter Ball von vorne, bereits vor dem Ball\n";
        return goto_pos_skill.getCmd (t);
      } else {
        goto_pos_skill.init (ballposfut-700*ortho*ballmovement.side(rloc.pos), ballvel.angle()-Angle::quarter, false);
        if (debug) LOUT << "Anfahrt bewegter Ball von vorne, noch hinter dem Ball\n";
        return goto_pos_skill.getCmd (t);        
      }
    }
    // Ball von der Seite anfahren
    double sd = (dir==rechts ? +1 : -1);
    double asd = ballmovement.side(rloc.pos);
    Vec stop_over_behind = ballpos-300*norm;
    Vec stop_over_side = ballposfut-700*ortho*sd;
    Angle hd = ballvel.angle()+(sd==-1 ? Angle::half : Angle::zero);
    if (sd!=asd) {
      goto_pos_skill.init (stop_over_behind, hd, false);
      if (debug) LOUT << "Anfahrt bewegter Ball von Seite, noch auf falscher Seite\n";
    } else if ((rloc.pos-ballposfut).angle(ballvel).in_between (Angle::five_eighth, Angle::zero) || (rloc.pos-ballposfut).angle(ballvel).in_between (Angle::zero, Angle::three_eighth)) {
      // der eigentlich interessante Fall, Anfahrt von der Seite, bereits auf der richtigen Seite
      Vec pballcontact=ballposfut;
      if ((rloc.pos-ballposfut)*ballvel>0)
        pballcontact = ballmovement.perpendicular_point (rloc.pos);
      Vec probotcontact=pballcontact-(rp.min_robot_radius+0.5*fg.ball_diameter)*sd*ortho;
      Vec probotwait=pballcontact-(rp.min_robot_radius+0.5*fg.ball_diameter+400)*sd*ortho;
      double exptime = expected_time_to_pos (rloc.pos, probotcontact, rloc.vtrans);
      Vec expball = expected_ballpos (ballpos, ballvel, exptime);
      bool wait = ((expball-pballcontact)*(ballpos-pballcontact)>0);  // warten, wenn Roboter zu frueh am Kontaktpunkt waere
      if (wait) {
        goto_pos_skill.init (probotwait, hd, 0.0);
      } else {
        goto_pos_skill.init (pballcontact, hd, target_velocity);
      }
      if (debug) LOUT << "Anfahrt bewegter Ball von Seite, direkte Anfahrt\n";
    } else {
      goto_pos_skill.init (stop_over_side, hd, target_velocity+0.5);
      if (debug) LOUT << "Anfahrt bewegter Ball von Seite, noch hinter Ball\n";
    }
    return goto_pos_skill.getCmd (t);
  } else {
    // Ball rollt nicht oder sehr langsam
    vector<ObstacleDescriptor>::const_iterator it;
    const unsigned int num_obstacles = obstacles.size();

    Vec point_behind_ball = ballpos+(0.5*fg.ball_diameter+rp.kicker_distance+350)*Vec::unit_vector(target_heading-Angle::quarter);  // Anfahrtpunkt hinter Ball bei gewuenschter Zielausrichtung
    for (unsigned int k=0; k<5; k++) {
      // maximal 5mal das Spielchen wiederholen, nach Hindernissen im Anfahrtbereich zu schauen
      // eine Heuristik, die auch schiefgehen kann, wenn mehrere Hindernisse beteiligt sind, daher Wiederholungsbegrenzung
      it = obstacles.begin();
      bool an_obstacle=false;
      for (unsigned int j=0; j<num_obstacles; j++) {
        if ((it->pos-point_behind_ball).squared_length()<it->width*it->width) {
          // Hindernis in der Naehe des Anfahrtpunktes, naeher untersuchen
          Quadrangle obstacle (it->pos, it->pos+it->width*(it->pos-rloc.pos).normalize(), it->width);
          Vec diag1 = (obstacle.p1-obstacle.p3).normalize();
          Vec diag2 = (obstacle.p2-obstacle.p4).normalize();
          double s = 1.5*rp.min_robot_radius;
          Quadrangle obstacle_free_space (obstacle.p1+s*diag1, obstacle.p2+s*diag2, obstacle.p3-s*diag1, obstacle.p4-s*diag2);
          Quadrangle obstacle_free_space2 (obstacle.p1+(s+1)*diag1, obstacle.p2+(s+1)*diag2, obstacle.p3-(s+1)*diag1, obstacle.p4-(s+1)*diag2);
          if (obstacle_free_space.is_inside (point_behind_ball)) {
            if (debug) LOUT << "Hindernis im Weg, verschiebe Ball-Anfahrtrichtung \n";
            an_obstacle=true;
            // Anfahrtswinkel verschieben, durch Schnittpunktbildung zwischen erweitertem Hindernisquadrat und Anfahrtskreis, siehe Tafel
            Circle behind_circle (ballpos, (point_behind_ball-ballpos).length());
            vector<Vec> pis = intersect (obstacle_free_space2, behind_circle);
            double len2 = 1e300;
            for (unsigned int i=0; i<pis.size(); i++) {
              if ((pis[i]-rloc.pos).squared_length()<len2) {
                point_behind_ball = pis[i];
                len2=(pis[i]-rloc.pos).squared_length();
              }
            }
            break; // ein Hindernis gefunden, was problematisch ist, dann noch mal von vorne alle Hindernisse durchsuchen
          }
        }
        it++;
      }
      if (!an_obstacle) break;
      target_heading = (point_behind_ball-ballpos).angle()+Angle::quarter;
    }
    Vec point_touching_ball = ballpos+(rp.kicker_distance)*Vec::unit_vector(target_heading-Angle::quarter);  // Anfahrtpunkt hinter Ball bei gewuenschter Zielausrichtung
    bool behind_ball = (abs(robot_ball.angle (ballpos-point_behind_ball).get_deg_180())<15);
    Angle heading_towards_ball = robot_ball.angle()-Angle::quarter;
    double dda = (heading_towards_ball-target_heading).get_deg_180();
    if (dda>15) heading_towards_ball = target_heading+Angle::deg_angle(15);
    if (dda<-15) heading_towards_ball = target_heading-Angle::deg_angle(15);
    bool heading_okay = (abs((rloc.heading-heading_towards_ball).get_deg_180())<=10);
    bool behind_stop_over = ((rloc.pos-ballpos).squared_length()>(point_behind_ball-ballpos).squared_length());
    if (behind_ball && heading_okay) { // Ballanfahrt, wenn Anfahrtrichtung und Ausrichtung stimmen
      goto_pos_skill.init (point_touching_ball, heading_towards_ball, target_velocity);
      goto_pos_skill.set_ball_as_obstacle (false);
    } else if (behind_ball && !behind_stop_over) { // erst richtig hindrehen, wenn zwar hinter Ball aber Ausrichtung noch nicht okay
      goto_pos_skill.init (rloc.pos, heading_towards_ball, target_velocity);
      goto_pos_skill.set_ball_as_obstacle (false);
    } else { // erst mal hinter den Ball fahren
      double stop_over_velocity = sqrt (1.2+target_velocity*target_velocity); // Maximalgeschwindigkeit am Zwischenpunkt, damit am Zielpunkt Wunschgeschwindigkeit erreicht werden kann
      goto_pos_skill.init (point_behind_ball, heading_towards_ball, stop_over_velocity);
      goto_pos_skill.set_ball_as_obstacle (true);
    }
    if (debug) LOUT << "Anfahrt ruhender Ball\n";
    return goto_pos_skill.getCmd (t);
  }
}

DriveVector SPhysGotoBallAvoidObstacles::getCmdNonMovingBall (const Time& t) throw () {
  // Ball rollt nicht oder sehr langsam
  const FieldGeometry& fg = MWM.get_field_geometry();
  const RobotProperties& rp = MWM.get_robot_properties();
  const RobotLocation& rloc = MWM.get_robot_location (t);
  const BallLocation& bloc = MWM.get_ball_location (t);
  const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
  Vec ballpos = bloc.pos.toVec();
  Vec robot_ball = ballpos-rloc.pos;
  vector<ObstacleDescriptor>::const_iterator it;
  const unsigned int num_obstacles = obstacles.size();
  Vec point_behind_ball = ballpos+(0.5*fg.ball_diameter+rp.kicker_distance+350)*Vec::unit_vector(target_heading-Angle::quarter);  // Anfahrtpunkt hinter Ball bei gewuenschter Zielausrichtung
  for (unsigned int k=0; k<5; k++) {
    // maximal 5mal das Spielchen wiederholen, nach Hindernissen im Anfahrtbereich zu schauen
    // eine Heuristik, die auch schiefgehen kann, wenn mehrere Hindernisse beteiligt sind, daher Wiederholungsbegrenzung
    it = obstacles.begin();
    bool an_obstacle=false;
    for (unsigned int j=0; j<num_obstacles; j++) {
      if ((it->pos-point_behind_ball).squared_length()<it->width*it->width) {
          // Hindernis in der Naehe des Anfahrtpunktes, naeher untersuchen
        Quadrangle obstacle (it->pos, it->pos+it->width*(it->pos-rloc.pos).normalize(), it->width);
        Vec diag1 = (obstacle.p1-obstacle.p3).normalize();
        Vec diag2 = (obstacle.p2-obstacle.p4).normalize();
        double s = 1.5*rp.min_robot_radius;
        Quadrangle obstacle_free_space (obstacle.p1+s*diag1, obstacle.p2+s*diag2, obstacle.p3-s*diag1, obstacle.p4-s*diag2);
        Quadrangle obstacle_free_space2 (obstacle.p1+(s+1)*diag1, obstacle.p2+(s+1)*diag2, obstacle.p3-(s+1)*diag1, obstacle.p4-(s+1)*diag2);
        if (obstacle_free_space.is_inside (point_behind_ball)) {
          if (debug) LOUT << "Hindernis im Weg, verschiebe Ball-Anfahrtrichtung \n";
          an_obstacle=true;
            // Anfahrtswinkel verschieben, durch Schnittpunktbildung zwischen erweitertem Hindernisquadrat und Anfahrtskreis, siehe Tafel
          Circle behind_circle (ballpos, (point_behind_ball-ballpos).length());
          vector<Vec> pis = intersect (obstacle_free_space2, behind_circle);
          double len2 = 1e300;
          for (unsigned int i=0; i<pis.size(); i++) {
            if ((pis[i]-rloc.pos).squared_length()<len2) {
              point_behind_ball = pis[i];
              len2=(pis[i]-rloc.pos).squared_length();
            }
          }
          break; // ein Hindernis gefunden, was problematisch ist, dann noch mal von vorne alle Hindernisse durchsuchen
        }
      }
      it++;
    }
    if (!an_obstacle) break;
    target_heading = (point_behind_ball-ballpos).angle()+Angle::quarter;
  }
  
  Angle target_approach_dir = target_heading+Angle::quarter;
  Angle actual_approach_dir = robot_ball.angle();
  bool approach_dir_reached = (target_approach_dir-actual_approach_dir).in_between (-heading_tolerance, heading_tolerance);
  bool headed_towards_ball = (abs((actual_approach_dir-rloc.heading-Angle::quarter).get_deg_180())<15);
  Angle stop_over_heading = actual_approach_dir-Angle::quarter;
  if ((actual_approach_dir-target_approach_dir).in_between (heading_tolerance, Angle::half))
    stop_over_heading = actual_approach_dir-Angle::quarter+heading_tolerance;
  else if ((actual_approach_dir-target_approach_dir).in_between (Angle::half, -heading_tolerance))
    stop_over_heading = actual_approach_dir-Angle::quarter-heading_tolerance;
  bool behind_stop_over = ((point_behind_ball-rloc.pos)*(point_behind_ball-ballpos)<0);

  if (debug) LOUT << "approach_dir_reached=" << approach_dir_reached << ", headed_towards_ball=" << headed_towards_ball << ", behind_stop_over=" << behind_stop_over << '\n';
  if ((gotoBall || approach_dir_reached) && headed_towards_ball) { // Ballanfahrt, wenn Anfahrtrichtung und Ausrichtung stimmen
    goto_pos_skill.init (ballpos, actual_approach_dir-Angle::quarter, target_velocity);
    goto_pos_skill.set_ball_as_obstacle (false);
    gotoBall = true;
    if (debug) LOUT << "Ruhender Ball/Ballposition anfahren\n";
  } else if (approach_dir_reached && !behind_stop_over) { // erst richtig hindrehen, wenn zwar hinter Ball aber Ausrichtung noch nicht okay
    goto_pos_skill.init (rloc.pos, actual_approach_dir-Angle::quarter, target_velocity);
    goto_pos_skill.set_ball_as_obstacle (false);
    if (debug) LOUT << "Ruhender Ball/richtig drehen\n";
  } else { // erst mal hinter den Ball fahren
    double stop_over_velocity = sqrt (2.5+target_velocity*target_velocity); // Maximalgeschwindigkeit am Zwischenpunkt, damit am Zielpunkt Wunschgeschwindigkeit erreicht werden kann
    goto_pos_skill.init (point_behind_ball, stop_over_heading, stop_over_velocity);
    goto_pos_skill.set_ball_as_obstacle (true);
    gotoBall = false;
    if (debug) LOUT << "Ruhender Ball/hinter Ball fahren\n";
  }
  return goto_pos_skill.getCmd (t);
}

DriveVector SPhysGotoBallAvoidObstacles::getCmdInterceptBall (const Time& t) throw () {
  goto_pos_skill.set_ball_as_obstacle (false);  // da Ball sowieso in Bewegung
  const RobotLocation& rloc = MWM.get_robot_location (t);
  const BallLocation& bloc = MWM.get_ball_location (t);
  Vec ballpos = bloc.pos.toVec();
  Vec ballvel = bloc.velocity.toVec();
  Line ballmovement (ballpos, ballpos+ballvel);
  Vec pp = ballmovement.perpendicular_point (rloc.pos);
  goto_pos_skill.init (pp, ballvel.angle()+Angle::quarter, 0.0);
  if (debug) LOUT << "Rollender Ball/Intercept\n";
  return goto_pos_skill.getCmd (t);
}

DriveVector SPhysGotoBallAvoidObstacles::getCmdFollowBall (const Time& t) throw () {
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
  
      // Ball von hinten anfahren
  Vec ball_rel = (bloc.pos.toVec()-rloc.pos)/rloc.heading;
  /*
  bool push_by_fingers = (ball_rel.y<rp.kicker_distance+200) && (abs(ball_rel.x)<0.5*rp.robot_width+150) && (abs(ball_rel.x)>0.5*rp.kicker_width+100);
  if (push_by_fingers) {
    if (debug) LOUT << "Ball durch Dribblehoernchen weggedrueckt\n";
    DriveVector nv (Vec(0,0), 0, false);
    return nv;
  }
  */
  
  Angle tg_heading_dyn = ballvel.angle()-Angle::quarter;
  Angle heading_towards_ball = (ballposfut-rloc.pos).angle()-Angle::quarter;
  Angle heading_towards_ball_mod = heading_towards_ball;
  double dda = (heading_towards_ball-tg_heading_dyn).get_deg_180();
  if (dda>15) heading_towards_ball_mod = tg_heading_dyn+Angle::deg_angle(15);
  if (dda<-15) heading_towards_ball_mod = tg_heading_dyn-Angle::deg_angle(15);
  Angle diff_head = (ballposfut-rloc.pos).angle (ballvel);
  if (abs(diff_head.get_deg_180())<20 && ballmovement.distance (rloc.pos)<200) {
    goto_pos_skill.init (ballposfut, heading_towards_ball, ballvel.length());
    if (debug) LOUT << "Ballanfahrt hinten/Ballposition anfahren\n";
  } else if ((ballpos-rloc.pos)*norm>300 && abs((ballpos-rloc.pos)*ortho)<700) {
    goto_pos_skill.init (ballposfut-200*norm, heading_towards_ball, ballvel.length());
    if (debug) LOUT << "Ballanfahrt hinten/etwas hinter Ballposition anfahren\n";    
  } else {
    // Iterativ Zielposition berechnen
    bool conflict=false;
    bool wait=false;
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
    goto_pos_skill.init (tg, heading_towards_ball_mod, (wait ? 0.0 : ballvel.length()+1.0));
  }
  return goto_pos_skill.getCmd (t);
}

