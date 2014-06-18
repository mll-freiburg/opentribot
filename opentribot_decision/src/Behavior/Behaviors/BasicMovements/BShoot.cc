#include "BShoot.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Structures/FieldGeometry.h"
#include <iostream>
#include <cmath>
#include <stdlib.h>
namespace Tribots
{

using namespace std;

BShoot::BShoot(int hackKickLength) : BShootImmediately("BShoot", hackKickLength)
{
  firstTimeLaneFree = Time();
  firstTimeLaneFree.add_sec(5);
  obstacles_behind_goal_distance = 300.0;
}

BShoot::~BShoot() throw()
{}

bool 
BShoot::checkCommitmentCondition(const Time& t) throw()
{
  return BShoot::checkInvocationCondition(t);
}


void BShoot::updateTactics (const TacticsBoard& tb) throw ()
{
  string val="";
  val = tb[string("obstacles_behind_goal_distance")];
  if (val!="") {
    obstacles_behind_goal_distance = atof(val.c_str());
  }
  LOUT << "obstacles_behind_goal_distance "
       << obstacles_behind_goal_distance << "\n";
}

bool 
BShoot::checkInvocationCondition(const Time& t) throw()
{
  Time                     tt_now;        
  Time                     tt_past;
  Time                     tt_sl;
  tt_past.add_msec(-100);
  FieldGeometry const&     fgeom            = MWM.get_field_geometry();
  RobotLocation            robot            = MWM.get_robot_location(tt_now);
  RobotLocation            robot_past       = MWM.get_robot_location(tt_past);

  //RobotLocation            robot_sl         = MWM.get_robot_location(tt_sl);
  // seems to be better
  RobotLocation            robot_sl         = MWM.get_slfilter_robot_location (tt_sl);

  //RobotProperties const&   robot_properties = MWM.get_robot_properties();
  Frame2d                  robot2world;
  robot2world.set_position(robot_past.pos);
  robot2world.set_angle(robot_past.heading);
  double                   shoot_dist     = 8000;// MWM.get_robot_properties().kickers > 1 ? 8000 : 8500;

  // - I ---- ballbesitz + n zyklen inaktiv ----------
  if (!BShootImmediately::checkInvocationCondition(tt_now)) 
    { 
//      LOUT << "BShootImmediately::checkInvocationCondition= false\n";
        firstTimeLaneFree = t;
        firstTimeLaneFree.add_sec(5);
      return false;   // mindestens erfüllt sein (ballbesitz + n zyklen inaktiv)
    }

  // Erlaubte Zielrichtung für Schuss: Torinnenraum mit Sicherheitsbereich
  Vec target= Vec(0., fgeom.field_length / 2.);
  LineSegment target_segment= LineSegment( target - Vec(fgeom.goal_width / 2. - 250.0, 0), target + Vec(fgeom.goal_width / 2. - 350.0 ,0));
  

  // - II ---- Roboter in Schussdistanz ---------
  if ( (robot.pos - target ).squared_length() > shoot_dist*shoot_dist ) {
    LOUT << "robot too far for shoot\n";
        firstTimeLaneFree = t;
        firstTimeLaneFree.add_sec(5);
    return false;
  }
  

  LOUT << "robot is near enough to shoot, checking shooting path ...\n";
  LOUT << "\% black thin circle " << robot.pos.x << " " << robot.pos.y << " 300\n";
  
  Vec head      = (Vec::unit_vector_y*300*10).rotate(robot.heading);
  Vec head_past = (Vec::unit_vector_y*300*10).rotate(robot_past.heading);

  LOUT << "\% yellow thin circle " << robot_past.pos.x << " " << robot_past.pos.y << " 300\n";
  LOUT << "robot_past.pos= " << robot_past.pos.x << " " << robot_past.pos.y << " head_past= " << head_past.x << " " << head_past.y << '\n';
  LOUT << "\% yellow thin line " << target_segment.p1.x << " " << target_segment.p1.y << " " << target_segment.p2.x << " " << target_segment.p2.y << '\n';
  

  // Ausrichtung des Roboters
  Vec  dum        = robot2world*(Vec(0,1)*10000);
  
  // Berechne Flugbahn des Balles
  // Annhame konstante Ballgeschwindigkeit 6000 mm/s nach Kick
  Vec  flugbahn   = Vec(0,6000.0);
  // addiere Komponente von Bahnbewegung bei Drehung des Roboters mit Ball (V=w*r)
  //flugbahn.x      =  - robot_past.vrot * 300.0;
  flugbahn.x      = - robot_sl.vrot * 300.0;
  LOUT << "Flugbahn rel: " << flugbahn << "\n";

  // addiere aktuelle Geschwindigkeit des Roboters in (relative) x-Richtung
  Vec robot_rel_vel = robot_past.vtrans / robot_past.heading;
  flugbahn.x       += robot_rel_vel.x * 1000.0;
  flugbahn = robot2world*(flugbahn.normalize()*10000.0);
  LOUT << "Flugbahn abs: " << flugbahn << "\n";

  Line shoot_line = Line(robot_past.pos, dum);
  Line shoot_line_flugbahn = Line(robot_past.pos, flugbahn);

  LOUT << "\n% yellow thin line " << robot_past.pos.x << " " << robot_past.pos.y 
       << " " << dum.x << " " << dum.y << "\n"; 
  
  LOUT << "\n% blue thin line " << robot_past.pos.x << " " << robot_past.pos.y 
       << " " << flugbahn.x << " " << flugbahn.y << "\n"; 
  
  // TODO: Hier mal was einbauen, was ueberprueft, ob die situation in zukunft noch besser wird (z.B. ueber drehrichtung oder ueber weltmodell). dann nur schiessen,
  //       wenn die aktuelle situation schon sehr gut ist, oder aber nicht mehr besser wird.
  Quadrangle shoot_quad= Quadrangle(robot_past.pos, dum , 350); // Korridor mit Ballbreite + Sicherheit
  LOUT << "\n\% yellow " << shoot_quad << endl;
  
  Quadrangle shoot_quad_flugbahn = Quadrangle(robot_past.pos, flugbahn , 350); // Korridor mit Ballbreite + Sicherheit
  LOUT << "\% blue " << shoot_quad_flugbahn << endl;

  // - III ---- Roboter Zielt auf Zielgebiet
  //if (!intersects_target_region(robot_past.pos, dum, target_segment))
  //  { 
  //    LOUT << "Target vector schneidet nicht das target segment\n";
  //    return false;
  //  }

  if (!intersects_target_region(robot_past.pos, flugbahn, target_segment))
    { 
      LOUT << "Target vector schneidet nicht das target segment\n";
        firstTimeLaneFree = t;
        firstTimeLaneFree.add_sec(5);
      return false;
    }

const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
      // - IV --- Hindernis im Schusskorridor
  for (unsigned int i=0; i< obstacles.size(); i++)
  {
    if (obstacles[i].pos.y >= fgeom.field_length/2.0 + obstacles_behind_goal_distance)
	continue; // check not for obst behind goal
    if ( shoot_quad_flugbahn.is_inside( obstacles[i].pos ) )
    {
        firstTimeLaneFree = t;
        firstTimeLaneFree.add_sec(5);
      return false;
    }
    else {
      double obstcordist = shoot_quad_flugbahn.distance(obstacles[i].pos);
      Vec closest = shoot_quad_flugbahn.closest_point(obstacles[i].pos);
      LOUT << "Obstacle not in Quad ... distance: " << obstcordist 
           << " ObstWidth: " << obstacles[i].width 
           << endl;
 //     LOUT << "\% white " << LineSegment(abs_obstacles.pos[i], closest) << endl;
      if( obstcordist < obstacles[i].width / 2.)  // TODO: bitte checken: habe hier halbe breite eingetragen (statt ganzer), weil das wohl gemeint ist?! (slange)
      {
        firstTimeLaneFree = t;
        firstTimeLaneFree.add_sec(5);
        return false;
      }
    }
  }

  // - V ---- Roboter hinter Tor ---------
  if ( (robot.pos.y > (fgeom.field_length/2) - 500) ) {
        firstTimeLaneFree = t;
        firstTimeLaneFree.add_sec(5);
    return false;
  }

  LOUT << "\% blue circle " << robot.pos.x << " " << robot.pos.y << " 100 \n";
  
  if (t.diff_msec(firstTimeLaneFree) > 100) 
  {
    return true;
  }
  
  if (firstTimeLaneFree > t) {
    firstTimeLaneFree = t;
  }
  
  return false;
}


bool BShoot::intersects_target_region(Vec robotpos, Vec new_pos, LineSegment const&  target) 
{
  Line shoot_line = Line(robotpos, new_pos);
  std::vector<Vec> res = intersect( target, shoot_line );
  if ( res.size() == 0 )
    return false;
  // Abfrage ob Schnittpunkt vor Roboter (sonst auch symmetrische Treffer in eigenes Tor möglich)
  if ( (res[0]-robotpos)*(new_pos-robotpos)  < 0 )
    return false;
  return true;
}





/*
  bool 
  BShoot::checkInvocationCondition(const Time& t) throw()
  {
  Time now;         // mit now fragen, da Schuß ziemlich schnell auslöst
  
  
  if (!BShootImmediately::checkInvocationCondition(now)) { // die müssen
  LOUT << "BShootImmediately::checkInvocationCondition= false\n";
  return false;   // mindestens erfüllt sein (ballbesitz + n zyklen inaktiv)
  }

  return check_art_invocation(t);
  
  bool freierSchusskorridor = WBOARD->isFreeShootCorridor(t);
  bool korrekteWinkeldifferenz=
    fabs(((WBOARD->getAbs2RelFrame(t) * WBOARD->getFreeGoalPos(t)).angle()
	  -Angle::quarter).get_deg_180()) <= 6; // 6 deg shoot tolerance

  bool nahGenugVormTor = 
    (WBOARD->getFreeGoalPos(t) - MWM.get_robot_location(t).pos).length() <= 
    3500;           // maximale schussdistanz TODO: ins config
 
  return freierSchusskorridor && korrekteWinkeldifferenz && nahGenugVormTor;  
}

bool BShoot::check_art_invocation(const Time& tt) {
   ObstacleLocation const& obstacles= MWM.get_obstacle_location(tt);
   FieldGeometry const& fgeom= MWM.get_field_geometry();
   Vec target= Vec(0., fgeom.field_length / 2.);
   LineSegment target_segment= LineSegment( target - Vec(1000.0 - 300.0, 0), target + Vec(1000.0 - 300.0 ,0));
   Time tt_past= tt;
   //tt_past.add_msec(-200);
   tt_past.add_msec(-100);
   RobotLocation robot= MWM.get_robot_location(tt);
   RobotLocation robot_past= MWM.get_robot_location(tt_past);
   double shoot_dist= 3500;

   
   if ( (robot.pos - target ).squared_length() > shoot_dist*shoot_dist ) {
      LOUT << "robot too far for shoot\n";
      return false;
   }

   LOUT << "robot is near enough to shoot, checking shooting path ...\n";
   LOUT << "\% black thin circle " << robot.pos.x << " " << robot.pos.y << " 300\n";
   Vec head = (Vec::unit_vector_y*300*10).rotate(robot.heading);
   Vec head_past = (Vec::unit_vector_y*300*10).rotate(robot_past.heading);
//   LOUT << "\% black thin line " << robot.pos.x << " " << robot.pos.y << " " << robot.pos.x + head.x << " " << robot.pos.y + head.y << '\n';
   LOUT << "\% yellow thin circle " << robot_past.pos.x << " " << robot_past.pos.y << " 300\n";
//   LOUT << "\% yellow thin line " << robot_past.pos.x << " " << robot_past.pos.y << " " << robot_past.pos.x + head_past.x << " " << robot_past.pos.y + head_past.y << '\n';
   LOUT << "robot_past.pos= " << robot_past.pos.x << " " << robot_past.pos.y << " head_past= " << head_past.x << " " << head_past.y << '\n';
   LOUT << "\% yellow thin line " << target_segment.p1.x << " " << target_segment.p1.y << " " << target_segment.p2.x << " " << target_segment.p2.y << '\n';
   bool shoot= check_for_shoot(
                               //WBOARD->getUnfilteredRobotLocation(tt),
                               robot_past,
                               target,
                               target_segment,
                               obstacles);
   return shoot;
}
  
bool BShoot::check_for_shoot(RobotLocation const& robotLocation, Vec target, LineSegment target_segment, ObstacleLocation const& abs_obstacles) {
  bool shoot= true;
  RobotProperties const& robot_properties= MWM.get_robot_properties();
  FieldGeometry const& fgeom= MWM.get_field_geometry();
  Frame2d robot2world;
  robot2world.set_position(robotLocation.pos);
  robot2world.set_angle(robotLocation.heading);

  Vec dum= robot2world*(Vec(0,1)*10000);
    
  Line shoot_line= Line(robotLocation.pos, dum);
  //DOUT << "\n\% yellow line " << robot.pos.x << " " << robot.pos.y << " " << dum.x << " " << dum.y;

  shoot= intersects_target_region(robotLocation.pos, dum, target_segment);
  if (!shoot)
  {
    LOUT << "\n% yellow thin line " << robotLocation.pos.x << " " << robotLocation.pos.y << " " << dum.x << " " << dum.y << "\n"; 
    LOUT << "Target vector schneidet nicht das target segment\n";
    return false;
  }
  
  Quadrangle shoot_quad= Quadrangle(robotLocation.pos, dum , 250); // Korridor mit Ballbreite + Sicherheit

  LOUT << "\n\% yellow line " << shoot_quad.p1.x << " " << shoot_quad.p1.y << " "
  << shoot_quad.p2.x << " " << shoot_quad.p2.y << " "
  << shoot_quad.p3.x << " " << shoot_quad.p3.y << " "
  << shoot_quad.p4.x << " " << shoot_quad.p4.y << " "
  << shoot_quad.p1.x << " " << shoot_quad.p1.y << "\n";

  for (unsigned int i=0; i< obstacles.pos.size(); i++) 
  {

    if (obstacles.pos[i].y >= fgeom.field_length/2.0 + 100.0) continue; // check not for obst behind goal

    if ( shoot_quad.is_inside( obstacles.pos[i] ) ) 
    {
      return false;
    }
    else {
     double obstcordist = shoot_quad.distance(obstacles.pos[i]);
     LOUT << "Obstacle not in Quad ... distance: " << obstcordist << " ObstWidth: " << obstacles.width[i] << "\n";
     if( obstcordist < obstacles.width[i])
     {
       return false;
     }
    }
  }
  //LOUT << "\n\% yellow line " << shoot_quad.p1.x << " " << shoot_quad.p1.y << " "
  //<< shoot_quad.p2.x << " " << shoot_quad.p2.y << " "
  //<< shoot_quad.p3.x << " " << shoot_quad.p3.y << " "
  //<< shoot_quad.p4.x << " " << shoot_quad.p4.y << " "
  //<< shoot_quad.p1.x << " " << shoot_quad.p1.y << "\n";
  //if ( shoot ) draw(shoot_quad,"yellow");
  return shoot;
}

bool BShoot::intersects_target_region(Vec robotpos, Vec new_pos, LineSegment const&  target) {
  Line shoot_line= Line(robotpos, new_pos);
  std::vector<Vec> res= intersect( target, shoot_line );
  if ( res.size() == 0 )
    return false;
  if ( (res[0]-robotpos)*(new_pos-robotpos)  < 0 )
    return false;
  return true;
}


*/
};
