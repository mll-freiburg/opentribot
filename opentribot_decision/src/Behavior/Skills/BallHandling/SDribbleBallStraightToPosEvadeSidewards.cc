#include "SDribbleBallStraightToPosEvadeSidewards.h"
#include "../../../Fundamental/Time.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Structures/GameState.h"
#include "../../../Fundamental/Frame2D.h"
#include "../../../Player/WhiteBoard.h"
#include "../../Predicates/freeCorridor.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"
#include <iostream>
#include <cmath>

namespace Tribots
{

using namespace std;
enum { left, right, neutral };
  
SDribbleBallStraightToPosEvadeSidewards::
SDribbleBallStraightToPosEvadeSidewards(double safetyMargin) throw()
	: Skill("SDribbleBallStraightToPosEvadeSidewards"),
    headingController(3., 0.0000 , 0.00, 3.5 , -3.5), 
    safety_margin(safetyMargin), saveInfight(true)
{}
	
SDribbleBallStraightToPosEvadeSidewards::~SDribbleBallStraightToPosEvadeSidewards() throw ()
{}
	
void SDribbleBallStraightToPosEvadeSidewards::updateTactics(const TacticsBoard& tb) throw()
{
  if (tb[string("KampfUmBall")] == string("NurEngAusweichen")) {
    saveInfight = true;
  }
  else {
    saveInfight = false;
  }
  LOUT << "Taktikupdate in SDribbleBallStraightToPosEvadeSidewards switched to saveInfight=" << (saveInfight ? "true" : "false") << endl; 
}

void SDribbleBallStraightToPosEvadeSidewards::gainControl(const Time& t) throw(TribotsException)
{
  tendency = neutral;
  startingPositionOfLastManeuver = MWM.get_robot_location(t).pos;
  timeOfLastSwitch = t;
  timeOfLastSwitch.add_msec(-1000);
}

void
SDribbleBallStraightToPosEvadeSidewards::setSafetyMargin(double margin)
{
  this->safety_margin = margin;
}

void
SDribbleBallStraightToPosEvadeSidewards::setTarget(const Vec& target,
                                                   const Vec& pointTo,
                                                   bool approachingGoal)
{
  this->target = target;
  this->pointTo = pointTo;
  this->approachingGoal = approachingGoal;
}

void
SDribbleBallStraightToPosEvadeSidewards::setTransVel(double transVel)
{
  this->transVel = transVel;
}


void
SDribbleBallStraightToPosEvadeSidewards::init_world_model_data(const Time& tt) {
  world2robot =  WBOARD->getAbs2RelFrame(tt);
  robot2world =  WBOARD->getRel2AbsFrame(tt);    
  abs_obstacles= &MWM.get_obstacle_location(tt);
  robot= &MWM.get_robot_location(tt);
  robot_properties= &MWM.get_robot_properties();
  fgeom= &MWM.get_field_geometry();
  oppGoalPos= Vec(0., fgeom->field_length / 2.);
  startpos= robot->pos+ (target- robot->pos).normalize()* 100;
  quad= Quadrangle(startpos, target,robot_properties->robot_width+safety_margin,2000);
  quad2world= Frame2d( startpos, (target- robot->pos).angle() - Angle::quarter);
  world2quad= quad2world;
  world2quad.invert();
  fieldarea= XYRectangle( Vec(- fgeom->field_width*0.5,-fgeom->field_length*0.5),
                          Vec(  fgeom->field_width*0.5, fgeom->field_length*0.5));
  behind_goalarea= Quadrangle( oppGoalPos, oppGoalPos+Vec(0,700),2000 );
}

DriveVector
SDribbleBallStraightToPosEvadeSidewards::getCmd(const Time& tt) 
  throw(TribotsException)
{  
  LOUT << "SDribbleStraightToPosEvadeSidewards::getCmd(" << tt << "):" << endl;
  init_world_model_data(tt);
  const ObstacleLocation& obstacles = MWM.get_obstacle_location(tt);
  ObstacleLocation rel_obstacles;
  fgeom= &MWM.get_field_geometry();
  
  double min_sqr_dist= -1;
  int idx_min= -1;
    
  // Hindernisse im Quadrangle merken (in relativen Koordinaten) und 
  // nahegelegenstes Hindernis im Quadrangle finden und merken in idx_min
  for (unsigned int i=0;i< obstacles.size();i++) {
    if ( quad.is_inside( obstacles.operator[](i).pos ) ) {
      ObstacleDescriptor od;
      od.pos=world2quad*obstacles.operator[](i).pos;
      od.width=obstacles.operator[](i).width;
      rel_obstacles.push_back(od);
      double sqr_dist= (robot->pos - obstacles.operator[](i).pos).squared_length();
      if (idx_min < 0 || sqr_dist < min_sqr_dist ) {
        min_sqr_dist= sqr_dist;
        idx_min= i;
      }
    }
  }
  
  // Logausgabe der Region und der wichtigen Hindernisse
  LOUT << "\% " << (idx_min >= 0 ? "dark_red" : "dark_green") 
       << " thin dotted " << quad << endl;

  for (unsigned int i=0; i< rel_obstacles.size(); i++) {
    Vec abs_pos = quad2world*rel_obstacles[i].pos;
    LOUT << "\% dark_green " << ((int)i==idx_min ? "" : "thin") << " solid circle "
         << abs_pos.x<<" "<< abs_pos.y<<" " << rel_obstacles[i].width*0.5
         << endl;
  }

  // Ausweichposition berechnen
  Vec new_pos= target;
  if ( idx_min >= 0 ) { 
    Vec abs_main_obstacle= obstacles.operator[](idx_min).pos;
//    double abs_main_obstacle_width = abs_obstacles->width[idx_min];
    new_pos= compute_next_position_to_approach(obstacles.operator[](idx_min).pos,
                                               obstacles.operator[](idx_min).width, 
																							 tt);
  }

  LOUT << "\% white circle " << new_pos.x << " " << new_pos.y << " 100" 
       << " white word " << new_pos.x + 240. << " " << new_pos.y + 160. 
       << " evasive_point" << endl;
  LOUT << "\% yellow circle " << target.x << " " << target.y << " 120" 
       << " yellow word " << target.x - 120. << " " << target.y + 180. 
       << " original_target " << endl;
  LOUT << "\% yellow arrow " << robot->pos.x << " " << robot->pos.y << " "
       << pointTo.x << " " << pointTo.y << endl;

  DriveVector dv;
  dv.kick = 0;

  double max_vel= transVel < 2.3 ? transVel : 2.3;  // mehr als 2.3 m/s geht leider im Moment noch nicht
  dv.vtrans = (world2robot*new_pos).normalize() * max_vel;
      
  LOUT << "\% dark_blue thick " << LineSegment(robot->pos, robot2world * (dv.vtrans*1000)) << endl;
  
#ifdef USE_ODESIM
  double _f1 = 3.0;
#else
  double _f1 = 1.;
  if (robot->pos.y < 0.) {
    _f1 = 1.2; // maximalwert von vrot bei 180 grad abweichung!
  }
  else if (robot->pos.y < 2000.) {
    _f1 = 1.6;
  }
  else if (robot->pos.y < 4000.) {
    _f1 = 1.8;
  }
  else _f1 = 2.0;
#endif
  
  PiecewiseLinearFunction plf;
  plf.addVertex(0.03,.8);
  plf.addVertex(0.1,1.0);
  plf.addVertex(1.0,2.4); 

  double dirDiff = ((world2robot*pointTo).angle()-Angle::quarter).get_deg_180() / 180.; 
  dv.vrot = plf.getValue(fabs(dirDiff))*_f1*     dirDiff/fabs(dirDiff);
  // Hier sollte jetzt die Anpassung der Ausrichtung passieren, fuer faelle
  // in denen die vorgegebene zielposition zu weit von der fahrtrichtung abweicht

  return dv;
}


static const char * get_active_tendency_str(int tendency) {
  switch (tendency) {
  case left: return "LEFT";
  case right: return "RIGHT";
  case neutral: return "NEUTRAL";
  }
  return "UNKNOWN -> this is BAD";
}

Vec SDribbleBallStraightToPosEvadeSidewards::compute_next_position_to_approach(Vec abs_main_obstacle, double abs_main_obstacle_width, const Time& t) {
  Vec new_pos= target;
   
  Quadrangle small_quad= 
    Quadrangle(startpos, target, 
               (robot_properties->robot_width + safety_margin + abs_main_obstacle_width));
    
  Vec rel_main_obstacle= world2quad* abs_main_obstacle;
  Vec rel_starting_pos = world2quad* startingPositionOfLastManeuver;

  Vec left_pos=  rel_main_obstacle + Vec(-(abs_main_obstacle_width+robot_properties->robot_width+safety_margin*(rel_main_obstacle.length() > 3000 ? 2. : 1.))*0.5,0);
  Vec right_pos=  rel_main_obstacle + Vec(+(abs_main_obstacle_width+robot_properties->robot_width+safety_margin*(rel_main_obstacle.length() > 3000 ? 2. : 1.))*0.5,0);

  left_pos= quad2world*left_pos;
  right_pos= quad2world*right_pos;
 
 /* if (left_pos.x < -fgeom->field_width / 2. + 500.) left_pos.x = -fgeom->field_width / 2. + 500.;
  if (right_pos.x > fgeom->field_width / 2. - 500.) right_pos.x = fgeom->field_width / 2. - 500.; */
  
  
  int newTendency = tendency;
  // First of all: push towards middle of field near the sideline
  if (robot->pos.x > fgeom->field_width * .5 - 800. ||
      right_pos.x > fgeom->field_width * .5 - 300.) {
    newTendency = left;
  }
  else if (robot->pos.y < -fgeom->field_width *.5 + 800. ||
           left_pos.x < -fgeom->field_width*.5 + 300.) {
    newTendency = right;
  }
  else if ( tendency == neutral) {      // haven't decided where to evade
    if (fabs(abs_main_obstacle.x) > fgeom->field_width / 2. - 1000. &&
        rel_main_obstacle.y > 700.) {
      newTendency = abs_main_obstacle.x > 0 ? left : right;
    }
    else {
      newTendency = rel_main_obstacle.x > 0 ? left : right;
    }
  }
  else if ( t.diff_msec(timeOfLastSwitch) > 300 &&
	    tendency == right && rel_main_obstacle.x > 100.) {   // should switch to left?
    newTendency = left;
  }
  else if ( t.diff_msec(timeOfLastSwitch) > 300 &&
	    tendency == left && rel_main_obstacle.x < -100.) {   // should switch to right?
    newTendency = right;
  }
  else if ( t.diff_msec(timeOfLastSwitch) > 500 &&
	    fabs(rel_starting_pos.x) > 3000.) {    // schon mehr als 3m seitlich ausgewichen
    newTendency = tendency == left ? right : left; // neutral has been decided earlier
  }
  
  LOUT << "Tendency after checking relative position: " << get_active_tendency_str(newTendency) << endl;
  
  // between 400 mm from the sidelines push towards middle any approaching robots (neutral allowed)
  if (newTendency == right && robot->pos.x > fgeom->field_width * .5 - 400.) {
    newTendency = left;
  }
  else if (newTendency == left && robot->pos.x < -fgeom->field_width * .5 + 400.) {
    newTendency = right;
  }

  LOUT << "Tendency after checking neutral zone near sidelines: " << get_active_tendency_str(newTendency) << endl;
  
  LOUT << "\% black circle " << Circle(left_pos, 50) << endl;
  LOUT << "\% black circle " << Circle(right_pos, 50) << endl; 

  if (approachingGoal && t.diff_msec(timeOfLastSwitch) > 300) {
    // Bei Anfahrt an Tor besondere Ueberlegung hinsichtlich der 
    // Ausweichposition anstellen
    XYRectangle goalyArea(Vec(-fgeom->goal_area_width / 2., 
                              fgeom->field_length / 2.+500.),
                          Vec( fgeom->goal_area_width / 2., 
                               fgeom->field_length / 2. - 1200.));
    if (robot->pos.y > fgeom->field_length / .6 && 
        goalyArea.is_inside(abs_main_obstacle)) {
      vector<Vec> leftS = intersect (Line(robot->pos, left_pos), 
                                     LineSegment(Vec(-fgeom->goal_width/2.+100,
                                                     fgeom->field_length/2.),
                                                 Vec( fgeom->goal_width/2.-100,
                                                      fgeom->field_length/2.)));
      vector<Vec> rightS= intersect (Line(robot->pos, right_pos), 
                                     LineSegment(Vec(-fgeom->goal_width/2.+100,
                                                     fgeom->field_length/2.),
                                                 Vec( fgeom->goal_width/2.-100,
                                                      fgeom->field_length/2.)));
      LineSegment leftG(robot->pos, Vec(-fgeom->goal_width/2.+300, fgeom->field_length/2));
      LineSegment rightG(robot->pos, Vec(fgeom->goal_width/2.-300, fgeom->field_length/2));
      double leftD = leftG.distance(abs_main_obstacle)-abs_main_obstacle_width / 2.;
      double rightD = rightG.distance(abs_main_obstacle)-abs_main_obstacle_width / 2.;
      
      LOUT << "leftD = " << leftD << " rightD = " << rightD << endl;
    
      if (leftS.size() > 0 || rightS.size() > 0) { // eine der ausweichpositionen fuehrt ins tor
        if (newTendency == left && leftS.size() == 0) {
          newTendency = right;
        }
        else if (newTendency == right && rightS.size() == 0) {
          newTendency = left;
        }
      } // ACHTUNG: Folgendes Umschalten macht es erst moeglich, unseren 
        //          Torwart auch im Simulator (ohne Schuss) auszutricksen
      else if (robot->pos.x < -700. && leftD < 300.) {
        newTendency = right;
      }
      else if (robot->pos.x > 700 && rightD < 300.) {
        newTendency = left;
      }
      else if (fabs(robot->pos.x) < 700 && tendency != neutral) {
        newTendency = tendency;
      }
      /* // folgendes abgewandelt wieder reinnehmen, wenn zu oft vorbei geschossen wird (zur zeit faehrt er absichtlich seitlich am tor vorbei)
	 else if (leftD < rightD) {
        tendency = right; // set tendency for the next cycle
        LOUT << "\n\% orange " << rightG << endl;  
        return Vec(fgeom->goal_width/2., fgeom->field_length/2);
      }
      else {
        tendency = left;
        LOUT << "\n\% orange " << leftG << endl;  
        return Vec(-fgeom->goal_width/2., fgeom->field_length/2);
      } */
    }
  }
  
  if (tendency != newTendency) {
    startingPositionOfLastManeuver = robot->pos;
    timeOfLastSwitch = t;
  }
  
  tendency = newTendency;
  LOUT << "Final tendency: " << get_active_tendency_str(tendency) << endl;
  
  if (tendency == left)
    new_pos= left_pos;
  else if ( tendency == right )
    new_pos= right_pos;
  else
    new_pos= target;

  /*
  // wenn hindernis relativ nah und noch direkt vorm roboter ist, dann staerker zur Seite fahren.
  if (rel_main_obstacle.y > 800. && rel_main_obstacle.y < 2500. &&
      fabs(rel_main_obstacle.x) < 700.) {
    LOUT << "main obstacle directly in front of robot" << endl;
    if (tendency == left) {
      new_pos = quad2world * Vec(-1500., 1500);
    }
    else if (tendency == right) {
      new_pos = quad2world * Vec(1500., 1500);
    }
  }*/
	
  if ((MWM.get_ball_location(t).pos.y < -fgeom->field_length/6. || saveInfight) && (tendency == left || tendency == right)) {
    // check, if engaged in an infight for ball possession
    // checks relative position and wheter or not this might be the goalie
    // TODO: has to be improved in training camp
    bool isInfight = 
    rel_main_obstacle.y < 600. && fabs(rel_main_obstacle.x) < 300. &&  // nah vorm botter
    !(abs_main_obstacle.y > fgeom->field_length/2.                     // ist nicht der torwart
      -fgeom->penalty_area_length &&
      fabs(abs_main_obstacle.x)<fgeom->penalty_area_width/2.) &&
    robot->vtrans.length() < 1500;                                     // wir fahren langsam
    if (isInfight) {
      // hier eine wesentlich enger ausweichen
      LOUT << "engaged in infight" << endl;
      new_pos = quad2world * (rel_main_obstacle + Vec(tendency == left ? -400. : 400., 100.)); 
      // sehr eng drad dran vorbei
      LOUT << "\% blue thick " << Circle(new_pos, 250.) << endl;
    }
  }
  
  return new_pos;
}

}
