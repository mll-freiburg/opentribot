/*
 *  BDraufhalten.cpp
 *  robotcontrol
 *
 *  Created by Sascha Lange on 01.05.06.
 *  Copyright 2006 University of Osnabrueck. All rights reserved.
 *
 */

#define DEBUG_DRAUFHALTEN 

#include "BDraufhalten.h"
#include "../../../Fundamental/random.h"
#include "../../Predicates/freeCorridor.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Structures/Journal.h"
#include "../../../Fundamental/stringconvert.h"

namespace Tribots {

  using namespace std;
  
  BDraufhalten::BDraufhalten(double probability, int hackKickLength, double maxShootDistance, 
                             int cyclesFreeCorridor)
    : BShootImmediately("BDraufhalten", hackKickLength), 
      probability(probability),
      clearToShoot(false), doPossesBallLastTime(false),
      checkCycles(cyclesFreeCorridor),
      consecutiveCyclesFree(0), maxShootDistance(maxShootDistance)
  {
    if (maxShootDistance < 0) {
      this->maxShootDistance = MWM.get_robot_properties().kickers > 1 ? 9000. : 6000.; // mit dem starken Kicker schon frueher
    }
    if (this->maxShootDistance > MWM.get_field_geometry().field_length / 2.) { // erst ab Mittellinie
      this->maxShootDistance = MWM.get_field_geometry().field_length / 2.;
    }
    probability_orig=probability;
  }
  
  BDraufhalten::~BDraufhalten() throw () {
  }
  

void BDraufhalten::updateTactics (const TacticsBoard& tb)
    throw ()
{
// BDraufhalten = 0 .1 *.2 .3 .4 .5 .6 .7 .8 .9 1
  double d;
  if (string2double (d, tb[string("BDraufhalten")])) {
    if (d>=0 && d<=1) probability=d;
  }

  LOUT << "Tactics changed in BDraufhalten probability=" << probability << endl;
}

  void 
  BDraufhalten::cycleCallBack(const Time& t) throw () {
    try {
    
      // Neue Ballbesitzphase?
      bool ballPossesion = WBOARD->doPossessBall(t);
      if (! doPossesBallLastTime && ballPossesion) {// gerade den Ball erhalten?
        clearToShoot = brandom(probability);        // Verhalten diesmal aktiv?
        consecutiveCyclesFree = 0;                  // neue Berechnung anfangen
      }
      doPossesBallLastTime = ballPossesion;
    
      if (!clearToShoot || !ballPossesion) {        // da braucht jetzt nicht 
        return;                                     // lange rumgerechnet werden
      }
    
      // const RobotLocation& robot = MWM.get_robot_location(t);
      Time tt_sl;
      const RobotLocation& robot = MWM.get_slfilter_robot_location (tt_sl);

      const FieldGeometry& field = MWM.get_field_geometry();
    
      Vec goal(0, field.field_length/2.);
      Vec goalHalfBaseLine;
      if ((robot.pos-goal).length() < 2000.) {
        goalHalfBaseLine = Vec(field.goal_width / 2. - 100, 0.);
      }
      else {
        goalHalfBaseLine = Vec(field.goal_width / 2. - 200, 0.);
      }
      LineSegment shootLine(robot.pos, robot.pos + Vec(0,field.field_length*1.5) * robot.heading);
    
      vector<Vec> intersections = intersect(LineSegment(goal-goalHalfBaseLine,
                                                        goal+goalHalfBaseLine),
                                            shootLine);
#ifdef DEBUG_DRAUFHALTEN
      LOUT << "\% black thick " << LineSegment(goal-goalHalfBaseLine,
                                               goal+goalHalfBaseLine) << endl;
#endif
      if (intersections.size() == 0) {               // zielt nicht aufs Tor, 
        consecutiveCyclesFree = 0;                   // zurücksetzen und
        return;                                      // nicht weiter rechnen
      }
#ifdef DEBUG_DRAUFHALTEN
      if (robot.pos != intersections[0]) {
        LOUT << "\% black " << LineSegment(robot.pos, intersections[0]) << endl;
      }
#endif
    
      if (robot.vtrans.length() > .3) { // Wenn der Roboter selbst faehrt
        double angleDiff = 
        fabs(robot.vtrans.angle(Vec(0.,100.) * robot.heading).get_deg_180());
        // Wenn Roboter nicht ins Tor faehrt, und der Winkel zwischen Fahrtrichtung
        // und Ausrichtung bei einem Abstand von mehr als 2.5m groesser als 11.25
        // grad oder, wenn naeher als 2.5m,  groesser als 22.5 grad ist, dann
        // kann ma nicht schiessen!
#ifdef DEBUG_DRAUFHALTEN
        LOUT << "AngleDiff: " << angleDiff << endl;
        LOUT << "\% white dotted " 
          << LineSegment(robot.pos, robot.pos+robot.vtrans.normalize() * 10000.) 
          << endl;
#endif
        // nun testen, ob der Roboter ziemlich genau aufs Tor zu faehrt.
        if (robot.vtrans.length() > 0 &&    // exception beim zweiten LineSegment vermeiden
            intersect(LineSegment(goal-Vec(field.goal_width/2., 0.),
                                  goal+Vec(field.goal_width/2., 0.)),
                      LineSegment(robot.pos, robot.pos + robot.vtrans.normalize() * field.field_length*1.5 )).size() == 0 &&
            (angleDiff > 10  || (goal-robot.pos).length() > 2500) &&  // wenn kleiner 15 und nah vorm tor, dann ist dieser test false
            angleDiff > 7) {
          consecutiveCyclesFree = 0;
          return;
        }
      }
      
      // TODO: ueberpruefen, ob roboter den starken kicker hat und wenn ja, schauen, ob man vielleicht ueber die hindernisse drueber schiessen kann.
      // Wenn die Linie nicht frei ist, koennen ma nicht schiessen!
      /*if (obstacle_distance_to_line_inside_field(robot.pos, intersections[0],
                                                 MWM.get_obstacle_location(t)) 
          < 180.) {
        consecutiveCyclesFree = 0;
#ifdef DEBUG_DRAUFHALTEN
        LOUT << "Schusslinie blockiert" << endl;
#endif
        return;
      }
    */
      consecutiveCyclesFree++;
      LOUT << "\% blue thin " << LineSegment(robot.pos, intersections[0]) << endl;
    }catch(std::exception& e) {
      JWARNING ("BUG: Ungefangene Exception in BDraufhalten:");
      JWARNING (e.what());
    }
  }

  bool
  BDraufhalten::checkInvocationCondition(const Time& t) throw() {
    if (! BShootImmediately::checkInvocationCondition(t)) {
      return false;
    }
    const FieldGeometry& field = MWM.get_field_geometry();
    if ((Vec(0, field.field_length / 2.) - 
         MWM.get_robot_location(t).pos).length() > maxShootDistance) {
      return false;
    }
    LOUT << "BDraufhalten: consecutiveCyclesFree == " << consecutiveCyclesFree
         << " clearToShoot: " << clearToShoot << endl;
    return consecutiveCyclesFree >= checkCycles && clearToShoot;
  }
  
}

