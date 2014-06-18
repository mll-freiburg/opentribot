/*
 *  BPassSpontaneously.cpp
 *  robotcontrol
 *
 *  Created by Sascha Lange on 15.05.06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include "BPassSpontaneously.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/Frame2D.h"
#include "../../../Fundamental/random.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Behavior/Predicates/freeCorridor.h"
#include <vector>
#include <cmath>
#include <sstream>

namespace Tribots {
  
  using namespace std;
 
  BPassSpontaneously::BPassSpontaneously(int shortKickDuration, 
                                         int longKickDuration,
																				 double passProbability,
                                         bool passOnlyIfGoalBlocked) 
  : Behavior("BPassSpontaneously"),
    lastActivation(Time()), shortKickDuration(shortKickDuration), 
    longKickDuration(longKickDuration), target(Vec(0,0)), targetId(0),
    kicked(false),
	  passProbability(passProbability), incProbLead(9999), freeToPass(false), 
    passOnlyIfGoalBlocked(passOnlyIfGoalBlocked), hasBall(false)
  {}
  
  BPassSpontaneously::~BPassSpontaneously() throw ()
  {}
  
  void BPassSpontaneously::loseControl(const Time&) throw(TribotsException)
  {
    kicked = false;
  }
  
  void BPassSpontaneously::cycleCallBack(const Time& t) throw()
  {
    int diff = MWM.get_game_state().own_score - MWM.get_game_state().opponent_score;
    double probModifier = 0.;
    if (diff - incProbLead + 1 > 0.) {
      probModifier = .25 * (diff-incProbLead+1); // 25% higher prob for every goal starting from incProbLead
    }    
    if (!hasBall && WBOARD->doPossessBall(t)) {
      freeToPass = brandom(passProbability +  probModifier);
      LOUT << "BPassSpontaneously recalculated decision. prob=" 
           << passProbability << " modifier=" << probModifier << " decision: " 
           << freeToPass << endl;
    }
    hasBall = WBOARD->doPossessBall(t);   
  }
	
  void BPassSpontaneously::updateTactics(const TacticsBoard& tb) throw()
  {
    string key = "SpontanerPass";
    if (tb[key] == string("nie")) {
      passProbability = 0.;
    }
    else if (tb[key] == string("selten")) {
      passProbability = .25;
    }
    else if (tb[key] == string("manchmal")) {
      passProbability = .5;
    }
    else if (tb[key] == string("oft")) {
      passProbability = .75;
    }
    else {
      passProbability = 1.;
    }
    
    key = "OefterPassenBeiVorsprung";
    if (tb[key] == string("abEinTor")) {
      incProbLead = 1;
    }
    else if (tb[key] == string("abZweiToren")) {
      incProbLead = 2;
    }  
    else if (tb[key] == string("abDreiToren")) {
      incProbLead = 3;
    }  
    else if (tb[key] == string("abFuenfToren")) {
      incProbLead = 5;
    }  
    else {
      incProbLead = 9999;
    }  
    
    key = "PassenWenn";
    if (tb[key] == string("TorBlockiert")) {
      passOnlyIfGoalBlocked = true;
    }
    else {
      passOnlyIfGoalBlocked = false;
    }  

    LOUT << "BPassSpontaneously tactics changed. passProb=" << passProbability
         << " incProbLead=" << incProbLead << endl;
  }
  
  bool BPassSpontaneously::checkCommitmentCondition(const Time& t) throw()
  {
    return fabs((double)lastActivation.diff_msec(t)) < 750; // 3/4 sekunden wegfahren
  }
  
  bool BPassSpontaneously::checkInvocationCondition(const Time& t) throw()
  {
    if (!freeToPass) { return false; }
    
    const RobotLocation& robot = MWM.get_robot_location(t);
    const FieldGeometry& fgeom = MWM.get_field_geometry();
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
    const BallLocation& ball = MWM.get_ball_location(t);
    const vector<TeammateLocation>& teammates = MWM.get_teammate_location();
    unsigned int robotSelfId = MWM.get_robot_id(); 
    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);
    Frame2d robot2world = WBOARD->getRel2AbsFrame(t);
    
    if (! WBOARD->doPossessBall(t)) { // nicht passen, wenn keinen ball
      return false;
    }
    if (WBOARD->onlyOneRobot()) { // nicht passen, wenn nicht alle da
      return false;
    }
    if (robot.pos.y < -fgeom.field_length/4.) { // nicht passen,
      return false;                   // wenn zu nah am eigenen tor
    }
    
    // Spezialbedingung fuer Pass in die Mitte aus den Spielfeldecken
    if (robot.pos.y > fgeom.field_length/3. &&
        fabs(robot.pos.x) > fgeom.field_width/3. &&
        WBOARD->getActiveRobots() > 2 &&
        obstacle_distance_to_line(robot.pos, robot.pos+Vec(0,2000.)*robot.heading,
                                  obstacles, true) > 600.) {
      vector<Vec> intersections = intersect(
        LineSegment(robot.pos, robot.pos+Vec(0.,fgeom.field_length)*robot.heading),
        LineSegment(Vec(0.,0.),Vec(0., fgeom.field_length/2.-fgeom.penalty_area_length-500.)));
      if (intersections.size() > 0) {
        LOUT << "% yellow thik solid line " 
             << LineSegment(robot.pos, robot.pos+Vec(0.,4000.)*robot.heading)
             << endl;
        LOUT << "Pass aus Ecke in die Mitte moeglich!";
        // Moegliches Ziel finden
        vector<unsigned int> possibleTargets;
        for (double minAngle=10.; minAngle < 45. && !possibleTargets.size(); minAngle+=5.) {
          for (unsigned int i=0; i < teammates.size(); i++) {
            if (teammates[i].number == robotSelfId ||                // selber
                fabs((double)teammates[i].timestamp.diff_msec(t)) > 2000. || // info zu alt
                teammates[i].pos.y < 0.) {                           // eigene haelfte
              continue;
            }
            Vec relTeammatePos = world2robot * teammates[i].pos;
            if (relTeammatePos.y < 1200. ||  // Minimaler Abstand, Achtung: folgender check funzt nur fuer y > 0 (wegen dem x +/- 250 und dem +/-180 grad)!!!
                Vec(0.,1.).angle(relTeammatePos+Vec(-250., -250.)).get_deg_180() < -minAngle || 
                Vec(0.,1.).angle(relTeammatePos+Vec(+250., -250.)).get_deg_180() > +minAngle) {
              continue;
            }
            possibleTargets.push_back(i);
          }
        }
        if (possibleTargets.size()) {
          LOUT << "Moegliches Ziel gefunden, fuehre Pass aus." << endl;
          targetId = teammates[possibleTargets[0]].number;
          target = teammates[possibleTargets[0]].pos;
          return true;
        }
      }
    }
    
    // Ende Spezialbedingung Pass aus den Spielfeldecken
    
    //      <- +5       -5 ->
    //      \     |   |  X  / innerhalb
    //       \    |   |    /
    //        \   |   |   /  O ausserhalb
    //         \  |   |  /
    //          \ |   | /
    //           \|   |/
    //  Tribot:   XXXXX        Berechnung: Ueberpruefen des Winkels zur etwas
    //             XXX                     zur Seite verschobenen Blickrichtung
    //              X                      + minimaler Abstand

    LOUT << "% white thin line " << robot2world * LineSegment(Vec(0,0), Vec(0,10000.)).s_rotate(Angle::deg_angle(-5)).s_translate(Vec(+250,250)) << endl;
    LOUT << "% white thin line " << robot2world * LineSegment(Vec(0,0), Vec(0,10000.)).s_rotate(Angle::deg_angle(+5)).s_translate(Vec(-250,250)) << endl;
    vector<unsigned int> possibleTargets;
    for (unsigned int i=0; i < teammates.size(); i++) {
      if (teammates[i].number == robotSelfId ||                // selber
          fabs((double)teammates[i].timestamp.diff_msec(t)) > 2000. || // info zu alt
          teammates[i].pos.y < 0.) {                           // eigene haelfte
        continue;
      }
      Angle toRobotFront = 
      Vec(0.,1.).angle((robot.pos-teammates[i].pos)*teammates[i].heading);
      if (fabs(toRobotFront.get_deg_180()) > 90) {  // schaut nicht
        LOUT << "Passwinkel" << toRobotFront.get_deg_180() << endl;
        continue;
      }

      // Occupancy grid auswerten
      int freeProximity= teammates[i].occupancy_grid.occupied((robot.pos-teammates[i].pos).angle());
      if (freeProximity == 0) {
        LOUT << "% white dotted line "   << LineSegment(robot.pos, teammates[i].pos) << endl;
      }
      else if (freeProximity == 2) {
        LOUT << "% yellow dotted line "  << LineSegment(robot.pos, teammates[i].pos) << endl;
      }
      else {
        LOUT << "% red dotted line "     << LineSegment(robot.pos, teammates[i].pos) << endl;
      }
      if (freeProximity & 1 == 1) { // Nahbereich belegt, kein Pass moeglich
        LOUT << "% dark_red dotted line "<< LineSegment(robot.pos, teammates[i].pos) << endl;
        continue;
      }
      Vec relTeammatePos = world2robot * teammates[i].pos;
      if (relTeammatePos.y < 1500. ||  // Minimaler Abstand, Achtung: folgender check funzt nur fuer y > 0 (wegen dem x +/- 250 und dem +/-180 grad)!!!
          Vec(0.,1.).angle(relTeammatePos+Vec(-250., -250.)).get_deg_180() < -7.5 || 
          Vec(0.,1.).angle(relTeammatePos+Vec(+250., -250.)).get_deg_180() > +7.5) {
        LOUT << "% blue circle " << Circle(teammates[i].pos, 250.) << endl;
        continue;
      }
      bool passingLaneFree = true; 
      for (unsigned int o=0; o < obstacles.size(); o++) { // schauen, ob hindernisse den weg blockieren
        if (obstacles[o].player >= 0) continue;               // Mitspieler, nicht beruecksichtigen!
        Vec relO = world2robot * obstacles[o].pos;
        if (relO.y > 0 &&
            Vec(0.,1.).angle(relO+Vec(-250.-250., -250.)).get_deg_180() > -3 &&  // breite des corridors und breite des hindernisses (maximale)
            Vec(0.,1.).angle(relO+Vec(+250.+250., -250.)).get_deg_180() < +3 &&
            relO.length() < relTeammatePos.length()-(freeProximity & 2 > 0 ? 1000. : 1700.)) {  // vermutlich nicht der Mitspieler
          passingLaneFree = false;
          break;
        }
      }
      if (passingLaneFree) {
        LOUT << "% yellow thick circle  " << Circle(teammates[i].pos, 250.) << endl; // Pass moeglich
        possibleTargets.push_back(i);
      }
      else {
        LOUT << "% red solid circle " << Circle(teammates[i].pos, 250.) << endl; 
      }
    }
    if (possibleTargets.size() == 0) {
      return false;
    }
    // Pass ginge prinzipiell, jetzt testen, ob es sich auch lohnt (weg zum tor ist dicht)
    if (passOnlyIfGoalBlocked) {
      Quadrangle front(robot.pos + Vec(-600., 0.) * robot.heading,
                       robot.pos + Vec( 600., 0.) * robot.heading,
                       robot.pos + Vec( 600., 1000.) * robot.heading,
                       robot.pos + Vec(-600., 1000.) * robot.heading);
      Quadrangle toGoal(ball.pos.toVec() + Vec(-1000., -200.),
                        ball.pos.toVec() + Vec( 1000., -200.),
                        robot.pos.x > 0 ? Vec(fgeom.goal_width / 2. + 300., 
                                              fgeom.field_length / 2.) : 
                        Vec(0, fgeom.field_length/2.),
                        robot.pos.x > 0 ? Vec(0, fgeom.field_length/2.) :
                        Vec(-fgeom.goal_width / 2. - 300, fgeom.field_length/2.));
      XYRectangle goalyArea(Vec(-fgeom.goal_width/2.-100., 
                                fgeom.field_length/2),
                            Vec( fgeom.goal_width/2.+100., 
                                 fgeom.field_length/2 - 1000.));
      int frontCount = 0;
      int toGoalCount = 0;
      int goalyCount = 0;
      for (unsigned int i=0; i < obstacles.size(); i++) {
        if (obstacles[i].player >= 0) continue; // Mitspieler nicht beruecksichtigen
        if (front.is_inside(obstacles[i].pos)) {
          frontCount++;
        }
        if (toGoal.is_inside(obstacles[i].pos)) {
          if (goalyArea.is_inside(obstacles[i].pos)) {
            goalyCount++;     // das muesste der Torwart sein
          }
          else {
            toGoalCount++;    // vermutlich nicht der Torwart
          }
        }
      }
      LOUT << "\% thin white solid quadrangle " << front << endl;
      LOUT << "\% thin white solid quadrangle " << toGoal << endl;
      LOUT << "\% thin yellow solid quadrangle " << goalyArea << endl;
      if (toGoalCount == 0  && frontCount == 0 && 
          goalyCount <= 1) { // alles frei (maximal 1 Torwart ist da)
        return false;
      }
    }
    unsigned int closestId = possibleTargets[0];    
    for (unsigned int i=1; i < possibleTargets.size(); i++) {
      if ((robot.pos-teammates[closestId].pos).length() > 
          (robot.pos-teammates[possibleTargets[i]].pos).length()) {
        closestId = possibleTargets[i];
      }
    }
    targetId = teammates[closestId].number;
    target = teammates[closestId].pos;
    return true;
  }
  
  DriveVector BPassSpontaneously::getCmd(const Time& t) throw(TribotsException)
  {
    const RobotLocation& robot = MWM.get_robot_location(t);
    DriveVector dv;
    
    if (!kicked) {
      dv.vtrans = robot.vtrans / robot.heading;
      dv.vrot = 0;
      dv.kick = 2;
      dv.klength = (robot.pos-target).length() < 5000. ? shortKickDuration : longKickDuration;
      if (MWM.get_robot_id() == 8) dv.klength = 20;
      kicked = true;
      lastActivation = t;
      stringstream msg;
      msg << "pass: " << targetId;
      MWM.get_message_board().publish(msg.str());
      LOUT << "Spontaner Pass zu Spieler " << targetId << endl;
      WBOARD->resetPossessBall(); // Ballbesitz durch pass verloren
    }
    else {  // Kurz weg vom Passziel fahren, damits huebscher aussieht
      dv.vtrans = ((robot.pos - target).normalize()) / robot.heading; // 1m/s
      dv.vrot = 0; 
      dv.kick = 0;
    }
    
    return dv;
  }
  
}

