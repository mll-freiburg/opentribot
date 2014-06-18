/*
 *  BQuerpassToSupport.cpp
 *  robotcontrol
 *
 *  Created by Sven Kerkling on 25.04.07.
 *
 */

#include "BQuerpassToSupport.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/Frame2D.h"
#include "../../../Fundamental/random.h"
#include "../../../Fundamental/geometry.h"
#include <vector>
#include <cmath>
#include <sstream>

namespace Tribots {
  
  using namespace std;
 
  BQuerpassToSupport::BQuerpassToSupport(int shortKickDuration, 
					 double passProbability,
                                         bool passOnlyIfGoalBlocked) 
  : Behavior("BQuerpassToSupport"),
    lastActivation(Time()), shortKickDuration(shortKickDuration), 
    kicked(false), passProbability(passProbability), incProbLead(9999), freeToPass(false), 
    passOnlyIfGoalBlocked(passOnlyIfGoalBlocked), hasBall(false), turned(false)
  {}
  
  BQuerpassToSupport::~BQuerpassToSupport() throw ()
  {}
  
  void BQuerpassToSupport::loseControl(const Time&) throw(TribotsException)
  {
    kicked = false;
    turned = false;
  }
  
  void BQuerpassToSupport::gainControl(const Time& t) throw(TribotsException)
  {
    seite = WBOARD->getZonePressureRole()=="ballL"?-1:1;
    oldheading = MWM.get_robot_location(t).heading;
  } 
  
  void BQuerpassToSupport::cycleCallBack(const Time& t) throw()
  {
    int diff = MWM.get_game_state().own_score - MWM.get_game_state().opponent_score;
    double probModifier = 0.;
    if (diff - incProbLead + 1 > 0.) {
      probModifier = .25 * (diff-incProbLead+1); // 25% higher prob for every goal starting from incProbLead
    }    
    if (!hasBall && WBOARD->doPossessBall(t)) {
      freeToPass = brandom(passProbability +  probModifier);
      LOUT << "BQuerpassToSupport recalculated decision. prob=" 
           << passProbability << " modifier=" << probModifier << " decision: " 
           << freeToPass << endl;
    }
    hasBall = WBOARD->doPossessBall(t);   
  }
	
  void BQuerpassToSupport::updateTactics(const TacticsBoard& tb) throw()
  {
    string key = "QuerPass";
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

    LOUT << "BQuerpassToSupport tactics changed. passProb=" << passProbability
         << " incProbLead=" << incProbLead << endl;
  }
  
  bool BQuerpassToSupport::checkCommitmentCondition(const Time& t) throw()
  {
    return fabs((double)lastActivation.diff_msec(t)) < 750; // 3/4 sekunden wegfahren
  }
  
  bool BQuerpassToSupport::checkInvocationCondition(const Time& t) throw()
  {
    if (!freeToPass) { return false; }
    const RobotLocation& robot = MWM.get_robot_location(t);
    
    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);
    Frame2d robot2world = WBOARD->getRel2AbsFrame(t);
    if (! WBOARD->doPossessBall(t)) { // nicht passen, wenn keinen ball
      LOUT << "Kein Ballbesitz" << endl;
      return false;
    }
    LOUT << "Ballbesitz" << endl;
    if (WBOARD->getActiveRobots()<4) { // nicht passen, wenn nicht alle da
      LOUT << "Nicht genug Spieler" << endl;
      return false;
    }
    LOUT << "Min 4 Spieler" << endl;
    if (robot.pos.y < 1000.) { // nicht passen,
      LOUT << "Noch nicht weit genug in der gegnerischen Hälfte " << robot.pos.y << endl;
      return false;                   // wenn nicht inder gegnerischen haelfte
    }
    LOUT << "In der gegnerischen Hälfte: " << endl;
    
    if (robot.pos.x > 500){
      LOUT << "Auf der Rechten Seite und habe die Rolle: " << WBOARD->getZonePressureRole() << endl;  
      if (WBOARD->getZonePressureRole() != "ballR"){
        return false;
      }
    }else if (robot.pos.x < -500){
      LOUT << "Auf der Rechten Seite und habe die Rolle: " << WBOARD->getZonePressureRole() << endl;  
      if (WBOARD->getZonePressureRole() != "ballL"){
        return false;
      }
    }else {
      
      return false;
      LOUT << "zu weit in der mitte";
    }

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

    // Pass ginge prinzipiell, jetzt testen, ob es sich auch lohnt (weg zum tor ist dicht)
/*    if (passOnlyIfGoalBlocked) {
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
                                 fgeom.field_length/2 - 800.));
      int frontCount = 0;
      int toGoalCount = 0;
      int goalyCount = 0;
      for (unsigned int i=0; i < obstacles.pos.size(); i++) {
        if (front.is_inside(obstacles.pos[i])) {
          frontCount++;
        }
        if (toGoal.is_inside(obstacles.pos[i])) {
          if (goalyArea.is_inside(obstacles.pos[i])) {
            goalyCount++;     // das muesste der Torwart sein
          }
          else {
            toGoalCount++;    // vermutlich nicht der Torwart
          }
        }
      }
      LOUT << "\% thin white solid " << front << endl;
      LOUT << "\% thin white solid " << toGoal << endl;
      LOUT << "\% thin yellow solid " << goalyArea << endl;
      if (toGoalCount == 0  && frontCount == 0 && 
          goalyCount <= 1) { // alles frei (maximal 1 Torwart ist da)
        return false;
      }
    }*/
    return true;
  }
  
  DriveVector BQuerpassToSupport::getCmd(const Time& t) throw(TribotsException)
  {
    const RobotLocation& robot = MWM.get_robot_location(t);
    const BallLocation& ball = MWM.get_ball_location(t);
    DriveVector dv;
    Angle ang1;
    Angle ang2;
    if(seite < 0){
      ang1 = Angle::deg_angle(-90.);
      ang2 = Angle::deg_angle(-80.);
    }else{
      ang1 = Angle::deg_angle(80.);
      ang2 = Angle::deg_angle(90.);
    }
    if(robot.pos.y < 0){
      dv.vtrans = Vec(0.,2.);
      return dv;
    }
    if(robot.heading.in_between(ang1,ang2))
      turned = true;
    LOUT << "Ausrichtung: " << robot.heading.get_deg_180() << " " << ang1.get_deg_180() << " " << ang2.get_deg_180() << " " << endl;    
    if(!turned){
      dv.vtrans=(robot.vtrans) / robot.heading;
      dv.vtrans = dv.vtrans.normalize()*2.0;
      dv.vrot=seite*3.0;
      oldheading = robot.heading;
    }else if (!kicked && turned) {
      dv.vtrans = (robot.vtrans) / robot.heading;
      dv.vrot = 0;
      dv.kick = 1;
      dv.klength = 30;
      kicked = true;
      lastActivation = t;
      WBOARD->resetPossessBall(); // Ballbesitz durch pass verloren
    }
    else {  // Kurz weg vom Passziel fahren, damits huebscher aussieht
      dv.vtrans = ((robot.pos - ball.pos.toVec() ).normalize()) / robot.heading; // 1m/s
      dv.vrot = 0; 
      dv.kick = 0;
    }
    
    
    
    return dv;
  }
}
