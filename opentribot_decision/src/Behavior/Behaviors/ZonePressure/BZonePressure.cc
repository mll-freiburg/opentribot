/*
 *  BZonePressure.cpp
 *  robotcontrol
 *
 *  Created by Sascha Lange on 08.04.2007
 *  Copyright 2007 University of Osnabrueck. All rights reserved.
 *
 */

#include "BZonePressure.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "BDoubleTeam.h"
#include "BSupportDoubleTeamMiddle.h"
#include "BSupportDoubleTeamSideline.h"
#include "BProtectGoal.h"
#include "BSafety.h"
#include "BDefendBall.h"
#include <iostream>

using namespace std;

namespace Tribots {
  
  /** im Bereich der Tor-Tor-Linie sind beide Verteidiger vor dem Ball. Ab einem Meter Entfernung uebernimmt der Spieler auf der Seite.*/
  class BDefendBallConditioned : public BDefendBall {
  public:
    BDefendBallConditioned() : BDefendBall("BDefendBallConditioned")
    {}
    bool checkCommitmentCondition(const Time& t) throw() {
      const BallLocation& ballLocation = MWM.get_ball_location(t);
      
      if (! WBOARD->onlyOneRobot() &&
          WBOARD->getZonePressureRole() != "ballL" &&
          WBOARD->getZonePressureRole() != "ballR") {
        return false;
      }
      if ((WBOARD->onlyTwoRobots() || WBOARD->onlyThreeRobots()) &&
          WBOARD->getZonePressureRole() == "ballR") { // in diesem fall ist ballL am Ball
        return false;
      }
      //hier: 1 robot -> irgendeine Rolle, 2 oder 3 roboter -> ballL, 4 oder 5 roboter -> ballL oder ballR
      return 
        (WBOARD->onlyTwoRobots() || WBOARD->onlyThreeRobots() || WBOARD->onlyOneRobot() ||
         (WBOARD->getZonePressureRole() == "ballL" && ballLocation.pos.x < +1200.) ||    // minimale hysterese um zittern zu vermeiden
         (WBOARD->getZonePressureRole() == "ballR" && ballLocation.pos.x > -1200.)) &&  
        BDefendBall::checkCommitmentCondition(t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      const BallLocation& ballLocation = MWM.get_ball_location(t);
      
      if (! WBOARD->onlyOneRobot() &&
          WBOARD->getZonePressureRole() != "ballL" &&
          WBOARD->getZonePressureRole() != "ballR") {
        return false;
      }
      if ((WBOARD->onlyTwoRobots() || WBOARD->onlyThreeRobots()) &&
          WBOARD->getZonePressureRole() == "ballR") { // in diesem fall ist ballL am Ball
        return false;
      }
      //hier: 1 robot -> irgendeine Rolle, 2 oder 3 roboter -> ballL, 4 oder 5 roboter -> ballL oder ballR
      return 
        (WBOARD->onlyTwoRobots() || WBOARD->onlyThreeRobots() || WBOARD->onlyOneRobot() ||
         (WBOARD->getZonePressureRole() == "ballL" && ballLocation.pos.x < +1000.) ||
         (WBOARD->getZonePressureRole() == "ballR" && ballLocation.pos.x > -1000.)) && 
        BDefendBall::checkInvocationCondition(t);
    }
  };
  
  class BDoubleTeamConditioned : public BDoubleTeam {
  public:
    BDoubleTeamConditioned() : BDoubleTeam("BDoubleTeamConditioned")
    {}
    bool checkCommitmentCondition(const Time& t) throw() {
      
      if ((WBOARD->getZonePressureRole() != "ballL") &&
          (WBOARD->getZonePressureRole() != "ballR")) {
        return false;
      }
      if (WBOARD->onlyOneRobot() || WBOARD->onlyTwoRobots() || WBOARD->onlyThreeRobots()) {     // ballL macht alles
        return false;
      }
      return BDoubleTeam::checkCommitmentCondition(t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      
      if ((WBOARD->getZonePressureRole() != "ballL") &&
          (WBOARD->getZonePressureRole() != "ballR")) {
        return false;
      }
      if (WBOARD->onlyOneRobot() || WBOARD->onlyTwoRobots() || WBOARD->onlyThreeRobots()) {     // ballL macht alles
        return false;
      }
      return BDoubleTeam::checkInvocationCondition(t);
    }
  };
  
  class BSupportDoubleTeamMiddleConditioned : public BSupportDoubleTeamMiddle {
  public:
    BSupportDoubleTeamMiddleConditioned() : BSupportDoubleTeamMiddle(true, "BSupportDoubleTeamMiddleConditioned")
    {}
    bool checkCommitmentCondition(const Time& t) throw() {
      return BSupportDoubleTeamMiddleConditioned::checkInvocationCondition(t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      if (WBOARD->onlyTwoRobots() || WBOARD->onlyOneRobot()) {  // mindestens 3 Roboter, sonst safety
        return false;
      }
      return 
        (WBOARD->getZonePressureRole() == "left" ||
         WBOARD->getZonePressureRole() == "right") &&
        BSupportDoubleTeamMiddle::checkInvocationCondition(t);
    }
    DriveVector getCmd(const Time& t) throw(TribotsException)
    {
      const FieldGeometry& field = MWM.get_field_geometry();
      if (WBOARD->getZonePressureRole() == "left") {
        protectPos = Vec(-0.75*field.goal_width/2.,-field.field_length/2.);
      }
      else if (WBOARD->getZonePressureRole() == "right") {
        protectPos = Vec(+0.75*field.goal_width/2.,-field.field_length/2.);
      }
      else {
        protectPos = Vec(0, -field.field_length/2.);
      }
      return BSupportDoubleTeamMiddle::getCmd(t);
    }
  };
  
  class BSafetyConditioned : public BSafety {
  public:
    BSafetyConditioned() : BSafety("BSafetyConditioned")
  {}
    bool checkCommitmentCondition(const Time& t) throw() {
      return BSafetyConditioned::checkInvocationCondition(t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      if (WBOARD->getZonePressureRole() != "safety" &&
          !(WBOARD->onlyTwoRobots() &&
            WBOARD->getZonePressureRole() != "ballL" &&
            WBOARD->getZonePressureRole() != "ballR")) {
        return false;
      }
      return BSafety::checkInvocationCondition(t);
    }
    DriveVector getCmd(const Time& t) throw(TribotsException)
    {
      const FieldGeometry& field = MWM.get_field_geometry();
      if (!WBOARD->onlyOneRobot() &&
          !WBOARD->onlyTwoRobots()) { // in diesen Faellen mitte zu, sonst kurzer Winkel zu
        const BallLocation& ballLocation = MWM.get_ball_location(t);
        if (ballLocation.pos.x < 0) {
          protectPos = Vec(-0.85*field.goal_width/2.,-field.field_length/2.);
        }
        else {
          protectPos = Vec(+0.85*field.goal_width/2.,-field.field_length/2.);
        }
      }
      else {
        protectPos = Vec(0,-field.field_length/2.-500.);
      }
      return BSafety::getCmd(t);
    }
  };
  
  class BSupportDoubleTeamSidelineConditioned : public BSupportDoubleTeamSideline {
  private:
    bool isActive;
  public:
    BSupportDoubleTeamSidelineConditioned() : BSupportDoubleTeamSideline("SupportDoubleTeamSidelineConditioned"), isActive(false)
    {}
    bool checkCommitmentCondition(const Time& t) throw() {
      const BallLocation& ballLocation = MWM.get_ball_location(t);
      const FieldGeometry& field = MWM.get_field_geometry();
      if ((WBOARD->getZonePressureRole() != "left" &&
           WBOARD->getZonePressureRole() != "right") ||   // bleibt nur bei links und rechts an
          WBOARD->onlyTwoRobots() ||                      // mindestens 5 roboter, 
          WBOARD->onlyOneRobot() ||                       // also safety erforderlich...
          WBOARD->onlyThreeRobots() ||
          WBOARD->onlyFourRobots() ||
          WBOARD->teamPossessBall()) {            
        return false;
      }
      return
        ((WBOARD->getZonePressureRole() == "left" &&      // Hysterese: erst abschalten, wenn
          ballLocation.pos.x < -200) ||                   // ball deutlich in der mitte 
         (WBOARD->getZonePressureRole() == "right" &&
          ballLocation.pos.x > +200)) &&
        (ballLocation.pos.y > -field.field_length/2.+field.penalty_area_length+200.) &&
        BSupportDoubleTeamSideline::checkCommitmentCondition(t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      const BallLocation& ballLocation = MWM.get_ball_location(t);
      const FieldGeometry& field = MWM.get_field_geometry();
      if ((WBOARD->getZonePressureRole() != "left" &&
           WBOARD->getZonePressureRole() != "right") ||   // geht nur bei links und rechts an
          WBOARD->onlyTwoRobots() ||                      // mindestens 5 roboter, 
          WBOARD->onlyOneRobot() ||                       // also safety erforderlich...
          WBOARD->onlyThreeRobots() ||
          WBOARD->onlyFourRobots() ||
          WBOARD->teamPossessBall()) {                    // TODO: ausgehen, wenn erweiterter Ballbesitz(!), dann absichern
        return false;
      }
      
      return
        ((WBOARD->getZonePressureRole() == "left" &&      // ball muss deutlich auf der Seite des spielers sein, sonst
          ballLocation.pos.x < -500) ||                   // wird protect goal oder supportMiddle verwendet
         (WBOARD->getZonePressureRole() == "right" &&
          ballLocation.pos.x > +500))   &&
        (ballLocation.pos.y > -field.field_length/2.+field.penalty_area_length+500.) &&  // nicht mehr vor eigenem Strafraum
        BSupportDoubleTeamSideline::checkInvocationCondition(t);
    }
  };
  
  class BProtectGoalConditioned : public BProtectGoal {
  private:
    bool isActive;
  public:
    BProtectGoalConditioned() : BProtectGoal(true, "BProtectGoalConditioned"), isActive(false)
    {}
    bool checkCommitmentCondition(const Time& t) throw() {
      const BallLocation& ballLocation = MWM.get_ball_location(t);
      if ((WBOARD->getZonePressureRole() != "left" &&
           WBOARD->getZonePressureRole() != "right") ||   // nur bei links und rechts an bleiben
          WBOARD->onlyTwoRobots() ||                      // mindestens 3 roboter
          WBOARD->onlyOneRobot()) {
        return false;
      }
      return
        ((WBOARD->getZonePressureRole() == "left" &&      // Hysterese: erst abschalten, wenn
          ballLocation.pos.x < 500) ||                    // ball deutlich aus der mitte raus
         (WBOARD->getZonePressureRole() == "right" &&
          ballLocation.pos.x > -500)) &&
        BProtectGoal::checkCommitmentCondition(t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      const BallLocation& ballLocation = MWM.get_ball_location(t);
      if ((WBOARD->getZonePressureRole() != "left" &&
           WBOARD->getZonePressureRole() != "right") ||   // geht nur bei links und rechts an
          WBOARD->onlyTwoRobots() ||                      // mindestens 3 roboter
          WBOARD->onlyOneRobot()) {
        return false;
      }
      return
        ((WBOARD->getZonePressureRole() == "left" &&      // ball muss auf der Seite des spielers sein, sonst
          ballLocation.pos.x < 0) ||                      // wird supportMiddle verwendet
         (WBOARD->getZonePressureRole() == "right" &&
          ballLocation.pos.x > 0))   &&
        BProtectGoal::checkInvocationCondition(t);
    }
    DriveVector getCmd(const Time& t) throw(TribotsException)
    {
      const FieldGeometry& field = MWM.get_field_geometry();
      if (!WBOARD->onlyOneRobot() &&
          !WBOARD->onlyTwoRobots()) { // in diesen Faellen mitte zu, sonst kurzer Winkel zu
        if (WBOARD->getZonePressureRole() == "left") {
          protectPos = Vec(-0.85*field.goal_width/2.,-field.field_length/2.);
        }
        else { // "right"
          protectPos = Vec(+0.85*field.goal_width/2.,-field.field_length/2.);
        }
      }
      else {
        protectPos = Vec(0,-field.field_length/2.-500.);
      }
      return BProtectGoal::getCmd(t);
    }
  };
    

  bool ZonePressure::checkConditions(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);
    
    if (ball.pos_known == BallLocation::unknown ||
        ball.pos_known == BallLocation::raised) {   // TODO: Ist raised noch richtig? Was ist mit Baellen ausserhalb?
      return false;
    }
    return true;
  }
  
  
  // SupportSideline, wenn ball nicht auf der spielfeldseite in naehe der grundlinie 
  // !(ball ausserhalb strafraumbreite, naeher als strafraumoberkante an tor). 
  // sonst protect goal (auch kurze ecke decken). 
  // ebenfalls protect goal, wenn kein safety
  ZonePressure::ZonePressure() : BDIBehavior("FieldPlayer07ZonePressure")
  {
    addOption (new BDefendBallConditioned());
    addOption (new BDoubleTeamConditioned());
    addOption (new BSupportDoubleTeamSidelineConditioned());  // Spieler 2. Reihe Ballseite, aber nur wenn Safety
    addOption (new BProtectGoalConditioned());  // Spieler 2. Reihe (wenn nicht Support), kurzen Pfosten decken
    addOption (new BSupportDoubleTeamMiddleConditioned());    // Spieler 2. Reihe Ballgegenseite
    addOption (new BSafetyConditioned());       // Kurzen Pfosten an Strafraumgrenze decken
  }
  
  bool ZonePressure::checkCommitmentCondition(const Time& t) throw() {
    return checkConditions(t);
  }
  
  bool ZonePressure::checkInvocationCondition(const Time& t) throw() {
    return checkConditions(t);
  }

}

