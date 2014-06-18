/*
 *  BBefreiungsschlag.cpp
 *  robotcontrol
 *
 *  Created by Sascha Lange on 13.06.06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include "BBefreiungsschlag.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/Frame2D.h"
#include "../../../Fundamental/random.h"
#include "../../../Fundamental/geometry.h"
#include "../../Predicates/freeCorridor.h"
#include <vector>
#include <cmath>
#include <sstream>

namespace Tribots {
  
  using namespace std;
 
  BBefreiungsschlag::BBefreiungsschlag(int kickDuration, 
                                       double probability) 
  : Behavior("BBefreiungsschlag"),
  lastActivation(Time()), kickDuration(kickDuration),
  probability(probability), freeToKick(false), 
  hasBall(false), isBallInOwnHalf(false)
  {}
  
  BBefreiungsschlag::~BBefreiungsschlag() throw ()
  {}
  
  void BBefreiungsschlag::cycleCallBack(const Time& t) throw()
  {
    if (!hasBall && WBOARD->doPossessBall(t)) {
      double modifiedProbability = probability;
      if (MWM.get_robot_properties().kickers > 1) {
        LOUT << "This robot has the Harting kicker and therefore has a doubled probability to use BBefreiungsschlag" << endl;
        modifiedProbability *= 2.;
      }
      if (isBallInOwnHalf && timeBallEnteredOwnHalf.elapsed_sec() > 180) {
        LOUT << "Ball since more than 180s in our half, therefore activate BBefreiungsschlag" << endl;
        modifiedProbability = 1.;
      }
      else if (isBallInOwnHalf && timeBallEnteredOwnHalf.elapsed_sec() > 120) { 
        LOUT << "Ball since more than 120s in our half, increase probability of BBefreiungsschlag" << endl;
        modifiedProbability += .25;
      }
      freeToKick = brandom(modifiedProbability);
      LOUT << "BBefreiungsschlag recalculated decision:" 
           << freeToKick << endl;
    }
    hasBall = WBOARD->doPossessBall(t);   
    
    const BallLocation& ball = MWM.get_ball_location(t);
    // observe the ball in order to find out, the time when it entered our own half.
    if (MWM.get_game_state().refstate == freePlay) {
      if (isBallInOwnHalf) {
        if ((ball.pos_known == BallLocation::known ||
             ball.pos_known == BallLocation::communicated) &&
            ball.pos.y > 1000. && fabs(ball.pos.x) < MWM.get_field_geometry().field_width / 2.) {
          isBallInOwnHalf = false;
        }
      }
      else if ((ball.pos_known == BallLocation::known ||
                ball.pos_known == BallLocation::communicated) &&
               ball.pos.y < 0. && fabs(ball.pos.x) < MWM.get_field_geometry().field_width / 2.) {
        isBallInOwnHalf = true;
        timeBallEnteredOwnHalf = t;
      }
    }
  }
	
  void BBefreiungsschlag::updateTactics(const TacticsBoard& tb) throw()
  {
    string key = "Befreiungsschlag";
    if (tb[key] == string("nie")) {
      probability = 0.;
    }
    else if (tb[key] == string("selten")) {
      probability = .25;
    }
    else if (tb[key] == string("manchmal")) {
      probability = .5;
    }
    else if (tb[key] == string("oft")) {
      probability = .75;
    }
    else {
      probability = 1.;
    }

    LOUT << "BBefreiungsschlag tactics changed. passProb=" << probability
         << endl;
  }
  
  bool BBefreiungsschlag::checkCommitmentCondition(const Time& t) throw()
  { return false; }
  
  bool BBefreiungsschlag::checkInvocationCondition(const Time& t) throw()
  {
    if (!freeToKick) { return false; }
    if (t.diff_msec(lastActivation) < 500) {return false; }
    
    const RobotLocation& robot = MWM.get_robot_location(t);
    const FieldGeometry& fgeom = MWM.get_field_geometry();
    const BallLocation& ball = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry();
    Time tt_sl; // wird von naechstem aufruf mit sl zeit gefuellt
    const RobotLocation& robot_sl = MWM.get_slfilter_robot_location(tt_sl);
      const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);

    
//    const vector<TeammateLocation>& teammates = MWM.get_teammate_location();
//    unsigned int robotSelfId = MWM.get_robot_id(); 
//    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);
//    Frame2d robot2world = WBOARD->getRel2AbsFrame(t);
    
    if (! WBOARD->doPossessBall(t)) { // nicht passen, wenn keinen ball
      return false;
    }
    if (ball.pos.y > -500.) { // nicht passen,
      return false;           // wenn schon in anderer hälfte
    }
    // Schaut der Roboter grob Richtung Tor?
    LineSegment bl(Vec(-field.goal_width/2. - timeBallEnteredOwnHalf.elapsed_sec() > 180 ? -1000. : 0. , field.field_length/2.),
                   Vec(+field.goal_width/2. + timeBallEnteredOwnHalf.elapsed_sec() > 180 ? +1000. : 0. , field.field_length/2.));
    LineSegment blick(robot_sl.pos, robot_sl.pos + (Vec(0,field.field_length*1.5)*robot_sl.heading));
//    LOUT << "% yellow linesegment " << bl << endl;
//    LOUT << "% yellow linesegment " << blick << endl;
    if (intersect(bl, blick).size() <= 0) {
      return false;
    }
    if (obstacle_distance_to_line_inside_field(robot.pos, robot.pos+(Vec(0.,1.)*robot.heading)*3000., obstacles) < 250.) {
      if (MWM.get_robot_properties().kickers > 1) {
        // checken, ob drueber kicken funktionieren wuerde
        // koordinatensysem mapping entlang der schusslinie definieren
        int minX = static_cast<int>(fgeom.field_length);
        int minI = -1;
        for (unsigned int i=0; i < obstacles.size(); i++) {
          Frame2d abs2rel = WBOARD->getAbs2RelFrame(t);
          Vec relobs = abs2rel * obstacles[i].pos;
          if (relobs.y > 0 && fabs(relobs.x)-obstacles[i].width/2. < 300 && fabs(relobs.x) < minX) {
            minX = static_cast<int>(fabs(relobs.x));
            minI = i;
          }
        }
        if (minI >= 0) {
          bool isReachable = false;
          WBOARD->getKickLength(minX, 1000., &isReachable);
          if (!isReachable) {
            return false;
          }
        }
        return true; // sonst: kein stoerendes hindernis gefunden: kann schiessen
      }
      return false; // kein starker kicker, geht also nicht drueber
    }
    for (unsigned int i=0; i < obstacles.size(); i++) {
      if ((robot.pos - obstacles[i].pos).length() < 2000. &&
          obstacles[i].player <= 0) {
        return true;
      }
    }
    LOUT << "Befreiungsschlag: Wuerde aktiv werden, aber kein einziger Gegner in der Naehe" << endl;
    return false; // keine Hindernisse in der Naehe, kein Grund ihn weg zu schiessen
  }
  
  DriveVector BBefreiungsschlag::getCmd(const Time& t) throw(TribotsException)
  {
    const RobotLocation& robot = MWM.get_robot_location(t);
	unsigned int robotSelfId = MWM.get_robot_id();
    DriveVector dv;

    //Suche den Spieler der am weitesten im gegnerischen Feld steht

    const vector<TeammateLocation>& teammates = MWM.get_teammate_location();

    int target = -1;
    double largestY = -9999999.;

    for (unsigned int i=0; i < teammates.size(); i++) {
  		if (teammates[i].number == robotSelfId ||                    // selber
			fabs((double)teammates[i].timestamp.diff_msec(t)) > 2000) {  // eigene haelfte
			  continue;
	  	}
      if (teammates[i].pos.y > largestY) {
      	largestY = teammates[i].pos.y;
      	target = teammates[i].number;
      }
    }    
    
    dv.vtrans = (robot.vtrans / robot.heading);
    dv.vrot = 0;
    dv.kick = 1;
    dv.klength = kickDuration; // ToDo: Entfernen und entweder nix setzen (DriveVector-Konstruktor fuellt mit maximalem Schuss) oder allgemeine Moeglichkeit schaffen, die laenge des maximalen Schuss zu erfragen
    lastActivation = t;

    if (target > -1) {
      stringstream msg;
      msg << "BefreiungsschlagZu: " << target;
      MWM.get_message_board().publish(msg.str());
      LOUT << "Befreiungsschlag! zu Spieler " << target << endl;
    }
    else {
      LOUT << "Befreiungsschlag! aber kein target gefunden." << endl;
    }
    WBOARD->resetPossessBall(); // Ballbesitz durch pass verloren
    
    return dv;
  }  
}

