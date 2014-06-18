/*
 *  BFarShoot.cc
 *  robotcontrol
 *
 *  Created by roha
 *  Copyright 2006 University of Osnabrueck. All rights reserved.
 *
 */

#include "BFarShoot.h"
#include "../../../Fundamental/random.h"
#include "../../Predicates/freeCorridor.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Structures/Journal.h"
#include "../../../Fundamental/stringconvert.h"

namespace Tribots {

  using namespace std;
  
  BFarShoot::BFarShoot(double _probability)
    : Behavior("BFarShoot"), probability(_probability), duration(60), activationLevel(OPP_IN_FRONT)
  {
    mayBecomeActive=false;
    m_bDecisionMade=false;
  }
  
  BFarShoot::~BFarShoot() throw () 
  {
    ;
  }
  
  void BFarShoot::updateTactics (const TacticsBoard& tb) throw ()
  {
    // BFarShootProb [0 - 1]
    double d;
    if (string2double (d, tb[string("BFarShootProb")])) {
      if (d>=0 && d<=1) probability=d;
    }
    
    // Bolzen = sofort *sobaldBedraengt gegnerImTrichter gegnerInSchusslinie aus
    if (tb[string("Bolzen")] == "sofort") {
      activationLevel = ALWAYS;
    } else if (tb[string("Bolzen")] == "sobaldBedraengt") {
      activationLevel = IF_PRESSURED;
    } else     if (tb[string("Bolzen")] == "gegnerInSchusslinie") {
      activationLevel = OPP_IN_FRONT;
    } else     if (tb[string("Bolzen")] == "aus") {
      activationLevel = NEVER;
    } else {
      activationLevel = OPP_IN_FUNNEL;
    } 
    
    LOUT << "Tactics changed in BFarShoot probability=" << probability << endl;
  }
  
  void BFarShoot::cycleCallBack(const Time& t) throw () 
  {
    if (! WBOARD->doPossessBall(t)) {
      m_bDecisionMade = false;
    }
    else if (m_bDecisionMade == false) {
      mayBecomeActive = brandom(probability);
      m_bDecisionMade = true;
    }
  }
  
  bool BFarShoot::checkInvocationCondition(const Time& t) throw() 
  {
    if (activationLevel == NEVER) {
      return false;
    }
    
    Time t_sl;

    // nur ausf¸hren wenn starker kicker (Harting-Kicker)
   /* if(MWM.get_robot_properties().kickers != 2) {
      LOUT << "No far shoot, only 1 kicker.";
      return false;
    }*/  //  Für neue roboter nicht mehr nötig	
    // wenn Ball ... 
    if (! WBOARD->doPossessBall(t)) return false; 

    // --- random Verhalten
    if (!mayBecomeActive) return false; 
    
    const RobotLocation& robot    = MWM.get_robot_location(t);
    const FieldGeometry& field    = MWM.get_field_geometry();
    const RobotLocation& robot_sl = MWM.get_slfilter_robot_location(t_sl);

    Frame2d              robot2world;
    //robot2world.set_position(robot.pos);
    //robot2world.set_angle(robot.heading);

    robot2world.set_position(robot_sl.pos);
    robot2world.set_angle(robot_sl.heading);

    // -- schaut ins Tor?
    double eps_mm = - 400;

    Vec world_pos_goal(0, field.field_length/2.);    
    LineSegment world_target_segment ( world_pos_goal - Vec(field.goal_width / 2. + eps_mm , 0.) , 
				       world_pos_goal + Vec(field.goal_width / 2. + eps_mm , 0.) );

    // --- berechne korrigierte Flugbahn des Balles
    Vec  flugbahn   = Vec(0,12000.0);
    // ---- addiere Komponente von Bahnbewegung bei Drehung des Roboters mit Ball (V=w*r)
    flugbahn.x      = - robot_sl.vrot * 400.0;
    // ---- addiere aktuelle Geschwindigkeit des Roboters in (relative) x-Richtung
    //Vec robot_rel_vel = robot.vtrans / robot.heading;
    //flugbahn.x     += robot_rel_vel.x * 100.0;
    flugbahn        = robot2world*(flugbahn.normalize()*20000.0);

    LOUT << "\n% blue thick line " << robot_sl.pos.x << " " << robot_sl.pos.y 
	 << " " << flugbahn.x << " " << flugbahn.y << "\n"; 

    Line shoot_line_flugbahn = Line(robot_sl.pos, flugbahn);
    std::vector<Vec> intersec = intersect( world_target_segment , shoot_line_flugbahn );
    if (intersec.size() == 0) return false;

    // TODO: Abfrage ob richtig ausgerichtet (Schnittpunkt Linie, Segment auch nach Hinten mˆglich)
    if ( (intersec[0]-robot_sl.pos)*(flugbahn-robot_sl.pos)  < 0 )
      return false;

    Quadrangle shoot_quad_flugbahn = Quadrangle(robot_sl.pos, flugbahn , 500); // Korridor mit Ballbreite + Sicherheit
    LOUT << "\% blue thick " << shoot_quad_flugbahn << endl;

    // - nur ab 2m vor und in  gegnerischer H~lfte
    if (robot.pos.y < -3000) return false;


    // - teste auf Hindernisse in Schusskoridor
    // -- starte mit 60 ms und finde passende Zeit f¸r Schuss ¸ber Hindernisse und ins Tor
    bool obs_present = false;
    bool above_goal  = false;
    bool above_obs   = false;
    duration    = 61;
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
    
    do {
      duration--;
      above_goal = false;
      above_obs  = true;
      for (unsigned int i=0; i< obstacles.size(); i++)
	{
	  if ( shoot_quad_flugbahn.is_inside( obstacles[i].pos ) )
	    {
	      obs_present = true;
	      if ( WBOARD->getKickTargetHeight( (obstacles[i].pos - robot_sl.pos).length() , duration ) < 700 ) {
		LOUT << "\% red thick circle " << obstacles[i].pos << " 300" << endl;
		above_obs = false;
	      } 
	      LOUT << "\% green thick circle " << obstacles[i].pos << " 300" << endl;
	    }
	}
      if ( WBOARD->getKickTargetHeight( ( intersec[0] - robot_sl.pos ).length() , duration ) > 750 ) 
	above_goal = true;
    } while( duration>25 && obs_present && ( above_goal || !above_obs ) );

    // - wenn Hindernis in Flugbahn und ¸berschossen werden kann, schiesse
    if (obs_present && !above_goal && above_obs) {
      LOUT << "BFarShoot will shoot with " << duration << " ms\n";
      return true;
    }
    // - wenn Hindernis in Flugbahn und nicht ¸berschossen werden kann nicht schiessen
    if (obs_present) {
      LOUT << "BFarShoot will not shoot, obstacle present in shoot path but no duration found for shooting over it in goal\n";
      return false;
    }
    
    // hier kann man sicher sein, dass nix im weg steht (oder man drueber kommt), also suchen wir uns die maximal moegliche schussstaerke unter die latte
    
    bool reachable = true;
    duration = WBOARD->getKickLength((intersec[0]-robot_sl.pos).length(), 800, &reachable);

    if (activationLevel == ALWAYS) {
      return true;
    }

    if (activationLevel == OPP_IN_FUNNEL || activationLevel == IF_PRESSURED) { // Trichter checken
      // keine Hindernisse in Schusskoridor, dann teste auf Hindernisse zwischen Roboter und Tor
      Quadrangle way_to_goal(robot.pos , Vec(0, field.field_length/2. - 1000.0) , 
                             800 , (robot.pos - Vec(0, field.field_length/2. - 1000.0)).length());
      LOUT << "\% green thin " << way_to_goal; 
      LOUT << "\n (BFarShoot: way_to_goal_quadrangle (green thin))\n";
      for (unsigned int i=0; i< obstacles.size(); i++) {
        if (activationLevel == IF_PRESSURED) { // umgebung checken
          if ((robot.pos-obstacles[i].pos).length() < 1500 && obstacles[i].player < 0) { // Gegner innerhalb Kreis mit Radius 1500mm
            LOUT << "BFarShoot will shoot , opponent in a circle with a radius of 1500mm\n";
            return true;
          }
        }
	      if ( way_to_goal.is_inside(obstacles[i].pos) ) { // trichter nach vorne checken
	        if ( obstacles[i].velocity.length() > 0.5 ) {
	          Line robot_goal(robot.pos , Vec(0, field.field_length/2.));
                  LineSegment obst_path(obstacles[i].pos , (obstacles[i].pos + obstacles[i].velocity).normalize() * 10000.0 );
	          std::vector<Vec> res = intersect (obst_path, robot_goal);
	          if (res.size() == 0) continue;
	        }
	        LOUT << "BFarShoot will shoot , obstacle in way to goal quadrangle\n";
	        return true;
	      }
      }
    }
    
    return false;
  }
  
  bool BFarShoot::checkCommitmentCondition(const Time& t) throw()
  {
    bool res = false;

    // wenn kein Ball ... 
    if (! WBOARD->doPossessBall(t)) return false; 

    return res;
  }

  DriveVector BFarShoot::getCmd(const Time&) throw(TribotsException)
  {
    DriveVector dv(0,0,0,0);

    dv = DriveVector(Vec(0.0 , 2.5) , 0.0 , HIGHKICK , duration);

    return dv;
  }

}
