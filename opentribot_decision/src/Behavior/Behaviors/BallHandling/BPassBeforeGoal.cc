#include "BPassBeforeGoal.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
//#include "../../Skills/BallHandling/SDribbleBallToPosRL.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include "../../../Fundamental/random.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace Tribots
{
  using namespace std;

  BPassBeforeGoal::BPassBeforeGoal(int kickDuration, double PassProbability) 
    : Behavior("BPassBeforeGoal"),
      skill(new SPass(kickDuration)),
      targetPos(Vec(0,0)),
      targetId(0),
      passProbability(passProbability),
			lostcounter(10),
			freeToPass(false),
			hasBall(false),
      alreadyCommunicated(false),
			kickDuration(kickDuration),
      waitPhase(false),
      messageID(0), sendCounter(0)
  {

    const FieldGeometry& field = MWM.get_field_geometry();
		allowedPositionRight = Quadrangle(Vec(field.field_width/2., field.field_length/2.),
                                Vec(field.field_width/2., field.field_length/4.),
                                Vec(field.penalty_area_width/2., field.field_length/4.),
                                Vec(field.penalty_area_width/2., field.field_length/2.));
		allowedPositionLeft = Quadrangle(Vec(-field.field_width/2., field.field_length/2.),
                                Vec(-field.field_width/2., field.field_length/4.),
                                Vec(-field.penalty_area_width/2., field.field_length/4.),
                                Vec(-field.penalty_area_width/2., field.field_length/2.));
	}


  BPassBeforeGoal::~BPassBeforeGoal() throw()
  {
    if (skill) {
      delete skill;
    }
  }
	
  void BPassBeforeGoal::updateTactics (const TacticsBoard& tb) 
    throw () 
  {
    // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
          transVel = 2.0;
    } else      
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
         transVel = 1.5;
    } else
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
         transVel = 2.7;
    } else { // =normal
         transVel = 2.3;
    }

    string key = "QuerPass";
    if (tb[key] == string("immer")) {
      passProbability = 1.;
    }
    else if (tb[key] == string("oft")) {
      passProbability = 0.7;
    }
    else if (tb[key] == string("manchmal")) {
      passProbability = 0.4;
    }
    else if (tb[key] == string("selten")) {
      passProbability = 0.2;
    }
    else {
      passProbability = 0.;
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

  }

	void BPassBeforeGoal::cycleCallBack(const Time& t) throw() {
		skill->cycleCallBack(t);
		//avoid reactivation for a few cycles
		lostcounter++;
    
		//------- copied from querpass ----------
		int diff = MWM.get_game_state().own_score - MWM.get_game_state().opponent_score;
    double probModifier = 0.;
    if (diff - incProbLead + 1 > 0.) {
      probModifier = .25 * (diff-incProbLead+1); // 25% higher prob for every goal starting from incProbLead
    }    
    if (!hasBall && WBOARD->doPossessBall(t)) {
      freeToPass = brandom(passProbability +  probModifier);
    }
    hasBall = WBOARD->doPossessBall(t); 
		
	}

  void BPassBeforeGoal::gainControl(const Time& t) throw(TribotsException)
  {
		skill->gainControl(t);
    waitPhase = false;
    stringstream msg;
    msg << "prepareForQuerpass: " << targetId << " " << messageID;
    MWM.get_message_board().publish(msg.str());
  }

  void BPassBeforeGoal::loseControl(const Time& t) throw(TribotsException)
  {
		skill->loseControl(t);
		//wenn die kontrolle verloren wurde, ein paar frames warten, bis wieder an
		lostcounter = 0;
    alreadyCommunicated = false;
		stringstream msg;
    if (!waitPhase) {
      msg << "passbreak!";
      MWM.get_message_board().publish(msg.str());
      LOUT << "BPassBeforeGoal: Pass abgerochen!, verliere die Kontrolle" << endl;
    }
    //dont wait on reentry
    waitPhase = false;
  }

  DriveVector
  BPassBeforeGoal::getCmd(const Time& t) throw(TribotsException)
  {
    const RobotLocation& robot = MWM.get_robot_location(t);
    if (waitPhase) {
      double ms;
      LOUT << "In waitphase since " << (ms = waitSince.elapsed_msec()) << "milliseconds" << endl;
      DriveVector dv;
      dv.vtrans = Vec(0.,0.);
      dv.vrot = 0;
      dv.kick = 0;
      return dv;
    }
    
    if (!alreadyCommunicated) {  // Kommunikation vorbereiten, indem die messageID hochgesetzt wird
      messageID++;
      sendCounter = 0;
    }
    
    if (fabs((Vec(0.,1.)*robot.heading).angle(targetPos-robot.pos).get_deg_180()) < 40 && // kurz vor abspiel kommunizieren (winkel zum ziel bereits ziemlich klein).
        (!alreadyCommunicated || (sendCounter++ % 3 == 0 && sendCounter < 45))) { // fuer 1,5s jeden 3. Zyklus die Nachricht wiederholen
      //notify the target robot about pass
      stringstream msg;
      msg << "querpass: " << targetId << " " << messageID;
      MWM.get_message_board().publish(msg.str());
      LOUT << "BPassBeforeGoal: Gezielter Pass zu Spieler " << targetId << ". " << (sendCounter / 3 + 1) << ". Wiederholung der Nachricht." << endl;
      LOUT << "Text der gesendeten Nachricht: " << msg.str() << endl;
      alreadyCommunicated = true;
    }
		
		bool leftside = allowedPositionLeft.is_inside(robot.pos);
		bool rightside = allowedPositionRight.is_inside(robot.pos);

		//feststellen, in welche richtung gedreht wird
		bool turnleft;
		if (robot.vrot >0) {
			turnleft = true;
		} else {
			turnleft = false;
		}

		//allowed angle errors in degrees
		double errorLeft = 0.0;
		double errorRight = 0.0;

		if ((!turnleft && leftside)) {
			errorLeft = 1.0;
			errorRight = 4.0;
		}
		if ((turnleft && leftside)) {
			errorLeft = 1.0;
			errorRight = 4.0;
		}
		if ((!turnleft && rightside)) {
			errorLeft = 4.0;
			errorRight = 1.0;
		}
		if ((turnleft && rightside)) {
			errorLeft = 4.0;
			errorRight = 1.0;
		}

    //invoke passing skill (RL)
    skill->setParameters(targetPos, transVel, errorLeft, errorRight);

    DriveVector dv = skill->getCmd(t);
    if (dv.kick) { LOUT << "Kicked, starting waitPhase" << endl; waitPhase = true; waitSince.update(); }
    return dv;
  }

  bool BPassBeforeGoal::checkCommitmentCondition(const Time& t) throw()
  {
    const RobotLocation& robot = MWM.get_robot_location(t);
    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);
    Frame2d robot2world = WBOARD->getRel2AbsFrame(t);
     const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
    
    if (waitPhase) { // stay active for three seconds after kick in order to wait at present position
      if (waitSince.elapsed_sec() < 3) {
        return true;
      }
      else {
        return false;
      }
    }

    if (!WBOARD->doPossessBall(t)) {
      LOUT << "BPassBeforeGoal not activated, reason: don't have the ball." << endl;
      return false;
    }
 
		//check if robot still exists and update position
		bool exists = false;
    const vector<TeammateLocation>& teammates = MWM.get_teammate_location();
		unsigned int number =0;

    for (unsigned int i=0; i < teammates.size(); i++) {
      if (teammates[i].number == targetId) {
				exists = true;
				number = i;
				targetPos = teammates[i].pos;
				LOUT << "BPassBeforeGoal: passpartner: " << i << " position: " << targetPos << endl;
			}
		}
		
    if (!exists) {
      //no passing target found
      LOUT << "BPassBeforeGoal not activated, reason: passpartner not there anymore." << endl;
      return false;
    } else {
			//check if passpartner still ok
			if ((targetPos-robot.pos).length() < 2000) {
				// roboter zu nah
				LOUT << "BPassBeforeGoal deactivated, reason: passpartner too close." << endl;
        LOUT << "% red solid circle " << Circle(targetPos, 600.) << endl;
				LOUT << "% red word " << Vec(500,0) + targetPos <<"too_close\n" << endl;
        return false;
      }

			if (targetPos.y < 0.) {
				// eigene Haelfte
				LOUT << "BPassBeforeGoal: deactivated, reason: passpartner in my own half, too risky" << endl;		
        LOUT << "% red solid circle " << Circle(targetPos, 600.) << endl;
				LOUT << "% red word " << Vec(500,0) + targetPos <<"own_half\n" << endl;
        return false;
			}

      Vec relTeammatePos = world2robot * targetPos;
			Angle robotHeading = robot.heading;
      Angle targetHeading = (targetPos - robot.pos).angle();
      double hdiff = (robotHeading - (targetHeading-Angle::quarter)).get_deg();
      //assimodulo
      if (hdiff > 180) {
        hdiff -= 360;
      }

      //in the passing area, check if pass still possible
      if (fabs(hdiff) < 10) { 
  			//can i still pass to him?
        int freeProximity= teammates[number].occupancy_grid.occupied((robot.pos-targetPos).angle());
        bool passingLaneFree = true;
        // schauen, ob hindernisse den weg blockieren,
        // mitspieler eingeschlossen (haben keine zeit mehr wegzufahren)
        for (unsigned int o=0; o < obstacles.size(); o++) { 
          Vec relO = world2robot * obstacles[o].pos;
          if (relO.y > 0 &&
              Vec(0.,1.).angle(relO+Vec(-250.-250., -250.)).get_deg_180() > -3 &&  // breite des corridors und breite des hindernisses (maximale)
              Vec(0.,1.).angle(relO+Vec(+250.+250., -250.)).get_deg_180() < +3 &&
              relO.length() < relTeammatePos.length()-(freeProximity & 2 > 0 ? 1000. : 1700.)) {  // vermutlich nicht der Mitspieler
            passingLaneFree = false;
  					// print the ostacle on screen
	  				LOUT << "% black solid circle " << Circle(obstacles[o].pos, 300.) << endl;
            break;
          }
        }
        
        bool angleSmallEnough = fabs(((robot.pos-targetPos).angle() - Angle::quarter).get_deg_180()) < 135;
        LOUT << "angleSmallEnough: " << angleSmallEnough << " (" << fabs(((robot.pos-targetPos).angle() - Angle::quarter).get_deg_180()) << ")" << endl;
        
        
        if (passingLaneFree && angleSmallEnough) {
          LOUT << "BPassBeforeGoal: pass immernoch moeglich zu roboter: " << targetId << endl;
          LOUT << "% blue solid circle  " << Circle(targetPos, 600.) << endl; // Pass moeglich
          LOUT << "% blue word " << Vec(500,0) + targetPos << "passpartner\n" << endl;
  				return true;
        } else {
          LOUT << "BPassBeforeGoal: pass nichtmehr moeglich zu roboter: " << targetId << endl;
          LOUT << "% red solid circle " << Circle(targetPos, 600.) << endl;
		  		LOUT << "% red word " << Vec(500,0) + targetPos << "passing lane occupied\n" << endl;
			  	return false;
  			}
      } else {
        LOUT << "BPassBeforeGoal: still not in target area continuing: " << fabs(hdiff) << endl;
      }
    } 
	
    return true;
  }

  bool BPassBeforeGoal::checkInvocationCondition(const Time& t) throw()
  {
    const FieldGeometry& field = MWM.get_field_geometry();
    const RobotLocation& robot = MWM.get_robot_location(t);
     const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
    const vector<TeammateLocation>& teammates = MWM.get_teammate_location();
    unsigned int robotSelfId = MWM.get_robot_id();
    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);
    Frame2d robot2world = WBOARD->getRel2AbsFrame(t);

		bool leftside = allowedPositionLeft.is_inside(robot.pos);
		bool rightside = allowedPositionRight.is_inside(robot.pos);


    if (!leftside && !rightside) {
      //LOUT << "BPassBeforeGoal not activated, reason: not inside a valid area." << endl;
      return false;
    }

    if (! WBOARD->doPossessBall(t)) {
      //LOUT << "BPassBeforeGoal not activated, reason: don't have the ball." << endl;
      return false;
    }
 

		//hold back, stick to probability setting
		if (!freeToPass) {
      LOUT << "BPassBeforeGoal not activated, reason: probability too low." << endl;		
			return false;
		}
	
		if (lostcounter < 7) {
      LOUT << "BPassBeforeGoal not activated, reason: hysteresis. staying inactive for 5 cycles." << endl;		
			return false;
		}

    vector<unsigned int> possibleTargets;
    for (unsigned int i=0; i < teammates.size(); i++) {
			LOUT << "BPassBeforeGoal: analyzing robot: " << teammates[i].number << "." << endl;		

      if (teammates[i].number == robotSelfId) {
				LOUT << "BPassBeforeGoal: " << teammates[i].number << " not a partner, its me" << endl;		
				continue;
			}
			if (fabs((double)teammates[i].timestamp.diff_msec(t)) > 2000.) {
				// info zu alt
				LOUT << "BPassBeforeGoal: " << teammates[i].number << " not a partner, info too old" << endl;		
        LOUT << "% red solid circle " << Circle(teammates[i].pos, 600.) << endl;
				LOUT << "% red word " << Vec(500,0) + teammates[i].pos <<"info_too_old\n" << endl;
				continue;
			}
			if (teammates[i].pos.y < (field.field_length/2. - field.penalty_area_length) / 4.) {
				// eigene Haelfte
				LOUT << "BPassBeforeGoal: " << teammates[i].number << " not a partner, own half" << endl;		
        LOUT << "% red solid circle " << Circle(teammates[i].pos, 600.) << endl;
				LOUT << "% red word " << Vec(500,0) + teammates[i].pos <<"own_half\n" << endl;
        continue;
			}
			if ((teammates[i].pos-robot.pos).length() < 3000) {
				// roboter zu nah
				LOUT << "BPassBeforeGoal: " << teammates[i].number << " not a partner, too close" << endl;		
        LOUT << "% red solid circle " << Circle(teammates[i].pos, 600.) << endl;
				LOUT << "% red word " << Vec(500,0) + teammates[i].pos <<"too_close\n" << endl;
        continue;
      }
			if (teammates[i].pos.y < robot.pos.y-field.field_length/3.) {
				LOUT << "BPassBeforeGoal: " << teammates[i].number << " not a partner, more than 1/4 field away" << endl;		
        LOUT << "% red solid circle " << Circle(teammates[i].pos, 600.) << endl;
				LOUT << "% red word " << Vec(500,0) + teammates[i].pos <<"more_than_1/4_field_away\n" << endl;
        continue;
      }
      if (teammates[i].pos.y > field.field_length/2. - field.penalty_area_length + 500.) {  // darf etwas in penalty area stehen
        LOUT << "BPassBeforeGoal: " << teammates[i].number << " not a partner, to close to opponent's base line" << endl;
        LOUT << "% red solid circle " << Circle(teammates[i].pos, 600.) << endl;
				LOUT << "% red word " << Vec(500,0) + teammates[i].pos <<"too close to base line \n" << endl;
        continue;
      }

			//if the current robot doesn't drop out, add it as a possible target
      //possibleTargets.push_back(i);

			//check if pass can be carried out
      int freeProximity= teammates[i].occupancy_grid.occupied((robot.pos-teammates[i].pos).angle());
      Vec relTeammatePos = world2robot * teammates[i].pos;
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
        LOUT << "BPassBeforeGoal: pass moeglich zu roboter: " << teammates[i].number << endl;
        LOUT << "% blue solid circle  " << Circle(teammates[i].pos, 600.) << endl; // Pass moeglich
				// remember id of the robot
        possibleTargets.push_back(i);
      }
      else {
				LOUT << "BPassBeforeGoal: " << teammates[i].number << " not a partner, passing lane occupied" << endl;		
        LOUT << "% red solid circle " << Circle(teammates[i].pos, 600.) << endl;
				LOUT << "% red word " << Vec(500,0) + teammates[i].pos <<"passing_lane_occupied\n" << endl;
      }
    }
		
    if (possibleTargets.size() == 0) {
      LOUT << "BPassBeforeGoal (Invocation) not activated, reason: no passpartner found." << endl;
      return false;
    }

    int closestId = -1;
    for (unsigned int i=0; i < possibleTargets.size(); i++) {
	LOUT << "BPassBeforeGoal: Nr. " << teammates[possibleTargets[i]].number << " kommt in frage" << endl;

      // Campare the line from the passing robot to the goal with the line
      // from the passing robot to the passed robot. 
	
        Frame2d robotGoalFrame(robot.pos, (Vec(0., field.field_length/2.)-robot.pos).angle());
        robotGoalFrame.invert();
        LOUT << "\% black solid " << LineSegment(robotGoalFrame.get_pos(), 
                                                 robotGoalFrame.get_pos()+(robotGoalFrame.get_x_axis()*2000.)) << endl;
        Vec teammateAbs = teammates[possibleTargets[i]].pos;
        Vec teammateInGoalFrame = robotGoalFrame * teammates[possibleTargets[i]].pos;                
        // nur spieler, die von Linie Roboter->Tor aus gesehen zur Mitte des Feldes sind
        // und die in der y-Koordinate nicht n‰her zur Seitenlinie als der Ballbesitzer sind
        LOUT << "TeammateInGoalFrame: " << teammateInGoalFrame << endl;
        LOUT << "(robot.pos.x-teammateAbs.x)*robot.pos.x: " << (robot.pos.x - teammateAbs.x) * robot.pos.x << endl;
        if(teammateInGoalFrame.y * robot.pos.x > 0 &&  //  check, zu  Mitte von Linie Robot->Tor aus gesehen
           (robot.pos.x - teammateAbs.x) * robot.pos.x > 0) { // check, in x-koordinate naeher zur Tor-Tor-Linie

	  if (closestId < 0) {
	    closestId = possibleTargets[i];
	    LOUT << "BPassBeforeGoal: noch niemand gewaehlt, nehme: " << teammates[possibleTargets[i]].number << endl;
	  } else {
            Vec closestTeammateInGoalFrame = robotGoalFrame * teammates[closestId].pos;
            if ((closestTeammateInGoalFrame.angle().get_deg() - 
                 teammateInGoalFrame.angle().get_deg()) * robot.pos.x > 0) {
              LOUT << "BPassBeforeGoal: nehme Nr. " << teammates[possibleTargets[i]].number << ". " << endl;
              closestId = possibleTargets[i];
	    } else {
	      LOUT << "BPassBeforeGoal: nehme nicht Nr. " << teammates[possibleTargets[i]].number << endl;
            }
          }
        } else {
          LOUT << "BPassBeforeGoal: target (" << teammates[possibleTargets[i]].number << ") not inside the tortenstueck. " << endl;
        }
      }
    

    if (closestId < 0) {
	LOUT << "BPassBeforeGoal: kein roboter kommt in frage. " << endl;
	return false;
    } 

    targetId = teammates[closestId].number;
    targetPos = teammates[closestId].pos;

    LOUT << "% blue word " << Vec(500,0) + teammates[closestId].pos <<"passpartner\n" << endl;

    return true;
  }

  bool BPassBeforeGoal::calculateTarget(const Time& t) {
	return true;
  }
}

