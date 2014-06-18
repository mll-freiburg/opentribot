#include "BPass.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/Frame2D.h"
#include "../../../Fundamental/random.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Behavior/Predicates/freeCorridor.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include <vector>
#include <cmath>
#include <sstream>

namespace Tribots {
  
  using namespace std;
 
  BPass::BPass(double passProbability)
  : Behavior("BPass"),
    target(Vec(0,0)), targetId(0),
    kicked(false),
	  passProbability(passProbability),
	  skill(new SDribbleBallToPos())
  {}
  
  BPass::~BPass() throw ()
  {}
  
  void BPass::loseControl(const Time&) throw(TribotsException)
  {
    kicked = false;
  }
  
  void BPass::cycleCallBack(const Time& t) throw()
  {
  }
	
  void BPass::updateTactics(const TacticsBoard& tb) throw()
  {
    string key = "ActivePassing";
    if (tb[key] == string("ja")) {
      passProbability = 1.;
    }
    else {
      passProbability = 0.;
    }
	}
  
  bool BPass::checkCommitmentCondition(const Time& t) throw()
  {
    return checkInvocationCondition(t);
  }
  
  bool BPass::checkInvocationCondition(const Time& t) throw()
  {
		//ballbesitz + mehr als 1 roboter + 1/4 abstand vom eigenen tor
	
    const RobotLocation& robot = MWM.get_robot_location(t);
    //const FieldGeometry& fgeom = MWM.get_field_geometry();
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
    //const BallLocation& ball = MWM.get_ball_location(t);
    const vector<TeammateLocation>& teammates = MWM.get_teammate_location();

    unsigned int robotSelfId = MWM.get_robot_id(); 
    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);
    Frame2d robot2world = WBOARD->getRel2AbsFrame(t);
    
    if (! WBOARD->doPossessBall(t)) {
			LOUT << "BPass not activated, reason: don't have the ball." << endl;
      return false;
    }
    /*
		if (WBOARD->onlyOneRobot()) {
			LOUT << "BPass not activated, reason: less than two players on the field." << endl;
      return false;
    }
		*/
		
		/*
    if (robot.pos.y < -fgeom.field_length/4.) { // nicht passen,
			LOUT << "BPass not activated, reason: too close to my own goal." << endl;
      return false;                   // wenn zu nah am eigenen tor
    }
		*/
    
    vector<unsigned int> possibleTargets;
    for (unsigned int i=0; i < teammates.size(); i++) {
      if (teammates[i].number == robotSelfId ||													// selber
          fabs((double)teammates[i].timestamp.diff_msec(t)) > 2000. //|| // info zu alt
          //teammates[i].pos.y < 0.
					) {																	// eigene haelfte
        continue;
      }

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
				LOUT << "BPass: pass moeglich zu roboter: " << teammates[i].number << endl;
        LOUT << "% blue solid circle  " << Circle(teammates[i].pos, 600.) << endl; // Pass moeglich
        possibleTargets.push_back(i);
      }
      else {
        LOUT << "% red solid circle " << Circle(teammates[i].pos, 600.) << endl; 
      }
    }
    if (possibleTargets.size() == 0) {
			LOUT << "BPass not activated, reason: no passpartner found." << endl;
      return false;
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

		//notify the target robot about pass
		stringstream msg;
		msg << "knutpass: " << targetId;
		MWM.get_message_board().publish(msg.str());
		LOUT << "BPass: Gezielter Pass zu Spieler " << targetId << endl;
		
		startPos = robot.pos;
		
    return true;
  }
  
  DriveVector BPass::getCmd(const Time& t) throw(TribotsException)
  {
    const RobotLocation& robot = MWM.get_robot_location(t);
    DriveVector dv;
    		
		//erstmal einen punkt an der au§enseite auf ungefŠhrer hšhe des ziels anfahren
		//dann drehen und schie§en.
		double turndistance = 2000.0;
		double borderdistance = 1000.0;
	
		if (target.y > robot.pos.y + turndistance) {
			LOUT << "BPass: Fahre an der au§enseite entlang, bis zum Ziel." << endl;

			double robx;
			
			//check distance to sideline!
			const FieldGeometry& fgeom = MWM.get_field_geometry();
			if (fabs(robot.pos.x) < fabs(fgeom.field_width/2) - borderdistance) {
				if (robot.pos.x > 0) {
					robx = (fgeom.field_width/2) - borderdistance;
				} else {
					robx = -((fgeom.field_width/2) - borderdistance);
				}
			} else {
				robx = startPos.x;
			}

			Vec intermediateTarget(robx, target.y);
			//
			skill->setParameters(intermediateTarget, 2.2, true);
		
		} else {
			skill->setParameters(target, 2.2, true);
		}
		
    dv = skill->getCmd(t);

    Angle robotHeading = robot.heading;
    Angle targetHeading=(target - robot.pos).angle();
		double hdiff = 180 - fabs(((robotHeading - (targetHeading-Angle::quarter)).get_deg()) -180);
		LOUT << "angle to destination in deg: " << hdiff << "\n";
   
		if (hdiff < 1) {
      dv.vtrans = robot.vtrans / robot.heading; //?
      dv.vrot = 0;
      dv.kick = 2; //flach passen

			//TODO: kickstŠrke anpassen
      dv.klength = 30;

      kicked = true;

      WBOARD->resetPossessBall(); // Ballbesitz durch pass verloren
    }
    
    return dv;
  }
  
}
