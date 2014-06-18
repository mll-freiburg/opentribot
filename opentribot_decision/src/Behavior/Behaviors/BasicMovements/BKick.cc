#include "BKick.h"
#include "../../Predicates/freeCorridor.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../Structures/FieldGeometry.h"
#include "../../Fundamental/geometry.h"
#include <iostream>
#include <cmath>

namespace Tribots
{


using namespace Tribots;
using namespace std;

double BKick::obstacle_distance_to_cone (Vec s, Vec target, Angle alpha, const ObstacleLocation& obstacles, const Time& t) throw () {

    Vec links = (target - s)/(0.5 * alpha) + s;
	Vec rechts =  (target -links) + target;
    LOUT << "% line " << MWM.get_robot_location(t).pos.x << " "
                        << MWM.get_robot_location(t).pos.y << " "
                        << links.x << " "
                        << links.y << "\n";


    LOUT << "% line " << MWM.get_robot_location(t).pos.x << " "
                        << MWM.get_robot_location(t).pos.y << " "
                        << rechts.x << " "
                        << rechts.y << "\n";


	Triangle* tri = new Triangle(s,links,rechts);
	
    const FieldGeometry& fg = MWM.get_field_geometry();
    vector<Vec>::const_iterator posit = obstacles.pos.begin();
    while(posit!=obstacles.pos.end()) {
	  if ( tri->is_inside(*posit) && (posit->y < (fg.field_length / 2.0)) ) { // wenn im Kegel und im Feld
        delete tri;
        LOUT << "BKick:: Hindernis im Kegel\n";
        return 0;
      }
      posit++;
    }
	double linkeDistanz = obstacle_distance_to_line(s, links, obstacles); //misst Hindernisse ausserhalb des Feldes mit (todo)
	double rechteDistanz = obstacle_distance_to_line(s, rechts, obstacles);

	delete tri;
	return (linkeDistanz < rechteDistanz) ? linkeDistanz : rechteDistanz;
}


BKick::BKick() : Behavior("BKick", false)
{}

BKick::~BKick() throw()
{}

bool 
BKick::checkCommitmentCondition(const Time& t) throw()
{
  return BKick::checkInvocationCondition(t);
}


bool 
BKick::checkInvocationCondition(const Time& t) throw()
{
  Time now;         // Standardkonstruktor erzeugt Objekt mit aktueller Zeit
                    // Schuß löst ziemlich schnell aus
  

   if (!WBOARD->doPossessBall(t)) {
      LOUT << "\n BKick:: Can't kick. Whiteboard says, I don't possess the ball.\n";
      return false;
   }

  
    const RobotLocation& rloc = MWM.get_robot_location (t);
    const FieldGeometry& fgeom = MWM.get_field_geometry ();

    double security_distance = 0.5; // at least ballradius

    Vec s = rloc.pos;
    Vec ziel = s+DISTANCE*Vec::unit_vector(rloc.heading+Angle::quarter); //
    double obstacle_distance =
    obstacle_distance_to_cone (s,ziel, Angle::deg_angle(SCATTER_ANGLE), MWM.get_obstacle_location(t), t);
    bool freierSchusskorridor = (obstacle_distance > security_distance*fgeom.ball_diameter); 

  // check if the goal is in our angle

  const double postWidth = 100.0; // to prevent posthits
  
  Vec* leftGoalpost = new Vec( -MWM.get_field_geometry().goal_width/2 + postWidth, 
                                MWM.get_field_geometry().field_length/2);
  Vec* rightGoalpost = new Vec( MWM.get_field_geometry().goal_width/2 - postWidth, 
                                MWM.get_field_geometry().field_length/2);
                                 

  LOUT << "% line " << MWM.get_robot_location(t).pos.x << " "
                    << MWM.get_robot_location(t).pos.y << " "
                    << leftGoalpost->x << " "
                    << leftGoalpost->y << "\n";

  LOUT << "% line " << MWM.get_robot_location(t).pos.x << " "
                    << MWM.get_robot_location(t).pos.y << " "
                    << rightGoalpost->x << " "
                    << rightGoalpost->y << "\n";

  *leftGoalpost -= MWM.get_robot_location(t).pos;
  *rightGoalpost -= MWM.get_robot_location(t).pos;

  bool goalInAngle = (MWM.get_robot_location(t).heading+Angle::quarter).in_between(rightGoalpost->angle(), leftGoalpost->angle());
  if (!goalInAngle)
     LOUT << "BKick:: Winkel stimmt nicht\n";
  else {
     LOUT << "BKick:: winkel korrekt\n";
     LOUT << "Vectorangle left: " << leftGoalpost->angle().get_deg() << endl;
     LOUT << "Vectorangle right: " << rightGoalpost->angle().get_deg() << endl;
     LOUT << "Heading: " << (MWM.get_robot_location(t).heading+Angle::quarter).get_deg() << endl;
  }


  bool nahGenugVormTor = 
    (WBOARD->getFreeGoalPos(t) - MWM.get_robot_location(t).pos).length()
    <= DISTANCE;           // maximale schussdistanz TODO: ins config
  
  if (!nahGenugVormTor)
     LOUT << "BKick:: nicht nah genug vorm Tor\n";

  if (!freierSchusskorridor) 
     LOUT << "BKick:: kein freier Schusskorridor\n";
 
  return freierSchusskorridor && goalInAngle && nahGenugVormTor;  
}



DriveVector 
BKick::getCmd(const Time& t) throw(TribotsException)
{
   RobotLocation robot= MWM.get_robot_location(t);
   LOUT << " \n kick got control \n";
   LOUT << "% white circle "  << robot.pos.x << " " << robot.pos.y << " 300" << endl; 

  DriveVector dv;
  dv.kick = 1;
  dv.vrot = 0;
  dv.vtrans = 
    MWM.get_robot_location(t).vtrans/MWM.get_robot_location(t).heading;
    
  return dv;
}


  

};
