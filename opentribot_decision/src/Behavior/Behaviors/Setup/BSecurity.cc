
#include "BSecurity.h"
#include <cmath>

using namespace Tribots;

BSecurity::BSecurity () {
  name="BSecurity";
	robsize = MWM.get_robot_properties().max_robot_radius;
}

BSecurity::~BSecurity () throw () {;}


bool BSecurity::checkInvocationCondition(const Time& t) throw() {
  bool isStuck = MWM.get_robot_location(t).stuck();
	bool obstacle = false;
	double angle_diff;
	const RobotLocation&  robloc = MWM.get_robot_location(t);
     const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
	
	for (unsigned int i=0; i<obstacles.size(); i++) {
		
		// wenn das hinderniss nur noch einen meter entfernt ist
		if( (obstacles[i].pos-robloc.pos).length() < 1000 ) {
			
			// und ich nicht daran vorbei komme
			angle_diff = ((obstacles[i].pos - robloc.pos).angle() - robloc.vtrans.angle()).get_rad();
			if( fabs(tan(angle_diff) * (obstacles[i].pos - robloc.pos).length()) -robsize < obstacles[i].width/2 ) {
				LOUT << "ein Hinderniss steht in meinem Weg \r\n";
			  obstacle = true;
				break;
			}
		}
	}

  return isStuck || obstacle;
}

bool BSecurity::checkCommitmentCondition(const Time& t) throw() {
  return checkInvocationCondition (t);
}
