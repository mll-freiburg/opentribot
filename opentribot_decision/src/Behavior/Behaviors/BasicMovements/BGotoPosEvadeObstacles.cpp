#define DEBUG_DRAUFHALTEN 

#include "BGotoPosEvadeObstacles.h"
#include "../../../WorldModel/WorldModel.h"

namespace Tribots {

  using namespace std;
  
  BGotoPosEvadeObstacles::BGotoPosEvadeObstacles(Vec targetPos, Vec targetHeading, double driveVel)
    : Behavior("BGotoPosEvadeObstacles"), targetPos(targetPos)
  {
    skill = new SGoToPosEvadeObstacles();

    bool avoidball=1;

    skill->init(targetPos, driveVel, targetHeading, avoidball);
  }

  DriveVector BGotoPosEvadeObstacles::getCmd(const Time& t) throw (TribotsException) {
    
    DriveVector dv;
    dv = skill->getCmd(t);

    return dv;
  }
  
  BGotoPosEvadeObstacles::~BGotoPosEvadeObstacles() throw () {
  }

	void BGotoPosEvadeObstacles::updateTactics (const TacticsBoard& tb) throw ()
  {
  }

  void BGotoPosEvadeObstacles::cycleCallBack(const Time& t) throw () {
  }

  bool BGotoPosEvadeObstacles::checkCommitmentCondition(const Time& t) throw() {
	  Vec robotPos = MWM.get_robot_location (t).pos;

		return (robotPos -targetPos).length() > 2000;
  }

  bool BGotoPosEvadeObstacles::checkInvocationCondition(const Time& t) throw() {
	  Vec robotPos = MWM.get_robot_location (t).pos;

		return (robotPos -targetPos).length() > 200;
  }
}
