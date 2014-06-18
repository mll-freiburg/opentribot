#include "BStuckDistanceShooter.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/random.h"
#include <iostream>

namespace Tribots
{
  using namespace std;
  BStuckDistanceShooter::BStuckDistanceShooter() 
    : Behavior("BStuckDistanceShooter")
  {}

  BStuckDistanceShooter::~BStuckDistanceShooter() throw()
  {}

  DriveVector
  BStuckDistanceShooter::getCmd(const Time& tt) throw(TribotsException)
  {
    Frame2d world2robot = WBOARD->getAbs2RelFrame(tt);
    Frame2d robot2world = WBOARD->getRel2AbsFrame(tt);
		
    //Position der letzten Blockade in Weltkoordinaten
    Vec _dir_of_stuck = MWM.get_robot_location(tt).stuck.dir_of_stuck;
    //Fahrtrichtung bei letzter Blockade in Weltkoordinaten
    Vec _pos_of_stuck = MWM.get_robot_location(tt).stuck.pos_of_stuck;
	
    DriveVector dv; 
    dv.vrot   = (MWM.get_robot_location(tt).pos.x < 0.) ? -2. : 2;
    dv.vtrans = Vec(0.,0.);  // velocity
    dv.kick = 0;
    
    return dv;
  }

  bool 
  BStuckDistanceShooter::checkInvocationCondition(const Time& tt) throw()
  {
    return  
      MWM.get_robot_location(tt).stuck();
  }

  void
  BStuckDistanceShooter::gainControl(const Time& t) throw(TribotsException) 
  {
    starttime=t;
    rotation=0.0;
  }
 
  bool 
  BStuckDistanceShooter::checkCommitmentCondition(const Time& tt) throw()
  {
    return starttime.elapsed_msec() < 1000; // maxmial 1.0s ausweichen
  }
}

