#include "BStuckOwnsBall.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/random.h"
#include <iostream>

namespace Tribots
{
  using namespace std;
  BStuckOwnsBall::BStuckOwnsBall() 
    : Behavior("BStuckOwnsBall")
  {}

  BStuckOwnsBall::~BStuckOwnsBall() throw()
  {}

  DriveVector
  BStuckOwnsBall::getCmd(const Time& tt) throw(TribotsException)
  {
    if (relTarget==Vec::zero_vector) {
      
      Frame2d world2robot = WBOARD->getAbs2RelFrame(tt);
      Frame2d robot2world = WBOARD->getRel2AbsFrame(tt);
		
      //Position der letzten Blockade in Weltkoordinaten
      Vec _dir_of_stuck = MWM.get_robot_location(tt).stuck.dir_of_stuck;
      //Fahrtrichtung bei letzter Blockade in Weltkoordinaten
      Vec _pos_of_stuck = MWM.get_robot_location(tt).stuck.pos_of_stuck;
	
      if (brandom(0.5)) {
	relTarget = 
	  (-(world2robot*_dir_of_stuck)*0.05) + 
	  (world2robot*_dir_of_stuck).rotate(Angle::quarter);
	rotation=+3;
      } 
      else {
	relTarget=
	  (-(world2robot*_dir_of_stuck)*0.05) + 
	  (world2robot*_dir_of_stuck).rotate(-Angle::quarter);
	rotation=-3;
      }
    }
    DriveVector dv; 
    dv.vrot   = rotation;
    dv.vtrans = relTarget.normalize() * 2;  // velocity
    
    return dv;
  }

  bool 
  BStuckOwnsBall::checkInvocationCondition(const Time& tt) throw()
  {
    return  
      MWM.get_robot_location(tt).stuck();
  }

  void
  BStuckOwnsBall::gainControl(const Time& t) throw(TribotsException) 
  {
    starttime=t;
    starttime.add_msec((long)urandom(-800., .200));
    relTarget=Vec::zero_vector;
    rotation=0.0;
  }
 
  bool 
  BStuckOwnsBall::checkCommitmentCondition(const Time& tt) throw()
  {
    return starttime.elapsed_msec() < 1500; // maxmial 1.5s ausweichen
  }
}

