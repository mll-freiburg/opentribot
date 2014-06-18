#include "SGoToPosEvadeObstacles.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"
#include <iostream>

namespace Tribots {

  using namespace std;

  SGoToPosEvadeObstacles::SGoToPosEvadeObstacles() 
  : Skill("SGoToEvadeObstacles"), physgoto (new SPhysGotoPosAvoidObstacles)
  {}

  SGoToPosEvadeObstacles::~SGoToPosEvadeObstacles() throw()
  {
    delete physgoto;
  }

  void 
  SGoToPosEvadeObstacles::init(Vec _targetPos, double _driveVel,
                                                  Vec _targetHeading,
                                                  bool _avoidBall) throw(TribotsException)
  {
    physgoto->init (_targetPos, _targetHeading.angle()-Angle::quarter, true);
    physgoto->set_dynamics (_driveVel);
  }

  DriveVector 
  SGoToPosEvadeObstacles::getCmd(const Time& t) throw(TribotsException)
  {
    return physgoto->getCmd(t);
  }

  void SGoToPosEvadeObstacles::set_target_evade_strategy (Angle a) throw () {
    return physgoto->set_target_evade_strategy (a);
  }
  
  void SGoToPosEvadeObstacles::force_target () throw () {
    return physgoto->force_target ();
  }
}

