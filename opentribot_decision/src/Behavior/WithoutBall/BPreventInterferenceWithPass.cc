#include "BPreventInterferenceWithPass.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

namespace Tribots
{
  using namespace std;
    
  BPreventInterferenceWithPass::BPreventInterferenceWithPass(const string name) 
    : Behavior(name), skill(new SGoToPosEvadeObstacles())
  {
    transVel=2.5;
  }

  BPreventInterferenceWithPass::~BPreventInterferenceWithPass() throw ()
  {
    delete skill;
  }
  
  void BPreventInterferenceWithPass::updateTactics (const TacticsBoard& tb) 
    throw () 
  {
    // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
         transVel=1.8;
    } else	  
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
        transVel=1.3;
    } else
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
        transVel=2.5;
    } else { // =normal
        transVel=2.0;
    }
  }
  
  void 
  BPreventInterferenceWithPass::gainControl(const Time& t) throw(TribotsException)
  {
    started.update();
    skill->gainControl(t);
  }      

  void
  BPreventInterferenceWithPass::loseControl(const Time& t) throw(TribotsException) 
  {
    skill->loseControl(t);
  }

  
  DriveVector
  BPreventInterferenceWithPass::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const RobotLocation& robot = MWM.get_robot_location(t);
    // im Moment einfach nur zurueck fahren
    skill->init(robot.pos-Vec(0,2000), transVel, ballLocation.pos.toVec()-robot.pos);
    return skill->getCmd(t);
  }

  bool 
  BPreventInterferenceWithPass::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return (seheBall || seheKommuniziertenBall);
  }

  bool BPreventInterferenceWithPass::checkCommitmentCondition(const Time& tt) throw()
  {
    return 
      BPreventInterferenceWithPass::checkInvocationCondition(tt) && 
      started.elapsed_sec() < 3;
  }
}

