#include "BAvoidGoalieArea.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

namespace Tribots
{
  using namespace std;
    
  BAvoidGoalieArea::BAvoidGoalieArea() 
    : Behavior("BAvoidGoalieArea"), skill(new SGoToPosEvadeObstacles())
  {
    transVel=2.5;
  }

  BAvoidGoalieArea::~BAvoidGoalieArea() throw ()
  {
    delete skill;
  }

  void BAvoidGoalieArea::updateTactics (const TacticsBoard& tb) 
    throw () 
  {
    // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
         transVel=1.4;
    } else	  
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
        transVel=1.0;
    } else
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
        transVel=2.5;
    } else { // =normal
        transVel=2.5;
    }
  }
  
  DriveVector
  BAvoidGoalieArea::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec blockingPos;
      blockingPos = Vec(ballLocation.pos.x,field.field_length/2.-2000.);
      
    skill->init(blockingPos, transVel, ballLocation.pos.toVec()-robot.pos);
    DriveVector dv = skill->getCmd(t);
    if (WBOARD->doPossessBall(t)) {
      dv.kick = 1;
    }
    return dv;
  }

  bool 
  BAvoidGoalieArea::checkInvocationCondition(const Time& tt) throw()
  {
    const FieldGeometry& fg (MWM.get_field_geometry());
    const BallLocation& bloc (MWM.get_ball_location (tt));
    XYRectangle goalieArea(Vec(- 0.5*fg.goal_area_width, 0.5*fg.field_length+500),
                           Vec(0.5*fg.goal_area_width, 0.5*fg.field_length-fg.goal_area_length+100.));//+ Balldurchmesser!
    XYRectangle behindGoal(Vec(-0.5*fg.goal_area_width-500, 0.5*fg.field_length+1000),
                           Vec(0.5*fg.goal_area_width+500, 0.5*fg.field_length+0));
      
    bool seheBall=bloc.pos_known==BallLocation::known;

    return seheBall && (goalieArea.is_inside(bloc.pos.toVec()) || behindGoal.is_inside(bloc.pos.toVec()));
  }

  bool BAvoidGoalieArea::checkCommitmentCondition(const Time& tt) throw()
  {
    return 
      BAvoidGoalieArea::checkInvocationCondition(tt);
  }
}

