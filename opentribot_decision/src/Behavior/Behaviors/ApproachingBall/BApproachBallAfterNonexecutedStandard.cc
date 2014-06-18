
#include "BApproachBallAfterNonexecutedStandard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/stringconvert.h"
#include "../../../Fundamental/geometry.h"
#include <cmath>

using namespace Tribots;
using namespace std;

namespace {
  inline double square (double x) {
    return x*x;
  }
}


BApproachBallAfterNonexecutedStandard::BApproachBallAfterNonexecutedStandard () throw () 
  : Behavior ("BApproachBallAfterNonexecutedStandard"), wait_for_free (false), active(false), active_over(false), waittime (10000)  {;}
  
void BApproachBallAfterNonexecutedStandard::cycleCallBack(const Time& t) throw ()
{
  bool ball_at_side = abs(MWM.get_ball_location (t).pos.x)>0.5*MWM.get_field_geometry().field_width-1000;
  bool ball_at_goal_kick = sqrt(square(abs(MWM.get_ball_location (t).pos.x)-0.25*MWM.get_field_geometry().field_width)+square(MWM.get_ball_location (t).pos.y-0.5*MWM.get_field_geometry().field_length+MWM.get_field_geometry().penalty_marker_distance))<1000;
  switch (MWM.get_game_state().refstate) {
    case postOpponentKickOff:
    case postOpponentGoalKick:
    case postOpponentCornerKick:
    case postOpponentFreeKick:
    case postOpponentThrowIn:
      active_over=false;
      if (!wait_for_free) {
        wait_for_free=true;
        post_standard_start.update();
        active=false;
      }
      if ((ball_at_side || ball_at_goal_kick) && post_standard_start.elapsed_msec()>waittime-200 && !active_over) {
        active=true;
        ballpos_post=MWM.get_ball_location (t).pos.toVec();
      }
      break;
    case freePlay:
      if ((ball_at_side || ball_at_goal_kick) && post_standard_start.elapsed_msec()>waittime-200 || !active_over) {
        active=true;
        ballpos_post=MWM.get_ball_location (t).pos.toVec();
      }
      wait_for_free=false;
      if (post_standard_start.elapsed_msec()>waittime+3000 || MWM.get_ball_location(t).velocity.toVec().length()>0.5 || !(ball_at_side || ball_at_goal_kick)) {
        active=false;
        active_over=true;
      }
      break;
    default:
      wait_for_free=false;
      active=false;
  };
}

bool BApproachBallAfterNonexecutedStandard::checkInvocationCondition (const Time&) throw () {
  return active;
}

bool BApproachBallAfterNonexecutedStandard::checkCommitmentCondition (const Time&) throw () {
  return active;
}

DriveVector BApproachBallAfterNonexecutedStandard::getCmd(const Time& t) throw () {
  Vec ball = MWM.get_ball_location (t).pos.toVec();
  Vec robot = MWM.get_robot_location (t).pos;
  Angle targetHeading = Angle::deg_angle (ball.x>0? 30 : -30);
  Vec waypointPosition = ball+Vec(ball.x>0 ? 300 : -300,-600);
  Line targetline (waypointPosition, ball);
  double diffToTargetline = targetline.distance (robot);
  if (diffToTargetline>150)
    goto_pos_skill.init (waypointPosition, targetHeading, true);
  else
    goto_pos_skill.init (ball, targetHeading, false);
  
  DriveVector dv = goto_pos_skill.getCmd (t);
  dv.kick=false;
  return dv;
}

void BApproachBallAfterNonexecutedStandard::updateTactics (const TacticsBoard& tb) throw () {
  string valstr = tb["StandardSituationWartezeit"];
  double val=10;
  if (!string2double (val, valstr))
    val=10;
  waittime=val*1000;
}

void BApproachBallAfterNonexecutedStandard::loseControl(const Time&) throw(TribotsException) {
  active=false;
  active_over=true;  
}
