
#include "BGoalieOpponentKickOff.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

BGoalieOpponentKickOff::BGoalieOpponentKickOff (bool act) throw () : Behavior ("BGoalieOpponentKickOff") {
  use_behavior = act;
  dynamic_on = true;
  dynamic_mode = false;
  was_on=false;
  time_of_free=1000000;
}

DriveVector BGoalieOpponentKickOff::getCmd(const Time& texec) throw () {
  const FieldGeometry& fg = MWM.get_field_geometry ();
  const BallLocation& bloc = MWM.get_ball_location (texec);
  RefereeState rs = MWM.get_game_state().refstate;
  if (rs==Tribots::preOpponentKickOff || rs==Tribots::postOpponentKickOff)
    activation_time.update();
  if (!(bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated) || bloc.pos.y>-800 ||  abs(bloc.pos.x)>0.5*fg.field_width+fg.side_band_width)
    ball_not_seen_in_own_half.update();
  goto_pos_skill.init (Vec(-1500, -0.5*MWM.get_field_geometry().field_length+1000), Angle::zero, true);
  goto_pos_skill.set_ball_as_obstacle (true);
  return goto_pos_skill.getCmd (texec);
}

bool BGoalieOpponentKickOff::checkInvocationCondition (const Time&) throw () {
  return (use_behavior && dynamic_on && (MWM.get_game_state().refstate==Tribots::preOpponentKickOff || MWM.get_game_state().refstate==Tribots::postOpponentKickOff));
}

bool BGoalieOpponentKickOff::checkCommitmentCondition (const Time& t) throw () {
  return (
      use_behavior && dynamic_on && (MWM.get_game_state().refstate==preOpponentKickOff || MWM.get_game_state().refstate==postOpponentKickOff || (activation_time.elapsed_msec()<500 && ball_not_seen_in_own_half.elapsed_msec()<200))
         );
}

void BGoalieOpponentKickOff::updateTactics (const TacticsBoard& tb) throw () {
  if (tb[string("GoalieKickOff")] == string("zentral")) {
    use_behavior = false;
    dynamic_mode = false;
    dynamic_on = true;
  } else if (tb[string("GoalieKickOff")] == string("seitlich")) {
    use_behavior = true;
    dynamic_mode = false;
    dynamic_on = true;
  } else if (tb[string("GoalieKickOff")] == string("dynamisch")) {
    use_behavior = true;
    dynamic_mode = true;
  } 
}

void BGoalieOpponentKickOff::loseControl(const Time&) throw(TribotsException) {
  was_on=true;
  time_free_started.update();
  time_of_free =100000;
}

void BGoalieOpponentKickOff::cycleCallBack(const Time&) throw() {
  if (was_on && MWM.get_game_state().refstate==Tribots::stopRobot) {
    time_of_free = time_free_started.elapsed_msec();
    was_on=false;
  }
  if (!was_on && MWM.get_game_state().refstate!=Tribots::stopRobot) 
    time_of_free = 1000000;
  if (MWM.get_game_state().opponent_score!=opponent_score) {
    if (time_of_free<5000 && dynamic_mode)
      dynamic_on=false;
    opponent_score=MWM.get_game_state().opponent_score;
  }
}
