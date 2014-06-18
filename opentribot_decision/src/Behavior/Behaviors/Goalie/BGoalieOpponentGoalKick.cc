
#include "BGoalieOpponentGoalKick.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

BGoalieOpponentGoalKick::BGoalieOpponentGoalKick (double ox, double oy, bool act) throw () : Behavior ("BGoalieOpponentGoalKick") {
  offset_x = ox;
  offset_y = oy;
  use_for_goal_kick = act;
  use_for_throw_in = false;
  use_for_free_kick = false;
  relevant_ball=Vec(0,0);
}

DriveVector BGoalieOpponentGoalKick::getCmd(const Time& texec) throw () {
  const FieldGeometry& fg = MWM.get_field_geometry ();
  const BallLocation& bloc = MWM.get_ball_location (texec);
  RefereeState rs = MWM.get_game_state().refstate;
  if (rs==Tribots::preOpponentGoalKick || rs==Tribots::postOpponentGoalKick ||
      rs==Tribots::preOpponentFreeKick || rs==Tribots::postOpponentFreeKick ||
      rs==Tribots::preOpponentThrowIn || rs==Tribots::postOpponentThrowIn
     )
    activation_time.update();
  if (bloc.pos_known!=BallLocation::known || bloc.pos.y>0 || abs(bloc.pos.x)>0.5*fg.field_width+fg.side_band_width)
    ball_not_seen_in_own_half.update();
  double f = 0.0;
  if (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated)
    f = (bloc.pos.x>0 ? +1.0 : -1.0);
  goto_pos_skill.init (Vec (f*offset_x, -0.5*fg.field_length+offset_y), Angle::zero, true);
  return goto_pos_skill.getCmd (texec);
}

bool BGoalieOpponentGoalKick::checkInvocationCondition (const Time& t) throw () {
  if (MWM.get_ball_location (t, false).pos_known==BallLocation::known || MWM.get_ball_location (t, false).pos_known==BallLocation::communicated) {
    relevant_ball = MWM.get_ball_location (t, false).pos.toVec();
    if (MWM.get_game_state().refstate==Tribots::preOpponentKickOff || MWM.get_game_state().refstate==Tribots::postOpponentKickOff) 
      relevant_ball=Vec(-100,0);
  }
  bool ball_on_sides = (abs(relevant_ball.x)>=2500) && (relevant_ball.y>-0.5*MWM.get_field_geometry().field_length+1500);
  return (
      (use_for_goal_kick && (MWM.get_game_state().refstate==Tribots::preOpponentGoalKick || MWM.get_game_state().refstate==Tribots::postOpponentGoalKick)) ||
      (use_for_free_kick &&ball_on_sides && (MWM.get_game_state().refstate==Tribots::preOpponentFreeKick || MWM.get_game_state().refstate==Tribots::postOpponentFreeKick)) ||
      (use_for_throw_in && ball_on_sides && (MWM.get_game_state().refstate==Tribots::preOpponentThrowIn || MWM.get_game_state().refstate==Tribots::postOpponentThrowIn)));
}

bool BGoalieOpponentGoalKick::checkCommitmentCondition (const Time& t) throw () {
  return (
      MWM.get_game_state().refstate==preOpponentGoalKick || MWM.get_game_state().refstate==postOpponentGoalKick || 
      MWM.get_game_state().refstate==preOpponentFreeKick || MWM.get_game_state().refstate==postOpponentFreeKick || 
      MWM.get_game_state().refstate==preOpponentThrowIn || MWM.get_game_state().refstate==postOpponentThrowIn || 
      (activation_time.elapsed_msec()<3000 && ball_not_seen_in_own_half.elapsed_msec()<200));
}

void BGoalieOpponentGoalKick::updateTactics (const TacticsBoard& tb) throw () {
  if (tb[string("AbstossGegner")] == string("blockTor")) {
    use_for_goal_kick = false;
    use_for_free_kick = use_for_throw_in = true;
  } else if (tb[string("AbstossGegner")] == string("normal")) {
    use_for_goal_kick=use_for_free_kick=use_for_throw_in=false;
  }
}

void BGoalieOpponentGoalKick::gainControl(const Time&) throw(TribotsException) {
  relevant_ball=Vec(0,0);
}
