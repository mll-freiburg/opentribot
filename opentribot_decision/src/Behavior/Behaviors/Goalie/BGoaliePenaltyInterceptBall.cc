
#include "BGoaliePenaltyInterceptBall.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"

using namespace Tribots;
using namespace std;

BGoaliePenaltyInterceptBall::BGoaliePenaltyInterceptBall () throw () : Behavior ("BGoaliePenaltyInterceptBall") {;}

bool BGoaliePenaltyInterceptBall::checkInvocationCondition (const Time& t) throw () {
  return (MWM.get_game_state().refstate==opponentPenalty && MWM.get_ball_location (t).pos_known==BallLocation::known);
}

bool BGoaliePenaltyInterceptBall::checkCommitmentCondition (const Time& t) throw () {
  return (MWM.get_game_state().refstate==opponentPenalty && MWM.get_ball_location (t).pos_known==BallLocation::known);
}

void BGoaliePenaltyInterceptBall::gainControl (const Time& t) throw (TribotsException) {
  goto_pos_skill.gainControl (t);
}

DriveVector BGoaliePenaltyInterceptBall::getCmd(const Time& t) throw () {
  Vec bpos = MWM.get_ball_location (t).pos.toVec();
  Vec bvel = MWM.get_ball_location (t).velocity.toVec();
  Vec tpos (0,-0.5*MWM.get_field_geometry().field_length);
  double gw2 = 0.5*MWM.get_field_geometry().goal_width;
  double rw2 = 0.5*MWM.get_robot_properties().robot_width;
  bool stop_at_point=true;
  if (bvel.length()<0.4 || bvel.angle().in_between (Angle::zero, Angle::half)) {
    if (bpos.y<tpos.y+2000) {
      tpos.x=bpos.x;
    }
  } else {
    try{
      Line bmovement (bpos, bpos+bvel);
      Line goalline (tpos, tpos+Vec(1,0));
      Vec ip = intersect (goalline, bmovement);
      tpos.x=ip.x;
      stop_at_point = (bvel.length()>1.5) && (tpos.x>-gw2-500) && (tpos.x<gw2+500);
    }catch(exception&){;}  // kein Schnittpunkt
  }
  if (tpos.x>=gw2-rw2) {
    tpos.x = gw2-rw2;
  } else if (tpos.x<-gw2+rw2) {
    tpos.x=-gw2+rw2;
  }
  
  goto_pos_skill.init (tpos, Angle::zero, stop_at_point ? 0.0 : 1.0);
  return goto_pos_skill.getCmd (t);
}
