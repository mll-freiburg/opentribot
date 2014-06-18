
#include "BGoaliePrePenalty.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace std;

BGoaliePrePenalty::BGoaliePrePenalty (double d, double e) throw () : Behavior ("BGoaliePrePenalty") {
  home = Vec (0,-0.5*MWM.get_field_geometry().field_length+d);
  max_excenter = 0;
  excenter=e;
  goto_pos_skill.set_dynamics (2.0);
}

void BGoaliePrePenalty::gainControl (const Time&) throw (TribotsException) {
  excenter=0;
}

DriveVector BGoaliePrePenalty::getCmd(const Time& texec) throw () {
  if (last_time_attacker_seen.elapsed_msec()>500) {
    excenter=0;
  }
  const BallLocation& bloc = MWM.get_ball_location (texec);
  const FieldGeometry& fg = MWM.get_field_geometry();
  Vec penalty_spot (0, -0.5*fg.field_length+fg.penalty_marker_distance);
  if (bloc.pos_known == BallLocation::known && (bloc.pos.toVec()-penalty_spot).length()<500) {
    Vec ball = bloc.pos.toVec();
    vector<ObstacleDescriptor>::const_iterator oit = obstacles.begin();
    Vec nearest_obstacle = Vec(0, 10000);
    while (oit!=obstacles.end()) {
      if (oit->pos.y>ball.y+100 && oit->pos.y<ball.y+1000 && oit->pos.x<ball.x+700 && oit->pos.x>ball.x-700 && oit->width<700 && oit->width>100) {
        if ((ball-oit->pos).squared_length()<(nearest_obstacle-ball).squared_length())
          nearest_obstacle = oit->pos;
      }
      oit++;
    }
    if (nearest_obstacle.y<ball.y+5000) {
      excenter = -1.5*(nearest_obstacle.x-ball.x);
      if (excenter>max_excenter) excenter=max_excenter;
      if (excenter<-max_excenter) excenter=-max_excenter;
      if (excenter>5 || excenter<-5) {
        LOUT << "% green cross " << nearest_obstacle << " word " << nearest_obstacle << " attacker!\n";
        LOUT << "Angreifer hinter Ball erkannt, passe Positionierung an\n";
      }
      last_time_attacker_seen.update();
    }
  }
  goto_pos_skill.init (home+Vec(excenter,0), Angle::zero, true);

  return goto_pos_skill.getCmd (texec);
}

bool BGoaliePrePenalty::checkInvocationCondition (const Time&) throw () {
  return (MWM.get_game_state().refstate==preOpponentPenalty || MWM.get_game_state().refstate==postOpponentPenalty);
}

bool BGoaliePrePenalty::checkCommitmentCondition (const Time& t) throw () {
  return checkInvocationCondition(t);
}

void BGoaliePrePenalty::updateTactics (const TacticsBoard& tb) throw () {
  double v;
  if (string2double (v, tb[string("GoaliePenaltyExzentrisch")])) {
    if (v<500 && v>-500)
      max_excenter=v;
  }
}
