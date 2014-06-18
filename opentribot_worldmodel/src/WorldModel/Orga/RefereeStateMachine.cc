
#include "RefereeStateMachine.h"
#include <cmath>

using namespace Tribots;
using namespace std;

RefereeStateMachine::RefereeStateMachine (const FieldGeometry& fg, int tc, RefereeState rs) throw () {
  team = tc;
  current = rs;
  ballknown = false;
  penalty_marker_y = -0.5*fg.field_length+fg.penalty_marker_distance;
  center_circle_radius = fg.center_circle_radius;
  precision_addon=0;
}

RefereeStateMachine::RefereeStateMachine (const RefereeStateMachine& src) throw () {
  team = src.team;
  current = src.current;
  latest_update = src.latest_update;
  ballpos = src.ballpos;
  ballknown = src.ballknown;
  penalty_marker_y = src.penalty_marker_y;
  precision_addon = src.precision_addon;
}

const RefereeStateMachine& RefereeStateMachine::operator= (const RefereeStateMachine& src) throw () {
  team = src.team;
  current = src.current;
  latest_update = src.latest_update;
  ballpos = src.ballpos;
  ballknown = src.ballknown;
  penalty_marker_y = src.penalty_marker_y;
  precision_addon = src.precision_addon;
  return (*this);
}

RefereeState RefereeStateMachine::get_state () const throw () {
  return current;
}

void RefereeStateMachine::set_state (RefereeState rs) throw () {
  current = rs;
}

void RefereeStateMachine::set_team_color (int tc) throw () {
  team = tc;
}

RefboxSignal RefereeStateMachine::update (RefboxSignal sig, const BallLocation& bloc, Vec robotpos, Vec robotvel, double robotvrot) throw () {
  // zunaechst cyan/magenta in own/opponent umcodieren
  if (team>0) {  // cyan=own magenta=opponent
    switch (sig) {
    case SIGcyanKickOff: sig=SIGownKickOff; break;
    case SIGmagentaKickOff: sig=SIGopponentKickOff; break;
    case SIGcyanFreeKick: sig=SIGownFreeKick; break;
    case SIGmagentaFreeKick: sig=SIGopponentFreeKick; break;
    case SIGcyanGoalKick: sig=SIGownGoalKick; break;
    case SIGmagentaGoalKick: sig=SIGopponentGoalKick; break;
    case SIGcyanCornerKick: sig=SIGownCornerKick; break;
    case SIGmagentaCornerKick: sig=SIGopponentCornerKick; break;
    case SIGcyanThrowIn: sig=SIGownThrowIn; break;
    case SIGmagentaThrowIn: sig=SIGopponentThrowIn; break;
    case SIGcyanPenalty: sig=SIGownPenalty; break;
    case SIGmagentaPenalty: sig=SIGopponentPenalty; break;
    case SIGcyanGoalScored: sig=SIGownGoalScored; break;
    case SIGmagentaGoalScored: sig=SIGopponentGoalScored; break;
    default: break;
    }
  } else {  // cyan=opponent, magenta=own
    switch (sig) {
    case SIGcyanKickOff: sig=SIGopponentKickOff; break;
    case SIGmagentaKickOff: sig=SIGownKickOff; break;
    case SIGcyanFreeKick: sig=SIGopponentFreeKick; break;
    case SIGmagentaFreeKick: sig=SIGownFreeKick; break;
    case SIGcyanGoalKick: sig=SIGopponentGoalKick; break;
    case SIGmagentaGoalKick: sig=SIGownGoalKick; break;
    case SIGcyanCornerKick: sig=SIGopponentCornerKick; break;
    case SIGmagentaCornerKick: sig=SIGownCornerKick; break;
    case SIGcyanThrowIn: sig=SIGopponentThrowIn; break;
    case SIGmagentaThrowIn: sig=SIGownThrowIn; break;
    case SIGcyanPenalty: sig=SIGopponentPenalty; break;
    case SIGmagentaPenalty: sig=SIGownPenalty; break;
    case SIGcyanGoalScored: sig=SIGopponentGoalScored; break;
    case SIGmagentaGoalScored: sig=SIGownGoalScored; break;
    default: break;
    }
  }

  // je nach Signal reagieren; Torsignale werden nicht beruecksichtigt
  switch (sig) {
  case SIGstop:
  case SIGhalt:
    current = stopRobot;
    break;
  case SIGstart:
    current = freePlay;
    break;
  case SIGready:
    switch (current) {
    case preOwnKickOff:
    case preOwnFreeKick:
    case preOwnGoalKick:
    case preOwnCornerKick:
    case preOwnThrowIn:
    case preDroppedBall:
      current = freePlay;
      break;
    case preOwnPenalty:
      current = ownPenalty;
      break;
    case preOpponentKickOff:
      current = postOpponentKickOff;
      ballpos = bloc.pos.toVec();
      ballknown = (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated);
      precision_addon = (((ballknown && (robotpos-bloc.pos.toVec()).length()>3000) || robotvel.length()>0.1 || abs(robotvrot)>0.5) ||bloc.pos_known==BallLocation::communicated ? 200 : 0);
      break;
    case preOpponentFreeKick:
      current = postOpponentFreeKick;
      ballpos = bloc.pos.toVec();
      ballknown = (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated);
      precision_addon = (((ballknown && (robotpos-bloc.pos.toVec()).length()>3000) || robotvel.length()>0.1 || abs(robotvrot)>0.5) ||bloc.pos_known==BallLocation::communicated ? 200 : 0);
      break;
    case preOpponentGoalKick:
      current = postOpponentGoalKick;
      ballpos = bloc.pos.toVec();
      ballknown = (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated);
      precision_addon = (((ballknown && (robotpos-bloc.pos.toVec()).length()>3000) || robotvel.length()>0.1 || abs(robotvrot)>0.5) ||bloc.pos_known==BallLocation::communicated ? 200 : 0);
      break;
    case preOpponentCornerKick:
      current = postOpponentCornerKick;
      ballpos = bloc.pos.toVec();
      ballknown = (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated);
      precision_addon = (((ballknown && (robotpos-bloc.pos.toVec()).length()>3000) || robotvel.length()>0.1 || abs(robotvrot)>0.5) ||bloc.pos_known==BallLocation::communicated ? 200 : 0);
      break;
    case preOpponentThrowIn:
      current = postOpponentThrowIn;
      ballpos = bloc.pos.toVec();
      ballknown = (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated);
      precision_addon = (((ballknown && (robotpos-bloc.pos.toVec()).length()>3000) || robotvel.length()>0.1 || abs(robotvrot)>0.5) ||bloc.pos_known==BallLocation::communicated ? 200 : 0);
      break;
    case preOpponentPenalty:
      current = postOpponentPenalty;
      ballpos = bloc.pos.toVec();
      ballknown = (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated);
      precision_addon = (((ballknown && (robotpos-bloc.pos.toVec()).length()>3000) || robotvel.length()>0.1 || abs(robotvrot)>0.5) ||bloc.pos_known==BallLocation::communicated ? 200 : 0);
      break;
    default:
      return sig;  // SIGreadey in anderem Zustand nicht interpretierbar
    }
    break;
  case SIGownKickOff:
    current = preOwnKickOff;
    break;
  case SIGopponentKickOff:
    current = preOpponentKickOff;
    break;
  case SIGownFreeKick:
    current = preOwnFreeKick;
    break;
  case SIGopponentFreeKick:
    current = preOpponentFreeKick;
    break;
  case SIGownGoalKick:
    current = preOwnGoalKick;
    break;
  case SIGopponentGoalKick:
    current = preOpponentGoalKick;
    break;
  case SIGownCornerKick:
    current = preOwnCornerKick;
    break;
  case SIGopponentCornerKick:
    current = preOpponentCornerKick;
    break;
  case SIGownThrowIn:
    current = preOwnThrowIn;
    break;
  case SIGopponentThrowIn:
    current = preOpponentThrowIn;
    break;
  case SIGownPenalty:
    current = preOwnPenalty;
    break;
  case SIGopponentPenalty:
    current = preOpponentPenalty;
    break;
  case SIGdroppedBall:
    current = preDroppedBall;
    break;
  case SIGtest1:
    current = testState1;
    break;
  case SIGtest2:
    current = testState2;
    break;
  case SIGtest3:
    current = testState3;
    break;
  case SIGtest4:
    current = testState4;
    break;
  case SIGtest5:
    current = testState5;
    break;
  case SIGtest6:
    current = testState6;
    break;
  case SIGtest7:
    current = testState7;
    break;
  case SIGtest8:
    current = testState8;
    break;
  default:
    return sig;   // anderes Signal
  }
  latest_update.update();
  return sig;
}


void RefereeStateMachine::update (const BallLocation& bloc, Vec robot, Vec vtrans, const ObstacleLocation& oloc) throw () {
  // Zeitkriterium
  if (current==postOpponentKickOff || current==postOpponentFreeKick || current==postOpponentGoalKick || current==postOpponentCornerKick  || current==postOpponentThrowIn) {
    if (latest_update.elapsed_msec ()>=10000) { // 10 Sekunden Warten
      current = freePlay;
      latest_update.update();
    }
  }

  if (!ballknown && (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated) && (current==postOpponentKickOff || current==postOpponentFreeKick || current==postOpponentGoalKick || current==postOpponentCornerKick  || current==postOpponentThrowIn)) {
    ballknown=true;
    ballpos = bloc.pos.toVec();
  }

  double kleineBallbewegung=150+vtrans.length()*200+precision_addon; // Der Roboter sollte stehen.
  /*  
  vector<Vec>::const_iterator oloc_posit = oloc.pos.begin();
  vector<double>::const_iterator oloc_widthit = oloc.width.begin();
  bool occluded_ball=false;
  Angle ballangle = (ball-robot).angle();
  Angle ballangle_diff = Angle::rad_angle (atan2 (110, (ball-robot).length()));
  double balldist = (ball-robot).length();
  while (oloc_posit<oloc.pos.end() && bk) {
    Angle obsangle = ((*oloc_posit)-robot).angle();
    Angle obsangle_diff = Angle::rad_angle (atan2 (0.5*(*oloc_widthit)+50, ((*oloc_posit)-robot).length()));
    if (balldist>((*oloc_posit)-robot).length() && !((obsangle+obsangle_diff).in_between (ballangle+ballangle_diff, ballangle-ballangle_diff) && (obsangle-obsangle_diff).in_between (ballangle+ballangle_diff, ballangle-ballangle_diff))) {
      occluded_ball=true;
      break;
    }
    oloc_posit++;
    oloc_widthit++;
  }
  if (occluded_ball && balldist>1500)
    kleineBallbewegung+=100;
  */
  if (bloc.pos_known==BallLocation::communicated)
    kleineBallbewegung+=200;
  if (kleineBallbewegung>500) kleineBallbewegung=500;
  double grosseBallbewegung=1000;
  double ballabstand=(robot-bloc.pos.toVec()).length();

  // Ballbewegungskriterium
  double len = (ballpos-bloc.pos.toVec()).length();
  if (current==postOpponentKickOff || current==postOpponentFreeKick || current==postOpponentGoalKick || current==postOpponentCornerKick  || current==postOpponentThrowIn || current==postOpponentPenalty) {
    if (ballknown && (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated)) {
        if ((ballabstand<2500 && len>kleineBallbewegung) || (len>grosseBallbewegung)) {
        // bei Ballbewegung
        if (current==postOpponentPenalty)
          current = opponentPenalty;
        else
          current = freePlay;
        latest_update.update();
      }
    }
  }
  // Ballbewegung in Strafraum bei gegnerischem Penalty
  if (current==postOpponentPenalty) {
//    if ((ballknown && bk && len>150) || (bk && ball.y<penalty_marker_y-500))	 
    if ((ballknown && bloc.pos_known==BallLocation::known && len>kleineBallbewegung) || (bloc.pos_known==BallLocation::known && bloc.pos.y<penalty_marker_y-500))
      current = opponentPenalty;
  }
  // Ballbewegung ausserhalb Mittelkreis bei gegnerischem Anstoss
  if (current==postOpponentKickOff) {
    if (((bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated) && bloc.pos.toVec().length()>700 && robot.length()<2500) || ((bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated) && bloc.pos.toVec().length()>1200))
      current = freePlay;
  }
}
