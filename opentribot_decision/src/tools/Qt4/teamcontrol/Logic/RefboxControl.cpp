
#include "RefboxControl.h"
#include "../States/RemoteBlackboard.h"
#include <iostream>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;


RefboxControl::RefboxControl (const Tribots::ConfigReader& cfg) throw (Tribots::TribotsException) : refbox ("refbox_client.log"), automaton(NULL), inverse_automaton(NULL), synch_signal (0) {
  FieldGeometry nfg (cfg);
  REMBB.team_state.field_geometry = nfg;
  cfg.get ("refbox_IP", REMBB.team_state.refbox_ip);
  cfg.get ("refbox_port", REMBB.team_state.refbox_port);
  std::string s;
  if (cfg.get ("own_half", s)) {
    if (s=="blue")
      REMBB.team_state.own_half=-1;
    else
      REMBB.team_state.own_half=+1;
  }
  if (cfg.get ("team_color", s)) {
    if (s=="magenta")
      REMBB.team_state.team_color=-1;
    else
      REMBB.team_state.team_color=+1;
  }
  own_half=REMBB.team_state.own_half;
  team_color=REMBB.team_state.team_color;
  was_connected_to_refbox=false;
  automaton = new RefereeStateMachine (REMBB.team_state.field_geometry, team_color, stopRobot);
  inverse_automaton = new RefereeStateMachine (REMBB.team_state.field_geometry, -team_color, stopRobot);
  REMBB.team_state.refstate = automaton->get_state();
}

RefboxControl::~RefboxControl () throw () {
  if (automaton)
    delete automaton;
  if (inverse_automaton)
    delete inverse_automaton;
}

void RefboxControl::update () throw () {
  // own half
  if (own_half!=REMBB.team_state.own_half) {
    own_half=REMBB.team_state.own_half;
    for (unsigned int i=0; i<REMBB.robot_state.size(); i++)
      REMBB.robot_state[i].desired_own_half = -REMBB.robot_state[i].own_half;
  }

  // team color
  if (team_color!=REMBB.team_state.team_color) {
    team_color=REMBB.team_state.team_color;
    automaton->set_team_color (team_color);
    inverse_automaton->set_team_color (-team_color);
  }

  // Refbox-Client connect/disconnect
  if (was_connected_to_refbox != REMBB.team_state.refbox_connected) {
    if (REMBB.team_state.refbox_connected) {
      refbox.connect (REMBB.team_state.refbox_ip.c_str(), REMBB.team_state.refbox_port);
    } else {
      refbox.disconnect ();
    }
  }

  // Refbox abfragen
  RefboxClient::RefboxMessage refboxsignal = refbox.listen();
  was_connected_to_refbox=REMBB.team_state.refbox_connected=refbox.is_connected();
  REMBB.team_state.refbox_okay=refbox.is_okay();
  if (refboxsignal.signal!=SIGnop)
    REMBB.team_state.refbox_signal=refboxsignal.signal;
  if (refboxsignal.signal==SIGmagentaGoalScored || refboxsignal.signal==SIGcyanGoalScored) {
    if ((REMBB.team_state.team_color>0 && refboxsignal.signal==SIGcyanGoalScored) || (REMBB.team_state.team_color<0 && refboxsignal.signal==SIGmagentaGoalScored))
      REMBB.team_state.own_score++;
    else
      REMBB.team_state.opponent_score++;    
    for (unsigned int j=0; j<REMBB.robot_state.size(); j++) {
      int own_color = (REMBB.robot_state[j].own_half*REMBB.team_state.own_half*REMBB.team_state.team_color);
      bool own_goal = (own_color>0 && refboxsignal.signal==SIGcyanGoalScored) || (own_color<0 && refboxsignal.signal==SIGmagentaGoalScored);
      string msg = string (own_goal ? "own" : "opponent") + string(" goal scored!");
      REMBB.robot_state[j].message_board.publish (msg);
    }
  }
  for (unsigned int i=0; i<refboxsignal.messages.size(); i++) {
    for (unsigned int j=0; j<REMBB.robot_state.size(); j++) {
      REMBB.robot_state[j].message_board.publish (refboxsignal.messages[i]);
    }
  }

  // Zustandsuebergang
  int state_change=0;  // state_change&1 --> Tribots Refereestate hat gewechselt, state_change&2 --> Gegner Refereestate hat gewechselt
  BallLocation bloc_null;
  bloc_null.pos_known=BallLocation::unknown;
  if (REMBB.team_state.refbox_signal!=SIGnop) {
    automaton->update (REMBB.team_state.refbox_signal, bloc_null, Vec::zero_vector, Vec::zero_vector, 0);
    inverse_automaton->update (REMBB.team_state.refbox_signal, bloc_null, Vec::zero_vector, Vec::zero_vector, 0);
    state_change=3;
  }
  Tribots::RefereeState opponent_refstate = inverse_automaton->get_state();
  ObstacleLocation oloc_dummy;
  oloc_dummy.pos.clear();
  oloc_dummy.width.clear();
  automaton->update (bloc_null, Vec::zero_vector, Vec::zero_vector, oloc_dummy);
  inverse_automaton->update (bloc_null,Vec::zero_vector, Vec::zero_vector, oloc_dummy);
  if (opponent_refstate!=inverse_automaton->get_state())
    state_change |= 2;
  if (REMBB.team_state.refstate!=automaton->get_state())
    state_change |= 1;
  REMBB.team_state.refbox_signal=SIGnop;   // zuruecksetzen
  REMBB.team_state.refstate=automaton->get_state();
  for (unsigned int i=0; i<REMBB.robot_state.size(); i++) {
    RefereeState desire = (REMBB.robot_state[i].own_half==own_half ? automaton->get_state() : inverse_automaton->get_state());
    RefereeState actual = REMBB.robot_state[i].refstate;
    if (actual==freePlay && (desire==postOpponentKickOff || desire==postOpponentThrowIn || desire==postOpponentGoalKick || desire==postOpponentCornerKick || desire==postOpponentFreeKick))
      continue;   // Roboter hat gemerkt, dass sich der Ball bewegt hat
    if (desire==postOpponentPenalty && actual==opponentPenalty)
      continue;   // Roboter hat gemerkt, dass sich der Ball bewegt hat
    if ((REMBB.robot_state[i].own_half==own_half && ( state_change&1)) || (REMBB.robot_state[i].own_half!=own_half && ( state_change&2)))
      REMBB.robot_state[i].desired_refstate = desire;
  }
}

void RefboxControl::update_synch () throw () {
  // Synchronisationssignal
  Time now;
  if (now>synch_signal_time) {
    REMBB.team_state.synch_signal = synch_signal++;
    REMBB.team_state.send_synch_signal = true;
    synch_signal_time.add_sec (10);
  } else
    REMBB.team_state.send_synch_signal = false;
}
