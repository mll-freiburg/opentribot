
#include "TeamcontrolSlaveClient.h"
#include "../States/RemoteBlackboard.h"
#include <arpa/inet.h>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

TeamcontrolSlaveClient::TeamcontrolSlaveClient () throw () : message_id (0), wrong_mid (0) {;}

TeamcontrolSlaveClient::~TeamcontrolSlaveClient () throw () {;}

bool TeamcontrolSlaveClient::connect (const std::string& hostname, unsigned int port) throw () {
  bool success = comm.init_as_client (hostname.c_str(), port);
  comm.putMessageID(message_id);
  comm.putHello ();
  comm.send();
  wrong_mid=0;
  return success;
}

void TeamcontrolSlaveClient::unconnect () throw () {
  comm.close();
}

bool TeamcontrolSlaveClient::is_connected () const throw () {
  return comm.started();
}

bool TeamcontrolSlaveClient::send () throw () {
  if (wrong_mid>3) {
    comm.putMessageID (message_id+1);
    comm.putHello ();
    comm.send ();
//    cerr << "Falsche MID: reset\n";
    return false;
  }
  BlackboardState state = REMBB;
  unsigned int i=0;
  while (i!=state.robot_state.size()) {
    bool found=false;
    for (unsigned int j=0; j<robot_ids_received.size(); j++) {
      if (state.robot_state[i].id==robot_ids_received[j]) {
        found=true;
        break;
      }
    }
    if (found) {
      i++;
    } else {
      state.robot_state.erase (state.robot_state.begin()+i);
    }
  }
  bool success = comm.putRemoteBlackboardState (state);
  success &= comm.putMessageID (message_id+1);
//  cerr << "SND: " << message_id+1 << endl;
  if (REMBB.robot_state.size()>0)
  success &= (comm.send()>0);
  if (success) {
    latest_own_state=state;
    for (unsigned int j=0; j<REMBB.robot_state.size(); j++) {
      REMBB.robot_state[j].debug_image_request=false;
      REMBB.robot_state[j].slhint_request=false;
      REMBB.robot_state[j].sl_mirror_hint_request=false;
      REMBB.robot_state[j].drive_to_request=false;
      if (REMBB.robot_state[j].comm_started && REMBB.robot_state[j].show_message_board) {
        cout << "MessageBoard " << REMBB.robot_state[j].name << " Ausgang:\n";
        const vector<string>& outgoing_messages = REMBB.robot_state[j].message_board.get_outgoing();
        for (unsigned int i=0; i<outgoing_messages.size(); i++)
          cout << outgoing_messages[i] << '\n';
      }
    }
    REMBB.coach_state.extra_message="";
  }
  return success;
}

bool TeamcontrolSlaveClient::receive () throw () {
  unsigned short int msg_id;
  bool success = (comm.receive()>0  && comm.getMessageID (msg_id));
  BlackboardState remstate;
  success &= comm.getRemoteBlackboardState (remstate);
//  bool suc2=success;
//  if (suc2)
//    cerr << "RCV: " << msg_id << ' ' << message_id;
  bool mid_okay = (msg_id>message_id || (message_id>40000 && msg_id<20000));
  if (success) {
    if (!mid_okay) {
      wrong_mid++;
    } else {
      wrong_mid=0;
    }
  }
  success &= mid_okay;
  if (!success) {
//    if (suc2)
//      cerr << " " << wrong_mid << " fail\n";
    return false;
  }
//  cerr << " " << wrong_mid << " success\n";
  message_id=msg_id;

  robot_ids_received.resize (remstate.robot_state.size());
  for (unsigned int i=0; i<remstate.robot_state.size(); i++)
    robot_ids_received[i]=remstate.robot_state[i].id;

  // Informationen ins REMBB einpflegen
  // robot_state: (DONE)
  for (unsigned int i=0; i<remstate.robot_state.size(); i++) {
    bool found=false;
    unsigned int k=0;
    for (unsigned int j=0; j<REMBB.robot_state.size(); j++) {
      if (remstate.robot_state[i].id==REMBB.robot_state[j].id) {
        found=true;
        k=j;
        break;
      }
    }
    if (!found) {
      cerr << "Kein Widget fuer Roboter " << remstate.robot_state[i].id << " vorhanden, ignoriere Roboter\n";
      continue;
    }

    bool found2=false;
    unsigned int k2=0;
    for (unsigned int j=0; j<latest_own_state.robot_state.size(); j++) {
      if (remstate.robot_state[i].id==latest_own_state.robot_state[j].id) {
        found2=true;
        k2=j;
        break;
      }
    }

    // Robotername, id, local_id etc. werden nicht angepasst
    if (!found2 || REMBB.robot_state[k].ip==latest_own_state.robot_state[k].ip)
      REMBB.robot_state[k].ip=remstate.robot_state[i].ip;
    if (!found2 || REMBB.robot_state[k].port==latest_own_state.robot_state[k].port)
      REMBB.robot_state[k].port=remstate.robot_state[i].port;
    REMBB.robot_state[k].comm_started=remstate.robot_state[i].comm_started;  // direkt, da Veraenderung ueber requests
    REMBB.robot_state[k].comm_okay=remstate.robot_state[i].comm_okay;  // direkt, da nur Info
    REMBB.robot_state[k].comm_interrupted=remstate.robot_state[i].comm_interrupted;
    REMBB.robot_state[k].comm_statistics_send=remstate.robot_state[i].comm_statistics_send;
    REMBB.robot_state[k].comm_statistics_receive=remstate.robot_state[i].comm_statistics_receive;
    REMBB.robot_state[k].robot_pos=remstate.robot_state[i].robot_pos;
    REMBB.robot_state[k].occ_grid=remstate.robot_state[i].occ_grid;
    REMBB.robot_state[k].ball_pos=remstate.robot_state[i].ball_pos;
    REMBB.robot_state[k].obs_pos=remstate.robot_state[i].obs_pos;
    REMBB.robot_state[k].refstate=remstate.robot_state[i].refstate;  // direkt, da Veraenderung ueber requests
    REMBB.robot_state[k].in_game=remstate.robot_state[i].in_game;  // direkt, da Veraenderung ueber requests
    REMBB.robot_state[k].own_half=remstate.robot_state[i].own_half;  // direkt, da Veraenderung ueber requests
    REMBB.robot_state[k].robot_data=remstate.robot_state[i].robot_data;
    REMBB.robot_state[k].visible_objects=remstate.robot_state[i].visible_objects;
    REMBB.robot_state[k].own_score=remstate.robot_state[i].own_score;
    REMBB.robot_state[k].opponent_score=remstate.robot_state[i].opponent_score;
    REMBB.robot_state[k].yellow_cards=remstate.robot_state[i].yellow_cards;
    REMBB.robot_state[k].playertype=remstate.robot_state[i].playertype;  // direkt, da Veraenderung ueber requests
    REMBB.robot_state[k].playerrole=remstate.robot_state[i].playerrole;  // direkt, da Veraenderung ueber requests
    REMBB.robot_state[k].list_players=remstate.robot_state[i].list_players;
    REMBB.robot_state[k].list_roles=remstate.robot_state[i].list_roles;
    REMBB.robot_state[k].message_board=remstate.robot_state[i].message_board;
    // tactics_board wird nicht uebertragen

    if (!found2 || REMBB.robot_state[k].desired_connect==latest_own_state.robot_state[k2].desired_connect)
      REMBB.robot_state[k].desired_connect=remstate.robot_state[i].desired_connect;
    if (!found2 || REMBB.robot_state[k].desired_in_game==latest_own_state.robot_state[k2].desired_in_game)
      REMBB.robot_state[k].desired_in_game=remstate.robot_state[i].desired_in_game;
    if (!found2 || REMBB.robot_state[k].desired_own_half==latest_own_state.robot_state[k2].desired_own_half)
      REMBB.robot_state[k].desired_own_half=remstate.robot_state[i].desired_own_half;
    if (!found2 || REMBB.robot_state[k].desired_refstate==latest_own_state.robot_state[k2].desired_refstate)
      REMBB.robot_state[k].desired_refstate=remstate.robot_state[i].desired_refstate;
    if (!found2 || REMBB.robot_state[k].desired_playertype==latest_own_state.robot_state[k2].desired_playertype)
      REMBB.robot_state[k].desired_playertype=remstate.robot_state[i].desired_playertype;
    if (!found2 || REMBB.robot_state[k].desired_playerrole==latest_own_state.robot_state[k2].desired_playerrole)
      REMBB.robot_state[k].desired_playerrole=remstate.robot_state[i].desired_playerrole;
    if (!found2 || REMBB.robot_state[k].desired_yellow_cards==latest_own_state.robot_state[k2].desired_yellow_cards)
      REMBB.robot_state[k].desired_yellow_cards=remstate.robot_state[i].desired_yellow_cards;

    // keine Uebetragung von:
    // robot_data_request, obstacle_request, visible_object_request, joystick_request, slhint_request, debug_image_request, sl_mirror_hint_request
    // slhint_pos, slhint_angle, sl_mirror_hint, show_message_board, field_geoemtry

    if (remstate.robot_state[i].comm_started && REMBB.robot_state[i].show_message_board) {
      cout << "MessageBoard " << remstate.robot_state[i].name << " Eingang:\n";
      const vector<string>& incoming_messages = remstate.robot_state[i].message_board.get_incoming();
      for (unsigned int i=0; i<incoming_messages.size(); i++)
        cout << incoming_messages[i] << '\n';
    }
    
  }

  // coach_state:
  if (REMBB.coach_state.policy_name==latest_own_state.coach_state.policy_name)
    REMBB.coach_state.policy_name=remstate.coach_state.policy_name;
  if (REMBB.coach_state.ball_position_mode==latest_own_state.coach_state.ball_position_mode)
    REMBB.coach_state.ball_position_mode=remstate.coach_state.ball_position_mode;
  if (REMBB.coach_state.ball_posession_mode==latest_own_state.coach_state.ball_posession_mode)
    REMBB.coach_state.ball_posession_mode=remstate.coach_state.ball_posession_mode;
  if (REMBB.coach_state.broadcast_mode==latest_own_state.coach_state.broadcast_mode)
    REMBB.coach_state.broadcast_mode=remstate.coach_state.broadcast_mode;
  if (REMBB.coach_state.teammate_mode==latest_own_state.coach_state.teammate_mode)
    REMBB.coach_state.teammate_mode=remstate.coach_state.teammate_mode;
  if (REMBB.coach_state.tactics_mode==latest_own_state.coach_state.tactics_mode)
    REMBB.coach_state.tactics_mode=remstate.coach_state.tactics_mode;
  if (REMBB.coach_state.sl_mirror_hint_mode==latest_own_state.coach_state.sl_mirror_hint_mode)
    REMBB.coach_state.sl_mirror_hint_mode=remstate.coach_state.sl_mirror_hint_mode;
  // Liste aller Policies ignorieren, wird vom Master aus dem Configfile gelesen
  std::map<std::string, std::string>::iterator it_tb = REMBB.coach_state.tactics_board.begin();
  while (it_tb!=REMBB.coach_state.tactics_board.end()) { // Taktikboard Attributweise vergleichen
    if (latest_own_state.coach_state.tactics_board [it_tb->first]==it_tb->second) {
      it_tb->second=remstate.coach_state.tactics_board [it_tb->first];
    }
    it_tb++;
  }
  // Liste aller Taktikattribute ignorieren, wird vom Master aus dem Configfile gelesen

  // joystick_state (wird vorlaeufig ignoriert)
  // team_state:
  if (REMBB.team_state.own_half==latest_own_state.team_state.own_half)
    REMBB.team_state.own_half=remstate.team_state.own_half;
  if (REMBB.team_state.team_color==latest_own_state.team_state.team_color)
    REMBB.team_state.team_color=remstate.team_state.team_color;
  REMBB.team_state.refstate=remstate.team_state.refstate;  // refstate wird immer vom Master gesetzt (Aktualisierung erfolgt ueber refbox_signal)
  if (REMBB.team_state.own_score==latest_own_state.team_state.own_score)
    REMBB.team_state.own_score=remstate.team_state.own_score;
  if (REMBB.team_state.opponent_score==latest_own_state.team_state.opponent_score)
    REMBB.team_state.opponent_score=remstate.team_state.opponent_score;
  REMBB.team_state.send_lines_rate=remstate.team_state.send_lines_rate;
  REMBB.team_state.comm_rate=remstate.team_state.comm_rate;
  REMBB.team_state.field_geometry=remstate.team_state.field_geometry;
  // Refereebox-Anbindung wird nicht angepasst
  // letztes Refboxsignal wird nicht benoetigt

  // help_state:
  REMBB.help_state=remstate.help_state;

  latest_receive_time.update();
  return true;
}

std::string TeamcontrolSlaveClient::commStatus () const throw () {
  if (!is_connected()) {
    return string ("Kommunikation nicht gestartet");
  } else {
    stringstream inout;
    unsigned long int tt = latest_receive_time.elapsed_msec();
    char buffer [200];
    const char* add = inet_ntop (AF_INET, &comm.partner_address().sin_addr, buffer, 200);
    inout << "Komunikation mit " << add << ':' << ntohs(comm.partner_address().sin_port) << "; letzte Nachricht vor ";
    if (tt>1000) {
      inout << tt/1000 << "s\n";
    } else {
      inout << tt << "ms\n";
    }
    string line;
    getline (inout, line);
    line+="\n";
    return line;
  }
}

bool TeamcontrolSlaveClient::comm_interrupted () const throw () {
  return (latest_receive_time.elapsed_msec()>5000);
}
