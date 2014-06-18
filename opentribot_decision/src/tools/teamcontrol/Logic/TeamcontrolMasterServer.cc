
#include "TeamcontrolMasterServer.h"
#include "../States/RemoteBlackboard.h"
#include <sstream>
#include <arpa/inet.h>
#include <algorithm>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

TeamcontrolMasterServer::TeamcontrolMasterServer () throw () {;}

TeamcontrolMasterServer::~TeamcontrolMasterServer () throw () {;}

bool TeamcontrolMasterServer::connect (unsigned int port) throw () {
  return comm.init_as_server (port);
}

void TeamcontrolMasterServer::unconnect () throw () {
  clients.clear();
  return comm.close();
}

bool TeamcontrolMasterServer::is_connected () const throw () {
  return comm.started();
}

bool TeamcontrolMasterServer::receive () throw () {
  for (unsigned int i=0; i<clients.size(); i++) {
    clients[i].do_send=false;
  }
  bool received_anything=false;
  unsigned short msg_id;
  while (comm.receive() && comm.getMessageID (msg_id)) {
    unsigned int index=0;
    while (index<clients.size()) {
      if (clients[index].address==comm.partner_address()) {
        break;
      }
      index++;
    }
    if (index==clients.size()) {
      // neuer Client
      PartnerStruct neu;
      neu.latest_other_state_valid=false;
      neu.do_send=true;
      neu.address=comm.partner_address();
      neu.message_id = msg_id;
      clients.push_back (neu);
      received_anything=true;
    }
//    cerr << "RCV: " << msg_id << ' ' << clients[index].message_id;
    if (comm.getHello()) {
      clients[index].do_send=true;
      received_anything=true;
      clients[index].latest_other_state_valid=false;
      clients[index].latest_receive_time.update();
      clients[index].message_id = msg_id;
    }
    if (comm.getBye()) {
      clients[index].do_send=false;
      clients[index].latest_other_state_valid=false;
      clients[index].latest_receive_time.add_sec(-1000);  // um die Verbindung anschliessend zu schliessen
    }

    BlackboardState remstate;
    if (comm.getRemoteBlackboardState (remstate) && (msg_id>clients[index].message_id || (clients[index].message_id>40000 && msg_id<20000))) {
      clients[index].message_id=msg_id;
      clients[index].do_send=true;
      received_anything=true;
      clients[index].latest_receive_time.update();

      if (!clients[index].latest_other_state_valid) {
        clients[index].latest_other_state=remstate;
        clients[index].latest_other_state_valid=true;
        continue;  // beim ersten Mal nur Zustand uebrnehmen, keine Aenderungen einpflegen
      }

      PartnerStruct& ps = clients[index];
      unsigned int msg_index = clients[index].latest_own_state_msg_id.size();
      for (unsigned int i=0; i<clients[index].latest_own_state_msg_id.size(); i++) {
        if (clients[index].latest_own_state_msg_id[i]+1==msg_id) {
          msg_index=i;
          break;
        }
      }
      if (msg_index>=clients[index].latest_own_state_msg_id.size()) {
        cerr << "Problem: keinen alten Sendezustand gefunden!?\n";
        continue;
      }
      clients[index].latest_own_state_msg_id.erase (clients[index].latest_own_state_msg_id.begin(), clients[index].latest_own_state_msg_id.begin()+msg_index);
      clients[index].latest_own_state.erase (clients[index].latest_own_state.begin(), clients[index].latest_own_state.begin()+msg_index);
      BlackboardState& latestown = ps.latest_own_state[0];

      // Informationen ins REMBB einpflegen
      // robot_state:
      deque<unsigned int> processed_ids;
      for (unsigned int ii=0; ii<remstate.robot_state.size(); ii++) {
        unsigned int i=remstate.robot_state.size()-ii-1;
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
        for (unsigned int j=0; j<ps.latest_other_state.robot_state.size(); j++) {
          if (remstate.robot_state[i].id==ps.latest_other_state.robot_state[j].id) {
            found=true;
            k2=j;
            break;
          }
        }
        unsigned int id = remstate.robot_state[i].id;
        bool secondtime=false;
        if (find (processed_ids.begin(), processed_ids.end(), id)!=processed_ids.end()) {
          secondtime=true;
          // ein und denselben Roboter nicht mehrfach aktualisieren
        } else {
          processed_ids.push_back(id);
        }

        if (!secondtime) {
          // name wird nicht angepasst
          if (latestown.robot_state[k].ip==REMBB.robot_state[k].ip)
            REMBB.robot_state[k].ip = remstate.robot_state[i].ip;
          if (latestown.robot_state[k].port==REMBB.robot_state[k].port)
            REMBB.robot_state[k].port = remstate.robot_state[i].port;
          // id und local_id werden nicht angepasst
          // comm_started wird nicht angepasst
          // comm_okay, comm_interrupted, comm_statistics_send, comm_statistics_receive werden nicht angepasst
          // robot_pos, occ_grid, ball_pos, obs_pos, refstate, in_game, own_half, robot_data, visible_objects, own_score, opponent_score, yellow_cards werden nicht angepasst
          if (latestown.robot_state[k].desired_yellow_cards==REMBB.robot_state[k].desired_yellow_cards && (!found2 || ps.latest_other_state.robot_state[k2].desired_yellow_cards!=remstate.robot_state[i].desired_yellow_cards))
            REMBB.robot_state[k].desired_yellow_cards = remstate.robot_state[i].desired_yellow_cards;
          // playertype, playerrole, list_players, list_roles werden nicht angepasst
          // message_board, tactics_board werden nicht angepasst
          if (latestown.robot_state[k].desired_connect==REMBB.robot_state[k].desired_connect && (!found2 || ps.latest_other_state.robot_state[k2].desired_connect!=remstate.robot_state[i].desired_connect))
            REMBB.robot_state[k].desired_connect = remstate.robot_state[i].desired_connect;
          if (latestown.robot_state[k].desired_in_game==REMBB.robot_state[k].desired_in_game && (!found2 || ps.latest_other_state.robot_state[k2].desired_in_game!=remstate.robot_state[i].desired_in_game))
            REMBB.robot_state[k].desired_in_game = remstate.robot_state[i].desired_in_game;
          if (latestown.robot_state[k].desired_own_half==REMBB.robot_state[k].desired_own_half && (!found2 || ps.latest_other_state.robot_state[k2].desired_own_half!=remstate.robot_state[i].desired_own_half))
            REMBB.robot_state[k].desired_own_half = remstate.robot_state[i].desired_own_half;
          if (latestown.robot_state[k].desired_refstate==REMBB.robot_state[k].desired_refstate && (!found2 || ps.latest_other_state.robot_state[k2].desired_refstate!=remstate.robot_state[i].desired_refstate))
            REMBB.robot_state[k].desired_refstate = remstate.robot_state[i].desired_refstate;
          if (latestown.robot_state[k].desired_playertype==REMBB.robot_state[k].desired_playertype && (!found2 || ps.latest_other_state.robot_state[k2].desired_playertype!=remstate.robot_state[i].desired_playertype))
            REMBB.robot_state[k].desired_playertype = remstate.robot_state[i].desired_playertype;
          if (latestown.robot_state[k].desired_playerrole==REMBB.robot_state[k].desired_playerrole && (!found2 || ps.latest_other_state.robot_state[k2].desired_playerrole!=remstate.robot_state[i].desired_playerrole))
            REMBB.robot_state[k].desired_playerrole = remstate.robot_state[i].desired_playerrole;
        }
        REMBB.robot_state[k].robot_data_request |= remstate.robot_state[i].robot_data_request;
        REMBB.robot_state[k].obstacle_request |= remstate.robot_state[i].obstacle_request;
        REMBB.robot_state[k].visible_object_request |= remstate.robot_state[i].visible_object_request;
        REMBB.robot_state[k].joystick_request |= remstate.robot_state[i].joystick_request;
        REMBB.robot_state[k].debug_image_request |= remstate.robot_state[i].debug_image_request;
        if (!REMBB.robot_state[k].drive_to_request) {
          REMBB.robot_state[k].drive_to_request = remstate.robot_state[i].drive_to_request;
          REMBB.robot_state[k].drive_to_pos = remstate.robot_state[i].drive_to_pos;
          REMBB.robot_state[k].drive_to_angle = remstate.robot_state[i].drive_to_angle;
        }
        if (!REMBB.robot_state[k].slhint_request) {
          REMBB.robot_state[k].slhint_request = remstate.robot_state[i].slhint_request;
          REMBB.robot_state[k].slhint_pos = remstate.robot_state[i].slhint_pos;
          REMBB.robot_state[k].slhint_angle = remstate.robot_state[i].slhint_angle;
        }
        // sl_mirror_hint wird nicht angepasst
        // show_message_board ist lokal
        // field_geometry wird nicht angepasst
      }
      // coach_state:
      if (latestown.coach_state.policy_name==REMBB.coach_state.policy_name && ps.latest_other_state.coach_state.policy_name!=remstate.coach_state.policy_name)
        REMBB.coach_state.policy_name = remstate.coach_state.policy_name;
      if (latestown.coach_state.ball_position_mode==REMBB.coach_state.ball_position_mode && ps.latest_other_state.coach_state.ball_position_mode!=remstate.coach_state.ball_position_mode)
        REMBB.coach_state.ball_position_mode = remstate.coach_state.ball_position_mode;
      if (latestown.coach_state.ball_posession_mode==REMBB.coach_state.ball_posession_mode && ps.latest_other_state.coach_state.ball_posession_mode!=remstate.coach_state.ball_posession_mode)
        REMBB.coach_state.ball_posession_mode = remstate.coach_state.ball_posession_mode;
      if (latestown.coach_state.broadcast_mode==REMBB.coach_state.broadcast_mode && ps.latest_other_state.coach_state.broadcast_mode!=remstate.coach_state.broadcast_mode)
        REMBB.coach_state.broadcast_mode = remstate.coach_state.broadcast_mode;
      if (latestown.coach_state.teammate_mode==REMBB.coach_state.teammate_mode && ps.latest_other_state.coach_state.teammate_mode!=remstate.coach_state.teammate_mode)
        REMBB.coach_state.teammate_mode = remstate.coach_state.teammate_mode;
      if (latestown.coach_state.tactics_mode==REMBB.coach_state.tactics_mode && ps.latest_other_state.coach_state.tactics_mode!=remstate.coach_state.tactics_mode)
        REMBB.coach_state.tactics_mode = remstate.coach_state.tactics_mode;
      if (latestown.coach_state.sl_mirror_hint_mode==REMBB.coach_state.sl_mirror_hint_mode && ps.latest_other_state.coach_state.sl_mirror_hint_mode!=remstate.coach_state.sl_mirror_hint_mode)
        REMBB.coach_state.sl_mirror_hint_mode = remstate.coach_state.sl_mirror_hint_mode;
      if (REMBB.coach_state.extra_message.length()==0)
        REMBB.coach_state.extra_message=remstate.coach_state.extra_message;
      // Liste aller Policies ignorieren, wird vom Master aus dem Configfile gelesen
      std::map<std::string, std::string>::iterator it_tb = REMBB.coach_state.tactics_board.begin();
      while (it_tb!=REMBB.coach_state.tactics_board.end()) { // Taktikboard Attributweise vergleichen
        if (latestown.coach_state.tactics_board [it_tb->first]==it_tb->second) {
          it_tb->second=remstate.coach_state.tactics_board [it_tb->first];
        }
        it_tb++;
      }
      // Liste aller Taktikattribute ignorieren, wird vom Master aus dem Configfile gelesen

      // joystick_state:
      // wird vorlaeufig ignoriert, ggf. spaetere Erweiterung, um Joysticksteuerung vom Slave zuzulassen

      // team_state:
      // nur own_half, team_color, refbox_signal, own_score, opponent_score werden ggf. eingepflegt
      // Refereeboxanbindung, Kommunikationszyklus sind Master-privat
      if (latestown.team_state.own_half==REMBB.team_state.own_half && ps.latest_other_state.team_state.own_half!=remstate.team_state.own_half)
        REMBB.team_state.own_half = remstate.team_state.own_half;
      if (latestown.team_state.team_color==REMBB.team_state.team_color && ps.latest_other_state.team_state.team_color!=remstate.team_state.team_color)
        REMBB.team_state.team_color = remstate.team_state.team_color;
      if (latestown.team_state.localization_side==REMBB.team_state.localization_side && ps.latest_other_state.team_state.localization_side!=remstate.team_state.localization_side)
        REMBB.team_state.localization_side = remstate.team_state.localization_side;
      if (latestown.team_state.opponent_score==REMBB.team_state.opponent_score && ps.latest_other_state.team_state.opponent_score!=remstate.team_state.opponent_score)
        REMBB.team_state.opponent_score = remstate.team_state.opponent_score;
      if (latestown.team_state.own_score==REMBB.team_state.own_score && ps.latest_other_state.team_state.own_score!=remstate.team_state.own_score)
        REMBB.team_state.own_score = remstate.team_state.own_score;
      if (latestown.team_state.refbox_signal==REMBB.team_state.refbox_signal && ps.latest_other_state.team_state.refbox_signal!=remstate.team_state.refbox_signal)
        REMBB.team_state.refbox_signal = remstate.team_state.refbox_signal;

      // help_state:
      // da Programmlogik Master-privat, keine Eintraege vom Slave akzeptieren

      ps.latest_other_state=remstate;
    }
  }

  return received_anything;
}

bool TeamcontrolMasterServer::send () throw () {
  bool send_anything=false;
  unsigned int i=0;
  while (i<clients.size()) {
    if (clients[i].do_send) {
      clients[i].message_id++;

      clients[i].latest_own_state.push_back (REMBB);
      clients[i].latest_own_state_msg_id.push_back (clients[i].message_id);

      bool success=comm.putRemoteBlackboardState (REMBB);
      success &= comm.putMessageID (clients[i].message_id);
      success&=comm.sendto(clients[i].address);
      send_anything|=success;
//      cerr << "SND: " << clients[i].message_id << '\n';
      i++;
    } else if (clients[i].latest_receive_time.elapsed_msec()<5000) {
      // gerade nichts bekommen, aber noch nicht aus der Client-Liste entfernen
      i++;
    } else {
      // lange keine Nachricht mehr empfangen: Client entfernen
      clients.erase (clients.begin()+i);
    }
  }
  return send_anything;
}

std::string TeamcontrolMasterServer::commStatus () const throw () {
  if (!is_connected()) {
    return string ("Kommunikation nicht gestartet");
  } else if (clients.size()==0) {
    return string ("keine aktiven Clients\n");
  } else {
    stringstream inout;
    for (unsigned int i=0; i<clients.size(); i++) {
      char buffer [200];
      const char* add = inet_ntop (AF_INET, &(clients[i].address.sin_addr), buffer, 200);
      inout << "Client " << add << ':' << ntohs(clients[i].address.sin_port) << "; letzte Nachricht vor " << clients[i].latest_receive_time.elapsed_msec() << " ms\n";
    }
    string msg;
    while (!inout.eof()) {
      string line;
      getline (inout, line);
      msg+=line+"\n";
    }
    return msg;
  }
}
