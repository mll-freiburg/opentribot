
#include "RemoteRobot.h"
#include "../States/RemoteBlackboard.h"
#include <iostream>
#include <cmath>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;


RemoteRobot::RemoteRobot (const Tribots::ConfigReader& cfg, unsigned int n, const std::string& rid) throw () {
  robot=n;
  if (!cfg.get ((rid+string("::name")).c_str(), REMBB.robot_state[robot].name))
    REMBB.robot_state[robot].name = rid;
  cfg.get ((rid+string("::ip")).c_str(), REMBB.robot_state[robot].ip);
  cfg.get ((rid+string("::port")).c_str(), REMBB.robot_state[robot].port);
  if (!cfg.get ((rid+string("::id")).c_str(), REMBB.robot_state[robot].id))
    cerr << "Warnung: Nummer des Roboters " << REMBB.robot_state[robot].name << " fehlt!\n";
  until_send_lines=0;
  first_receive=true;
  ping.setAddress (REMBB.robot_state[robot].ip);
  ping.start();
}

RemoteRobot::~RemoteRobot () throw () {;}

void RemoteRobot::receive () throw () {
  // Nachrichten empfangen
  unsigned int num_received_packets = comm.receive ();

  vector<string> stringlist;
  string s;
  int an_int;
  unsigned int an_uint;
  TacticsBoard tb;
  unsigned int ownsc, oppsc, yell;

  // Wenn Roboter explizit 'bye' sendet, Verbindung schliessen; sollte normalerweise nicht auftreten
  if (comm.getBye())
    REMBB.robot_state[robot].desired_connect=false;

  // Spielertypliste
  stringlist.clear();
  if (comm.getPlayerTypeList(stringlist)) {
    REMBB.robot_state[robot].list_players = stringlist;
    playertypelist_request=false;
  }

  // Spielerrollenliste
  stringlist.clear();
  if (comm.getPlayerRoleList(stringlist)) {
    REMBB.robot_state[robot].list_roles = stringlist;
    playerrolelist_request=false;
  }

  // Spielertyp
  if (comm.getPlayerType(s)) {
    if (s!=REMBB.robot_state[robot].playertype)
      playerrolelist_request=true;
    if (REMBB.robot_state[robot].playertype==REMBB.robot_state[robot].desired_playertype)
      REMBB.robot_state[robot].desired_playertype=s;
    REMBB.robot_state[robot].playertype = s;
  }

  // Spielerrolle
  if (comm.getPlayerRole(s)) {
    if (REMBB.robot_state[robot].playerrole==REMBB.robot_state[robot].desired_playerrole)
      REMBB.robot_state[robot].desired_playerrole=s;
    REMBB.robot_state[robot].playerrole = s;
  }

  // Feldgeometrie
  if (comm.getFieldGeometry(REMBB.robot_state[robot].field_geometry)) {
    if (REMBB.robot_state[robot].field_geometry!=REMBB.team_state.field_geometry)
      cerr << "Achtung: Feldgeometrie von " << REMBB.robot_state[robot].name << " passt nicht!\n";
    field_geometry_request=false;
  }

  // Roboter-ID
  if (comm.getRobotID(an_uint))
    REMBB.robot_state[robot].local_id = an_uint;

  // Taktik
  if (comm.getTacticsBoard(tb))
    REMBB.robot_state[robot].tactics_board = tb;

  if (comm.getTacticsBoardForce(tb)) {
    std::map<std::string, std::string>::const_iterator it = tb.begin();
    while (it!=tb.end()) {
      REMBB.coach_state.tactics_board[it->first] = it->second;
      it++;
    }
  }

  // Spielstand
  if (comm.getScore (ownsc, oppsc, yell)) {
    REMBB.robot_state[robot].own_score = ownsc;
    REMBB.robot_state[robot].opponent_score = oppsc;
    REMBB.robot_state[robot].yellow_cards = yell;
  }

  // Objekte im Bild
  comm.getVisibleObjectList(REMBB.robot_state[robot].visible_objects);

  // GameState
  Tribots::GameState gs;
  if (comm.getGameState(gs)) {
    if (REMBB.robot_state[robot].refstate==REMBB.robot_state[robot].desired_refstate && !first_receive)
      REMBB.robot_state[robot].desired_refstate=gs.refstate;
    REMBB.robot_state[robot].refstate=gs.refstate;
    if (REMBB.robot_state[robot].in_game==REMBB.robot_state[robot].desired_in_game)
      REMBB.robot_state[robot].desired_in_game=gs.in_game;
    REMBB.robot_state[robot].in_game=gs.in_game;
  }

  // own_half
  if (comm.getOwnHalf(an_int)) {
    if (REMBB.robot_state[robot].own_half==REMBB.robot_state[robot].desired_own_half && !first_receive)
      REMBB.robot_state[robot].desired_own_half=an_int;
    REMBB.robot_state[robot].own_half=an_int;
  }

  // Ball-, Hindernis- und Roboterposition
  comm.getBallLocation(REMBB.robot_state[robot].ball_pos);
  comm.getRobotLocation(REMBB.robot_state[robot].robot_pos, REMBB.robot_state[robot].occ_grid);
  comm.getObstacleLocation(REMBB.robot_state[robot].obs_pos);

  // Roboterdaten
  comm.getRobotData(REMBB.robot_state[robot].robot_data);
  comm.getVcc (REMBB.robot_state[robot].robot_data.motor_vcc);

  // Messageboard
  REMBB.robot_state[robot].message_board.clear_incoming();
  vector<string> incoming_messages;
  comm.getMessageBoard (incoming_messages);
  REMBB.robot_state[robot].message_board.receive (incoming_messages);

  if (REMBB.robot_state[robot].comm_started && REMBB.robot_state[robot].show_message_board) {
    cout << "MessageBoard " << REMBB.robot_state[robot].name << " Eingang:\n";
    for (unsigned int i=0; i<incoming_messages.size(); i++)
      cout << incoming_messages[i] << '\n';
  }

  // Kommunikationsstatus
  REMBB.robot_state[robot].comm_okay = (num_received_packets>0);
  REMBB.robot_state[robot].comm_started = comm.started();
  if (REMBB.robot_state[robot].comm_okay) latest_receive.update();
  REMBB.robot_state[robot].comm_interrupted = (latest_receive.elapsed_msec()>5000);
  if (latest_receive.elapsed_msec()>10000) {
    // bei unterbrochener Verbindung koennte ein Programmabbruch vorliegen,
    // daher einige Werte zurucksetzen, um bei wiederaufgenommener
    // Kommunikation diese Werte erneut abzufragen
    REMBB.robot_state[robot].tactics_board.clear();
    REMBB.robot_state[robot].local_id = 0;
    field_geometry_request = true;
    playertypelist_request = true;
    playerrolelist_request = true;
    REMBB.robot_state[robot].own_score=0;
    REMBB.robot_state[robot].opponent_score=0;
    REMBB.robot_state[robot].yellow_cards=0;
  }

  if (!comm.started()) {
    REMBB.robot_state[robot].in_game=false;
    REMBB.robot_state[robot].desired_in_game = REMBB.robot_state[robot].in_game;
    REMBB.robot_state[robot].desired_own_half = REMBB.robot_state[robot].own_half;
    REMBB.robot_state[robot].desired_refstate = REMBB.robot_state[robot].refstate;
    REMBB.robot_state[robot].desired_playertype = REMBB.robot_state[robot].playertype;
    REMBB.robot_state[robot].desired_playerrole = REMBB.robot_state[robot].playerrole;
    REMBB.robot_state[robot].comm_statistics_send.packet_rate=0.0;
    REMBB.robot_state[robot].comm_statistics_send.packet_size=0.0;
    REMBB.robot_state[robot].comm_statistics_receive.packet_rate=0.0;
    REMBB.robot_state[robot].comm_statistics_receive.packet_size=0.0;
  } else {
    Time now;
    int dt = now.get_sec ();
    if (dt>10) dt=10;
    REMBB.robot_state[robot].comm_statistics_send = comm.send_load (dt);
    REMBB.robot_state[robot].comm_statistics_receive = comm.receive_load (dt);
  }
  first_receive &= (num_received_packets==0);
  if (REMBB.robot_state[robot].comm_interrupted) {
    first_receive=true;
  }

  if (REMBB.robot_state[robot].ip!=ping.getAddress())
    ping.setAddress(REMBB.robot_state[robot].ip);
  REMBB.robot_state[robot].pingtime = ping.getPingTime();
}

void RemoteRobot::send () throw () {
  // drive_to-Anfragen beruecksichtigen
  if (REMBB.robot_state[robot].drive_to_request) {
    REMBB.robot_state[robot].drive_to_request=false;
    REMBB.robot_state[robot].message_board.publish_stream () << "GotoPos: " << static_cast<int>(REMBB.robot_state[robot].drive_to_pos.x) << ' ' << static_cast<int>(REMBB.robot_state[robot].drive_to_pos.y) << ' ' << static_cast<int>(REMBB.robot_state[robot].drive_to_angle.get_deg()) << '\n';
  }

  // Aktivierungs-/Deaktivierungswunsch vom Joystick beruecksichtigen
  if (REMBB.robot_state[robot].joystick_request) {
    if (REMBB.joystick_state.joystick_okay) {
      if (REMBB.joystick_state.activation_request)
        REMBB.robot_state[robot].desired_in_game=true;
      if (REMBB.joystick_state.deactivation_request)
        REMBB.robot_state[robot].desired_in_game=false;
    }
  }

  // connect/disconnect
  if (REMBB.robot_state[robot].comm_started!=REMBB.robot_state[robot].desired_connect) {
    if (REMBB.robot_state[robot].desired_connect) {
      if (comm.started())
        comm.close();
      comm.init_as_client (REMBB.robot_state[robot].ip.c_str(), REMBB.robot_state[robot].port);
      field_geometry_request=true;
      playertypelist_request=true;
      playerrolelist_request=true;
      REMBB.robot_state[robot].tactics_board.clear();
      REMBB.robot_state[robot].local_id = 0;
      REMBB.robot_state[robot].comm_started=true;
      REMBB.robot_state[robot].own_score=0;
      REMBB.robot_state[robot].opponent_score=0;
      REMBB.robot_state[robot].yellow_cards=0;
      REMBB.robot_state[robot].desired_slhint_and_activate=false;
    } else {
      comm.close();
      REMBB.robot_state[robot].comm_started=false;
      REMBB.robot_state[robot].desired_in_game=REMBB.robot_state[robot].in_game=false;
      REMBB.robot_state[robot].desired_slhint_and_activate=false;
    }
    REMBB.robot_state[robot].desired_connect=REMBB.robot_state[robot].comm_started;
    first_receive=true;
  }

  // Joystick
  if (REMBB.robot_state[robot].joystick_request) {
    if (REMBB.joystick_state.joystick_okay) {
      if (REMBB.joystick_state.perspective<0)
        REMBB.robot_state[robot].message_board.publish_stream() << "JoyDrv: " << REMBB.joystick_state.vx << ' ' << REMBB.joystick_state.vy << ' ' << REMBB.joystick_state.vphi << ' ' << (REMBB.joystick_state.kick ? 1 : 0) << ' ' << REMBB.joystick_state.kick_length << '\n';
      else {
        Vec vtrans (REMBB.joystick_state.vx, REMBB.joystick_state.vy);
        vtrans*=Angle::deg_angle (REMBB.joystick_state.perspective);
        REMBB.robot_state[robot].message_board.publish_stream() << "JoyDrvGlob: " << vtrans.x << ' ' << vtrans.y << ' ' << REMBB.joystick_state.vphi << ' ' << (REMBB.joystick_state.kick ? 1 : 0) << ' ' << REMBB.joystick_state.kick_length << '\n';
      }
    }
  }

  // Nachrichten senden
  if (REMBB.robot_state[robot].comm_started && REMBB.robot_state[robot].show_message_board) {
    cout << "MessageBoard " << REMBB.robot_state[robot].name << " Ausgang:\n";
    vector<string> outgoing_messages = REMBB.robot_state[robot].message_board.get_outgoing();
    for (unsigned int i=0; i<outgoing_messages.size(); i++)
      cout << outgoing_messages[i] << '\n';
  }

  // Roboter lokalisieren & aktivieren
  if (REMBB.robot_state[robot].in_game)
    REMBB.robot_state[robot].desired_slhint_and_activate=false;
  if (REMBB.robot_state[robot].desired_slhint_and_activate) {
    double dp = (REMBB.robot_state[robot].robot_pos.pos-REMBB.robot_state[robot].slhint_pos).length();
    double da = (REMBB.robot_state[robot].robot_pos.heading-REMBB.robot_state[robot].slhint_angle).get_deg_180();
    if (dp<1500 && abs(da)<45) {
      REMBB.robot_state[robot].desired_in_game=true;
      REMBB.robot_state[robot].slhint_request=false;
      REMBB.robot_state[robot].desired_slhint_and_activate=false;
    } else {
      REMBB.robot_state[robot].slhint_request=true;
    }
  }

  // aktivieren/deaktivieren
  if (REMBB.robot_state[robot].in_game!=REMBB.robot_state[robot].desired_in_game) {
    comm.putInGame (REMBB.robot_state[robot].desired_in_game);
  }

  // MessageBoard/Ausgang
  comm.putMessageBoard (REMBB.robot_state[robot].message_board.get_outgoing());
  REMBB.robot_state[robot].message_board.clear_outgoing();

  // Spielertyp
  if (REMBB.robot_state[robot].playertype!=REMBB.robot_state[robot].desired_playertype) {
    comm.putPlayerType (REMBB.robot_state[robot].desired_playertype.c_str());
    playerrolelist_request=true;
  }

  // Spielerrolle
  if (REMBB.robot_state[robot].playerrole!=REMBB.robot_state[robot].desired_playerrole)
    comm.putPlayerRole (REMBB.robot_state[robot].desired_playerrole.c_str());

  // Refereestate
  if (REMBB.robot_state[robot].refstate!=REMBB.robot_state[robot].desired_refstate)
    comm.putRefereeState (REMBB.robot_state[robot].desired_refstate);

  // own_half
  if (REMBB.robot_state[robot].own_half!=REMBB.robot_state[robot].desired_own_half)
    comm.putOwnHalf (REMBB.robot_state[robot].desired_own_half);

  // SL-Hint
  if (REMBB.robot_state[robot].slhint_request)
    comm.putSLHint (REMBB.robot_state[robot].slhint_pos, REMBB.robot_state[robot].slhint_angle);
  REMBB.robot_state[robot].slhint_request=false;

  // SL-Mirror-Hint
  if (REMBB.robot_state[robot].sl_mirror_hint_request)
    comm.putSLMirrorHint (REMBB.robot_state[robot].sl_mirror_hint);
  REMBB.robot_state[robot].sl_mirror_hint_request=false;

  // Synchronisationssignale
  if (REMBB.team_state.send_synch_signal)
    comm.putSynchronisationSignal (REMBB.team_state.synch_signal);

  // Mitspielerpositionen
  if (REMBB.coach_state.teammate_mode) {
    std::vector<TeammateLocation> tlo;
    for (unsigned int i=0; i<REMBB.robot_state.size(); i++) {
      if (i!=robot && REMBB.robot_state[i].in_game) {
        TeammateLocation tl;
        RobotLocation& rl (tl);
        rl = REMBB.robot_state[i].robot_pos;
        tl.number = REMBB.robot_state[i].id;
        tl.occupancy_grid = REMBB.robot_state[i].occ_grid;
        tlo.push_back (tl);
      }
    }
    if (tlo.size()>0)
      comm.putTeammateLocation (tlo, REMBB.team_state.comm_rate);  // in der Kommunikation steckt ca. ein Takt Verzoegerung drin
  }

  // Roboter-ID
  if (REMBB.robot_state[robot].local_id != REMBB.robot_state[robot].id)
    comm.putRobotID (REMBB.robot_state[robot].id);

  // Taktik-Board
  if (REMBB.coach_state.tactics_mode) {
    if (REMBB.robot_state[robot].tactics_board != REMBB.coach_state.tactics_board)
      comm.putTacticsBoard (REMBB.coach_state.tactics_board);
  }

  // Spielstand
  if ((REMBB.team_state.own_half==REMBB.robot_state[robot].own_half &&
       (REMBB.team_state.own_score!=REMBB.robot_state[robot].own_score ||
       REMBB.team_state.opponent_score!=REMBB.robot_state[robot].opponent_score ||
       REMBB.robot_state[robot].yellow_cards!=REMBB.robot_state[robot].desired_yellow_cards))) {
    comm.putScore (REMBB.team_state.own_score, REMBB.team_state.opponent_score, REMBB.robot_state[robot].desired_yellow_cards);
  }
  if ((REMBB.team_state.own_half!=REMBB.robot_state[robot].own_half &&
       (REMBB.team_state.own_score!=REMBB.robot_state[robot].opponent_score ||
       REMBB.team_state.opponent_score!=REMBB.robot_state[robot].own_score ||
       REMBB.robot_state[robot].yellow_cards!=REMBB.robot_state[robot].desired_yellow_cards))) {
    comm.putScore (REMBB.team_state.opponent_score, REMBB.team_state.own_score, REMBB.robot_state[robot].desired_yellow_cards);
  }

  // sonstige Requests
  if (until_send_lines>0) until_send_lines--;
  bool reset_until_send_lines=false;
  comm.putStandardRequest();
  if (field_geometry_request)
    comm.putFieldGeometryRequest();
  if (playertypelist_request)
    comm.putPlayerTypeListRequest();
  if (playerrolelist_request)
    comm.putPlayerRoleListRequest();
  if (REMBB.robot_state[robot].debug_image_request) {
    REMBB.robot_state[robot].debug_image_request=false;
    comm.putDebugImageRequest();
  }
  if (REMBB.robot_state[robot].obstacle_request && (until_send_lines==0)) {
    comm.putObstacleLocationRequest();
    reset_until_send_lines=true;
  }
  if (REMBB.robot_state[robot].visible_object_request && (until_send_lines==0)) {
    comm.putVisibleObjectListRequest();
    reset_until_send_lines=true;
  }
  if (REMBB.robot_state[robot].robot_data_request)
    comm.putRobotDataRequest();
  comm.putMessageBoardRequest();
  if (reset_until_send_lines)
    until_send_lines = REMBB.team_state.send_lines_rate;
  if (REMBB.robot_state[robot].exit_request) {
    comm.putExitRequest ();
    REMBB.robot_state[robot].exit_request=false;
  }

  comm.send();
}
