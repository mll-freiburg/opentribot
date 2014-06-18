
#include "TeamcontrolUDPCommunication.h"
#include "../../../Communication/encoding.h"
#include <cstring>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

// Tags sind wiefolgt belegt (gerade Zahl=Wert, ungerade Zahl=Request).
// Prioritaeten fuer Werte in Klammern; Requests haben stets Prioritaet 0
//
// 2: Hello
// 4: CoachState
// 6: TeamState
// 8: HelpState
// 10: Joystick
// 12: RemoteRobot
// 14: MessageID
// 254: Bye

//
//
//  TODO: manchmal werden Spielertypliste und Spielerrollenliste vermischt
//


namespace {
  // um diese beiden Variablen nicht in jeder einzelnen Methode erneut deklarieren zu muessen, hier:
  const char* retbuf;
  unsigned int retlen;
  const char magic_bytes [] = { -6, 18 };
}


TeamcontrolUDPCommunication::TeamcontrolUDPCommunication () throw (std::bad_alloc) : NonspecificTaggedUDPCommunication (magic_bytes) {;}

TeamcontrolUDPCommunication::~TeamcontrolUDPCommunication () throw () {
  close(); 
}

void TeamcontrolUDPCommunication::close () throw () {
  clear_send_buffer();
  putBye ();
  send ();
  NonspecificTaggedUDPCommunication::close ();
}



bool TeamcontrolUDPCommunication::putHello () throw (std::bad_alloc) {
  return socket.put (2,NULL,0);
}
bool TeamcontrolUDPCommunication::getHello () throw () {
  return socket.get (2,retbuf,retlen);
}

bool TeamcontrolUDPCommunication::putBye () throw (std::bad_alloc) {
  return socket.put (254,NULL,0);
}
bool TeamcontrolUDPCommunication::getBye () throw () {
  return socket.get (254,retbuf,retlen);
}

bool TeamcontrolUDPCommunication::putMessageID (unsigned short int src) throw (std::bad_alloc) {
  char buffer [2];
  write_unsigned_short (buffer, src);
  return socket.put (14, buffer, 2);
}
bool TeamcontrolUDPCommunication::getMessageID (unsigned short int& dest) throw () {
  if (socket.get (14,retbuf,retlen) && retlen>=2) {
    dest=read_unsigned_short (retbuf);
    return true;
  }
  return false;
}


bool TeamcontrolUDPCommunication::putCoachState (const CoachState& src) throw (std::bad_alloc) {
  unsigned short int len_policies = bufferSizeStringList (src.list_policies);
  unsigned short int len_tacticsboard = bufferSizeTacticsBoard(src.tactics_board);
  unsigned short int len_policy = src.policy_name.length();
  unsigned short int len_extram = src.extra_message.length();
  unsigned short int buflen=len_policies+len_policy+len_tacticsboard+len_extram+12;
  char buffer [buflen];
  char* pt = buffer;
  pt = NonspecificTaggedUDPCommunication::write_string_nl (pt, src.policy_name);
  *(pt++)=src.ball_position_mode;
  *(pt++)=src.ball_posession_mode;
  *(pt++)=src.broadcast_mode;
  *(pt++)=src.teammate_mode;
  *(pt++)=src.tactics_mode;
  *(pt++)=src.sl_mirror_hint_mode;
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, len_policies);
  pt+=2;
  encodeStringList (pt, src.list_policies);
  pt+=len_policies;
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, len_tacticsboard);
  pt+=2;
  encodeTacticsBoard (pt, src.tactics_board);
  pt+=len_tacticsboard;
  pt = NonspecificTaggedUDPCommunication::write_string_nl (pt, src.extra_message);
  return socket.put (4,buffer,buflen);
}
bool TeamcontrolUDPCommunication::getCoachState (CoachState& dest, unsigned int n) throw (std::bad_alloc) {
  if (socket.get (4,retbuf,retlen,n) && retlen>=12) {
    const char* pt = retbuf;
    pt = NonspecificTaggedUDPCommunication::read_string_nl (pt, dest.policy_name, retlen);
    dest.ball_position_mode=*(pt++);
    dest.ball_posession_mode=*(pt++);
    dest.broadcast_mode=*(pt++);
    dest.teammate_mode=*(pt++);
    dest.tactics_mode=*(pt++);
    dest.sl_mirror_hint_mode=*(pt++);
    unsigned short int len_policies = NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeStringList (pt, len_policies, dest.list_policies);
    pt+=len_policies;
    unsigned short int len_tacticsboard = NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeTacticsBoard (pt, len_tacticsboard, dest.tactics_board);
    pt+=len_tacticsboard;
    pt = NonspecificTaggedUDPCommunication::read_string_nl (pt, dest.extra_message, retlen-(pt-retbuf));
    return true;
  }
  return false;
}

unsigned int TeamcontrolUDPCommunication::numCoachState () throw () {
  return socket.num_messages (4);
}

bool TeamcontrolUDPCommunication::putTeamState (const TeamState& src) throw (std::bad_alloc) {
  unsigned short int buflen=10;
  char buffer [buflen];
  char* pt = buffer;
  *(pt++)=src.refstate;
  *(pt++)=src.own_half;
  *(pt++)=src.team_color;
  *(pt++)=src.send_lines_rate;
  *(pt++)=src.own_score;
  *(pt++)=src.opponent_score;
  *(pt++)=src.refbox_signal;
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(src.comm_rate));
  pt+=2;
  *(pt++)=src.localization_side==0 ? 0 : (src.localization_side>0 ? 1 : 2);
  return socket.put (6,buffer,buflen);
}
bool TeamcontrolUDPCommunication::getTeamState (TeamState& dest) throw (std::bad_alloc) {
  if (socket.get (6,retbuf,retlen) && retlen>=10) {
    const char* pt = retbuf;
    dest.refstate = Tribots::RefereeState (*(pt++));
    dest.own_half=*(pt++);
    dest.team_color=*(pt++);
    dest.send_lines_rate=*(pt++);
    dest.own_score=*(pt++);
    dest.opponent_score=*(pt++);
    dest.refbox_signal=Tribots::RefboxSignal(*(pt++));
    dest.comm_rate = NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    dest.localization_side = ((*pt)==0 ? 0 : ((*pt)==1 ? +1 : -1));
    pt++;
    return true;
  }
  return false;
}

bool TeamcontrolUDPCommunication::putHelpState (const HelpState& src) throw (std::bad_alloc) {
  unsigned int buflen=2;
  std::list<DisplayRobotArrow>::const_iterator it1;
  for (it1=src.robotarrows.begin(); it1!=src.robotarrows.end(); it1++)
    buflen+=6+it1->color.length()+it1->text.length();
  std::list<DisplayRobotText>::const_iterator it2;
  for (it2=src.robottext.begin(); it2!=src.robottext.end(); it2++)
    buflen+=5+it2->color.length()+it2->text.length();
  
  char buffer [buflen];
  char* pt = buffer;
  *(pt++) = static_cast<char>(static_cast<unsigned char>(src.robotarrows.size()));
  for (it1=src.robotarrows.begin(); it1!=src.robotarrows.end(); it1++) {
    *(pt++)=static_cast<unsigned char>(it1->source_id);
    *(pt++)=static_cast<unsigned char>(it1->target_id);
    pt=NonspecificTaggedUDPCommunication::write_string_nl (pt, it1->color);
    pt=NonspecificTaggedUDPCommunication::write_string_nl (pt, it1->text);
    Time now;
    NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(it1->deadline.diff_msec(now)));
    pt+=2;
  }
  *(pt++) = static_cast<char>(static_cast<unsigned char>(src.robottext.size()));
  for (it2=src.robottext.begin(); it2!=src.robottext.end(); it2++) {
    *(pt++)=static_cast<unsigned char>(it2->robot_id);
    pt=NonspecificTaggedUDPCommunication::write_string_nl (pt, it2->color);
    pt=NonspecificTaggedUDPCommunication::write_string_nl (pt, it2->text);
    Time now;
    NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(it2->deadline.diff_msec(now)));
    pt+=2;
  }
  return socket.put (8,buffer,buflen);
}
bool TeamcontrolUDPCommunication::getHelpState (HelpState& dest) throw (std::bad_alloc) {
  if (socket.get (8,retbuf,retlen) && retlen>=2) {
    dest.robottext.clear();
    dest.robotarrows.clear();
    const char* pt = retbuf;
    unsigned int numarrows = static_cast<unsigned int>(static_cast<unsigned char>(*(pt++)));
    for (unsigned int i=0; i<numarrows; i++) {
      DisplayRobotArrow newarrow;
      newarrow.source_id = static_cast<unsigned char>(*(pt++));
      newarrow.target_id = static_cast<unsigned char>(*(pt++));
      pt=NonspecificTaggedUDPCommunication::read_string_nl (pt, newarrow.color, retbuf-(pt-retlen));
      pt=NonspecificTaggedUDPCommunication::read_string_nl (pt, newarrow.text, retbuf-(pt-retlen));
      newarrow.deadline.add_msec (NonspecificTaggedUDPCommunication::read_unsigned_short (pt));
      pt+=2;
      dest.robotarrows.push_back (newarrow);
    }
    unsigned int numtext = static_cast<unsigned int>(static_cast<unsigned char>(*(pt++)));
    for (unsigned int i=0; i<numtext; i++) {
      DisplayRobotText newtext;
      newtext.robot_id = static_cast<unsigned char>(*(pt++));
      pt=NonspecificTaggedUDPCommunication::read_string_nl (pt, newtext.color, retbuf-(pt-retlen));
      pt=NonspecificTaggedUDPCommunication::read_string_nl (pt, newtext.text, retbuf-(pt-retlen));
      newtext.deadline.add_msec (NonspecificTaggedUDPCommunication::read_unsigned_short (pt));
      pt+=2;
      dest.robottext.push_back (newtext);
    }
    return true;
  }
  return false;
}

bool TeamcontrolUDPCommunication::putJoystickState (const JoystickState&) throw (std::bad_alloc) {
  // TODO: ergaenzen, wenn Joystick beim Slave-Teamcontrol betrieben werden soll
  return socket.put (10,NULL,0);
}
bool TeamcontrolUDPCommunication::getJoystickState (JoystickState&) throw (std::bad_alloc) {
  // TODO: ergaenzen, wenn Joystick beim Slave-Teamcontrol betrieben werden soll
  return socket.get (10,retbuf,retlen);
}

bool TeamcontrolUDPCommunication::putRemoteRobotState (const RemoteRobotState& src) throw (std::bad_alloc) {
  vector<string> icm = src.message_board.get_incoming();
  vector<string> ogm = src.message_board.get_outgoing();
  unsigned int buflen = src.name.length()+src.ip.length()+bufferSizeRobotLocation (src.robot_pos, src.occ_grid)+bufferSizeBallLocation (src.ball_pos)+bufferSizeObstacleLocation(src.obs_pos)+bufferSizeRobotData(src.robot_data)+bufferSizeVisibleObjectList(src.visible_objects)+src.playertype.length()+src.playerrole.length()+bufferSizeStringList(src.list_players)+bufferSizeStringList(src.list_roles)+bufferSizeStringList(icm)+bufferSizeStringList(ogm)+bufferSizeTacticsBoard(src.tactics_board)+src.desired_playertype.length()+src.desired_playerrole.length()+bufferSizeSLHint(src.slhint_pos, src.slhint_angle)+bufferSizeSLHint(src.drive_to_pos, src.drive_to_angle)+62;
  char buffer [buflen];
  char* pt=buffer;
  pt = NonspecificTaggedUDPCommunication::write_string_nl (pt, src.name);
  pt = NonspecificTaggedUDPCommunication::write_string_nl (pt, src.ip);
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(src.port));
  pt+=2;
  *(pt++)=static_cast<unsigned char>(src.id);
  *(pt++)=static_cast<unsigned char>(src.local_id);
  *(pt++)=src.comm_started;
  *(pt++)=src.comm_okay;
  *(pt++)=src.comm_interrupted;
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, 10*src.comm_statistics_send.packet_rate);
  pt+=2;
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, 10*src.comm_statistics_send.packet_size);
  pt+=2;
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, 10*src.comm_statistics_receive.packet_rate);
  pt+=2;
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, 10*src.comm_statistics_receive.packet_size);
  pt+=2;

  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeRobotLocation(src.robot_pos, src.occ_grid)));
  pt+=2;
  pt+=encodeRobotLocation (pt, src.robot_pos, src.occ_grid);
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeBallLocation(src.ball_pos)));
  pt+=2;
  pt+=encodeBallLocation (pt, src.ball_pos);
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeObstacleLocation(src.obs_pos)));
  pt+=2;
  pt+=encodeObstacleLocation (pt, src.obs_pos);
  *(pt++)=src.refstate;
  *(pt++)=src.in_game;
  *(pt++)=src.own_half;
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeRobotData(src.robot_data)));
  pt+=2;
  pt+=encodeRobotData (pt, src.robot_data);
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeVisibleObjectList(src.visible_objects)));
  pt+=2;
  pt+=encodeVisibleObjectList (pt, src.visible_objects);
  
  *(pt++)=src.own_score;
  *(pt++)=src.opponent_score;
  *(pt++)=src.yellow_cards;
  *(pt++)=src.desired_yellow_cards;

  pt = NonspecificTaggedUDPCommunication::write_string_nl (pt, src.playertype);
  pt = NonspecificTaggedUDPCommunication::write_string_nl (pt, src.playerrole);
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeStringList(src.list_players)));
  pt+=2;
  pt+=encodeStringList (pt, src.list_players);
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeStringList(src.list_roles)));
  pt+=2;
  pt+=encodeStringList (pt, src.list_roles);
  
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeStringList(icm)));
  pt+=2;
  pt+=encodeStringList (pt, icm);
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeStringList(ogm)));
  pt+=2;
  pt+=encodeStringList (pt, ogm);
  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeTacticsBoard(src.tactics_board)));
  pt+=2;
  pt+=encodeTacticsBoard (pt, src.tactics_board);

  *(pt++)=src.desired_connect;
  *(pt++)=src.desired_in_game;
  *(pt++)=src.desired_own_half;
  *(pt++)=src.desired_refstate;
  pt = NonspecificTaggedUDPCommunication::write_string_nl (pt, src.desired_playertype);
  pt = NonspecificTaggedUDPCommunication::write_string_nl (pt, src.desired_playerrole);
  
  *(pt++)=src.robot_data_request;
  *(pt++)=src.obstacle_request;
  *(pt++)=src.visible_object_request;
  *(pt++)=src.joystick_request;
  *(pt++)=src.slhint_request;
  *(pt++)=src.debug_image_request;
  *(pt++)=src.sl_mirror_hint_request;
  *(pt++)=src.drive_to_request;

  NonspecificTaggedUDPCommunication::write_unsigned_short (pt, static_cast<unsigned short int>(bufferSizeSLHint(src.slhint_pos, src.slhint_angle)));
  pt+=2;
  pt+=encodeSLHint (pt, src.slhint_pos, src.slhint_angle);
  pt+=encodeSLHint (pt, src.drive_to_pos, src.drive_to_angle);
  // sl_mirror_hint wird nicht uebertragen
  // show_message_board wird nicht uebertragen
  // field_geometry wird nicht uebertragen

  return socket.put (12,buffer,(pt-buffer));
}
bool TeamcontrolUDPCommunication::getRemoteRobotState (RemoteRobotState& dest, unsigned int n) throw (std::bad_alloc) {
  if (socket.get (12,retbuf,retlen,n) && retlen>=60) {
    vector<string> icm, ogm;
    const char* pt = retbuf;
    unsigned short int usi;
    pt = NonspecificTaggedUDPCommunication::read_string_nl (pt, dest.name, retlen);
    pt = NonspecificTaggedUDPCommunication::read_string_nl (pt, dest.ip, retlen-(pt-retbuf));
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    dest.port=usi;
    pt+=2;
    dest.id = static_cast<unsigned char>(*(pt++));
    dest.local_id = static_cast<unsigned char>(*(pt++));
    dest.comm_started = *(pt++);
    dest.comm_okay = *(pt++);
    dest.comm_interrupted = *(pt++);
    dest.comm_statistics_send.packet_rate=0.1*static_cast<double>(NonspecificTaggedUDPCommunication::read_unsigned_short (pt));
    pt+=2;
    dest.comm_statistics_send.packet_size=0.1*static_cast<double>(NonspecificTaggedUDPCommunication::read_unsigned_short (pt));
    pt+=2;
    dest.comm_statistics_receive.packet_rate=0.1*static_cast<double>(NonspecificTaggedUDPCommunication::read_unsigned_short (pt));
    pt+=2;
    dest.comm_statistics_receive.packet_size=0.1*static_cast<double>(NonspecificTaggedUDPCommunication::read_unsigned_short (pt));
    pt+=2;

    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;    
    decodeRobotLocation (pt, usi, dest.robot_pos, dest.occ_grid);
    pt+=usi;
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;    
    decodeBallLocation (pt, usi, dest.ball_pos);
    pt+=usi;
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeObstacleLocation (pt, usi, dest.obs_pos);
    pt+=usi;
    dest.refstate = Tribots::RefereeState (*(pt++));
    dest.in_game=*(pt++);
    dest.own_half=*(pt++);
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeRobotData (pt, usi, dest.robot_data);
    pt+=usi;
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeVisibleObjectList (pt, usi, dest.visible_objects);
    pt+=usi;
  
    dest.own_score = *(pt++);
    dest.opponent_score = *(pt++);
    dest.yellow_cards = *(pt++);
    dest.desired_yellow_cards = *(pt++);

    pt = NonspecificTaggedUDPCommunication::read_string_nl (pt, dest.playertype, retlen-(pt-retbuf));
    pt = NonspecificTaggedUDPCommunication::read_string_nl (pt, dest.playerrole, retlen-(pt-retbuf));
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeStringList (pt, usi, dest.list_players);
    pt+=usi;
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeStringList (pt, usi, dest.list_roles);
    pt+=usi;
  
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeStringList (pt, usi, icm);
    pt+=usi;
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeStringList (pt, usi, ogm);
    pt+=usi;
    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeTacticsBoard (pt, usi, dest.tactics_board);
    pt+=usi;
    dest.message_board.clear_incoming();
    dest.message_board.clear_outgoing();
    dest.message_board.receive (icm);
    dest.message_board.publish (ogm);
    
    dest.desired_connect=*(pt++);
    dest.desired_in_game=*(pt++);
    dest.desired_own_half=*(pt++);
    dest.desired_refstate=RefereeState (*(pt++));
    pt = NonspecificTaggedUDPCommunication::read_string_nl (pt, dest.desired_playertype, retlen-(pt-retbuf));
    pt = NonspecificTaggedUDPCommunication::read_string_nl (pt, dest.desired_playerrole, retlen-(pt-retbuf));
  
    dest.robot_data_request=*(pt++);
    dest.obstacle_request=*(pt++);
    dest.visible_object_request=*(pt++);
    dest.joystick_request=*(pt++);
    dest.slhint_request=*(pt++);
    dest.debug_image_request=*(pt++);
    dest.sl_mirror_hint_request=*(pt++);
    dest.drive_to_request=*(pt++);

    usi=NonspecificTaggedUDPCommunication::read_unsigned_short (pt);
    pt+=2;
    decodeSLHint (pt, usi, dest.slhint_pos, dest.slhint_angle);
    pt+=usi;
    decodeSLHint (pt, usi, dest.drive_to_pos, dest.drive_to_angle);
    pt+=usi;
    
    return true;
  }
  return false;
}
unsigned int TeamcontrolUDPCommunication::numRemoteRobotState () throw () {
  return socket.num_messages (12);
}

bool TeamcontrolUDPCommunication::putRemoteBlackboardState (const BlackboardState& src) throw (std::bad_alloc) {
  bool success=true;
  success &= putCoachState (src.coach_state);
  success &= putJoystickState (src.joystick_state);
  success &= putTeamState (src.team_state);
  success &= putHelpState (src.help_state);
  for (unsigned int i=0; i<src.robot_state.size(); i++)
    success &= putRemoteRobotState (src.robot_state[i]);
  return success;
}
bool TeamcontrolUDPCommunication::getRemoteBlackboardState (BlackboardState& dest) throw (std::bad_alloc) {
  bool success=true;
  unsigned int nc = numCoachState ();
  success &= getCoachState (dest.coach_state, nc-1);
  CoachState cs;
  for (unsigned int i=0; i+1<nc; i++) {
    success &= getCoachState(cs, i);
    dest.coach_state.extra_message+=cs.extra_message;
  }
  success &= getJoystickState (dest.joystick_state);
  success &= getTeamState (dest.team_state);
  success &= getHelpState (dest.help_state);
  unsigned int n=numRemoteRobotState ();
  dest.robot_state.clear();
  for (unsigned int i=0; i<n; i++) {
    RemoteRobotState rrs;
    success &= getRemoteRobotState (rrs, i);
    if (success)
     dest.robot_state.push_back (rrs);
  }
  return success;
}
