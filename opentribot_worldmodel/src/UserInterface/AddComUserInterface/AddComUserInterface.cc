#include "stdlib.h"
#include "AddComUserInterface.h"
#include "../CliUserInterface/CliUserInterface.h"
#include "../StreamUserInterface/StreamUserInterface.h"
#include "../../WorldModel/WorldModel.h"
#include "../../WorldModel/Prediction/update_robot_location.h"


#include "../../Structures/Journal.h"

#include "../../Fundamental/RemoteTune.h"

using namespace Tribots;
using namespace std;


Tribots::AddComUserInterface::AddComUserInterface (const ConfigReader& conf, WorldModel& _wm) throw (Tribots::TribotsException, std::bad_alloc) :  the_world_model (_wm), max_message_board_fail(1500), tournament_mode (false), synch_out (NULL), requestedImage (false)
{
  string confline;
  if (conf.get("user_interface_type", confline)<=0) {
    JERROR("no config line \"user_interface_type\" found");
    throw Tribots::InvalidConfigurationException ("user_interface_type");
  }
  if (confline=="CliUserInterface") {
    the_local_user_interface = new CliUserInterface(conf,_wm);
  }
  else if (confline=="CliUserInterface") {
    the_local_user_interface = new CliUserInterface(conf, _wm);
  }
  else if  (confline=="StreamUserInterface") {
    the_local_user_interface = new StreamUserInterface(conf, _wm);
  } else {
    JERROR("No UserInterfaceType of this type found");
    throw Tribots::InvalidConfigurationException ("user_interface_type");
  }
  conf.get("tournament_mode", tournament_mode);
  unsigned int port=6012;
  if (conf.get ("communication_port", port)<=0) {
    JERROR("No communication port given");
    throw Tribots::InvalidConfigurationException ("communication_port");
  }
  bool stop_if_no_socket = true;
  if (conf.get ("communication_stop_if_no_socket", stop_if_no_socket)<=0) {
    JERROR("No communication possible since socket not available");
    throw Tribots::InvalidConfigurationException ("communication_stop_if_no_socket");
  }
  debug_image_filename_base = "debug_image";
  conf.get ("debug_image_filename_base", debug_image_filename_base);

  while (!comm.init_as_server (port)) {
    port++;
    if (stop_if_no_socket)
      throw Tribots::HardwareException ("requested socket not available");
  }
  std::cout << "UDP server on port " << port << '\n';

  bool b;
  if (conf.get("add_write_world_model", b)>0 && b) {
    string fbasename="";
    stringstream filename;
    bool rotate_log = true;
    conf.get("rotate_log",rotate_log);

    if (conf.get ("write_world_model_info", fbasename)==0)
      fbasename = "wminfo";
    filename << fbasename;

    if (rotate_log) {
      struct timeval tv1;
      Time ttstart;
      ttstart.set_usec(0);
      ttstart.get (tv1);

      struct tm* mytm;
      mytm = localtime(&tv1.tv_sec);
      stringstream timestring;
      timestring << (1900+mytm->tm_year) << "-";
      if (mytm->tm_mon < 9) timestring << "0";
      timestring << mytm->tm_mon+1 << "-";
      if (mytm->tm_mday < 10) timestring << "0";
      timestring << mytm->tm_mday << "-";
      if (mytm->tm_hour < 10) timestring << "0";
      timestring << mytm->tm_hour;
      if (mytm->tm_min < 10) timestring << "0";
      timestring << mytm->tm_min;
      if (mytm->tm_sec < 10) timestring << "0";
      timestring << mytm->tm_sec;

      filename << "_" << timestring.str() ;
    }
    filename << ".syc";

    synch_out = new ofstream (filename.str().c_str());
    if (!synch_out) {
      JERROR ("Synchronisations-Datei nicht zugreifbar");
      synch_out=NULL;
    }

    if (rotate_log) {
      // delete existing link or file journal.out
      stringstream deletecmd;
      deletecmd << "rm -f " << fbasename << ".syc" << " >/dev/null 2>&1";
      system(deletecmd.str().c_str());
      // update link to journal.out
      stringstream relinkcmd;
      relinkcmd << "ln -s " << filename.str() << " " << fbasename << ".syc";  
      system(relinkcmd.str().c_str());
    }
  }
  
  // force_tactics einlesen
  Tribots::read_tactics (force_tactics, conf);
  vector<TacticsAttribute>::iterator it = force_tactics.begin();
  while (it!=force_tactics.end()) {
    if (it->force_default) {
      it++;
    } else {
      it=force_tactics.erase (it);
    }
  }
}


Tribots::AddComUserInterface::~AddComUserInterface () throw () 
{
  delete the_local_user_interface;
  if (synch_out) {
    (*synch_out) << flush;
    delete synch_out;
  }
}


bool Tribots::AddComUserInterface::process_messages () throw () 
{

  int own_half = MWM.get_own_half ();

  unsigned int num_packets_received = comm.receive ();
  if (num_packets_received>0)
    timestamp_latest_comm.update();
  if (tournament_mode && timestamp_latest_comm.elapsed_sec()>5) {
    // im Turniermodus nach 5 Sekunden ohne Verbindung automatisch starten.
    timestamp_latest_comm.update();
    if (MWM.get_game_state().in_game)
      MWM.update_refbox (SIGstart);
  }

  // eingehende Informationen verarbeiten
  bool b;
  int i;
  RefereeState rs;
  string s;
  Vec v;
  Angle a;
  unsigned short int usi;
  unsigned int ui;
  unsigned int own_score, opponent_score, yellow_cards;
  std::vector<TeammateLocation> tlo;
  vector<string> incoming_messages;
  TacticsBoard tb;
  bool exitRequest = comm.getExitRequest();
  if (comm.getSynchronisationSignal(usi) && synch_out) {
    Time now;
    (*synch_out) << now << '\t' << usi << '\n' << flush;
  }
  if (comm.getInGame (b))
    MWM.startstop (b);
  if (comm.getRefereeState (rs)) {
    switch (rs) {  // versuche, dem gewuenschten Refereestate moeglichst nahe zu kommen
    case stopRobot: MWM.update_refbox (SIGstop); break;
    case freePlay: MWM.update_refbox (SIGstart); break;
    case preOwnKickOff: MWM.update_refbox (SIGownKickOff); break;
    case preOpponentKickOff: MWM.update_refbox (SIGopponentKickOff); break;
    case preOwnFreeKick: MWM.update_refbox (SIGownFreeKick); break;
    case preOpponentFreeKick: MWM.update_refbox (SIGopponentFreeKick); break;
    case preOwnGoalKick: MWM.update_refbox (SIGownGoalKick); break;
    case preOpponentGoalKick: MWM.update_refbox (SIGopponentGoalKick); break;
    case preOwnCornerKick: MWM.update_refbox (SIGownCornerKick); break;
    case preOpponentCornerKick: MWM.update_refbox (SIGopponentCornerKick); break;
    case preOwnThrowIn: MWM.update_refbox (SIGownThrowIn); break;
    case preOpponentThrowIn: MWM.update_refbox (SIGopponentThrowIn); break;
    case preOwnPenalty: MWM.update_refbox (SIGownPenalty); break;
    case preOpponentPenalty: MWM.update_refbox (SIGopponentPenalty); break;
    case postOpponentKickOff: case postOpponentGoalKick: 
    case postOpponentCornerKick: case postOpponentThrowIn:
    case postOpponentPenalty: case postOpponentFreeKick: 
    case ownPenalty: case opponentPenalty: MWM.update_refbox (SIGready); break;
    case preDroppedBall: MWM.update_refbox (SIGdroppedBall); break;
    case errorState: MWM.update_refbox (SIGstop); break;
    case testState1: MWM.update_refbox (SIGtest1); break;
    case testState2: MWM.update_refbox (SIGtest2); break;
    case testState3: MWM.update_refbox (SIGtest3); break;
    case testState4: MWM.update_refbox (SIGtest4); break;
    case testState5: MWM.update_refbox (SIGtest5); break;
    case testState6: MWM.update_refbox (SIGtest6); break;
    case testState7: MWM.update_refbox (SIGtest7); break;
    case testState8: MWM.update_refbox (SIGtest8); break;
    }
  }
  if (comm.getOwnHalf (i))
    MWM.set_own_half (i);
  if (comm.getSLHint (v, a)) {
    if (MWM.get_own_half()<0) {
      v*=-1;
      a+=Angle::half;
    }
    MWM.reset (v,a);
  }
  if (comm.getSLMirrorHint (v)) {
    MWM.slMirrorHint (v);
  }
  // Mitspielerpositionen
  if (comm.getTeammateLocation (tlo)) {
    for (unsigned int i=0; i<tlo.size(); i++) {
      RobotLocation& rlo (tlo[i]);
      rlo = flip_sides (rlo, own_half);  // ggf. Seite wechseln
    }
    MWM.set_teammates (tlo);
  }

  // Messageboard
  bool smb = comm.getMessageBoard (incoming_messages);
  if (smb) {
    latest_message_board.update();
    MWM.get_message_board().clear_incoming();
    MWM.get_message_board().receive (incoming_messages);
    // nach Informationen ueber Spielbeginn und -ende suchen
    std::string line;
    if ((line = MWM.get_message_board().scan_for_prefix ("start first half!")).length()>0) JMESSAGETS (line.c_str());
    if ((line = MWM.get_message_board().scan_for_prefix ("start second half!")).length()>0) JMESSAGETS (line.c_str());
    if ((line = MWM.get_message_board().scan_for_prefix ("end first half!")).length()>0) JMESSAGETS (line.c_str());
    if ((line = MWM.get_message_board().scan_for_prefix ("end second half!")).length()>0) JMESSAGETS (line.c_str());
    if ((line = MWM.get_message_board().scan_for_prefix ("own goal scored!")).length()>0) JMESSAGETS (line.c_str());
    if ((line = MWM.get_message_board().scan_for_prefix ("opponent goal scored!")).length()>0) JMESSAGETS (line.c_str());
  } else {
    if (latest_message_board.elapsed_msec()>max_message_board_fail) {
      MWM.get_message_board().clear_incoming();  // zu lange keine Nachrichten mehr bekommen => Nachrichten veraltet => Nachrichten entfernen
      latest_message_board.update();
    }
  }
  // Roboter-ID
  if (comm.getRobotID(ui)) {
    MWM.set_robot_id (ui);
    comm.putRobotID(MWM.get_robot_id()); // zur Kontrolle den gesetzten ID wieder zurueckschicken
  }
  // Spielstand
  if (comm.getScore(own_score, opponent_score, yellow_cards)) {
    MWM.set_score (own_score, opponent_score, yellow_cards);
    const GameState& gs = MWM.get_game_state();
    LOUT << "got new score. New score is: " << gs.own_score << ':' << gs.opponent_score << ", yellow=" << yellow_cards << endl;
    comm.putScore (gs.own_score, gs.opponent_score, gs.yellow_cards); // zur Kontrolle zurueckschicken
  }
  // Debug-Image

  // Requests bearbeiten
  Time now;
  if (comm.getObstacleLocationRequest())
    comm.putObstacleLocation (flip_sides (MWM.get_obstacle_location(now), own_half));
  if (comm.getVisibleObjectListRequest())
    comm.putVisibleObjectList (MWM.get_visible_objects());
  if (comm.getRobotDataRequest())
    comm.putRobotData (MWM.get_robot_data(now));
  if (comm.getFieldGeometryRequest())
    comm.putFieldGeometry (MWM.get_field_geometry());
  if (comm.getMessageBoardRequest ()) {
    comm.putMessageBoard (MWM.get_message_board().get_outgoing());
    MWM.get_message_board().clear_outgoing();
    latest_message_board_request.update();
  } else {
    if (latest_message_board_request.elapsed_msec()>max_message_board_fail) {
      MWM.get_message_board().clear_outgoing();  // zu lange keinen Request mehr bekommen => Nachrichten veraltet => Nachrichten entfernen
      latest_message_board_request.update();
    }
  }
  if (comm.getStandardRequest()) {
    comm.putRobotLocation (flip_sides(MWM.get_robot_location (now), own_half), TeammateOccupancyGrid (MWM.get_robot_location (now).pos, MWM.get_obstacle_location (now)));
    comm.putBallLocation (flip_sides(MWM.get_ball_location (now), own_half));
    comm.putGameState (MWM.get_game_state ());
    comm.putOwnHalf (MWM.get_own_half());
    comm.putVcc (MWM.get_robot_data(now).motor_vcc);
  }

  if (force_tactics.size()!= 0 && MWM.get_game_state().in_game) {
    TacticsBoard tbf;
    make_default_tactics_board (tbf, force_tactics);
    comm.putTacticsBoardForce (tbf);
  }

  comm.send();


  return (!exitRequest) && the_local_user_interface->process_messages();
}

