/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you want to add, delete, or rename functions or slots, use
** Qt Designer to update this file, preserving your code.
**
** You should not define a constructor or destructor in this file.
** Instead, write your code in functions called init() and destroy().
** These will automatically be called by the form's constructor and
** destructor.
*****************************************************************************/

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <qapplication.h>
#include <qstatusbar.h>
#include "../../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace TribotsTools;
using namespace std;

enum { NoBlackScreen=0, BlackScreenTribots, BlackScreenNeutral };

struct FileDescription {
  FileDescription () {;}
  FileDescription (const FileDescription& f) : name (f.name), modified (f.modified) {;}
  QString name;
  QDateTime modified;
  bool operator== (const FileDescription& f) const { return name==f.name && modified==f.modified; }
};

namespace {

  std::vector<FileDescription> filesDebugImage;

  void readFiles(std::vector<FileDescription>& dest) {
    dest.clear();
    QDir dir (".");
    const QFileInfoList* files = dir.entryInfoList("debug_image_*.ppm", QDir::Files, QDir::Time | QDir::Reversed);
    for (QFileInfoList::ConstIterator it=files->begin(); it!=files->end(); it++) {
      FileDescription d;
      d.name=(*it)->fileName();
      d.modified=(*it)->lastModified();
      dest.push_back (d);
    }
  }

}

void ControlGUI::init()
{
  robotcontrol_child_pid=-1;
  extraDeactivateButton=new DeactivateWidget (NULL);
  connect (extraDeactivateButton, SIGNAL(clicked()), this, SLOT(deactivate()));
  extraDeactivateButton->hide();
  ConfigReader cfg;
  cfg.add_command_line_shortcut ("h","help",false);
  cfg.add_command_line_shortcut ("d","suppress_deactivation_button",false);
  cfg.add_command_line_shortcut ("a","activation_time",true);
  cfg.append_from_command_line (qApp->argc(), qApp->argv());
  std::string configfile = "../config_files/robotcontrol.cfg";
  cfg.get ("ConfigReader::unknown_argument_1", configfile);
  cfg.append_from_file (configfile.c_str());
  cfg.append_from_command_line (qApp->argc(), qApp->argv());
  bool help=false;
  if (cfg.get("help", help) && help) {
    cerr << "Aufruf: " << qApp->argv()[0] << " [cfg-file] [--host=hostname] [-d] [-a WartezeitNachAktivierungInMSec]\n";
    cerr << " -d unterdrueckt das Umschalten auf schwarzen Bildschirm\n";
    exit (-1);
  }
  msecUntilActivation=1000;
  activateRequest=false;
  blackScreenMode=BlackScreenNeutral;
  latestBlackScreenMode=BlackScreenNeutral;
  bool dummy;
  if (cfg.get ("suppress_deactivation_button", dummy) && dummy)
    blackScreenMode=NoBlackScreen;
  checkBoxBlackScreen->setChecked (blackScreenMode!=NoBlackScreen);
  cfg.get ("activation_time", msecUntilActivation);

  state.field_geometry = FieldGeometry (cfg);

  PaintPreferences pp;
  pp.show_wm_robot=pp.show_wm_ball=pp.show_wm_obstacles=pp.show_vis_lines=true;
  pp.show_vis_ball=pp.show_vis_obstacles=pp.show_vis_goals=pp.show_sl_pos=pp.show_aux_lines=false;
  pp.show_robot_ids=pp.show_robot_ball_links=false;
  pp.show_robot_trace=pp.show_ball_trace=false;
  pp.use_exec_time=false;
  pp.show_colored_goals=false;
  pp.show_direction=true;

  pp.field_heading=+1;
  pp.reference_robot=0;
  pp.imagesource_id=0;
  pp.zoom.orientation=1;
  pp.zoom.own_half=1;

  PaintActionSelect pa;
  pa.show_wm_robot=
  pa.show_wm_ball=
  pa.show_wm_obstacles=
  pa.show_vis_lines=
  pa.show_vis_ball=
  pa.show_vis_obstacles=
  pa.show_vis_goals=true;
  pa.show_sl_pos=
  pa.show_aux_lines=
  pa.show_robot_ids=
  pa.show_robot_ball_links=
  pa.show_robot_trace=
  pa.show_ball_trace=
  pa.clear_lines=
  pa.use_exec_time=
  pa.next_refrobot=
  pa.next_imagesource=false;
  pa.zoom_in=
  pa.zoom_out=
  pa.zoom_all=
  pa.zoom_undo=
  pa.zoom_redo=
  pa.flip_sides=true;
  pa.flip_goals=false;

  field->init (this, state.field_geometry, pp, pa, Qt::DockTop);
  connect (field, SIGNAL(robotDisplacement(Tribots::Vec,Tribots::Angle)), this, SLOT(robotGoTo(Tribots::Vec,Tribots::Angle)));

  state.hostname = "localhost";
  state.port = 59361;
  cfg.get ("host", state.hostname);
  cfg.get ("communication_port", state.port);

  comm.init_as_client (state.hostname.c_str(), state.port);
  state.field_geometry_request=true;
  state.playertypelist_request=true;
  state.playerrolelist_request=true;
  state.first_receive=true;

  state.playerrole=state.desired_playerrole="---";
  state.playertype=state.desired_playertype="---";
  state.refstate=Tribots::RefereeState(0);
  comboBoxRefereeState->clear();
  for (int i=0; i<num_referee_states; i++)
    comboBoxRefereeState->insertItem (referee_state_names[i]);
  state.was_comm_okay.resize (10);
  for (unsigned int i=0; i<state.was_comm_okay.size(); i++)
    state.was_comm_okay[i]=true;

  state.in_game=state.desired_in_game=false;
  state.slhint_request=false;
  state.field_geometry_request=true;
  state.playertypelist_request=true;
  state.playerrolelist_request=true;
  state.exitRequest=false;
  state.drive_to_request=false;

  timer.start (150);
  connect (&timer, SIGNAL(timeout()), this, SLOT(cycleTask()));
  connect (field, SIGNAL(slDisplacement()), this, SLOT(slHintClicked()));
  connect (field, SIGNAL(unresolvedKeyPressEvent(QKeyEvent*)), this, SLOT(keyPressEvent(QKeyEvent*)));

  journalWindow=new QTextEdit (NULL);
  journalWindow->setCaption("Journal");
  journalWindow->setReadOnly(true);

  readFiles (filesDebugImage);
  filepath=get_filepath (std::string(qApp->argv()[0]));
  statusBar()->message(filepath.c_str());
}


void ControlGUI::destroy()
{
  delete extraDeactivateButton;
  delete journalWindow;
  timer.stop();
  killChild();
}


void ControlGUI::activate()
{
  if (!activateRequest)
    activationTime.update();
  activateRequest=true;
  pushButtonDeactivate->setActiveWindow();
  if (blackScreenMode==BlackScreenTribots) {
    extraDeactivateButton->setBlackScreen(false);
    extraDeactivateButton->showFullScreen();
  } else if (blackScreenMode==BlackScreenNeutral) {
    extraDeactivateButton->setBlackScreen(true);
    extraDeactivateButton->showFullScreen();
  }
}


void ControlGUI::deactivate()
{
  state.desired_in_game=false;
  activateRequest=false;
}


void ControlGUI::debugImage()
{
  state.debug_image_request=true;
  timeDebugImage.update();
  readFiles (filesDebugImage);
}


void ControlGUI::playerType( const QString& s )
{
  state.desired_playertype=s.ascii();
}


void ControlGUI::playerRole( const QString& s )
{
  state.desired_playerrole=s.ascii();
}


void ControlGUI::refereeState( const QString& s )
{
  std::string ss (s.ascii());
  for (int i=0; i<num_referee_states; i++)
    if (ss==referee_state_names[i])
      state.desired_refstate=RefereeState(i);
}


void ControlGUI::cycleTask()
{
  if (activateRequest && activationTime.elapsed_msec()>msecUntilActivation) {
    state.desired_in_game=true;
    activateRequest=true;
  }
  if (!extraDeactivateButton->isVisible() || (state.in_game!=state.desired_in_game)) {
    receive ();
    send ();
    updateDisplay ();
    field->setFocus();
    if (extraDeactivateButton->isVisible())
      extraDeactivateButton->update();
  }
  if (timeDebugImage.elapsed_msec()<3000) {
    vector<FileDescription> filesNew;
    readFiles (filesNew);
    unsigned int indexOld=0;
    unsigned int indexNew=0;
    while (indexOld<filesDebugImage.size() || indexNew<filesNew.size()) {
      if (indexOld>=filesDebugImage.size() && indexNew<filesNew.size()) {
        // Eintraege hinzugekommen
        string command = string("display ")+filesNew[indexNew].name.ascii()+" &";
        system (command.c_str());
        indexNew++;
        continue;
      }
      if (indexOld<filesDebugImage.size() && indexNew>=filesNew.size()) {
        // Eintraege entfallen
        break;
      }
      if (filesDebugImage[indexOld]==filesNew[indexNew]) {
        // gleiche Eintraege
        indexNew++;
        indexOld++;
        continue;
      }
      if (filesDebugImage[indexOld].modified<filesNew[indexNew].modified) {
        // Eintrage entfallen
        indexOld++;
        continue;
      }
      if (filesDebugImage[indexOld].modified>filesNew[indexNew].modified) {
        // Eintrage hinzugekommen
        string command = string("display ")+filesNew[indexNew].name.ascii()+" &";
        system (command.c_str());
        indexNew++;
        continue;
      }
      // Eintraege mit gleicher Zeit aber irgenwie sonst anders
      string command = string("display ")+filesNew[indexNew].name.ascii()+" &";
      system (command.c_str());
      indexNew++;
      indexOld++;
    }
    filesDebugImage=filesNew;
  }
}


void ControlGUI::send()
{
  if (state.in_game!=state.desired_in_game) {
    comm.putInGame (state.desired_in_game);
  }

  if (state.playertype!=state.desired_playertype) {
    comm.putPlayerType (state.desired_playertype.c_str());
    state.playerrolelist_request=true;
  }

  if (state.playerrole!=state.desired_playerrole)
    comm.putPlayerRole (state.desired_playerrole.c_str());

  if (state.refstate!=state.desired_refstate)
    comm.putRefereeState (state.desired_refstate);

  if (state.slhint_request)
    comm.putSLHint (state.slhint_pos, state.slhint_angle);
  state.slhint_request=false;

  if (state.drive_to_request) {
    state.drive_to_request=false;
    state.message_board.publish_stream () << "GotoPos: " << static_cast<int>(state.drive_to_pos.x) << ' ' << static_cast<int>(state.drive_to_pos.y) << ' ' << static_cast<int>(state.drive_to_angle.get_deg()) << '\n';
  }

  comm.putMessageBoard (state.message_board.get_outgoing());
  state.message_board.clear_outgoing();

  comm.putStandardRequest();
  if (state.field_geometry_request)
    comm.putFieldGeometryRequest();
  if (state.playertypelist_request)
    comm.putPlayerTypeListRequest();
  if (state.playerrolelist_request)
    comm.putPlayerRoleListRequest();
  if (state.debug_image_request) {
    state.debug_image_request=false;
    comm.putDebugImageRequest();
  }
  comm.putObstacleLocationRequest();
  comm.putVisibleObjectListRequest();
  comm.putMessageBoardRequest();
  if (state.exitRequest) {
    comm.putExitRequest();
    if (state.comm_interrupted)
      state.exitRequest=false;
  }

  comm.send();
}


void ControlGUI::receive()
{
  unsigned int num_received_packets = comm.receive ();

  vector<string> stringlist;
  string s;

  stringlist.clear();
  if (comm.getPlayerTypeList(stringlist)) {
    state.list_players = stringlist;
    state.playertypelist_request=false;
  }

  if (comm.getPlayerRoleList(stringlist)) {
    state.list_roles = stringlist;
    state.playerrolelist_request=false;
  }

  if (comm.getPlayerType(s)) {
    if (s!=state.playertype)
      state.playerrolelist_request=true;
    if (state.playertype==state.desired_playertype)
      state.desired_playertype=s;
    state.playertype = s;
  }

  if (comm.getPlayerRole(s)) {
    if (state.playerrole==state.desired_playerrole)
      state.desired_playerrole=s;
    state.playerrole = s;
  }

  if (comm.getPlayerRole(s)) {
    if (state.playerrole==state.desired_playerrole)
      state.desired_playerrole=s;
    state.playerrole = s;
  }

  comm.getVisibleObjectList(state.visible_objects);
  comm.getBallLocation(state.ball_pos);
  TeammateOccupancyGrid occ;
  comm.getRobotLocation(state.robot_pos, occ);
  comm.getObstacleLocation(state.obs_pos);
  comm.getVcc (state.motor_vcc);
  if (comm.getFieldGeometry(state.field_geometry)) {
    state.field_geometry_request=false;
    field->set_field_geometry (state.field_geometry);
    field->zoom_all();
  }

  Tribots::GameState gs;
  if (comm.getGameState(gs)) {
    if (state.refstate==state.desired_refstate && !state.first_receive)
      state.desired_refstate=gs.refstate;
    state.refstate=gs.refstate;
    if (state.in_game==state.desired_in_game)
      state.desired_in_game=gs.in_game;
    state.in_game=gs.in_game;
  }

  state.message_board.clear_incoming();
  vector<string> incoming_messages;
  comm.getMessageBoard (incoming_messages);
  state.message_board.receive (incoming_messages);

  state.comm_okay = (num_received_packets>0);
  if (state.comm_okay) state.latest_receive.update();
  state.comm_interrupted = (state.latest_receive.elapsed_msec()>5000);
  if (state.latest_receive.elapsed_msec()>10000) {
    // bei unterbrochener Verbindung koennte ein Programmabbruch vorliegen,
    // daher einige Werte zurucksetzen, um bei wiederaufgenommener
    // Kommunikation diese Werte erneut abzufragen
    state.field_geometry_request = true;
    state.playertypelist_request = true;
    state.playerrolelist_request = true;
  }
  state.first_receive &= (num_received_packets==0);
  if (state.comm_interrupted) {
    state.first_receive=true;
  }
}


void ControlGUI::updateDisplay()
{
  // Kommunikation & Status
  unsigned int num_comm_okay=0;
  for (unsigned int i=1; i<state.was_comm_okay.size(); i++) {
    state.was_comm_okay[i-1]=state.was_comm_okay[i];
    if (state.was_comm_okay[i])
      num_comm_okay++;
  }
  state.was_comm_okay[state.was_comm_okay.size()-1]=state.comm_okay;
  if (state.was_comm_okay[state.was_comm_okay.size()-1])
    num_comm_okay++;
  monoLEDConnection->setColor(num_comm_okay==0 ? Qt::blue : (num_comm_okay==state.was_comm_okay.size() ? Qt::green : Qt::yellow));
  monoLEDConnection->setOn(true);
  monoLEDActivated->setColor(state.in_game ? Qt::green : Qt::red);
  monoLEDActivated->setOn(true);
  int mvcc=static_cast<int>(state.motor_vcc);
  lCDNumberVoltage->display (mvcc);
  lCDNumberVoltage->setPaletteBackgroundColor (mvcc<20 ? Qt::red : (mvcc<22 ? Qt::yellow : Qt::green));

  // Playertypelist
  bool playertypelist_changed = (static_cast<unsigned int>(comboBoxPlayertype->count()) != state.list_players.size());
  if (!playertypelist_changed) {
    for (unsigned int i=0; i<state.list_players.size(); i++) {
      bool found=false;
      for (unsigned int j=0; j<state.list_players.size(); j++) {
        if (state.list_players[i]==std::string(comboBoxPlayertype->text (j).ascii())) {
          found=true;
          break;
        }
      }
      if (!found) {
        playertypelist_changed=true;
        break;
      }
    }
  }
  if (playertypelist_changed) {
    comboBoxPlayertype->clear();
    for (unsigned  int i=0; i<state.list_players.size(); i++)
      comboBoxPlayertype->insertItem(state.list_players[i].c_str());
  }

  // Playertype
  if (playertypelist_changed || state.playertype!=comboBoxPlayertype->currentText().ascii() ) {
    bool playertype_found=false;
    for (int i=0; i<comboBoxPlayertype->count(); i++)
      if (std::string(comboBoxPlayertype->text(i).ascii())==state.playertype)
        playertype_found=true;
    if (!playertype_found)
      comboBoxPlayertype->insertItem (state.playertype.c_str());
    comboBoxPlayertype->setCurrentText (state.playertype.c_str());
  }

  // Playerrolelist
  bool playerrolelist_changed = (static_cast<unsigned int>(comboBoxPlayerrole->count()) != state.list_roles.size());
  if (!playerrolelist_changed) {
    for (unsigned int i=0; i<state.list_roles.size(); i++) {
      bool found=false;
      for (unsigned int j=0; j<state.list_roles.size(); j++)
        if (state.list_roles[i]==std::string(comboBoxPlayerrole->text (j).ascii())) {
          found=true;
          break;
      }
      if (!found) {
        playerrolelist_changed=true;
        break;
      }
    }
  }
  if (playerrolelist_changed) {
    comboBoxPlayerrole->clear();
    for (unsigned  int i=0; i<state.list_roles.size(); i++)
      comboBoxPlayerrole->insertItem(state.list_roles[i].c_str());
  }

  // Playerrole
  if (playerrolelist_changed || state.playerrole!=comboBoxPlayerrole->currentText().ascii()) {
    bool playerrole_found=false;
    for (int i=0; i<comboBoxPlayerrole->count(); i++)
      if (std::string(comboBoxPlayerrole->text(i).ascii())==state.playerrole)
        playerrole_found=true;
      if (!playerrole_found)
        comboBoxPlayerrole->insertItem (state.playerrole.c_str());
    comboBoxPlayerrole->setCurrentText (state.playerrole.c_str());
  }

   // Refstate
  if (referee_state_names[state.refstate]!=comboBoxRefereeState->currentText().ascii()) {
    comboBoxRefereeState->setCurrentText(Tribots::referee_state_names[state.refstate]);
  }

  CycleInfo ci;
  ci.rloc_vis.push_back (state.robot_pos);
  ci.bloc_vis.push_back (state.ball_pos);
  ci.oloc=state.obs_pos;
  if (state.visible_objects.size()>0)
    ci.vloc.push_back (state.visible_objects[0]);
  if (state.message_board.scan_for_prefix ("pB!")=="pB!") {
    vector<int> attrvec (1);  // nur doPossessBall anzeigen
    attrvec[0]=1;
    ci.robot_attr.push_back (attrvec);
  }

  field->next_cycle (ci);
}


void ControlGUI::slHintClicked()
{
  const TribotsTools::CycleInfo& ci = field->get_cycle_info ();
  if (ci.rloc_vis.size()>0) {
    state.slhint_request=true;
    state.slhint_pos=ci.rloc_vis[0].pos;
    state.slhint_angle=ci.rloc_vis[0].heading;
  }
  field->forceMouseLocalizationClick (false);
  SLHintAction->setOn(false);
}


void ControlGUI::slHintActivated()
{
  field->forceMouseLocalizationClick (true);
}


void ControlGUI::keyPressEvent ( QKeyEvent* ev ) 
{
  deactivate();
}


void ControlGUI::execRobotcontrol()
{
  killChild();
  if (blackScreenMode!=NoBlackScreen)
    blackScreenMode = BlackScreenTribots;
  int pid = fork ();
  if (pid==0) {
    execlp ("xterm", "xterm", "-hold", "-sb", "-e", "./robotcontrol", "-no_rotate_log", "-restart", NULL);
  } else if (pid>0) {
    robotcontrol_child_pid=pid;
  } else {
    statusBar()->message("Fehler beim Aufruf von fork()",5000);
  }
}


void ControlGUI::quitRobotcontrol()
{
  state.exitRequest=true;
}


void ControlGUI::robotGoTo( Tribots::Vec p, Tribots::Angle a)
{
  state.drive_to_pos=p;
  state.drive_to_angle=a;
  state.drive_to_request=true;
  GotoAction->setOn(false);
 field->forceMouseGotoClick (false);
}


void ControlGUI::gotoActivated()
{
 field->forceMouseGotoClick (true);
}


void ControlGUI::execRestartCaller()
{
  killChild();
  if (blackScreenMode!=NoBlackScreen)
    blackScreenMode = BlackScreenNeutral;
  int pid = fork ();
  if (pid==0) {
    execlp ("xterm", "xterm", "-hold", "-sb", "-e", "~/.robotcontrol/competitiondir/start.sh", NULL);
  } else if (pid>0) {
    robotcontrol_child_pid=pid;
  } else {
    statusBar()->message("Fehler beim Aufruf von fork()",5000);
  }
}


void ControlGUI::execColorToolOmni()
{
  string command="./colorTool -o &";
  system(command.c_str());
}


void ControlGUI::execColorToolPerspective()
{
  string command="./colorTool -p &";
  system(command.c_str());
}


void ControlGUI::execCalibrationTool()
{
  string command="./CalibrationTool &";
  system(command.c_str());
}


void ControlGUI::execMarkerEditor()
{
  string command="./marker_editor &";
  system(command.c_str());
}


void ControlGUI::execCoriander()
{
  string command ("coriander &");
  system(command.c_str());
}


void ControlGUI::execConfigEditor()
{
  string command="./config_editor &";
  system(command.c_str());
}


void ControlGUI::execJournalViewer()
{
  std::string journalText;
  ifstream jin ("journal.out");
  if (!jin) {
    statusBar()->message ("Journal file 'journal.out' not found in working directory.", 5000);
    return;
  }
  while (!jin.eof()) {
    string line;
    getline (jin, line);
    journalText+=line+"\n";
  }
  journalWindow->setText(journalText.c_str());
  journalWindow->showMaximized();
}


void ControlGUI::execTribotsview()
{
  string command="./tribotsview &";
  system(command.c_str());
}


void ControlGUI::toggleBlackScreen( bool b )
{
  if (b) {
    blackScreenMode=latestBlackScreenMode;
  } else {
    latestBlackScreenMode=blackScreenMode;
    blackScreenMode=NoBlackScreen;
  }
}

void ControlGUI::killChild ()
{
  if (robotcontrol_child_pid>0) {
    stringstream inout;
    inout << "kill " << robotcontrol_child_pid << '\n';
    std::string killcommand;
    getline (inout, killcommand);
    system (killcommand.c_str());
  }
}
