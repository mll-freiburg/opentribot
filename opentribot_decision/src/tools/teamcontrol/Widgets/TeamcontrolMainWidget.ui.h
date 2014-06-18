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

#include <qapplication.h>
#include "../Logic/CoachLogger.h"

void TeamcontrolMainWidget::init() 
{  // zunaechst muss init(ConfigReader&) aufgerufen werden
  cycle_timer=NULL;
  cycle_timer_refbox=NULL;
  teamcontrol_mode=TribotsTools::teamcontrolUnknownMode;
  masterslave_client=new TribotsTools::TeamcontrolSlaveClient;
  masterslave_server=new TribotsTools::TeamcontrolMasterServer;
  cycle_task_mutex = 0;
  layout=NULL;
  setCentralWidget(new QWidget (this));
}


void TeamcontrolMainWidget::destroy()
{
  TribotsTools::CoachLogger::getCoachLogger().destroy();
  if (cycle_timer) {
    delete cycle_timer;
    delete cycle_timer_refbox;
    delete field_widget;
    delete coach_widget;
    delete refbox_widget;
    delete joystick_widget;
    delete communication_load_widget;
    for (unsigned int i=0; i<robot_widget.size(); i++)
      delete robot_widget[i];
    delete coach;
    delete joystick_control;
    delete refbox_control;
    for (unsigned int i=0; i<remote_robot.size(); i++)
      delete remote_robot[i];
  }
  delete masterslave_client;
  delete masterslave_server;
}


void TeamcontrolMainWidget::init(const Tribots::ConfigReader& cfg) 
{
  // Quietmode dient dazu, das Teamcontrol ohne grosse Fenster zu starten; Sinnvoll vor allem für Simulatortests
  try_to_connect=false;
  cfg.get("try_to_connect",try_to_connect);
  std::string wrd="";
  cfg.get ("lokalisierung_seite", wrd);
  if (wrd=="links")
    REMBB.team_state.localization_side = -1;
  else if (wrd=="rechts")
    REMBB.team_state.localization_side = +1;
  else
    REMBB.team_state.localization_side = 0;
  bool noLogging=false;
  cfg.get ("nolog", noLogging);

  // zuerst die Logik erzeugen
  refbox_control = new TribotsTools::RefboxControl (cfg);
  joystick_control = new TribotsTools::JoystickControl;
  std::vector<std::string> words;
  cfg.get ("robots", words);
  if (words.size()==0)
    throw Tribots::InvalidConfigurationException ("robots");
  REMBB.robot_state.resize (words.size());
  for (unsigned int i=0; i<words.size(); i++)
    remote_robot.push_back (new TribotsTools::RemoteRobot (cfg, i, words[i]));
  coach = new TribotsTools::Coach (cfg);

  //  dann die Fenster erzeugen
  bool b;
  refbox_widget  = new RefboxWidget;
  connect (fensterRefereeStatesAction, SIGNAL(activated()), refbox_widget, SLOT(show()));
  if (cfg.get ("refbox_widget", b) && b)
    refbox_widget->show();

  joystick_widget = new JoystickWidget;
  connect (fensterJoystickAction, SIGNAL(activated()), joystick_widget, SLOT(show()));
  if (cfg.get ("joystick_widget", b) && b)
    joystick_widget->show();

  for (unsigned int i=0; i<remote_robot.size(); i++) {
    robot_widget.push_back (new RobotWidget (centralWidget()));
    show_robot_widget.push_back (false);
    robot_widget[i]->init (i,cfg);
    TribotsTools::IDAction* ra = new TribotsTools::IDAction (NULL,i);
    ra->setToggleAction(true);
    ra->setText (REMBB.robot_state[i].name.c_str());
    ra->addTo (Roboter);
    connect (ra, SIGNAL(toggled(unsigned int, bool)), this, SLOT(robotToggled(unsigned int, bool)));
    if (cfg.get ((words[i]+std::string("::widget")).c_str(), b) && b) {
      ra->setOn(true);
      robot_widget[i]->show();
      show_robot_widget[show_robot_widget.size()-1]=true;
    }
  }
  rearrangeRobotWidgets();
  coach_widget = new CoachWidget;
  connect (fensterTrainerAction, SIGNAL(activated()), coach_widget, SLOT(show()));
  coach_widget->init(cfg);
  if (cfg.get ("coach_widget", b) && b)
    coach_widget->show();

  field_widget = new TeamcontrolFieldWidget;
  connect (fensterSpielfeldAction, SIGNAL(activated()), field_widget, SLOT(show()));
  if (cfg.get ("field_widget", b) && b)
    field_widget->show();

  communication_load_widget = new CommunicationLoadWidget;
  connect (fensterCommunicationLoadAction, SIGNAL(activated()), communication_load_widget, SLOT(show()));
  connect (communication_load_widget->spin_rate, SIGNAL(valueChanged(int)), this, SLOT(cycleRateChanged(int)));
  connect (communication_load_widget->spin_lines, SIGNAL(valueChanged(int)), this, SLOT(linesRateChanged(int)));
  if (cfg.get ("communication_load_widget", b) && b)
    communication_load_widget->show();

  // die Timer einstellen; cycle_timer_refbox ist unabhaengig von anderem Timer, um keine Refereebox-Signale zu verpassen
  cycle_timer = new QTimer;
  cycle_timer->start (150);
  connect (cycle_timer, SIGNAL(timeout()), this, SLOT(cycle_task()));
  cycle_timer_refbox = new QTimer;
  cycle_timer_refbox->start (50);
  connect (cycle_timer_refbox, SIGNAL(timeout()), this, SLOT(cycle_task_refbox()));

  if (!startAsMaster()) {
    stopAsMaster();
    startAsSingle();
    teamcontrol_mode=TribotsTools::teamcontrolSingleMode;
  } else {
    teamcontrol_mode=TribotsTools::teamcontrolMasterMode;
  }

  if (!noLogging)
    TribotsTools::CoachLogger::getCoachLogger().create();
}


void TeamcontrolMainWidget::cycleRateChanged(int n) 
{
  cycle_timer->changeInterval (n);
  REMBB.team_state.comm_rate = n;
}


void TeamcontrolMainWidget::cycle_task() 
{
  if (cycle_task_mutex) { std::cerr << "cycle task mutex is locked" << std::endl; return; }
  cycle_task_mutex = 1;

  if (try_to_connect) {
    // ein Bisschen gefaehrlich, also nur im Simulator einsetzen
    for (unsigned int i=0; i<remote_robot.size(); i++) {
      REMBB.robot_state[i].desired_connect=true;
    }
  }

  // erst Update auf die Logik
  for (unsigned int i=0; i<robot_widget.size(); i++)
    robot_widget[i]->updateRequests();
  field_widget->updateRequests();

  if (teamcontrol_mode!=TribotsTools::teamcontrolSlaveMode) {
    if (teamcontrol_mode==TribotsTools::teamcontrolMasterMode) {
      masterslave_server->receive();
    }
    for (unsigned int i=0; i<remote_robot.size(); i++)
      remote_robot[i]->receive();
    refbox_control->update_synch();
    coach->update();
    joystick_control->update();
    if (teamcontrol_mode==TribotsTools::teamcontrolMasterMode) {
      masterslave_server->send();
    }
  } else {
    masterslave_client->receive();
  }

  // dann Update auf die Fenster
  for (unsigned int i=0; i<robot_widget.size(); i++)
    robot_widget[i]->update();
  joystick_widget->update();
  coach_widget->update();

  refbox_widget->update();
  field_widget->update();
  communication_load_widget->update();

  // Loggen
  TribotsTools::CoachLogger::getCoachLogger().log_cycleend();
  TribotsTools::CoachLogger::getCoachLogger().log_cyclebegin();

  // zum Schluss senden
  if (teamcontrol_mode!=TribotsTools::teamcontrolSlaveMode) {
    for (unsigned int i=0; i<remote_robot.size(); i++)
      remote_robot[i]->send();
  } else {
    masterslave_client->send();
  }
  if (teamcontrol_mode==TribotsTools::teamcontrolSlaveMode) {
    if (masterslave_client->comm_interrupted())
      setPaletteBackgroundColor (QColor(255,0,0));
    else
      setPaletteBackgroundColor (QColor (238, 185, 175));
  }
  cycle_task_mutex = 0;
}


void TeamcontrolMainWidget::cycle_task_refbox()
{
  if (teamcontrol_mode!=TribotsTools::teamcontrolSlaveMode) {
    // Update auf Refbox-Control
    refbox_control->update();
  }
}

void TeamcontrolMainWidget::linesRateChanged( int n )
{
  REMBB.team_state.send_lines_rate=n;
}


void TeamcontrolMainWidget::masterModeSelected()
{
  if (startAsMaster()) {
    stopAsSingle();
    if (teamcontrol_mode==TribotsTools::teamcontrolSlaveMode)
      stopAsSlave();
    teamcontrol_mode=TribotsTools::teamcontrolMasterMode;
    statusBar()->message("Master Modus");
  } else {
    stopAsMaster();
  }
}

void TeamcontrolMainWidget::slaveModeSelected()
{
  if (startAsSlave()) {
    stopAsMaster();
    stopAsSingle();
    teamcontrol_mode=TribotsTools::teamcontrolSlaveMode;
    statusBar()->message("Slave Modus");
  } else {
    stopAsSlave();
  }
}

void TeamcontrolMainWidget::singleModeSelected()
{
  if (startAsSingle()) {
    stopAsMaster();
    if (teamcontrol_mode==TribotsTools::teamcontrolSlaveMode)
      stopAsSlave();
    teamcontrol_mode=TribotsTools::teamcontrolSingleMode;
    statusBar()->message("Single Modus");
  } else {
    stopAsSingle();
  }
}

bool TeamcontrolMainWidget::startAsSlave()
{
  std::string addressport = QInputDialog::getText ("Master-Adresse", "IP-Adresse und Port des Masterteamcontrols:", QLineEdit::Normal, "localhost:59381");
  std::string::size_type poscolon = addressport.find (':',0);
  bool addressokay=true;
  bool connectionokay=false;
  std::string address="";
  unsigned int port=0;
  if (poscolon==std::string::npos) {
    addressokay=false;
  } else {
    address = addressport.substr (0, poscolon);
    addressokay &= Tribots::string2uint (port, addressport.substr (poscolon+1, addressport.length()));
    addressokay &= (address.length()>0);

    connectionokay=masterslave_client->connect (address, port);
  }

  if (addressokay && connectionokay) {
    MasterAction->setOn (false);
    SlaveAction->setOn (true);
    MasterOnlyAction->setOn (false);
    communication_load_widget->spin_rate->setEnabled(false);
    communication_load_widget->spin_lines->setEnabled(false);
    setPaletteBackgroundColor (QColor (238, 185, 175));
    update();
    refbox_widget->startAsSlave();
    for (unsigned int i=0; i<robot_widget.size(); i++)
      robot_widget[i]->startAsSlave();
    REMBB.team_state.refbox_connected=false;
    refbox_control->update();
    cycle_timer->changeInterval (47);
    return true;
  } else {
    return false;
  }
}

bool TeamcontrolMainWidget::startAsMaster()
{
  if (teamcontrol_mode==TribotsTools::teamcontrolMasterMode)
    return true;
  if (!masterslave_server->connect (59381))
    return false;
  MasterAction->setOn (true);
  SlaveAction->setOn (false);
  MasterOnlyAction->setOn (false);
  setPaletteBackgroundColor (QColor (238, 238, 230));
  update();

  return true;
}

bool TeamcontrolMainWidget::startAsSingle()
{
  MasterAction->setOn (false);
  SlaveAction->setOn (false);
  MasterOnlyAction->setOn (true);
  setPaletteBackgroundColor (QColor (255, 255, 255));
  update();
  return true;
}

bool TeamcontrolMainWidget::stopAsSlave()
{
  communication_load_widget->spin_rate->setEnabled(true);
  communication_load_widget->spin_lines->setEnabled(true);
  SlaveAction->setOn (false);
  refbox_widget->stopAsSlave();
  for (unsigned int i=0; i<robot_widget.size(); i++) {
    robot_widget[i]->stopAsSlave();
    REMBB.robot_state[i].desired_connect=false;
  }

  masterslave_client->unconnect();
  cycle_timer->changeInterval (REMBB.team_state.comm_rate);
  return true;
}

bool TeamcontrolMainWidget::stopAsMaster()
{
  MasterAction->setOn (false);
  masterslave_server->unconnect();
  return true;
}

bool TeamcontrolMainWidget::stopAsSingle()
{
  MasterOnlyAction->setOn (false);
  return true;
}


void TeamcontrolMainWidget::showMasterSlaveComm()
{
  std::string msg;
  switch (teamcontrol_mode) {
  case TribotsTools::teamcontrolSingleMode : msg="Single-Modus\n"; break;
  case TribotsTools::teamcontrolSlaveMode : msg="Slave-Modus\n"+masterslave_client->commStatus(); break;
  case TribotsTools::teamcontrolMasterMode : msg="Master-Modus\n"+masterslave_server->commStatus(); break;
  default: msg = "Kommunikationsstatus unbekannt\n"; break;
  }
  QMessageBox::information (this, "Master-Slave-Kommunikationsstatus", msg.c_str(), QMessageBox::Ok);
}


void TeamcontrolMainWidget::robotToggled( unsigned int id, bool b)
{
  show_robot_widget[id]=b;
  rearrangeRobotWidgets();
}


void TeamcontrolMainWidget::rearrangeRobotWidgets()
{
  if (layout)
    delete layout;
  layout=new QGridLayout (centralWidget(), 1, 1, 7, 3);
  unsigned int numActive=0;
  for (unsigned int i=0; i<robot_widget.size(); i++) {
    if (show_robot_widget[i])
      numActive++;
    else
      robot_widget[i]->hide();
  }
  if (numActive==0)
    return;
//  unsigned int screenWidth=qApp->desktop()->screenGeometry().width();
  unsigned int screenHeight=qApp->desktop()->screenGeometry().height();
//  unsigned int widgetWidth=robot_widget[0]->sizeHint().width();
  unsigned int widgetHeight=robot_widget[0]->sizeHint().height();
//  unsigned int maxHorizontal = screenWidth/widgetWidth;
  unsigned int maxVertical = screenHeight/(widgetHeight+10);
  unsigned int maxCols=numActive/maxVertical+(numActive%maxVertical==0 ? 0 : 1);
  unsigned int maxRows=numActive/maxCols+(numActive%maxCols==0 ? 0 : 1);
  unsigned int pos=0;
  for (unsigned int i=0; i<robot_widget.size(); i++) {
    if (show_robot_widget[i]) {
      layout->addWidget ( robot_widget[i], pos%maxRows, pos/maxRows);
      robot_widget[i]->show();
      pos++;
    }
  }
}
