
#include "../States/RemoteBlackboard.h"
#include "../../../../Fundamental/stringconvert.h"
#include "TeamcontrolMainWidget.h"
#include <iostream>
#include <QtGui/QInputDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QStatusBar>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

#define MONITOR_CYCLE_TIME 0

#if MONITOR_CYCLE_TIME
#include "../../../../Fundamental/Time.h"
namespace {
  unsigned int counter0=0;
  Tribots::Time timer0;
}
#endif

TeamcontrolMainWidget::TeamcontrolMainWidget(QWidget* p, Qt::WindowFlags f) : QMainWindow (p,f)
{  // zunaechst muss init(ConfigReader&) aufgerufen werden
  setupUi(this);
  connect (spin_rate,SIGNAL(valueChanged(int)),this,SLOT(cycleRateChanged(int)));
  connect (spin_lines,SIGNAL(valueChanged(int)),this,SLOT(linesRateChanged(int)));
  connect (SlaveAction,SIGNAL(triggered()),this,SLOT(slaveModeSelected()));
  connect (MasterAction,SIGNAL(triggered()),this,SLOT(masterModeSelected()));
  connect (MasterOnlyAction,SIGNAL(triggered()),this,SLOT(singleModeSelected()));
  connect (masterSlaveMessageAction,SIGNAL(triggered()),this,SLOT(showMasterSlaveComm()));

  cycle_timer=NULL;
  cycle_timer_refbox=NULL;
  teamcontrol_mode=TribotsTools::teamcontrolUnknownMode;
  masterslave_client=new TribotsTools::TeamcontrolSlaveClient;
  masterslave_server=new TribotsTools::TeamcontrolMasterServer;
}


TeamcontrolMainWidget::~TeamcontrolMainWidget ()
{
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
  try_to_connect=false;
  cfg.get("try_to_connect",try_to_connect);
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
  connect (fensterRefereeStatesAction, SIGNAL(triggered()), refbox_widget, SLOT(show()));
  if (cfg.get ("refbox_widget", b) && b)
    refbox_widget->show();

  joystick_widget = new JoystickWidget;
  connect (fensterJoystickAction, SIGNAL(triggered()), joystick_widget, SLOT(show()));
  if (cfg.get ("joystick_widget", b) && b)
    joystick_widget->show();

  for (unsigned int i=0; i<remote_robot.size(); i++) {
    robot_widget.push_back (new RobotWidget);
    robot_widget[i]->init (i,cfg);
    QAction* ra = new QAction ("",NULL);
    ra->setCheckable(false);
    ra->setText (REMBB.robot_state[i].name.c_str());
    Roboter->addAction (ra);
    connect (ra, SIGNAL(triggered()), robot_widget[i], SLOT(show()));
    if (cfg.get ((words[i]+std::string("::widget")).c_str(), b) && b) {
      robot_widget[i]->show();
    }
  }
  coach_widget = new CoachWidget;
  connect (fensterTrainerAction, SIGNAL(triggered()), coach_widget, SLOT(show()));
  coach_widget->init(cfg);
  if (cfg.get ("coach_widget", b) && b)
    coach_widget->show();

  field_widget = new TeamcontrolFieldWidget;
  connect (fensterSpielfeldAction, SIGNAL(triggered()), field_widget, SLOT(show()));
  if (cfg.get ("field_widget", b) && b)
    field_widget->show();

  communication_load_widget = new CommunicationLoadWidget;
  connect (fensterCommunicationLoadAction, SIGNAL(triggered()), communication_load_widget, SLOT(show()));
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

}


void TeamcontrolMainWidget::cycleRateChanged(int n) 
{
  cycle_timer->setInterval (n);
  REMBB.team_state.comm_rate = n;
}


void TeamcontrolMainWidget::cycle_task() 
{
#if MONITOR_CYCLE_TIME
  counter0++;
  if (counter0>=30) {
    std::cerr << "Zykluszeit: " << timer0.elapsed_msec()/30 << " msec\n";
    timer0.update();
    counter0=0;
  }
#endif

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

  // zum Schluss senden
  if (teamcontrol_mode!=TribotsTools::teamcontrolSlaveMode) {
    for (unsigned int i=0; i<remote_robot.size(); i++)
      remote_robot[i]->send();
  } else {
    masterslave_client->send();
  }
  QPalette qpal;
  if (teamcontrol_mode==TribotsTools::teamcontrolSlaveMode) {
    if (masterslave_client->comm_interrupted())
      qpal.setBrush (QPalette::Background, Qt::red);
    else
      qpal.setBrush (QPalette::Background, QColor(238,185,175));
    setPalette (qpal);
  }
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
    statusBar()->showMessage("Master Modus");
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
    statusBar()->showMessage("Slave Modus");
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
    statusBar()->showMessage("Single Modus");
  } else {
    stopAsSingle();
  }
}

bool TeamcontrolMainWidget::startAsSlave()
{
  std::string addressport = std::string(QInputDialog::getText (this, "Master-Adresse", "IP-Adresse und Port des Masterteamcontrols:", QLineEdit::Normal, "localhost:59381").toAscii());
  unsigned int poscolon = addressport.find (':',0);
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
    MasterAction->setChecked (false);
    SlaveAction->setChecked (true);
    MasterOnlyAction->setChecked (false);
    spin_rate->setEnabled(false);
    spin_lines->setEnabled(false);
    QPalette qpal;
    qpal.setBrush (QPalette::Background, QColor(238,185,175));
    setPalette (qpal);
    update();
    refbox_widget->startAsSlave();
    for (unsigned int i=0; i<robot_widget.size(); i++)
      robot_widget[i]->startAsSlave();
    REMBB.team_state.refbox_connected=false;
    refbox_control->update();
    cycle_timer->setInterval (47);
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
  MasterAction->setChecked (true);
  SlaveAction->setChecked (false);
  MasterOnlyAction->setChecked (false);
  QPalette qpal;
  setPalette (qpal);
  update();

  return true;
}

bool TeamcontrolMainWidget::startAsSingle()
{
  MasterAction->setChecked (false);
  SlaveAction->setChecked (false);
  MasterOnlyAction->setChecked (true);
  QPalette qpal;
  qpal.setBrush (QPalette::Background, QColor(255,255,255));
  setPalette (qpal);
  update();
  return true;
}

bool TeamcontrolMainWidget::stopAsSlave()
{
  spin_rate->setEnabled(true);
  spin_lines->setEnabled(true);
  SlaveAction->setChecked (false);
  refbox_widget->stopAsSlave();
  for (unsigned int i=0; i<robot_widget.size(); i++) {
    robot_widget[i]->stopAsSlave();
    REMBB.robot_state[i].desired_connect=false;
  }

  masterslave_client->unconnect();
  cycle_timer->setInterval (REMBB.team_state.comm_rate);
  return true;
}

bool TeamcontrolMainWidget::stopAsMaster()
{
  MasterAction->setChecked (false);
  masterslave_server->unconnect();
  return true;
}

bool TeamcontrolMainWidget::stopAsSingle()
{
  MasterOnlyAction->setChecked (false);
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

void TeamcontrolMainWidget::closeEvent ( QCloseEvent *)
{
  emit(widgetClosed());
}
