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

#include <sstream>
#include <qapplication.h>
#include <stdlib.h>
void RobotWidget::init()
{
  robotcontrol_child_pid=-1;
  data_widget=new RobotDataWidget;
  playerrole = "---";
  playertype = "---";
  ip="";
  port=-1;
  refstate = Tribots::RefereeState (0);
  combo_refstate->clear();
  for (int i=0; i<Tribots::num_referee_states; i++)
    combo_refstate->insertItem (Tribots::referee_state_names[i]);
  was_comm_okay.resize (10);
  for (unsigned int i=0; i<was_comm_okay.size(); i++)
    was_comm_okay[i]=true;
}


void RobotWidget::destroy()
{
  killChild();
  delete data_widget;
}


void RobotWidget::init( unsigned int n, const Tribots::ConfigReader& cfg )
{
  robot=n;
  setCaption(REMBB.robot_state[n].name.c_str());
  data_widget->init(n);
  check_connect->setChecked(REMBB.robot_state[robot].comm_started);
  check_activate->setChecked(REMBB.robot_state[robot].in_game);
  check_joystick->setChecked(false);
  check_robotdata->setChecked(false);
  check_messageboard->setChecked(false);
  std::stringstream inout;
  inout << REMBB.robot_state[robot].port << '\n';
  std::string port_string;
  std::getline (inout, port_string);
  edit_ip->setText((REMBB.robot_state[robot].ip+std::string(":")+port_string).c_str());
  ip=REMBB.robot_state[robot].ip;
  port=REMBB.robot_state[robot].port;
  tabWidget->setTabLabel(tabWidget->page(0), REMBB.robot_state[n].name.c_str());
  update ();
}


void RobotWidget::hide()
{
  REMBB.robot_state[robot].in_game=false;
  REMBB.robot_state[robot].desired_connect=false;
  data_widget->hide();
  QWidget::hide();
}


void RobotWidget::connectChanged()
{
  REMBB.robot_state[robot].desired_connect = check_connect->isChecked();
}


void RobotWidget::activateChanged()
{
  REMBB.robot_state[robot].desired_in_game = check_activate->isChecked();
}


void RobotWidget::messageBoardChanged()
{
  REMBB.robot_state[robot].show_message_board = check_messageboard->isChecked();
}


void RobotWidget::playertypeChanged( int )
{
  REMBB.robot_state[robot].desired_playertype = combo_playertype->currentText().ascii();
}


void RobotWidget::playerroleChanged( int )
{
  REMBB.robot_state[robot].desired_playerrole = combo_playerrole->currentText().ascii();
}


void RobotWidget::refstateChanged( int i )
{
  REMBB.robot_state[robot].desired_refstate = Tribots::RefereeState (i);
}


void RobotWidget::ipChanged()
{
  std::string ipport = edit_ip->text().ascii();
  unsigned int dp = ipport.find (':',0);
  std::string ip = ipport.substr (0, dp);
  std::string port_string = ipport.substr (dp+1, ipport.length()-dp);
  if (ip.length()>0)
    REMBB.robot_state[robot].ip = ip;
  if (port_string.length()>0)
    Tribots::string2int (REMBB.robot_state[robot].port, port_string);  
}


void RobotWidget::teamChanged()
{
  // Slot nicht mehr verwendet
  REMBB.robot_state[robot].desired_own_half= (REMBB.robot_state[robot].own_half>0 ? -1 : +1);
}


void RobotWidget::robotdataChanged( bool b )
{
  if (b)
    data_widget->show();
  else
    data_widget->hide();
}


void RobotWidget::update()
{
  // Kommunikation
  unsigned int num_comm_okay=0;
  for (unsigned int i=1; i<was_comm_okay.size(); i++) {
    was_comm_okay[i-1]=was_comm_okay[i];
    if (was_comm_okay[i])
      num_comm_okay++;
  }
  was_comm_okay[was_comm_okay.size()-1]=REMBB.robot_state[robot].comm_okay || (!REMBB.robot_state[robot].comm_started);
  if (was_comm_okay[was_comm_okay.size()-1])
    num_comm_okay++;
  if (REMBB.robot_state[robot].comm_started!=check_connect->isChecked())
    check_connect->setChecked(REMBB.robot_state[robot].comm_started);
  if (REMBB.robot_state[robot].comm_started) {
    if (num_comm_okay==was_comm_okay.size())
      check_connect->setPaletteBackgroundColor(Qt::green);
    else if (num_comm_okay==0)
      check_connect->setPaletteBackgroundColor(Qt::blue);
    else
      check_connect->setPaletteBackgroundColor(Qt::yellow);
  } else
    check_connect->setPaletteBackgroundColor(Qt::red);

  // Aktivierung
  if (REMBB.robot_state[robot].in_game!=check_activate->isChecked()) {
    check_activate->setChecked(REMBB.robot_state[robot].in_game);
  }
  if (REMBB.robot_state[robot].in_game && REMBB.robot_state[robot].desired_in_game)
    check_activate->setPaletteBackgroundColor(Qt::green);
  else if (!REMBB.robot_state[robot].in_game && !REMBB.robot_state[robot].desired_in_game)
    check_activate->setPaletteBackgroundColor(Qt::red);
  else
    check_activate->setPaletteBackgroundColor(Qt::blue);

  // Roboterdaten
  check_robotdata->setChecked (data_widget->isVisible());

  // Team
  bool teamChanging = (REMBB.robot_state[robot].own_half!=REMBB.robot_state[robot].desired_own_half);
  buttonGroupTeam->setPaletteBackgroundColor (teamChanging ? Qt::blue : this->paletteBackgroundColor());
  if (!teamChanging) {
    if (REMBB.robot_state[robot].own_half==REMBB.team_state.own_half && !radioButtonTeamTribots->isChecked())
      radioButtonTeamTribots->setChecked(true);
    if (REMBB.robot_state[robot].own_half!=REMBB.team_state.own_half && !radioButtonTeamOpponents->isChecked())
      radioButtonTeamOpponents->setChecked(true);
  }

  // Batteriespannung
  std::stringstream inout;
  int mvcc = static_cast<int>(REMBB.robot_state[robot].robot_data.motor_vcc);
  inout << mvcc << std::endl;
  std::string txt;
  std::getline (inout, txt);
  lineEdit_vcc->setText(txt.c_str());
  if (mvcc<20)
    lineEdit_vcc->setPaletteBackgroundColor (Qt::red);
  else if (mvcc<22)
    lineEdit_vcc->setPaletteBackgroundColor (Qt::yellow);
  else
    lineEdit_vcc->setPaletteBackgroundColor (Qt::green);

  // Pingzeiten
  int pg = static_cast<int>(REMBB.robot_state[robot].pingtime);
  if (pg<0)
    inout << "--" << std::endl;
  else
    inout << pg << std::endl;
  std::getline (inout, txt);
  lineEdit_ping->setText(txt.c_str());
  if (pg<0)
    lineEdit_ping->setPaletteBackgroundColor (Qt::red);
  else if (pg<50)
    lineEdit_ping->setPaletteBackgroundColor (Qt::green);
  else if (pg<100)
    lineEdit_ping->setPaletteBackgroundColor (Qt::yellow);
  else
    lineEdit_ping->setPaletteBackgroundColor (Qt::blue);

  // Playertypelist
  bool playertypelist_changed = (static_cast<unsigned int>(combo_playertype->count()) != REMBB.robot_state[robot].list_players.size());
  if (!playertypelist_changed) {
    for (unsigned int i=0; i<REMBB.robot_state[robot].list_players.size(); i++) {
      bool found=false;
      for (unsigned int j=0; j<REMBB.robot_state[robot].list_players.size(); j++)
        if (REMBB.robot_state[robot].list_players[i]==std::string(combo_playertype->text (j).ascii())) {
        found=true;
        break;
      }
      if (!found) {
        playertypelist_changed=true;
        break;
      }
    }
  }
  if (playertypelist_changed) {
    combo_playertype->clear();
    for (unsigned  int i=0; i<REMBB.robot_state[robot].list_players.size(); i++)
      combo_playertype->insertItem(REMBB.robot_state[robot].list_players[i].c_str());
  }

  // Playertype
  if (REMBB.robot_state[robot].playertype!=REMBB.robot_state[robot].desired_playertype) {
    frame_playertype->setPaletteBackgroundColor(Qt::blue);
  } else {
    frame_playertype->setPaletteBackgroundColor(this->paletteBackgroundColor());
  }
  if (playertype!=REMBB.robot_state[robot].playertype || playertypelist_changed || !REMBB.robot_state[robot].comm_started) {
    playertype=REMBB.robot_state[robot].playertype;
    bool playertype_found=false;
    for (int i=0; i<combo_playertype->count(); i++)
      if (std::string(combo_playertype->text(i).ascii())==playertype)
 playertype_found=true;
    if (!playertype_found)
      combo_playertype->insertItem (playertype.c_str());
    combo_playertype->setCurrentText (playertype.c_str());
  }

  // Playerrolelist
  bool playerrolelist_changed = (static_cast<unsigned int>(combo_playerrole->count()) != REMBB.robot_state[robot].list_roles.size());
  if (!playerrolelist_changed) {
    for (unsigned int i=0; i<REMBB.robot_state[robot].list_roles.size(); i++) {
      bool found=false;
      for (unsigned int j=0; j<REMBB.robot_state[robot].list_roles.size(); j++)
      if (REMBB.robot_state[robot].list_roles[i]==std::string(combo_playerrole->text (j).ascii())) {
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
    combo_playerrole->clear();
    for (unsigned  int i=0; i<REMBB.robot_state[robot].list_roles.size(); i++)
      combo_playerrole->insertItem(REMBB.robot_state[robot].list_roles[i].c_str());
  }

  // Playerrole
  if (REMBB.robot_state[robot].playerrole!=REMBB.robot_state[robot].desired_playerrole) {
    frame_playerrole->setPaletteBackgroundColor(Qt::blue);
  } else {
    frame_playerrole->setPaletteBackgroundColor(this->paletteBackgroundColor());
  }
  if (playerrole!=REMBB.robot_state[robot].playerrole || playerrolelist_changed || !REMBB.robot_state[robot].comm_started) {
    playerrole=REMBB.robot_state[robot].playerrole;
    bool playerrole_found=false;
    for (int i=0; i<combo_playerrole->count(); i++)
      if (std::string(combo_playerrole->text(i).ascii())==playerrole)
        playerrole_found=true;
    if (!playerrole_found)
      combo_playerrole->insertItem (playerrole.c_str());
    combo_playerrole->setCurrentText (playerrole.c_str());
  }

  // Refstate
  if (REMBB.robot_state[robot].refstate!=REMBB.robot_state[robot].desired_refstate) {
    frame_refstate->setPaletteBackgroundColor(Qt::blue);
  } else {
    frame_refstate->setPaletteBackgroundColor(this->paletteBackgroundColor());
  }
  if (refstate!=REMBB.robot_state[robot].refstate || !REMBB.robot_state[robot].comm_started) {
    refstate=REMBB.robot_state[robot].refstate;
    combo_refstate->setCurrentText(Tribots::referee_state_names[refstate]);
  }

  // Gelbe Karten
  if (REMBB.robot_state[robot].yellow_cards!=REMBB.robot_state[robot].desired_yellow_cards)
    spinBoxYellowCards->setPaletteBackgroundColor(Qt::blue);
  else
    spinBoxYellowCards->setPaletteBackgroundColor(this->paletteBackgroundColor());
  if (REMBB.robot_state[robot].yellow_cards==REMBB.robot_state[robot].desired_yellow_cards && REMBB.robot_state[robot].yellow_cards!=static_cast<unsigned int>(spinBoxYellowCards->value()))
    spinBoxYellowCards->setValue (REMBB.robot_state[robot].yellow_cards);

  // IP-Adresse
  if (ip!=REMBB.robot_state[robot].ip || port!=REMBB.robot_state[robot].port) {
    ip=REMBB.robot_state[robot].ip;
    port=REMBB.robot_state[robot].port;
    std::stringstream inout;
    inout << ip << ':' << port << '\n';
    std::string iptext;
    std::getline (inout, iptext);
    edit_ip->setText(iptext.c_str());
  }

  // Delokalisierung
  if (REMBB.robot_state[robot].in_game
      && !REMBB.robot_state[robot].robot_pos.valid)
    tabWidget->setPaletteBackgroundColor (Qt::yellow);
  else
    tabWidget->setPaletteBackgroundColor (qApp->palette().color(QPalette::Normal, QColorGroup::Background));

  if (REMBB.robot_state[robot].comm_started && !pushButtonExitRobotcontrol->isEnabled())
    pushButtonExitRobotcontrol->setEnabled(true);
  if (!REMBB.robot_state[robot].comm_started && pushButtonExitRobotcontrol->isEnabled())
    pushButtonExitRobotcontrol->setEnabled(false);

  QWidget::update();
  data_widget->update();
}

void RobotWidget::debugImageClicked()
{
  REMBB.robot_state[robot].debug_image_request=true;
}

void RobotWidget::yellowCardsChanged( int v )
{
  REMBB.robot_state[robot].desired_yellow_cards = v;
}


void RobotWidget::startAsSlave()
{
  check_joystick->setEnabled(false);
}

void RobotWidget::stopAsSlave()
{
  check_joystick->setEnabled(true);
}

void RobotWidget::updateRequests()
{
  REMBB.robot_state[robot].joystick_request = check_joystick->isChecked();
  REMBB.robot_state[robot].robot_data_request = data_widget->isVisible();
}

void RobotWidget::startPressed()
{
  if (REMBB.team_state.localization_side!=0) {
    double fwh = 0.5*REMBB.robot_state[robot].field_geometry.field_width+300;
    REMBB.robot_state[robot].slhint_request=true;
    REMBB.robot_state[robot].desired_slhint_and_activate=true;
    if (REMBB.team_state.localization_side<0) {
      REMBB.robot_state[robot].slhint_pos=-Tribots::Vec(fwh,0);
      REMBB.robot_state[robot].slhint_angle=Tribots::Angle::quarter;
    } else {
      REMBB.robot_state[robot].slhint_pos=Tribots::Vec(fwh,0);
      REMBB.robot_state[robot].slhint_angle=Tribots::Angle::three_quarters;
    }
  } else {
    REMBB.robot_state[robot].desired_in_game = true;
  }
}


void RobotWidget::teamClicked()
{
  if (radioButtonTeamTribots->isChecked())
     REMBB.robot_state[robot].desired_own_half=REMBB.team_state.own_half;
  if (radioButtonTeamOpponents->isChecked())
     REMBB.robot_state[robot].desired_own_half=-REMBB.team_state.own_half;
}


void RobotWidget::startRobotcontrol()
{
  killChild();
  int pid = fork ();
  if (pid==0) {
    std::stringstream inout;
    inout << "tribot@" << REMBB.robot_state[robot].ip << '\n';
    std::string login;
    std::getline (inout, login);
    execlp ("xterm", "xterm", "-hold", "-sb", "-e", "ssh", "-t", login.c_str(), "~/.robotcontrol/competitiondir/start.sh", NULL);
  } else if (pid>0) {
    robotcontrol_child_pid=pid;
  } else {
    std::cerr << "Fehler beim Aufruf von fork()\n";
  }
}

void RobotWidget::killChild()
{
  if (robotcontrol_child_pid>0) {
    std::stringstream inout;
    inout << "kill " << robotcontrol_child_pid << '\n';
    std::string killcommand;
    std::getline (inout, killcommand);
    system (killcommand.c_str());
  }
}


void RobotWidget::exitRobotcontrol()
{
  REMBB.robot_state[robot].exit_request=true;
}
