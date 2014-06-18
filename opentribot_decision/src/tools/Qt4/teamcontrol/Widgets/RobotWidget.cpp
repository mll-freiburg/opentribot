
#include "RobotWidget.h"
#include "../../../../Fundamental/stringconvert.h"
#include "../States/RemoteBlackboard.h"
#include <sstream>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

#define PRINT(a) // cerr << a << '\n';

namespace {
  void set_combo_text (QComboBox& cb, const char* s) {
    int index= cb.findText (s);
    if (index<0) {
      cb.addItem (s);
      index=cb.findText (s);
    }
    cb.setCurrentIndex (index);
  }
}

RobotWidget::RobotWidget(QWidget* p, Qt::WindowFlags f) : QWidget (p,f)
{
  setupUi(this);

  connect (check_connect,SIGNAL(released()),this,SLOT(connectChanged()));
  connect (check_activate,SIGNAL(released()),this,SLOT(activateChanged()));
  connect (check_robotdata,SIGNAL(toggled(bool)),this,SLOT(robotdataChanged(bool)));
  connect (button_team,SIGNAL(released()),this,SLOT(teamChanged()));
  connect (combo_playerrole,SIGNAL(activated(int)),this,SLOT(playerroleChanged(int)));
  connect (combo_playertype,SIGNAL(activated(int)),this,SLOT(playertypeChanged(int)));
  connect (combo_refstate,SIGNAL(activated(int)),this,SLOT(refstateChanged(int)));
  connect (edit_ip,SIGNAL(returnPressed()),this,SLOT(ipChanged()));
  connect (check_messageboard,SIGNAL(released()),this,SLOT(messageBoardChanged()));
  connect (button_image,SIGNAL(clicked()),this,SLOT(debugImageClicked()));
  connect (spinBoxYellowCards,SIGNAL(valueChanged(int)),this,SLOT(yellowCardsChanged(int)));
  connect (button_start,SIGNAL(clicked()),this,SLOT(startPressed()));

  data_widget=new RobotDataWidget;
  playerrole = "---";
  playertype = "---";
  ip="";
  port=-1;
  refstate = Tribots::RefereeState (0);
  combo_refstate->clear();
  for (int i=0; i<Tribots::num_referee_states; i++)
    combo_refstate->addItem (Tribots::referee_state_names[i]);
  was_comm_okay.resize (10);
  for (unsigned int i=0; i<was_comm_okay.size(); i++)
    was_comm_okay[i]=true;
  localization_side = 0;
}


RobotWidget::~RobotWidget()
{
  delete data_widget;
}


void RobotWidget::init( unsigned int n, const Tribots::ConfigReader& cfg )
{
  robot=n;
  setWindowTitle(REMBB.robot_state[n].name.c_str());
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
  update ();
  std::string wrd="";
  cfg.get ("lokalisierung_seite", wrd);
  if (wrd=="links")
    localization_side = -1;
  else if (wrd=="rechts")
    localization_side = +1;
  else
    localization_side = 0;
}


void RobotWidget::hide()
{
  REMBB.robot_state[robot].in_game=false;
  REMBB.robot_state[robot].desired_connect=false;
  data_widget->hide();
  QWidget::hide();
  PRINT("HIDE");
}


void RobotWidget::connectChanged()
{
  REMBB.robot_state[robot].desired_connect = check_connect->isChecked();
  PRINT("CONNECT");
}


void RobotWidget::activateChanged()
{
  REMBB.robot_state[robot].desired_in_game = check_activate->isChecked();
  PRINT("ACTIV");
}


void RobotWidget::messageBoardChanged()
{
  REMBB.robot_state[robot].show_message_board = check_messageboard->isChecked();
  PRINT("MBOARD");
}


void RobotWidget::playertypeChanged( int )
{
  REMBB.robot_state[robot].desired_playertype = std::string(combo_playertype->currentText().toAscii());
  PRINT("PLTY");
}


void RobotWidget::playerroleChanged( int )
{
  REMBB.robot_state[robot].desired_playerrole = std::string(combo_playerrole->currentText().toAscii());
  PRINT("PLRL");
}


void RobotWidget::refstateChanged( int i )
{
  REMBB.robot_state[robot].desired_refstate = Tribots::RefereeState (i);
  PRINT("REFST");
}


void RobotWidget::ipChanged()
{
  std::string ipport = std::string(edit_ip->text().toAscii());
  unsigned int dp = ipport.find (':',0);
  std::string ip = ipport.substr (0, dp);
  std::string port_string = ipport.substr (dp+1, ipport.length()-dp);
  if (ip.length()>0)
    REMBB.robot_state[robot].ip = ip;
  if (port_string.length()>0)
    Tribots::string2int (REMBB.robot_state[robot].port, port_string);
  PRINT("IP");
}


void RobotWidget::teamChanged()
{
  REMBB.robot_state[robot].desired_own_half= (REMBB.robot_state[robot].own_half>0 ? -1 : +1);
  PRINT("TEAM");
}


void RobotWidget::robotdataChanged( bool b )
{
  if (b)
    data_widget->show();
  else
    data_widget->hide();
  PRINT("RDATA");
}


void RobotWidget::update ()
{
  PRINT("UPDATE");
  QPalette qpal_cyan;
  QPalette qpal_magenta;
  QPalette qpal_yellow;
  QPalette qpal_blue;
  QPalette qpal_red;
  QPalette qpal_green;
  QPalette qpal_white;
  QPalette qpalb_yellow;
  QPalette qpalb_blue;
  QPalette qpalb_red;
  QPalette qpalb_green;
  QPalette qpalk_yellow;
  QPalette qpalk_blue;
  QPalette qpalk_red;
  QPalette qpalk_green;
  qpal_cyan.setBrush (QPalette::Base, Qt::cyan);
  qpal_magenta.setBrush (QPalette::Base, Qt::magenta);
  qpal_yellow.setBrush (QPalette::Base, Qt::yellow);
  qpal_blue.setBrush (QPalette::Base, Qt::blue);
  qpal_red.setBrush (QPalette::Base, Qt::red);
  qpal_green.setBrush (QPalette::Base, Qt::green);
  qpal_white.setBrush (QPalette::Base, Qt::white);
  qpalb_yellow.setBrush (QPalette::Button, Qt::yellow);
  qpalb_blue.setBrush (QPalette::Button, Qt::blue);
  qpalb_red.setBrush (QPalette::Button, Qt::red);
  qpalb_green.setBrush (QPalette::Button, Qt::green);
  qpalk_yellow.setBrush (QPalette::Background, Qt::yellow);
  qpalk_blue.setBrush (QPalette::Background, Qt::blue);
  qpalk_red.setBrush (QPalette::Background, Qt::red);
  qpalk_green.setBrush (QPalette::Background, Qt::green);

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
      check_connect->setPalette(qpalb_green);
    else if (num_comm_okay==0)
      check_connect->setPalette(qpalb_blue);
    else
      check_connect->setPalette(qpalb_yellow);
  } else
    check_connect->setPalette(qpalb_red);

  // Aktivierung
  if (REMBB.robot_state[robot].in_game!=check_activate->isChecked()) {
    check_activate->setChecked(REMBB.robot_state[robot].in_game);
  }
  if (REMBB.robot_state[robot].in_game && REMBB.robot_state[robot].desired_in_game)
    check_activate->setPalette(qpalb_green);
  else if (!REMBB.robot_state[robot].in_game && !REMBB.robot_state[robot].desired_in_game)
    check_activate->setPalette(qpalb_red);
  else
    check_activate->setPalette(qpalb_blue);

  // Roboterdaten
  check_robotdata->setChecked (data_widget->isVisible());

  // Team
  if (REMBB.robot_state[robot].own_half>0) {
    edit_own_half->setText ("gelb");
    if (REMBB.robot_state[robot].own_half==REMBB.robot_state[robot].desired_own_half)
      edit_own_half->setPalette(qpal_yellow);
    else
      edit_own_half->setPalette(qpal_white);
  } else {
    edit_own_half->setText ("blau");
    if (REMBB.robot_state[robot].own_half==REMBB.robot_state[robot].desired_own_half)
      edit_own_half->setPalette(qpal_blue);
    else
      edit_own_half->setPalette(qpal_white);
  }
  if (REMBB.robot_state[robot].own_half*REMBB.team_state.own_half*REMBB.team_state.team_color>0) {
    edit_colorlabel->setText("cyan");
    if (REMBB.robot_state[robot].own_half==REMBB.robot_state[robot].desired_own_half)
      edit_colorlabel->setPalette(qpal_cyan);
    else
      edit_own_half->setPalette(qpal_white);
  } else {
    edit_colorlabel->setText("mag.");
    if (REMBB.robot_state[robot].own_half==REMBB.robot_state[robot].desired_own_half)
      edit_colorlabel->setPalette(qpal_magenta);
    else
      edit_own_half->setPalette(qpal_white);
  }

  // Batteriespannung
  int mvcc = static_cast<int>(REMBB.robot_state[robot].robot_data.motor_vcc);
  lcd_vcc->display(mvcc);
  if (mvcc<20)
    lcd_vcc->setPalette(qpalk_red);
  else if (mvcc<22)
    lcd_vcc->setPalette(qpalk_yellow);
  else
    lcd_vcc->setPalette(qpalk_green);

  // Playertypelist
  bool playertypelist_changed = (static_cast<unsigned int>(combo_playertype->count()) != REMBB.robot_state[robot].list_players.size());
  if (!playertypelist_changed) {
    for (unsigned int i=0; i<REMBB.robot_state[robot].list_players.size(); i++) {
      bool found=false;
      for (unsigned int j=0; j<REMBB.robot_state[robot].list_players.size(); j++)
 if (REMBB.robot_state[robot].list_players[i]==std::string(combo_playertype->itemText (j).toAscii())) {
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
      combo_playertype->addItem(REMBB.robot_state[robot].list_players[i].c_str());
  }

  // Playertype
  if (REMBB.robot_state[robot].playertype!=REMBB.robot_state[robot].desired_playertype) {
    frame_playertype->setPalette(qpalk_blue);
  } else {
    frame_playertype->setPalette(this->palette());
  }
  if (playertype!=REMBB.robot_state[robot].playertype || playertypelist_changed || !REMBB.robot_state[robot].comm_started) {
    playertype=REMBB.robot_state[robot].playertype;
    bool playertype_found=false;
    for (int i=0; i<combo_playertype->count(); i++)
      if (std::string(combo_playertype->itemText(i).toAscii())==playertype)
 playertype_found=true;
    if (!playertype_found)
      combo_playertype->addItem (playertype.c_str());
    set_combo_text (*combo_playertype, playertype.c_str());
  }

  // Playerrolelist
  bool playerrolelist_changed = (static_cast<unsigned int>(combo_playerrole->count()) != REMBB.robot_state[robot].list_roles.size());
  if (!playerrolelist_changed) {
    for (unsigned int i=0; i<REMBB.robot_state[robot].list_roles.size(); i++) {
      bool found=false;
      for (unsigned int j=0; j<REMBB.robot_state[robot].list_roles.size(); j++)
 if (REMBB.robot_state[robot].list_roles[i]==std::string(combo_playerrole->itemText (j).toAscii())) {
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
      combo_playerrole->addItem(REMBB.robot_state[robot].list_roles[i].c_str());
  }

  // Playerrole
  if (REMBB.robot_state[robot].playerrole!=REMBB.robot_state[robot].desired_playerrole) {
    frame_playerrole->setPalette(qpalk_blue);
  } else {
    frame_playerrole->setPalette(this->palette());
  }
  if (playerrole!=REMBB.robot_state[robot].playerrole || playerrolelist_changed || !REMBB.robot_state[robot].comm_started) {
    playerrole=REMBB.robot_state[robot].playerrole;
    bool playerrole_found=false;
    for (int i=0; i<combo_playerrole->count(); i++)
      if (std::string(combo_playerrole->itemText(i).toAscii())==playerrole)
 playerrole_found=true;
    if (!playerrole_found)
      combo_playerrole->addItem (playerrole.c_str());
    set_combo_text (*combo_playerrole, playerrole.c_str());
  }

  // Refstate
  if (REMBB.robot_state[robot].refstate!=REMBB.robot_state[robot].desired_refstate) {
    frame_refstate->setPalette(qpalk_blue);
  } else {
    frame_refstate->setPalette(this->palette());
  }
  if (refstate!=REMBB.robot_state[robot].refstate || !REMBB.robot_state[robot].comm_started) {
    refstate=REMBB.robot_state[robot].refstate;
    set_combo_text (*combo_refstate, Tribots::referee_state_names[refstate]);
  }

  // Gelbe Karten
  if (REMBB.robot_state[robot].yellow_cards!=REMBB.robot_state[robot].desired_yellow_cards)
    spinBoxYellowCards->setPalette(qpal_blue);
  else
    spinBoxYellowCards->setPalette(this->palette());
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

  data_widget->update();
  QWidget::update();
}

void RobotWidget::debugImageClicked()
{
  REMBB.robot_state[robot].debug_image_request=true;
  PRINT("DEBIMG");
}

void RobotWidget::yellowCardsChanged( int v )
{
  REMBB.robot_state[robot].desired_yellow_cards = v;
  PRINT("YELCRD");
}


void RobotWidget::startAsSlave()
{
  check_joystick->setEnabled(false);
  PRINT("STARTSLAVE");
}

void RobotWidget::stopAsSlave()
{
  check_joystick->setEnabled(true);
  PRINT("STOPSLAVE");
}

void RobotWidget::updateRequests()
{
  REMBB.robot_state[robot].joystick_request = check_joystick->isChecked();
  REMBB.robot_state[robot].robot_data_request = data_widget->isVisible();
  PRINT("REQ");
}

void RobotWidget::startPressed()
{
  REMBB.robot_state[robot].desired_in_game = true;
  if (localization_side!=0) {
    double fwh = 0.5*REMBB.robot_state[robot].field_geometry.field_width+300;
    REMBB.robot_state[robot].slhint_request=true;
    if (localization_side<0) {
      REMBB.robot_state[robot].slhint_pos=-Tribots::Vec(fwh,0);
      REMBB.robot_state[robot].slhint_angle=Tribots::Angle::quarter;
    } else {
      REMBB.robot_state[robot].slhint_pos=Tribots::Vec(fwh,0);
      REMBB.robot_state[robot].slhint_angle=Tribots::Angle::three_quarters;
    }
  }
  PRINT("START");
}
