
#include "RefboxWidget.h"
#include "../States/RemoteBlackboard.h"
#include "../../../../Fundamental/stringconvert.h"

#define PRINT(a) // std::cerr << a << '\n';

using namespace TribotsTools;

RefboxWidget::RefboxWidget (QWidget* p, Qt::WindowFlags f) : QWidget (p,f)
{
  setupUi(this);
  testStateComboBox->addItem("Test1");
  testStateComboBox->addItem("Test2");
  testStateComboBox->addItem("Test3");
  testStateComboBox->addItem("Test4");
  testStateComboBox->addItem("Test5");
  testStateComboBox->addItem("Test6");
  testStateComboBox->addItem("Test7");
  testStateComboBox->addItem("Test8");

  connect (spinBoxOwnScore,SIGNAL(valueChanged(int)),this,SLOT(ownScoreChanged(int)));
  connect (DroppedBallButton,SIGNAL(clicked()),this,SLOT(droppedBallPressed()));
  connect (check_refboxconnect,SIGNAL(toggled(bool)),this,SLOT(connectRefboxChanged(bool)));
  connect (edit_refboxip,SIGNAL(returnPressed()),this,SLOT(refboxipChanged()));
  connect (ChangeGoalColorButton,SIGNAL(clicked()),this,SLOT(changeOwnHalfPressed()));
  connect (ChangeLabelColorButton,SIGNAL(clicked()),this,SLOT(changeLabelPressed()));
  connect (GoButton,SIGNAL(clicked()),this,SLOT(readyPressed()));
  connect (StartButton,SIGNAL(clicked()),this,SLOT(startPressed()));
  connect (StopButton,SIGNAL(clicked()),this,SLOT(stopPressed()));
  connect (ThrowInOwnButton,SIGNAL(clicked()),this,SLOT(throwInOwnPressed()));
  connect (ThrowInOpponentButton,SIGNAL(clicked()),this,SLOT(throwInOpponentPressed()));
  connect (PenaltyOwnButton,SIGNAL(clicked()),this,SLOT(penaltyKickOwnPressed()));
  connect (PenaltyOpponentButton,SIGNAL(clicked()),this,SLOT(penaltyKickOpponentPressed()));
  connect (GoalKickOwnButton,SIGNAL(clicked()),this,SLOT(goalKickOwnPressed()));
  connect (GoalKickOpponentButton,SIGNAL(clicked()),this,SLOT(goalKickOpponentPressed()));
  connect (FreeKickOwnButton,SIGNAL(clicked()),this,SLOT(freeKickOwnPressed()));
  connect (FreeKickOpponentButton,SIGNAL(clicked()),this,SLOT(freeKickOpponentPressed()));
  connect (CornerKickOwnButton,SIGNAL(clicked()),this,SLOT(cornerKickOwnPressed()));
  connect (CornerKickOpponentButton,SIGNAL(clicked()),this,SLOT(cornerKickOpponentPressed()));
  connect (KickOffOwnButton,SIGNAL(clicked()),this,SLOT(kickOffOwnPressed()));
  connect (KickOffOpponentButton,SIGNAL(clicked()),this,SLOT(kickOffOpponentPressed()));
  connect (spinBoxOpponentScore,SIGNAL(valueChanged(int)),this,SLOT(opponentScoreChanged(int)));
  connect (testStateComboBox,SIGNAL(activated(const QString&)),this,SLOT(testStatePressed(const QString&)));

  std::stringstream inout;
  inout << REMBB.team_state.refbox_port << '\n';
  std::string port_string;
  std::getline (inout, port_string);
  edit_refboxip->setText((REMBB.team_state.refbox_ip+std::string(":")+port_string).c_str());
  update();
}

void RefboxWidget::changeOwnHalfPressed() 
{
  REMBB.team_state.own_half = (REMBB.team_state.own_half>0 ? -1 : +1);
  PRINT("CHOH");
}

void RefboxWidget::changeLabelPressed() 
{
  REMBB.team_state.team_color = (REMBB.team_state.team_color>0 ? -1 : +1);
  PRINT("CHLB");
}

void RefboxWidget::stopPressed() 
{
  REMBB.team_state.refbox_signal = Tribots::SIGstop;
  PRINT("STOP");
}

void RefboxWidget::readyPressed() 
{
  REMBB.team_state.refbox_signal = Tribots::SIGready;
  PRINT("RDY");
}

void RefboxWidget::startPressed() 
{
  REMBB.team_state.refbox_signal = Tribots::SIGstart;
  PRINT("START");
}

void RefboxWidget::droppedBallPressed() 
{
  REMBB.team_state.refbox_signal = Tribots::SIGdroppedBall;
  PRINT("DROP");
}

void RefboxWidget::kickOffOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanKickOff : Tribots::SIGmagentaKickOff;
  PRINT("KO1");
}

void RefboxWidget::throwInOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanThrowIn : Tribots::SIGmagentaThrowIn;
  PRINT("TI1");
}

void RefboxWidget::goalKickOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanGoalKick : Tribots::SIGmagentaGoalKick;
  PRINT("GK1");
}

void RefboxWidget::cornerKickOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanCornerKick : Tribots::SIGmagentaCornerKick;
  PRINT("CK1");
}

void RefboxWidget::freeKickOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanFreeKick : Tribots::SIGmagentaFreeKick;
  PRINT("FK1");
}

void RefboxWidget::penaltyKickOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanPenalty : Tribots::SIGmagentaPenalty;
  PRINT("PK1");
}

void RefboxWidget::kickOffOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanKickOff : Tribots::SIGmagentaKickOff;
  PRINT("KO2");
}

void RefboxWidget::throwInOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanThrowIn : Tribots::SIGmagentaThrowIn;
  PRINT("TI2");
}

void RefboxWidget::goalKickOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanGoalKick : Tribots::SIGmagentaGoalKick;
  PRINT("GK2");
}

void RefboxWidget::cornerKickOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanCornerKick : Tribots::SIGmagentaCornerKick;
  PRINT("CK2");
}

void RefboxWidget::freeKickOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanFreeKick : Tribots::SIGmagentaFreeKick;
  PRINT("FK2");
}

void RefboxWidget::penaltyKickOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanPenalty : Tribots::SIGmagentaPenalty;
  PRINT("PK2");
}


void RefboxWidget::connectRefboxChanged( bool b )
{
  REMBB.team_state.refbox_connected = b;
  PRINT("REFB");
}


void RefboxWidget::refboxipChanged()
{
  std::string ipport (edit_refboxip->text().toAscii());
  unsigned int dp = ipport.find (':',0);
  std::string ip = ipport.substr (0, dp);
  std::string port_string = ipport.substr (dp+1, ipport.length()-dp);
  if (ip.length()>0)
    REMBB.team_state.refbox_ip = ip;
  if (port_string.length()>0)
    Tribots::string2int (REMBB.team_state.refbox_port, port_string);
  PRINT("REFIP");
}

void RefboxWidget::update () {
  PRINT ("UPDATE");
  edit_refstate->setText(Tribots::referee_state_names [REMBB.team_state.refstate]);
  QPalette qpal_cyan;
  QPalette qpal_magenta;
  QPalette qpal_yellow;
  QPalette qpal_blue;
  QPalette qpalb_red;
  QPalette qpalb_green;
  QPalette qpalb_blue;
  qpal_cyan.setBrush (QPalette::Base, Qt::cyan);
  qpal_magenta.setBrush (QPalette::Base, Qt::magenta);
  qpal_yellow.setBrush (QPalette::Base, Qt::yellow);
  qpal_blue.setBrush (QPalette::Base, Qt::blue);
  qpalb_blue.setBrush (QPalette::Button, Qt::blue);
  qpalb_red.setBrush (QPalette::Button, Qt::red);
  qpalb_green.setBrush (QPalette::Button, Qt::green);

  if (REMBB.team_state.team_color>0) {
    edit_tribotcolor->setPalette(qpal_cyan);
    edit_tribotcolor->setText("cyan");
    edit_opponentcolor->setPalette(qpal_magenta);
    edit_opponentcolor->setText("magenta");
  } else {
    edit_tribotcolor->setPalette(qpal_magenta);
    edit_tribotcolor->setText("magenta");
    edit_opponentcolor->setPalette(qpal_cyan);
    edit_opponentcolor->setText("cyan");
  }
  if (REMBB.team_state.own_half>0) {
    edit_tribotownhalf->setPalette(qpal_yellow);
    edit_tribotownhalf->setText("gelb");
    edit_opponentownhalf->setPalette(qpal_blue);
    edit_opponentownhalf->setText("blau");
  } else {
    edit_tribotownhalf->setPalette(qpal_blue);
    edit_tribotownhalf->setText("blau");
    edit_opponentownhalf->setPalette(qpal_yellow);
    edit_opponentownhalf->setText("gelb");
  }
  check_refboxconnect->setChecked(REMBB.team_state.refbox_connected);
  if (REMBB.team_state.refbox_connected) {
    if (REMBB.team_state.refbox_okay) {
      check_refboxconnect->setPalette(qpalb_green);
    } else {
      check_refboxconnect->setPalette(qpalb_blue);
    }
  } else
    check_refboxconnect->setPalette(qpalb_red);
  if (REMBB.team_state.own_score!=static_cast<unsigned int>(spinBoxOwnScore->value()))
    spinBoxOwnScore->setValue(REMBB.team_state.own_score);
  if (REMBB.team_state.opponent_score!=static_cast<unsigned int>(spinBoxOpponentScore->value()))
    spinBoxOpponentScore->setValue(REMBB.team_state.opponent_score);

  QWidget::update();
}

void RefboxWidget::ownScoreChanged (int v)
{
  REMBB.team_state.own_score=v;
  PRINT("CHSC1");
}

void RefboxWidget::opponentScoreChanged(int v)
{
  REMBB.team_state.opponent_score=v;
  PRINT("CHSC2");
}


void RefboxWidget::startAsSlave()
{
  check_refboxconnect->setEnabled(false);
  edit_refboxip->setEnabled(false);
  PRINT("SLAVESTART");
}

void RefboxWidget::stopAsSlave()
{
  check_refboxconnect->setEnabled(true);
  edit_refboxip->setEnabled(true);
  PRINT("SLAVESTOP");
}

void RefboxWidget::testStatePressed(const QString& s) {
  std::cerr << "TESTSTATE-FUNKTION NOCH NICHT IMPLEMENTIERT: " << std::string(s.toAscii()) << "\n";
/*  if (s=="Test1") REMBB.team_state.refbox_signal = Tribots::SIGtest1;
  if (s=="Test2") REMBB.team_state.refbox_signal = Tribots::SIGtest2;
  if (s=="Test3") REMBB.team_state.refbox_signal = Tribots::SIGtest3;
  if (s=="Test4") REMBB.team_state.refbox_signal = Tribots::SIGtest4;
  if (s=="Test5") REMBB.team_state.refbox_signal = Tribots::SIGtest5;
  if (s=="Test6") REMBB.team_state.refbox_signal = Tribots::SIGtest6;
  if (s=="Test7") REMBB.team_state.refbox_signal = Tribots::SIGtest7;
  if (s=="Test8") REMBB.team_state.refbox_signal = Tribots::SIGtest8; */
}
