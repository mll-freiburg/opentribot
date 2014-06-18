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


void RefboxWidget::init() 
{
  std::stringstream inout;
  inout << REMBB.team_state.refbox_port << '\n';
  std::string port_string;
  std::getline (inout, port_string);
  edit_refboxip->setText((REMBB.team_state.refbox_ip+std::string(":")+port_string).c_str());
  update();
}

void RefboxWidget::changeOwnHalfPressed() 
{
    // nicht mehr verwendeter Slot
  REMBB.team_state.own_half = (REMBB.team_state.own_half>0 ? -1 : +1);
}

void RefboxWidget::changeLabelPressed() 
{
  // nicht mehr verwendeter Slot
  REMBB.team_state.team_color = (REMBB.team_state.team_color>0 ? -1 : +1);
}

void RefboxWidget::stopPressed() 
{
  REMBB.team_state.refbox_signal = Tribots::SIGstop;
}

void RefboxWidget::readyPressed() 
{
  REMBB.team_state.refbox_signal = Tribots::SIGready;
}

void RefboxWidget::startPressed() 
{
  REMBB.team_state.refbox_signal = Tribots::SIGstart;
}

void RefboxWidget::droppedBallPressed() 
{
  REMBB.team_state.refbox_signal = Tribots::SIGdroppedBall;
}

void RefboxWidget::kickOffOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanKickOff : Tribots::SIGmagentaKickOff;
}

void RefboxWidget::throwInOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanThrowIn : Tribots::SIGmagentaThrowIn;
}

void RefboxWidget::goalKickOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanGoalKick : Tribots::SIGmagentaGoalKick;
}

void RefboxWidget::cornerKickOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanCornerKick : Tribots::SIGmagentaCornerKick;
}

void RefboxWidget::freeKickOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanFreeKick : Tribots::SIGmagentaFreeKick;
}

void RefboxWidget::penaltyKickOwnPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color>0 ? Tribots::SIGcyanPenalty : Tribots::SIGmagentaPenalty;
}

void RefboxWidget::kickOffOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanKickOff : Tribots::SIGmagentaKickOff;
}

void RefboxWidget::throwInOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanThrowIn : Tribots::SIGmagentaThrowIn;
}

void RefboxWidget::goalKickOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanGoalKick : Tribots::SIGmagentaGoalKick;
}

void RefboxWidget::cornerKickOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanCornerKick : Tribots::SIGmagentaCornerKick;
}

void RefboxWidget::freeKickOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanFreeKick : Tribots::SIGmagentaFreeKick;
}

void RefboxWidget::penaltyKickOpponentPressed() 
{
  REMBB.team_state.refbox_signal = REMBB.team_state.team_color<0 ? Tribots::SIGcyanPenalty : Tribots::SIGmagentaPenalty;
}

void RefboxWidget::testActivated( const QString & s)
{
  Tribots::RefboxSignal  sig = REMBB.team_state.refbox_signal;
  if (s=="Test1") sig=Tribots::SIGtest1;
  if (s=="Test2") sig=Tribots::SIGtest2;
  if (s=="Test3") sig=Tribots::SIGtest3;
  if (s=="Test4") sig=Tribots::SIGtest4;
  if (s=="Test5") sig=Tribots::SIGtest5;
  if (s=="Test6") sig=Tribots::SIGtest6;
  if (s=="Test7") sig=Tribots::SIGtest7;
  if (s=="Test8") sig=Tribots::SIGtest8;
  REMBB.team_state.refbox_signal = sig;
}


void RefboxWidget::connectRefboxChanged( bool b )
{
  REMBB.team_state.refbox_connected = b;
}


void RefboxWidget::refboxipChanged()
{
  std::string ipport = edit_refboxip->text().ascii();
  unsigned int dp = ipport.find (':',0);
  std::string ip = ipport.substr (0, dp);
  std::string port_string = ipport.substr (dp+1, ipport.length()-dp);
  if (ip.length()>0)
    REMBB.team_state.refbox_ip = ip;
  if (port_string.length()>0)
    Tribots::string2int (REMBB.team_state.refbox_port, port_string);
}


void RefboxWidget::update()
{
  edit_refstate->setText(Tribots::referee_state_names [REMBB.team_state.refstate]);
  if (REMBB.team_state.team_color>0 && !radioButtonCyan->isChecked())
    radioButtonCyan->setChecked(true);
  if (REMBB.team_state.team_color<0 && !radioButtonMagenta->isChecked())
    radioButtonMagenta->setChecked(true);
  if (REMBB.team_state.localization_side>0 && !radioButtonSetInSideRight->isChecked()) {
    radioButtonSetInSideRight->setChecked(true);
  }
  if (REMBB.team_state.localization_side<0 && !radioButtonSetInSideLeft->isChecked()) {
    radioButtonSetInSideLeft->setChecked(true);
  }

  check_refboxconnect->setChecked(REMBB.team_state.refbox_connected);
  if (REMBB.team_state.refbox_connected) {
    if (REMBB.team_state.refbox_okay)
      check_refboxconnect->setPaletteBackgroundColor(Qt::green);
    else
      check_refboxconnect->setPaletteBackgroundColor(Qt::blue);
  } else
    check_refboxconnect->setPaletteBackgroundColor(Qt::red);    
  if (REMBB.team_state.own_score!=static_cast<unsigned int>(spinBoxOwnScore->value()))
    spinBoxOwnScore->setValue(REMBB.team_state.own_score);
  if (REMBB.team_state.opponent_score!=static_cast<unsigned int>(spinBoxOpponentScore->value()))
    spinBoxOpponentScore->setValue(REMBB.team_state.opponent_score);

  QWidget::update();
}

void RefboxWidget::ownScoreChanged (int v)
{
  REMBB.team_state.own_score=v;
}

void RefboxWidget::opponentScoreChanged(int v)
{
  REMBB.team_state.opponent_score=v;
}


void RefboxWidget::startAsSlave()
{
  check_refboxconnect->setEnabled(false);
  edit_refboxip->setEnabled(false);
}

void RefboxWidget::stopAsSlave()
{
  check_refboxconnect->setEnabled(true);
  edit_refboxip->setEnabled(true);
}


void RefboxWidget::setInSideChanged()
{
  if (radioButtonSetInSideLeft->isChecked())
    REMBB.team_state.localization_side=-1;
  else if (radioButtonSetInSideRight->isChecked())
    REMBB.team_state.localization_side=+1;
  else
    REMBB.team_state.localization_side=0;
}


void RefboxWidget::teamColorChanged()
{
  if (radioButtonMagenta->isChecked())
    REMBB.team_state.team_color=-1;
  else
    REMBB.team_state.team_color=+1;
}
