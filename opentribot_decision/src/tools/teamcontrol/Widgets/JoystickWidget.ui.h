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

namespace {
  const unsigned int num_perspectives = 9;
  const char* perspective_name [] = {
    "robozentrisch",
    "hinter gelbem Tor",
    "hinter blauem Tor",
    "linke Seitenlinie",
    "rechte Seitenlinie",
    "links hinten",
    "rechts hinten",
    "links vorne",
    "rechts vorne"
  };
  const int perspective_angle [] = {
    -1,
    0,
    180,
    270,
    90,
    315,
    45,
    225,
    135
  };
}


void JoystickWidget::init()
{
  comboBoxJoyDev->setCurrentText(REMBB.joystick_state.device_name.c_str());
  spinBoxKickButtonIdx->setValue(REMBB.joystick_state.kick_button+1);
  spinBoxKickLength->setValue(REMBB.joystick_state.kick_length);
  spinBoxXAxisIdx->setValue(REMBB.joystick_state.x_axis);
  spinBoxYAxisIdx->setValue(REMBB.joystick_state.y_axis);
  spinBoxPhiAxisIdx->setValue(REMBB.joystick_state.phi_axis);
  spinBoxXAxisMaxV->setValue(static_cast<int>(10*REMBB.joystick_state.max_vx));
  spinBoxYAxisMaxV->setValue(static_cast<int>(10*REMBB.joystick_state.max_vy));
  spinBoxPhiAxisMaxV->setValue(static_cast<int>(10*REMBB.joystick_state.max_vphi));
  spinBoxXAxisNull->setValue(static_cast<int>(100*REMBB.joystick_state.null_x));
  spinBoxYAxisNull->setValue(static_cast<int>(100*REMBB.joystick_state.null_y));
  spinBoxPhiAxisNull->setValue(static_cast<int>(100*REMBB.joystick_state.null_phi));
  checkBoxXInvert->setChecked(REMBB.joystick_state.sign_x<0);
  checkBoxYInvert->setChecked(REMBB.joystick_state.sign_y<0);
  checkBoxPhiInvert->setChecked(REMBB.joystick_state.sign_phi<0);
  spinBoxActivateButtonIdx->setValue(REMBB.joystick_state.activate_button+1);
  spinBoxDeactivateButtonIdx->setValue(REMBB.joystick_state.deactivate_button+1);
  for (unsigned int i=0; i<num_perspectives; i++)
    comboBoxPerspective->insertItem (QString(perspective_name[i]));
  comboBoxPerspective->setCurrentText (QString(perspective_name[0]));
}

void JoystickWidget::destroy()
{
  // leer, da keine dynamische Objekte erzeugt wurden  
}

void JoystickWidget::deviceChanged( int )
{
  REMBB.joystick_state.device_name=comboBoxJoyDev->currentText().ascii();
  mouseJoyWidget1->activate(REMBB.joystick_state.device_name=="Maus");
}

void JoystickWidget::valueChanged( int )
{
  REMBB.joystick_state.kick_button=spinBoxKickButtonIdx->value()-1;
  REMBB.joystick_state.kick_length=spinBoxKickLength->value();
  REMBB.joystick_state.x_axis=spinBoxXAxisIdx->value();
  REMBB.joystick_state.y_axis=spinBoxYAxisIdx->value();
  REMBB.joystick_state.phi_axis=spinBoxPhiAxisIdx->value();
  REMBB.joystick_state.max_vx=0.1*spinBoxXAxisMaxV->value();
  REMBB.joystick_state.max_vy=0.1*spinBoxYAxisMaxV->value();
  REMBB.joystick_state.max_vphi=0.1*spinBoxPhiAxisMaxV->value();
  REMBB.joystick_state.null_x=0.01*spinBoxXAxisNull->value();
  REMBB.joystick_state.null_y=0.01*spinBoxYAxisNull->value();
  REMBB.joystick_state.null_phi=0.01*spinBoxPhiAxisNull->value();
  REMBB.joystick_state.activate_button=spinBoxActivateButtonIdx->value()-1;
  REMBB.joystick_state.deactivate_button=spinBoxDeactivateButtonIdx->value()-1;
}

void JoystickWidget::valueChanged( bool )
{
  REMBB.joystick_state.sign_x=checkBoxXInvert->isChecked() ? -1 : +1;
  REMBB.joystick_state.sign_y=checkBoxYInvert->isChecked() ? -1 : +1;
  REMBB.joystick_state.sign_phi=checkBoxPhiInvert->isChecked() ? -1 : +1;
}

void JoystickWidget::update()
{
  if (REMBB.joystick_state.joystick_okay)
    frameJoyDev->setPaletteBackgroundColor (Qt::green);
  else
    frameJoyDev->setPaletteBackgroundColor (Qt::red);
  comboBoxJoyDev->update();

  QWidget::update();
}

void JoystickWidget::perspectiveChanged( const QString & np )
{
  for (unsigned int i=0; i<num_perspectives; i++)
    if (np == QString(perspective_name[i]))
      REMBB.joystick_state.perspective=perspective_angle[i];
}
