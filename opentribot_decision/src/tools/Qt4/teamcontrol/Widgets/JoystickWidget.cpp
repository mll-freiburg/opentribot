
#include "JoystickWidget.h"
#include "../States/RemoteBlackboard.h"

using namespace TribotsTools;

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
  void set_combo_text (QComboBox& cb, const char* s) {
    int index= cb.findText (s);
    if (index<0) {
      cb.addItem (s);
      index=cb.findText (s);
    }
    cb.setCurrentIndex (index);
  }
}


JoystickWidget::JoystickWidget(QWidget* p, Qt::WindowFlags f) : QWidget (p,f)
{
  setupUi(this);

  connect (comboBoxJoyDev,SIGNAL(activated(int)),this,SLOT(deviceChanged(int)));
  connect (spinBoxKickButtonIdx,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxPhiAxisIdx,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxPhiAxisMaxV,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxPhiAxisNull,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxXAxisIdx,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxXAxisMaxV,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxXAxisMaxV,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxYAxisIdx,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxYAxisMaxV,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxYAxisNull,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (checkBoxPhiInvert,SIGNAL(toggled(bool)),this,SLOT(valueChanged(bool)));
  connect (checkBoxXInvert,SIGNAL(toggled(bool)),this,SLOT(valueChanged(bool)));
  connect (checkBoxYInvert,SIGNAL(toggled(bool)),this,SLOT(valueChanged(bool)));
  connect (spinBoxActivateButtonIdx,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (spinBoxDeactivateButtonIdx,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));
  connect (comboBoxPerspective,SIGNAL(activated(const QString&)),this,SLOT(perspectiveChanged(const QString&)));
  connect (spinBoxKickLength,SIGNAL(valueChanged(int)),this,SLOT(valueChanged(int)));

  set_combo_text (*comboBoxJoyDev, REMBB.joystick_state.device_name.c_str());
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
    comboBoxPerspective->addItem (QString(perspective_name[i]));
  set_combo_text (*comboBoxPerspective, perspective_name[0]);
}

JoystickWidget::~JoystickWidget ()
{
  // leer, da keine dynamische Objekte erzeugt wurden  
}

void JoystickWidget::deviceChanged( int )
{
  REMBB.joystick_state.device_name=std::string(comboBoxJoyDev->currentText().toAscii());
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

void JoystickWidget::update ()
{
  QPalette qpal_red;
  QPalette qpal_green;
  qpal_red.setBrush (QPalette::Background, Qt::red);
  qpal_green.setBrush (QPalette::Background, Qt::green);
  if (REMBB.joystick_state.joystick_okay)
    frameJoyDev->setPalette (qpal_green);
  else
    frameJoyDev->setPalette (qpal_red);
  comboBoxJoyDev->update();
  QWidget::update();
}

void JoystickWidget::perspectiveChanged( const QString & np )
{
  for (unsigned int i=0; i<num_perspectives; i++)
    if (np == QString(perspective_name[i]))
      REMBB.joystick_state.perspective=perspective_angle[i];
}
