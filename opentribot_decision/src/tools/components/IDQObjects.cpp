
#include "IDQObjects.h"

using namespace TribotsTools;

IDSlider::IDSlider ( QWidget * parent, unsigned int id1 ) : QSlider (Qt::Horizontal, parent), id(id1) {
  connect(this, SIGNAL(valueChanged(int)), this, SLOT(captureValueChanged(int)));
}

void IDSlider::captureValueChanged (int val) {
  emit (sliderChanged(id, val));
}

IDSpinBox::IDSpinBox ( QWidget * parent, unsigned int id1 ) : QSpinBox (parent), id(id1) {
  connect(this, SIGNAL(valueChanged(int)), this, SLOT(captureValueChanged(int)));
}

void IDSpinBox::captureValueChanged (int val) {
  emit (spinBoxChanged(id, val)); 
}

IDComboBox::IDComboBox ( QWidget * parent, unsigned int id1 ) : QComboBox (parent), id(id1), validText("") {
  connect(this, SIGNAL(activated(int)), this, SLOT(captureValueChanged(int)));
}

void IDComboBox::captureValueChanged (int val) {
  validText = text (val);
  emit (comboBoxChanged(id, text (val)));
}

const QString& IDComboBox::lastValidText () const {
  return validText;
}

void IDComboBox::setCurrentItem ( int index ) {
  QComboBox::setCurrentItem (index);
  validText = currentText();
}

void IDComboBox::setCurrentText ( const QString & s ) {
  QComboBox::setCurrentText (s);
  validText = currentText();
}

IDAction::IDAction ( QWidget * parent, unsigned int id1 ) : QAction (parent), id(id1) {
  connect(this, SIGNAL(activated()), this, SLOT(captureActivated()));
  connect(this, SIGNAL(toggled(bool)), this, SLOT(captureToggled(bool)));
}

void IDAction::captureActivated () {
  emit (activated(id));
}

void IDAction::captureToggled (bool b) {
  emit (toggled(id,b));
}
