
#include "CameraSettingsWidget.h"
#include "../../components/IDQObjects.h"
#include <QtGui/QFrame>
#include <QtGui/QPushButton>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <iostream>
#include <typeinfo>

using namespace TribotsTools;
using namespace Tribots;

// ---------------------- CameraSettingsWidget::FeatureInfo ------------------------
struct TribotsTools::CameraSettingsWidget::FeatureInfo {
  IIDC::CameraFeature feature;
  IIDC::CameraFeatureMode mode;
  unsigned int value;
  bool uv;  // fuer Weissabgleich (u-Wert(true), v-Wert(false)

  QSpinBox* spinBox;
  QSlider* slider;
  QComboBox* comboBox;
};


// --------------------- Methoden von CameraSettingsWidget: --------------------------
CameraSettingsWidget::CameraSettingsWidget ( QWidget* parent, Qt::WFlags flags) : QScrollArea (parent) {
  num_features=0;
  features=NULL;
  camera=NULL;
  timer = new QTimer;
  connect (timer, SIGNAL(timeout()), this, SLOT(updateAutoFeatures()));
}


CameraSettingsWidget::~CameraSettingsWidget () {
  if (features) delete [] features;
  if (timer) delete timer;
}

void CameraSettingsWidget::deinit () {
  camera=NULL;
  setWidget(new QFrame);
}

void CameraSettingsWidget::init (Tribots::IIDC* cam1, bool memChan, bool head) {
  lockedCamera=true;
  lockedDisplay=false;

  camera = cam1;
  if (!camera) return;
  features = new TribotsTools::CameraSettingsWidget::FeatureInfo [11];  // z.Z. werden maximal 11 verwendet

  // das Widget aufbauen
  unsigned int y0=0;  // die aktuelle y0-Position des naechsten Frames
  const unsigned int pwidth= 350;  // die Breite der Frames
  QFrame* frame = new QFrame;
  frame->setFrameShape (QFrame::NoFrame);
  frame->setFrameShadow (QFrame::Plain);
  // das Feld mit Kamerabezeichnung und Modus
  QFrame* iframe = new QFrame (frame);
  iframe->setFrameShape (QFrame::NoFrame);
  iframe->setFrameShadow (QFrame::Plain);
  if (head) {
    iframe->setGeometry (0, y0, pwidth, 60);
    y0+=60;
    QLineEdit* edit = new QLineEdit (iframe);
    edit->setGeometry (5,5,200,20);
    edit->setReadOnly (true);
    edit->setText (camera->getCameraType().c_str());
    edit = new QLineEdit (iframe);
    edit->setGeometry (215,5,130,20);
    edit->setReadOnly (true);
    edit->setText (camera->getCameraUID().c_str());
    edit = new QLineEdit (iframe);
    edit->setGeometry (5,35,340,20);
    edit->setReadOnly (true);
    edit->setText (camera->getCameraMode().c_str());
  }
  // die Featurefelder
  IIDC::CameraFeature feats [] = { IIDC::whiteBalance, IIDC::shutter, IIDC::gain, IIDC::brightness, IIDC::exposure, IIDC::gamma, IIDC::sharpness, IIDC::hue, IIDC::saturation, IIDC::filter };
  QString featstring [] = { "WhiteBalance", "Shutter", "Gain", "Brightness", "Exposure", "Gamma", "Sharpness", "Hue", "Saturation", "Filter" };
  num_features=0;
  for (unsigned int fn = 0; fn<10; fn++) {
    unsigned int fn_available = camera->availableFeatureModes (feats[fn]);
    if (fn_available & IIDC::featureUnavailable) {
      continue;
    }
    iframe = new QFrame (frame);
    iframe->setFrameShape (QFrame::NoFrame);
    iframe->setFrameShadow (QFrame::Plain);
    QLabel* label = new QLabel (iframe);
    label->setText (featstring[fn]);
    label->setGeometry (5, 5, 80, 20);
    IDComboBox* comboBox = new IDComboBox (iframe, num_features);
    comboBox->setGeometry (90,5,60,20);
    if (fn_available & IIDC::featureOff)
      comboBox->addItem ("Off");
    if (fn_available & IIDC::featureMan)
      comboBox->addItem ("Man");
    if (fn_available & IIDC::featureAuto)
      comboBox->addItem ("Auto");
    IDSlider* slider = new IDSlider (iframe, num_features);
    slider->setGeometry (160, 5, 120, 20);
    unsigned int minv, maxv;
    camera->getFeatureMinMaxValue (minv, maxv, feats[fn]);
    slider->setMinimum (minv);
    slider->setMaximum (maxv);
    IDSpinBox* spinBox = new IDSpinBox (iframe, num_features);
    spinBox->setGeometry (290, 5, 57, 22);
    spinBox->setMinimum (minv);
    spinBox->setMaximum (maxv);
    if (feats[fn]==IIDC::whiteBalance) {
      iframe->setGeometry (0, y0, pwidth, 60);
      y0+=60;
      IDSlider* slider2 = new IDSlider (iframe, num_features+1);
      slider2->setGeometry (160, 35, 120, 20);
      slider2->setMinimum (minv);
      slider2->setMaximum (maxv);
      IDSpinBox* spinBox2 = new IDSpinBox (iframe, num_features+1);
      spinBox2->setGeometry (290, 35, 57, 22);
      spinBox2->setMinimum (minv);
      spinBox2->setMaximum (maxv);
      features[num_features].feature = feats[fn];
      features[num_features].spinBox = spinBox;
      features[num_features].slider = slider;
      features[num_features].comboBox = comboBox;
      features[num_features].uv = true;
      features[num_features+1].feature = feats[fn];
      features[num_features+1].spinBox = spinBox2;
      features[num_features+1].slider = slider2;
      features[num_features+1].comboBox = comboBox;
      features[num_features+1].uv = false;
      num_features+=2;
      connect (comboBox,SIGNAL(comboBoxChanged (unsigned int, const QString&)), this, SLOT(comboBoxChanged (unsigned int, const QString&)));
      connect (spinBox,SIGNAL(spinBoxChanged (unsigned int, unsigned int)), this, SLOT(spinBoxChanged (unsigned int, unsigned int)));
      connect (slider,SIGNAL(sliderChanged (unsigned int, unsigned int)), this, SLOT(sliderChanged (unsigned int, unsigned int)));
      connect (spinBox2,SIGNAL(spinBoxChanged (unsigned int, unsigned int)), this, SLOT(spinBoxChanged (unsigned int, unsigned int)));
      connect (slider2,SIGNAL(sliderChanged (unsigned int, unsigned int)), this, SLOT(sliderChanged (unsigned int, unsigned int)));
      updateFeatureMode (num_features-2);
      updateFeatureValue (num_features-2);
      updateFeatureValue (num_features-1);
    } else {
      iframe->setGeometry (0, y0, pwidth, 30);
      y0+=30;
      features[num_features].feature = feats[fn];
      features[num_features].spinBox = spinBox;
      features[num_features].slider = slider;
      features[num_features].comboBox = comboBox;
      num_features++;
      connect (comboBox,SIGNAL(comboBoxChanged (unsigned int, const QString&)), this, SLOT(comboBoxChanged (unsigned int, const QString&)));
      connect (spinBox,SIGNAL(spinBoxChanged (unsigned int, unsigned int)), this, SLOT(spinBoxChanged (unsigned int, unsigned int)));
      connect (slider,SIGNAL(sliderChanged (unsigned int, unsigned int)), this, SLOT(sliderChanged (unsigned int, unsigned int)));
      updateFeatureMode (num_features-1);
      updateFeatureValue (num_features-1);
    }
  }
  // Load/Save memory channel:
  if (camera->numChannel()>0 && memChan) {
    iframe = new QFrame (frame);
    iframe->setFrameShape (QFrame::NoFrame);
    iframe->setFrameShadow (QFrame::Plain);
    iframe->setGeometry (0, y0, pwidth, 30);
    y0+=30;
    QLabel* label = new QLabel (iframe);
    label->setText ("Camera memory channel");
    label->setGeometry (5, 5, 180, 20);
    QPushButton* loadButton = new QPushButton (iframe);
    loadButton->setGeometry (170, 5, 70, 20);
    loadButton->setText ("load");
    QPushButton* saveButton = new QPushButton (iframe);
    saveButton->setGeometry (270, 5, 70, 20);
    saveButton->setText ("save");
    connect (loadButton, SIGNAL(clicked()), this, SLOT(loadMemoryChannel()));
    connect (saveButton, SIGNAL(clicked()), this, SLOT(saveMemoryChannel()));
  }
  // ganzes Widget:
  frame->setMinimumSize (pwidth, y0);
  frame->setMaximumSize (pwidth, y0);
  setWidget (frame);

  lockedCamera=false;
  lockedDisplay=false;
}


void CameraSettingsWidget::showEvent ( QShowEvent * ) {
  if (!camera)
    return;
  for (unsigned int fn=0; fn<num_features; fn++) {
    updateFeatureValue (fn);
    updateFeatureMode (fn);
  }
  timer->start (1000);
}


void CameraSettingsWidget::hideEvent ( QHideEvent * ) {
  timer->stop();
  emit (widgetClosed());
}


void CameraSettingsWidget::updateAutoFeatures () {
  if (!camera)
    return;
  for (unsigned int fn=0; fn<num_features; fn++) {
    if (features[fn].mode==IIDC::featureAuto)
      updateFeatureValue (fn);
  }
}


void CameraSettingsWidget::updateFeatureValue (unsigned int fn) {
  if (!camera)
    return;
  lockedCamera = true;
  if (!lockedDisplay) {
    lockedDisplay=true;
    if (features[fn].feature==IIDC::whiteBalance) {
      unsigned int u, v;
      camera->getWhiteBalance (u, v);
      unsigned int uf, vf;
      if (features[fn].uv) {
        uf=fn;
        vf=fn+1;
      } else {
        uf=fn-1;
        vf=fn;
      }
      features[uf].value = u;
      features[uf].spinBox->setValue (u);
      features[uf].slider->setValue (u);
      features[vf].value = v;
      features[vf].spinBox->setValue (v);
      features[vf].slider->setValue (v);
    } else {
      features[fn].value = camera->getFeatureValue (features[fn].feature);
      features[fn].spinBox->setValue (features[fn].value);
      features[fn].slider->setValue (features[fn].value);
    }
    lockedDisplay=false;
  }
  lockedCamera=false;
}


void CameraSettingsWidget::updateFeatureMode (unsigned int fn) {
  if (!camera)
    return;
  lockedCamera=true;
  if (!lockedDisplay) {
    lockedDisplay=true;
    features[fn].mode = camera->getFeatureMode (features[fn].feature);
    bool enable = (features[fn].mode==IIDC::featureMan);
    features[fn].spinBox->setEnabled (enable);
    features[fn].slider->setEnabled (enable);
    QString desired_text = (features[fn].mode==IIDC::featureOff ? "Off" : (features[fn].mode==IIDC::featureMan ? "Man" : (features[fn].mode==IIDC::featureAuto ? "Auto" : "")));
    bool text_found=false;
    for (int i=0; i<features[fn].comboBox->count(); i++)
      if (desired_text== features[fn].comboBox->itemText(i))
        text_found=true;
    if (!text_found) {
      desired_text = "???";
      features[fn].comboBox->addItem (desired_text);
    }
    int index=features[fn].comboBox->findText (desired_text);
    if (index<0) {
      features[fn].comboBox->addItem (desired_text);
      index=features[fn].comboBox->findText (desired_text);
    }
    features[fn].comboBox->setCurrentIndex (index);
    if (features[fn].feature==IIDC::whiteBalance) {
      unsigned f2 = (features[fn].uv ? fn+1 : fn-1);
      features[f2].mode = features[fn].mode;
      features[f2].spinBox->setEnabled (enable);
      features[f2].slider->setEnabled (enable);
    }
    lockedDisplay=false;
  }
  lockedCamera=false;
}

void CameraSettingsWidget::sliderChanged (unsigned int fn, unsigned int val) {
  if (!camera)
    return;
  if (!lockedCamera && (features[fn].mode==IIDC::featureMan)) {
    if (features[fn].feature==IIDC::whiteBalance) {
      unsigned int u=val;
      unsigned int v=val;
      if (features[fn].uv) v=features[fn+1].value;
      else u=features[fn-1].value;
      camera->setWhiteBalance (u,v);
    } else {
      camera->setFeatureValue (features[fn].feature, val);
    }
  }
  updateFeatureValue (fn);
}


void CameraSettingsWidget::spinBoxChanged (unsigned int fn, unsigned int val) {
  sliderChanged (fn, val);
}


void CameraSettingsWidget::comboBoxChanged (unsigned int fn, const QString& qtext) {
  if (!camera)
    return;
  if (qtext=="???")
    return;
  camera->setFeatureMode (features[fn].feature, (qtext=="Off" ? IIDC::featureOff : (qtext=="Auto" ? IIDC::featureAuto : IIDC::featureMan)));
  updateFeatureMode (fn);
  if (features[fn].mode==IIDC::featureMan)
    updateFeatureValue (fn);
}


void CameraSettingsWidget::loadMemoryChannel () {
  if (!camera)
    return;
  camera->loadSettings();
  showEvent (NULL);
}

void CameraSettingsWidget::saveMemoryChannel () {
  if (!camera)
    return;
  camera->saveSettings();
}

void CameraSettingsWidget::writeFeatureSettings (std::ostream& dest) {
  if (!camera)
    return;
  // sicherheitshalber nochmal die Kamerawerte abfragen
  if (typeid (*camera)==typeid(Tribots::IIDC)) {
    for (unsigned int fn=0; fn<num_features; fn++) {
      updateFeatureValue (fn);
      updateFeatureMode (fn);
    }
  }

  dest << "# Camera type: " << camera->getCameraType() << '\n';
  dest << "uid = " << camera->getCameraUID() << '\n';
//  dest << "mode = " << camera->getCameraMode() << '\n';
  unsigned int fn=0;
  while (fn<num_features) {
    if (features[fn].feature==IIDC::whiteBalance) {
      dest << "white_balance = ";
      if (features[fn].mode==IIDC::featureOff)
        dest << "Off\n";
      else if (features[fn].mode==IIDC::featureAuto)
        dest << "Auto\n";
      else
        dest << features[fn].value << ' ' << features[fn+1].value << '\n';
      fn+=2;
    } else {
      switch (features[fn].feature) {
        case IIDC::shutter: dest << "shutter"; break;
        case IIDC::gain: dest << "gain"; break;
        case IIDC::brightness: dest << "brightness"; break;
        case IIDC::exposure: dest << "exposure"; break;
        case IIDC::gamma: dest << "gamma"; break;
        case IIDC::sharpness: dest << "sharpness"; break;
        case IIDC::hue: dest << "hue"; break;
        case IIDC::saturation: dest << "saturation"; break;
        case IIDC::filter: dest << "filter"; break;
        default: dest << "unknownFeature"; break;
      }
      dest << " = ";
      if (features[fn].mode==IIDC::featureOff)
        dest << "Off\n";
      else if (features[fn].mode==IIDC::featureAuto)
        dest << "Auto\n";
      else
        dest << features[fn].value << '\n';
      fn++;
    }
  }
  dest << std::flush;
}

void CameraSettingsWidget::writeFeatureSettings (Tribots::ConfigReader& config, const std::string& section) {
  if (!camera)
    return;
  // sicherheitshalber nochmal die Kamerawerte abfragen
  if (typeid (*camera)==typeid(Tribots::IIDC)) {
    for (unsigned int fn=0; fn<num_features; fn++) {
      updateFeatureValue (fn);
      updateFeatureMode (fn);
    }
  }

  unsigned int fn=0;
  while (fn<num_features) {
    std::string key = section+"::";
    if (features[fn].feature==IIDC::whiteBalance) {
      key+="white_balance";
      if (features[fn].mode==IIDC::featureOff)
        config.set(key.c_str(), std::string("Off"));
      else if (features[fn].mode==IIDC::featureAuto)
        config.set(key.c_str(), std::string("Auto"));
      else {
        std::vector<int> v (2);
        v[0]=features[fn].value;
        v[1]=features[fn+1].value;
        config.set(key.c_str(), v);
      }
      fn+=2;
    } else {
      switch (features[fn].feature) {
        case IIDC::shutter: key+="shutter"; break;
        case IIDC::gain: key+="gain"; break;
        case IIDC::brightness: key+="brightness"; break;
        case IIDC::exposure: key+="exposure"; break;
        case IIDC::gamma: key+="gamma"; break;
        case IIDC::sharpness: key+="sharpness"; break;
        case IIDC::hue: key+="hue"; break;
        case IIDC::saturation: key+="saturation"; break;
        case IIDC::filter: key+="filter"; break;
        default: key+="unknownFeature"; break;
      }
      if (features[fn].mode==IIDC::featureOff)
        config.set(key.c_str(), std::string("Off"));
      else if (features[fn].mode==IIDC::featureAuto)
        config.set(key.c_str(), std::string("Auto"));
      else
        config.set(key.c_str(), features[fn].value);
      fn++;
    }
  }
}
