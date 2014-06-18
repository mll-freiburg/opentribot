
#include "SettingsWidget.h"
#include <cmath>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

SettingsWidget::SettingsWidget(VisionToolImageSource& is, Tribots::ConfigReader& cfg, QStatusBar& stb, QWidget* p, Qt::WindowFlags f) : VisionToolWidget (cfg,stb,p,f), imageSource(is) {
  setupUi (this);

  connect(checkBox_softWhiteBalance,SIGNAL(toggled(bool)),this,SLOT(softWhiteBalanceToggled(bool)));
  connect(checkBox_softExposure,SIGNAL(toggled(bool)),this,SLOT(softAutoExposureToggled(bool)));
  connect(checkBox_adjustGain,SIGNAL(toggled(bool)),this,SLOT(softAdjustGainToggled(bool)));
  connect(checkBox_shutterPosLogic,SIGNAL(toggled(bool)),this,SLOT(softShutterPositiveLogicToggled(bool)));
  connect(spinBox_shutterMin,SIGNAL(valueChanged(int)),this,SLOT(softShutterMinChanged(int)));
  connect(spinBox_shutterMax,SIGNAL(valueChanged(int)),this,SLOT(softShutterMaxChanged(int)));
  connect(spinBox_gainMin,SIGNAL(valueChanged(int)),this,SLOT(softGainMinChanged(int)));
  connect(spinBox_gainMax,SIGNAL(valueChanged(int)),this,SLOT(softGainMaxChanged(int)));
  connect(spinBox_uvMin,SIGNAL(valueChanged(int)),this,SLOT(softUVMinChanged(int)));
  connect(spinBox_uvMax,SIGNAL(valueChanged(int)),this,SLOT(softUVMaxChanged(int)));
  connect(spinBox_targetExposure,SIGNAL(valueChanged(int)),this,SLOT(softTargetExposureChanged(int)));

  setFocusPolicy (Qt::StrongFocus);
}

SettingsWidget::~SettingsWidget () {;}


void SettingsWidget::enable(bool b) {
  checkBox_softWhiteBalance->setEnabled(b);
  checkBox_softExposure->setEnabled(b);
  checkBox_adjustGain->setEnabled(b);
  checkBox_shutterPosLogic->setEnabled(b);
  spinBox_shutterMin->setEnabled(b);
  spinBox_shutterMax->setEnabled(b);
  spinBox_gainMin->setEnabled(b);
  spinBox_gainMax->setEnabled(b);
  spinBox_uvMin->setEnabled(b);
  spinBox_uvMax->setEnabled(b);
  spinBox_targetExposure->setEnabled(b);
}

void SettingsWidget::start () {
  imageSource.setMode (true, false);
  imageWidget->centerImage();

  enable (imageSource.getCamera()!=NULL);

  if (imageSource.getCamera()) {
    cameraSettingsWidget->init (imageSource.getCamera(), false, false);
  }

  config.get ("VisionTool::Section", section);
  section+="::";

  bool b;
  int i;
  vector<int> v;
  unsigned int minv, maxv;
  IIDC* camera = imageSource.getCamera();
  b=false;
  config.get((section+"soft_auto_exposure").c_str(), b);
  checkBox_softExposure->setChecked(b);
  b=false;
  config.get((section+"soft_white_balance").c_str(), b);
  checkBox_softWhiteBalance->setChecked(b);
  b=true;
  config.get((section+"adjust_gain").c_str(), b);
  checkBox_adjustGain->setChecked(b);
  i=+1;
  config.get((section+"shutter_logic").c_str(), i);
  checkBox_shutterPosLogic->setChecked(i>0);
  spinBox_targetExposure->setMinimum (0);
  spinBox_targetExposure->setMaximum (255);
  i=127;
  config.get((section+"software_exposure").c_str(), i);
  spinBox_targetExposure->setValue(i);
  v.clear();
  config.get((section+"shutter_range").c_str(), v);
  if (camera) {
    camera->getFeatureMinMaxValue (minv, maxv, IIDC::shutter);
    spinBox_shutterMin->setMinimum (minv);
    spinBox_shutterMax->setMinimum (minv);
    spinBox_shutterMin->setMaximum (maxv);
    spinBox_shutterMax->setMaximum (maxv);
    if (v.size()==2) {
      spinBox_shutterMin->setValue (v[0]);
      spinBox_shutterMax->setValue (v[1]);
    } else {
      spinBox_shutterMin->setValue (minv);
      spinBox_shutterMax->setValue (maxv);
    }
  }
  v.clear();
  config.get((section+"gain_range").c_str(), v);
  if (camera) {
    camera->getFeatureMinMaxValue (minv, maxv, IIDC::gain);
    spinBox_gainMin->setMinimum (minv);
    spinBox_gainMax->setMinimum (minv);
    spinBox_gainMin->setMaximum (maxv);
    spinBox_gainMax->setMaximum (maxv);
    if (v.size()==2) {
      spinBox_gainMin->setValue (v[0]);
      spinBox_gainMax->setValue (v[1]);
    } else {
      spinBox_gainMin->setValue (minv);
      spinBox_gainMax->setValue (maxv);
    }
  }
  v.clear();
  config.get((section+"uv_range").c_str(), v);
  if (camera) {
    camera->getFeatureMinMaxValue (minv, maxv, IIDC::whiteBalance);
    spinBox_uvMin->setMinimum (minv);
    spinBox_uvMax->setMinimum (minv);
    spinBox_uvMin->setMaximum (maxv);
    spinBox_uvMax->setMaximum (maxv);
    if (v.size()==2) {
      spinBox_uvMin->setValue (v[0]);
      spinBox_uvMax->setValue (v[1]);
    } else {
      spinBox_uvMin->setValue (minv);
      spinBox_uvMax->setValue (maxv);
    }
  }
}

void SettingsWidget::stop () {
  imageSource.setMode ();  // default-Modus, tut nichts
  cameraSettingsWidget->writeFeatureSettings (config, section.substr(0, section.length()-2));
  cameraSettingsWidget->deinit();
}

void SettingsWidget::loop () {
  Tribots::Image& image (imageSource.getImage());
  imageWidget->setImage (image);
}


void SettingsWidget::changeParameters (const char* key, int val1, int val2) {
  std::vector<int> vals (2);
  vals[0]=val1;
  vals[1]=val2;
  config.set ((section+key).c_str(), vals);
  imageSource.notify (section+key);
}
void SettingsWidget::changeParameters (const char* key, bool val) {
  config.set ((section+key).c_str(), val);
  imageSource.notify (section+key);
}
void SettingsWidget::changeParameters (const char* key, int val) {
  config.set ((section+key).c_str(), val);
  imageSource.notify (section+key);
}


void SettingsWidget::softWhiteBalanceToggled (bool b) {
  changeParameters ("soft_white_balance", b);
}
void SettingsWidget::softAutoExposureToggled (bool b) {
  changeParameters ("soft_auto_exposure", b);
}
void SettingsWidget::softAdjustGainToggled (bool b) {
  changeParameters ("adjust_gain", b);
}
void SettingsWidget::softShutterPositiveLogicToggled (bool b) {
  changeParameters ("shutter_logic", b? +1 : -1);
}
void SettingsWidget::softShutterMinChanged (int v) {
  changeParameters ("shutter_range", v, spinBox_shutterMax->value());
}
void SettingsWidget::softShutterMaxChanged (int v) {
  changeParameters ("shutter_range", spinBox_shutterMin->value(), v);
}
void SettingsWidget::softGainMinChanged (int v) {
  changeParameters ("gain_range", v, spinBox_gainMax->value());
}
void SettingsWidget::softGainMaxChanged (int v) {
  changeParameters ("gain_range", spinBox_gainMin->value(), v);
}
void SettingsWidget::softUVMinChanged (int v) {
  changeParameters ("uv_range", v, spinBox_uvMax->value());
}
void SettingsWidget::softUVMaxChanged (int v) {
  changeParameters ("uv_range", spinBox_uvMin->value(), v);
}
void SettingsWidget::softTargetExposureChanged (int v) {
  changeParameters ("software_exposure", v);
}
