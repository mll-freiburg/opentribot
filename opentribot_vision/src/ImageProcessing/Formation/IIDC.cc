
#include "IIDC.h"
#include "CamDriver.h"
#include "ImageSourceFactory.h"
#include "../../Fundamental/stringconvert.h"
#include <sstream>

using namespace std;
using namespace Tribots;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public Tribots::ImageSourceBuilder {
    public:
      Builder () {
        Tribots::ImageSourceFactory::get_image_source_factory ()->sign_up (string("CameraSource"), this);
      }
      Tribots::ImageSource* get_image_source (const std::string&, const Tribots::ConfigReader& reader, const std::string& section) throw (Tribots::TribotsException,std::bad_alloc) {
        return new Tribots::IIDC (reader, section);
      }
  };
  Builder the_builder;
}





IIDC::IIDC (const ConfigReader& cfg, const std::string& section, bool force_blocking) throw (HardwareException, InvalidConfigurationException) : cam_driver(NULL) {
  string sTmp;
  string device;
  string mode;
  string uid = "";
  bool isBlocking = false;
  int delay = 0;

  bool doSoftExposure=false;
  bool doSoftWhiteBalance=false;
  bool adjustGain=false;
  int balanceStep=1;
  vector<int> shutterRange;
  vector<int> gainRange;
  vector<int> uvRange;
  bool balanceArea = false;
  vector<int> area;
  bool doSelfTest = true;
  int shutterLogic=1;
  unsigned int exposure=127;

  if (cfg.get((section+"::device_name").c_str(), device) <= 0)
    throw InvalidConfigurationException((section+"::device_name").c_str());
  if (cfg.get((section+"::uid").c_str(), uid) <= 0) {      // requested unit ID of the camera
    uid = "";
  }
  if (cfg.getline((section+"::mode").c_str(), mode) <= 0)
    throw InvalidConfigurationException((section+"::mode").c_str());
  if (cfg.get((section+"::blocking").c_str(), isBlocking) <= 0)
    throw InvalidConfigurationException((section+"::blocking").c_str());
  if (force_blocking)
    isBlocking=true;
  if (cfg.get((section+"::delay").c_str(), delay) <= 0)
    throw InvalidConfigurationException((section+"::delay").c_str());
  cfg.get((section+"::self_test").c_str(), doSelfTest);

  if (cfg.get((section+
      "::soft_auto_exposure").c_str(), 
  doSoftExposure) < 0)
    throw InvalidConfigurationException((section+
        "::soft_auto_exposure").c_str());
  if (cfg.get((section+
      "::soft_white_balance").c_str(), 
  doSoftWhiteBalance) < 0)
    throw InvalidConfigurationException((section+
        "::soft_white_balance").c_str());
  if (cfg.get((section+
      "::adjust_gain").c_str(), 
  adjustGain) < 0)
    throw InvalidConfigurationException((section+
      "::adjust_gain").c_str());
  if (cfg.get((section+
      "::balance_step").c_str(), 
  balanceStep) < 0)
    throw InvalidConfigurationException((section+
      "::balance_step").c_str());
  if (cfg.get((section+"::shutter_range").c_str(),
      shutterRange) < 0 || 
      (shutterRange.size() != 0 && shutterRange.size() != 2))
    throw InvalidConfigurationException((section+
          "::shutter_range").c_str());
  if (cfg.get((section+"::shutter_logic").c_str(),
      shutterLogic) < 0)
    throw InvalidConfigurationException((section+
          "::shutter_logic").c_str());
  if (cfg.get((section+"::software_exposure").c_str(),
      exposure) < 0)
    throw InvalidConfigurationException((section+
          "::software_exposure").c_str());
  if (cfg.get((section+"::gain_range").c_str(),
      gainRange) < 0|| 
      (gainRange.size() != 0 && gainRange.size() != 2))
    throw InvalidConfigurationException((section+
          "::gain_range").c_str());
  if (cfg.get((section+"::uv_range").c_str(),
      uvRange) < 0|| 
      (uvRange.size() != 0 && uvRange.size() != 2))
    throw InvalidConfigurationException((section+
          "::uv_range").c_str());
  if (cfg.get((section+"::balance_area").c_str(),
              area) < 0 || (area.size() != 4 && area.size() != 0))
    throw InvalidConfigurationException((section+
          "::balance_area").c_str());
  else if (area.size() == 4) {
    balanceArea = true;
  }



  cam_driver = CamDriver::getCamera (device.c_str(), 0, mode, uid, isBlocking, delay);
  if (!cam_driver) throw HardwareException ("Camera driver not ready, cannot create camera object");

  // CameraFeatures einlesen, falls vorhanden, und setzen
  // ACHTUNG: aus unerfindlichen Gruenden arbeitet die SONY DFW nur dann korrekt, wenn Filter vor WhiteBalance steht !?! 
  IIDC::CameraFeature feats [] = { IIDC::filter, IIDC::whiteBalance, IIDC::shutter, IIDC::gain, IIDC::brightness, IIDC::exposure, IIDC::gamma, IIDC::sharpness, IIDC::hue, IIDC::saturation };
  std::string featkeys [] = { "filter", "white_balance", "shutter", "gain", "brightness", "exposure", "gamma", "sharpness", "hue", "saturation" };
  for (unsigned int fn=0; fn<10; fn++) {
    vector<string> vals;
    if (cfg.get((section+string("::")+featkeys[fn]).c_str(), vals)>0) {
      if (vals[0]=="Auto")
        setFeatureMode (feats[fn], IIDC::featureAuto);
      else if (vals[0]=="Off")
        setFeatureMode (feats[fn], IIDC::featureOff);
      else {
        setFeatureMode (feats[fn], IIDC::featureMan);
        unsigned int minv, maxv;
        getFeatureMinMaxValue (minv, maxv, feats[fn]);
        if (feats[fn]==IIDC::whiteBalance && vals.size()>=2) {
          unsigned int u, v;
          string2uint (u, vals[0]);
          string2uint (v, vals[1]);
          if (u<minv) u=minv;
          if (v<minv) v=minv;
          if (u>maxv) u=maxv;
          if (v>maxv) v=maxv;
          setWhiteBalance (u,v);
        } else if (feats[fn]!=IIDC::whiteBalance) {
          unsigned int s;
          string2uint (s, vals[0]);
          if (s<minv) s=minv;
          if (s>maxv) s=maxv;
          setFeatureValue(feats[fn], s);
        }
      }
    }
  }

  if (shutterRange.size() == 0) {
    shutterRange.push_back(0); shutterRange.push_back(0);
  }

  if (gainRange.size() == 0) {
    gainRange.push_back(0); gainRange.push_back(0);
  }

  if (uvRange.size() == 0) {
    uvRange.push_back(0); uvRange.push_back(0);
  }

  initSoftBalance(doSoftExposure, doSoftWhiteBalance, adjustGain, 
                          balanceStep, exposure, shutterLogic, shutterRange[0], shutterRange[1],
                          gainRange[0], gainRange[1], uvRange[0], uvRange[1]);    
  if (balanceArea) {
    setBalanceArea(area[0], area[1], area[2], area[3]);
  }

  if (doSelfTest) {
    bool image_available = false;
    for (unsigned int i=0; i<7; i++) {
      try{
        getImage ();
        image_available=true;
        break;
      }catch(ImageFailed){;}
    }
    if (!image_available)
      throw HardwareException("CameraSelfTest: no images available, camera not ready");
    stringstream inout;
    bool camera_okay=checkImageSource (inout);
    if (!camera_okay) {
      string errormsg = "CameraSelfTest: camera not working properly:\n";
      while (!inout.eof()) {
        string s;
        getline (inout, s);
        errormsg+=s+string("\n");
      }
      throw HardwareException (errormsg.c_str());
    }
  }
}

IIDC::~IIDC () throw () {
  CamDriver::destroyCamera (cam_driver);
}

ImageBuffer IIDC::getImage () throw (ImageFailed) {
  return cam_driver->getImage();
}

int IIDC::getWidth() const throw () {
  return cam_driver->getWidth();
}

int IIDC::getHeight() const throw () {
  return cam_driver->getHeight();
}

std::string IIDC::getCameraType () throw () {
  return cam_driver->getCameraType ();
}

std::string IIDC::getCameraMode () throw () {
  return cam_driver->getCameraMode ();
}

std::string IIDC::getCameraUID () const throw () {
  return cam_driver->getCameraUID ();
}

int IIDC::getDelay () const throw () {
  return cam_driver->getDelay();
}

bool IIDC::setWhiteBalance (unsigned int u, unsigned int v) throw () {
  return cam_driver->setWhiteBalance (u,v);
}

bool IIDC::getWhiteBalance (unsigned int& u, unsigned int& v) throw () {
  return cam_driver->getWhiteBalance(u,v);
}

unsigned int IIDC::getFeatureValue (IIDC::CameraFeature f) throw () {
  return cam_driver->getFeatureValue(f);
}

bool IIDC::setFeatureValue (IIDC::CameraFeature f, unsigned int val) throw () {
  return cam_driver->setFeatureValue (f, val);
}

IIDC::CameraFeatureMode IIDC::getFeatureMode (IIDC::CameraFeature f) throw () {
  return cam_driver->getFeatureMode (f);
}

bool IIDC::setFeatureMode (IIDC::CameraFeature f, IIDC::CameraFeatureMode m) throw () {
  return cam_driver->setFeatureMode (f,m);
}

unsigned int IIDC::availableFeatureModes (IIDC::CameraFeature f) throw () {
  return cam_driver->availableFeatureModes (f);
}

bool IIDC::getFeatureMinMaxValue (unsigned int& minv, unsigned int& maxv, IIDC::CameraFeature f) throw () {
  return cam_driver->getFeatureMinMaxValue (minv, maxv, f);
}

void IIDC::toggleSoftwareExposure(bool on) throw() {
  cam_driver->toggleSoftwareExposure (on);
}

bool IIDC::isSoftwareExposure() const throw() {
  return cam_driver->isSoftwareExposure();
}

void IIDC::toggleSoftwareWhiteBalance(bool on) throw() {
  return cam_driver->toggleSoftwareWhiteBalance (on);
}

bool IIDC::isSoftwareWhiteBalance() const throw() {
  return cam_driver->isSoftwareWhiteBalance ();
}

void IIDC::initSoftBalance(bool doSoftExposure, bool doSoftWhiteBalance, 
                      bool adjustGain, int balanceStep,
                      unsigned char exposure,
                      int shutterLogic,
                      unsigned int shutterMin,
                      unsigned int shutterMax,
                      unsigned int gainMin,
                      unsigned int gainMax,
                      unsigned int uvMin,
                      unsigned int uvMax)
{
  cam_driver->initSoftBalance (doSoftExposure,doSoftWhiteBalance, adjustGain, balanceStep, exposure,
                               shutterLogic, shutterMin, shutterMax, gainMin, gainMax, uvMin, uvMax);
}

bool IIDC::loadSettings(int channel) throw ()
{
  return cam_driver->loadSettings (channel);
}

bool IIDC::saveSettings(int ch) throw () {
  return cam_driver->saveSettings (ch);
}

unsigned int IIDC::numChannel() throw() {
  return cam_driver->numChannel();
}

void IIDC::setBalanceArea(int x, int y, int w, int h)
{
  cam_driver->setBalanceArea(x, y, w, h);
}

void IIDC::getBalanceArea(int *x, int *y, int *w, int *h) const
{
  cam_driver->getBalanceArea(x,y,w,h);
}

bool IIDC::checkImageSource (std::ostream& logout) throw () {
  return cam_driver->checkCamera (logout);
}

int IIDC::getOffsetX() const throw () {
  return cam_driver->getOffsetX();
}

int IIDC::getOffsetY() const throw () {
  return cam_driver->getOffsetY();
}
