// a qt class must be included in order for the os defines to work
#include <QtCore/QtGlobal> 

#include "VisionToolImageSource.h"
#include "../../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../../ImageProcessing/Formation/FileSource.h"
#include "../../../../ImageProcessing/Formation/CamDriver.h"
#include <iostream>
#include <fstream>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;


VisionToolImageSource::VisionToolImageSource (Tribots::ConfigReader& cfg) throw () : config(cfg), imageSource(NULL), defaultImage(640,480), returnImage(NULL), grabThread(NULL) {
  RGBTuple black = {0,0,0};
  RGBTuple white = {255,255,255};
  for (unsigned int j=0; j<480; j++) {
    for (unsigned int i=0; i<640; i++) {
      defaultImage.setPixelRGB (i,j, (4*j<3*i ? black : white));
    }
  }
  config.get("VisionTool::Section", section);
  section+="::";
  isCamera=false;
  useGrabbingThread=false;
  useColorClassifier=false;
}

VisionToolImageSource::~VisionToolImageSource () throw () {
  if (grabThread) {
    grabThread->cancel();
    grabThread->waitForExit();
    delete grabThread;
  }
  if (imageSource)
    delete imageSource;
  if (returnImage)
    delete returnImage;
}

bool VisionToolImageSource::isStarted () const throw () {
  return (imageSource!=NULL);
}




void VisionToolImageSource::setMode (bool singlePicture , bool classify ) {
  if (singlePicture && grabThread)
    grabThread->stopGrabbing();
  if (!singlePicture) {
    if (!grabThread) {
      if (imageSource) {
        grabThread = new GrabbingThread (imageSource);
        grabThread->setClassifier (&classifier);
        grabThread->start();
      } else {
        singlePicture=true;
      }
    } else {
      grabThread->contGrabbing();
    }
  }
  useGrabbingThread=!singlePicture;
  useColorClassifier=classify;
}

Tribots::Image& VisionToolImageSource::getImage () {
  if (returnImage)
    delete returnImage;

  if (!imageSource) {
    returnImage=defaultImage.clone();
    return *returnImage;
  }

  if (useGrabbingThread) {
    returnImage = grabThread->getImage().clone();
  } else {
    GrabbingThread::mutexImageSourceAccess.lock();
    try{
      ImageBuffer buffer = imageSource->getImage ();
      returnImage = new RGBImage (buffer);
    }catch(TribotsException& e) {
      cerr << "Exception in getImage(): " << e.what() << endl;
      returnImage=defaultImage.clone();
    }
    GrabbingThread::mutexImageSourceAccess.unlock();
  }
  if (useColorClassifier && !useGrabbingThread) {
    int w = returnImage->getWidth();
    int h = returnImage->getHeight();
    returnImage->setClassifier(&classifier);
    unsigned char c;
    RGBTuple rgb;
    rgb.r = 0;
    rgb.g = 0;
    rgb.b = 0;
    for (int x=0; x<w; x++) {
      for (int y=0; y<h; y++) {
        c = returnImage->getPixelClass(x,y);
        if (c != COLOR_IGNORE) {
          returnImage->setPixelRGB(x,y, colorInfos.classList[c]->color);
        }
      }
    }
  }

  return *returnImage;
}


void VisionToolImageSource::startFileSource () throw (Tribots::TribotsException) {
  std::string section;
  config.get ("VisionTool::Section", section);
  imageSource = new FileSource (config, section);
  isCamera=false;
}

void VisionToolImageSource::startCameraSource () throw (Tribots::TribotsException) {
  std::string section;
  config.get ("VisionTool::Section", section);
  imageSource = new IIDC (config, section, true);
  isCamera=true;
}

void VisionToolImageSource::stop () throw () {
  if (grabThread) {
    grabThread->cancel();
    grabThread->waitForExit();
    delete grabThread;
    grabThread=NULL;
  }
  delete imageSource;
  imageSource=NULL;
}



void VisionToolImageSource::notify (const std::string& s) {
  cerr << "notify " << s << endl;  // nur fuers debuggen, kann am Ende wieder raus
  if (s.substr(0,section.length())!=section)
    return;
  string s1 = s.substr(section.length(), s.length());
  if (isCamera && imageSource) {
    Tribots::IIDC* cam = dynamic_cast<Tribots::IIDC*>(imageSource);
    if (s1=="soft_white_balance") {
      bool b;
      config.get ((section+"soft_white_balance").c_str(), b);
      cam->toggleSoftwareWhiteBalance (b);
    }
    if (s1=="balance_area") {
      vector<unsigned int> v;
      config.get ((section+"balance_area").c_str(), v);
      if (v.size()==4) {
        cam->setBalanceArea (v[0], v[1], v[2], v[3]);
      }
    }
    if (s1=="soft_white_balance" ||
        s1=="soft_auto_exposure" ||
        s1=="adjust_gain" ||
        s1=="shutter_logic" ||
        s1=="shutter_range" ||
        s1=="gain_range" ||
        s1=="uv_range" ||
        s1=="software_exposure") {
      bool doSoftExposure=false;
      bool doSoftWhiteBalance=false;
      bool adjustGain=false;
      unsigned int exposure=127;
      int shutterLogic=1;
      vector<unsigned int> shutterRange (2);
      vector<unsigned int> gainRange (2);
      vector<unsigned int> uvRange (2);
      config.get ((section+"soft_white_balance").c_str(), doSoftWhiteBalance);
      config.get ((section+"soft_auto_exposure").c_str(), doSoftExposure);
      config.get ((section+"adjust_gain").c_str(), adjustGain);
      config.get ((section+"shutter_logic").c_str(), shutterLogic);
      config.get ((section+"shutter_range").c_str(), shutterRange);
      config.get ((section+"gain_range").c_str(), gainRange);
      config.get ((section+"uv_range").c_str(), uvRange);
      config.get ((section+"software_exposure").c_str(), exposure);
      cam->initSoftBalance(doSoftExposure, doSoftWhiteBalance, adjustGain, 1, exposure, shutterLogic, shutterRange[0], shutterRange[1], gainRange[0], gainRange[1], uvRange[0], uvRange[1]);
    }




    
  }

  // TODO

}



std::vector<std::string> VisionToolImageSource::getUIDs () throw (Tribots::TribotsException) {

	#ifdef Q_OS_MAC
		//in libdc2 the uids are not used anymore
		std::vector<std::string> lst;
	#else
		std::vector<std::string> lst = CamDriver::getCameraUIDs();
	#endif
  return lst;
}

Tribots::IIDC* VisionToolImageSource::getCamera () throw () {
  if (isCamera)
    return dynamic_cast<Tribots::IIDC*>(imageSource);
  else
    return NULL;
}

Tribots::HSISegmentationTool& VisionToolImageSource::getClassifier () throw () {
  return classifier;
}
