
#include "ImageProcessingDistanceCalibration.h"
#include <iostream>
#include <vector>
#include <cmath>

#include "../../ImageProcessing/ObjectAnalysis/ScanLines.h"
#include "../../ImageProcessing/VisionFactory.h"
#include "../../ImageProcessing/Formation/Image.h"
#include "../../ImageProcessing/PixelAnalysis/RobotMask.h"
#include "../../Structures/Journal.h"
//#include "../../WorldModel/WorldModel.h"
#include "../../ImageProcessing/Formation/Painter.h"


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public Tribots::VisionBuilder {
    static Builder the_builder;
  public:
    Builder () {
      Tribots::VisionFactory::get_vision_factory ()->sign_up (std::string("DistanceCalibration"), this);
    }
    Tribots::VisionType* get_vision (const std::string&, const std::string& headline, const Tribots::ConfigReader& reader, Tribots::VisionType*) throw (Tribots::TribotsException,std::bad_alloc) {
      return new TribotsTools::ImageProcessingDistanceCalibration (reader, headline);
    }
  };
  Builder the_builder;
}

using namespace std;
using namespace Tribots;
using namespace TribotsTools;



ImageProcessingDistanceCalibration::ImageProcessingDistanceCalibration(const ConfigReader& config, const string& headline) {
  rawImageRequested=processedImageRequested=imageCollected=false;
  requestedImage=NULL;
  int innerRadius, outerRadius;
  int nScanLines;
  string image_producer_name="";
  string markerLogFile = "";
  vector<double> realDistances;
  vector<double> prototypes;
  vector<string> markerTypeString;
  vector<DistanceCalibration::TransitionMarker> markerTypeEnum;
  double minClusterDistance = 8;
  
  if (config.get((headline+"::inner_scan_radius").c_str(), innerRadius) <= 0) {
    throw InvalidConfigurationException((headline+"::inner_scan_radius").c_str());
  }   
  if (config.get((headline+"::outer_scan_radius").c_str(), outerRadius) <= 0) {
    throw InvalidConfigurationException((headline+"::outer_scan_radius").c_str());
  }   
  if (config.get((headline+"::number_of_scanlines").c_str(), nScanLines) <= 0) {
    throw InvalidConfigurationException((headline+"::number_of_scanlines").c_str());
  }
  if (config.get("OmniDistanceCalibration::image_producer", image_producer_name) <= 0) {
    throw InvalidConfigurationException("OmniDistanceCalibration::image_producer");
  }
  if (config.get("OmniDistanceCalibration::target_file", markerFile) <= 0) {
    throw InvalidConfigurationException("OmniDistanceCalibration::target_file");
  }
  if (config.get("OmniDistanceCalibration::real_distances", realDistances) <=0) {
    throw InvalidConfigurationException("OmniDistanceCalibration::realDistances");
  }
  if (config.get("OmniDistanceCalibration::real_distances", prototypes) <=0) {
    throw InvalidConfigurationException("OmniDistanceCalibration::prototypes");
  }
  if (config.get("OmniDistanceCalibration::marker_type", markerTypeString) <=0) {
    throw InvalidConfigurationException("OmniDistanceCalibration::marker_type");
  }
  if (config.get("OmniDistanceCalibration::min_cluster_distance", minClusterDistance) <0) {
    throw InvalidConfigurationException("OmniDistanceCalibration::min_cluster_distance");
  }
  if (config.get("OmniDistanceCalibration::marker_logfile", markerLogFile) <0) {
    throw InvalidConfigurationException("OmniDistanceCalibration::marker_logfile");
  }

  markerTypeEnum.resize (markerTypeString.size());
  for (unsigned int i=0; i<markerTypeString.size(); i++) {
    if (markerTypeString[i]=="null")
      markerTypeEnum[i] = DistanceCalibration::virtual_zero_change;
    else if (markerTypeString[i]=="bw")
      markerTypeEnum[i] = DistanceCalibration::blue_white_change;
    else if (markerTypeString[i]=="wb")
      markerTypeEnum[i] = DistanceCalibration::white_blue_change;
    else if (markerTypeString[i]=="wr")
      markerTypeEnum[i] = DistanceCalibration::white_red_change;
    else if (markerTypeString[i]=="rw")
      markerTypeEnum[i] = DistanceCalibration::red_white_change;
    else if (markerTypeString[i]=="rm")
      markerTypeEnum[i] = DistanceCalibration::red_middle_change;
    else {
      markerTypeEnum[i] = DistanceCalibration::virtual_zero_change;  // auf irgendwas setzen, was keinen SegFault erzeugt, passiert nur bei fehlerhaften Eingaben
      JERROR ("MarkerType not clearly defined");
    }
  }
  
  imgProd = new ImageProducer(config, image_producer_name);
  colClass = new ColorClassInfoList(config);
  colClass->classList[COLOR_BLUE]->findTransitions=true;
  colClass->classList[COLOR_BALL]->findTransitions=true;
  Image* image = imgProd->nextImage();
  
  robotMask = imgProd->getRobotMask();
  image_center = imgProd->getImageCenter();
  ScanLines* lines = new ScanLines(robotMask,
                                   image_center,
                                   innerRadius,
                                   outerRadius,
                                   image->getWidth(), image->getHeight(),
                                   nScanLines, false);
  
  
  scanner = new LineScanner(lines, colClass);
  scanResults = new ScanResultList(colClass->classList.size());
  
  image2real = imgProd->getImageWorldMapping();
  real2image = imgProd->getWorldImageMapping();
  image_center = imgProd->getImageCenter();
    
  ballDetector = new BallDetector(robotMask, image2real, real2image);
  distcal = new DistanceCalibration (realDistances, prototypes, markerTypeEnum, image_center,
                                     minClusterDistance, markerLogFile);

  delete image;
}

void ImageProcessingDistanceCalibration::process_images() throw (Tribots::BadHardwareException) {
  try{ // to catch ImageFailed
  Image* image = imgProd->nextImage();
  image->setBlackBorder();
  
  if (rawImageRequested || processedImageRequested) {
    requestedImage = image->clone();
    rawImageRequested = false;
  }
  
  if (processedImageRequested) {          // ImageMask und ImageCenter
    visualizeCommonInfo();
  }

  if (processedImageRequested) 
    scanner->setVisualize(requestedImage);  // Scanlinien
  scanResults->clear();     // remove results of the last frame
  scanner->scan(*image, scanResults);

  RegionList ballRegions;
  if (processedImageRequested)
    ballDetector->setVisualize(requestedImage);
  ballDetector->searchBall(*image, scanResults->results[COLOR_BALL],
                           image->getTimestamp(), &ballRegions);
  Vec ballPosition;
  if (ballRegions.list.size()==0) {
    JERROR ("Keinen Ball gesehen!");
  }
  else {
    if (ballRegions.list.size() > 1) JWARNING ("Mehr als eine rote Region gesehen! Ich verwende die groesste.");
    unsigned int maxRegion = 0;
    for (unsigned int i=1; i < ballRegions.list.size(); i++) {
      if (ballRegions.list[i]->getArea() > ballRegions.list[maxRegion]->getArea()) {
        maxRegion = i;
      }
    }
    ballPosition = ballRegions.list[maxRegion]->getCenterOfGravity();
  }

  // Ab jetzt nach den Kalibriermerkmalen suchen
  if (false) { // Ball gefunden
    Vec dir = (ballPosition-image_center);
    double tau = 10000;
    if (image_center.x+tau*dir.x<1)
      tau = (1-image_center.x)/dir.x;
    if (image_center.x+tau*dir.x+1>=image->getWidth())
      tau = (image->getWidth()-2-image_center.x)/dir.x;
    if (image_center.y+tau*dir.y<1)
      tau = (1-image_center.y)/dir.y;
    if (image_center.y+tau*dir.y+1>=image->getHeight())
      tau = (image->getHeight()-2-image_center.y)/dir.y;
    Vec endpos = image_center+tau*dir;
    ScanLines* line = new ScanLines(robotMask, image_center, endpos, image->getWidth(), image->getHeight()) ;
    LineScanner* lscanner = new LineScanner(line, colClass);
    ScanResultList lScanResults(colClass->classList.size());
    
    if (processedImageRequested) 
      lscanner->setVisualize(requestedImage);  // Scanlinien
    lscanner->scan(*image, &lScanResults);
    distcal->search_markers (lScanResults);
	
    delete lscanner;
  }

  delete image;
  
  }catch(ImageFailed&){
    JERROR ("failed in grabbing an image");
  }
}

ImageProcessingDistanceCalibration::~ImageProcessingDistanceCalibration() throw () {
  distcal->writeMarkerFile (markerFile);

  delete distcal;
  delete imgProd;
  delete colClass;
  delete scanResults;
  delete scanner;
  delete ballDetector;
  if (requestedImage)
    delete requestedImage;
}

void ImageProcessingDistanceCalibration::request_image_raw(int) throw(TribotsException) {
  if (imageCollected) {
    throw TribotsException("You have collected an image, but have not "
        "freed the image yet.");
  }
  if (rawImageRequested || processedImageRequested) {
    throw TribotsException("At the moement, I'm already preparing another "
        "image.");
  }
  if (requestedImage) {
    delete requestedImage;
    requestedImage = 0;
  }
  rawImageRequested = true;
}

void ImageProcessingDistanceCalibration::request_image_processed(int) throw(TribotsException) {
  if (imageCollected) {
    throw TribotsException("You have collected an image, but have not "
        "freed the image yet.");
  }
  if (rawImageRequested || processedImageRequested) {
    throw TribotsException("At the moement, I'm already preparing another "
        "image.");
  }
  if (requestedImage) {
    delete requestedImage;
    requestedImage = 0;
  }
  processedImageRequested = true;
}

bool ImageProcessingDistanceCalibration::is_image_available(int) throw() {
  return requestedImage != 0;
}

const Image* ImageProcessingDistanceCalibration::get_image(int) throw(TribotsException) {
  if (! requestedImage) {
    throw TribotsException("No image ready to collect. Please request an "
        "image first and make sure it's ready to be "
        "collected (check is_image_available from time "
        " to time until it returns true).");
  }
  imageCollected = true;
  return requestedImage;
}

void ImageProcessingDistanceCalibration::free_image(int) throw(TribotsException) {
  if (! imageCollected) {
    throw TribotsException("You have not collected an image. There is "
        "nothing to free.");
  }
  delete requestedImage;
  requestedImage = NULL;
  imageCollected = false;
}

void ImageProcessingDistanceCalibration::visualizeCommonInfo() 
{
  RGBTuple rgb;
    
  for (int x=0; x < requestedImage->getWidth(); x++) {  // draw image mask
    for (int y=0; y < requestedImage->getHeight(); y++) {
      if (!robotMask->isValid(x,y)) {
        requestedImage->getPixelRGB(x,y, &rgb);
        rgb.r = ((rgb.r + 175) / 2);
        rgb.g = ((rgb.g + 175) / 2);
        rgb.b = ((rgb.b + 250) / 2);
        requestedImage->setPixelRGB(x,y, rgb);
      }
    }
  }

  Painter painter(*requestedImage);            // draw image center
  painter.setColor(Painter::white);
  painter.markCrosshair(static_cast<int>(image_center.x), static_cast<int>(image_center.y), 5);
}
