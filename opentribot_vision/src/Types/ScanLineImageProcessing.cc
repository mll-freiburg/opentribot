#include "ScanLineImageProcessing.h"

#include <string>
#include <iostream>
#include <sstream>
#include <cmath>

#include "../ImageProcessingFactory.h"
#include "../ObjectAnalysis/ScanLines.h"
#include "../ObjectAnalysis/SimpleObstacleDetector.h"
#include "../ObjectAnalysis/PolarObstacleDetector.h"
#include "../../WorldModel/WorldModel.h"
#include "../../Structures/Journal.h"
#include "../Formation/Painter.h"


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public Tribots::ImageProcessingBuilder {
    static Builder the_builder;
  public:
    Builder () {
      Tribots::ImageProcessingFactory::get_image_processing_factory ()->sign_up (std::string("ScanLine"), this);
    }
    Tribots::ImageProcessingType* get_image_processing (const std::string&, const std::string& section, const Tribots::ConfigReader& reader, Tribots::ImageProcessingType*, const Tribots::ImageProducer* producer) throw (Tribots::TribotsException,std::bad_alloc) {
      return new Tribots::ScanLineImageProcessing (reader, section.c_str(), producer->getRobotMask(), producer->getImageCenter(), producer->getImageWorldMapping(), producer->getWorldImageMapping(), producer->getWidth(), producer->getHeight());
    }
  };
  Builder the_builder;
}

namespace Tribots {

  using namespace std;

  ScanLineImageProcessing::ScanLineImageProcessing(const ConfigReader& config, 
                                                   const char* section,
                                                   const RobotMask* robotMask,
                                                   Vec imageCenter,
                                                   const ImageWorldMapping* image2real,
                                                   const WorldImageMapping* real2image,
                                                   int imageWidth, int imageHeight)
    throw (InvalidConfigurationException, HardwareException,
	   BadHardwareException, TribotsException)
    : configSection(section), scannerTime(0), lineDetectorTime(0), 
      ballDetectorTime(0), obstacleDetectorTime(0),
      fieldMapperTime(0), cycles(0),
      image2real(image2real), real2image(real2image),
      robotMask(robotMask),
      centerSurveillance (imageCenter.x, imageCenter.y),
      report_computation_time(false), 
      report_computation_time_gnuplot(false),
      rawImageRequested(false), processedImageRequested(false),
      requestedImage(0), imageCollected(false), imageCenter(imageCenter),
      preferClosestBall(true)
  {
    int innerRadius, outerRadius;
    int nScanLines;
    string markerFile="";
    double minWidth, maxWidth;
    string imageLogFilename;
    string obstDectName ="";

    bool checkForGreen = false;
    bool use2Steps = false;
    bool cutOutsideField = false;
    bool useHalfField = false;
    bool useMaximalDistance = false;
    double maximalDistance;
    bool useFieldMapper = false;
    bool useLineFilter= false;
    bool scanBlackPixelNeighbourhood = false;

    if (config.get((configSection+"::inner_scan_radius").c_str(), innerRadius) <= 0) {
      throw InvalidConfigurationException("ScanLine::inner_scan_radius");
    }   
    if (config.get((configSection+"::outer_scan_radius").c_str(), outerRadius) <= 0) {
      throw InvalidConfigurationException("ScanLine::outer_scan_radius");
    }   
    if (config.get((configSection+"::number_of_scanlines").c_str(), nScanLines) <= 0) {
      throw InvalidConfigurationException("ScanLine::number_of_scanlines");
    }   
    if (config.get((configSection+"::min_line_width").c_str(), minWidth) <= 0) {
      throw InvalidConfigurationException("ScanLine::min_line_width");
    }   
    if (config.get((configSection+"::max_line_width").c_str(), maxWidth) <= 0) {
      throw InvalidConfigurationException("ScanLine::max_line_width");
    }   
    if (config.get((configSection+"::obstacle_detector").c_str(), obstDectName) < 0) {
      throw InvalidConfigurationException("ScanLine::obstacle_detector");
    }   
    if (config.get((configSection+"::use_two_steps").c_str(), use2Steps) < 0) {
      throw InvalidConfigurationException("ScanLine::use_two_steps");
    }   
    if (config.get((configSection+"::cut_lines_outside_field").c_str(), cutOutsideField) < 0) {
      throw InvalidConfigurationException("ScanLine::cut_lines_outside_field");
    }  
    if (config.get((configSection+"::check_for_field").c_str(), checkForGreen) < 0) {
      throw InvalidConfigurationException("ScanLine::check_for_field");
    }  
    if (config.get((configSection+"::use_maximal_distance").c_str(), useMaximalDistance) < 0) {
      throw InvalidConfigurationException("ScanLine::use_maximal_distance");
    }  
    if (config.get((configSection+"::useFieldMapper").c_str(), useFieldMapper) < 0) {
      throw InvalidConfigurationException("ScanLine::useFieldMapper");
    }  
    if (config.get((configSection+"::useLineFilter").c_str(), useLineFilter) < 0) {
      throw InvalidConfigurationException("ScanLine::useLineFilter");
    }  
    if (useMaximalDistance) {
      if (config.get((configSection+"::maximal_distance").c_str(), maximalDistance) <= 0) {
	throw InvalidConfigurationException("ScanLine::maximal_distance");
      }
    }
    if (config.get((configSection+"::use_half_field").c_str(), useHalfField) < 0) {
      throw InvalidConfigurationException("ScanLine::use_half_field");
    }
    if (config.get((configSection+"::prefer_closest_ball").c_str(), preferClosestBall) < 0) {
      throw InvalidConfigurationException("ScanLine::prefer_closest_ball");
    }     
    if (config.get("report_computation_time", report_computation_time) < 0) {
      throw InvalidConfigurationException("report_computation_time");
    }  
    if (config.get("report_computation_time_gnuplot", 
		   report_computation_time_gnuplot) < 0) {
      throw InvalidConfigurationException("report_computation_time_gnuplot");
    }

    if (config.get("scan_black_pixel_neighbourhood" , scanBlackPixelNeighbourhood)< 0) {
      throw InvalidConfigurationException("scan_black_pixel_neighbourhood");
    }

    colClass = new ColorClassInfoList(config);
    
    if (!robotMask)
      throw TribotsException ("ScanLineImageProcessing: no robot mask given "
                              "or invalid robot mask.");

    if (!image2real || !real2image)
      throw TribotsException ("no world-to-image mapping found or mapping invalid.");    
    
    ScanLines* lines = new ScanLines(robotMask, 
                                     imageCenter, 
                                     innerRadius,
                                     outerRadius,
                                     imageWidth, imageHeight,
                                     nScanLines);
    scanner = new LineScanner(lines, colClass , scanBlackPixelNeighbourhood);
    scanner->use_linefilter=useLineFilter;
    scanResults = new ScanResultList(colClass->classList.size());

    fieldmapper=new FieldMapper();
    
    
    lineDetector = new LineDetector(image2real,fieldmapper, minWidth, maxWidth,
				    use2Steps, checkForGreen, 
				    cutOutsideField, useMaximalDistance,
				    maximalDistance, useHalfField, useFieldMapper);   
    

    ballDetector = 
      new BallDetector(robotMask, image2real, real2image);

    
    // check config which Detctor should be used
    bool obstacleDetectorSelected = false;
    if (obstDectName == "SimpleObstacleDetector") {   
      obstacleDetector = new SimpleObstacleDetector(config, image2real,
   	                                             real2image, 1);
      obstacleDetectorSelected = true;
    }
    if (obstDectName == "PolarObstacleDetector") {   
      obstacleDetector = new PolarObstacleDetector(config, image2real,
						    real2image, 1);
      obstacleDetectorSelected = true;
    }

    if (!obstacleDetectorSelected) { 
      JWARNING("No obstacle detector selected!");
      obstacleDetector = 0;
    }

  }

  /* lokale Hilfsmethode, die eine Modulnachricht erzeugt. */
  static const char* slip_prepareMsg(string module, double time)
  {
    string str;
    stringstream stream;
    stream << "average " << module << " time was " 
	   << (time / 1000.) << "ms" << endl;
    getline(stream, str);
    return str.c_str();
  }

  ScanLineImageProcessing::~ScanLineImageProcessing() throw()
  {  
    if (cycles > 0 && report_computation_time) {
      JMESSAGE(slip_prepareMsg("line scanning", 
			       static_cast<double>(scannerTime) / cycles));
      JMESSAGE(slip_prepareMsg("ball detecting", 
			       static_cast<double>(ballDetectorTime)/ cycles));
      JMESSAGE(slip_prepareMsg("line detecting", 
			       static_cast<double>(lineDetectorTime)/ cycles));
      JMESSAGE(slip_prepareMsg("field mapper", 
			       static_cast<double>(fieldMapperTime)/ cycles));
      JMESSAGE(slip_prepareMsg("obstacle detecting", 
      		       static_cast<double>(obstacleDetectorTime) 
      		       / cycles));
    }      

    delete ballDetector;
    delete lineDetector;
    delete obstacleDetector;
    delete fieldmapper;
    //delete cc;
    delete scanner;
    delete scanResults;
    delete colClass;
  }

  void 
  ScanLineImageProcessing::process_image(Image* image, VisibleObjectList* vol, RegionList* ballRegions) 
    throw (Tribots::BadHardwareException)
  {
    try{// to catch ImageFailed
    Time time;
    stringstream gnuplot;
    image->setBlackBorder();

    if (rawImageRequested || processedImageRequested) {
      requestedImage = image->clone();
      rawImageRequested = false;
    }
    
    if (processedImageRequested) {          // visualize global information?
      visualizeCommonInfo();
    }


    //Field Mapper
   

    time.update();
    fieldmapper->buildFieldMap(*image);
    fieldMapperTime+= time.elapsed_usec();
    if (report_computation_time_gnuplot) 
      gnuplot  << time.elapsed_usec() <<" ";//Field Mapper build Fieldmap ready
    if (processedImageRequested)
      fieldmapper->drawVisualization(requestedImage);
    
    if (processedImageRequested) 
      scanner->setVisualize(requestedImage);
    time.update();
    scanResults->clear();     // remove results of the last frame
    scanner->scan(*image, scanResults);
    scannerTime += time.elapsed_usec();
    if (report_computation_time_gnuplot) 
      gnuplot  << time.elapsed_usec() << " ";

    if (processedImageRequested)
      lineDetector->setVisualize(requestedImage);
    time.update();
    lineDetector->searchLines(scanResults->results[COLOR_LINE],
			      image->getTimestamp(), vol); 
       // side effect: adds detected line transitions to world model
    lineDetectorTime += time.elapsed_usec();
    if (report_computation_time_gnuplot) 
      gnuplot  << time.elapsed_usec() << " ";

    if (report_computation_time_gnuplot) 
      gnuplot  << time.elapsed_usec() << " ";
    
    if (processedImageRequested)
      ballDetector->setVisualize(requestedImage);
    time.update();
    RegionList potentialBallRegions;
    int nBalls = 
      ballDetector->searchBall(*image, scanResults->results[COLOR_BALL],
			                         image->getTimestamp(), &potentialBallRegions);
//    LOUT<<"SCANLINE: NBALLS: " << nBalls << endl;
    if (nBalls == 1) {
      ballRegions->list.push_back(potentialBallRegions.list[0]->clone());
    }
        ballRegions->list.push_back(potentialBallRegions.list[minPos]->clone());
      }
      else {
        double minSDist = (potentialBallRegions.list[0]->getCenterOfGravity() - real2image->map3D(bpos)).squared_length();
        double tmp;
        int minPos = 0;
        for (unsigned int i=1; i < potentialBallRegions.list.size(); i++) {
          if ((tmp = (potentialBallRegions.list[i]->getCenterOfGravity() - real2image->map3D(bpos)).squared_length()) <= minSDist) {
            minSDist = tmp;
            minPos = i;
          }
        }
          ballRegions->list.push_back(potentialBallRegions.list[minPos]->clone());
#ifdef REGIONDEBUG
        if (minPos != -1)
        {
          // add the rest of the regions
          for (unsigned int i=0;i<potentialBallRegions.list.size();i++)
          {
            if (minPos != i)
              ballRegions->list.push_back(potentialBallRegions.list[i]->clone());
          }
        }
#endif

      }
    }
//    LOUT << "SCANLINE: BallRegions zu GMSV: " << ballRegions->list.size() << endl;
    ballDetectorTime += time.elapsed_usec();
    if (report_computation_time_gnuplot) 
      gnuplot  << time.elapsed_usec() << " ";
    time.update();
    // der obstacledetector geht davon aus, dass Randpixel eine andere 
    // farbe haben...

    if (obstacleDetector) { // if obstacle Detector selected
      try {
	if (processedImageRequested) 
	  obstacleDetector->setVisualize(requestedImage);
	obstacleDetector->searchObstacles(*image, 
					  scanResults->results[COLOR_OBSTACLE],
					  image->getTimestamp(), 
					  vol);
      } catch (TribotsException &e) {
	std::cerr << "Fehler in ObstDetector: " << e.what() << "\n";
      }
    }
    
    obstacleDetectorTime += time.elapsed_usec();
    if (report_computation_time_gnuplot) gnuplot  << time.elapsed_usec() << " ";
    cycles++;    

    if (report_computation_time_gnuplot) {
      string str;
      getline(gnuplot, str);
      JMESSAGE(str.c_str());
    }

    if (processedImageRequested) {
      processedImageRequested = false;       // image preparation finished
    }
  }catch(ImageFailed&){
    JERROR ("failed in grabbing an image");
  }
  }  // end process_image

  void ScanLineImageProcessing::request_image_raw() throw(TribotsException)
  {
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
      
  void ScanLineImageProcessing::request_image_processed() throw(TribotsException)
  {
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

  bool ScanLineImageProcessing::is_image_available() throw()
  {
    return requestedImage != 0;
  }
  
  const Image* ScanLineImageProcessing::get_image() 
    throw(TribotsException)
  {
    if (! requestedImage) {
      throw TribotsException("No image ready to collect. Please request an "
			     "image first and make sure it's ready to be "
			     "collected (check is_image_available from time "
			     " to time until it returns true).");
    }
    imageCollected = true;
    return requestedImage;
  }

  void ScanLineImageProcessing::free_image() throw(TribotsException)
  {
    if (! imageCollected) {
      throw TribotsException("You have not collected an image. There is "
			     "nothing to free.");
    }
    delete requestedImage;
    requestedImage = 0;
    imageCollected = false;
  }


  void ScanLineImageProcessing::visualizeCommonInfo() 
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
    painter.markCrosshair(static_cast<int>(imageCenter.x), static_cast<int>(imageCenter.y), 5);
  }

}
