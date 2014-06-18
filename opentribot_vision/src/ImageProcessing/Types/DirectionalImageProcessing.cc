#include "DirectionalImageProcessing.h"

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>

#include "../ImageProcessingFactory.h"
#include "../../Fundamental/Vec.h"
#include "../../Structures/Journal.h"
#include "../Formation/Image.h"
#include "../PixelAnalysis/Image2WorldMapping.h"
#include "../../Structures/Journal.h"
//#include "../../WorldModel/WorldModel.h"
#include "../ObjectAnalysis/BallDetector.h"
#include "../../Player/WhiteBoard.h"
#include "../Formation/ImageProducer.h"

#include <cmath>

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public Tribots::ImageProcessingBuilder {
    static Builder the_builder;
  public:
    Builder () {
      Tribots::ImageProcessingFactory::get_image_processing_factory ()->sign_up (std::string("Directional"), this);
    }
    Tribots::ImageProcessingType* get_image_processing (const std::string&, const std::string& section, const Tribots::ConfigReader& reader, Tribots::ImageProcessingType*, const Tribots::ImageProducer* producer) throw (Tribots::TribotsException,std::bad_alloc) {
      return new Tribots::DirectionalImageProcessing (reader, section, producer->getRobotMask(), producer->getWidth(), producer->getHeight(), producer->getImageWorldMapping(), producer->getWorldImageMapping() );
    }
  };
  Builder the_builder;
}

namespace Tribots {

  using namespace std;

  DirectionalImageProcessing::DirectionalImageProcessing(const ConfigReader& config, const string& section,
                                                         const RobotMask* robotMask,
                                                         int imageWidth, int imageHeight,
                                                         const ImageWorldMapping* i2w,
                                                         const WorldImageMapping* w2i)
    : scannerTime(0), 
      ballDetectorTime(0),
      lineScannerTime(0),
      lineDetectorTime(0),
      cycles(0), image2world(i2w), world2image(w2i), 
      report_computation_time(false), 
      report_computation_time_gnuplot(false),
      rawImageRequested(false), processedImageRequested(false),
      requestedImage(0),imageCollected(false),
      bordermap(0), bordermapSize(0),  
      detectLines(false), scanResults(0), scanner(0), lineDetector(0),
      ballDetector(0), preferClosestBall(true)
  {
    int centerX, centerY;
    string markerFile="";
    string imageLogFilename;
    int nScanLines = 10;
    double minWidth, maxWidth;
    bool use2Steps = false;
    bool checkForGreen = false;
    bool cutOutsideField = true;
    bool useMaximalDistance = false;
    bool useFieldMapper = false;
    bool useLineFilter = false;
    FieldMapper* fieldmapper = 0;  // im Moment nicht verwendet

    const FieldGeometry& fg = MWM.get_field_geometry ();
    max_y = 0.5*fg.field_length+fg.goal_band_width;
    max_x = 0.5*fg.field_width+fg.side_band_width;

    if (config.get((section+"::image_center_x").c_str(), centerX) <=0) {
      throw 
	InvalidConfigurationException((section+"::image_center_x").c_str());
    }   
    if (config.get((section+"::image_center_y").c_str(), centerY) <=0) {
      throw 
	InvalidConfigurationException((section+"::image_center_y").c_str());
    }   
  
    
    // LineDetecting
    if (config.get((section+"::detect_lines").c_str(), detectLines) < 0) {
      throw InvalidConfigurationException((section+"::detectLines").c_str());
    }
    if (detectLines) {
      if (config.get((section+"::number_of_scanlines").c_str(), nScanLines) <= 0) {
        throw InvalidConfigurationException((section+"::number_of_scanlines").c_str());
      }   
      if (config.get((section+"::min_line_width").c_str(), minWidth) <= 0) {
        throw InvalidConfigurationException((section+"::min_line_width").c_str());
      }   
      if (config.get((section+"::max_line_width").c_str(), maxWidth) <= 0) {
        throw InvalidConfigurationException((section+"::max_line_width").c_str());
      }    
      if (config.get((section+"::use_two_steps").c_str(), use2Steps) < 0) {
        throw InvalidConfigurationException((section+"::use_two_steps").c_str());
      }
      if (config.get((section+"::use_line_filter").c_str(), useLineFilter) <= 0) {
        throw InvalidConfigurationException((section+"::use_line_filter").c_str());
      }        
    }
    if (config.get((section+"::prefer_closest_ball").c_str(), preferClosestBall) < 0) {
      throw InvalidConfigurationException("ScanLine::prefer_closest_ball");
    }
    if (config.get("report_computation_time", report_computation_time) < 0) {
      throw InvalidConfigurationException("report_computation_time");
    }  
    if (config.get("report_computation_time_gnuplot", 
		   report_computation_time_gnuplot) < 0) {
      throw InvalidConfigurationException("report_computation_time_gnuplot");
    }  
    
    /*
    if (!robotMask)
      throw TribotsException ("DirectionalImageProcessing: no robot mask given "
                              "or invalid robot mask."); */
    
    ColorClassInfoList* colClass = new ColorClassInfoList();
    colClass->classList[COLOR_BALL]->findPoints=false;
    colClass->classList[COLOR_OBSTACLE]->findTransitions=false;
    cc = new ChainCoder();
    ScanLines* lines = 
      ScanLines::createForDirectedCamera(robotMask, imageWidth, imageHeight, 
                                         nScanLines);
    scanner = new LineScanner(lines, colClass);
    scanner->use_linefilter = useLineFilter;
    scanResults = new ScanResultList(colClass->classList.size());
    lineDetector = new LineDetector(image2world,fieldmapper, minWidth, maxWidth,
                                    use2Steps, checkForGreen, 
                                    cutOutsideField, useMaximalDistance,
                                    false, false, useFieldMapper);
    ballDetector = 
      new BallDetector(robotMask, image2world, world2image);  
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

  DirectionalImageProcessing::~DirectionalImageProcessing() throw()
  {  
    if (cycles > 0 && report_computation_time) {
      JMESSAGE(slip_prepareMsg("image scanning", 
			       static_cast<double>(scannerTime) / cycles));
      JMESSAGE(slip_prepareMsg("ball detecting", 
			       static_cast<double>(ballDetectorTime)/ cycles));
    }      
    if (detectLines) {
      JMESSAGE(slip_prepareMsg("line scanning", 
                               static_cast<double>(lineScannerTime) / cycles));
      JMESSAGE(slip_prepareMsg("line detecting", 
                               static_cast<double>(lineDetectorTime)/ cycles));
    }
    
//    delete image2real;
    if (scanResults) delete scanResults;
    if (scanner) delete scanner;
    if (lineDetector) delete lineDetector;
    if (ballDetector) delete ballDetector;
    
    fout.flush();
    fout.close();
  }
  
  void
  DirectionalImageProcessing::scan(const Image &image, vector<Vec>* results)
  {
    int horizon_y = 0;
    int start_step = 10;
    double step_factor = 0.96;

    int width=image.getWidth()-1;
    int y=image.getHeight()-1;
    int x;
    double stepsize = start_step;
    int step = static_cast<int>(stepsize);
      
    while(y>horizon_y) {
      x=0;
      while(x<width) {
	      if(image.getPixelClass(x,y) == COLOR_BALL) { 
 	        results->push_back(Vec(x,y));
        }
  	    x+=step;  
      }
      y-=step;
      step = static_cast<int>(stepsize*=step_factor);
      if(step<2) step = 2;
    }
    if (processedImageRequested) {
      RGBTuple red  = { 255,   0,   0 };
      RGBTuple grey = { 128, 128, 128 };
      y=image.getHeight()-1;
      stepsize = start_step;      
      step = static_cast<int>(stepsize);      
      while(y>horizon_y) {
        x=0;
        while(x<width) {
  	      if(image.getPixelClass(x,y) == COLOR_BALL) {
   	        requestedImage->setPixelRGB(x,y, red);
          }
          else {
            requestedImage->setPixelRGB(x,y, grey);
          }
    	    x+=step;  
        }
        y-=step;
        step = static_cast<int>(stepsize*=step_factor);
        if(step<2) step = 2;
      }    
    }
  }  
   
  class RegionComparator {
  public:
    bool operator() (const Region& r1, const Region& r2) {
      return r1.getArea() > r2.getArea();
    }
  };

  void 
  DirectionalImageProcessing::process_image(Image* image, VisibleObjectList* vol, RegionList* ballRegions) 
    throw (Tribots::BadHardwareException)
  {
    stringstream gnuplot;

    image->setBlackBorder();

    if (rawImageRequested || processedImageRequested) {
      requestedImage = image->clone();
      rawImageRequested = false;
    }
    gnuplot << "DIRECTIONAL_IMAGEPROCESSING_ELAPSED_GNUPLOT: "
	    << WorldModel::get_main_world_model().get_game_state().cycle_num
	    << " ";

    // Scanning the image for pixels of the ball
    Time time;
    vector<Vec> redPixels;
    this->scan(*image, &redPixels);
    scannerTime += time.elapsed_usec();
    
    if (report_computation_time_gnuplot) 
      gnuplot  << time.elapsed_usec() << " ";

    // find red regions
    time.update();
    if (processedImageRequested)
      ballDetector->setVisualize(requestedImage);
    time.update();
    RegionList potentialBallRegions;
    int nBalls = 
      ballDetector->searchBall(*image, redPixels,
                               image->getTimestamp(), &potentialBallRegions);
//    LOUT<<"DIRECTIONAL: NBALLS: " << nBalls << endl;
    if (nBalls == 1) {
      if (potentialBallRegions.list[0]->getArea() > 100)
        ballRegions->list.push_back(potentialBallRegions.list[0]->clone());
    }
    else if (nBalls > 1) {  // \todo: 3D-Ballposition wo moeglich verwenden
      const BallLocation& bloc = MWM.get_ball_location(image->getTimestamp()); 
      const RobotLocation& rloc = MWM.get_robot_location(image->getTimestamp());
      const FieldGeometry& fg = MWM.get_field_geometry ();
      Vec3D bpos(bloc.pos);
      double maxY = 0.5*fg.field_length+fg.goal_band_width;
      double maxX = 0.5*fg.field_width+fg.side_band_width;
      Frame2d rel2abs(rloc.pos, rloc.heading);     // \todo: ggf. 3D-Position verwenden
      Vec firstWorld = rel2abs * 
        image2world->map(potentialBallRegions.list[0]->getCenterOfGravity());
      if (preferClosestBall &&          // wenn baelle Nahe beim Roboter bevorzugt werden sollen
          fabs(firstWorld.x) < maxX &&  // und der erste ball in der liste auf dem feld liegt,
          fabs(firstWorld.y) < maxY) {  // dann suche den ball, der am nächsten am roboter ist
                // implizit: potentialBallRegions enthält aktuell entweder nur bälle auf dem Feld oder
                //           nur Bälle außerhalb des Feldes.
        double minSDist = image2world->map(potentialBallRegions.list[0]->getCenterOfGravity()).squared_length();
        int minPos = -1;
        double tmp;
        for (unsigned int i=0; i < potentialBallRegions.list.size(); i++) {
          if (((tmp = image2world->map(potentialBallRegions.list[i]->getCenterOfGravity()).squared_length()) <= minSDist) &&
              (potentialBallRegions.list[i]->getArea() > 100))
          {
            minSDist = tmp;
            minPos = i;
          }
        }
        if (minPos != -1) // only if a region was found
          ballRegions->list.push_back(potentialBallRegions.list[minPos]->clone());
#ifdef REGIONDEBUG
        // add the rest of the regions
        if (minPos != -1)
        {
          for (unsigned int i=0;i<potentialBallRegions.list.size();i++)
          {
            if (minPos != i)
              ballRegions->list.push_back(potentialBallRegions.list[i]->clone());
          }
        }
#endif
      } else
      {
        double minSDist = (potentialBallRegions.list[0]->getCenterOfGravity() - world2image->map3D(bpos)).squared_length();
        double tmp;
        int minPos = -1;
        for (unsigned int i=0; i < potentialBallRegions.list.size(); i++) {
          if (((tmp = (potentialBallRegions.list[i]->getCenterOfGravity() - world2image->map3D(bpos)).squared_length()) <= minSDist) &&
             (potentialBallRegions.list[i]->getArea() > 100)) 
          {
            minSDist = tmp;
            minPos = i;
          }
        }
        if (minPos != -1)
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
//    LOUT << "DIRECTIONAL: BallRegions zu GMSV: " << ballRegions->list.size() << endl;
    ballDetectorTime += time.elapsed_usec();
    if (report_computation_time_gnuplot) gnuplot  << time.elapsed_usec() << " ";
    
    // linescanning und linedetector
    if (detectLines) {
      if (processedImageRequested) scanner->setVisualize(requestedImage);
      time.update();
      scanResults->clear();     // remove results of the last frame
      scanner->scan(*image, scanResults);
      lineScannerTime += time.elapsed_usec();
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
    }
    
    cycles++;    
    if (report_computation_time_gnuplot) {
      string str;
      getline(gnuplot, str);
      JMESSAGE(str.c_str());
    }

    if (processedImageRequested) {
      processedImageRequested = false;       // image preparation finished
    }
  }

  void DirectionalImageProcessing::request_image_raw() throw(TribotsException)
  {
    if (imageCollected) {
      throw TribotsException("You have collected an image, but have not "
			     "freed the image yet.");
    }
    if (rawImageRequested || processedImageRequested) {
      throw TribotsException("At the moment, I'm already preparing another "
			     "image.");
    }
    if (requestedImage) {
      delete requestedImage;
      requestedImage = 0;
    }
    rawImageRequested = true;
  }
      
  void DirectionalImageProcessing::request_image_processed() throw(TribotsException)
  {
    if (imageCollected) {
      throw TribotsException("You have collected an image, but have not "
			     "freed the image yet.");
    }
    if (rawImageRequested || processedImageRequested) {
      throw TribotsException("At the moment, I'm already preparing another "
			     "image.");
    }
    if (requestedImage) {
      delete requestedImage;
      requestedImage = 0;
    }
    processedImageRequested = true;
  }

  bool DirectionalImageProcessing::is_image_available() throw()
  {
    return requestedImage != 0;
  }
  
  const Image* DirectionalImageProcessing::get_image() 
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

  void DirectionalImageProcessing::free_image() throw(TribotsException)
  {
    if (! imageCollected) {
      throw TribotsException("You have not collected an image. There is "
			     "nothing to free.");
    }
    delete requestedImage;
    requestedImage = 0;
    imageCollected = false;
  }

}
