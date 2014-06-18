#include "DribbleImageProcessing.h"

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "../ImageProcessingFactory.h"
#include "../../Fundamental/Vec.h"
#include "../../Structures/Journal.h"
#include "../Formation/Image.h"
#include "../../Structures/Journal.h"
#include "../../WorldModel/WorldModel.h"


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public Tribots::ImageProcessingBuilder {
    static Builder the_builder;
  public:
    Builder () {
      Tribots::ImageProcessingFactory::get_image_processing_factory ()->sign_up (std::string("Dribble"), this);
    }
    Tribots::ImageProcessingType* get_image_processing (const std::string&, const std::string& section, const Tribots::ConfigReader& reader, Tribots::ImageProcessingType*, const Tribots::ImageProducer*) throw (Tribots::TribotsException,std::bad_alloc) {
      return new Tribots::DribbleImageProcessing (reader, section);
    }
  };
  Builder the_builder;
}

namespace Tribots {

  using namespace std;

  DribbleImageProcessing::DribbleImageProcessing(const ConfigReader& config, const string& section)
    : scannerTime(0), 
      ballDetectorTime(0),
      cycles(0), report_computation_time(false), 
      report_computation_time_gnuplot(false),
      rawImageRequested(false), processedImageRequested(false),
      requestedImage(0), imageCollected(false)
  {
    if (config.get("report_computation_time", report_computation_time) < 0) {
      throw InvalidConfigurationException("report_computation_time");
    }  
    if (config.get("report_computation_time_gnuplot", 
		   report_computation_time_gnuplot) < 0) {
      throw InvalidConfigurationException("report_computation_time_gnuplot");
    }  
    
    colClass = new ColorClassInfoList(config);
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

  DribbleImageProcessing::~DribbleImageProcessing() throw()
  {  
    if (cycles > 0 && report_computation_time) {
      JMESSAGE(slip_prepareMsg("image scanning", 
			       static_cast<double>(scannerTime) / cycles));
      JMESSAGE(slip_prepareMsg("ball detecting", 
			       static_cast<double>(ballDetectorTime)/ cycles));
    }      
  }
  
  void
  DribbleImageProcessing::scan(const Image &image, Vec* center, int *n)
  {
    int step = 8;

    int width = image.getWidth();
    int height = image.getHeight();
    Vec pos(0.,0.);
    int count=0;
    
    for (int y=0; y < height; y+=step) {
      for (int x=0; x < width; x+=step) {
	if(image.getPixelClass(x,y) == COLOR_BALL) { 
	  pos += Vec(x,y);
	  ++count;
        }
      }
    }
    if (count > 0) *center = pos * (1./count);
    *n = count;

    if (processedImageRequested) {
      RGBTuple red  = { 255,   0,   0 };
      RGBTuple grey = { 128, 128, 128 };
      RGBTuple blue = {   0,   0, 255 };

      for (int y=0; y < height; y+=step) {
        for (int x=0; x < width; x+=step) {
	  if(image.getPixelClass(x,y) == COLOR_BALL) { 
	    requestedImage->setPixelRGB(x,y, red);
          }
	  else {
	    requestedImage->setPixelRGB(x,y, grey);
          }
        }
      }
      if (*n > 10) {
        for (int i=-4; i< 4; i++) {
	  requestedImage->setPixelRGB((int)center->x+i,(int)center->y, blue);
	  requestedImage->setPixelRGB((int)center->x,(int)center->y+i, blue);
        }
      }
    }
  }  
   
  void 
  DribbleImageProcessing::process_image(Image* image, VisibleObjectList* vol, RegionList*) 
    throw (Tribots::BadHardwareException)
  {
    stringstream gnuplot;

    if (rawImageRequested || processedImageRequested) {
      requestedImage = image->clone();
      rawImageRequested = false;
    }
    gnuplot << "DRIBBLE_IMAGEPROCESSING_ELAPSED_GNUPLOT: "
	    << WorldModel::get_main_world_model().get_game_state().cycle_num
	    << " ";

    // Scanning the image for pixels of the ball
    Time time;
    int n; Vec center;
    this->scan(*image, &center, &n);
    scannerTime += time.elapsed_usec();
    
    if (report_computation_time_gnuplot) 
      gnuplot  << time.elapsed_usec() << " ";

    // find red regions
    time.update();
    
    if (n > 10) {
      cerr << "Center: " << center << "\r\n";
      if (center.x < 280) { 
	MWM.get_message_board().receive("rel_ball_left");
      }
      else if (center.x > 400) {
	MWM.get_message_board().receive("rel_ball_right");
      }
      MWM.get_message_board().receive("rel_ball_middle");
    }
    else {
      MWM.get_message_board().receive("rel_ball_no");
    }

    ballDetectorTime += time.elapsed_usec();
    if (report_computation_time_gnuplot) 
      gnuplot  << time.elapsed_usec() << " ";
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

  void DribbleImageProcessing::request_image_raw() throw(TribotsException)
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
      
  void DribbleImageProcessing::request_image_processed() throw(TribotsException)
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

  bool DribbleImageProcessing::is_image_available() throw()
  {
    return requestedImage != 0;
  }
  
  const Image* DribbleImageProcessing::get_image() 
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

  void DribbleImageProcessing::free_image() throw(TribotsException)
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
