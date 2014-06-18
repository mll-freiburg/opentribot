#ifndef _DribbleImageProcessing_h_
#define _DribbleImageProcessing_h_

#include <fstream>

#include "../ImageProcessingType.h"
#include "../ObjectAnalysis/ColorClasses.h"
#include <string>
#include <vector>

namespace Tribots {

  using std::string;
  using std::vector;

  /**
   */
  class DribbleImageProcessing : public Tribots::ImageProcessingType {
  public:
    DribbleImageProcessing(const ConfigReader& config, const string& section);
    virtual ~DribbleImageProcessing() throw();

    virtual void process_image(Image*, VisibleObjectList*, RegionList*) 
      throw (Tribots::BadHardwareException);   

    virtual void request_image_raw() throw(TribotsException);
    virtual void request_image_processed() throw(TribotsException);
    virtual bool is_image_available() throw();
    virtual const Image* get_image() throw(TribotsException);
    virtual void free_image() throw(TribotsException); 

  protected: 
    long scannerTime;
    long ballDetectorTime;
    long cycles;

    ColorClassInfoList* colClass;
    
    bool report_computation_time;
    bool report_computation_time_gnuplot;

    bool rawImageRequested;
    bool processedImageRequested;
    Image* requestedImage;
    bool imageCollected;
    
    void scan(const Image &image, Vec *center, int *n);
    
  };
}

#endif
