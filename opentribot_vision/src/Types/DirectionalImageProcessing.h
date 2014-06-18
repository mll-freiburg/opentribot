#ifndef _DirectionalImageProcessing_h_
#define _DirectionalImageProcessing_h_

#include <fstream>

#include "../ImageProcessingType.h"
#include "../ObjectAnalysis/ColorClasses.h"
#include "../PixelAnalysis/Image2WorldMapping.h"
#include "../ObjectAnalysis/ChainCoding.h"
#include "../ObjectAnalysis/ColorClasses.h"
#include "../ObjectAnalysis/LineScanning.h"
#include "../ObjectAnalysis/LineDetector.h"
#include "../ObjectAnalysis/BallDetector.h"
#include "../ObjectAnalysis/FieldMapper.h"
#include <string>
#include <vector>

namespace Tribots {

  using std::string;
  using std::vector;

  class CoordinateMapping;
  class RegionList;

  /**
   */
  class DirectionalImageProcessing : public Tribots::ImageProcessingType {
  public:
    DirectionalImageProcessing(const ConfigReader& config, const string& section,
                               const RobotMask* robotMask, int imageWidth, int imageHeight,
                               const ImageWorldMapping* image2world,
                               const WorldImageMapping* world2image);
    virtual ~DirectionalImageProcessing() throw();

    virtual void process_image(Image*, VisibleObjectList*, RegionList*) 
      throw (Tribots::BadHardwareException);   

    virtual void request_image_raw() throw(TribotsException);
    virtual void request_image_processed() throw(TribotsException);
    virtual bool is_image_available() throw();
    virtual const Image* get_image() throw(TribotsException);
    virtual void free_image() throw(TribotsException); 

  protected: 
    std::ofstream fout;
  
    long scannerTime;
    long ballDetectorTime;
    long lineScannerTime;
    long lineDetectorTime;

    long cycles;

//    ColorClassInfoList* colClass;
    const ImageWorldMapping* image2world;
    const WorldImageMapping* world2image;
    ChainCoder* cc;
    
    bool report_computation_time;
    bool report_computation_time_gnuplot;

    bool rawImageRequested;
    bool processedImageRequested;
    Image* requestedImage;
    bool imageCollected;
    
    char* bordermap;
    int bordermapSize;

    double max_x, max_y;
    
    bool detectLines;
    ScanResultList* scanResults;
    LineScanner* scanner;
    LineDetector* lineDetector;
    BallDetector* ballDetector;
    
    bool preferClosestBall;
    
    void scan(const Image &image, vector<Vec> *results);
    
  };
}

#endif
