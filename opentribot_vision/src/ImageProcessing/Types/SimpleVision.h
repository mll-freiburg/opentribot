#ifndef _SimpleImageProcessing_h_
#define _SimpleImageProcessing_h_

#include "../ImageProcessingType.h"
#include "../Formation/MultiImageProducer.h"
#include "../ObjectAnalysis/ColorClasses.h"
#include "../ObjectAnalysis/LineScanning.h"
#include "../ObjectAnalysis/LineDetector.h"
#include "../ObjectAnalysis/BallDetector.h"
#include "../ObjectAnalysis/BallLearner.h"
//#include "../ObjectAnalysis/GoalDetector.h"
#include "../ObjectAnalysis/ObstacleDetector.h"
#include "../ObjectAnalysis/FieldMapper.h"
#include "../PixelAnalysis/RobotMask.h"
#include "../PixelAnalysis/CoordinateMapping.h"
#include "../Calibration/ImageCenterSurveillance.h"

#include <string>

namespace Tribots {

  /**
   * Diese Bildverarbeitung verwendet radial angeordnete Scanlinine um
   * interessante Farben und Farb�berg�nge im Bild zu finden und diese
   * einzelnen Objekten zuzuordnen.
   */
  class SimpleImageProcessing : public Tribots::ImageProcessingType {
  public:
    SimpleImageProcessing(const ConfigReader& config, const char* section,
                            const RobotMask* robotMask, Vec imageCenter,
                            const ImageWorldMapping* image2real,
                            const WorldImageMapping* real2image, 
                            int imageWidth, int imageHeight) 
      throw(InvalidConfigurationException, HardwareException,
	    BadHardwareException, TribotsException);
    virtual ~SimpleImageProcessing() throw();

    virtual void process_image(Image*, VisibleObjectList*, RegionList* ballRegions) 
      throw (Tribots::BadHardwareException);   

    virtual void request_image_raw() throw(TribotsException);
    virtual void request_image_processed() throw(TribotsException);
    virtual bool is_image_available() throw();
    virtual const Image* get_image() throw(TribotsException);
    virtual void free_image() throw(TribotsException); 

  protected: 
    std::string configSection;

    long scannerTime;
    long lineDetectorTime;
    long ballDetectorTime;
    long obstacleDetectorTime;
    long fieldMapperTime;

    long cycles;

    ColorClassInfoList* colClass;
    ScanResultList* scanResults;
    LineScanner* scanner;
    const ImageWorldMapping* image2real;
    const WorldImageMapping* real2image;
    LineDetector* lineDetector;
    BallLearner* ballDetector;
    ChainCoder* cc;
    ObstacleDetector* obstacleDetector;
    FieldMapper * fieldmapper;
    const RobotMask* robotMask;
    ImageCenterSurveillance centerSurveillance;

    bool report_computation_time;
    bool report_computation_time_gnuplot;

    bool rawImageRequested;
    bool processedImageRequested;
    Image* requestedImage;
    bool imageCollected;

    Vec imageCenter;
    bool preferClosestBall;

    void visualizeCommonInfo();
  };
}

#endif
