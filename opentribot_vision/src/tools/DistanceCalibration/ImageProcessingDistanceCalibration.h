
#ifndef TribotsTools_ImageProcessingDistanceCalibration_h
#define TribotsTools_ImageProcessingDistanceCalibration_h

#include "../../ImageProcessing/VisionType.h"
#include "../../ImageProcessing/Formation/ImageProducer.h"
#include "../../ImageProcessing/ObjectAnalysis/ColorClasses.h"
#include "../../ImageProcessing/ObjectAnalysis/LineScanning.h"
#include "../../ImageProcessing/ObjectAnalysis/BallDetector.h"
#include "../../ImageProcessing/PixelAnalysis/RobotMask.h"
#include "DistanceCalibration.h"
#include <string>
#include <fstream>


namespace TribotsTools {

  /**
   * Pseudobildverarbeitung, um eine Distanztabelle zu erzeugen
   */
  class ImageProcessingDistanceCalibration : public Tribots::VisionType {
  public:
    ImageProcessingDistanceCalibration(const Tribots::ConfigReader& config, const std::string&);
    virtual ~ImageProcessingDistanceCalibration() throw();

    int get_num_sources() const throw() { return 1; }
    void process_images() throw (Tribots::BadHardwareException);    

    void request_image_raw(int) throw(Tribots::TribotsException);
    void request_image_processed(int) throw(Tribots::TribotsException);
    bool is_image_available(int) throw();
    const Tribots::Image* get_image(int) throw(Tribots::TribotsException);
    void free_image(int) throw(Tribots::TribotsException);
    
  private:
    void visualizeCommonInfo();
    
    Tribots::Vec image_center;
    std::string markerFile;

    Tribots::ImageProducer* imgProd;
    Tribots::ColorClassInfoList* colClass;
    Tribots::ScanResultList* scanResults;
    Tribots::LineScanner* scanner;
    const Tribots::ImageWorldMapping* image2real;
    const Tribots::WorldImageMapping* real2image;
    Tribots::BallDetector* ballDetector;
    const Tribots::RobotMask* robotMask;
    TribotsTools::DistanceCalibration* distcal;
    
    // fuer debug-Images:
    bool rawImageRequested;
    bool processedImageRequested;
    Tribots::Image* requestedImage;
    bool imageCollected;
  };
}

#endif
