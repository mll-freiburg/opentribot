#ifndef _GenericMultiSourceVision_h_
#define _GenericMultiSourceVision_h_

#include <vector>

#include "../VisionType.h"
#include "../Formation/MultiImageProducer.h"
#include "../Formation/Image.h"
#include "../ImageProcessing.h"          // abstract creation of image processing units

namespace Tribots {

  using std::vector;

  /**
   * Dieses Visionmodul betreibt n-Bildsourcen und verwaltet fr jede Bildquelle
   * eine zugeh�ige Bildverarbeitungsroutine.
   */
  class GenericMultiSourceVision : public Tribots::VisionType {
  public:
  
    enum
    {
      FUSE_FAILSAFE,    /** only use information from additional cameras if primary camera does not yield (ball) information */
      FUSE_INDEPENDENT, /** use information from all cameras (indepedently) but do not fuse */
      FUSE_3D           /** if possible, fuse camera information yielding stereoscopic depth */ 
    };
  
  
    GenericMultiSourceVision(const ConfigReader& config, const char* section) 
      throw(InvalidConfigurationException, HardwareException,
	    BadHardwareException, TribotsException);
    virtual ~GenericMultiSourceVision() throw();
    virtual int get_num_sources() const throw();

    virtual void process_images() throw (Tribots::BadHardwareException);   

    virtual const vector<RegionList*>& get_regionlists() throw(TribotsException);
    
    virtual void request_image_raw(int) throw(TribotsException);
    virtual void request_image_processed(int) throw(TribotsException);
    virtual bool is_image_available(int) throw();
    virtual const Image* get_image(int) throw(TribotsException);
    virtual void free_image(int) throw(TribotsException); 
    const VisibleObjectList& get_last_seen_objects(int camera) const throw()
    { return visibleObjects.at(camera); }

  protected: 

    vector <long> producerTimes;
    vector <long> imageProcessingTimes;

    long cycles;  // FIXME: Need this?

    vector <ImageProducer*> imgProds;
    vector <ImageProcessing*> imgProc;
    vector <Image*> images;
    vector <VisibleObjectList> visibleObjects;
    vector<RegionList*> ballRegionLists;
    
    bool report_computation_time;
    bool report_computation_time_gnuplot;
    
    int cameraFusionStyle;
    // for fuse, define close area (in omni) that is only 2d
    double closeNonFuseRange;
    
    // multi camera statistics (success vs. misses on robotcontrol console)
    bool multiCameraStats;
    vector<int> goodFrames;
    vector<int> badFrames;
    Time normalOperationStartingTime;
  };
}

#endif
