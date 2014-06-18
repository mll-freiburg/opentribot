
#ifndef _ODE_SIM_PACK_VISION_H_
#define _ODE_SIM_PACK_VISION_H_

#include "../../ImageProcessing/VisionType.h"
#include "OdeSimPackClient.h"

namespace Tribots {

  /** Dummy-Implementierung der Bildverarbeitung; tut nichts */
  class OdeSimPackVision:public VisionType {
  public:
    OdeSimPackVision(const ConfigReader& cfg) throw (Tribots::TribotsException);
    void process_images () throw (Tribots::BadHardwareException);
    int get_num_sources() const throw() { return 1; } 

  protected:
    OdeSimPackClient* client;
  };

}

#endif

