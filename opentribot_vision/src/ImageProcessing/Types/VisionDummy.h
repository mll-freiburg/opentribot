
#ifndef tribots_vision_dummy_h
#define tribots_vision_dummy_h

#include "../VisionType.h"

namespace Tribots {

  /** Dummy-Implementierung der Bildverarbeitung; tut nichts */
  class VisionDummy:public VisionType {
  public:
    void process_images () throw ();
    int get_num_sources() const throw() { return 1; } 
  };

}

#endif

