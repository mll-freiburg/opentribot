
#ifndef tribots_image_processing_dummy_h
#define tribots_image_processing_dummy_h

#include "../ImageProcessingType.h"

namespace Tribots {

  /** Dummy-Implementierung der Bildverarbeitung; tut nichts */
  class ImageProcessingDummy:public ImageProcessingType {
  public:
    void process_image () throw ();
  };

}

#endif

