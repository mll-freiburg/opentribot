
#ifndef _TribotsTools_MarkerLog_h_
#define _TribotsTools_MarkerLog_h_

#include "../../Fundamental/Angle.h"

namespace TribotsTools {

  /** Kalibrierungsmarker */
  struct MarkerLog {
    enum MarkerType { WB, BW, WR, RM, RW, NL };  ///< white-blue, blue-white, white-red, red-middle, red-white, null

    Tribots::Angle angle;
    double distance;
    MarkerType type;
  };

}

#endif
