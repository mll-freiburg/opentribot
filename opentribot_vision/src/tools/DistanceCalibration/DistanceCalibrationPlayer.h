
#ifndef _Tribots_DistanceCalibrationPlayer_h_
#define _Tribots_DistanceCalibrationPlayer_h_

#include "../../Player/SingleRolePlayer.h"

namespace TribotsTools {

  /**
   * Spielertyp, der im DistanceCalibration verwendet wird.
   */
  class DistanceCalibrationPlayer : public Tribots::SingleRolePlayer {
  public:
    DistanceCalibrationPlayer () throw ();
    ~DistanceCalibrationPlayer () throw () {}
    Tribots::DriveVector process_drive_vector (Tribots::Time t) throw ();
   private:
     Tribots::Time timer;
     unsigned int mode;
  };

}

#endif

