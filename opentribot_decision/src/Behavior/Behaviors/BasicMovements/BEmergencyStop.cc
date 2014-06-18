#include "BEmergencyStop.h"

namespace Tribots {

  BEmergencyStop::BEmergencyStop() : Behavior("BEmergencyStop") 
  {}

  DriveVector BEmergencyStop::getCmd(const Time&) 
    throw(TribotsException)
  {
    DriveVector dv;

    dv.vrot   = 0.;
    dv.vtrans = Vec(0., 0.);
    dv.kick = 0.;

    return dv;
  }

}
