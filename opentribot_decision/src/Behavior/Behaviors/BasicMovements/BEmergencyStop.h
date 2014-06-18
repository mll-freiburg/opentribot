#ifndef _BEMERGENCYSTOP_H_
#define _BEMERGENCYSTOP_H_

#include "../../Behavior.h"

namespace Tribots {

  /** Hält den Roboter so schnell wie möglich an. Benötigt keine Informationen 
   *  aus dem Weltmodell. */
  class BEmergencyStop : public Behavior {
  public:
    BEmergencyStop();
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
  };

}

#endif 
