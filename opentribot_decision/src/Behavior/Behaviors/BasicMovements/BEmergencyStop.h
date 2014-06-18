#ifndef _BEMERGENCYSTOP_H_
#define _BEMERGENCYSTOP_H_

#include "../../Behavior.h"

namespace Tribots {

  /** H�lt den Roboter so schnell wie m�glich an. Ben�tigt keine Informationen 
   *  aus dem Weltmodell. */
  class BEmergencyStop : public Behavior {
  public:
    BEmergencyStop();
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
  };

}

#endif 
