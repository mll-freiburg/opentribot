#ifndef _imagemonitor_h_
#define _imagemonitor_h_

#include "ImageBuffer.h"

namespace Tribots {

  /** abstrakte Schnittstelle fuer Bild-Wegspeicherer */
  class ImageMonitor {
  public:
    virtual ~ImageMonitor() {};
    /** Bild (arg1) wegspeichern; 
      (arg2): Zeit, zu der das Bild dem Programm vorlag, 
      (arg3): Zeit, zu dem das Bild mutmasslich aufgenommen wurde */
    virtual void monitor(const ImageBuffer&, 
			 const Time&, const Time&)=0;
  };

}

#endif
