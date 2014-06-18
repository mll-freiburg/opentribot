#ifndef _obstacledetector_h_
#define _obstacledetector_h_

#include "../../Structures/TribotsException.h"
#include "../../Structures/VisibleObject.h"
#include "../../Fundamental/Time.h"
#include "../Formation/Image.h"
#include "LineScanning.h"

namespace Tribots {

  /** abstrakte Schnittstelle fuer verschiedene Hinderniserkenner */
  class ObstacleDetector {
  public:
    virtual ~ObstacleDetector() {;}

    /**
     *
     * \attention Ändert über einen Seiteneffekt das Weltmodell.
     *
     * \param image Aktuelles Bild
     * \param scanResults ScanResult Struktur der Farbklasse COLOR_LINE
     * \param vol Wird hier eine VisibleObjectList übergeben, werden die
     *            Objekte nicht nur ins Weltmodell sondern auch an diese Liste 
     *            angehängt.
     * \param writeWM gibt an, ob die gefundenen Objekte ins Weltmodell
     *                geschrieben werden sollen
     */
    virtual void searchObstacles(const Image& image,
				 const ScanResult* scanResult, 
				 Time time,
				 VisibleObjectList* vol)
      throw (TribotsException) =0;

    /** trage die Ergebnisse/Zwischenergebnisse in das Bild ein */
    virtual void setVisualize(Image*) {;}
  };
};


#endif
