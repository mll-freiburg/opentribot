#ifndef _contourobstacledetector_h_
#define _contourobstacledetector_h_

#include "../PixelAnalysis/CoordinateMapping.h"
#include "../Formation/Image.h"
#include "ChainCoding.h"
#include "ObstacleDetector.h"
#include <vector>

// #define DEBUG_OBSTACLE
#define STATUS_OBSTACLE

namespace Tribots {

  class ContourObstacleDetector : public ObstacleDetector {
  public:
    /**
     * Erzeugt und initialisiert einen Obstacledetektor mit der übergebenen
     * Koordinatentransformation (Bild->Welt).
     *
     * \param mapping Koordinatentransformation von Bild zu egozentrischen
     *                Weltkoordinaten
     */
    ContourObstacleDetector(const ImageWorldMapping* mapping,
			    ChainCoder* cc,
			    double minTransitionSize = 4);

    virtual ~ContourObstacleDetector();

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
				 Time time = Time(),
				 VisibleObjectList* vol=0, 
				 bool writeWM = true)
      throw (TribotsException);
    
  protected:
    const ImageWorldMapping* mapping;  ///< Koordinatentransformation
    ChainCoder* cc;

    char* borderMap;
    int borderMapSize;

    double minTransitionSize;

  };
};


#endif
