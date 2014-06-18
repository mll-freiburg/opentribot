#ifndef _simpleobstacledetector_h_
#define _simpleobstacledetector_h_

#include "ObstacleDetector.h"
#include "SimpleCluster.h"
#include "../PixelAnalysis/CoordinateMapping.h"
#include "../../Fundamental/ConfigReader.h"

namespace Tribots {

  /** Hinderniserkenner, der im Wesentlichen die als schwarz erkannten Bereiche
    der Scanlinien auswertet und clustert */
  class SimpleObstacleDetector : public ObstacleDetector {
  public:
    /**
     * Erzeugt und initialisiert einen Obstacledetektor mit der übergebenen
     * Koordinatentransformation (Bild->Welt).
     *
     * \param mapping Koordinatentransformation von Bild zu egozentrischen
     *                Weltkoordinaten
     */
    SimpleObstacleDetector(const ConfigReader& config,
			   const ImageWorldMapping* mapping,
			   const WorldImageMapping* rel2img,
			   double minTransitionSize = 4);

    virtual ~SimpleObstacleDetector();

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
    virtual void searchObstacles(const Image&,
				 const ScanResult* scanResult, 
				 Time time,
				 VisibleObjectList* vol)
      throw (TribotsException);

    virtual void setVisualize(Image* vis) { this->vis=vis; }
    
  protected:
    const ImageWorldMapping* mapping;  ///< Koordinatentransformation
    const WorldImageMapping* rel2img;  ///< Relative Roboterkoordinaten in Bildkoordinaten
    double minTransitionSize;
    double clustering_thresh;
    double min_width;

    Image* vis;


    virtual void paintResults(SimpleVecCluster& cluster);
  };
};


#endif
