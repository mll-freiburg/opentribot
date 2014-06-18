#ifndef _polarobstacledetector_h_
#define _polarobstacledetector_h_

#include "ObstacleDetector.h"
#include "../PixelAnalysis/CoordinateMapping.h"
#include "../../Fundamental/ConfigReader.h"
#include "PolarObstacleCluster.h"

namespace Tribots {

  /** Hinderniserkenner, der im Wesentlichen die als schwarz erkannten Bereiche
    der Scanlinien auswertet und clustert */
  class PolarObstacleDetector : public ObstacleDetector {
  public:
    /**
     * Erzeugt und initialisiert einen Obstacledetektor mit der übergebenen
     * Koordinatentransformation (Bild->Welt).
     *
     * \param mapping Koordinatentransformation von Bild zu egozentrischen
     *                Weltkoordinaten
     */
    PolarObstacleDetector(const ConfigReader& config,
			  const ImageWorldMapping* mapping,
			  const WorldImageMapping* rel2img,
			  double minTransitionSize = 4);
    
    virtual ~PolarObstacleDetector();

    /**
     * \param image Aktuelles Bild
     * \param scanResults ScanResult Struktur der Farbklasse COLOR_LINE
     * \param vol Wird hier eine VisibleObjectList übergeben, werden die
     *            Objekte nicht nur ins Weltmodell sondern auch an diese 
     *            Liste angehängt.
     */
    virtual void searchObstacles(const Image&,
				 const ScanResult* scanResult, 
				 Time time,
				 VisibleObjectList* vol)
      throw (TribotsException);
    
    virtual void setVisualize(Image* vis) { this->vis=vis; }
    
  protected:
    const  ImageWorldMapping* mapping;  ///< Koordinatentransformation
    const  WorldImageMapping* rel2img;  ///< Relative Roboterkoordinaten in Bildkoordinaten
    double minTransitionSize;

    Image*                vis;
    PolarObstacleCluster* pcluster;
    
    virtual void paintResults(std::vector< Obstacle > &);
  };
};


#endif
