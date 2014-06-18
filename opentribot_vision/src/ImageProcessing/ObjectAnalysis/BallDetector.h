#ifndef _balldetector_h_
#define _balldetector_h_

#include "LineScanning.h"
#include "ChainCoding.h"
#include "../PixelAnalysis/RobotMask.h"
#include "../PixelAnalysis/CoordinateMapping.h"
#include "../../Structures/VisibleObject.h"
#include "Regions.h"
#include <vector>

namespace Tribots {

  #if 1
  extern Vec __ballImgPos;
  #endif 

  using std::vector;

  /** Teil vom Ballfilter; eine Art evolutonaere Suche nach farbigen Pixeln */
  class ImageParticleTracker {
  public:
    ImageParticleTracker(int colClass, 
			 int windowWidth  = 40,
			 int windowHeight = 40,
			 int n = 200, 
			 unsigned int minParticles = 4);
    ~ImageParticleTracker() {;};

    void addParticle(const Vec& p);
    void addSeedsAround(const Vec& x, int n);
    void propagate(const Image& image);

    Vec getPositionMean() const;
    const vector<Vec>& getParticles() { return survivingparticles; }

    bool found() const;

    void setVisualize(Image* vis) { this->vis=vis; }
    
  protected:
    int colClass;
    Vec center;
    int windowWidth;
    int windowHeight;
    int n;
    unsigned int minParticles;

    std::vector<Vec> particles;
    std::vector<Vec> survivingparticles;

    Vec middle;      ///< mean of all positive particles
    Image* vis;

    void visualizeSample(int x, int y, int c);
    void visualizeCenter();
  };

  
  /** Ballerkenner arbeitet mit Pixelmengen, die zufaellig gestreut werden */
  class BallDetector {
  public:
    /**
     * Erzeugt und initialisiert einen Balldetektoren mit der übergebenen
     * Koordinatentransformation (Bild->Welt).
     *
     * \param mapping Koordinatentransformation von Bild zu egozentrischen
     *                Weltkoordinaten
     */
    BallDetector(const RobotMask*, const ImageWorldMapping* mapping,
                 const WorldImageMapping* world2image, bool returnAll = false);

    /**
     * Findet die Bälle in den übergebenen Resultaten des Linienscanners und
     * gibt die zugehörigen Regionen zurück. Dabei werden erste Filterstufen
     * angewendet:
     *   1. Gibt es mindestens einen Ball, dessen initialer Punkt innerhalb der
     *      Spielfeldgrenzen liegt, werden alle Bälle mit Punkten außerhalb des
     *      Spielfeldes ignoriert.
     *   2. <Filter über minimale Compactness, derzeit nicht angewendet>
     *
     * \param Image Bild, in dem gesucht werden soll
     * \param scanResults ScanResult Struktur der Farbklasse COLOR_LINE
     * \param time Zeitpunkt der Bildaufnahme, die für Anfragen an den Filter im WM
     *             verwendet wird
     * \returns Liste mit Regionen potentieller Bälle. Bereits vorgefiltert. Regionen
     *          gehen in den Besitz des Users über und sollten mit freeRegionList
     *          freigegeben werden.
     */
    int searchBall(const Image& image,
                   const ScanResult* scanResult, Time time, RegionList* rl)
      throw (TribotsException);
    int searchBall(const Image& image,
                   const vector<Vec> redPoints, Time time, RegionList* rl)
      throw (TribotsException);

    void setVisualize(Image* vis) { this->vis=vis; }

  protected:
    const RobotMask* robotMask;
    const ImageWorldMapping* mapping;  ///< Koordinatentransformation
    const WorldImageMapping* world2image;
    ImageParticleTracker tracker;
    Image* vis;
    
    bool returnAll;

    char* bordermap;
    int bordermapSize;

    void visualizeBall(const Vec& pos);
    void visualizeRegion(const Region& region);
  };

  class LookForColorInSurrounding {
  public:
    LookForColorInSurrounding(const Image& image);
    Vec search(const Vec& start, int color, int maxDist) const;    
  protected:
    const Image& image;
  };

};


#endif
