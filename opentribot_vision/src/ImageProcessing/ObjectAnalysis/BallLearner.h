#ifndef _balllearner_h_
#define _balllearner_h_

#include "LineScanning.h"
#include "ChainCoding.h"
#include "../PixelAnalysis/RobotMask.h"
#include "../PixelAnalysis/CoordinateMapping.h"
#include "../../Structures/VisibleObject.h"
#include "Regions.h"
#include <vector>
#include "../PixelAnalysis/YUVLookupLearner.h"
#include "opentribot_messages/VisionSignal.h"
#include "ColorClasses.h"
#include "ros/ros.h"

namespace Tribots {

  #if 1
  extern Vec __ballImgPos;
  #endif 

  using std::vector;


  class Accumulator{
	  public:
	  int shiftY;   ///< Anzahl der Bits, um den die Y-Werte geschiftet werden
	    int shiftU;   ///< Anzahl der Bits, um den die Y-Werte geschiftet werden
	    int shiftV;   ///< Anzahl der Bits, um den die Y-Werte geschiftet werden
	    int sizeY;    ///< sizeY = 255 >> shiftY
	    int sizeU;    ///< sizeU = 255 >> shiftU
	    int sizeV;    ///< sizeV = 255 >> shiftV
	    int size;     ///< Gesamtzahl der Eintr�ge in der Nachschlagetabelle

	    Accumulator(){
	    	shiftY=2;
	    	shiftU=2;
	    	shiftV=2;
	    	sizeY=255>>shiftY;
	    	sizeU=255>>shiftU;
	    	sizeV=255>>shiftV;
	    	size=sizeY*sizeU*sizeV;
	    	lutpositive=new double[size];
	    	lutnegative=new double[size];
	    	lutdiff=new double[size];
	    }
	    void add_negative(YUVTuple *yuv){
	    	lutpositive[((yuv->y >> shiftY) * sizeU +
	    		 (yuv->u >> shiftU)) * sizeV +
	    		(yuv->v >> shiftV)]+=1;
	    }
	    void add_positive(YUVTuple *yuv){
	    	lutnegative[((yuv->y >> shiftY) * sizeU +
	    		    		 (yuv->u >> shiftU)) * sizeV +
	    		    		(yuv->v >> shiftV)]+=1;
	    }
	    void normalize(){
	    	int maxp=0;
	    	int maxn=0;

	    	for (int i=0;i< size;i++){
	    		if (lutpositive[i]>maxp)maxp=lutpositive[i];
	    		if (lutnegative[i]>maxn)maxn=lutnegative[i];
	    	}
	    	for (int i=0;i< size;i++){
	    		lutpositive[i]=lutpositive[i]/maxp;
	    		lutnegative[i]=lutnegative[i]/maxn;
	    	}
	    }
	    void calc_diff(){
	    	for (int i=0;i<size;i++){
	    		lutdiff[i]=lutpositive[i]-lutnegative[i];

	    	}

	    }


	    double* lutpositive;
	    double* lutnegative;
	    double* lutdiff;




  };













  /** Teil vom Ballfilter; eine Art evolutonaere Suche nach farbigen Pixeln */
  class ImageParticleTracker_ {
  public:
    ImageParticleTracker_(int colClass,
			 int windowWidth  = 40,
			 int windowHeight = 40,
			 int n = 200, 
			 unsigned int minParticles = 4);
    ~ImageParticleTracker_() {;};

    void addParticle(const Vec& p);
    void addSeedsAround(const Vec& x, int n);
    void propagate(const Image& image);


    Vec getPositionMean() const;
    const vector<Vec>& getParticles() { return survivingparticles; }

    bool found() const;

    void setVisualize(Image* vis) { this->vis=vis; }
    
    YUVLookupLearner yuvlearner;



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
  class BallLearner{
  public:
    /**
     * Erzeugt und initialisiert einen Balldetektoren mit der �bergebenen
     * Koordinatentransformation (Bild->Welt).
     *
     * \param mapping Koordinatentransformation von Bild zu egozentrischen
     *                Weltkoordinaten
     */
    BallLearner(const RobotMask*, const ImageWorldMapping* mapping,
                 const WorldImageMapping* world2image, bool returnAll = false);

    /**
     * Findet die B�lle in den �bergebenen Resultaten des Linienscanners und
     * gibt die zugeh�rigen Regionen zur�ck. Dabei werden erste Filterstufen
     * angewendet:
     *   1. Gibt es mindestens einen Ball, dessen initialer Punkt innerhalb der
     *      Spielfeldgrenzen liegt, werden alle B�lle mit Punkten au�erhalb des
     *      Spielfeldes ignoriert.
     *   2. <Filter �ber minimale Compactness, derzeit nicht angewendet>
     *
     * \param Image Bild, in dem gesucht werden soll
     * \param scanResults ScanResult Struktur der Farbklasse COLOR_LINE
     * \param time Zeitpunkt der Bildaufnahme, die f�r Anfragen an den Filter im WM
     *             verwendet wird
     * \returns Liste mit Regionen potentieller B�lle. Bereits vorgefiltert. Regionen
     *          gehen in den Besitz des Users �ber und sollten mit freeRegionList
     *          freigegeben werden.
     */

    void updateCommands(const::opentribot_messages::VisionSignal::ConstPtr & msg);



    int searchBall(const Image& image,
                   const ScanResult* scanResult, Time time, RegionList* rl)
      throw (TribotsException);
    int searchBall(const Image& image,
                   const vector<Vec> redPoints, Time time, RegionList* rl)
      throw (TribotsException);

    void setVisualize(Image* vis) { this->vis=vis; }

    ros::Subscriber sub;
    int gatherNegative;
    int gatherPositive;
    int calculateDifference;
    Accumulator accu;
    int counter;
    int state;
  protected:
    const RobotMask* robotMask;
    const ImageWorldMapping* mapping;  ///< Koordinatentransformation
    const WorldImageMapping* world2image;
    ImageParticleTracker_ tracker;
    Image* vis;
    const Image* imagesave;

    bool returnAll;

    char* bordermap;
    int bordermapSize;

    void visualizeBall(const Vec& pos);
    void visualizeRegion(const Region& region);
    void visualizeImage();
  };

  class LookForColorInSurrounding_ {
  public:
    LookForColorInSurrounding_(const Image& image);
    Vec search(const Vec& start, int color, int maxDist) const;    
  protected:
    const Image& image;
  };

};


#endif
