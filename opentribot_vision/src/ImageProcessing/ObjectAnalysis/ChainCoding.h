#ifndef _chaincoding_h_
#define _chaincoding_h_

#include <vector>
#include "../Formation/Image.h"
#include "../../Fundamental/Vec.h"
#include "Regions.h"

namespace Tribots {

  /**
   * Schreitet eine Region (Zusammenhängende Pixel mit der gleichen Farbklasse)
   * ab und erzeugt einen Kettencode. Die Schrittrichtungen werden dabei wie
   * folgt kodiert: 0 rechts, 1 "oben" (größere y-Koordinate), 3 links, 4 unten
   *
   *    |1           y ^ 
   * 2  |              |
   * --- ---           |
   *    |  0           |
   *   3|              ------> x
   *
   * Imlpementierung mit 8er Nachbarschaft wäre auch noch möglich...
   */
  class ChainCoder {
  public:
    static int xDir[4];
    static int yDir[4];

    ChainCoder();

    virtual ~ChainCoder() {;}
    virtual int traceBorder(const Image& image, int x, int y, Region* region,
                            char* borderMap = 0);

    void setVisualize(Image* vis) { this->vis = vis; }
  protected:
    Image* vis;

    void visualizeRegion(const Region* region);
  };

  class RegionDetector 
  {
  public: 
    /**
     * Konstruktor, der mit einem Kettenkodierer initialisiert wird, der in 
     * den Besitz des RegionDetector übergeht.
     */
    RegionDetector(ChainCoder* cc = new ChainCoder());
    virtual ~RegionDetector();
    /**
     * findet alle Regionen der Farbe colClass im Bild und fügt sie an
     * die übergebene Liste an. Alle Randpixel des Bildes image müssen dabei 
     * von der Hintergrundklasse (0) sein!
     */
    virtual void findRegions (const Image& image, int colClass, 
			                        std::vector<Region>* regions) const;
  protected:
    char* buf;
    int size;
    ChainCoder* cc;
  };

};


#endif
