#include "ChainCoding.h"
#include "../../Structures/Journal.h"
#include <cstdlib>
#include <iostream>
#include <cstring>
namespace Tribots {
  
  using namespace std;

  int ChainCoder::xDir[4] = {  1,  0, -1,  0 };
  int ChainCoder::yDir[4] = {  0,  1,  0, -1 };  

  ChainCoder::ChainCoder() : vis(0) 
  {}

  int 
  ChainCoder::traceBorder(const Image& image, int x, int y, Region* region,
			  char* borderMap)
  {
    if (x!=x || y!=y) {
      JERROR("started traceBorder with nan position!");
      return 0;
    }
    
    int width  = image.getWidth();
    int height = image.getHeight();
    
    if (x < 0 || y < 0 || x >= image.getWidth() || y >= image.getHeight()) {
      JERROR("started traceBorder with position outside image!");
      Tribots::Journal::the_journal.flush();
      return 0;
    }
    
    int dir;                      // Richtung des letzten Schritts
    int colClass =
      image.getPixelClass(x,y);   // Farbe der abzuschreitenden Region

    *region = Region(colClass, x, y);

    for (dir=0; dir < 4; dir++) { // check, dass kein einzelnes Pixel
      if (image.getPixelClass(x+xDir[dir], y+yDir[dir]) == colClass) {
	break;
      }
    }
    if (dir == 4) {               // es ist ein einzelnes Pixel!
      if (vis) vis = 0;
      return 1;
    }

    // Startpunkt bestimmen: es muss sichergestellt werden, dass die Region
    // immer gegen den Uhrzeigersinn abgeschritten wird, egal an welcher 
    // Stelle sie von der Scanlinie getroffen wurde.

    do {
      for (dir=0; dir < 4; dir++) {// suche ein andersfarbiges Pixel
        if (image.getPixelClass(x+xDir[dir], y+yDir[dir]) != colClass) {
	  break;
        }
      }

      if (dir == 4) {             // Punkt ist kein Randpunkt!
        while (image.getPixelClass(x-1, y) == colClass) {
          x--;
	}
      }
    } while (dir == 4);

    region->x = x;

    if (borderMap && borderMap[x+y*width] > 0) {
      if (vis) vis = 0;
      return 0;                   // dieser Rand wurde schon einmal bearbeitet
    }
    
    dir++;                        // in der schleife wird einen zurückgedreht

    // gegen den Uhrzeigersinn nach nächstem Pixel suchen (einen Schritt
    // vor der letzten Schrittrichtung beginnen. Also bei Schritt nach unten ->
    // links mit der Suche anfangen), gegen den Uhrzeigersinn abschreiten, 
    // bis am Ausgangspunkt (in richtiger Richtung) angekommen. 
    // Test: Aktuelle Position auf zweitem Punkt der Kette, vorige Position
    // (-dir) ist Ausgangspunkt region->x, region->y

    do {
      int sDir;
      int xTmp, yTmp;
      for (sDir = (dir+3) % 4; ; sDir = (sDir+1) % 4) {
	xTmp = x+xDir[sDir];
	yTmp = y+yDir[sDir];
	if (xTmp < 0 || xTmp >= width || yTmp < 0 || yTmp >= height) {
	  continue;
	}
	if (image.getPixelClass(xTmp, yTmp) == colClass) {
	  break;                  // nächstes Pixel auf Rand gefunden
	}
      }
      x+=xDir[sDir];
      y+=yDir[sDir];
      dir = sDir;

      if (borderMap) {
	borderMap[x+y*width] = 1; // Punkt markieren
      }

      region->chainCode.push_back(dir);

      region->minX = min(region->minX, x);
      region->maxX = max(region->maxX, x);
      region->minY = min(region->minY, y);
      region->maxY = max(region->maxY, y);
    } while (region->chainCode.size() < 2 ||  // Kann noch nicht rum sein
	     x != region->x+xDir[static_cast<int>(region->chainCode[0])] ||
	     y != region->y+yDir[static_cast<int>(region->chainCode[0])] ||
	     x-xDir[dir] != region->x ||
	     y-yDir[dir] != region->y);
    
    region->chainCode.pop_back(); // letzter Schritt war mit erstem identisch

    if (vis) {
      visualizeRegion(region);
      vis = 0;
    }
    if (region->getArea() <= 1) {    // holes in regions (e.g. spotlights) have negative size
      return 0;
    }

    return 1;
  }

  void
  ChainCoder::visualizeRegion(const Region* region)
  {
    RGBTuple white = {255, 255, 255};
    RGBTuple blue =  {  0,   0, 255};
    int x = region->x; int y = region->y;
    for (unsigned int i=0; i < region->chainCode.size(); i++) {
      vis->setPixelRGB(x,y, white);
      x += xDir[(int)region->chainCode[i]];
      y += yDir[(int)region->chainCode[i]];
    }
    // mark start of chaincode with black cross
    for (int i=-1; i <= 1; i++) {
      vis->setPixelRGB(region->x+i, region->y, blue);
      vis->setPixelRGB(region->x, region->y+i, blue);
    }
  }

  RegionDetector::RegionDetector(ChainCoder* cc)
    : buf(0), size(0), cc(cc)
  {}

  RegionDetector::~RegionDetector()
  {
    if (buf) delete [] buf;
    if (cc) delete cc;
  }

  void
  RegionDetector::findRegions (const Image& image, int colClass,
			       vector<Region>* regions) const
  {
    int w = image.getWidth();
    int h = image.getHeight();

    if (size != w*h) {         // puffer neu anlegen
      if (buf) {
	delete [] const_cast<RegionDetector*>(this)->buf;
      }
      const_cast<RegionDetector*>(this)->buf = new char[w*h];
      const_cast<RegionDetector*>(this)->size = w*h;
    }
    
    memset(buf, 0, sizeof(char)*size); // puffer leeren
    

    int actClass = 0;          // aktuelle Farbklasse
    Region region;

    for (int x=1; x < w-1; x++) {
      for (int y=1; y < h-1; y++) {
	int newClass = image.getPixelClass(x,y);
	if (actClass != colClass &&      // nicht in farbe gewesen
	    newClass == colClass &&      // jetzt aber in farbe eingetreten
	    buf[x+y*w] == 0) {           // und dieser rand noch nicht verfolgt
	  if (cc->traceBorder(image, x,y, &region, buf)) { // verfolge
	    if (region.getArea() >= 0) {  // nur merken, wenn area größer 0,
	      regions->push_back(region); // denn bei area<0 handelt es sich
	    }                             // um ein loch!
	  }
	}
	actClass = newClass;
      }
    }    
  }

};

