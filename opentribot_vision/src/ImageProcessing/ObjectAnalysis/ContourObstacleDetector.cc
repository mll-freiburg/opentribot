#include "ContourObstacleDetector.h"
#include "ColorClasses.h"
#include <cstdlib>
#include <cstring>

namespace Tribots {
  
  using namespace std;

  ContourObstacleDetector
  ::ContourObstacleDetector(const ImageWorldMapping* mapping, 
			    ChainCoder* cc,
			    double minTransitionSize) 
    : ObstacleDetector(), mapping(mapping), cc(cc), borderMap(0), 
      borderMapSize(0), minTransitionSize(minTransitionSize)
  {} 

  ContourObstacleDetector::~ContourObstacleDetector()
  {
    if (borderMap != 0) {
      delete [] borderMap;
    }
  }
  
  void
  ContourObstacleDetector::searchObstacles(const Image& image, 
					   const ScanResult* scanResult, 
					   Time time, 
					   VisibleObjectList* vol, 
					   bool writeWM)
    throw (TribotsException)
  {
    int width  = image.getWidth();
    int height = image.getHeight();

    // der object-tracer erwarten am bildrand pixel einer anderen farbklasse..

    Image& nCImage = const_cast<Image&>(image);

    RGBTuple rgb = { 255, 0, 0 };
    int X = image.getWidth()-1;
    int Y = image.getHeight()-1;
    
    for (int x=0; x<=X; x++) {
      nCImage.setPixelRGB(x,0, rgb);
      nCImage.setPixelRGB(x,Y, rgb);
    }
    for (int y=0; y<=Y; y++) {
      nCImage.setPixelRGB(0,y, rgb);
      nCImage.setPixelRGB(X,y, rgb);
    }

    if (scanResult->id != COLOR_OBSTACLE) {
      throw TribotsException(__FILE__ 
			     ": Expected results of color class "
			     "COLOR_OBSTACLE but received results of another "
			     "class.");
    }

    // borderMap vorbereiten 
    if (width * height != borderMapSize) {
      if (borderMap != 0) {
	delete [] borderMap;
      }
      borderMap = new char[width * height];
      borderMapSize = width * height;
    }
    memset(borderMap, 0, width * height * sizeof(char) ); // clear

    Vec obstaclePosition;
    Region region;

    for (unsigned int i=0; i < scanResult->transitions.size(); i+=2) {

      if (scanResult->transitions.size() <= i+1) { // a pair left?
	throw TribotsException(__FILE__
			       ": The transition list is not organized "
			       "pair-wise. One end transision is missng.");
      }

      const Transition& transStart = scanResult->transitions[i];
      const Transition& transEnd   = scanResult->transitions[i+1];
      
      if (transStart.type != Transition::START ||
	  transEnd.type   != Transition::END) {
	throw TribotsException(__FILE__
			       ": The transition list is not organized "
			       "pair-wise. Each start transtition has to be "
			       "followed by exactly one endtransision!");
      }

      if (transStart.virt == true || transEnd.virt == true) { // skip virtual
	continue;
      }
      
#ifdef DEBUG_OBSTACLE    
      cerr << "Transition at " << transStart.toPos.x << ", "
	   << transStart.toPos.y << "\r\n";
#endif
      
      // schauen, ob kannte schon verarbeitet wurde
      if (borderMap[((int)transStart.toPos.x) + 
		    ((int)transStart.toPos.y) * width] > 0) {
#ifdef DEBUG_OBSTACLE    
      cerr << "Border Test failed. Skip transition\r\n";
#endif
	continue;
      }
      
      // plausibilitätstest, minimale Breite erforderlich
      if ((transStart.toPos - transEnd.fromPos).length() < minTransitionSize) {
	continue;
      }

      if (! cc->traceBorder(image, 
		            (int)transStart.toPos.x, (int)transStart.toPos.y,
		            &region, borderMap)) {   // keine Region (zum Beispiel schon gesehen)
        continue;
      }

      // Untersuchung der gefundenen region; \todo fläche, breite?!
      if (region.maxX - region.minX < 2 ||
	  region.maxY - region.minY < 2) {
	continue;
      }

      // Position berechnen; \todo nicht einfach den ersten Punkt nehmen
      obstaclePosition = mapping->map(Vec(region.x, region.y));
      
#ifdef STATUS_OBSTACLE
      cerr << "Obstacle at relative position: " << obstaclePosition.x << ", " 
	   << obstaclePosition.y << "\r\n";
#endif

      if (vol) {
	vol->objectlist.push_back(
	  VisibleObject (obstaclePosition, VisibleObject::ball)
	  );
      }
/*      if (writeWM) {
	WorldModel::get_main_world_model().set_visual_information(
	  VisibleObject (obstaclePosition, VisibleObject::obstacle), time
	);
      }
*/
    
}
  }
};


