#include "BallDetector.h"
#include "ColorClasses.h"
#include "../Formation/Painter.h"
#include "../../Structures/Journal.h"
//#include "../../WorldModel/WorldModel.h"
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <cstring>

namespace Tribots {
  
  using namespace std;
  
  ImageParticleTracker::ImageParticleTracker(int colClass,
                                             int windowWidth,
                                             int windowHeight,
                                             int n,
                                             unsigned int minParticles)
  : colClass(colClass),  
  windowWidth(windowWidth),
  windowHeight(windowHeight), n(n), minParticles(minParticles),  vis(0)
{}

void
ImageParticleTracker::addParticle(const Vec& p)
{
  survivingparticles.push_back(p);
}

void
ImageParticleTracker::addSeedsAround(const Vec& x, int n)
{
  for (int i=0; i < n; i++) {
    Vec p = x + Vec( (rand() % windowWidth) - windowWidth / 2,
                     (rand() % windowHeight) - windowHeight / 2 );    
    survivingparticles.push_back(p);
  }
}
  

void
ImageParticleTracker::propagate(const Image& image)
{
  particles = survivingparticles;
  survivingparticles.clear();
  middle = Vec(0,0);        // reset middle particle
  
  if (particles.size() > 0) {
    for (int i=0; i < n; i++) {
      Vec p = particles[rand() % particles.size()];
      p = p + Vec( (rand() % windowWidth) - windowWidth / 2,
                   (rand() % windowHeight) - windowHeight / 2 );
      
      p.x = min(max(p.x, 0.), image.getWidth()-1.);  // clip at boundaries
      p.y = min(max(p.y, 0.), image.getHeight()-1.); // clip at boundaries
      
      int c = image.getPixelClass((int)(p.x+.5), (int)(p.y+.5));
      
      
      if (c==colClass) {
        survivingparticles.push_back(p);
          
        // sum up to calculate mean position:
        middle = middle + p;
      }
      if (vis) {
        visualizeSample((int)p.x, (int)p.y, c);
      }
    }
    
    if (survivingparticles.size() > 0) {
      middle /= (double) survivingparticles.size(); // calculate mean position
    }
    
#if 0 
// alle ballpunkt aufgrund von v-u-y finden
    YUVTuple yuv;
    YUVTuple setyuv;
    setyuv.y=42;
    setyuv.u=108;
    setyuv.v=162;
    
    int x1,x2,y1,y2;
    x1=(int)middle.x-windowWidth/2;
    x2=(int)middle.x+ windowWidth/2;
    y1=(int)middle.y-windowHeight/2;
    y2=(int)middle.y+ windowHeight/2;
    
    if(x1<0) x1=0; if(x1>image.getWidth()) x1=image.getWidth();
    if(y1<0) y1=0; if(y1>image.getHeight()) y1=image.getHeight();
    if(x2<0) x2=0; if(x2>image.getWidth()) x2=image.getWidth();
    if(y2<0) y2=0; if(y2>image.getHeight()) y2=image.getHeight();
    
    
    //TODO:aufpassen, nicht aus dem bild zu laufen 
    for(int i=(int)x1; i<x2; i++ ) {
	for(int j=(int)y1; j<y2; j++ ) {
    
		image.getPixelYUV(i,j, &yuv);
		
		
		if ((yuv.v - yuv.u -yuv.y/2)>5) {
			//survivingparticles.push_back(Vec(i,j));
			(*(Image*)(void*)&image).setPixelYUV(i,j, setyuv);
		
			if(vis) {
				Painter p(*vis);
				p.setColor(200,0,0); //ColorClassInfoList::getColorClassInfoList()->classList[c]->color);
				p.drawPoint(i+50,j);
			}
		} 
    
	}
    }    
   
#endif
    
  }    
  if (vis) {
    if (found()) visualizeCenter();
    vis = 0;
  }
}

void
ImageParticleTracker::visualizeCenter()
{
  Painter p(*vis);
  p.setPen(Painter::PEN_DOTTED);
  p.setColor(Painter::white);
  p.markRect((int)middle.x, (int)middle.y, 2);
}

void 
ImageParticleTracker::visualizeSample(int x, int y, int c)
{
  Painter p(*vis);
  p.setColor(ColorClassInfoList::getColorClassInfoList()->classList[c]->color);
  p.drawPoint(x,y);
}

bool
ImageParticleTracker::found() const
{
  return survivingparticles.size() >= minParticles; 
}

Vec
ImageParticleTracker::getPositionMean() const
{
  if (found()) {
    return middle;
  }
  else {
    throw TribotsException(__FILE__ ": No object found.");
  }
}

BallDetector::BallDetector(const RobotMask* rm, const ImageWorldMapping* mapping, 
                           const WorldImageMapping* world2image, bool returnAll) 
: robotMask(rm), mapping(mapping), world2image(world2image),
  tracker(COLOR_BALL), vis(0), returnAll(returnAll),bordermap(0), bordermapSize(0)
{}
   
   
int
BallDetector::searchBall (const Image& image,
                          const ScanResult* scanResult, Time time, RegionList* rl)
throw (TribotsException)
{
  if (scanResult->id != COLOR_BALL) {
    throw TribotsException (__FILE__
                            ": Expected results of color class COLOR_BALL "
                            "but received results of another class.");
  }
  vector<Vec> redPoints;
  for (unsigned int i=0; i < scanResult->points.size(); i++) {
    redPoints.push_back (scanResult->points[i]);
  }
  return searchBall (image, redPoints, time, rl);
}

int
BallDetector::searchBall(const Image& image, 
                         const vector<Vec> redPoints, Time time, RegionList* rl)
  throw (TribotsException)
{
  bool useScanlinesOnlyIfBallNotSeen = false; // should not be changed.. \todo: komplett weg damit, wenns ausreichend lange getestet ist

  if (bordermapSize != image.getWidth() * image.getHeight()) {
    if (bordermap) delete [] bordermap;
    bordermap = new char[image.getWidth() * image.getHeight()];
    bordermapSize = image.getWidth() * image.getHeight();
  }
  //for (int i=0; i < image.getWidth() * image.getHeight(); i++) {bordermap[i]=0; } // efence mag kein memset (alignment an int grenzen?)
  memset(bordermap, 0,  image.getWidth() * image.getHeight() * sizeof (char));
  vector<Region> regions;
  
  if (!tracker.found() || !useScanlinesOnlyIfBallNotSeen) { // if the particle tracker has not found the ball, add hypothesis from scanlines
    for (unsigned int i=0; i < redPoints.size(); i++) {
      tracker.addParticle(redPoints[i]);
    }
  }
  
  // add additional seeds to tracker, propagate and anaylize particles
  if (vis) {
    tracker.setVisualize(vis);
  }

  tracker.propagate(image);
  vector<Vec> allRedPoints = tracker.getParticles();
  for (unsigned int i=0; i < redPoints.size(); i++) {
    if (image.getPixelClass((int)(redPoints[i].x+.5),
                            (int)(redPoints[i].y+.5)) != COLOR_BALL) {
      stringstream s;
      s << "Im Ballfilter: Vom Linescanner uebergebener roter Punkt war gar nicht rot! " 
        << "Position: " << redPoints[i] << endl;
        
      JWARNING(s.str().c_str());
      continue;
    }
    allRedPoints.push_back(redPoints[i]); // auf jeden Fall die Punkte von
  }                              // den Scanlinien ueberpruefen

  vector<Vec> insideField;       // partikel sortieren nach denen, die sicher im
  vector<Vec> outsideField;      // feld liegen und denen, die ausserhalb sind
  for (unsigned int i=0; i < allRedPoints.size(); i++) {
      insideField.push_back(allRedPoints[i]);
    }
  
  vector<vector<Vec>*> pointLists;
  pointLists.push_back(&insideField);    // durchsuche die Punktlisten, die n-te aber nur,
  pointLists.push_back(&outsideField);   // wenn in der vorhergehenden gar keine Punkte gefunden wurden
  ChainCoder cc;
  int count=0;
  Region region;
  for (unsigned int pl=0; pl < pointLists.size() && count==0; pl++) {
    for (unsigned int i=0; i < pointLists[pl]->size(); i++) {  
      Vec point = (*pointLists[pl])[i];
      if (image.getPixelClass((int)(point.x+.5), 
                              (int)(point.y+.5)) != COLOR_BALL) {  // Initialer Ballpunkt nicht gefunden
        JWARNING("Im Ballfilter: Initialer Punkt war gar nicht rot! Das sollte eigentlich nicht passieren.");
        continue;
      }
      int ret = cc.traceBorder(image, (int)(point.x+.5), (int)(point.y+.5), &region, bordermap);
      if (ret<1 || region.getArea() < 4) {
        continue;   // fehlerhaft initialisiert oder bordermap getroffen oder einzelnes pixel oder sehr klein
      }
      if (vis) {
        visualizeRegion(region);
      }
      rl->list.push_back(region.clone());
      count++;
    }
  }
  if (vis) {
    vis = 0;
  }
  return count; // Anzahl der gefundenen potentiellen Ballregionen
}

LookForColorInSurrounding::LookForColorInSurrounding(const Image& image)
: image(image)
{}

Vec
LookForColorInSurrounding::search(const Vec& start, int color, 
                                  int maxDist) const
{
  int x = (int) start.x;
  int y = (int) start.y;
  
  if (image.getPixelClass(x,y) == color) {
    return start;
  }
  
  for (int i=1; i <= maxDist; i++) {
    for (int d=0; d < 4; d++) {
      
      int lx = x+i*ChainCoder::xDir[d];
      int ly = y+i*ChainCoder::yDir[d];
      
      if (lx < 0 || lx >= image.getWidth() ||  // auﬂerhalb?
          ly < 0 || ly >= image.getHeight()) {
        continue;
      }
      
      if (image.getPixelClass(lx, ly) == color) {
        return Vec(lx, ly);
      }
    }
  }
  /*    
    JWARNING("Have not found a pixel of the requested color in the "
             "surrounding of the start point.");
  */
  return start;
}

void BallDetector::visualizeBall(const Vec& pos)
{
  Painter p(*vis);
  p.setColor(ColorClassInfoList::getColorClassInfoList()->classList[COLOR_BALL]->color);
  p.markCrosshair((int)pos.x, (int)pos.y, 15);
  p.setColor(0,0,0);
  p.markCrosshair((int)pos.x+1, (int)pos.y+1, 15);
}

void BallDetector::visualizeRegion(const Region& region)
{
  Painter p(*vis);
  p.setColor(Painter::white);
  Vec point(region.x, region.y);
  for (unsigned int i=0; i < region.chainCode.size(); i++) {
    p.drawPoint((int)point.x, (int)point.y);
    point += Vec(ChainCoder::xDir[(int)region.chainCode[i]],
                 ChainCoder::yDir[(int)region.chainCode[i]]);
  }
  p.markCrossbar((int)region.getCenterOfGravity().x,
                 (int)region.getCenterOfGravity().y, 2);
}

};
