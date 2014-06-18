
#include "ImageMaskBuilder.h"
#include <cmath>
#include <algorithm>
#include <deque>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

int min(int a, int b){ return a<b ? a : b; }
int max(int a, int b){ return a>b ? a : b; }

namespace {
  double square (double d) {
    return d*d;
  }

  struct Pos {
    int x;
    int y;
    Pos (int x1=0, int y1=0) : x(x1), y(y1) {;}
    Pos (const Pos& p) : x(p.x), y(p.y) {;}
    const Pos& operator= (const Pos& p) { x=p.x; y=p.y; return *this; }
    bool operator< (const Pos& p) const { return (y<p.y || (y==p.y && x<p.x)); }
    bool operator== (const Pos& p) const { return (x==p.x) && (y==p.y); }
    bool operator!= (const Pos& p) const { return (x!=p.x) || (y!=p.y); }
  };

  struct MArea {
    deque<Pos>* pixels; // aufsteigend geordnet
    deque<Pos>::const_iterator itUpper;
    bool ofInterest;  // bei der Suche nach benachbarten Pixeln ueberhaupt noch von Interesse?

    MArea (const Pos& p) : pixels (new deque<Pos>) { pixels->push_back (p); itUpper=pixels->begin(); ofInterest=true; }
    ~MArea () { delete pixels; }
    unsigned int size () const { return pixels->size(); }
    void push (const Pos& p) {
      unsigned int i=itUpper-pixels->begin();
      pixels->push_back(p); // Achtung, hierbei koennte der Iterator itUpper kaputt gehen, daher anschliessend wieder hinbiegen
      itUpper=pixels->begin()+i;
    }
    bool containsUpper (const Pos& p) {
      if (!ofInterest)
        return false;
      deque<Pos>::const_iterator itEnd=pixels->end();
      while (itUpper!=itEnd && (itUpper->y<p.y || (itUpper->y==p.y && itUpper->x<p.x))) {
        ++itUpper;
      }
      if (itUpper==itEnd)
        ofInterest=false;
      return (ofInterest && itUpper->x==p.x);
    }
    void mergeDestroy (MArea& a) {
      Pos upperPos1;
      Pos upperPos2;
      if (itUpper==pixels->end())
        upperPos1 = *(pixels->end()-1);
      else
        upperPos1 = *itUpper;
      if (a.itUpper==a.pixels->end())
        upperPos2 = *(a.pixels->end()-1);
      else
        upperPos2 = *a.itUpper;
      if (pixels->back()<a.pixels->front()) {
        pixels->insert (pixels->end(), a.pixels->begin(), a.pixels->end());
      } else if (a.pixels->back()<pixels->front()) {
        deque<Pos>* pt = pixels;
        pixels=a.pixels;
        a.pixels=pt;
        pixels->insert (pixels->end(), a.pixels->begin(), a.pixels->end());
      } else {
        deque<Pos>* pt = new deque<Pos> (pixels->size()+a.pixels->size());
        std::merge (pixels->begin(), pixels->end(), a.pixels->begin(), a.pixels->end(), pt->begin());
        delete pixels;
        pixels=pt;
      }
      itUpper=lower_bound (pixels->begin(), pixels->end(), upperPos1<upperPos2 ? upperPos1 : upperPos2);
    }
  };
  void findAreas (deque<MArea* >& areas, const RobotMask mask, bool on) {
    int width=mask.getWidth();
    int height=mask.getHeight();
    MArea* latestArea=NULL;  // wenn letztes Pixel eingefuegt wurde, Pointer auf die entsprechende MArea, sonst NULL
    for (int j=0; j<height; j++) {
      latestArea=NULL;  // am Zeilensprung zuruecksetzen, da kein linker Nachbar vorhanden
      for (int i=0; i<width; i++) {
        if (mask.isValid(i,j)!=on) {
          latestArea=NULL;  // latestArea zuruecksetzen, da nicht eingefuegt wurde
          continue;
        }
//        cerr << '(' << i << ',' << j << ")  " << (latestArea!=NULL) << ' ' << areas.size();
        if (latestArea) {
          latestArea->push (Pos(i,j));  // latestArea bleibt unveraendert
//          cerr << " append";
          // weitersuchen, denn es koennte sein, dass es ein Merger geben muss
        }
        deque<MArea* >::reverse_iterator it = areas.rbegin();
        deque<MArea* >::reverse_iterator itEnd = areas.rend();
        Pos upperPos (i, j-1);
        while (it!=itEnd) {
          if ((*it)!=latestArea && (*it)->containsUpper (upperPos))
            break;
          ++it;
        }
        if (it==itEnd) {
          // keine benachbarte Region gefunden
          if (!latestArea) {
            // Punkt noch nicht eingefuegt -> neue Area einfuegen
//            cerr << " newArea" << areas.size();
            latestArea=new MArea (Pos(i,j));
            areas.push_back (latestArea);
          } // ansonsten nichts tun, da Punkt bereits eingefuegt wurde
        } else {
          // eine benachbarte Region gefunden
          if (!latestArea) {
            // Punkt noch nicht eingefuegt -> einfuegen
            latestArea = (*it);
            latestArea->push (Pos(i,j));
//            cerr << " insert" << areas.size()-1-(it-areas.rbegin());
          } else {
            // Punkt bereits in latestArea eingefuegt und existiert weitere benachbarte Region -> merge
//            cerr << " merge" << areas.size()-1-(it-areas.rbegin());
            latestArea->mergeDestroy (**it);  // Bereiche mergen
            delete (*it); // die obsolete MArea entfernen
            (*it) = areas.back();  // die letzte MArea an den Platz der geloschten MArea haengen
            areas.pop_back ();  // das letzte Element aus der MArea-Liste loschen, da dieses zuvor an den frei gewordenen Platz gehaengt wurde
          }
        }
//        cerr << '\n';
      }
    }
  }
  void setArea (RobotMask& mask, const::deque<Pos>& area, bool on) {
    for (deque<Pos>::const_iterator it = area.begin(); it!=area.end(); it++) {
      mask.set(it->x, it->y, on);
    }
  }
  void removeSmallAreas (RobotMask& mask, bool on, unsigned int minsize) {
    deque<MArea* > areas;
    findAreas (areas, mask, on);
    for (unsigned int i=0; i<areas.size(); i++) {
      if (areas[i]->size()<minsize)
        setArea (mask, *(areas[i]->pixels), !on);
      delete areas[i];
    }
  }
}

ImageMaskBuilder::ImageMaskBuilder () {
  init (640,480);
}

ImageMaskBuilder::~ImageMaskBuilder () {;}

unsigned int ImageMaskBuilder::numSamples () const throw () {
  return num;
}

void ImageMaskBuilder::init (unsigned int width1, unsigned int height1) {
  width=width1;
  height=height1;
  num=0;
  g.assign (width*height, 0);
  gg.assign (width*height, 0);
}

void ImageMaskBuilder::addImage (const Tribots::Image& img) {
  if (width!=static_cast<unsigned int>(img.getWidth()) || height!=static_cast<unsigned int>(img.getHeight())) {
    init (img.getWidth(), img.getHeight());
  }
  num++;
  vector<double>::iterator itg = g.begin();
  vector<double>::iterator itgg = gg.begin();
  YUVTuple yuv;
  for (unsigned int j=0; j<height; j++) {
    for (unsigned int i=0; i<width; i++) {
      img.getPixelYUV (i,j,&yuv);
      (*itg++)+=yuv.y;
      (*itgg++)+=yuv.y*yuv.y;
    }
  }
}

Tribots::RobotMask* ImageMaskBuilder::generateMask (double threshold) {
  RobotMask* mask1 = new RobotMask (width, height);
  const double thresh2=threshold*threshold;
  if (num>10) {
    vector<double>::iterator itg = g.begin();
    vector<double>::iterator itgg = gg.begin();
    double dnum=num;
    for (unsigned int j=0; j<height; j++) {
      for (unsigned int i=0; i<width; i++) {
        double var = (*itgg++)/dnum-square(*itg++/dnum);
        mask1->set (i,j,var>thresh2);
      }
    }
  }
  return mask1;
}

Tribots::RobotMask* ImageMaskBuilder::dilateMask (const Tribots::RobotMask* mask, unsigned int numDilate) {
  if (numDilate==0) {
    return new RobotMask (*mask);
  }
  RobotMask* mask2;
  RobotMask* mask1=NULL;
  for (unsigned int k=0; k<numDilate; k++) {
    if (mask1) {
      mask2=mask1->dilate();
      delete mask1;
    } else {
      mask2=mask->dilate();
    }
    mask1=mask2;
  }
  removeSmallAreas (*mask1, false, 3000);
  removeSmallAreas (*mask1, true, 3000);
  mask1->addFrame ();
  return mask1;
}


// #include <fstream>
// #include "../../ImageProcessing/Formation/RGBImage.h"
// int main () {
//   RobotMask mask ("testmask.ppm");
//   deque<MArea* > areas1, areas2;
//   findAreas (areas1, mask, true);
//   findAreas (areas2, mask, false);
//   Tribots::RGBImage img (640,480);
//   Tribots::RGBTuple rgb;
//   for (unsigned int i=0; i<areas1.size(); i++) {
//     cerr << "Area " << i << ": " << areas1[i]->size() << '\n';
//     unsigned int c = 20*(i+3);
//     if (c>255) {
//       rgb.r=255;
//       c-=255;
//     } else {
//       rgb.r=c;
//       c=0;
//     }
//     if (c>255) {
//       rgb.g=255;
//       c-=255;
//     } else {
//       rgb.g=c;
//       c=0;
//     }
//     rgb.b=c;
//     for (unsigned int j=0; j<areas1[i]->size(); j++)
//       img.setPixelRGB (areas1[i]->pixels->operator[](j).x, areas1[i]->pixels->operator[](j).y, rgb);
//   }
//   for (unsigned int i=0; i<areas2.size(); i++) {
//     cerr << "Area " << i << ": " << areas2[i]->size() << '\n';
//     unsigned int c = 20*(i+3);
//     if (c>255) {
//       rgb.b=255;
//       c-=255;
//     } else {
//       rgb.b=c;
//       c=0;
//     }
//     if (c>255) {
//       rgb.g=255;
//       c-=255;
//     } else {
//       rgb.g=c;
//       c=0;
//     }
//     rgb.r=c;
//     for (unsigned int j=0; j<areas2[i]->size(); j++)
//       img.setPixelRGB (areas2[i]->pixels->operator[](j).x, areas2[i]->pixels->operator[](j).y, rgb);
//   }
//   ofstream dest ("testmask1.ppm");
//   dest << "P6 640 480 255\n";
//   for (unsigned int j=0; j<480; j++) {
//     for (unsigned int i=0; i<640; i++) {
//       img.getPixelRGB(i,j,&rgb);
//       dest << rgb.r << rgb.g << rgb.b;
//     }
//   }
//   dest << flush;
//   ofstream dest2 ("testmask2.ppm");
// //  removeSmallAreas (mask, false, 3000);
// //  removeSmallAreas (mask, true, 3000);
//   mask.writeToStream (dest2);
//   dest2 << flush;
//   return 0;
// }
