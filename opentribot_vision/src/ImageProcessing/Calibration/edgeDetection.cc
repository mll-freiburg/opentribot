
#include <cmath>
#include <algorithm>
#include "Convolution.h"
#include "edgeDetection.h"

using namespace std;
using namespace Tribots;

namespace {

  int dxs [] = { +1, +1, 0, -1, -1, -1, 0, +1, +1, +1, 0, -1, -1, -1, 0, +1 };
  int dys [] = { 0, +1, +1, +1, 0, -1, -1, -1, 0, +1, +1, +1, 0, -1, -1, -1 };
  inline void neighborIndeces (int& dx, int& dy, float direction) {
    int index = static_cast<int>((direction+0.392699f)/0.785398f);
    dx=dxs[index];
    dy=dys[index];
  }

  bool around (const GrayLevelImage& gradient, int x, int y, float threshold) {
    unsigned int numActive=0;
    if (gradient(x-1,y)>=threshold) numActive++;
    if (gradient(x+1,y)>=threshold) numActive++;
    if (gradient(x,y-1)>=threshold) numActive++;
    if (gradient(x,y+1)>=threshold) numActive++;
    if (gradient(x-1,y-1)>=threshold) numActive++;
    if (gradient(x+1,y-1)>=threshold) numActive++;
    if (gradient(x-1,y+1)>=threshold) numActive++;
    if (gradient(x+1,y+1)>=threshold) numActive++;
    return numActive>=2;
  }

  void followEdge (GrayLevelImage& gradientMaxima, const GrayLevelImage& gradientDirection, int x, int y, int mode, float threshold, Pixelset& newList) {
    // mode kontrolliert die Suchrichtung. Es gilt:
    // +1 : hinten anfuegen und weiterverketten
    // +2: hinten anfuegen und stoppen
    // 0: Pixel nicht anfuegen aber vorne weiterverketten
    // -1: vorne anfuegen und weiterverketten
    // -2: vorne anfuegen und stoppen

    int width = gradientMaxima.width();
    int height = gradientMaxima.height();

    // Pixel einfuegen:
    gradientMaxima (x,y)=1e20; // Pixel als verarbeitet markieren
    if (mode>0)
      newList.push_back (ImageCoordinate (x,y));
    else if (mode<0)
      newList.push_front (ImageCoordinate (x,y));
    else if (mode==0)
      mode=-1;
    if (mode==2 || mode==-2 || x<=0 || y<=0 || x>=width || y>=height)
      return;  // andere Kante erreicht

    // Nachbarn in Suchrichtung mit maximalem Gradienten suchen:
    float edgeDirection = gradientDirection (x,y) + (mode>0 ? 0.785398f : 3.92699f);
    int dx, dy;
    neighborIndeces (dx, dy, edgeDirection);
    int mdx=dx;
    int mdy=dy;
    float mv = gradientMaxima(x+dx, y+dy);
    edgeDirection += 0.785398f;
    neighborIndeces (dx, dy, edgeDirection);
    if (gradientMaxima(x+dx, y+dy)>mv) {
      mv=gradientMaxima(x+dx, y+dy);
      mdx=dx;
      mdy=dy;
    }
    edgeDirection += 0.785398f;
    neighborIndeces (dx, dy, edgeDirection);
    if (gradientMaxima(x+dx, y+dy)>mv) {
      mv=gradientMaxima(x+dx, y+dy);
      mdx=dx;
      mdy=dy;
    }

    if (mv>1e10) {
      // Anschluss an bereits erfasste Kante. Dieses Pixel einfuegen und stoppen
      followEdge (gradientMaxima, gradientDirection, x+mdx, y+mdy, 2*mode, threshold, newList);
      return;
    }
    if (mv>=threshold) {
      // Kante weiterfuehren
      followEdge (gradientMaxima, gradientDirection, x+mdx, y+mdy, mode, threshold, newList);
      return;
    }
    // Ende der Kante erreicht, keine sinnvolle Weiterfuehrung moeglich
    return;
  }

} // Ende namespace

void Tribots::gradient (GrayLevelImage& gradientMagnitude, GrayLevelImage& gradientDirection, const GrayLevelImage& pixels) {
  unsigned int width = pixels.width();
  unsigned int height = pixels.height();
  GrayLevelImage imageSobelX (width, height);
  GrayLevelImage imageSobelY (width, height);
  SobelX sobelX;
  SobelY sobelY;
  sobelX (imageSobelX, pixels);
  sobelY (imageSobelY, pixels);
  for (unsigned int y=0; y<height; y++) {
    for (unsigned int x=0; x<width; x++) {
      double dIdx = imageSobelX (x,y);
      double dIdy = imageSobelY (x,y);
      gradientMagnitude (x,y) = sqrt(dIdx*dIdx+dIdy*dIdy);
      gradientDirection (x,y) = atan2 (dIdy, dIdx);
      if (gradientDirection (x,y)<0)
        gradientDirection (x,y)+=2*M_PI;
    }
  }
}

void Tribots::canny (std::deque<Pixelset >& dest,
            const GrayLevelImage& pixels,
            unsigned int gausswidth,
            float lowerThreshold,
            float upperThreshold) {
  unsigned int width =pixels.width();
  unsigned int height = pixels.height();

  // 1. smooth image using Gaussian
  GrayLevelImage imageSmooth (width, height);
  GaussianSmoothing smoother (gausswidth);
  smoother (imageSmooth, pixels);

  // 2. Gradienten berechnen
  GrayLevelImage gradientMagnitude (width, height);
  GrayLevelImage gradientDirection (width, height);
  gradient (gradientMagnitude, gradientDirection, imageSmooth);

  // 3. Nonmaxima-Suppression
  GrayLevelImage gradientMaxima (width, height);
  int dx, dy;
  for (unsigned int y=1; y+1<height; y++) {
    for (unsigned int x=1; x+1<width; x++) {
      neighborIndeces (dx, dy, gradientDirection (x,y));
      float mag0 = gradientMagnitude (x,y);
      float mag1 = gradientMagnitude (x+dx, y+dy);
      float mag2 = gradientMagnitude (x-dx, y-dy);
      gradientMaxima (x,y) = (mag0>=mag1 && mag0>=mag2 ? mag0 : 0);
    }
  }

  // 4. Kanten verbinden
  dest.clear();
  for (unsigned int y=1; y+1<height; y++) {
    for (unsigned int x=1; x+1<width; x++) {
      if (gradientMaxima (x,y)>=upperThreshold && around (gradientMaxima, x, y, upperThreshold)) {
        Pixelset newList2;
        dest.push_front (newList2);
        Pixelset& newList (dest.front()); // um nachtraegliches Kopieren zu vermeiden
        followEdge (gradientMaxima, gradientDirection, x, y, +1, lowerThreshold, newList);
        followEdge (gradientMaxima, gradientDirection, x, y, 0, lowerThreshold, newList);
        if (newList.size()<5) {
          dest.pop_front();  // ganz kurze Kanten wegschmeisen
        }
      }
    }
  }
}

void Tribots::drawLine (Image& dest, const Vec& p1, const Vec& p2, const RGBTuple& color, const RGBTuple& ccolor) {
  int width = dest.getWidth();
  int height = dest.getHeight();
  if (abs(p1.x-p2.x)>=abs(p1.y-p2.y)) {
    float slope = (p2.y-p1.y)/(p2.x-p1.x);
    float offset = p1.y-p1.x*slope;
    int index0 = min (width-1, max (0, static_cast<int>((p1.x<p2.x ? p1.x : p2.x)+0.5)));
    int indexN = min (width-1, max (0, static_cast<int>((p1.x>=p2.x ? p1.x : p2.x)+0.5)));
    for (int x=index0; x<=indexN; x++) {
      int y = static_cast<int>(offset+slope*x+0.5);
      if (y>=0 && y<height) {
        dest.setPixelRGB (x,y,color);
      }
    }
  } else {
    float slope = (p2.x-p1.x)/(p2.y-p1.y);
    float offset = p1.x-p1.y*slope;
    unsigned int index0 = min (height-1, max (0, static_cast<int>((p1.y<p2.y ? p1.y : p2.y)+0.5)));
    unsigned int indexN = min (height-1, max (0, static_cast<int>((p1.y>=p2.y ? p1.y : p2.y)+0.5)));
    for (unsigned int y=index0; y<=indexN; y++) {
      int x = static_cast<int>(offset+slope*y+0.5);
      if (x>0 && x<width) {
        dest.setPixelRGB (x,y,color);
      }
    }
  }
  // Endpunkte der Linien
  if (p1.x+0.5>=0 && p1.x+0.5<width && p1.y+0.5>=0 && p1.y+0.5<height)
    dest.setPixelRGB (static_cast<int>(p1.x+0.5),static_cast<int>(p1.y+0.5),ccolor);
  if (p2.x+0.5>=0 && p2.x+0.5<width && p2.y+0.5>=0 && p2.y+0.5<height)
    dest.setPixelRGB (static_cast<int>(p2.x+0.5),static_cast<int>(p2.y+0.5),ccolor);
}

namespace {

  void distanceFromLine (const std::vector<float>::iterator& distance, const Pixelset::const_iterator& first, const Pixelset::const_iterator& last) {
    std::vector<float>::iterator distance_it = distance;
    if ((*first)==(*last)) {
      // 1-Pixel-Linie oder Ring
      for (Pixelset::const_iterator pxl_it=first; pxl_it!=last+1; pxl_it++)
        *(distance_it++)=std::sqrt((double)(first->x-pxl_it->x)*(first->x-pxl_it->x)+(first->y-pxl_it->y)*(first->y-pxl_it->y));
      return;
    }
    // Hesse-NF der Geraden:
    float n1 = first->y-last->y;
    float n2 = last->x-first->x;
    float nn = sqrt(n1*n1+n2*n2);
    float fx = n1/nn;
    float fy = n2/nn;
    float f0 = (last->y*first->x-last->x*first->y)/nn;
    // Geradengleichung lautet: fx*x+fy*y+f0=0
    for (Pixelset::const_iterator pxl_it=first; pxl_it!=last+1; pxl_it++)
      *(distance_it++)=abs(fx*(pxl_it->x)+fy*(pxl_it->y)+f0);
  }

}

void Tribots::breakLine (std::deque<unsigned int>& breakpoints, const Pixelset& pixels, float maxdist) {
    // Prinzip des sukzessiven Splits
  unsigned int numPixels=pixels.size();
  // in "breakpoints" werden die Indeces gespeichert, an denen die Linie aufgebrochen wird
  breakpoints.push_front (0);
  breakpoints.push_back (numPixels-1);
  vector<float> distances (numPixels);

  // Kantensegmente anhand der Endpunkte in Linie ueberfuehren und
  // sukzessives Aufspalten an den Punkten maximalen Abstandes
  while (true) {
    unsigned int numSegments = breakpoints.size()-1;
    for (unsigned int segment=0; segment<numSegments; segment++) {
      distanceFromLine (distances.begin()+breakpoints[segment], pixels.begin()+breakpoints[segment], pixels.begin()+breakpoints[segment+1]);
    }
    vector<float>::iterator maxelem = max_element (distances.begin(), distances.end());
    if (*maxelem>maxdist) {
      // neuen Bruchpunkt einfuegen
      unsigned int pixelIndex = maxelem-distances.begin();
      unsigned int segmentIndex = 1;
      while (breakpoints [segmentIndex]<pixelIndex)
        segmentIndex++;
      breakpoints.insert (breakpoints.begin()+segmentIndex, pixelIndex);
    } else
      break;  // keine weiteren Bruchpunkte notwendig
  }
}

namespace {
  struct LenPair {
    unsigned int indexBreakpoint1;
    unsigned int indexBreakpoint2;
    bool operator< (const LenPair& lp) const { return (indexBreakpoint2-indexBreakpoint1)>(lp.indexBreakpoint2-lp.indexBreakpoint1); }
  };
}

void Tribots::splitLineSorted (std::deque<Pixelset >& dest, const Pixelset& src, const std::deque<unsigned int>& breakpoints) throw (std::bad_alloc) {
  dest.clear();
  if (breakpoints.size()<2) return;

  vector<LenPair> segments (breakpoints.size()-1);
  for (unsigned int i=0; i+1<breakpoints.size(); i++) {
    LenPair lp;
    lp.indexBreakpoint1=breakpoints[i];
    lp.indexBreakpoint2=breakpoints[i+1];
    segments.push_back (lp);
  }
  sort (segments.begin(), segments.end());
  dest.resize (segments.size());
  for (unsigned int i=0; i<segments.size(); i++) {
    dest[i].assign (src.begin()+segments[i].indexBreakpoint1, src.begin()+segments[i].indexBreakpoint2+1);
  }
}
