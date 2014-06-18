
#include "centerRingOperation.h"
#include "ImageArea.h"
#include "../Formation/Painter.h"
#include "../../Fundamental/Vec.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace Tribots;
using namespace std;

namespace {
  struct ImageAreaAttributes {
    bool big_enough;
    bool inside_center;
    bool encloses_center;
    bool is_ring;
    bool right_size;
    bool is_filled;
    double center [2];
    double radius;
    double min_center_radius;
    double max_center_radius;
  };
}

bool Tribots::findCenterRing (double& ccx, double& ccy, double& ccrmin, double& ccrmax, RGBImage& dest, const Image& src, bool debug) {
  unsigned int imageWidth = src.getWidth();
  unsigned int imageHeight = src.getHeight();
  unsigned int x0 = (imageWidth>302 ? imageWidth/2-151 : 0);
  unsigned int y0 = (imageHeight>302 ? imageHeight/2-151 : 0);
  unsigned int x1 = (imageWidth>302 ? imageWidth/2+151 : imageWidth-1);
  unsigned int y1 = (imageHeight>302 ? imageHeight/2+151 : imageHeight-1);
  return Tribots::findCenterRing (ccx, ccy, ccrmin, ccrmax, dest, src, x0, y0, x1, y1, debug);
}

bool Tribots::findCenterRing (double& ccx, double& ccy, double& ccrmin, double& ccrmax, RGBImage& dest, const Image& src, unsigned int x0, unsigned int y0, unsigned x1, unsigned int y1, bool debug) {
  RGBTuple rgb_black = {0,0,0};
  RGBTuple rgb_white = {255,255,255};
  RGBTuple rgb_red = {255,0,0};
  RGBTuple rgb_blue = {0,0,255};
  RGBTuple rgb_green = {0,255,0};
  RGBTuple rgb_yellow = {255,255,0};
  RGBTuple rgb_magenta = {255,0,255};
  RGBTuple rgb_cyan = {0,255,255};
  RGBTuple rgb_orange = {255,186,0};

  RGBTuple rgb_bg = rgb_black;
  RGBTuple rgb_int = rgb_white;

  unsigned int imageWidth = src.getWidth();
  unsigned int imageHeight = src.getHeight();
  ImageCoordinate leftupper (x0, y0);
  ImageCoordinate rightlower (x1, y1);
  if (debug) {
    leftupper = ImageCoordinate (0,0);
    rightlower = ImageCoordinate (imageWidth-1, imageHeight-1);
  }

  // "Schwarzabgleich" durchfuehren, dazu Pixel an den Bildecken sammeln
  unsigned int norm_red=0, norm_green=0, norm_blue=0;
  unsigned int xs [] = {2, 4, 2, 4, 2, 4, 2, 4, imageWidth-5, imageWidth-3, imageWidth-5, imageWidth-3, imageWidth-5, imageWidth-3, imageWidth-5, imageWidth-3 };
  unsigned int ys [] = {2, 2, 4, 4, imageHeight-3, imageHeight-3, imageHeight-5, imageHeight-5, 2, 2, 4, 4, imageHeight-3, imageHeight-3, imageHeight-5, imageHeight-5 };
  for (unsigned int i=0; i<16; i++) {
    RGBTuple rgb;
    src.getPixelRGB(xs[i],ys[i],&rgb);
    norm_red+=rgb.r;
    norm_green+=rgb.g;
    norm_blue+=rgb.b;
  }
  norm_red/=16;
  norm_green/=16;
  norm_blue/=16;
  unsigned int mini=norm_red;
  if (norm_green<mini)
    mini=norm_green;
  if (norm_blue<mini)
    mini=norm_blue;
  norm_red-=mini;
  norm_green-=mini;
  norm_blue-=mini;
  if (debug) {
    cerr << "Schwarzabgleich RGB= " << norm_red << ' ' << norm_green << ' ' << norm_blue << '\n';
  }

  // Kandidatenbereiche finden:
  for (int i=leftupper.x; i<=rightlower.x; i++) {
    for (int j=leftupper.y; j<=rightlower.y; j++) {
      RGBTuple rgb1;
      RGBTuple rgb2;
      src.getPixelRGB (i,j,&rgb1);

      // Gelbliche Bereiche finden
      double valg = 50*(static_cast<double>(rgb1.g)-norm_green)/(static_cast<double>(rgb1.b>norm_blue ? rgb1.b-norm_blue : 0)+1);
      double valr = 50*(static_cast<double>(rgb1.r)-norm_red)/(static_cast<double>(rgb1.b>norm_blue ? rgb1.b-norm_blue : 0)+1);
      if (valg>80 && valr>80 && rgb1.r-norm_red>20 && rgb1.g-norm_green>20) {
        rgb2=rgb_int;
      } else {
        rgb2=rgb_bg;
      }

//    Blaeuliche Bereiche finden
//       double valg = 50*(static_cast<double>(rgb1.b)-norm_blue)/(static_cast<double>(rgb1.g>norm_green ? rgb1.g-norm_green : 0)+1);
//       double valr = 50*(static_cast<double>(rgb1.b)-norm_blue)/(static_cast<double>(rgb1.r>norm_red ? rgb1.r-norm_red : 0)+1);
//       if (valg>80 && valr>80 && rgb1.b-norm_blue>15) {
//         rgb2=rgb_int;
//       } else {
//         rgb2=rgb_bg;
//       }

      dest.setPixelRGB(i,j,rgb2);
    }
  }

  // Bereiche aufsammeln und kleine Bereiche (<50 Pixel) ignorieren
  deque<ImageArea> areas;
  vector<ImageAreaAttributes> area_attributes;
  areas = ImageArea::collectErase (dest, rgb_int, rgb_bg, 0, leftupper, rightlower);
  area_attributes.resize (areas.size());
  for (unsigned int i=0; i<areas.size(); i++) {
    area_attributes[i].big_enough=(areas[i].size()>=50);
  }

  for (unsigned int i=0; i<areas.size(); i++) {
    // Pruefen, ob alle Pixel innerhalb des Mittelbereichs liegen
    area_attributes[i].inside_center = areas[i].checkInsideCircle (ImageCoordinate (dest.getWidth()/2, dest.getHeight()/2), 150);
    if (!area_attributes[i].inside_center || !area_attributes[i].big_enough)
      continue;

    // einen Kreis fitten
    double cx = 0;
    double cy = 0;
    double r = 0;
    areas[i].fitCircle (cx, cy, r);
    area_attributes[i].center[0]=cx;
    area_attributes[i].center[1]=cy;
    area_attributes[i].radius=r;

    // pruefen, ob es eine Ringform ist
    double maxangle=0;
    double sumangle=0;
    areas[i].ringCheck (area_attributes[i].min_center_radius, area_attributes[i].max_center_radius, maxangle, sumangle, cx, cy, r);
    area_attributes[i].encloses_center = (maxangle<=0.6*M_PI && sumangle<=0.8*M_PI);

    area_attributes[i].is_ring = (area_attributes[i].encloses_center && area_attributes[i].min_center_radius>12 && area_attributes[i].min_center_radius>0.3*area_attributes[i].max_center_radius);
    area_attributes[i].right_size = (area_attributes[i].radius>20 && area_attributes[i].radius<60);
    area_attributes[i].is_filled = (areas[i].size()/(M_PI*(area_attributes[i].max_center_radius*area_attributes[i].max_center_radius-area_attributes[i].min_center_radius*area_attributes[i].min_center_radius))>0.3);
  }

  // die beste Area suchen
  int best_area = -1;
  double best_area_value = -1;
  for (unsigned int i=0; i<areas.size(); i++) {
    if (area_attributes[i].is_ring && area_attributes[i].right_size && area_attributes[i].is_filled) {
      double value = exp(-1e-5*(static_cast<double>(areas[i].size())-1200.0)*(static_cast<double>(areas[i].size())-1200.0))*exp(-1e-3*(area_attributes[i].radius-35)*(area_attributes[i].radius-35));
      if (value>best_area_value) {
        best_area_value=value;
        best_area=i;
      }
    }
  }
  bool area_found = false;
  deque<int> best_area_indeces;
  if (best_area>=0) {
    area_found=true;
    ccx = area_attributes[best_area].center[0];
    ccy = area_attributes[best_area].center[1];
    ccrmin = area_attributes[best_area].min_center_radius;
    ccrmax = area_attributes[best_area].max_center_radius;
    best_area_indeces.push_back (best_area);
  }

  for (unsigned int k=0; k<3; k++) {
    if (best_area<0) {
      if (debug) {
        cerr << "Kein Bereich gefunden. Versuche, Bereiche zusammenzubauen.\n";
      }
      // keine Area gefunden. Versuchen, Areas zusammenzusetzen
      for (unsigned int i=0; i<area_attributes.size(); i++) {
        if (area_attributes[i].big_enough && area_attributes[i].inside_center) {
          // Ring i untersuchen
          ImageArea na;
          deque<int> contrib_areas;
          for (unsigned int j=0; j<areas.size(); j++) {
            if (area_attributes[j].inside_center) {
              // pruefen, ob Area j zu Ring i passt
              double minr, maxr, maxa, suma;
              areas[j].ringCheck (minr, maxr, maxa, suma, area_attributes[i].center[0], area_attributes[i].center[1], area_attributes[i].radius);
              if ((area_attributes[j].big_enough && minr+15>area_attributes[i].min_center_radius && area_attributes[i].max_center_radius+15>maxr) ||
                  (minr+5>area_attributes[i].min_center_radius && area_attributes[i].max_center_radius+5>maxr)) {
                na+=areas[j];
                contrib_areas.push_back (j);
              }
            }
          }
          // in na sind jetz alle Pixel, die zu dem Ring gehoeren koennten
          bool success = na.fitCircle (area_attributes[i].center[0], area_attributes[i].center[1], area_attributes[i].radius);
          if (!success) {
            area_attributes[i].center[0]=area_attributes[i].center[1]=area_attributes[i].radius=0;
          }
          double maxangle=0;
          double sumangle=0;
          na.ringCheck (area_attributes[i].min_center_radius, area_attributes[i].max_center_radius, maxangle, sumangle, area_attributes[i].center[0], area_attributes[i].center[1], area_attributes[i].radius);
          area_attributes[i].encloses_center = (maxangle<=0.6*M_PI && sumangle<=0.8*M_PI);
          area_attributes[i].is_ring = (area_attributes[i].encloses_center && area_attributes[i].min_center_radius>12 && area_attributes[i].min_center_radius>0.3*area_attributes[i].max_center_radius);
          area_attributes[i].right_size = (area_attributes[i].radius>20 && area_attributes[i].radius<60);
          area_attributes[i].is_filled = (areas[i].size()/(M_PI*(area_attributes[i].max_center_radius*area_attributes[i].max_center_radius-area_attributes[i].min_center_radius*area_attributes[i].min_center_radius))>0.3);
          if (area_attributes[i].is_ring && area_attributes[i].right_size && area_attributes[i].is_filled) {
            double value = exp(-1e-5*(static_cast<double>(na.size())-1200.0)*(static_cast<double>(na.size())-1200.0))*exp(-1e-3*(area_attributes[i].radius-35)*(area_attributes[i].radius-35));
            if (value>best_area_value) {
              best_area_value=value;
              best_area=i;
              best_area_indeces=contrib_areas;
            }
          }
        }
      }
      if (best_area>=0) {
        area_found=true;
        ccx = area_attributes[best_area].center[0];
        ccy = area_attributes[best_area].center[1];
        ccrmin = area_attributes[best_area].min_center_radius;
        ccrmax = area_attributes[best_area].max_center_radius;
        break;
      }
    }
  }

  if (debug) {
    // Ergebnis plotten
    for (int i=0; static_cast<unsigned int>(i)<areas.size(); i++) {
      RGBTuple rgb = rgb_red;
      if (!area_attributes[i].big_enough) {
        rgb=rgb_blue;
      } else if (!area_attributes[i].inside_center) {
        rgb=rgb_green;
      } else if (!area_attributes[i].is_ring) {
        rgb=rgb_cyan;
      } else if (!area_attributes[i].is_filled) {
        rgb=rgb_orange;
      } else if (!area_attributes[i].right_size) {
        rgb=rgb_magenta;
      }
      bool belongs_to_best = (find (best_area_indeces.begin(), best_area_indeces.end(), i)!=best_area_indeces.end());
      if (belongs_to_best)
        rgb=rgb_red;
      if (area_attributes[i].big_enough && area_attributes[i].inside_center ) {
        cerr << "ImageArea " << i << ": " << areas[i].size() << '\t' << area_attributes[i].center[0] << ' ' << area_attributes[i].center[1] << ' ' << area_attributes[i].radius << '\t' << area_attributes[i].min_center_radius << ' ' << area_attributes[i].max_center_radius << ' ' << area_attributes[i].encloses_center;
        if (belongs_to_best)
          ; //cerr << " *\n";
        else
          ; //cerr << '\n';
      }
      areas[i].draw (dest, rgb);
      if (area_attributes[i].big_enough && area_attributes[i].inside_center) {
        Painter paint (dest);
        paint.setColor (rgb_yellow);
        paint.drawCircle (Vec(area_attributes[i].center[0],area_attributes[i].center[1]), area_attributes[i].radius);
        if (i==best_area) {
          paint.drawLine (Vec(area_attributes[i].center[0]-10,area_attributes[i].center[1]), Vec(area_attributes[i].center[0]+10,area_attributes[i].center[1]));
          paint.drawLine (Vec(area_attributes[i].center[0],area_attributes[i].center[1]-10), Vec(area_attributes[i].center[0],area_attributes[i].center[1]+10));
        }
      }
    }
  }

  return area_found;
}


void Tribots::determineBalanceArea (unsigned int& x1, unsigned int& y1, unsigned int& x2, unsigned int& y2, double ccx, double ccy, double ccrmin, double ccrmax) throw () {
  double off =ccrmin-4;
  x1=static_cast<int>(ccx-off);
  y1=static_cast<int>(ccy-off);
  x2=static_cast<int>(ccx+off);
  y2=static_cast<int>(ccy+off);
}
