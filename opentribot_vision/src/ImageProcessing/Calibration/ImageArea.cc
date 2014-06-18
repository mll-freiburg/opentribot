
#include "ImageArea.h"
#include "../Formation/RGBImage.h"
#include "../Formation/Painter.h"
#include "../../Fundamental/circleFit.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <matrix_operations.h>

using namespace Tribots;
using namespace Martin;
using namespace std;


ImageArea::ImageArea () throw () {;}

ImageArea::ImageArea (const ImageArea& a) throw () : pixels (a.pixels) {;}

const ImageArea& ImageArea::operator= (const ImageArea& a) throw () {
  pixels = a.pixels;
  return *this;
}

const ImageArea& ImageArea::operator+= (const ImageArea& a) throw () {
  pixels.insert (pixels.end(), a.pixels.begin(), a.pixels.end());
  return *this;
}

ImageArea ImageArea::operator+ (const ImageArea& a) const throw () {
  ImageArea b (*this);
  b+=a;
  return b;
}

unsigned int ImageArea::size () const throw () {
  return pixels.size();
}

void ImageArea::draw (RGBImage& im, const RGBTuple& color) const throw () {
  for (unsigned int i=0; i<pixels.size(); i++)
    im.setPixelRGB (pixels[i].x, pixels[i].y, color);
}

void ImageArea::collectErase (RGBImage& im, const RGBTuple& replcolor, ImageCoordinate seed, ImageCoordinate leftupper, ImageCoordinate rightlower) throw () {
  RGBTuple color;
  im.getPixelRGB (seed.x, seed.y, &color);
  collectErase (im, color, replcolor, seed.x, seed.x, seed.y, +1, leftupper, rightlower);
}

void ImageArea::collectErase (RGBImage& im, const RGBTuple& color, const RGBTuple& replcolor, int x1, int x2, int y, int dir, ImageCoordinate leftupper, ImageCoordinate rightlower) throw () {
  if (y<leftupper.y || y>rightlower.y)
    return;
  RGBTuple rgb;
  im.getPixelRGB (x1, y, &rgb);
  int leftmax=x1;
  bool do_fill=false;
  if (rgb==color) {
    do_fill=true;
    while (leftmax>leftupper.x) {
      RGBTuple rgb;
      im.getPixelRGB (leftmax-1, y, &rgb);
      if (rgb!=color)
        break;
      im.setPixelRGB (leftmax-1, y, replcolor);
      pixels.push_back (ImageCoordinate (leftmax-1,y));
      leftmax--;
    }
    if (leftmax<x1) {
      collectErase (im, color, replcolor, leftmax, x1-1, y-dir, -dir, leftupper, rightlower);
    }
  }
  int x=x1;
  while (x<=x2 || (do_fill && x<=rightlower.x)) {
    im.getPixelRGB (x, y, &rgb);
    if (rgb==color) {
      if (!do_fill) {
        leftmax=x;
        do_fill=true;
      }
      im.setPixelRGB (x, y, replcolor);
      pixels.push_back (ImageCoordinate (x,y));
    } else {
      if (do_fill) {
        do_fill=false;
        collectErase (im, color, replcolor, leftmax, x-1, y+dir, dir, leftupper, rightlower);
        if (x>x2+1) {
          collectErase (im, color, replcolor, (x2>leftmax ? x2 : leftmax), x-1, y-dir, -dir, leftupper, rightlower);
        }
      }
    }
    x++;
  }
  if (do_fill) {
    collectErase (im, color, replcolor, leftmax, x-1, y+dir, dir, leftupper, rightlower);
    if (x>x2+1) {
      collectErase (im, color, replcolor, (x2>leftmax ? x2 : leftmax), x-1, y-dir, -dir, leftupper, rightlower);
    }
  }
}

std::deque<ImageArea> ImageArea::collectErase (RGBImage& im, const RGBTuple& color, const RGBTuple& replcolor, unsigned int minsize, ImageCoordinate leftupper, ImageCoordinate rightlower) throw () {
  deque<ImageArea> areas;
  for (int j=leftupper.y; j<=rightlower.y; j++) {
    for (int i=leftupper.x; i<=rightlower.x; i++) {
      RGBTuple rgb1;
      im.getPixelRGB (i,j,&rgb1);
      if (rgb1==color) {
        ImageArea na;
        na.collectErase (im, replcolor, ImageCoordinate (i,j), leftupper, rightlower);
        if (na.size()>=minsize)
          areas.push_back (na);
      }
    }
  }
  return areas;
}

bool ImageArea::checkInsideCircle (ImageCoordinate center, double radius) const throw () {
  double radius2=radius*radius;
  for (deque<ImageCoordinate>::const_iterator it=pixels.begin(); it!=pixels.end(); ++it)
    if ((it->x-center.x)*(it->x-center.x)+(it->y-center.y)*(it->y-center.y)>radius2)
      return false;
  return true;
}

bool ImageArea::fitCircle (double& cx, double& cy, double& r) const throw () {
  try{
    Tribots::Vec c;
    Tribots::algebraicCircleFit (c, r, pixels.begin(), pixels.end());
    cx=c.x;
    cy=c.y;
    return true;
  }catch(std::invalid_argument&) {
    return false;  // numerisches Problem, kollineare Punkte oder zu wenige Punkte
  }
}

void ImageArea::ringCheck (double& minradius, double& maxradius, double& maxangle, double& sumangle, double cx, double cy, double r) const throw () {
  minradius = 1e300;
  maxradius = 0;
  vector<double> angles (pixels.size());
  deque<ImageCoordinate>::const_iterator it = pixels.begin();
  vector<double>::iterator ita = angles.begin();
  for (; it!=pixels.end(); it++, ita++) {
    double dist = sqrt((it->x-cx)*(it->x-cx)+(it->y-cy)*(it->y-cy));
    if (dist<minradius)
      minradius=dist;
    if (maxradius<dist)
      maxradius=dist;
    (*ita)=atan2 (it->y-cy,it->x-cx);
  }
  sort (angles.begin(), angles.end());
  double la = angles[angles.size()-1]-2*M_PI;
  maxangle=0;
  sumangle=0;
  for (ita = angles.begin(); ita!=angles.end(); ita++) {
    double d = (*ita)-la;
    la = (*ita);
    if (d>maxangle) {
      maxangle=d;
    }
    if (d>0.087) { // wenn Winkel groesser als 5 Grad
      sumangle+=d;
    }
  }
}
