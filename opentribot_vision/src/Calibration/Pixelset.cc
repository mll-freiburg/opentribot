
#include "Pixelset.h"

using namespace std;
using namespace Tribots;

void Tribots::setPixelsetAll (Pixelset& dest, unsigned int width, unsigned int height) throw (std::bad_alloc) {
  for (unsigned int j=0; j<width; j++) {
    for (unsigned int i=0; i<height; i++) {
       dest.push_back (ImageCoordinate (i,j));
    }
  }
}

void Tribots::findBluishPixels (Pixelset& dest, const Image& src, double sensitivity) throw (std::bad_alloc) {
  double minimD = (sensitivity-1.0)*255;
  unsigned int minim = (minimD<0 ? 0 : static_cast<int>(minimD));
  RGBTuple rgb;
  for (int j=0; j<src.getHeight(); j++) {
    for (int i=0; i<src.getWidth(); i++) {
      src.getPixelRGB (i,j,&rgb);
      if (rgb.b>=sensitivity*rgb.r && rgb.b>=sensitivity*rgb.g && rgb.b>=minim)
        dest.push_back (ImageCoordinate (i,j));
    }
  }
}

void Tribots::findGreenishPixels (Pixelset& dest, const Image& src, double sensitivity) throw (std::bad_alloc) {
  double minimD = (sensitivity-1.0)*255;
  unsigned int minim = (minimD<0 ? 0 : static_cast<int>(minimD));
  RGBTuple rgb;
  for (int j=0; j<src.getHeight(); j++) {
    for (int i=0; i<src.getWidth(); i++) {
      src.getPixelRGB (i,j,&rgb);
      if (rgb.g>=sensitivity*rgb.r && rgb.g>=sensitivity*rgb.b && rgb.g>=minim)
        dest.push_back (ImageCoordinate (i,j));
    }
  }
}

void Tribots::findReddishPixels (Pixelset& dest, const Image& src, double sensitivity) throw (std::bad_alloc) {
  double minimD = (sensitivity-1.0)*255;
  unsigned int minim = (minimD<0 ? 0 : static_cast<int>(minimD));
  RGBTuple rgb;
  for (int j=0; j<src.getHeight(); j++) {
    for (int i=0; i<src.getWidth(); i++) {
      src.getPixelRGB (i,j,&rgb);
      if (rgb.r>sensitivity*rgb.b && rgb.r>sensitivity*rgb.g && rgb.r>=minim)
        dest.push_back (ImageCoordinate (i,j));
    }
  }
}

namespace {
  inline bool pixless (const ImageCoordinate& c1, const ImageCoordinate& c2) {
    if (c1.y<c2.y) return true;
    if (c1.y>c2.y) return false;
    return (c1.x<c2.x);
  }
}

void Tribots::intersectSortedSets (Pixelset& dest, const Pixelset& src1, const Pixelset& src2) throw (std::bad_alloc) {
  Pixelset::const_iterator it1 = src1.begin();
  Pixelset::const_iterator it2 = src2.begin();
  while (it1!=src1.end() && it2!=src2.end()) {
    if ((*it1) == (*it2)) {
      dest.push_back (*it1);
      ++it1;
      ++it2;
    } else if (pixless (*it1, *it2)) {
      ++it1;
    } else {
      ++it2;
    }
  }
}

Vec Tribots::centerOfGravity (const Pixelset& src) throw () {
  Vec result(0,0);
  for (Pixelset::const_iterator it = src.begin(); it!=src.end(); ++it)
    result+=it->toVec();
  double n=src.size();
  if (n<=0)
    n=1;  // um Division durch Null zu vermeiden
  return result/n;
}

void Tribots::drawPixelset (Image& image, const Pixelset& pixelset, RGBTuple color) throw () {
  for (Pixelset::const_iterator it=pixelset.begin(); it!=pixelset.end(); ++it)
    image.setPixelRGB (it->x, it->y, color);
}
