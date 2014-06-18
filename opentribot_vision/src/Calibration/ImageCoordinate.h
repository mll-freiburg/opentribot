
#ifndef _Tribots_ImageCoordinate_h_
#define _Tribots_ImageCoordinate_h_

#include "../../Fundamental/Vec.h"
#include <cmath>

namespace Tribots {

  /** Struktur, um 2-dimensionale ganzzahlige Koordinaten darzustellen */
  struct ImageCoordinate {
    int x;
    int y;

    ImageCoordinate (int i =0, int j=0) throw () : x(i), y(j) {;}
    ImageCoordinate (const ImageCoordinate& c) throw () : x(c.x), y(c.y) {;}
    bool operator== (const ImageCoordinate& c) const throw () { return x==c.x && y==c.y; }
    bool operator!= (const ImageCoordinate& c) const throw () { return x!=c.x || y!=c.y; }
    const ImageCoordinate& operator= (const ImageCoordinate& c) throw () { x=c.x; y=c.y; return *this; }
    const ImageCoordinate& operator+= (const ImageCoordinate& c) throw () { x+=c.x; y+=c.y; return *this; }
    const ImageCoordinate& operator-= (const ImageCoordinate& c) throw () { x-=c.x; y-=c.y; return *this; }
    const ImageCoordinate& operator*= (double d) throw () { x=static_cast<int>(static_cast<double>(x)*d); y=static_cast<int>(static_cast<double>(y)*d); return *this; }
    const ImageCoordinate& operator/= (double d) throw () { x=static_cast<int>(static_cast<double>(x)/d); y=static_cast<int>(static_cast<double>(y)/d); return *this; }
    ImageCoordinate operator+ (const ImageCoordinate& c) const throw () { ImageCoordinate r (*this); r+=c; return r; }
    ImageCoordinate operator- (const ImageCoordinate& c) const throw () { ImageCoordinate r (*this); r-=c; return r; }
    ImageCoordinate operator* (double d) const throw () { ImageCoordinate r (*this); r*=d; return r; }
    ImageCoordinate operator/ (double d) const throw () { ImageCoordinate r (*this); r/=d; return r; }
    double distance (const ImageCoordinate& c) const throw () { return std::sqrt ((double)((c.x-x)*(c.x-x)+(c.y-y)*(c.y-y))); }
    Vec toVec () const throw () { return Vec (x,y); }
  };

}

#endif
