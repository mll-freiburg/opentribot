#ifndef REGIONS_H_
#define REGIONS_H_

#include <vector>
#include "../../Fundamental/Vec.h"
#include "../../Fundamental/Time.h"

namespace Tribots {

  /** Region, repräsentiert als Chaincode */
  class Region {
  public:
    int colClass;
    int x, y;
    std::vector<char> chainCode;
    int maxX, maxY, minX, minY;

    inline Region(int colClass, int x, int y);
    inline Region();
//    inline Region(const Region&);
    
    inline Region* clone() const;

    int getArea() const;
    Vec getCenterOfGravity() const;
    double getCompactness() const;
    
  protected:
    int area;
    Vec centerOfGravity;
  };

  /** Liste von erkannten Regionen mit Zeitstempel */
  class RegionList 
  {
    public:
      Time timestamp;
      std::vector<Region*> list; /** Regionsliste */

    RegionList (unsigned int len =0) throw () : list(len) {}
    ~RegionList () throw ();

    /** Writes a ascii serialization of the object to a stream. 
       Arg1: stream to write on. 
    **/
    void writeAt(std::ostream &stream) const;

    /** reads a ascii serialization of the object from a stream.
     Arg1: stream to read from.
     Returns number of correct read obstacles.
    **/
    int  readFrom(std::istream &stream);

    private:
    /** copy constructor */
    RegionList (const RegionList&) {}
  };
  
  // inlines

  Region::Region(int colClass, int x, int y) 
    : colClass(colClass), x(x), y(y), maxX(x), maxY(y), minX(x), minY(y),
      area(-1)
  {}

  Region::Region()
    : colClass(0), x(0), y(0), maxX(0), maxY(0), minX(0), minY(0),
      area(-1)
  {}
  
  /*Region::Region(const Region& r)
    : colClass(r.colClass), x(r.x), y(r.y), chainCode(r.chainCode), maxX(r.maxX), maxY(r.maxY), minX(r.minX), minY(r.minY),
      area(r.area)
  {}*/
  
  Region*
  Region::clone() const
  {
    return new Region(*this);
  }

};
#endif /*REGIONS_H_*/
