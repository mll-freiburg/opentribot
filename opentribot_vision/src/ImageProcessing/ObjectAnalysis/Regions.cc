#include "Regions.h"
#include "ChainCoding.h"
#include <cstdlib>
#include <iostream>

namespace Tribots {
  
  using namespace std;

  int
  Region::getArea() const
  {
    if (area < 0) {               // have to calculate it first
      int a = 0;

      int vertPos=0;
      int horiPos=0;

      int cX = 0;
      int cY = 0;      
   
      // walk through the chaincode and integrate (see Horn for a desc.)
      for (unsigned int i=0; i < chainCode.size(); i++) {
  switch(chainCode[i]) {
  case 0: a-=vertPos; cX-=vertPos*horiPos; break;
  case 1: cY+=vertPos*horiPos; break;
  case 2: a+=vertPos; cX+=vertPos*horiPos; break;
  case 3: cY-=horiPos*vertPos; break;
  }
  vertPos += ChainCoder::yDir[(int)chainCode[i]];
  horiPos += ChainCoder::xDir[(int)chainCode[i]];
      }      
      const_cast<Region*>(this)->area=a;
      const_cast<Region*>(this)->centerOfGravity=
  Vec(cX/(double)a, cY/(double)a) + Vec(x,y);
    }
    return area;   
  }

  double
  Region::getCompactness() const
  {
    return ((double)chainCode.size() * chainCode.size()) / getArea();
  }
  
  Vec 
  Region::getCenterOfGravity() const
  {
    if (area < 0) {
      getArea();               // berechnet auch das Zentrum
    }
    return centerOfGravity;
  }
  
    RegionList::~RegionList () throw()
    {
      for (unsigned int i=0; i < list.size(); i++) {
        delete list[i];
      }
      list.clear();
    }
  
  
    void RegionList::writeAt(std::ostream &stream) const
    {
      //TODO
    }

    int RegionList::readFrom(std::istream &stream)
    {
      //TODO
      return(0);
    }
  
};
