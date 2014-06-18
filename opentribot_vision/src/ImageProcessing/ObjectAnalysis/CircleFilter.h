#ifndef _circlefilter_h_
#define _circlefilter_h_

#include "LineFilter.h"

namespace Tribots
{

  class CircleFilter: public LineFilter
  {
  public:
    CircleFilter();
    void scanLine(const Tribots::Image & img ,Vec start, Vec end,int kernelmode=3);
    void visualize(Tribots::Image & img ,Vec start, Vec end);
  };

};


#endif
