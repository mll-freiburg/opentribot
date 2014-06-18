#ifndef _linefilter_h_
#define _linefilter_h_

#include "../Formation/Image.h"
#include "../../Fundamental/Vec.h"
#include "../../Structures/VisibleObject.h"

namespace Tribots
{

class ScanResultList;



  class LineFilter
  {
  public:
    LineFilter();
    void setIntegrationWidth(int width);
    void scanLine(const Tribots::Image & img ,Vec start, Vec end,int kernelmode=3);
    void setScanResultList(ScanResultList* list);    
    void visualize(Tribots::Image & img ,Vec start, Vec end);
    ScanResultList* scanresultlist;

    int buffer[1000]; // buffer for a line  Gray Values
    int num_buffer;
    int fbuffer[1000]; // filtered line  filtered Gray Values
    int num_fbuffer;
    int results[1000]; // results  index of the maxima
    int num_results;
    double lines[1000]; // found line
    Vec resultpos[1000]; // found line

    int num_lines;
    enum{MP,MOP,MMPP,MMMPPP,MMMMPPPP};
    int   integrationwidth;
  };

};


#endif
