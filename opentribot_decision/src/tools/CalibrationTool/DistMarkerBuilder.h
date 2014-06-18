
#ifndef _TribotsTools_DistMarkerBuilder_h_
#define _TribotsTools_DistMarkerBuilder_h_

#include "MarkerLog.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../Fundamental/Table.h"
#include "../../Structures/TribotsException.h"
#include <vector>
#include <deque>
#include <iostream>

namespace TribotsTools {

  class DistMarkerBuilder {
  private:
    unsigned int numDistanceLines;
    std::vector<double> trueDistances;
    std::vector<MarkerLog::MarkerType> markerTypes;
    Tribots::Table<double> distanceTable;
    unsigned int regularisationRep;
    unsigned int maxNumWithoutAssignment;
    double minClusterDistance;

    void cluster (std::vector<std::deque<double> >& dest, const std::deque<MarkerLog>& src);
    void circleSmooth (const std::vector<MarkerLog>& marker);
  public:
    DistMarkerBuilder (const Tribots::ConfigReader& cfg, const std::string& section) throw (Tribots::TribotsException);
    ~DistMarkerBuilder () throw ();

    void createTable (const std::vector<MarkerLog>&);
    void writeTable (std::ostream& dest, unsigned int imageWidth, unsigned int imageHeight, unsigned int imageCenterX, unsigned int imageCenterY);
  };

}

#endif
