
#ifndef TribotsTools_DistanceCalibration_h
#define TribotsTools_DistanceCalibration_h

#include <string>
#include <deque>
#include <vector>
#include "../../ImageProcessing/ObjectAnalysis/ColorClasses.h"
#include "../../ImageProcessing/ObjectAnalysis/LineScanning.h"
#include "../../Fundamental/Vec.h"
#include <fstream>

namespace TribotsTools {

  class DistanceCalibration {
  private:
   // Struktur, um fuer jeden erkannten Marker Winkel, Entfernung in Echt und Entfernung im Bild zu merken
    struct MarkerInfo {
      Tribots::Angle angle;
      double image_distance;
      MarkerInfo () throw () {;}
      MarkerInfo (const Tribots::Angle& m, double dist) : angle(m), image_distance(dist) {}
      MarkerInfo (const MarkerInfo& m) : angle(m.angle), image_distance(m.image_distance) {;}
      const MarkerInfo& operator= (const MarkerInfo& m) { angle=m.angle; image_distance=m.image_distance; return *this; }
    };

    std::vector<double> realDistances [6];
    std::vector<double> prototypesInit [6];
    std::vector<unsigned> markerNum [6];
    std::deque<MarkerInfo> markers [6];    // Index siehe enum TransitionMarker
    Tribots::Vec image_center;
    double minClusterDistance;
    bool logMarkers;

  public:
    enum TransitionMarker {
      virtual_zero_change=0,
      blue_white_change=1,
      white_blue_change=2,
      white_red_change=3,
      red_white_change=4,
      red_middle_change=5
    };

    DistanceCalibration (const std::vector<double>& realDistances1,
                         const std::vector<double>& prototypes,
                         const std::vector<TransitionMarker>& markerType,
                         Tribots::Vec,  
                         double minClusterDistance=8.,
                         const std::string& markerLogFile = "");
    ~DistanceCalibration ();
    void search_markers (const Tribots::ScanResultList&);
    void writeMarkerFile (const std::string&);

  private:
    // Hilfsfunktion: in einer Liste von Markern den am naechstliegenden suchen mit Minimalentfernung arg3
    double search_min_transition_index (const std::deque<MarkerInfo>&, double);
    std::ofstream mout;
  };

}

#endif
