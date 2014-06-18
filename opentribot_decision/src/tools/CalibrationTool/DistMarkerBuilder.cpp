
#include "DistMarkerBuilder.h"
#include "../../Fundamental/geometry.h"
#include "../../Fundamental/circleFit.h"
#include <cmath>
#include <algorithm>

using namespace Tribots;
using namespace TribotsTools;
using namespace std;

DistMarkerBuilder::DistMarkerBuilder (const Tribots::ConfigReader& cfg, const std::string& section) throw (TribotsException) {
  if (!cfg.get ((section+"::real_distances").c_str(), trueDistances))
    throw InvalidConfigurationException ((section+"::real_distances").c_str());
  vector<string> para;
  if (!cfg.get ((section+"::marker_type").c_str(), para))
    throw InvalidConfigurationException ((section+"::marker_type").c_str());
  for (unsigned int i=0; i<para.size(); i++) {
    if (para[i]=="null")
      markerTypes.push_back (MarkerLog::NL);
    else if (para[i]=="bw" || para[i]=="BW")
      markerTypes.push_back (MarkerLog::BW);
    else if (para[i]=="wb" || para[i]=="WB")
      markerTypes.push_back (MarkerLog::WB);
    else if (para[i]=="wr" || para[i]=="WR")
      markerTypes.push_back (MarkerLog::WR);
    else if (para[i]=="rw" || para[i]=="RW")
      markerTypes.push_back (MarkerLog::RW);
    else if (para[i]=="rm" || para[i]=="RM")
      markerTypes.push_back (MarkerLog::RM);
    else
      throw InvalidConfigurationException ((section+"::markerType").c_str());
  }
  numDistanceLines=markerTypes.size();
  if (markerTypes.size()!=trueDistances.size())
    throw InvalidConfigurationException ((section+"::markerType and "+section+"::readDistances").c_str());

  regularisationRep = 3;
  maxNumWithoutAssignment = 20;
  minClusterDistance = 5;  ///< eventuell aus cfg einlesen lassen
}

DistMarkerBuilder::~DistMarkerBuilder () throw () {;}

void DistMarkerBuilder::writeTable (std::ostream& dest, unsigned int imageWidth, unsigned int imageHeight, unsigned int imageCenterX, unsigned int imageCenterY) {
  // Kopf der Datei schreiben:
  dest << imageWidth << ' ' << imageHeight << ' ' << imageCenterX << ' ' << imageCenterY << '\n';
  dest << "3 " << numDistanceLines << '\n';
  for (unsigned int i=0; i<numDistanceLines; i++)
    dest << trueDistances[i] << ' ';
  dest << '\n';

  // Tabelle schreiben:
  for (unsigned int i=0; i<45; i++) {
    dest << i << " 0";
    for (unsigned int j=0; j<numDistanceLines; j++) {
      dest << ' ' << distanceTable (i,j);
    }
    dest << '\n';
  }
  dest << flush;
}

namespace {
  struct Cluster {
    double distance;
    unsigned int numEmpty;
    std::deque<double> markersAssigned;
    double spread;
    bool operator< (const Cluster& cl) const throw () { return distance<cl.distance; }
    Cluster () : distance(0), numEmpty(0), spread(0) {;}
  };
}

void DistMarkerBuilder::cluster (std::vector<std::deque<double> >& dest, const std::deque<MarkerLog>& src) {
  unsigned int numCluster=dest.size();
  for (unsigned int j=0; j<numCluster; j++) {
    dest[j].clear();
  }
  std::vector<Cluster> prototypes (numCluster);
  for (int iterations = -135; iterations<45; iterations++) {
    Angle cangle = Angle::deg_angle (iterations*8);
    Angle leftangle = Angle::deg_angle (iterations*8+10);
    Angle rightangle = Angle::deg_angle (iterations*8-10);


    // zur Regularisierung schon mal die alte Position eintragen
    for (unsigned int j=0; j<prototypes.size(); j++) {
      prototypes[j].markersAssigned.clear();
      for (unsigned int k=0; k<regularisationRep; k++)
        prototypes[j].markersAssigned.push_back (prototypes[j].distance);
    }
    // jeden Marker dem naechstliegenden Prototypen zuordnen
    for (std::deque<MarkerLog>::const_iterator it=src.begin(); it!=src.end(); it++) {
      if (it->angle.in_between (rightangle, leftangle)) {
        unsigned int minIndex=0;
        double minDistance=1e300;
        for (unsigned int j=0; j<prototypes.size(); j++) {
          if (abs(it->distance-prototypes[j].distance)<minDistance) {
            minIndex=j;
            minDistance=abs(it->distance-prototypes[j].distance);
          }
        }
        prototypes[minIndex].markersAssigned.push_back (it->distance);
      }
    }
    // die Prototypen neu berechnen durch Medianbildung und die Attribute berechnen
    for (unsigned int j=0; j<prototypes.size(); j++) {
      unsigned int n=prototypes[j].markersAssigned.size();
      if (n<=regularisationRep)
        prototypes[j].numEmpty++;
      else
        prototypes[j].numEmpty=0;
      sort (prototypes[j].markersAssigned.begin(), prototypes[j].markersAssigned.end());
      if (n%2==0)
        prototypes[j].distance=0.5*(prototypes[j].markersAssigned[n/2]+prototypes[j].markersAssigned[n/2-1]);
      else
        prototypes[j].distance=prototypes[j].markersAssigned[n/2];
      prototypes[j].spread=prototypes[j].markersAssigned[4*n/5]-prototypes[j].markersAssigned[n/5];
    }
    // die Nachbarschaft mehrerer Prototypen berechnen sowie problematische Cluster erfassen
    sort (prototypes.begin(), prototypes.end());
    unsigned int broadestClusterIndex=0;
    unsigned int closeClusterIndex=0;
    double closeClusterDistance=1e300;
    int emptyClusterIndex=-1;
    for (unsigned int j=0; j<prototypes.size(); j++) {
      if (prototypes[j].numEmpty>maxNumWithoutAssignment)
        emptyClusterIndex=j;
      if (prototypes[j].spread>prototypes[broadestClusterIndex].spread)
        broadestClusterIndex=j;
      if (j>0) {
        double d=prototypes[j].distance-prototypes[j-1].distance;
        if (d<closeClusterDistance) {
          closeClusterDistance=d;
          closeClusterIndex=j;
        }
      }
    }
    int removeCluster=-1;
    if (closeClusterDistance<minClusterDistance)
      removeCluster=closeClusterIndex;
    if (emptyClusterIndex>=0)
      removeCluster=emptyClusterIndex;
    if (removeCluster>=0) {
      prototypes[removeCluster].distance=prototypes[broadestClusterIndex].distance+1;
      prototypes[removeCluster].numEmpty=0;
      prototypes[broadestClusterIndex].distance-=1;
      sort (prototypes.begin(), prototypes.end());
    }

    if (iterations>=0) {
      for (unsigned int j=0; j<numCluster; j++)
        dest[j].push_back (prototypes[j].distance);
    }
  }
}

void DistMarkerBuilder::createTable (const std::vector<MarkerLog>& marker) {
  distanceTable.resize (45, numDistanceLines);
  std::vector<std::deque<MarkerLog> > tmarker (5);
  for (std::vector<MarkerLog>::const_iterator it = marker.begin(); it!=marker.end(); it++) {
    tmarker[it->type].push_back (*it);
  }
  for (unsigned int i=0; i<distanceTable.rows(); i++)
    for (unsigned int j=0; j<distanceTable.columns(); j++)
      distanceTable (i,j)=0;
  for (unsigned int mt=0; mt<5; mt++) {
    vector<unsigned int> idx;
    for (unsigned int i=0; i<markerTypes.size(); i++) {
      if (markerTypes[i]==MarkerLog::MarkerType(mt)) {
        idx.push_back (i);
      }
    }
    if (idx.size()==0)
      continue;
    vector<deque<double> > distanceLines (idx.size());
    cluster (distanceLines, tmarker[mt]);
    for (unsigned int i=0; i<distanceLines.size(); i++) {
      for (unsigned int j=0; j<distanceLines[i].size(); j++) {
        distanceTable (j,idx[i])=distanceLines[i][j];
      }
    }
  }
  circleSmooth(marker);
}

void DistMarkerBuilder::circleSmooth (const std::vector<MarkerLog>& marker) {
  for (unsigned int i=0; i<numDistanceLines; i++) {
    if (markerTypes[i]==MarkerLog::NL)
      continue;
    deque<Vec> markerpos;
    for (std::vector<MarkerLog>::const_iterator it=marker.begin(); it!=marker.end(); it++) {
      unsigned int bin = static_cast<unsigned int>(it->angle.get_deg()/8);
      unsigned closestLineIndex=0;
      double closestLineDistance=1e300;
      for (unsigned int j=0; j<numDistanceLines; j++) {
        if (it->type==markerTypes[j]) {
          double d=abs(it->distance-distanceTable(bin,j));
          if (d<closestLineDistance) {
            closestLineDistance=d;
            closestLineIndex=j;
          }
        }
      }
      if (closestLineIndex==i) {
        markerpos.push_back (it->distance*Vec::unit_vector (it->angle));
      }
    }
    Vec center;
    double radius;
    algebraicCircleFit (center, radius, markerpos.begin(), markerpos.end());
    Circle circ (center, radius);
    cerr << "KREIS: " << trueDistances[i] << ' ' << center << ' ' << radius << '\n';
    for (unsigned int j=0; j<45; j++) {
      distanceTable(j,i)=sqrt((center.x+radius*cos(8.*j/180*M_PI))*(center.x+radius*cos(8.*j/180*M_PI))+(center.y+radius*sin(8.*j/180*M_PI))*(center.y+radius*sin(8.*j/180*M_PI)));
    }
  }
}
