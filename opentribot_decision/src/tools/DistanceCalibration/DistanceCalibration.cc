
#include "DistanceCalibration.h"
#include "../../Structures/Journal.h"
#include "../../Fundamental/Table.h"
#include "../../WorldModel/WorldModel.h"
#include <algorithm>
#include <fstream>
#include <cstdio>
#include <cmath>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

DistanceCalibration::DistanceCalibration (const vector<double>& realDistances1, 
                                          const vector<double>& prototypes,
                                          const std::vector<TransitionMarker>& markerType,
                                          Tribots::Vec p,
                                          double minClusterDistance,
                                          const string& markerLogFile)
: image_center (p),
  minClusterDistance(minClusterDistance), logMarkers(false)
{
  for (unsigned int i=0; i<realDistances1.size(); i++) {
    realDistances[markerType[i]].push_back (realDistances1[i]);
    prototypesInit[markerType[i]].push_back (20+20*i);
    markerNum[markerType[i]].push_back (i);
  }
  if (markerLogFile != "") {
    mout.open(markerLogFile.c_str());
    logMarkers = true;
  }
}

DistanceCalibration::~DistanceCalibration () 
{
  mout << flush;
  mout.close();
}

double DistanceCalibration::search_min_transition_index (const std::deque<MarkerInfo>& m, double dmin) {
  double min=1e300;
  for (unsigned int i=0; i<m.size(); i++)
    if (m[i].image_distance>dmin && m[i].image_distance<min) {
      min = m[i].image_distance;
    }
      return min;
}

void DistanceCalibration::search_markers (const Tribots::ScanResultList& scan_result) {
  
  deque<MarkerInfo> white_blue;  // Uebergaenge von weiss (innen) nach blau (aussen)
  deque<MarkerInfo> blue_white; // Uebergaenge von blau (innen) nach weiss (aussen)
  
  vector<Transition>::const_iterator it = 
    scan_result.results[COLOR_BLUE]->transitions.begin();
  vector<Transition>::const_iterator itend = 
    scan_result.results[COLOR_BLUE]->transitions.end();
  
  while (it<itend) {
    MarkerInfo mi((it->toPos-image_center).angle(), 
                  (it->toPos-image_center).length());
    if (it->type==Transition::START && 
        (it->from == COLOR_LINE || it->twoStep==COLOR_LINE)) {
      white_blue.push_back (mi);
    } else if (it->type==Transition::END && 
               (it->to == COLOR_LINE || it->twoStep==COLOR_LINE)) {
      blue_white.push_back(mi);
    }
    it++;
  }
  
  // copy transitions to markers
  for (unsigned int i=0; i<blue_white.size(); i++) {
    markers[blue_white_change].push_back (blue_white[i]);
    if (logMarkers) {
      mout << blue_white[i].angle.get_deg() << " "
          << blue_white[i].image_distance << " bw" << endl;
    }
  }
  for (unsigned int i=0; i<white_blue.size(); i++) {
    markers[white_blue_change].push_back (white_blue[i]);
    if (logMarkers) {
      mout << white_blue[i].angle.get_deg() << " "
          << white_blue[i].image_distance << " wb" << endl;
    }
  }
  
  // now look for the ball transitions 
  // at the moment: no special treatment during clustering
  it = scan_result.results[COLOR_BALL]->transitions.begin();
  itend = scan_result.results[COLOR_BALL]->transitions.end();
  bool foundBallStart = false;
  MarkerInfo ballStart;
  
  while (it != itend) {
    if (it->type==Transition::START) {  // suche noch nach Ballanfang
      if ((it->from == COLOR_LINE || it->twoStep == COLOR_LINE)) {
        ballStart = MarkerInfo((it->toPos-image_center).angle(),
                               (it->toPos-image_center).length());
        markers[white_red_change].push_back(ballStart);
        if (logMarkers) {
          mout << ballStart.angle.get_deg() << " " << ballStart.image_distance << " wr" << endl;
        }
        foundBallStart = true;
        it++;
        continue;
      }
    }
    if (foundBallStart && it->type == Transition::END) {  // suche bereits nach ball ende; Farbe nicht beachten, koennte auch don't care sein (am Rand)
      MarkerInfo mi((it->fromPos-image_center).angle(),
                    (it->fromPos-image_center).length());
      MarkerInfo ballMiddle(ballStart.angle, (ballStart.image_distance+mi.image_distance) *.5);
       markers[red_middle_change].push_back(ballMiddle);
      if (logMarkers) {
        mout << ballMiddle.angle.get_deg() << " " << ballMiddle.image_distance << " rm" << endl;
      }
      
      markers[red_white_change].push_back(mi);
      if (logMarkers) {
        mout << mi.angle.get_deg() << " " << mi.image_distance << " rw" << endl;
      }
      break; // both transitions found since line is assumed to not start in ball
    }
    it++;
  }
}



void DistanceCalibration::writeMarkerFile (const std::string& filename) {
  unsigned int num_markers = 0;
  for (unsigned int i=0; i<6; i++)
    num_markers+=realDistances[i].size();

  // zuerst Tabelle berechnen
  Table<double> result_table (45, num_markers);
  for (unsigned int nu=0; nu<6; nu++) {
    if (realDistances[nu].size()>=1) {
      if (nu==static_cast<unsigned int>(virtual_zero_change)) {
        // Fall eines virtuellen Null-Markers, diese muessen explizit gesetzt werden
        for (unsigned int angleiter=0; angleiter<result_table.rows(); angleiter++) {
          for (unsigned int i=0; i<realDistances[nu].size(); i++) {
            result_table(angleiter, markerNum[nu][i])=0.0;
          }
        }
      } else {
        // Fall "richtiger" Marker, die in marker[nu] stehen
        unsigned int num_markers_type = prototypesInit[nu].size();
        vector<double> values (num_markers_type);
        vector<double> values2 (num_markers_type);
        vector<unsigned int> value_count (num_markers_type);
        vector<unsigned int> iter_wo_assignments (num_markers_type);
        vector<double> prototypes = prototypesInit[nu];
        vector<deque<double> > vischange (num_markers_type);
  
        for (int angleiter=-90; angleiter<45; angleiter++) {  // Iterationen -90 ... -1 zum Einschwingen
          Angle cangle = Angle::deg_angle (angleiter*8);
          Angle leftangle = Angle::deg_angle (angleiter*8+10);
          Angle rightangle = Angle::deg_angle (angleiter*8-10);
          for (unsigned int i=0; i<num_markers_type; i++) {
            values[i]=6*prototypes[i];   // Regularisierung
            value_count[i]=6;
          }
      
          unsigned int trial=0;
          bool clusters_okay=false;
          while (!clusters_okay && trial< 5) {
            trial++;
            for (unsigned int kmeansiter=0; kmeansiter<10; kmeansiter++) {
              for (unsigned int i=0; i<vischange.size(); i++)
                vischange[i].clear();
              for (unsigned int i=0; i<num_markers_type; i++) {
                values[i]=3*prototypes[i];
                values2[i]=3*prototypes[i]*prototypes[i];
                value_count[i]=3;
                vischange[i].push_back (prototypes[i]);
                vischange[i].push_back (prototypes[i]);
                vischange[i].push_back (prototypes[i]);
              }
              deque<MarkerInfo>::const_iterator it = markers[nu].begin();
              deque<MarkerInfo>::const_iterator itend = markers[nu].end();
              while (it<itend) {
                if (it->angle.in_between(rightangle, leftangle)) {
                  unsigned int bestprototype=0;
                  double bestdist=1e300;
                  for (unsigned int i=0; i<num_markers_type; i++) {
                    double d = abs(prototypes[i]-it->image_distance);
                    if (d<bestdist) {
                      bestdist=d;
                      bestprototype=i;
                    }
                  }
                  values[bestprototype]+=it->image_distance;
                  values2[bestprototype]+=it->image_distance*it->image_distance;
                  vischange[bestprototype].push_back (it->image_distance);
                  value_count[bestprototype]++;
                  iter_wo_assignments[bestprototype]=0;
                }
                it++;
              }
              for (unsigned int i=0; i<num_markers_type; i++) {
                if (num_markers_type==1) {
                  sort (vischange[i].begin(), vischange[i].end());
                  prototypes[i]=0.5*(vischange[i][vischange[i].size()/2]+vischange[i][(vischange[i].size()-1)/2]);  // Median, wegen Robustheit
                } else {
                  prototypes[i]=values[i]/=(static_cast<double>(value_count[i])+1e-20);  // 1e-20 um Division durch 0 zu vermeiden
                }
                values2[i]=values2[i]/(static_cast<double>(value_count[i])+1e-20)-values[i]*values[i];
              }
          
            }
            clusters_okay=true;
            unsigned int similar_clusters=0;
            unsigned int cluster_wo_assignments=0;
            for (unsigned int i=0; i<num_markers_type-1; i++) {
              if (abs(values[i]-values[i+1])<=minClusterDistance) {
                clusters_okay=false;
                similar_clusters=i+1;
              }
            }
            for (unsigned int i=0; i<num_markers_type; i++) {
              iter_wo_assignments[i]++;
              if (iter_wo_assignments[i]>10) {
                clusters_okay=false;
                cluster_wo_assignments=i;
              }
            }
            if (angleiter>=0)
              clusters_okay=true;
            if (!clusters_okay) {
              unsigned int broad_cluster=0;
              double max_var=0;
              for (unsigned int i=0; i<num_markers_type; i++) {
                if (values2[i]>max_var && i!=similar_clusters && i!=cluster_wo_assignments) {
                  max_var = values2[i];
                  broad_cluster=i;
                }
              }
          
              unsigned int min_cluster = (similar_clusters>0 ? similar_clusters : cluster_wo_assignments);
              if (min_cluster<broad_cluster) {
                for (unsigned int i=min_cluster; i<broad_cluster; i++)
                  prototypes[i]=prototypes[i+1];
                prototypes[broad_cluster-1]-=2;
                prototypes[broad_cluster]+=2;
              } else if (min_cluster>broad_cluster) {
                for (unsigned int i=min_cluster; i>broad_cluster; i--)
                  prototypes[i]=prototypes[i-1];
                prototypes[broad_cluster]-=2;
                prototypes[broad_cluster+1]+=2;
              }
            }      
          }
  
          if (angleiter>=0) {
            // die Werte in die Tabelle eintragen
            for (unsigned int j=0; j<num_markers_type; j++) {
              result_table(angleiter, markerNum[nu][j]) = prototypes[j];
            }
          }
        }
      }
    }
  }
  
  // jetzt steht in result_table eine erste Version der Distanztabelle
  // Tabelle noch etwas glaetten (Median-3, Mittelwert-5-Filter)
  unsigned int row_num = result_table.rows();
  unsigned int column_num = result_table.columns();
  Table<double> result_table_im (row_num, column_num);
  for (unsigned int zeile=0; zeile<row_num; zeile++) {
    for (unsigned int spalte=0; spalte<column_num; spalte++) {
      double v1 = result_table ((zeile+row_num-1)%row_num, spalte);
      double v2 = result_table (zeile, spalte);
      double v3 = result_table ((zeile+1)%row_num, spalte);
      result_table_im (zeile, spalte) = (v1>v2 ? (v2>v3 ? v2 : (v3>v1 ? v1 : v3)) : (v1>v3 ? v1 : (v2>v3 ? v3 : v2)));  // median (v1,v2,v3)
    }
  }
  for (unsigned int zeile=0; zeile<row_num; zeile++) {
    for (unsigned int spalte=0; spalte<column_num; spalte++) {
      double v1 = result_table_im ((zeile+row_num-2)%row_num, spalte);
      double v2 = result_table_im ((zeile+row_num-1)%row_num, spalte);
      double v3 = result_table_im (zeile, spalte);
      double v4 = result_table_im ((zeile+1)%row_num, spalte);
      double v5 = result_table_im ((zeile+2)%row_num, spalte);
      result_table_im (zeile, spalte) = (v1+v2+v3+v4+v5)/5.0;  // mean (v1,v2,v3,v4,v5)
    }
  }
  
  // Ergebnistabelle in die Datei schreiben
  string filenamebak = filename+".bak";
  if (rename (filename.c_str(), filenamebak.c_str())!=0)
    JERROR ("Kann Markerdatei nicht zu .bak-Datei umbenennen");
  ofstream dest (filename.c_str());
  if (!dest)
    JERROR ("Kann Markerdatei nicht schreiben!");
  
  dest << 640 << ' ' << 480 << ' ' << static_cast<int>(image_center.x) << ' ' <<static_cast<int>(image_center.y) << '\n';
  dest << 3 << ' ' << num_markers << '\n';
  vector<double> allRealDistances;
  for (unsigned int i=0; i<6; i++)
    for (unsigned int j=0; j<realDistances[i].size(); j++)
      allRealDistances.push_back (realDistances[i][j]);
  sort (allRealDistances.begin(), allRealDistances.end());
  for (unsigned int i=0; i<num_markers; i++)
    dest << allRealDistances[i] << ' ';
  dest << '\n';
  
  for (unsigned int zeile = 0; zeile<row_num; zeile++) {
    dest << zeile << " 0";  // weshalb die Null, weiss ich nicht
    for (unsigned int spalte=0; spalte<column_num; spalte++) {
      dest << ' ' << result_table (zeile, spalte);
    }
    dest << endl;
  }
}


