
#include "freeCorridor.h"
#include "../../Fundamental/geometry.h"
#include "../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

double Tribots::obstacle_distance_to_line (Vec s, Vec t, const ObstacleLocation& obstacles, bool ign) throw () {
  try{
    Line st (s,t);
    vector<ObstacleDescriptor>::const_iterator it = obstacles.begin();
    double minimal_distance = 1e300;
    Vec ts = t-s;
    double s_ts = (s*ts);
    double t_ts = (t*ts);
    while (it!=obstacles.end()) {
      double p_ts = (it->pos*ts);
      if (p_ts>=s_ts && p_ts<=t_ts && (!ign || it->player<0)) { // Punkt liegt zwischen s und t
        double d = abs(st.distance(it->pos)-(it->width/2.0)); // abs noetig???
        if (d<minimal_distance)
          minimal_distance = d;
      }
      it++;
    }
    if (minimal_distance<0)
      minimal_distance=0;
    return minimal_distance;
  }catch(invalid_argument&) { return 1e300; } // Fall s==t
}

double Tribots::obstacle_distance_to_line_inside_field (Vec s, Vec t, const ObstacleLocation& obstacles, bool ign) throw () {
  const FieldGeometry& fgeom = MWM.get_field_geometry();
  Vec u = t-s;
  double tau = 1;
  if (u.x>0) {
    double taum=(0.5*fgeom.field_width-s.x)/u.x;
    if (taum<tau) tau=taum;
  }
  if (u.x<0) {
    double taum=(-0.5*fgeom.field_width-s.x)/u.x;
    if (taum<tau) tau=taum;
  }
  if (u.y>0) {
    double taum=(0.5*fgeom.field_length-s.y)/u.y;
    if (taum<tau) tau=taum;
  }
  if (u.y<0) {
    double taum=(-0.5*fgeom.field_length-s.y)/u.y;
    if (taum<tau) tau=taum;
  }
  if (tau<=0)
    return 1e300;
  return Tribots::obstacle_distance_to_line (s, s+tau*u, obstacles, ign);
}

double Tribots::distance_to_obstacles (Vec s, Angle a, double w, const ObstacleLocation& obstacles, bool ign) throw () {
  Vec ts = 100000*Vec::unit_vector(a);
  Vec t = s+ts;
  Line st (s, t);
  double minimal_distance=1e300;
  double s_ts = (s*ts);
  vector<ObstacleDescriptor>::const_iterator it = obstacles.begin();
  while (it!=obstacles.end()) {
    double p_ts = (it->pos*ts);
    if (p_ts>=s_ts && (!ign || it->player<0)) {  // Punkt liegt in der entsprechenden Richtung
      Vec pp = st.perpendicular_point (it->pos);
      double d=(pp-it->pos).length()-it->width;
      if (d<w) {  // Punkt liegt naeher als gefordert
        double d2=(pp-s).length();
        if (d2<minimal_distance)
          minimal_distance=d2;
      }
    }
    it++;
  }
  return minimal_distance;
}
