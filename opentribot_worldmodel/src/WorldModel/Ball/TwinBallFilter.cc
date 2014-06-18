
#include "TwinBallFilter.h"
#include <cmath>

using namespace Tribots;
using namespace std;


TwinBallFilter::TwinBallFilter (const ConfigReader& reader, const OdometryContainer& odobox) throw (std::bad_alloc) : sl_filter (reader), odo_filter (reader, odobox) {;}

void TwinBallFilter::update (const VisibleObjectList& vis, const RobotLocation& rpos) throw () {
  sl_filter.update (vis, rpos);
  odo_filter.update (vis, rpos);
}

BallLocation TwinBallFilter::get (const Time t) const throw () {
  BallLocation sl = sl_filter.get(t);
  BallLocation odo = odo_filter.get(t);
  if (odo.pos_known==sl.pos_known) {
    BallLocation dest;
    double q1 = (sl.quality+0.01)/(sl.quality+odo.quality+0.02);
    double q2 = (odo.quality+0.01)/(sl.quality+odo.quality+0.02);
    dest.pos=q1*sl.pos+q2*odo.pos;
    dest.velocity=q1*sl.velocity+q2*odo.velocity;
    dest.quality = (sl.quality>odo.quality ? sl.quality : odo.quality);
    dest.pos_known = odo.pos_known;
    dest.lastly_seen = (sl.lastly_seen>odo.lastly_seen ? sl.lastly_seen : odo.lastly_seen);
    return dest;
  } else if (odo.pos_known==BallLocation::known) 
    return odo;
  else if (sl.pos_known==BallLocation::known)
    return sl;
  else if (odo.pos_known==BallLocation::raised)
    return odo;
  else
    return sl;
}

