
#include "RobotPositionPredictor.h"
#include "../Prediction/update_robot_location.h"
#include <cmath>

using namespace Tribots;


RobotPositionPredictor::RobotPositionPredictor () throw () {
  current_rloc.pos=Vec::zero_vector;
  current_rloc.vtrans=Vec::zero_vector;
  current_rloc.vrot=0;
}

void RobotPositionPredictor::set (const RobotLocation& r, Time t1, Time t2) throw () {
  current_rloc=r;
  timestamp_current_rloc=t1;
  timestamp_calculation=t2;
}

RobotLocation RobotPositionPredictor::get (Time t) const throw () {
  double dt1 = t.diff_msec(timestamp_current_rloc);
  double dt2 = t.diff_msec(timestamp_calculation);
  RobotLocation rloc = update_robot_location (current_rloc, dt1);
  rloc.valid &= (dt2<2500);

  return rloc;
}

