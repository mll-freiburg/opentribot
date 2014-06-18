
#include <cmath>
#include "update_robot_location.h"

using namespace Tribots;

RobotLocation Tribots::update_robot_location (const RobotLocation& start, double dt) throw () {
  // Prinzip: Euler-Verfahren
  RobotLocation end=start;
  double turning (start.vrot*dt*1e-3);
  end.heading=start.heading+Angle::rad_angle (turning);
  end.vtrans=start.vtrans.rotate (Angle::rad_angle (turning));
  if (start.vrot==0) {
    // geradlinige Fahrt ohne Drehung
    end.pos=start.pos+start.vtrans*dt;
  } else {
    // gekruemmte Fahrt auf einem Kreisbogen oder Drehung auf der Stelle
    Vec vtrans_rob = start.vtrans.rotate (-start.heading);
    double sf = std::sin(turning)/start.vrot*1e3;
    double cf = (std::cos(turning)-1.0)/start.vrot*1e3;
    Vec trans_rob (sf*vtrans_rob.x+cf*vtrans_rob.y, -cf*vtrans_rob.x+sf*vtrans_rob.y);
    end.pos=start.pos+trans_rob.rotate (start.heading);
  }
  return end;
}

RobotLocation Tribots::flip_sides (const RobotLocation& src, int dir) throw () {
  if (dir>0) return src;
  RobotLocation dest = src;
  dest.pos*=-1;
  dest.heading+=Angle::half;
  dest.vtrans*=-1;
  dest.stuck.pos_of_stuck*=-1;
  return dest;
}

BallLocation Tribots::flip_sides (const BallLocation& src, int dir) throw () {
  if (dir>0) return src;
  BallLocation dest = src;
  dest.pos.x*=-1;
  dest.pos.y*=-1;
  dest.velocity.x*=-1;
  dest.velocity.y*=-1;
  return dest;
}

ObstacleLocation Tribots::flip_sides (const ObstacleLocation& src, int dir) throw () {
  if (dir>0) return src;
  ObstacleLocation dest = src;
  std::vector<ObstacleDescriptor>::iterator it = dest.begin();
  std::vector<ObstacleDescriptor>::iterator itend = dest.end();
  while (it<itend)
    (it++)->pos *= -1;
  return dest;
}

RobotLocation Tribots::update_robot_location (const RobotLocation& start, double dt, const DriveVector& dvtg) throw () {
  // Prinzip: Heun-Verfahren
  // Vorwaertsintegration, Integration mit Anfangsgeschwindigkeit:
  RobotLocation end=update_robot_location (start, dt);
  // Rueckwartsintegration, Integration mit Endgeschwindigkeit:
  RobotLocation start2=start;
  start2.vrot=dvtg.vrot;
  start2.vtrans=dvtg.vtrans.rotate (start.heading);
  RobotLocation end2=update_robot_location (start2, dt);
  // Mittelung:
  end2.pos=0.5*(end2.pos+end.pos);
  end2.heading+=Angle::rad_angle(((end.heading-end2.heading).get_rad_pi())/2);
  return end2;
}
