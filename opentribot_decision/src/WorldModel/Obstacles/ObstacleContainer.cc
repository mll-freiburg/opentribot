
#include "ObstacleContainer.h"
#include "../WorldModel.h"

using namespace Tribots;

ObstacleContainer::ObstacleContainer (const ConfigReader& cfg, const FieldGeometry& fg) throw () {
  stuck_obstacle_delay=2000;
  cfg.get ("ObstacleFilter::stuck_obstacle_delay", stuck_obstacle_delay);
  if (fg.pole_diameter>0.01 && fg.pole_height>0.01) {
    poles.resize(4);
    double dx = 0.5*fg.field_width+fg.pole_position_offset_x;
    double dy = 0.5*fg.field_length+fg.pole_position_offset_y;
    poles[0].pos = Vec(dx,dy);
    poles[1].pos = Vec(-dx,dy);
    poles[2].pos = Vec(-dx,-dy);
    poles[3].pos = Vec(dx,-dy);
    poles[0].width=poles[1].width=poles[2].width=poles[3].width=fg.pole_diameter;
  }
}

ObstacleLocation ObstacleContainer::get () const throw () {
  return obstacles;
}

ObstacleLocation ObstacleContainer::get_with_poles () const throw () {
  ObstacleLocation dest = obstacles;
  dest.insert (dest.end(), poles.begin(), poles.end());
  return dest;
}

ObstacleLocation ObstacleContainer::get_with_poles_and_stuck () const throw () {
  ObstacleLocation dest = get ();
  dest.insert (dest.end(), poles.begin(), poles.end());
  Time now;
  RobotLocation rloc = MWM.get_robot_location (now, false);
  if (rloc.stuck.msec_since_stuck<=stuck_obstacle_delay) {
    BallLocation bloc = MWM.get_ball_location (now, false);
    const RobotProperties& rp = MWM.get_robot_properties ();
    const FieldGeometry& fg = MWM.get_field_geometry ();
    double dist = rp.max_robot_radius;
    if ((bloc.pos-rloc.pos).length()< rp.max_robot_radius + fg.ball_diameter + 500)
      if (rloc.stuck.dir_of_stuck.angle (bloc.pos.toVec()-rloc.pos).in_between (Angle::seven_eighth, Angle::eighth) && bloc.pos_known==BallLocation::known)
        dist+=fg.ball_diameter;
    ObstacleDescriptor od;
    od.pos=(rloc.stuck.pos_of_stuck+dist*rloc.stuck.dir_of_stuck.normalize());
    od.width=rp.robot_width;
    dest.push_back (od);
  }
  return dest;
}

void ObstacleContainer::update (const VisibleObjectList& vis, const RobotLocation& rpos) throw () {
  unsigned int n=vis.objectlist.size();
  obstacles.resize(n);
  for (unsigned int i=0; i<n; i++) {
    obstacles[i].pos = rpos.pos+vis.objectlist[i].pos.rotate(rpos.heading);
    obstacles[i].width = vis.objectlist[i].width;
  }
}

void ObstacleContainer::update (const std::vector<Vec>& p, const std::vector<double>& w, Time) throw () {
  obstacles.resize(p.size());
  for (unsigned int i=0; i<p.size(); i++) {
    obstacles[i].pos=p[i];
    obstacles[i].width=w[i];
  }
}
