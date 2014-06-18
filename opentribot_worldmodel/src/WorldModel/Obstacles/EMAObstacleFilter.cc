
#include "EMAObstacleFilter.h"
#include "../WorldModel.h"
#include "../../Fundamental/geometry.h"
#include <cmath>

using namespace Tribots;
using namespace std;


EMAObstacleFilter::EMAObstacleFilter (const ConfigReader& reader, const FieldGeometry& fg) throw () : assignments (0), ema (0.6), hysterese_lower (0.2), hysterese_higher(0.7), stuck_obstacle_delay(2000), remove_ball_obstacles (true), min_obstacle_width (0), max_num_recent_pos(10) {
  vector<double> params;
  if (reader.get ("ObstacleFilter::parameters", params) && params.size()>=3) {
    ema = params[0];
    hysterese_lower = params[1];
    hysterese_higher = params[2];
  }
  reader.get ("ObstacleFilter::stuck_obstacle_delay", stuck_obstacle_delay);
  reader.get ("ObstacleFilter::remove_ball_obstacles", remove_ball_obstacles);
  reader.get ("ObstacleFilter::minimal_obstacle_width", min_obstacle_width);
  reader.get ("ObstacleFilter::maximal_number_recent_positions", max_num_recent_pos);

  if (fg.pole_height>0.01 && fg.pole_diameter>0.01) {
    poles.resize (4);
    double dx = 0.5*fg.field_width+fg.pole_position_offset_x;
    double dy = 0.5*fg.field_length+fg.pole_position_offset_y;
    poles[0].pos = Vec(dx,dy);
    poles[1].pos = Vec(-dx,dy);
    poles[2].pos = Vec(-dx,-dy);
    poles[3].pos = Vec(dx,-dy);
    poles[0].width=poles[1].width=poles[2].width=poles[3].width=fg.pole_diameter;
  }
}

EMAObstacleFilter::~EMAObstacleFilter () throw () {;}

ObstacleLocation EMAObstacleFilter::get () const throw () {
  ObstacleLocation oloc;
  oloc.reserve (obstacles.size()+10);
  try{
    deque<ObstacleProperties>::const_iterator it = obstacles.begin();
    deque<ObstacleProperties>::const_iterator itend = obstacles.end();
    while (it<itend) {
      if (it->active) {
        ObstacleDescriptor od;
        od.pos=it->pos;
        od.width=(it->width>min_obstacle_width ? it->width : min_obstacle_width);
        od.player=-1;
        od.velocity=it->velocity;
        oloc.push_back (od);
      }
      it++;
    }
  }catch (std::bad_alloc&) {;} // grrr, einfach nichts zurueckliefern
  return oloc;
}

ObstacleLocation EMAObstacleFilter::get_with_poles () const throw () {
  ObstacleLocation dest = get ();
  dest.insert (dest.end(), poles.begin(), poles.end());
  return dest;
}

ObstacleLocation EMAObstacleFilter::get_with_poles_and_stuck () const throw () {
  ObstacleLocation dest = get ();
  dest.insert (dest.end(), poles.begin(), poles.end());
  Time now;
  RobotLocation rloc = MWM.get_robot_location (now,false);
  if (rloc.stuck.msec_since_stuck<=stuck_obstacle_delay) {
    BallLocation bloc = MWM.get_ball_location (now,false);
    const RobotProperties& rp = MWM.get_robot_properties ();
    const FieldGeometry& fg = MWM.get_field_geometry ();
    double dist = rp.max_robot_radius;
    if ((bloc.pos.toVec()-rloc.pos).length()< rp.max_robot_radius + fg.ball_diameter + 500)
      if (rloc.stuck.dir_of_stuck.angle (bloc.pos.toVec()-rloc.pos).in_between (Angle::seven_eighth, Angle::eighth) && bloc.pos_known==BallLocation::known)
        dist+=fg.ball_diameter;
    ObstacleDescriptor od;
    od.pos=(rloc.stuck.pos_of_stuck+dist*rloc.stuck.dir_of_stuck.normalize());
    od.width=rp.robot_width;
    dest.push_back (od);
  }
  return dest;
}


namespace {

  // Toleranzfunktion fuer Hindernisse, die einander zugeordnet werden in Abhaengigkeit von
  // der Entfernung Roboter-Hindernis
  inline double tolerance_radius (const double& v) {
    return (v<2000 ? 500 : (v>4000 ? 1000 : v/4));
  }

}

void EMAObstacleFilter::update (const VisibleObjectList& vis, const RobotLocation& rloc, const BallLocation& bloc) throw () {
  try{
    // eine "very greedy" Zuordnung zwischen gespeicherten und neuen Hindernissen aufbauen
    assignments.clear();
    vector<VisibleObject>::const_iterator visit (vis.objectlist.begin());
    deque<ObstacleProperties>::iterator obsit;
    unsigned int size_vis = vis.objectlist.size();
    unsigned int size_obs = obstacles.size();
    AssignmentProperties ap;
    for (unsigned int i=0; i<size_vis; i++) {
      ap.pos = rloc.pos+(visit->pos).rotate(rloc.heading);
      ap.near_robot = (visit->pos.length()<1000);
      ap.width = visit->width;
      ap.nearest_index = -1;
      ap.nearest_dist = 1e100;
      double tolerance = tolerance_radius ((visit->pos).length());
      obsit = obstacles.begin();

      for (unsigned int j=0; j<size_obs; j++) {
        double d=(obsit->pos-ap.pos).length();
        if (d<=tolerance && d<ap.nearest_dist) {
          ap.nearest_dist=d;
          ap.nearest_index=j;
        }
        obsit++;
      }
      assignments.push_back (ap);
      visit++;
    }

    // jetzt stehen in assignments alle gesehenen Hindernisse mit iherer Position in Weltkoordinaten und Breite
    // sowie das naechste gespeicherte Hindernis (oder -1) mit dem jeweiligen Abstand

    obsit = obstacles.begin();
    vector<AssignmentProperties>::iterator asit;
    unsigned int size_as = assignments.size();
    int i=0;
    while (obsit<obstacles.end()) {
      asit = assignments.begin();
      int index = -1;
      double mindist = 1e100;
      for (unsigned int j=0; j<size_as; j++) {
        if (asit->nearest_index==i) {
          if (index>=0) {
            if (asit->nearest_dist < assignments[index].nearest_dist) {
              assignments[index].nearest_index=-1;
              index=j;
              mindist=asit->nearest_dist;
            } else
              asit->nearest_index=-1;
          } else {
            index=j;
            mindist=asit->nearest_dist;
          }
        }
        asit++;
      }

      // jetzt steht in index, ob es ein naechstes gesehenenes Hindernis gibt (-1/>=0) und ggf. sein Index in assignments
      // jetzt muss noch der Update fuer obsit gemacht werden
      if (index>=0) {
        TPos np;
        np.pos=assignments[index].pos;
        np.t=vis.timestamp;
        if (obsit->recentPositions.size()<max_num_recent_pos) {
          obsit->recentPositions.insert (np);
        } else {
          obsit->recentPositions.set (np);
        }
        obsit->recentPositions.step();
        obsit->pos= (assignments[index].near_robot ? assignments[index].pos : ema*obsit->pos+(1-ema)*assignments[index].pos);
        obsit->width=ema*obsit->width+(1-ema)*assignments[index].width;
        obsit->probability=ema*obsit->probability+1-ema;
        if (!obsit->active && obsit->probability>=hysterese_higher)
          obsit->active=true;
        obsit++;        // falls Hindernis gesehen, auf jeden Fall beibehalten
      } else {
        obsit->probability*=ema;
        if (obsit->probability>=hysterese_lower)
          obsit++;      // Hindernis nicht gesehen, trotzdem beibehalten
        else
          obsit = obstacles.erase (obsit);  // Hindernis entfernen
      }
      i++;
    }

    // jetzt noch nicht zugeordnete gesehene Hindernisse einfuegen
    asit = assignments.begin();
    ObstacleProperties op;
    for (unsigned int i=0; i<size_as; i++) {
      if (asit->nearest_index==-1) {
        op.pos=asit->pos;
        op.width=asit->width;
        op.probability=1-ema;
        op.active=false;
        TPos np;
        np.pos=asit->pos;
        np.t=vis.timestamp;
        op.recentPositions.set (np);
        obstacles.push_back (op);
      }
      asit++;
    }

    // Hindernisse vor dem Ball entfernen
    if (remove_ball_obstacles) {
      try{
        Line robot_ball (bloc.pos.toVec(), rloc.pos);
        double rbd2 = (bloc.pos-rloc.pos).squared_length();
        deque<ObstacleProperties>::iterator oit = obstacles.begin();
        deque<ObstacleProperties>::iterator oitend = obstacles.end();
        while (oit<oitend) {
          Vec p = robot_ball.perpendicular_point (oit->pos);
          if ((oit->width<300) && ((oit->pos-bloc.pos.toVec()).squared_length()<90000) && ((p-(oit->pos)).squared_length()<40000) && ((p-rloc.pos).squared_length()<rbd2)) {
            oit = obstacles.erase (oit);
            oitend = obstacles.end();
          } else
            oit++;
        }
      }catch(std::invalid_argument&){;} // Ball- und Roboterposition identisch
    }

    // Geschwindigkeiten schaetzen:
    for (std::deque<ObstacleProperties>::iterator it = obstacles.begin(); it!=obstacles.end(); it++) {
      if (it->recentPositions.size()<3)
        it->velocity=Vec(0,0);
      else {
        unsigned int n=it->recentPositions.size();
        double sum_ts=0;
        double sum_ts2=0;
        Vec sum_pos (0,0);
        Vec sum_ts_pos (0,0);
        for (unsigned int i=0; i<n; i++) {
          double tau = it->recentPositions.get(i).t.diff_msec (vis.timestamp);
          Vec p = it->recentPositions.get(i).pos;
          sum_ts+=tau;
          sum_ts2+=tau*tau;
          sum_pos+=p;
          sum_ts_pos+=tau*p;
        }
        double det = n*sum_ts2-sum_ts*sum_ts;
        if (abs(det)<1e-5)
          it->velocity=Vec(0,0);
        else {
          it->velocity=(-sum_ts*sum_pos+static_cast<double>(n)*sum_ts_pos)/det;
        }
      }
    }

  }catch(std::bad_alloc){;}   // grrr. besser nichts anstellen, Programm stuerzt eh gleich ab ...
}
