
#include "RadialObstacleView.h"
#include "../../WorldModel/WorldModel.h"
#include "../../Fundamental/geometry.h"
#include <cmath>

using namespace Tribots;
using namespace std;

void Tribots::add_obstacle (std::vector<RadialObstacleView>& dest, Time t, Vec c, double w) throw (std::bad_alloc) {
  const RobotLocation& robot = MWM.get_robot_location(t);
  RadialObstacleView newobs;
  newobs.width = w;
  newobs.distance = (robot.pos-c).length();
  newobs.mainangle = (c-robot.pos).angle();
  Angle addangle = Angle::rad_angle (atan2 (0.5*newobs.width, newobs.distance));
  newobs.leftangle = newobs.mainangle+addangle;
  newobs.rightangle = newobs.mainangle-addangle;
  dest.push_back (newobs);
}

void Tribots::make_radial_obstacle_view (std::vector<RadialObstacleView>& dest, Time t, bool consider_ball_as_obstacle, bool consider_boundaries_as_obstacle, bool consider_lines_as_obstacle, bool consider_goals_as_obstacle) throw (std::bad_alloc) {
  const RobotLocation& robot = MWM.get_robot_location(t);
  const BallLocation& ball = MWM.get_ball_location(t);
  const FieldGeometry& fgeom = MWM.get_field_geometry();
  const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
  
   consider_ball_as_obstacle &= (ball.pos_known==BallLocation::known);
  dest.resize (obstacles.size()+(consider_ball_as_obstacle ? 1 : 0)+(consider_boundaries_as_obstacle ? 4 : 0)+(consider_lines_as_obstacle ? 4 : 0)+(consider_goals_as_obstacle ? 32 : 0));

  vector<ObstacleDescriptor>::const_iterator it=obstacles.begin();
  vector<ObstacleDescriptor>::const_iterator itend=obstacles.end();
  vector<RadialObstacleView>::iterator rit = dest.begin();
  while (it<itend) {
    rit->distance = (it->pos-robot.pos).length();
    rit->mainangle = (it->pos-robot.pos).angle();
    Angle addangle = Angle::rad_angle (atan2 (0.5*it->width, rit->distance));
    rit->leftangle = rit->mainangle+addangle;
    rit->rightangle = rit->mainangle-addangle;
    rit->width = it->width;
    it++;
    rit++;
  }
  if (consider_ball_as_obstacle) {
    // Ball als zusaetzliches Hindernis eintragen
    rit->distance = (ball.pos-robot.pos).length();
    rit->width = MWM.get_field_geometry().ball_diameter+200;  // +200 als Sicherheitsmarge (10cm auf beiden Seiten)
    rit->mainangle = (ball.pos-robot.pos).angle();
    Angle addangle = Angle::rad_angle (atan2 (0.5*rit->width, rit->distance));
    rit->leftangle = rit->mainangle+addangle;
    rit->rightangle = rit->mainangle-addangle;
    rit++;
  }
  if (consider_boundaries_as_obstacle) {
    // Spielfeldbegrenzungen als zusaetzliche Hindernisse eintragen
    double xd = 0.5*fgeom.field_width+fgeom.side_band_width;
    double yd = 0.5*fgeom.field_length+fgeom.goal_band_width;
    Line lines [4];
    lines[0] = Line (Vec(xd,yd), Vec(xd,-yd));
    lines[1] = Line (Vec(-xd,yd), Vec(-xd,-yd));
    lines[2] = Line (Vec(xd,yd), Vec(-xd,yd));
    lines[3] = Line (Vec(xd,-yd), Vec(-xd,-yd));
    for (unsigned int i=0; i<4; i++) {
      Vec pp = lines[i].perpendicular_point (robot.pos);
      if (lines[i].side(Vec(0,0))*lines[i].side(robot.pos)<0) {// Roboter ausserhalb
        rit->distance = 10;
        rit->mainangle = (pp-robot.pos).angle()+Angle::half;
      } else {
        rit->distance = (pp-robot.pos).length();
        rit->mainangle = (pp-robot.pos).angle();
      }
      rit->width = 50000;
      Angle addangle = Angle::rad_angle (atan2 (0.5*rit->width, rit->distance));
      rit->leftangle = rit->mainangle+addangle;
      rit->rightangle = rit->mainangle-addangle;
      rit++;
    }
  }
  if (consider_lines_as_obstacle) {
    // Spielfeldrand (weisse Linien) als zusaetzliche Hindernisse eintragen
    double xd = 0.5*fgeom.field_width;
    double yd = 0.5*fgeom.field_length;
    Line lines [4];
    lines[0] = Line (Vec(xd,yd), Vec(xd,-yd));
    lines[1] = Line (Vec(-xd,yd), Vec(-xd,-yd));
    lines[2] = Line (Vec(xd,yd), Vec(-xd,yd));
    lines[3] = Line (Vec(xd,-yd), Vec(-xd,-yd));
    for (unsigned int i=0; i<4; i++) {
      Vec pp = lines[i].perpendicular_point (robot.pos);
      if (lines[i].side(Vec(0,0))*lines[i].side(robot.pos)<0) {// Roboter ausserhalb
        rit->distance = 10;
        rit->mainangle = (pp-robot.pos).angle()+Angle::half;
      } else {
        rit->distance = (pp-robot.pos).length();
        rit->mainangle = (pp-robot.pos).angle();
      }
      rit->width = 50000;
      Angle addangle = Angle::rad_angle (atan2 (0.5*rit->width, rit->distance));
      rit->leftangle = rit->mainangle+addangle;
      rit->rightangle = rit->mainangle-addangle;
      rit++;
    }
  }
  if (consider_goals_as_obstacle) {
    Vec goalpoints [16]; // 8 Hindernisse fuer die Rueck, je zwei fuer die Seitenwaende, je zwei um den Raum hinter dem Tor abzuschotten
    double goalpointswidth [16];
    goalpointswidth[0]=fgeom.goal_width/8.0;
    goalpointswidth[1]=fgeom.goal_width/8.0;
    goalpointswidth[2]=fgeom.goal_width/8.0;
    goalpointswidth[3]=fgeom.goal_width/8.0;
    goalpointswidth[4]=fgeom.goal_width/8.0;
    goalpointswidth[5]=fgeom.goal_width/8.0;
    goalpointswidth[6]=fgeom.goal_width/8.0;
    goalpointswidth[7]=fgeom.goal_width/8.0;
    goalpointswidth[8]=fgeom.goal_length/2.0;
    goalpointswidth[9]=fgeom.goal_length/2.0;
    goalpointswidth[10]=fgeom.goal_length/2.0;
    goalpointswidth[11]=fgeom.goal_length/2.0;
    goalpointswidth[12]=goalpointswidth[13]=goalpointswidth[14]=goalpointswidth[15]=(fgeom.goal_band_width-fgeom.goal_length)/2.0;
    goalpoints[0]=Vec(fgeom.goal_width/16.0, 0.5*fgeom.field_length+fgeom.goal_length);
    goalpoints[1]=Vec(fgeom.goal_width*3.0/16.0, 0.5*fgeom.field_length+fgeom.goal_length);
    goalpoints[2]=Vec(fgeom.goal_width*5.0/16.0, 0.5*fgeom.field_length+fgeom.goal_length);
    goalpoints[3]=Vec(fgeom.goal_width*7.0/16.0, 0.5*fgeom.field_length+fgeom.goal_length);
    goalpoints[4]=Vec(-fgeom.goal_width/16.0, 0.5*fgeom.field_length+fgeom.goal_length);
    goalpoints[5]=Vec(-fgeom.goal_width*3.0/16.0, 0.5*fgeom.field_length+fgeom.goal_length);
    goalpoints[6]=Vec(-fgeom.goal_width*5.0/16.0, 0.5*fgeom.field_length+fgeom.goal_length);
    goalpoints[7]=Vec(-fgeom.goal_width*7.0/16.0, 0.5*fgeom.field_length+fgeom.goal_length);
    goalpoints[8]=Vec(fgeom.goal_width/2.0, 0.5*fgeom.field_length+fgeom.goal_length/4.0);
    goalpoints[9]=Vec(fgeom.goal_width/2.0, 0.5*fgeom.field_length+3.0*fgeom.goal_length/4.0);
    goalpoints[10]=Vec(-fgeom.goal_width/2.0, 0.5*fgeom.field_length+fgeom.goal_length/4.0);
    goalpoints[11]=Vec(-fgeom.goal_width/2.0, 0.5*fgeom.field_length+3.0*fgeom.goal_length/4.0);
    goalpoints[12]=Vec(fgeom.goal_width/2.0, 0.5*fgeom.field_length+fgeom.goal_length+(fgeom.goal_band_width-fgeom.goal_length)/4.0);
    goalpoints[13]=Vec(fgeom.goal_width/2.0, 0.5*fgeom.field_length+fgeom.goal_length+(fgeom.goal_band_width-fgeom.goal_length)*3.0/4.0);
    goalpoints[14]=Vec(-fgeom.goal_width/2.0, 0.5*fgeom.field_length+fgeom.goal_length+(fgeom.goal_band_width-fgeom.goal_length)/4.0);
    goalpoints[15]=Vec(-fgeom.goal_width/2.0, 0.5*fgeom.field_length+fgeom.goal_length+(fgeom.goal_band_width-fgeom.goal_length)*3.0/4.0);
    for (unsigned int i=0; i<16; i++) {
      rit->distance = (goalpoints[i]-robot.pos).length();
      rit->width = goalpointswidth[i];
      rit->mainangle = (goalpoints[i]-robot.pos).angle();
      Angle addangle = Angle::rad_angle (atan2 (0.5*rit->width, rit->distance));
      rit->leftangle = rit->mainangle+addangle;
      rit->rightangle = rit->mainangle-addangle;
      rit++;
      rit->distance = (-goalpoints[i]-robot.pos).length();
      rit->width = goalpointswidth[i];
      rit->mainangle = (-goalpoints[i]-robot.pos).angle();
      addangle = Angle::rad_angle (atan2 (0.5*rit->width, rit->distance));
      rit->leftangle = rit->mainangle+addangle;
      rit->rightangle = rit->mainangle-addangle;
      rit++;
    }
  }
}

void Tribots::visualize_radial_obstacle_view (const std::vector<RadialObstacleView>& obstacles, Vec p) throw () {
  LOUT << "%black solid thin";;
  std::vector<RadialObstacleView>::const_iterator it = obstacles.begin();
  std::vector<RadialObstacleView>::const_iterator it_end = obstacles.end();
  double r=1000;
  while (it<it_end) {
    Vec pl = p+r*Vec::unit_vector(it->leftangle);
    Vec pr = p+r*Vec::unit_vector(it->rightangle);
    LOUT << " line " << pr << ' ' << p << ' ' << pl << " arc " << p << ' ' << r << ' ' << it->rightangle.get_deg() << ' ' << it->leftangle.get_deg();
    it++;
    r+=50;
  }
  LOUT << '\n';
}

double Tribots::scan_radial_obstacle_view (unsigned int& leftindex, unsigned int& rightindex, double& leftdist, double& rightdist, const std::vector<RadialObstacleView>& obstacles, double maxd, Angle cangle) throw () {
  double mind = 1e300;
  leftdist=1e300;
  rightdist=1e300;
  vector<RadialObstacleView>::const_iterator rit = obstacles.begin();
  vector<RadialObstacleView>::const_iterator ritend = obstacles.end();
  Line ray (Vec(0,0), Vec::unit_vector (cangle));
  while (rit<ritend) {
    Angle alpha = cangle-rit->mainangle;
    double cosalpha = cos (alpha.get_rad());
    double truedist = (cosalpha<=1e-20 ? -rit->distance : rit->distance/cosalpha);  // cos wichtig bei sehr breiten Hindernissen, zum Beispiel Spielfeldrand
    if (truedist<maxd && truedist>-200) {  // auch ein wenig zurueckliegende Hindernisse betrachten, um nichts umzurempeln
      if (cangle.in_between (rit->rightangle, rit->leftangle)) {
        // Hindernis im Weg
        if (mind>truedist) {
          mind=truedist;
          leftindex=rightindex=rit-obstacles.begin();
          leftdist=rightdist=0;
          rit++;
          continue;
          // evtl gibt es ein noch naeher im Weg liegendes Hindernis
        }
      }

      if (rit->rightangle.in_between (cangle, cangle+Angle::quarter)) {
        double d = ray.distance (rit->distance*Vec::unit_vector (rit->mainangle)+0.5*rit->width*Vec::unit_vector (rit->mainangle-Angle::quarter));
        if (d<leftdist) {
          leftdist=d;
          leftindex=(rit-obstacles.begin());
        }
      }
      if (rit->leftangle.in_between (cangle-Angle::quarter, cangle)) {
        double d = ray.distance (rit->distance*Vec::unit_vector (rit->mainangle)+0.5*rit->width*Vec::unit_vector (rit->mainangle+Angle::quarter));
        if (d<rightdist) {
          rightdist=d;
          rightindex=(rit-obstacles.begin());
        }
      }
    }
    rit++;
  }
  return mind;
}

double Tribots::cone_radial_obstacle_view (const std::vector<RadialObstacleView>& obstacles, Angle rightangle, Angle leftangle) throw () {
  double mind = 1e300;
  vector<RadialObstacleView>::const_iterator rit = obstacles.begin();
  vector<RadialObstacleView>::const_iterator ritend = obstacles.end();
  while (rit<ritend) {
    if (rit->rightangle.in_between (rightangle, leftangle) || rit->leftangle.in_between (rightangle, leftangle) || rightangle.in_between (rit->rightangle, rit->leftangle)) {
      // Hindernis im Weg
      if (mind>rit->distance)
        mind=rit->distance;
    }
    rit++;
  }

  return mind;
}
