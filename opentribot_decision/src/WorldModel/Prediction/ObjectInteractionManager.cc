
#include "ObjectInteractionManager.h"
#include "../WorldModel.h"
#include "../../Fundamental/geometry.h"
#include <cmath>

using namespace Tribots;
using namespace std;

#define DEBUG_IA 0


void ObjectInteractionManager::get (RobotLocation& destr, BallLocation& destb, ObstacleLocation& desto, const RobotLocation& srcr, const BallLocation& srcb, unsigned int interval) const throw (std::bad_alloc) {
  const double robot_radius = MWM.get_robot_properties().min_robot_radius;
  const double ball_radius = 0.5*MWM.get_field_geometry().ball_diameter;
  const double robot_radius2 = robot_radius*robot_radius;
  const double ball_radius2 = ball_radius*ball_radius;

  vector<Vec> opos1 (desto.size());  // Hindernisbegrenzungen links
  vector<Vec> opos2 (desto.size());  // Hindernisbegrenzungen rechts
  vector<ObstacleDescriptor>::const_iterator it = desto.begin();
  vector<ObstacleDescriptor>::const_iterator itend = desto.end();
  vector<Vec>::iterator ito1 = opos1.begin();
  vector<Vec>::iterator ito2 = opos2.begin();
  vector<Vec>::iterator ito1end = opos1.end();
  while (it<itend) {
    Vec cc = it->pos-srcr.pos;
    Vec ccnn = cc.rotate_quarter().normalize();
    (*ito1) = it->pos+0.5*it->width*ccnn;
    (*ito2) = it->pos-0.5*it->width*ccnn;
    it++;
    ito1++;
    ito2++;
  }

  // fuehre Schritt-fuer-Schritt-Analyse durch mit Intervalllaengen von `ivlen` msec
  // der Einfachheit halber nehme geradlinige Bewegung des Roboters an
  int ivlen = 15;
  unsigned int ct = 0;   // Zeit im aktuellen Praediktionsschritt in ms nach Start
  Vec rp = srcr.pos;     // aktuelle Roboterposition
  Vec bp = srcb.pos.toVec();  // aktuelle Ballposition
  Angle rh = srcr.heading;  // aktuelle Roboterausrichtung

  Vec rv = (destr.pos-srcr.pos)/(0.001+abs(static_cast<double>(interval)));  // linearisierte Robotergeschwindigkeit in Weltkoordinaten
  Vec bv = srcb.velocity.toVec();  // Ballgeschwindigkeit in Weltkoordinaten

#if DEBUG_IA
  LOUT << "\% blue cross " << rp.x << ' ' << rp.y << " dark_red cross " << bp.x << ' ' << bp.y << '\n';
#endif

  while (ct<interval) {
    int steplen = interval-ct;
    if (steplen>ivlen)
      steplen=ivlen;   // fuehre eine Praediktion um steplen ms durch
    ct+=steplen;

    Vec newrp = rp + steplen*rv;
    Angle newrh = rh + Angle::rad_angle (1e-3*steplen*srcr.vrot);
    Vec newbp = bp + steplen*bv;

    vector<Vec>::iterator ito1 = opos1.begin();
    vector<Vec>::iterator ito2 = opos2.begin();
    bool indirect_interaction=false;
    while (ito1<ito1end) {
      // gehe alle Hindernisse durch:
      bool obst_ball_interaction=false;
      bool robot_ball_interaction=false;
      if (srcb.pos_known==BallLocation::known && ((newbp-(*ito1)).squared_length()<ball_radius2 || (newbp-(*ito2)).squared_length()<ball_radius2 || (newbp-0.5*((*ito1)+(*ito2))).squared_length()<ball_radius2)) {
        // Ball gegen Hindernis
        newbp=bp;
        obst_ball_interaction=true;
#if DEBUG_IA
        LOUT << "Ball-Obstacle-Interaction\n";
#endif
      }
      if ((newrp-(*ito1)).squared_length()<robot_radius2 || (newrp-(*ito2)).squared_length()<robot_radius2 || (newrp-0.5*((*ito1)+(*ito2))).squared_length()<robot_radius2) {
        // Roboter gegen Hindernis
        newrp=rp;
#if DEBUG_IA
        LOUT << "Robot-Obstacle-Interaction\n";
#endif
      }
      robot_ball_interaction = ((newbp-newrp).length()<(robot_radius + ball_radius)) && srcb.pos_known==BallLocation::known;

      if (obst_ball_interaction && robot_ball_interaction) {
        // moeglicherweise ist Ball zwischen Hindernis und Roboter eingeklemmt, genauer ueberpruefen
        Angle angle_left = ((*ito1)-newrp).angle();
        Angle angle_right = ((*ito2)-newrp).angle();
        if (rv.angle().in_between (angle_right, angle_left) && (newbp-newrp).angle().in_between (angle_right, angle_left)) {
          // eingeklemmter Ball -> indirekte Beeinflussung des Roboters durch das Hindernis
          newrp=rp;
          indirect_interaction=true;
#if DEBUG_IA
          LOUT << "Indirect Robot-Ball-Interaction\n";
#endif
        }
      }
      ito1++;
      ito2++;
    }
    if (!indirect_interaction &&  srcb.pos_known == BallLocation::known && ((newbp-newrp).length()<(robot_radius + ball_radius + 100))) {
      // Kontakt zwischen Roboter und Ball ohne Hindernisbeteiligung
      // Ball ausserhalb von Roboterradius schieben
      Line robotline (rp, rp+Vec::unit_vector (rh+Angle::quarter));
      Vec pp = robotline.perpendicular_point (bp);
      bool ball_in_front = (pp-bp).length()<200 && ((pp-rp)*Vec::unit_vector (rh+Angle::quarter))>=0;
      if (ball_in_front) {
        newbp = newrp + (bp-rp)*(newrh-rh);
        destb.velocity.s_rotate (newrh-rh);
#if DEBUG_IA
        LOUT << "Robot-Ball-Interaction front\n";
#endif
      } else {
        Vec rb = newbp-newrp;

        double tau = (robot_radius+ball_radius)/(rb.length()+0.0001);
        newbp = tau*newbp+(1-tau)*newrp;
        if (bv*rb<0)
          bv = Vec::zero_vector;
#if DEBUG_IA
        LOUT << "Robot-Ball-Interaction side/back\n";
#endif
      }
    }

    rp=newrp;
    bp=newbp;
#if DEBUG_IA
    LOUT << "\% blue cross " << rp.x << ' ' << rp.y << " dark_red cross " << bp.x << ' ' << bp.y << '\n';
#endif
   }

#if DEBUG_IA
    LOUT << "\% blue circle " << rp.x << ' ' << rp.y << " 50 dark_red circle " << bp.x << ' ' << bp.y << " 50\n";
#endif
  destr.pos=rp;
  destb.pos.x=bp.x;
  destb.pos.y=bp.y;
  // original z-Koordinate beibehalten
}
