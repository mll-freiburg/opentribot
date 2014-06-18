
#include "SingleStepHeuristicVelocityPredictor.h"
#include "update_robot_location.h"
#include "../WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

SingleStepHeuristicVelocityPredictor::SingleStepHeuristicVelocityPredictor (const ConfigReader& cfg, OdometryContainer& odobox1) throw (std::bad_alloc, TribotsException) :
    max_pred_steps (10),
    max_store_steps (15),
    burn_in (max_store_steps+20),
    do_predict (true),
    odobox (odobox1),
    drv (max_pred_steps+max_store_steps),
    pred (max_store_steps+max_pred_steps)
{
  cfg.get ("VelocityPredictor::do_predict", do_predict);
  if (do_predict) {
    max_lacc = 2.0;
    max_ldec = 3.8;
    max_aacc = 20;
    max_lv = 4.0;
    max_av = 10.0;
    ignore_lv = 0.1;
    ignore_av = 0.5;
    cfg.get ("VelocityPredictor::max_lacc", max_lacc);
    cfg.get ("VelocityPredictor::max_ldec", max_ldec);
    cfg.get ("VelocityPredictor::max_aacc", max_aacc);
    cfg.get ("VelocityPredictor::max_lv", max_lv);
    cfg.get ("VelocityPredictor::max_av", max_av);
    cfg.get ("VelocityPredictor::ignore_lv", ignore_lv);
    cfg.get ("VelocityPredictor::ignore_av", ignore_av);
    max_lacc*=1e-3;
    max_ldec*=1e-3;
    max_aacc*=1e-3;
  }
}

SingleStepHeuristicVelocityPredictor::~SingleStepHeuristicVelocityPredictor () throw () {;}


void SingleStepHeuristicVelocityPredictor::notify_position (const RobotLocation&, Time) throw () {;}

void SingleStepHeuristicVelocityPredictor::notify_drive_vector (DriveVector dv, Time t) throw () {
  OdometryContainer::TimestampDriveVector tdv;
  tdv.timestamp = t;
  tdv.dv = dv;
  if (tdv.dv.vtrans.length()>max_lv)
    tdv.dv.vtrans*=max_lv/tdv.dv.vtrans.length();
  if (tdv.dv.vrot>max_av)
    tdv.dv.vrot = max_av;
  if (tdv.dv.vrot<-max_av)
    tdv.dv.vrot = -max_av;
  if (tdv.dv.vtrans.length()<ignore_lv)
    tdv.dv.vtrans = Vec::zero_vector;
  if (abs(tdv.dv.vrot)<ignore_av)
    tdv.dv.vrot=0;
  drv.get()=tdv;
  drv.step();
}

void SingleStepHeuristicVelocityPredictor::update () throw () {
  if (!do_predict)
    return;

  if (burn_in>0) {
    burn_in--;
    return;
  }
  // Sicherheitshalber die drv-Liste auffuellen
  OdometryContainer::TimestampDriveVector tdv = drv[-1];
  for (unsigned int i=0; i<max_pred_steps; i++) {
    tdv.timestamp.add_msec (33);
    drv[i]=tdv;
  }

  // den kleinsten Zeitstempel suchen, der groesser als latest_basic_timestamp ist
  unsigned int drvindex=max_pred_steps;
  while (drvindex+1<max_store_steps+max_pred_steps) {
    if (drv[drvindex].timestamp<=latest_basic_timestamp) {
      drvindex++;
    } else {
      break;
    }
  }

  DriveVector so_far_dv = latest_basic_dv;
  Time so_far_timestamp = latest_basic_timestamp;
  for (unsigned int i=0; i<max_store_steps+max_pred_steps; i++) {
    double delta_t = drv[drvindex+i].timestamp.diff_msec (so_far_timestamp);
    if (delta_t<=0) delta_t = 1e-10;
    DriveVector des_dv = drv[drvindex+i-1].dv;
    Vec delta_lin = des_dv.vtrans-so_far_dv.vtrans;
    Vec lin_norm = (so_far_dv.vtrans.squared_length()>0.0001 ? so_far_dv.vtrans : (des_dv.vtrans.squared_length()>0.0001 ? des_dv.vtrans : Vec::unit_vector_y)).normalize();
    Vec lin_ortho = lin_norm.rotate_quarter();
    double delta_vn = delta_lin*lin_norm;
    double delta_vo = delta_lin*lin_ortho;
    // Maximalbeschleunigung beruecksichtigen:
    if (delta_vn/delta_t > max_lacc)
      delta_vn = max_lacc*delta_t;
    if (delta_vn/delta_t < -max_ldec)
      delta_vn = - max_ldec*delta_t;
    if (delta_vo/delta_t > max_lacc)
      delta_vo = max_lacc*delta_t;
    if (delta_vo/delta_t < -max_lacc)
      delta_vo = -max_lacc*delta_t;
    // Das selbe fuer die Rotation:
    double delta_ang = des_dv.vrot-so_far_dv.vrot;
    if (delta_ang/delta_t > max_aacc)
      delta_ang = max_aacc*delta_t;
    if (delta_ang/delta_t < -max_aacc)
      delta_ang = -max_aacc*delta_t;
    
    so_far_dv.vtrans += delta_vn*lin_norm+delta_vo*lin_ortho;
    so_far_dv.vrot += delta_ang;
    so_far_timestamp = drv[drvindex+i].timestamp;
    so_far_dv.kick = drv[drvindex+i].dv.kick;
        
    pred[i].dv =so_far_dv;
    pred[i].timestamp=so_far_timestamp;
    drv2vel (pred[i].dv.vtrans, pred[i].dv.vrot);
    if (i==0) {
      latest_basic_dv = so_far_dv;
      latest_basic_timestamp = so_far_timestamp;
    }
  }

  if (do_predict)
    odobox.set_velocity_estimates (pred);
}

void SingleStepHeuristicVelocityPredictor::drv2vel (Vec& lin, double& ang) const throw () {
  // zuerst die Geschwindigkeit reduzieren:
  double vlen = lin.length();
  lin *= (1-0.08*vlen);
  ang *=(1-0.008*abs(ang));
  // Abhaengigkeit zwischen beiden Geschwindigkeiten berechnen
  double symfac = abs(fmod (lin.angle().get_deg(), 60.0)-30.0)/30.0;  // bei 30 +k*60 Grad symetrische Fahrt, bei k*60 Grad unsymetrische Fahrt
  double velfac = vlen*abs(ang)/30.0;  // je schneller, desto groesser
  lin*=(1.0-0.3*symfac*velfac);
  ang*=(1.0+0.3*symfac*velfac);
}

