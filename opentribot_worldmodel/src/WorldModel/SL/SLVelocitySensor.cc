
#include "SLVelocitySensor.h"
#include <cmath>

#define ZWEIPI 6.283185307179586232

using namespace Tribots;
using namespace std;

SLVelocitySensor::SLVelocitySensor (unsigned int n1) throw (std::bad_alloc) : n(n1), burn_in(n1), buffer(n1) {
  model.pos = Vec::zero_vector;
  model.heading = Angle::zero;
  model.vtrans = Vec::zero_vector;
  model.vrot = 0;
  model.kick = false;
  vmax = 5.0;
  vamax = 12.0;
}

void SLVelocitySensor::set_max_velocity (double a1, double a2) throw () {
  vmax=a1;
  vamax=a2;
}

const RobotLocation& SLVelocitySensor::get (Time& t) const throw () {
  t=timestamp;
  return model;
}

void SLVelocitySensor::update (const RobotLocation& rloc, Time t_ref) throw () {
  // neue Beobachtung in Puffer einfuegen
  TPH new_tph;
  new_tph.timestamp = t_ref;
  new_tph.pos=rloc.pos;
  new_tph.heading=rloc.heading.get_rad();
  buffer[0]=new_tph;
  buffer.step ();

  if (burn_in>0)
    burn_in--;

  if (burn_in==0) {
    // Modell neu berechnen, andernfalls Initialisierungphase, in der keine Modelle berechnet werden

    // 1. Schritt: Rotationsgeschwindigkeit und Anfangsorientierung berechnen
    double num = static_cast<double>(n);
    double sum_t=0;        // sum_i t_i
    double sum_tt=0;       // sum_i (t_i)^2
    double sum_phi=0;      // sum_i phi_i
    double sum_phit=0;     // sum_i (phi_i t_i)
    double latest_phi = buffer.get().heading;
    for (unsigned int i=0; i<n; i++) {
      double t = static_cast<double>(buffer.get().timestamp.diff_msec(t_ref));
      double phi = buffer.get().heading;
      while (phi>=latest_phi+M_PI)
        phi-=ZWEIPI;
      while (phi<latest_phi-M_PI)
        phi+=ZWEIPI;
      latest_phi=phi;
      sum_t+=t;
      sum_tt+=t*t;
      sum_phi+=phi;
      sum_phit+=t*phi;
      buffer.step();
    }
    double det = num*sum_tt-sum_t*sum_t;
    if (det==0) {
      model.heading.set_rad (sum_phi/num);
      model.vrot = 0;
    } else {
      model.heading.set_rad ((sum_tt*sum_phi-sum_t*sum_phit)/det);
      model.vrot = ((-sum_t*sum_phi+num*sum_phit)/det);
    }

    if (abs(model.vrot)<=0.1e-3) {
      // Fall 2.1: vrot~=~0; berechne Anfangsposition und Geschwindigkeiten aus geradliniger Bewegung
      double sum_x=0;   // sum_i x_i
      double sum_y=0;   // sum_i y_i
      double sum_xt=0;  // sum_i (x_i*t_i)
      double sum_yt=0;  // sum_i (y_i*t_i)
      for (unsigned int i=0; i<n; i++) {
        double t = static_cast<double>(buffer.get().timestamp.diff_msec(t_ref));
        double x = buffer.get().pos.x;
        double y = buffer.get().pos.y;
        sum_x+=x;
        sum_y+=y;
        sum_xt+=t*x;
        sum_yt+=t*y;
        buffer.step();
      }
      if (det==0) {
        model.pos.x = sum_x/num;
        model.pos.y = sum_y/num;
        model.vtrans = Vec::zero_vector;
      } else {
        model.pos.x = (sum_tt*sum_x-sum_t*sum_xt)/det;
        model.pos.y = (sum_tt*sum_y-sum_t*sum_yt)/det;
        model.vtrans = (Vec ( (-sum_t*sum_x+num*sum_xt)/det,  (-sum_t*sum_y+num*sum_yt)/det));
      }
    } else {
      // Fall 2.2: vrot!=0; berechne x0, y0, vx, vy aus Fahrt auf einer Kreisbahn
      double sum_sin=0;   // sum_i sin(model.vrot t_i)
      double sum_cos=0;   // sum_i (cos(model.vrot t_i)-1)
      double sum_sin2=0;  // sum_i (sin(model.vrot t_i))^2
      double sum_cos2=0;  // sum_i (cos(model.vrot t_i)-1)^2
      double sum_x=0;   // sum_i x_i
      double sum_y=0;   // sum_i y_i
      double sum_sinx=0;  // sum_i (sin(model.vrot t_i) x_i)
      double sum_cosx=0;  // sum_i ((cos(model.vrot t_i)-1) x_i)
      double sum_siny=0;  // sum_i (sin(model.vrot t_i) y_i)
      double sum_cosy=0;  // sum_i ((cos(model.vrot t_i)-1) y_i)
      for (unsigned int i=0; i<n; i++) {
        double t = static_cast<double>(buffer.get().timestamp.diff_msec(t_ref));
        double x = buffer.get().pos.x;
        double y = buffer.get().pos.y;
        double s = sin(model.vrot*t);
        double c = cos(model.vrot*t)-1;
        sum_sin+=s;
        sum_cos+=c;
        sum_sin2+=s*s;
        sum_cos2+=c*c;
        sum_x+=x;
        sum_y+=y;
        sum_sinx+=s*x;
        sum_cosx+=c*x;
        sum_siny+=s*y;
        sum_cosy+=c*y;
        buffer.step();
      }
      sum_sin/=model.vrot;
      sum_cos/=model.vrot;
      sum_sin2/=(model.vrot*model.vrot);
      sum_cos2/=(model.vrot*model.vrot);
      sum_sinx/=model.vrot;
      sum_cosx/=model.vrot;
      sum_siny/=model.vrot;
      sum_cosy/=model.vrot;
      double denom = num*(sum_sin2+sum_cos2)-sum_sin*sum_sin-sum_cos*sum_cos;
      if (denom==0) {
        model.pos.x = sum_x/num;
        model.pos.y = sum_y/num;
        model.vtrans = Vec::zero_vector;
      } else {
        model.pos.x = ((sum_sin2+sum_cos2)*sum_x-sum_sin*(sum_sinx-sum_cosy)-sum_cos*(sum_cosx+sum_siny))/denom;
        model.pos.y = ((sum_sin2+sum_cos2)*sum_y+sum_cos*(sum_sinx-sum_cosy)-sum_sin*(sum_cosx+sum_siny))/denom;
        model.vtrans = (Vec ((-sum_sin*sum_x+sum_cos*sum_y+num*(sum_sinx-sum_cosy))/denom, (-sum_cos*sum_x-sum_sin*sum_y+num*(sum_cosx+sum_siny))/denom));
      }
    }
    if (model.vtrans.length()>vmax || abs(model.vrot)>vamax) {
      // Korrektur bei unmoeglichen Geschwindigkeitsschaetzungen
      model.vtrans = Vec::zero_vector;
      model.vrot = 0;
      for (unsigned int i=0; i+1<buffer.size(); i++) {
        buffer[i].pos = buffer[-1].pos;
        buffer[i].heading = buffer[-1].heading;
      }
    }

    timestamp = t_ref;
    model.vrot*=1e3;   // umrechnen von rad/ms in rad/s
  }

}
  
