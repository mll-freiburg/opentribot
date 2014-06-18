
#include "VisualPositionOptimiser.h"
#include "../WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;
 
#define DEBUG_VISUALOPTIMISER 0

VisualPositionOptimiser::VisualPositionOptimiser (const FieldLUT& fl, double c1, double d1) throw () : the_field_lut (fl), c(c1), c2(c1*c1), d2(d1*d1), weights (300) {;}

double VisualPositionOptimiser::calculate_distance_weights (const VisibleObjectList& vis, unsigned  int max_lines) throw () {
  unsigned int nlines=(max_lines>vis.objectlist.size() ? vis.objectlist.size() : max_lines);
  double ret=0;
  weights.resize (nlines);
  double reference = 1500*1500;
  vector<VisibleObject>::const_iterator vis_it = vis.objectlist.begin();
  vector<double>::iterator weight_it = weights.begin();
  for (unsigned int i=0; i<nlines; i++) {
    ret += ((*weight_it) = (reference+d2)/(d2+vis_it->pos.squared_length()));
    weight_it++;
    vis_it++;
  }
  return ret;
}

void VisualPositionOptimiser::error (double& err, double& dx, double& dy, double& dphi, double x, double y, double phi, const VisibleObjectList& vis, unsigned int max_lines) const throw () {
  unsigned int nlines=(max_lines>vis.objectlist.size() ? vis.objectlist.size() : max_lines);
  double sinphi = sin (phi);
  double cosphi = cos(phi);

  err=dx=dy=dphi=0;
  vector<VisibleObject>::const_iterator vis_it = vis.objectlist.begin();
  vector<VisibleObject>::const_iterator vis_itend = vis.objectlist.begin()+nlines;
  vector<double>::const_iterator weight_it = weights.begin();
  while (vis_it < vis_itend) {
    Vec vp (x+cosphi*vis_it->pos.x-sinphi*vis_it->pos.y, y+sinphi*vis_it->pos.x+cosphi*vis_it->pos.y);   // gesehene weise Linie in Weltkoordinaten
    double dist = the_field_lut.distance (vp);   // Abstand gesehene Linie<-> naechste Modelllinie
    double ef = c2+dist*dist;
    err+=(*weight_it)*(1-c2/ef);   // Fehleranteil berechnen
    double derrddist = (2*c2*dist)/(ef*ef);   // Ableitung der Fehlerfunktion nach der Distanz
    Tribots::Vec ddistdpos = the_field_lut.gradient (vp);   // Ableitung der Distanzfunktion nach der Position
    dx+=(*weight_it)*derrddist*ddistdpos.x;   // Gradient: x-Anteil
    dy+=(*weight_it)*derrddist*ddistdpos.y;   // Gradient: y-Anteil
    dphi+=(*weight_it)*derrddist*(ddistdpos.x*(-sinphi*vis_it->pos.x-cosphi*vis_it->pos.y)+ddistdpos.y*(cosphi*vis_it->pos.x-sinphi*vis_it->pos.y));   // Gradient: phi-Anteil
    vis_it++;
    weight_it++;
  }
}

double VisualPositionOptimiser::optimise (Vec& xy, Angle& h, const VisibleObjectList& vis, unsigned int niter, unsigned int max_lines) const throw () {
  double param [3];  // zu optimierende Parameter
  param[0]=xy.x;
  param[1]=xy.y;
  param[2]=h.get_rad();
  double grad [3];  // Gradient
  double stepwidth [3] = {40,40,0.1};  // Schrittweiten
  double latest_grad [3] = {0,0,0};  // letzter Gradient
  double err;  // error value

  for (unsigned int i=0; i<niter; i++) {
    error (err, grad[0], grad[1], grad[2], param[0], param[1], param[2], vis, max_lines);

//    double delta_angle = (h-Angle::rad_angle(param[2])).get_rad_pi();

#if DEBUG_VISUALOPTIMISER
    WorldModel::get_main_world_model().log_stream() << "VisualOptimiser: " << param[0] << ' ' << param[1] << ' ' << param[2]*180/M_PI << ' ' << grad[0] << ' ' << grad[1] << ' ' << grad[2] << '\n';
#endif

    for (unsigned int j=0; j<3; j++) {
      // update fuer jeden Parameter vornehmen
      if (grad[j]==0)
	latest_grad[j]=0;
      else {
	// Schrittweitenanpassung
	if (grad[j]*latest_grad[j]>0)
	  stepwidth[j]*=1.2;
	else if (grad[j]*latest_grad[j]<0)
	  stepwidth[j]*=0.5;
	latest_grad[j]=grad[j];

	// Anpassung der Parameter
	if (grad[j]>0)
	  param[j]-=stepwidth[j];
	else if (grad[j]<0)
	  param[j]+=stepwidth[j];
      }
    }
  }
  xy.x=param[0];
  xy.y=param[1];
  h.set_rad(param[2]);
  return err;
}

double VisualPositionOptimiser::analyse (Vec& hxy, double& hphi, Vec xy, Angle h, const VisibleObjectList& vis, unsigned int max_lines) const throw () {
  unsigned int nlines=(max_lines>vis.objectlist.size() ? vis.objectlist.size() : max_lines);
  double phi = h.get_rad();
  double sinphi = sin (phi);
  double cosphi = cos(phi);
  
  double err=0;  // Fehler
  hxy.x=hxy.y=hphi=0;
  double derr;  // Fehler 1. Ableitung
  double dderr;  // Fehler 2. Ableitung
  double dist;  // Abstand
  Tribots::Vec ddist;  // Abstand 1. Ableitung
  // 2. Ableitung der Distanzfunktion nach der Position wird als konstant 0 angenommen
  Tribots::Vec pos;
  Tribots::Vec dposdphi;
  Tribots::Vec ddposdphi2;
  
  double t1, t2;  // Hilfgroessen

  vector<VisibleObject>::const_iterator vis_it = vis.objectlist.begin();
  vector<VisibleObject>::const_iterator vis_itend = vis.objectlist.begin()+nlines;
  vector<double>::const_iterator weight_it = weights.begin();
  while (vis_it < vis_itend) {
    pos.x = xy.x+cosphi*vis_it->pos.x-sinphi*vis_it->pos.y;
    pos.y =  xy.y+sinphi*vis_it->pos.x+cosphi*vis_it->pos.y;   // gesehene weise Linie in Weltkoordinaten
    dist = the_field_lut.distance (pos);   // Abstand gesehene Linie<-> naechste Modelllinie
    err+=(*weight_it)*(1-c2/(c2+dist*dist));
    if (dist<2*c) {  // Heuristik, um nachfolgende Mogelei etwas auszugleichen; Annahm: alle weit entfernten Punkte sind fehlerhaft
      derr=dist/c2;  // gemogelt: hier kommt die quadratische Fehlerfunktion ins Spiel, da 'err' selbst nicht positiv definit
      dderr=1/c2;  // dito
      ddist = the_field_lut.gradient (pos);   // Ableitung der Distanzfunktion nach der Position
      dposdphi.x = -sinphi*vis_it->pos.x-cosphi*vis_it->pos.y;
      dposdphi.y = cosphi*vis_it->pos.x-sinphi*vis_it->pos.y;
      ddposdphi2.x = -cosphi*vis_it->pos.x+sinphi*vis_it->pos.y;
      ddposdphi2.y = -sinphi*vis_it->pos.x-cosphi*vis_it->pos.y;
      
      hxy.x += (*weight_it)*dderr*ddist.x*ddist.x;
      hxy.y += (*weight_it)*dderr*ddist.y*ddist.y;
      t1 = ddist.x*dposdphi.x+ddist.y*dposdphi.y;
      t2 = ddist.x*ddposdphi2.x+ddist.y*ddposdphi2.y;
      hphi += (*weight_it)*(dderr*t1*t1+derr*t2);
    }
    vis_it++;
    weight_it++;
  }
  return err;
}
 
