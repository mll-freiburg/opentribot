
#include "FieldLUT.h"
#include "../../Fundamental/geometry.h"
#include <cmath>

#include <fstream>
#define TEST_FIELDLUT 0

using namespace Tribots;
using namespace std;

FieldLUT::~FieldLUT () throw () {
  delete [] array;
}

FieldLUT::FieldLUT (const FieldGeometry& fg, unsigned int c) throw (std::bad_alloc) : cell_size(c), array(NULL) {
  x_res = static_cast<unsigned int>(ceil((0.5*fg.field_width+fg.side_band_width)/static_cast<double>(cell_size)));
  y_res = static_cast<unsigned int>(ceil((0.5*fg.field_length+fg.goal_band_width)/static_cast<double>(cell_size)));
  array = new double [4*x_res*y_res];
  grad = new Tribots::Vec [4*x_res*y_res];

  error_outside = (fg.side_band_width>fg.goal_band_width ? fg.side_band_width : fg.goal_band_width);

  // alle Eintrage auf maximale Werte setzen
  double max_val = 1e100;

  double* arr_ptr=array;
  const double* end_arr_ptr=array+4*x_res*y_res;
  while (arr_ptr<end_arr_ptr) 
    *(arr_ptr++) = max_val;

  // Versuch: "Torpfosten"-Effekt modellieren
  draw_line_segment (Vec (0.5*fg.goal_width,-0.5*fg.field_length), Vec (0.5*fg.goal_width+500,-0.5*fg.field_length-500));
  draw_line_segment (Vec (0.5*fg.goal_width,0.5*fg.field_length), Vec (0.5*fg.goal_width+500,0.5*fg.field_length+500));
  draw_line_segment (Vec (-0.5*fg.goal_width,-0.5*fg.field_length), Vec (-0.5*fg.goal_width-500,-0.5*fg.field_length-500));
  draw_line_segment (Vec (-0.5*fg.goal_width,0.5*fg.field_length), Vec (-0.5*fg.goal_width-500,0.5*fg.field_length+500));

  // Feldlinien einzeichnen
  vector<LineSegment> field_lines;
  vector<Arc> field_arcs;
  fg.make_lineset (field_lines, field_arcs);
  for (std::vector<Tribots::LineSegment>::const_iterator  it = field_lines.begin(); it<field_lines.end(); it++)
    draw_line_segment (it->p1, it->p2);
  for (std::vector<Tribots::Arc>::const_iterator  it = field_arcs.begin(); it<field_arcs.end(); it++)
    draw_arc (it->center, it->radius, it->start, it->end);

  // Gradient approximieren mit Sobelfilter
  // zunaechst den Rand ignorieren und das Innere berechnen
  for (unsigned int xi = 1; xi<2*x_res-1; xi++)
    for (unsigned int yi=1; yi<2*y_res-1; yi++) {  // Sobelfilter anwenden
      double lo = array[xi+2*x_res*yi-1+2*x_res];
      double lm = array[xi+2*x_res*yi-1];
      double lu = array[xi+2*x_res*yi-1-2*x_res];
      double ro = array[xi+2*x_res*yi+1+2*x_res];
      double rm = array[xi+2*x_res*yi+1];
      double ru = array[xi+2*x_res*yi+1-2*x_res];
      double mo = array[xi+2*x_res*yi+2*x_res];
      double mu = array[xi+2*x_res*yi-2*x_res];
      grad[xi+2*x_res*yi].x = 0.125*(-lo+ro-2*lm+2*rm-lu+ru)/static_cast<double>(cell_size);
      grad[xi+2*x_res*yi].y = 0.125*(lo-lu+2*mo-2*mu+ro-ru)/static_cast<double>(cell_size);
    }
  // am Rand einfach die benachbarten Gradienten uebernehmen
  for (unsigned int xi = 1; xi<2*x_res-1; xi++) {
    grad[xi] = grad[xi+2*x_res];
    grad[xi+2*x_res*(2*y_res-1)] = grad[xi+2*x_res*(2*y_res-2)];
  }
  for (unsigned int yi= 1; yi<2*y_res-1; yi++) {
    grad[2*x_res*yi] = grad[2*x_res*yi+1];
    grad[2*x_res*yi+2*x_res-1] = grad[2*x_res*yi+2*x_res-2];
  }
  // Ecken durch Mittelung benachbarter Randpunkte
  grad[0]=0.5*(grad[1]+grad[2*x_res]);
  grad[2*x_res-1]=0.5*(grad[2*x_res-2]+grad[4*x_res-1]);
  grad[2*x_res*(y_res-1)]=0.5*(grad[2*x_res*(y_res-2)]+grad[2*x_res*(y_res-1)+1]);
  grad[4*x_res*y_res-1]=0.5*(grad[4*x_res*y_res-2]+grad[4*x_res*y_res-2*x_res-1]);

#if TEST_FIELDLUT
  // nur zu Testzwecken, grafische Ausgabe des Abstandsarrays als PGM-Frame
  {
    ofstream foo ("fieldlut_f.pgm");
    foo << "P5\n" << 2*y_res << ' ' << 2*x_res << " 255\n";
    for (unsigned int xi = 0; xi<2*x_res; xi++)
      for (unsigned int yi = 0; yi<2*y_res; yi++)
	foo.put(static_cast<unsigned int>(255-array[xi+2*x_res*(2*y_res-yi-1)]/10));
  }
  {
    ofstream foo ("fieldlut_gx.pgm");
    foo << "P5\n" << 2*y_res << ' ' << 2*x_res << " 255\n";
    for (unsigned int xi = 0; xi<2*x_res; xi++)
      for (unsigned int yi = 0; yi<2*y_res; yi++)
	foo.put(static_cast<unsigned int>(127+127*grad[xi+2*x_res*(2*y_res-yi-1)].x));
  }
  {
    ofstream foo ("fieldlut_gy.pgm");
    foo << "P5\n" << 2*y_res << ' ' << 2*x_res << " 255\n";
    for (unsigned int xi = 0; xi<2*x_res; xi++)
      for (unsigned int yi = 0; yi<2*y_res; yi++)
	foo.put(static_cast<unsigned int>(127+127*grad[xi+2*x_res*(2*y_res-yi-1)].y));
  }
#endif
}

void FieldLUT::update (unsigned int xi, unsigned int yi, double v) {
  if (xi>=2*x_res || yi>=2*y_res)
    return;
  if (array[xi+2*x_res*yi]>v)
    array[xi+2*x_res*yi]=v;
}

double Tribots::FieldLUT::distance (const Tribots::Vec& p) const throw () {
  int xind = static_cast<int>(floor (p.x/cell_size)+x_res);
  int yind = static_cast<int>(floor (p.y/cell_size)+y_res);
  if ((xind<0) || (yind<0) || (xind>=2*static_cast<int>(x_res)) || (yind>=2*static_cast<int>(y_res)))
    return error_outside;   // default-Wert fuer Werte ausserhalb des Feldes; sollte eigentlich nicht vorkommen
  return array[static_cast<unsigned int>(xind+2*x_res*yind)];
}

Tribots::Vec Tribots::FieldLUT::gradient (const Vec& p) const throw () {
  int xind = static_cast<int>(floor (p.x/cell_size)+x_res);
  int yind = static_cast<int>(floor (p.y/cell_size)+y_res);
  if (xind<0)  // am Rand abschneiden
    xind=0;
  if (xind>=2*static_cast<int>(x_res))
    xind=2*static_cast<int>(x_res)-1;
  if (yind<0)
    yind=0;
  if (yind>=2*static_cast<int>(y_res))
    yind=2*static_cast<int>(y_res)-1;
  return grad[static_cast<unsigned int>(xind+2*x_res*yind)];  
}

void FieldLUT::draw_line_segment (Vec start, Vec end) {
  LineSegment line (start, end);
  for (unsigned int xi=0; xi<2*x_res; xi++)
    for (unsigned int yi=0; yi<2*y_res; yi++) {
      Vec pp ((static_cast<double>(xi)-static_cast<double>(x_res)+0.5)*static_cast<double>(cell_size), (static_cast<double>(yi)-static_cast<double>(y_res)+0.5)*static_cast<double>(cell_size));
      update (xi,yi,line.distance (pp));
    }
}

void FieldLUT::draw_arc (Vec center, double radius, Angle start, Angle end) {
  Arc arc (center, radius, start, end);
  for (unsigned int xi=0; xi<2*x_res; xi++)
    for (unsigned int yi=0; yi<2*y_res; yi++) {
      Vec pp ((static_cast<double>(xi)-static_cast<double>(x_res)+0.5)*static_cast<double>(cell_size), (static_cast<double>(yi)-static_cast<double>(y_res)+0.5)*static_cast<double>(cell_size));
      update (xi,yi,arc.distance (pp));
    }
}

void FieldLUT::draw_dot (Vec p) {
  for (unsigned int xi=0; xi<2*x_res; xi++)
    for (unsigned int yi=0; yi<2*y_res; yi++) {
      Vec pp ((static_cast<double>(xi)-static_cast<double>(x_res)+0.5)*static_cast<double>(cell_size), (static_cast<double>(yi)-static_cast<double>(y_res)+0.5)*static_cast<double>(cell_size));
      update (xi,yi,(p-pp).length());
    }
}
