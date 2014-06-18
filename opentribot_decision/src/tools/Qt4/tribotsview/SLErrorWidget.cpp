
#include "SLErrorWidget.h"
#include <QtGui/QPainter>
#include <cmath>
#include "../../../WorldModel/Orga/VisualContainer.h"

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

#define CFILTER 0

namespace {
  double round (double& x){
    return floor(x+0.5);
  }
}

const double SLErrorWidget::cellsize = 50;

SLErrorWidget::SLErrorWidget ( QWidget * parent, Qt::WFlags f ) : QWidget (parent, f) {
  error_array=NULL;
  width=height=0;
  ci=NULL;
  fl=NULL;
  fg=NULL;
  optimiser=NULL;
  quit_button= new QPushButton (this);
  quit_button->setText("Quit");
  revert_button= new QPushButton (this);
  revert_button->setText("Revert all");
  revert_opt_button = new QPushButton (this);
  revert_opt_button->setText("Revert pos");

  connect (quit_button, SIGNAL (clicked()), this, SLOT(hide()));
  connect (revert_button, SIGNAL (clicked()), this, SLOT(update_error()));
  connect (revert_opt_button, SIGNAL (clicked()), this, SLOT(update_optimising_pos ()));
  setMinimumSize (250,200);
}

SLErrorWidget::~SLErrorWidget () {
  if (error_array) {
    for (unsigned int i=0; i<width; i++)
      delete [] error_array [i];
    delete [] error_array;
  }
  if (fl)
    delete fl;
  if (fg)
    delete fg;
  if (optimiser)
    delete optimiser;
  delete quit_button;
  delete revert_button;
  delete revert_opt_button;
}

void SLErrorWidget::init_pointers (const ConfigReader& reader, const FieldGeometry& fg2) {
  vector<double> dval (2);
#if CFILTER
  if (reader.get ("CondensationFilter::sensor_probabilities", dval)>=2) {
    probability_line_sdev = abs(dval[0]);
    probability_line_min = abs(dval[1]);
  } else {
    probability_line_sdev = 500;
    probability_line_min = 0.2;
  }
#else
   if (!reader.get ("ErrorMinimiserSL::error_width", probability_line_sdev)>=1) {
     probability_line_sdev = 500;
   } 
#endif
  if (!reader.get ("ErrorMinimiserSL::error_width", optimiser_error_param))
    optimiser_error_param=250;
  if (!reader.get ("ErrorMinimiserSL::num_iterations", num_opt_iter))
    num_opt_iter=20;
  fg = new Tribots::FieldGeometry (fg2);
}

void SLErrorWidget::update_error (CycleInfo& c, Tribots::Vec e1, Tribots::Vec e2, int ow) {
  if (!fl) {
    cout << "Erzeuge Field-LUT ...\n";
    fl = new FieldLUT (*fg, static_cast<unsigned int>(cellsize));
  }
  if (!optimiser)
    optimiser=new VisualPositionOptimiser (*fl, optimiser_error_param, 1e50);

  ci = &c;
  own_half=ow;
  center = 0.5*(e1+e2);
  xext = abs (e1.x-e2.x);
  yext = abs (e1.y-e2.y);

  if (error_array) {
    for (unsigned int i=0; i<width; i++)
      delete [] error_array [i];
    delete [] error_array;
  }

  width = static_cast<unsigned int>(ceil (yext/cellsize)+1);
  height = static_cast<unsigned int>(ceil (xext/cellsize)+1);
  error_array = new unsigned char* [width];
  for (unsigned int i=0; i<width; i++)
    error_array[i]=new unsigned char [height];

  update_error ();
}

void SLErrorWidget::resizeEvent ( QResizeEvent * ev ) {
  unsigned int w = ev->size().width();
  unsigned int h = ev->size().height();
  revert_button->setGeometry (0,h-30,(w-50)/2,30);
  revert_opt_button->setGeometry ((w-50)/2,h-30,w-50-(w-50)/2,30);
  quit_button->setGeometry (w-50,h-30,50,30);
}

void SLErrorWidget::paintEvent(QPaintEvent *) {
  QPainter paint (this);
  double ww = QWidget::width();
  double wh = QWidget::height()-30;
  double scale_x = ww/static_cast<double>(width);
  double scale_y = wh/static_cast<double>(height);
  scale = (scale_x<scale_y ? scale_x : scale_y);
  offset_x = 0.5*(ww-scale*width);
  offset_y = 0.5*(wh-scale*height);
  if (own_half>0)
    paint.setWorldMatrix (QMatrix (scale, 0, 0, scale, offset_x, offset_y));
  else
    paint.setWorldMatrix (QMatrix (-scale, 0, 0, -scale, ww-offset_x, wh-offset_y));


  for (unsigned int i=0; i<width; i++)
    for (unsigned int j=0; j<height; j++) {
      unsigned int g=error_array[i][j];
      paint.setPen (QColor (g,g,g));
      paint.fillRect (i,j,2,2, QBrush (QColor (g,g,g), Qt::SolidPattern));
    }
  paint.setPen (QPen(Qt::blue,2));
  paint.drawArc (best_i-1, best_j-1, 3, 3, 0, 5760);  // beste Position einzeichnen
  if (ci) {
    paint.setPen (QPen(Qt::cyan,2));
    if (ci->rloc_vis.size()>0) {
      Tribots::Vec cur = field2widget (ci->rloc_vis[0].pos);
      paint.drawArc (static_cast<int>(cur.x)-1, static_cast<int>(cur.y)-1, 3, 3, 0, 5760);  // aktuelle Position zeichnen

      paint.setPen (QPen(Qt::magenta,2));
      cur = field2widget (optimising_pos);
      paint.drawArc (static_cast<int>(cur.x)-1, static_cast<int>(cur.y)-1, 3, 3, 0, 5760);  // optimierte Position zeichnen
    }
  }
}

void SLErrorWidget::mousePressEvent(QMouseEvent* ev) {
  Tribots::Vec x = widget2field (1.0/scale*Tribots::Vec(ev->x()-offset_x, ev->y()-offset_y));
  double bp;
  calculate_fitness (bp, x);
  if (ci && ci->rloc_vis.size()>0) {
    ci->rloc_vis[0].pos = x;
    ci->rloc_vis[0].heading.set_deg (bp);
  }
  emit (robot_update());
}

Tribots::Vec SLErrorWidget::widget2field (Tribots::Vec w) {
  return center+own_half*cellsize*(Tribots::Vec(w.y-height/2,w.x-width/2));
}

Tribots::Vec SLErrorWidget::field2widget (Tribots::Vec x) {
  return Tribots::Vec(width/2, height/2)+own_half/cellsize*Tribots::Vec(x.y-center.y, x.x-center.x);
}

double SLErrorWidget::calculate_fitness (double& bp, Tribots::Vec x) {
  VisualContainer visbox;
  if (ci->vloc.size()>0)
    visbox.add (ci->vloc[0]);
  double m2=-1;
  bp=0; // bestes phi fuer diese Position
  for (double phi=0; phi<360; phi+=5) {
#if CFILTER
    double r=1;
#else
    double r=0;
#endif
    vector<VisibleObject>::const_iterator vit = visbox.get_lines().objectlist.begin();
    vector<VisibleObject>::const_iterator vit_end = visbox.get_lines().objectlist.end();
    unsigned int n=0;
    while (vit<vit_end) {
      if (vit->object_type==VisibleObject::white_line) {
        double d = fl->distance (x+vit->pos.rotate (Angle::deg_angle(phi)));
#if CFILTER
        double p = (d>=probability_line_sdev ? probability_line_min : 1.0-d/probability_line_sdev+probability_line_min);
        r *= p;
#else
        double c2=probability_line_sdev*probability_line_sdev;
        double ef = c2+d*d;
        r+=1-c2/ef;   // Fehleranteil berechnen
#endif
        n++;
      }
      vit++;
    }
#if CFILTER
    double value = static_cast<unsigned int>(255*sqrt(pow (r, 1.0/static_cast<double>(n))-probability_line_min));
#else
    double value = static_cast<unsigned int>(255-255*r/n);
#endif
    if (value>m2) {
      m2=value;
      bp=phi;
    }
  }
  return m2;
}

void SLErrorWidget::update_error () {
  cout << "Berechne Fehlerlandschaft...\n";
  if (ci) {
    // Annahme: error_array wurde bereits vorher in der richtigen Groese erzeugt, d.h.
    // width, height, center, xext, yext sind bereits korrekt
    // hier muss berechnet werden: error_array[.][.], best_i, best_j, best_phi
    double m=-1;   // das aktuelle Minimum

    for (unsigned int i=0; i<width; i++)
      for (unsigned int j=0; j<height; j++) {
        Tribots::Vec x = widget2field (Tribots::Vec (i,j));
        double bp;
        double fit = calculate_fitness (bp, x);
        error_array[i][j]=static_cast<unsigned char>(fit);
        if (fit>m) {
          m=fit;
          best_phi = static_cast<unsigned int>(bp);
          best_i=i;
          best_j=j;
        }
      }
  }
  update_optimising_pos();
  update ();
}

void SLErrorWidget::update_optimising_pos () {
  if (ci->rloc_vis.size()>0) {
    cout << "Optimiere Position: ";
    optimising_pos.x=ci->rloc_vis[0].pos.x;
    optimising_pos.y=ci->rloc_vis[0].pos.y;
    optimising_angle=ci->rloc_vis[0].heading;
    VisualContainer visbox;
    if (ci->vloc.size()>0)
      visbox.add (ci->vloc[0]);

    optimiser->calculate_distance_weights (visbox.get_lines());
    optimiser->optimise (optimising_pos, optimising_angle, visbox.get_lines(), num_opt_iter);
    double err, hphi;
    Vec hxy;
    err = optimiser->analyse (hxy, hphi, optimising_pos, optimising_angle, visbox.get_lines());
    cout << err << '\t' << hxy.x << '\t' << hxy.y << '\t' << hphi << '\n';
    update();
  }
}
