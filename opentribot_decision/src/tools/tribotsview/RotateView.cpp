
#include "RotateView.h"
#include <qpainter.h>
#include <cmath>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

RotateView::RotateView ( QWidget*  p, const char*  c, WFlags  f ) : QWidget (p,c,f), fop (NULL) {;}

RotateView::~RotateView () {;}

void RotateView::init (const FieldOfPlay* p) {
  fop = p;
  connect (fop, SIGNAL(slDisplacement()), this, SLOT(update()));
  connect (fop, SIGNAL(preferencesChanged()), this, SLOT(update()));
  connect (fop, SIGNAL(cycleChanged()), this, SLOT(update()));
}

void RotateView::paintEvent(QPaintEvent *) {
  const CycleInfo& ci (fop->get_cycle_info ());
  const PaintPreferences& pp (fop->get_preferences ());
  const vector<RobotLocation>& rloc (pp.use_exec_time ? ci.rloc_exec : ci.rloc_vis);
  const vector<BallLocation>& bloc (pp.use_exec_time ? ci.bloc_exec : ci.bloc_vis);

  if (rloc.size()>pp.reference_robot) {
    double add_dir = (pp.zoom.own_half>0 ? 0.0 : M_PI);
    double angle = add_dir+rloc[pp.reference_robot].heading.get_rad();
    double vrot = rloc[pp.reference_robot].vrot;
    
    QPainter paint (this);
    int w = width();
    int h = height();
    int r = ((h>w ? w : h )-5)/2;
    QWMatrix mapping (1,0,0,-1,w/2, h/2);
    paint.setWorldMatrix (mapping);
    
    paint.setPen (QPen (Qt::black,2));
    paint.drawArc (-r,-r,2*r,2*r,0,5760);
    paint.drawArc (-r,-r,2*r,2*r,5000,800);
  
    paint.setPen (NoPen);
    paint.setBrush (QBrush (Qt::darkCyan, Qt::Dense4Pattern));
    paint.drawPie (-r,-r,2*r,2*r,-static_cast<int>((angle*16*180)/M_PI), -static_cast<int>((vrot*16*180)/M_PI/2));
    if (bloc.size()>0) {
      double ball_angle = add_dir-M_PI/2+(bloc[0].pos-rloc[0].pos).angle().get_rad();
      paint.setPen (QPen (Qt::red,2));
      paint.drawLine (0,0,static_cast<int>(0.7*r*cos(ball_angle)), static_cast<int>(0.7*r*sin(ball_angle)));
    }
    paint.setPen (QPen (Qt::darkCyan,2));
    paint.drawLine (0,0,static_cast<int>(r*cos(angle)), static_cast<int>(r*sin(angle)));
  }
}

