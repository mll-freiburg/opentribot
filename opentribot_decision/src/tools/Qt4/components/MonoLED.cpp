
#include "MonoLED.h"
#include <QtGui/QPainter>

using namespace std;
using namespace TribotsTools;

MonoLED::MonoLED ( QWidget* p, Qt::WFlags f, QColor color) : QWidget (p,f), col(color), on (false) {;}

MonoLED::~MonoLED () {;}

const QColor& MonoLED::color () const throw () {
  return col;
}

bool MonoLED::isOn () const throw () {
  return on;
}

void MonoLED::setColor (QColor c) throw () {
  col=c;
}

void MonoLED::setOn (bool b) {
  on=b;
  update ();
}

void MonoLED::paintEvent(QPaintEvent *) {
  QPainter paint (this);
  int w = width();
  int h = height();
  int r = ((h>w ? w : h )-5)/2;
  QMatrix mapping (1,0,0,1,w/2, h/2);
  paint.setWorldMatrix (mapping);

  paint.setPen (QPen (Qt::black));
  if (on)
    paint.setBrush (QBrush (col, Qt::SolidPattern));
  else
    paint.setBrush (QBrush (col, Qt::Dense5Pattern));
  paint.drawChord (-r,-r,2*r,2*r,0,5760);
  paint.setPen (QPen (Qt::black));
  paint.drawArc (-r,-r,2*r,2*r,0,5760);
}
