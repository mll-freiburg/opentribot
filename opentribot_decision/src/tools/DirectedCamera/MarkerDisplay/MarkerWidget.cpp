
#include "MarkerWidget.h"
#include <qpainter.h>
#include <iostream>

using namespace std;
using namespace TribotsTools;

MarkerWidget::MarkerWidget( QWidget* parent, const char* name, WFlags fl )
    : QWidget( parent, name, fl )
{
  x=y=0;
  mode=blackScreen;
  setPaletteBackgroundColor (Qt::black);
}

MarkerWidget::~MarkerWidget()
{
}

void MarkerWidget::paintEvent( QPaintEvent * )
{
  int smallCircle = height()/10;
  QPainter painter (this);
  painter.setBrush (QBrush(Qt::black));
  painter.setPen (Qt::black);
  latestMode=mode;
  latestX=x;
  latestY=y;
  switch (mode) {
    case blackScreen:
      painter.setBrush (QBrush(Qt::white));
      painter.setPen (Qt::white);
      painter.drawRect (0,0,width(), height());
      painter.setBrush (Qt::NoBrush);
      painter.setPen (Qt::red);
      painter.drawRect (20,20,width()-40,height()-40);
      break;
    case redScreen:
      painter.setBrush (QBrush(Qt::red));
      painter.setPen (Qt::red);
      painter.drawRect (0,0,width(), height());
      break;
    case greenScreen:
      painter.setBrush (QBrush(Qt::green));
      painter.setPen (Qt::green);
      painter.drawRect (0,0,width(), height());
      break;
    case blueScreen:
      painter.setBrush (QBrush(Qt::blue));
      painter.setPen (Qt::blue);
      painter.drawRect (0,0,width(), height());
      break;
    case redMarker:
      painter.setBrush (QBrush(Qt::red));
      painter.setPen (Qt::red);
      painter.drawChord(x-smallCircle, y-smallCircle, 2*smallCircle, 2*smallCircle, 0, 5760);
      break;
    case calibrationMarker:
      painter.setBrush (QBrush(Qt::white));
      painter.setPen (Qt::white);
      painter.drawRect (0,0,x,y);
      painter.drawRect (x+1, y+1, width()-x, height()-y);
      break;
    default:
      painter.setBrush (QBrush(Qt::white));
      painter.setPen (Qt::white);
      painter.drawLine (0,0,width()-1, height()-1);
  }
}

void MarkerWidget::setPosition (unsigned int x1, unsigned int y1) {
  x=x1;
  y=y1;
}

void MarkerWidget::setMode (Mode m) {
  mode=m;
  repaint();
}

void MarkerWidget::switchmode () {
  if (mode>6)
    close();
  setMode (Mode(mode+1));
}

void MarkerWidget::getModePosition (Mode& m1, unsigned int& x1, unsigned int& y1) {
  m1=latestMode;
  x1=latestX;
  y1=latestY;
}
