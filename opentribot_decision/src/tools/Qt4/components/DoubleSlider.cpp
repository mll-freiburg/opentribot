
#include "DoubleSlider.h"
#include <QtGui/QPainter>
#include <QtGui/QApplication>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

DoubleSlider::DoubleSlider ( QWidget*  p, Qt::WFlags  f ) : QWidget (p,f) {
  valMin=0;
  valMax=100;
  val1=val2=0;
  stepWidth=10;
  triangleHeight=11;
  triangleWidth=8;
  rangeLightningMode=0;
  doDrawNumbers=true;
  isHorizontal=true;
}

DoubleSlider::~DoubleSlider () {;}

void DoubleSlider::setMinValue (int v) {
  valMin=v;
  if (val1<v) val1=v;
  if (val2<v) val2=v;
  update();
}
void DoubleSlider::setMaxValue (int v) {
  valMax=v;
  if (val1>v) val1=v;
  if (val2>v) val2=v;
  update();
}
void DoubleSlider::setVal1 (int v) {
  if (v<valMin) val1=valMin;
  else if (v>valMax) val1=valMax;
  else val1=v;
  emit (valueChangedExplicitely (val1, val2));
  update();
}
void DoubleSlider::setVal2 (int v) {
  if (v<valMin) val2=valMin;
  else if (v>valMax) val2=valMax;
  else val2=v;
  emit (valueChangedExplicitely (val1, val2));
  update();
}
int DoubleSlider::getVal1 () const { return val1; }
int DoubleSlider::getVal2 () const { return val2; }
void DoubleSlider::setOrientation (bool horizontal) {
  isHorizontal=horizontal;
  update();
}
void DoubleSlider::setStepWidth (int s) {
  stepWidth=s;
}
void DoubleSlider::setSliderSize (int s) {
  triangleWidth=2*s;
  triangleHeight=static_cast<int>(6.5+1.732*s);
}
void DoubleSlider::setSliderImage (const QImage& image) {
  sliderImage=image;
  update();
}
void DoubleSlider::unsetRangeLightning () { rangeLightningMode=0; update(); }
void DoubleSlider::setRangeLightningOrdered () { rangeLightningMode=1; update(); }
void DoubleSlider::setRangeLightningRing () { rangeLightningMode=2; update(); }
void DoubleSlider::setDrawNumbers (bool b) { doDrawNumbers=b; update(); }

void DoubleSlider::paintEvent(QPaintEvent *) {
  QPainter paint (this);
  int sliderlen;
  if (isHorizontal) {
    paint.translate (width()/2,height()/2);
    sliderlen = width()-10;
  } else {
    QMatrix trans (0, -1, 1, 0, width()/2, height()/2);
    paint.setWorldMatrix (trans);
    sliderlen = height()-10;
  }

  int x1 = getWidgetPos (val1)-sliderlen/2-5;
  int x2 = getWidgetPos (val2)-sliderlen/2-5;
  QColor mainColor = Qt::black;
  QColor highlightColor = Qt::lightGray;

  // Hervorhebung zeichnen:
  paint.setPen (QPen (highlightColor));
  paint.setBrush (QBrush (highlightColor, Qt::SolidPattern));
  if (rangeLightningMode==1) {
    if (val1<=val2) {
      paint.drawRect (x1, -triangleHeight, x2-x1+1, 2*triangleHeight);
    }
  } else if (rangeLightningMode==2) {
    if (val1<=val2) {
      paint.drawRect (x1, -triangleHeight, x2-x1+1, 2*triangleHeight);
    } else {
      paint.drawRect (x1, -triangleHeight, sliderlen/2-x1+1, 2*triangleHeight);
      paint.drawRect (-sliderlen/2, -triangleHeight, x2+sliderlen/2, 2*triangleHeight);
    }
  }

  // Mittelleiste zeichnen:
  paint.setPen (QPen (mainColor));
  paint.setBrush (QBrush (Qt::black, Qt::SolidPattern));
  if (!sliderImage.isNull())
    paint.drawImage (QRect (-sliderlen/2,-2,sliderlen, 4), sliderImage, sliderImage.rect());
  else
    paint.drawRect (-sliderlen/2,-2,sliderlen, 4);

  // Dreiecke zeichnen:
  QPoint trc1 [] = {
    QPoint (x1, -6),
    QPoint (x1-triangleWidth/2, -triangleHeight),
    QPoint (x1+triangleWidth/2, -triangleHeight) };
  QPoint trc2 [] = {
    QPoint (x2, +6),
    QPoint (x2-triangleWidth/2, +triangleHeight),
    QPoint (x2+triangleWidth/2, +triangleHeight) };
  paint.drawConvexPolygon (trc1, 3);
  paint.drawConvexPolygon (trc2, 3);

  // Nummern zeichnen:
  if (doDrawNumbers) {
    if (isHorizontal) {
      QPainter tpaint (this);
      if (x1<0) {
        int x =getWidgetPos (val1)+triangleWidth/2+3;
        tpaint.drawText (x, height()/2-triangleHeight-30, 100, triangleHeight-4+30, Qt::AlignLeft | Qt::AlignBottom, QString::number(val1));
      } else {
        int x =getWidgetPos (val1)-triangleWidth/2-3;
        tpaint.drawText (x-100, height()/2-triangleHeight-30, 100, triangleHeight-4+30, Qt::AlignRight | Qt::AlignBottom, QString::number(val1));
      }
      if (x2<0) {
        int x =getWidgetPos (val2)+triangleWidth/2+3;
        tpaint.drawText (x, height()/2+4, 100, triangleHeight+30, Qt::AlignLeft | Qt::AlignTop, QString::number(val2));
      } else {
        int x =getWidgetPos (val2)-triangleWidth/2-3;
        tpaint.drawText (x-100, height()/2+4, 100, triangleHeight+30, Qt::AlignRight | Qt::AlignTop, QString::number(val2));
      }
    } else {
      QPainter tpaint (this);
      if (x1<0) {
        int x = height()-1-(getWidgetPos (val1)+triangleWidth/2+3);
        tpaint.drawText (width()/2-4-100, x-30, 100, 30, Qt::AlignRight | Qt::AlignBottom, QString::number(val1));
      } else {
        int x =height()-1-(getWidgetPos (val1)-triangleWidth/2-3);
        tpaint.drawText (width()/2-4-100, x, 100, 30, Qt::AlignRight | Qt::AlignTop, QString::number(val1));
      }
      if (x2<0) {
        int x =height()-1-(getWidgetPos (val2)+triangleWidth/2+3);
        tpaint.drawText (width()/2+4, x-30, 100, 30, Qt::AlignLeft | Qt::AlignBottom, QString::number(val2));
      } else {
        int x =height()-1-(getWidgetPos (val2)-triangleWidth/2-3);
        tpaint.drawText (width()/2+4, x, 100, 30, Qt::AlignLeft | Qt::AlignTop, QString::number(val2));
      }
    }
  }
}

int DoubleSlider::getWidgetPos (int val) {
  int len = (isHorizontal ? width() : height())-10;
  return (val-valMin)*len/(valMax-valMin)+5;
}

int DoubleSlider::getWidgetValue (int pos) {
  int len = (isHorizontal ? width() : height())-10;
  return valMin+(pos-5)*(valMax-valMin)/len;
}

void DoubleSlider::keyPressEvent(QKeyEvent* ev) {
  switch (ev->key()) {
    case Qt::Key_Left:
      if (val1>valMin) val1--;
      emit (valueChangedManually (val1, val2));
      update();
      break;
    case Qt::Key_Right:
      if (val1<valMax) val1++;
      emit (valueChangedManually (val1, val2));
      update();
      break;
    case Qt::Key_Down:
      if (val2>valMin) val2--;
      emit (valueChangedManually (val1, val2));
      update();
      break;
    case Qt::Key_Up:
      if (val2<valMax) val2++;
      emit (valueChangedManually (val1, val2));
      update();
      break;
    default:
      break;
  }
}

void DoubleSlider::mousePressEvent(QMouseEvent* ev) {
  int x = (isHorizontal ? ev->x() : height()-1-ev->y());
  int y = (isHorizontal ? ev->y() : ev->x());
  int h = (isHorizontal ? height()/2 : width()/2);
  int p1 = getWidgetPos (val1);
  int p2 = getWidgetPos (val2);
  bool isSlider1 = y<h && y+3>=h-triangleHeight;
  bool isSlider2 = y>h && y<=h+triangleHeight+3;
  if (!isSlider1 && !isSlider2) return;
  int p = (isSlider1 ? p1 : p2);
  bool holdSlider = x+3>=p-triangleWidth/2 && x<=p+triangleWidth/2+3;
  mousePressOffset = x-p;
  mousePressed = (holdSlider ? (isSlider1 ? +1 : -1) : 0);
  if (!holdSlider) {
    int offset = (mousePressOffset>0 ? stepWidth : (mousePressOffset<0 ? -stepWidth : 0));
    int& val (isSlider1 ? val1 : val2);
    val+=offset;
    if (val<valMin) val=valMin;
    if (val>valMax) val=valMax;
    emit (valueChangedManually (val1, val2));
    update();
  }
}
void DoubleSlider::mouseMoveEvent(QMouseEvent* ev) {
  if (mousePressed==0) return;
  int x = (isHorizontal ? ev->x() : height()-1-ev->y());
  int& val (mousePressed>0 ? val1 : val2);
  val = getWidgetValue (x-mousePressOffset);
  if (val<valMin) val=valMin;
  if (val>valMax) val=valMax;
  emit (valueChangedManually (val1, val2));
  update();
}
