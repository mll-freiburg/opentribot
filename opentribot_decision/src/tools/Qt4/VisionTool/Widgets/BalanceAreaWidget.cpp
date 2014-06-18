
#include "BalanceAreaWidget.h"
#include <cmath>
#include "../../../../ImageProcessing/Formation/Painter.h"
#include "../../../../ImageProcessing/Calibration/centerRingOperation.h"

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

BalanceAreaWidget::BalanceAreaWidget(VisionToolImageSource& is, Tribots::ConfigReader& cfg, QStatusBar& stb, QWidget* p, Qt::WindowFlags f) : VisionToolWidget (cfg,stb,p,f), started (false), whileDrawing(false),  imageSource(is) {
  setupUi (this);
  connect (checkBox_auto, SIGNAL(toggled(bool)), this, SLOT(autoToggled(bool)));
  connect (imageWidget, SIGNAL(mousePressed(QMouseEvent*)), this, SLOT(mousePressedInImage(QMouseEvent*)));
  connect (imageWidget, SIGNAL(mouseMoved(QMouseEvent*)), this, SLOT(mouseMovedInImage(QMouseEvent*)));
  connect (imageWidget, SIGNAL(keyPressed(QKeyEvent*)), this, SLOT(keyPressEvent(QKeyEvent*)));
  setFocusPolicy (Qt::StrongFocus);
}

BalanceAreaWidget::~BalanceAreaWidget () {
  if (started)
    stop();
}


void BalanceAreaWidget::autoToggled(bool b) {
  autom=b;
  whileDrawing=false;
  config.set ((section+"auto_balance_area").c_str(), autom);
  imageSource.notify (section+"auto_balance_area");
}

void BalanceAreaWidget::mousePressedInImage(QMouseEvent* event) {
  if (!autom) {
    if (event->button()==Qt::LeftButton) {
      x0=event->x();
      y0=event->y();
      x1=x0;
      y1=y0;
      whileDrawing=true;
    } else {
      whileDrawing=false;
    }
  }
}

void BalanceAreaWidget::mouseMovedInImage(QMouseEvent* event) {
  if (whileDrawing) {
    x1=event->x();
    y1=event->y();
    vector<double> v (4);
    v[0]=(x0<x1 ? x0 : x1);
    v[1]=(y0<y1 ? y0 : y1);
    v[2]=(x0<x1 ? x1-x0 : x0-x1);
    v[3]=(y0<y1 ? y1-y0 : y0-y1);
    config.set ((section+"balance_area").c_str(), v);
    if (abs((static_cast<int>(x0)-static_cast<int>(x1))*(static_cast<int>(y0)-static_cast<int>(y1)))>8)
      imageSource.notify (section+"balance_area");
  }
}

void BalanceAreaWidget::keyPressEvent(QKeyEvent* event) {
  if (!autom) {
    switch (event->key()) {
      case Qt::Key_Up: y0-=1; y1-=1; break;
      case Qt::Key_Down: y0+=1; y0+=1; break;
      case Qt::Key_Right: x0+=1; x1+=1; break;
      case Qt::Key_Left: x0-=1; x1-=1; break;
      case Qt::Key_Plus: y1+=1; break;
      case Qt::Key_Minus: y1-=1; break;
      case Qt::Key_Less: x1-=1; break;
      case Qt::Key_Greater: x1+=1; break;
      default: break;
    }
  }
}

void BalanceAreaWidget::start () {
  started=true;
  whileDrawing=false;
  imageSource.setMode (true, false);
  imageWidget->centerImage();

  autom = true;

  centerx=320;
  centery=240;
  minradius=35;

  x0=300;
  y0=220;
  x1=340;
  y1=260;

  config.get ("VisionTool::Section", section);
  section+="::";
  vector<double> v;
  config.get ((section+"image_center").c_str(), v);
  if (v.size()==2) {
    centerx=v[0];
    centery=v[1];
  }
  v.clear();
  config.get ((section+"center_radius").c_str(), v);
  if (v.size()>=1) {
    minradius=v[0];
    maxradius=minradius;
    if (v.size()>=2) {
      maxradius=v[1];
    }
  }
  v.clear();
  config.get ((section+"balance_area").c_str(), v);
  if (v.size()==4) {
    x0=static_cast<unsigned int>(v[0]);
    y0=static_cast<unsigned int>(v[1]);
    x1=x0+static_cast<unsigned int>(v[2]);
    y1=y0+static_cast<unsigned int>(v[3]);
  }
  config.get ((section+"auto_balance_area").c_str(), autom);
  checkBox_auto->setChecked(autom);
}

void BalanceAreaWidget::stop () {
  started=false;
  whileDrawing=false;
  imageSource.setMode ();  // default-Modus, tut nichts

  vector<unsigned int> v (4);
  v[0]=(x0<x1 ? x0 : x1);
  v[1]=(y0<y1 ? y0 : y1);
  v[2]=(x0<x1 ? x1-x0 : x0-x1);
  v[3]=(y0<y1 ? y1-y0 : y0-y1);
  config.set ((section+"balance_area").c_str(), v);
  config.set ((section+"auto_balance_area").c_str(), autom);
  imageSource.notify (section+"balance_area");
  imageSource.notify (section+"auto_balance_area");
}

void BalanceAreaWidget::loop () {
  Tribots::Image& image (imageSource.getImage());
  if (autom) {
    unsigned int x0i, y0i, x1i, y1i;
    Tribots::determineBalanceArea (x0i, y0i, x1i, y1i, centerx, centery, minradius, maxradius);
    if (x0i!=x0 || x1i!=x1 || y0i!=y0 || y1i!=y1) {
      vector<unsigned int> v (4);
      v[0]=x0=x0i;
      v[1]=y0=y0i;
      x1=x1i;
      y1=y1i;
      v[2]=x1-x0;
      v[3]=y1-y0;
      config.set ((section+"balance_area").c_str(), v);
      imageSource.notify (section+"balance_area");
    }
  }
  Tribots::Painter paint (image);
  RGBTuple blue = { 0,0,255 };
  if (!autom) {
    RGBTuple grey = {128,128,128};
    paint.setColor(grey);
    paint.drawLine (x0, 0, x0, image.getHeight()-1);
    paint.drawLine (x1, 0, x1, image.getHeight()-1);
    paint.drawLine (0, y0, image.getWidth()-1, y0);
    paint.drawLine (0, y1, image.getWidth()-1, y1);
  }
  paint.setColor (blue);
  paint.drawLine (x0, y0, x1, y0);
  paint.drawLine (x0, y0, x0, y1);
  paint.drawLine (x1, y1, x1, y0);
  paint.drawLine (x1, y1, x0, y1);
  imageWidget->setImage (image);
}
