
#include "ImageCenterWidget.h"
#include <cmath>
#include <QtGui/QStatusBar>
#include "../../../..//ImageProcessing/Formation/Painter.h"
#include "../../../..//ImageProcessing/Calibration/centerRingOperation.h"

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

ImageCenterWidget::ImageCenterWidget(VisionToolImageSource& is, Tribots::ConfigReader& cfg, QStatusBar& stb, QWidget* p, Qt::WindowFlags f) : VisionToolWidget (cfg,stb,p,f), started (false), imageSource(is) {
  setupUi (this);
  connect (checkBox_auto, SIGNAL(toggled(bool)), this, SLOT(autoToggled(bool)));
  connect (checkBox_debug, SIGNAL(toggled(bool)), this, SLOT(debugToggled(bool)));
  connect (imageWidget, SIGNAL(mousePressed(QMouseEvent*)), this, SLOT(mousePressedInImage(QMouseEvent*)));
  connect (imageWidget, SIGNAL(mouseMoved(QMouseEvent*)), this, SLOT(mouseMovedInImage(QMouseEvent*)));
  connect (imageWidget, SIGNAL(keyPressed(QKeyEvent*)), this, SLOT(keyPressEvent(QKeyEvent*)));
  setFocusPolicy (Qt::StrongFocus);
}

ImageCenterWidget::~ImageCenterWidget () {
  if (started)
    stop();
}


void ImageCenterWidget::autoToggled(bool b) {
  autom=b;
  if (!autom) {
    whileDrawingCenter=false;
    whileDrawingRadius=false;
  }
}

void ImageCenterWidget::debugToggled(bool b) {
  debug=b;
}

void ImageCenterWidget::mousePressedInImage(QMouseEvent* event) {
  if (!autom) {
    if (event->button()==Qt::LeftButton) {
      centerx = event->x();
      centery = event->y();
      whileDrawingCenter=true;
      whileDrawingRadius=false;
    } else if (event->button()==Qt::RightButton) {
      minradius=maxradius=sqrt((centerx-event->x())*(centerx-event->x())+(centery-event->y())*(centery-event->y()));
      whileDrawingCenter=false;
      whileDrawingRadius=true;
    }
  }
}

void ImageCenterWidget::mouseMovedInImage(QMouseEvent* event) {
  if (whileDrawingCenter) {
    centerx = event->x();
    centery = event->y();
  } else if (whileDrawingRadius) {
    minradius=maxradius=sqrt((centerx-event->x())*(centerx-event->x())+(centery-event->y())*(centery-event->y()));
  }
}

void ImageCenterWidget::keyPressEvent(QKeyEvent* event) {
  if (!autom) {
    switch (event->key()) {
      case Qt::Key_Up: centery-=1.0; break;
      case Qt::Key_Down: centery+=1.0; break;
      case Qt::Key_Right: centerx+=1.0; break;
      case Qt::Key_Left: centerx-=1.0; break;
      case Qt::Key_Plus: minradius+=1.0; maxradius+=1.0; break;
      case Qt::Key_Minus: minradius-=1.0; maxradius-=1.0; break;
      default: break;
    }
  }
}

void ImageCenterWidget::start () {
  started=true;
  imageSource.setMode (true, false);
  imageWidget->centerImage();
  whileDrawingCenter=false;
  whileDrawingRadius=false;

  autom=true;
  debug=false;

  config.get ("VisionTool::Section", section);
  section+="::";
  vector<double> v;
  config.get ((section+"image_center").c_str(), v);
  if (v.size()==2) {
    centerx = v[0];
    centery = v[1];
  }
  v.clear();
  config.get ((section+"center_radius").c_str(),v);
  if (v.size()==2) {
    minradius=v[0];
    maxradius=v[1];
  } else if (v.size()==1) {
    minradius=maxradius=v[0];
  }
  config.get ((section+"auto_center").c_str(),autom);

  checkBox_auto->setChecked(autom);
  checkBox_debug->setChecked(debug);
}

void ImageCenterWidget::stop () {
  started=false;
  imageSource.setMode ();  // default-Modus, tut nichts
  whileDrawingCenter=false;
  whileDrawingRadius=false;
  vector<double> v (2);
  v[0]=centerx;
  v[1]=centery;
  config.set ((section+"image_center").c_str(), v);
  v[0]=minradius;
  v[1]=maxradius;
  config.set ((section+"center_radius").c_str(),v);
  config.set ((section+"auto_center").c_str(),autom);
  imageSource.notify (section+"image_center");
  imageSource.notify (section+"center_radius");
  imageSource.notify (section+"auto_center");
}

void ImageCenterWidget::loop () {
  Tribots::Image& image (imageSource.getImage());
  if (autom) {
    Tribots::RGBImage destimage (image.getWidth(), image.getHeight());
    double ccx, ccy, ccrmin, ccrmax;
    if (Tribots::findCenterRing (ccx, ccy, ccrmin, ccrmax, destimage, image, debug)) {
      centerx=ccx;
      centery=ccy;
      minradius=ccrmin;
      maxradius=ccrmax;
    } else {
      statusBar.showMessage ("AutoCenter fehlgeschlagen", 2000);
    }
    if (debug) {
      imageWidget->setImage (destimage);
    } else {
      Tribots::Painter paint (image);
      RGBTuple blue = { 0,0,255 };
      paint.setColor (blue);
      paint.drawCircle (centerx, centery, 2);
      paint.drawCircle (centerx, centery, minradius);
      paint.drawCircle (centerx, centery, maxradius);
      imageWidget->setImage (image);
    }
  } else {
    Tribots::Painter paint (image);
    RGBTuple blue = { 0,0,255 };
    paint.setColor (blue);
    paint.drawCircle (centerx, centery, 2);
    paint.drawCircle (centerx, centery, maxradius);
    imageWidget->setImage (image);
  }
}
