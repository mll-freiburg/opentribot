
#include "ColorCalibrationWidget.h"
#include <cmath>
#include <sstream>
#include <QtGui/QStatusBar>
#include "../../../../ImageProcessing/ObjectAnalysis/ColorClasses.h"

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

ColorCalibrationWidget::ColorCalibrationWidget(VisionToolImageSource& is, Tribots::ConfigReader& cfg, QStatusBar& stb, QWidget* p, Qt::WindowFlags f) : VisionToolWidget (cfg,stb,p,f), started (false), imageSource(is) {
  setupUi (this);
  setFocusPolicy (Qt::StrongFocus);
  sliderBall->setMinValue (-512);
  sliderBall->setMaxValue (255);
  sliderBall->setRangeLightningOrdered ();
  sliderBall->setSliderSize (6);
  sliderHue->setMinValue (0);
  sliderHue->setMaxValue (360);
  sliderHue->setRangeLightningRing ();
  sliderHue->setSliderSize (6);
  sliderHue->setSliderImage (QImage (":/slider/Icons/hue.png"));
  sliderSaturation->setMinValue (0);
  sliderSaturation->setMaxValue (100);
  sliderSaturation->setRangeLightningOrdered ();
  sliderSaturation->setSliderSize (6);
  sliderSaturation->setSliderImage (QImage (":/slider/Icons/saturation.png"));
  sliderIntensity->setMinValue (0);
  sliderIntensity->setMaxValue (100);
  sliderIntensity->setRangeLightningOrdered ();
  sliderIntensity->setSliderSize (6);
  sliderIntensity->setSliderImage (QImage (":/slider/Icons/intensity.png"));

  connect (sliderBall, SIGNAL(valueChangedManually (int, int)), this, SLOT(ballValueChanged(int,int)));
  connect (sliderHue, SIGNAL(valueChangedManually (int, int)), this, SLOT(hueValueChanged(int,int)));
  connect (sliderSaturation, SIGNAL(valueChangedManually (int, int)), this, SLOT(saturationValueChanged(int,int)));
  connect (sliderIntensity, SIGNAL(valueChangedManually (int, int)), this, SLOT(intensityValueChanged(int,int)));
  Tribots::ColorClassInfoList colorInfoList;
  for (unsigned int i=0; i<colorInfoList.classList.size(); i++) {
    comboBoxColorClass->addItem (colorInfoList.classList[i]->name.c_str());
  }
  connect (comboBoxColorClass, SIGNAL(activated (int)), this, SLOT(colorSelected(int)));
  connect (imageWidget, SIGNAL(mousePressed(QMouseEvent*)), this, SLOT(mousePressedInImage(QMouseEvent*)));
  connect (pushButtonSetColor, SIGNAL(pressed()), this, SLOT(setButtonPressed()));
  connect (pushButtonAddColor, SIGNAL(pressed()), this, SLOT(addButtonPressed()));
  connect (pushButtonUndoColor, SIGNAL(pressed()), this, SLOT(undoButtonPressed()));
}

ColorCalibrationWidget::~ColorCalibrationWidget () {
  if (started)
    stop();
}


void ColorCalibrationWidget::start () {
  started=true;
  config.get ("VisionTool::Section", section);
  imageSource.setMode (false, true);
  section+="::";
  imageWidget->centerImage();
}

void ColorCalibrationWidget::stop () {
  started=false;
  imageSource.setMode ();  // default-Modus, tut nichts
}

void ColorCalibrationWidget::loop () {
  Tribots::Image& image (imageSource.getImage());
  if (clickPosition[0]>=0 && clickPosition[1]>=0) {
    RGBTuple rgb;
    image.getPixelRGB (clickPosition[0], clickPosition[1],& rgb);
    rgb2hsy (rgb, clickColorHSI[0], clickColorHSI[1], clickColorHSI[2]);
    stringstream str;
    str << "XY: (" << clickPosition[0] << ", " << clickPosition[1]
    << ")  RGB: (" << (int)rgb.r << ", " << (int)rgb.g << ", " << (int)rgb.b
    << ")  HSI: (" << clickColorHSI[0] << ", " << clickColorHSI[1] << ", " << clickColorHSI[2] << ")";
    statusBar.showMessage(str.str().c_str());
    frameColorPatch->setAutoFillBackground(true);
    QPalette qpal;
    qpal.setBrush (QPalette::Background, QColor (rgb.r, rgb.g, rgb.b));
    frameColorPatch->setPalette(qpal);
    clickPosition[0]=-1;
  }
  imageWidget->setImage (image);
}


void ColorCalibrationWidget::setRangeSliders () {
  int cid = imageSource.getClassifier().getActiveColor ();
  ColorRange cr = imageSource.getClassifier().getColorRange (cid);
  sliderBall->setVal1 (static_cast<int>(cr.getMin(3)));
  sliderBall->setVal2 (static_cast<int>(cr.getMax(3)));
  sliderHue->setVal1 (static_cast<int>(cr.getMin(0)));
  sliderHue->setVal2 (static_cast<int>(cr.getMax(0)));
  sliderSaturation->setVal1 (static_cast<int>(100*cr.getMin(1)));
  sliderSaturation->setVal2 (static_cast<int>(100*cr.getMax(1)));
  sliderIntensity->setVal1 (static_cast<int>(100*cr.getMin(2)));
  sliderIntensity->setVal2 (static_cast<int>(100*cr.getMax(2)));
}

void ColorCalibrationWidget::sliderChanged (int r, double v1, double v2) {
  int cid = imageSource.getClassifier().getActiveColor ();
  ColorRange cr = imageSource.getClassifier().getColorRange (cid);
  cr.setMin (r, v1);
  cr.setMax (r,v2);
  imageSource.getClassifier().setColorRange (cid, cr);
}

void ColorCalibrationWidget::ballValueChanged (int v1, int v2) {
  sliderChanged (3,v1,v2);
}

void ColorCalibrationWidget::hueValueChanged (int v1, int v2) {
  sliderChanged (0,v1,v2);
}

void ColorCalibrationWidget::saturationValueChanged (int v1, int v2) {
  sliderChanged (1,0.01*static_cast<double>(v1),0.01*static_cast<double>(v2));
}

void ColorCalibrationWidget::intensityValueChanged (int v1, int v2) {
  sliderChanged (2,0.01*static_cast<double>(v1),0.01*static_cast<double>(v2));
}

void ColorCalibrationWidget::colorSelected (int index) {
  imageSource.getClassifier().setActiveColor (index);
  setRangeSliders();
}

void ColorCalibrationWidget::mousePressedInImage (QMouseEvent* event) {
  clickPosition[0] = event->x();
  clickPosition[1]  = event->y();
}

void ColorCalibrationWidget::setButtonPressed () {
  int cid = imageSource.getClassifier().getActiveColor ();
  undoColorRanges = imageSource.getClassifier().getColorRange (cid);
  double minH = (clickColorHSI[0]>=10 ? clickColorHSI[0]-10 : clickColorHSI[0]+350);
  double maxH = (clickColorHSI[0]<=350 ? clickColorHSI[0]+10 : clickColorHSI[0]-350);
  double minS = (clickColorHSI[1]>=0.1 ? clickColorHSI[1]-0.1 : 0.0);
  double maxS = (clickColorHSI[1]<=0.9 ? clickColorHSI[1]+0.1 : 1.0);
  double minI = (clickColorHSI[2]>=0.1 ? clickColorHSI[2]-0.1 : 0.0);
  double maxI = (clickColorHSI[2]<=0.9 ? clickColorHSI[2]+0.1 : 1.0);
  sliderChanged (0, minH, maxH);
  sliderChanged (1, minS, maxS);
  sliderChanged (2, minI, maxI);
  if (undoColorRanges.getMin(3)==0 && undoColorRanges.getMax(3)==0)
    sliderChanged (3, -512, 255);
  setRangeSliders();
}
void ColorCalibrationWidget::addButtonPressed () {
  int cid = imageSource.getClassifier().getActiveColor ();
  undoColorRanges = imageSource.getClassifier().getColorRange (cid);
  double minH = (clickColorHSI[0]>=10 ? clickColorHSI[0]-10 : clickColorHSI[0]+350);
  double maxH = (clickColorHSI[0]<=350 ? clickColorHSI[0]+10 : clickColorHSI[0]-350);
  double minS = (clickColorHSI[1]>=0.1 ? clickColorHSI[1]-0.1 : 0.0);
  double maxS = (clickColorHSI[1]<=0.9 ? clickColorHSI[1]+0.1 : 1.0);
  double minI = (clickColorHSI[2]>=0.1 ? clickColorHSI[2]-0.1 : 0.0);
  double maxI = (clickColorHSI[2]<=0.9 ? clickColorHSI[2]+0.1 : 1.0);
  double isMinH = undoColorRanges.getMin(0);
  double isMaxH = undoColorRanges.getMax(0);
  if ( (isMinH<=minH && minH<=maxH && maxH<=isMaxH) ||
        (isMaxH<isMinH && isMinH<=minH && minH<=maxH) ||
        (maxH<=isMaxH && isMaxH<isMinH && isMinH<=minH) ||
        (minH<=maxH && maxH<=isMaxH && isMaxH<isMinH) )
  {
    // [minH, maxH] vollstaendig enthalten in [isMinH, isMaxH]
    minH=isMinH;
    maxH=isMaxH;
  } else if ( (isMinH<=minH && minH<=isMaxH && isMaxH<maxH) ||
                  (maxH<isMinH && isMinH<=minH && minH<=isMaxH) ||
                  (isMaxH<maxH && maxH<isMinH && isMinH<=minH) ||
                  (minH<=isMaxH && isMaxH<maxH && maxH<isMinH) )
  {
    // minH ist in [isMinH, isMaxH] enthalten, maxH aber nicht
    minH=isMinH;
  } else if ( (minH<isMinH && isMinH<=maxH && maxH<=isMaxH) ||
                  (isMaxH<minH && minH<isMinH && isMinH<=maxH) ||
                  (maxH<=isMaxH && isMaxH<minH && minH<isMinH) ||
                  (isMinH<=maxH && maxH<=isMaxH && isMaxH<minH) )
  {
    // maxH ist in [isMinH, isMaxH] enthalten, minH aber nicht
    maxH=isMaxH;
  } else if ( (isMinH<=maxH && maxH<minH && minH<=isMaxH) ||
                  (isMaxH<isMinH && isMinH<=maxH && maxH<minH) ||
                  (minH<=isMaxH && isMaxH<isMinH && isMinH<=maxH) ||
                  (maxH<minH && minH<=isMaxH && isMaxH<isMinH) )
  {
    // minH und maxH sind in [isMinH, isMaxH] enthalten, aber [minH, maxH] ist nicht in [isMinH, isMaxH] enthalten
    minH=0;
    maxH=360;
  } else {
    // [minH, maxH] und [isMinH, isMaxH] sind disjunkt -> pruefe, welche Ergaenzung das kleinere Intervall ergibt
    double d1=(isMaxH<minH ? minH-isMaxH : 360+minH-isMaxH);
    double d2=(maxH<isMinH ? isMinH-maxH : 360+isMinH-maxH);
    if (d1<d2)
       minH=isMinH;
    else
      maxH=isMaxH;
  }
  if (minS>undoColorRanges.getMin(1)) minS=undoColorRanges.getMin(1);
  if (maxS<undoColorRanges.getMax(1)) maxS=undoColorRanges.getMax(1);
  if (minI>undoColorRanges.getMin(2)) minI=undoColorRanges.getMin(2);
  if (maxI<undoColorRanges.getMax(2)) maxI=undoColorRanges.getMax(2);
  sliderChanged (0, minH, maxH);
  sliderChanged (1, minS, maxS);
  sliderChanged (2, minI, maxI);
  setRangeSliders();
}

void ColorCalibrationWidget::undoButtonPressed () {
  int cid = imageSource.getClassifier().getActiveColor ();
  ColorRange sw = imageSource.getClassifier().getColorRange (cid);
  imageSource.getClassifier().setColorRange (cid, undoColorRanges);
  undoColorRanges = sw;
  setRangeSliders();
}
