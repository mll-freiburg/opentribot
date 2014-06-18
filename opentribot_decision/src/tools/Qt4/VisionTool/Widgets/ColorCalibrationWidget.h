
#ifndef _TribotsTools_ColorCalibrationWidget_h_
#define _TribotsTools_ColorCalibrationWidget_h_

#include "UI/ColorCalibrationWidget.h"
#include "VisionToolWidget.h"
#include "../Logic/VisionToolImageSource.h"

namespace TribotsTools {

  class ColorCalibrationWidget : public VisionToolWidget, private Ui::ColorCalibrationWidget {
    Q_OBJECT

  public:
    ColorCalibrationWidget (VisionToolImageSource& , Tribots::ConfigReader&, QStatusBar&, QWidget* =0, Qt::WindowFlags =0);
    ~ColorCalibrationWidget ();

  private slots:
    void setRangeSliders ();
    void ballValueChanged (int v1, int v2);
    void hueValueChanged (int v1, int v2);
    void saturationValueChanged (int v1, int v2);
    void intensityValueChanged (int v1, int v2);
    void colorSelected (int index);
    void mousePressedInImage (QMouseEvent* event);
    void setButtonPressed ();
    void addButtonPressed ();
    void undoButtonPressed ();

  public:
    void start ();
    void stop ();
    void loop ();

  private:
    bool started;
    bool autom;
    bool debug;

    std::string section;
    VisionToolImageSource& imageSource;
    Tribots::ColorRange undoColorRanges;
    double clickColorHSI [3];
    int clickPosition [2];

    void sliderChanged (int r, double v1, double v2);  ///< Abstraktion fuer XXXValueChanged(v1,v2)
  };

}

#endif
