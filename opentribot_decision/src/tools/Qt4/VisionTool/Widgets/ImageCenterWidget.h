
#ifndef _TribotsTools_ImageCenterWidget_h_
#define _TribotsTools_ImageCenterWidget_h_

#include "UI/ImageCenterWidget.h"
#include "VisionToolWidget.h"
#include "../Logic/VisionToolImageSource.h"

namespace TribotsTools {

  class ImageCenterWidget : public VisionToolWidget, private Ui::ImageCenterWidget {
    Q_OBJECT

  public:
    ImageCenterWidget (VisionToolImageSource& , Tribots::ConfigReader&, QStatusBar&, QWidget* =0, Qt::WindowFlags =0);
    ~ImageCenterWidget ();

  private slots:
    void autoToggled(bool);
    void debugToggled(bool);
    void mousePressedInImage(QMouseEvent*);
    void mouseMovedInImage(QMouseEvent*);
    void keyPressEvent(QKeyEvent*);

  public:
    void start ();
    void stop ();
    void loop ();

  private:
    bool started;
    bool autom;
    bool debug;
    bool whileDrawingCenter;
    bool whileDrawingRadius;

    double centerx;
    double centery;
    double minradius;
    double maxradius;

    std::string section;
    VisionToolImageSource& imageSource;
  };

}

#endif
