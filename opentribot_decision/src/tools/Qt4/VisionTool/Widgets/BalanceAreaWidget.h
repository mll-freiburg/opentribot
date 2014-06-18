
#ifndef _TribotsTools_BalanceAreaWidget_h_
#define _TribotsTools_BalanceAreaWidget_h_

#include "UI/BalanceAreaWidget.h"
#include "VisionToolWidget.h"
#include "../Logic/VisionToolImageSource.h"

namespace TribotsTools {

  class BalanceAreaWidget : public VisionToolWidget, private Ui::BalanceAreaWidget {
    Q_OBJECT

  public:
    BalanceAreaWidget (VisionToolImageSource& , Tribots::ConfigReader&, QStatusBar&, QWidget* =0, Qt::WindowFlags =0);
    ~BalanceAreaWidget ();

  private slots:
    void autoToggled(bool);
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
    bool whileDrawing;

    double centerx;
    double centery;
    double minradius;
    double maxradius;

    unsigned int x0;
    unsigned int y0;
    unsigned int x1;
    unsigned int y1;

    std::string section;
    VisionToolImageSource& imageSource;
  };

}

#endif
