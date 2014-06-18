
#ifndef _TribotsTools_CameraDefinitionWidget_h_
#define _TribotsTools_CameraDefinitionWidget_h_

#include "UI/CameraDefinitionWidget.h"
#include "VisionToolWidget.h"
#include "../Logic/VisionToolImageSource.h"

namespace TribotsTools {

  class CameraDefinitionWidget : public VisionToolWidget, private Ui::CameraDefinitionWidget {
    Q_OBJECT

  public:
    CameraDefinitionWidget (VisionToolImageSource& , Tribots::ConfigReader&, QStatusBar&, QWidget* =0, Qt::WindowFlags =0);
    ~CameraDefinitionWidget ();

  private slots:
    void startPressed();
    void stopPressed();
    void restartPressed();
    void selectDevicePressed();

  public:
    void start ();
    void stop ();
    void loop ();

  private:
    void takeArguments ();  ///< Werte aus der GUI in die internen Variablen und in ConfigReader uebernehmen

    std::string device;
    std::string mode;
    std::string name;
    std::string uid;
    int delay;
    bool do_selftest;
    bool do_blocking;
    bool is_fileSource;

    VisionToolImageSource& imageSource;
  };

}

#endif
