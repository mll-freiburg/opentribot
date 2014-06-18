
#ifndef _TribotsTools_SettingsWidget_h_
#define _TribotsTools_SettingsWidget_h_

#include "UI/SettingsWidget.h"
#include "VisionToolWidget.h"
#include "../Logic/VisionToolImageSource.h"

namespace TribotsTools {

  class SettingsWidget : public VisionToolWidget, private Ui::SettingsWidget {
    Q_OBJECT

  public:
    SettingsWidget (VisionToolImageSource& , Tribots::ConfigReader&, QStatusBar&, QWidget* =0, Qt::WindowFlags =0);
    ~SettingsWidget ();

  private slots:
    void softWhiteBalanceToggled (bool);
    void softAutoExposureToggled (bool);
    void softAdjustGainToggled (bool);
    void softShutterPositiveLogicToggled (bool);
    void softShutterMinChanged (int);
    void softShutterMaxChanged (int);
    void softGainMinChanged (int);
    void softGainMaxChanged (int);
    void softUVMinChanged (int);
    void softUVMaxChanged (int);
    void softTargetExposureChanged (int);

  public:
    void start ();
    void stop ();
    void loop ();

  private:
    void changeParameters (const char* key, int val1, int val2);
    void changeParameters (const char* key, bool val);
    void changeParameters (const char* key, int val);
    void enable (bool b);

    std::string section;
    VisionToolImageSource& imageSource;
  };

}

#endif
