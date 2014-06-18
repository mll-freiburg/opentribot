
#ifndef _TribotsTools_TeamcontrolMainWidget_h_
#define _TribotsTools_TeamcontrolMainWidget_h_

#include "UI/TeamcontrolMainWidget.h"
#include <QtGui/QMainWindow>
#include <QtCore/QTimer>
#include "CoachWidget.h"
#include "JoystickWidget.h"
#include "RefboxWidget.h"
#include "RobotWidget.h"
#include "TeamcontrolFieldWidget.h"
#include "../Logic/Coach.h"
#include "../Logic/JoystickControl.h"
#include "../Logic/RefboxControl.h"
#include "../Logic/RemoteRobot.h"
#include "CommunicationLoadWidget.h"
#include "../States/TeamcontrolMode.h"
#include "../Logic/TeamcontrolMasterServer.h"
#include "../Logic/TeamcontrolSlaveClient.h"
#include <vector>


namespace TribotsTools {

  class TeamcontrolMainWidget : public QMainWindow, private Ui::TeamcontrolMainWidget {
    Q_OBJECT

  public:
    TeamcontrolMainWidget(QWidget* =0, Qt::WindowFlags =0);
    ~TeamcontrolMainWidget ();
    void init( const Tribots::ConfigReader & cfg );

  signals:
    void widgetClosed();

  protected:
    bool startAsSlave();
    bool startAsMaster();
    bool startAsSingle();
    bool stopAsSlave();
    bool stopAsMaster();
    bool stopAsSingle();

  protected slots:
    void cycleRateChanged( int n );
    void cycle_task();
    void cycle_task_refbox();
    void linesRateChanged( int n );
    void masterModeSelected();
    void slaveModeSelected();
    void singleModeSelected();
    void showMasterSlaveComm();
    void closeEvent ( QCloseEvent *);

  protected:
    TribotsTools::TeamcontrolSlaveClient* masterslave_client;
    TribotsTools::TeamcontrolMasterServer* masterslave_server;
    std::vector<TribotsTools::RemoteRobot*> remote_robot;
    TribotsTools::RefboxControl* refbox_control;
    TribotsTools::JoystickControl* joystick_control;
    TribotsTools::Coach* coach;
    CoachWidget* coach_widget;
    JoystickWidget* joystick_widget;
    RefboxWidget* refbox_widget;
    TeamcontrolFieldWidget* field_widget;
    std::vector<RobotWidget*> robot_widget;
    CommunicationLoadWidget* communication_load_widget;

    QTimer* cycle_timer;
    QTimer* cycle_timer_refbox;
    TribotsTools::TeamcontrolMode teamcontrol_mode;
    bool try_to_connect;
  };

}

#endif
