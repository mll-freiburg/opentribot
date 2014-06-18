
#ifndef _TribotsTools_RobotWidget_h_
#define _TribotsTools_RobotWidget_h_

#include "UI/RobotWidget.h"
#include <QtGui/QWidget>
#include <vector>
#include <string>
#include "../../../../Structures/GameState.h"
#include "RobotDataWidget.h"
#include "../../../../Fundamental/ConfigReader.h"


namespace TribotsTools {

  class RobotWidget : public QWidget, private Ui::RobotWidget {
    Q_OBJECT

  public:
    RobotWidget(QWidget* =0, Qt::WindowFlags =0);
    ~RobotWidget ();

    void startAsSlave();
    void stopAsSlave();
    void updateRequests();

  public slots:
    void init( unsigned int n, const Tribots::ConfigReader& cfg );
    void hide();
    void connectChanged();
    void activateChanged();
    void messageBoardChanged();
    void playertypeChanged( int );
    void playerroleChanged( int );
    void refstateChanged( int i );
    void ipChanged();
    void teamChanged();
    void robotdataChanged( bool b );
    void debugImageClicked();
    void yellowCardsChanged( int v );
    void startPressed();
    void update();

  private:
    std::vector<bool> was_comm_okay;
    unsigned int robot;
    RobotDataWidget* data_widget;
    std::string playerrole;
    std::string playertype;
    Tribots::RefereeState refstate;
    std::string ip;
    int port;
    int localization_side;
  };

}

#endif
