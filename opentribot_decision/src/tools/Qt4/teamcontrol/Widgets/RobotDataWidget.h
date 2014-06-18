
#ifndef _TribotsTools_RobotDataWidget_h_
#define _TribotsTools_RobotDataWidget_h_

#include "UI/RobotDataWidget.h"
#include <QtGui/QWidget>


namespace TribotsTools {

  class RobotDataWidget : public QWidget, private Ui::RobotDataWidget {
    Q_OBJECT

  public:
    RobotDataWidget(QWidget* =0, Qt::WindowFlags =0);
    ~RobotDataWidget ();

  public slots:
    void init( unsigned int n);
    void update ();

  private:
    unsigned int robot;
  };

}

#endif
