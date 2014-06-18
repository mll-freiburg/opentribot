
#ifndef _TribotsTools_TeamcontrolFieldWidget_h_
#define _TribotsTools_TeamcontrolFieldWidget_h_

#include "UI/TeamcontrolFieldWidget.h"
#include <QtGui/QMainWindow>
#include "../../../../Fundamental/Vec.h"


namespace TribotsTools {

  class TeamcontrolFieldWidget : public QMainWindow, private Ui::TeamcontrolFieldWidget {
    Q_OBJECT

  public:
    TeamcontrolFieldWidget(QWidget* =0, Qt::WindowFlags =0);
    ~TeamcontrolFieldWidget ();

  public slots:
    void paintEvent ( QPaintEvent * );
    void showEvent ( QShowEvent * );
    void unresolvedKeyPressEvent(QKeyEvent * );
    void gotoPos( Tribots::Vec p, Tribots::Angle h );
    void slhint();
    void updateRequests();

  };

}

#endif
