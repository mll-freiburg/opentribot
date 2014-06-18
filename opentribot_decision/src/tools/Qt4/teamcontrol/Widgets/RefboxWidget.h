
#ifndef _TribotsTools_RefboxWidget_h_
#define _TribotsTools_RefboxWidget_h_

#include "UI/RefboxWidget.h"
#include <QtGui/QWidget>

namespace TribotsTools {

  class RefboxWidget : public QWidget, private Ui::RefboxWidget {
    Q_OBJECT

  public:
    RefboxWidget(QWidget* =0, Qt::WindowFlags =0);
    virtual ~RefboxWidget () {;}

  public slots:
    void changeOwnHalfPressed();
    void changeLabelPressed();
    void stopPressed();
    void readyPressed();
    void startPressed();
    void droppedBallPressed();
    void kickOffOwnPressed();
    void throwInOwnPressed();
    void goalKickOwnPressed();
    void cornerKickOwnPressed();
    void freeKickOwnPressed();
    void penaltyKickOwnPressed();
    void kickOffOpponentPressed();
    void throwInOpponentPressed();
    void goalKickOpponentPressed();
    void cornerKickOpponentPressed();
    void freeKickOpponentPressed();
    void penaltyKickOpponentPressed();
    void connectRefboxChanged( bool b );
    void refboxipChanged();
    void ownScoreChanged (int v);
    void opponentScoreChanged(int v);
    void testStatePressed(const QString&);
    void update ();

  public:
    void startAsSlave();
    void stopAsSlave();
  };

}

#endif
