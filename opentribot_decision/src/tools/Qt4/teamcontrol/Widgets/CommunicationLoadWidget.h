
#ifndef _TribotsTools_CommunicationLoadWidget_h_
#define _TribotsTools_CommunicationLoadWidget_h_

#include "UI/CommunicationLoadWidget.h"
#include <QtGui/QWidget>
#include <QtGui/QPaintEvent>
#include <QtCore/QTime>

namespace TribotsTools {

  class CommunicationLoadWidget : public QWidget, private Ui::CommunicationLoadWidget {
    Q_OBJECT

  public:
    CommunicationLoadWidget(QWidget* =0, Qt::WindowFlags =0);
    virtual ~CommunicationLoadWidget () {;}

  public slots:
    virtual void update ();

  private:
    QTime latest_update;
  };

}

#endif
