
#ifndef _TribotsTools_JoystickWidget_h_
#define _TribotsTools_JoystickWidget_h_

#include "UI/JoystickWidget.h"
#include <QtGui/QWidget>


namespace TribotsTools {

  class JoystickWidget : public QWidget, private Ui::JoystickWidget {
    Q_OBJECT

  public:
    JoystickWidget(QWidget* =0, Qt::WindowFlags =0);
    ~JoystickWidget ();

  public slots:
    void deviceChanged( int );
    void valueChanged( int );
    void valueChanged( bool );
    void perspectiveChanged( const QString & np );
    void update ();

  };

}

#endif
