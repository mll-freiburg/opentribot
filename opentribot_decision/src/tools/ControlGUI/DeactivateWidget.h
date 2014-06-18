
#ifndef DEACTIVATEWIDGET_H
#define DEACTIVATEWIDGET_H

#include <qwidget.h>
#include <qimage.h>
#include <qtimer.h>
#include "../../Fundamental/Time.h"

class DeactivateWidget : public QWidget
{
    Q_OBJECT

public:
    DeactivateWidget( QWidget* parent = 0, const char* name = 0, WFlags fl = 0 );
    ~DeactivateWidget();
    void setBlackScreen (bool);

signals:
    void clicked();

protected slots:
    virtual void paintEvent( QPaintEvent * );
    virtual void keyPressEvent( QKeyEvent * );
    virtual void closeEvent( QCloseEvent * );
    virtual void mousePressEvent( QMouseEvent * );
    virtual void mouseReleaseEvent( QMouseEvent * );
    virtual void sendDeactivate();

protected:
  QImage backgroundImage;
  bool blackScreen;
  QTimer mousePressTimer;
};

#endif
