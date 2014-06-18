
#ifndef _MarkerWidget_H
#define _MarkerWidget_H

#include <qwidget.h>
#include <qimage.h>
#include <qtimer.h>

namespace TribotsTools {

  class MarkerWidget : public QWidget
  {
      Q_OBJECT

    public:
      enum Mode { blackScreen, redScreen, greenScreen, blueScreen, redMarker, calibrationMarker, nextPosition };
      MarkerWidget( QWidget* parent = 0, const char* name = 0, WFlags fl = 0 );
      ~MarkerWidget();

      void setPosition (unsigned int, unsigned int);
      void setMode (Mode);
      void getModePosition (Mode&, unsigned int&, unsigned int&);

    protected slots:
      void paintEvent( QPaintEvent * );
      void switchmode ();

    protected:
      Mode mode;
      unsigned int x;
      unsigned int y;

      Mode latestMode;
      unsigned int latestX;
      unsigned int latestY;
  };

}

#endif
