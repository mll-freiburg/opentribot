
#ifndef _TribotsTools_RotateView_h_
#define _TribotsTools_RotateView_h_

#include <QtGui/QWidget>
#include "../components/FieldOfPlay.h"

namespace TribotsTools {

  /** Widget, das eine runde Scheibe mit Angabe der Roboterausrichtung und 
      -Rotationsgeschwindigkeit darstellt */
  class RotateView : public QWidget {
    Q_OBJECT

   public:
    /** Konstruktor; Achtung: vor Verwendung muss init() aufgerufen werden 
        Argumente wie bei normalen Widgets */
    RotateView ( QWidget* = 0, Qt::WFlags = 0 );
    /** Destruktor */
    ~RotateView ();

    /** Initialisierung 
        RotateView wird an ein FieldOfPlay angebunden, das hier uebergeben wird.
        RotateView aktualisiert sich automatisch bei Veraenderungen im referenzierten FieldOfPlay */
    void init (const FieldOfPlay*);

  protected slots:
    void paintEvent(QPaintEvent *);   ///< Zeichnen des Widgetinhalts

  private:
    const FieldOfPlay* fop;           ///< das FieldOfPlay, aus dem die Informationen genommen werden
 };

}

#endif

