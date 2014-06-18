
#ifndef _TribotsTools_MonoLED_h_
#define _TribotsTools_MonoLED_h_

#include <QtGui/QWidget>
#include <QtGui/QPaintEvent>

namespace TribotsTools {

  /** Widget, eine einfarbige LED simuliert */
  class MonoLED : public QWidget {
    Q_OBJECT

   public:
    /** Konstruktor; 
        Argumente wie bei normalen Widgets
        Arg4: Farbe der LED bei Aktivierung */
    MonoLED ( QWidget* = 0, Qt::WFlags = 0, QColor = Qt::red);
    /** Destruktor */
    ~MonoLED ();

    const QColor& color () const throw ();   ///< Farbe abfragen
    bool isOn () const throw ();             ///< Zustand abfragen
    void setColor (QColor) throw ();         ///< Farbe setzen

  public slots:
    void setOn (bool);                ///< an/ausschalten

  protected slots:
    void paintEvent(QPaintEvent *);   ///< Zeichnen des Widgetinhalts

  private:
    QColor col;
    bool on;
 };

}

#endif

