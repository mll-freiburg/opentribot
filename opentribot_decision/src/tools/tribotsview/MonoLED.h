
#ifndef TribotsTools_MonoLED_h
#define TribotsTools_MonoLED_h

#include <qwidget.h>

namespace TribotsTools {

  /** Widget, eine einfarbige LED simuliert */
  class MonoLED : public QWidget {
    Q_OBJECT

   public:
    /** Konstruktor; 
	Argumente wie bei normalen Widgets 
	Arg4: Farbe der LED bei Aktivierung */
    MonoLED ( QWidget* = 0, const char* = 0, WFlags = 0, QColor = Qt::red);
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

