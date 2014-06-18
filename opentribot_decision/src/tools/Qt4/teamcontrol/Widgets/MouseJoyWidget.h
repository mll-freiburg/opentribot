
#ifndef _TribotsTools_MouseJoy_Widget_h_
#define _TribotsTools_MouseJoy_Widget_h_

#include <QtGui/QFrame>
#include <QtGui/QMouseEvent>

namespace TribotsTools {

  /** Widget, Joystick mit Maus und Tatstatur zu simulieren */
  class MouseJoyWidget : public QFrame
  {
  Q_OBJECT

  protected:
    void paintEvent(QPaintEvent*);
    void mousePressEvent (QMouseEvent*);
    void mouseReleaseEvent (QMouseEvent*);
    void mouseMoveEvent (QMouseEvent*);
    void keyPressEvent (QKeyEvent*);
    void keyReleaseEvent (QKeyEvent*);
    void enterEvent (QEvent*);
    void leaveEvent (QEvent*);
    void focusInEvent (QFocusEvent*);
    void focusOutEvent (QFocusEvent*);

    bool is_active;
    bool has_focus;
    bool mouse_inside;
    bool button_pressed;

    double vx, vy, vphi;
    bool kick;

    virtual void setJoyPos ();
    virtual void setNoVel ();  ///< Geschwindigkeit auf Null setzen
    virtual void centerRadius (double&, double&, double&); ///< Mittelpunkt und Radius der Steuerflaeche berechnen und in arg1-arg3 zurueckliefern

    void debugState (const char*);

  public:
    MouseJoyWidget(QWidget* parent=0, Qt::WindowFlags f=0);
    ~MouseJoyWidget () {;}

    virtual void activate (bool);   ///< an/ausschalten
  };

}

#endif
