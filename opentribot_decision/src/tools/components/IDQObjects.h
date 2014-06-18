
#ifndef _TribotsTools_IDQObjects_h_
#define _TribotsTools_IDQObjects_h_

#include <qspinbox.h>
#include <qslider.h>
#include <qcombobox.h>
#include <qaction.h>

namespace TribotsTools {

  /** QSlider erweitert um ein Index-Attribut, das beim Signal sliderChanged zureuckgeliefert wird */
  class IDSlider : public QSlider {
    Q_OBJECT
    public:
      IDSlider ( QWidget *, unsigned int);
    protected:
      unsigned int id;
    protected slots:
      void captureValueChanged (int);
    signals:
      void sliderChanged (unsigned int, unsigned int);
  };
  
  /** QSpinBox erweitert um ein Index-Attribut, das beim Signal spinBoxChanged zureuckgeliefert wird */
  class IDSpinBox : public QSpinBox {
    Q_OBJECT
    public:
      IDSpinBox ( QWidget *, unsigned int);
    protected:
      unsigned int id;
    protected slots:
      void captureValueChanged (int);
    signals:
      void spinBoxChanged (unsigned int, unsigned int);
  };

  /** QComboBox erweitert um ein Index-Attribut, das beim Signal comboBoxChanged zureuckgeliefert wird.
      Zusaetzlich wird intern der letzte "gueltige" Wert der Combobox gespeichert (insbesondere fuer editierbare
      ComboBoxen von Vorteil */
  class IDComboBox : public QComboBox {
    Q_OBJECT
    public:
      IDComboBox ( QWidget *, unsigned int);
      /** liefere den letzten gueltigen Wert, der entweder explizit gesetzt wurde oder durch "enter" betstaetigt wurde */
      const QString& lastValidText () const;
      void setCurrentItem ( int );
      void setCurrentText ( const QString & );
    protected:
      unsigned int id;
      QString validText;
    protected slots:
      void captureValueChanged (int);
    signals:
      void comboBoxChanged (unsigned int, const QString&);
  };

  class IDAction : public QAction {
    Q_OBJECT
    public:
      IDAction ( QWidget *, unsigned int);
    protected:
      unsigned int id;
    protected slots:
      void captureActivated ();
      void captureToggled (bool);
    signals:
      void activated (unsigned int);
      void toggled (unsigned int,bool);
  };

}

#endif
