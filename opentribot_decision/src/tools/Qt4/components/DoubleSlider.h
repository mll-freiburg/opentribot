
#ifndef _TribotsTools_DoubleSlider_h_
#define _TribotsTools_DoubleSlider_h_

#include <QtGui/QWidget>
#include "../components/FieldOfPlay.h"

namespace TribotsTools {

  /** Doppel-Slider zur Intervallauswahl */
  class DoubleSlider : public QWidget {
    Q_OBJECT

   public:
     DoubleSlider ( QWidget* = 0, Qt::WFlags = 0 );
    ~DoubleSlider ();

  signals:
    void valueChangedExplicitely (int, int);  ///< explizites Aendern eines Wertes mit setVal1(), setVal2()
    void valueChangedManually (int, int);  ///< manuelles Aendern eines Wertes durch Maus/Tastatur

  public:
    int getVal1 () const;  ///< ersten Wert abfragen
    int getVal2 () const;  ///< zweiten Wert abfragen

  public slots:
    void setMinValue (int);  ///< Minimum setzen (default=0)
    void setMaxValue (int);  ///< Maximum setzen (default=100)
    void setVal1 (int);  ///< ersten Wert setzen
    void setVal2 (int);  ///< zweiten Wert setzen
    void setOrientation (bool horizontal);  ///< horizontal (true, default) oder vertikal (false) anordnen
    void setStepWidth (int);  ///< die Sprungweite, wenn man neben den Schieber klickt (default=10)
    void setSliderSize (int);  ///< Groesse des Schiebers (default=4)
    void setSliderImage (const QImage&);  ///< das Bild, das in der Mittelachse des Schiebers angezeigt wird
    void unsetRangeLightning ();  ///< das gewaehlte Intervall nicht optisch hervorheben (default)
    void setRangeLightningOrdered ();  ///< das gewaehlte Intervall grau hinterlegen; fuer geordnete Werte
    void setRangeLightningRing ();  ///< dito; fuer Ringe (z.B. Hue-Wert von Farben)
    void setDrawNumbers (bool);  ///< die Intervallgrenzen als Nummern anzeigen (default=true)


  protected slots:
    void paintEvent(QPaintEvent *);
    void mousePressEvent(QMouseEvent*);
    void mouseMoveEvent(QMouseEvent*);
    void keyPressEvent(QKeyEvent*);

  private:
    int valMin;
    int valMax;
    int val1;
    int val2;
    int stepWidth;

    bool isHorizontal;
    int triangleHeight;
    int triangleWidth;
    int rangeLightningMode; ///< 0=nichts, 1=geordnete Struktur, 2=Ring
    QImage sliderImage;
    bool doDrawNumbers;

    int mousePressed;  ///< 0=nicht gedrueckt, +1=halte Slider 1, -1=halte Slider 2
    int mousePressOffset;  ///< der Offset gegenueber des genauen Wertes, wenn Mouse den Slider haelt

    int getWidgetPos (int val);  ///< zum Umrechnen von Werten in Positionen im Widget
    int getWidgetValue (int pos);  ///< Inverse von getWidgetPos
 };

}

#endif
