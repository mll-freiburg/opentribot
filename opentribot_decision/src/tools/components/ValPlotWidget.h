#ifndef _VALPLOTWIDGET_H_
#define _VALPLOTWIDGET_H_

#include <qframe.h>
#include <qpainter.h>
#include <vector>
#include <deque>

namespace TribotsTools {

  /** Widget, um Funktionsverlaeufe ueber der Zeit zu plotten */
  class ValPlotWidget : public QFrame
  {
  Q_OBJECT

  protected:
    virtual void paintEvent(QPaintEvent *ev);
    float min_val, max_val, length;
    std::vector<std::deque<float> > values;
    
  public:
    ValPlotWidget(QWidget* parent=0,const char * name=0, WFlags f=0);
    ~ValPlotWidget () {;}

    /** arg1=Maximalwert (Minimalwert=0), arg2=Breite gleitendes Fenster */
    virtual void init_widget(float _max_val, float _length);
    /** arg1=Minimalwert, arg2=Maximalwert, arg3=Breite gleitendes Fenster */
    virtual void init_widget(float min_val, float _max_val, float _length);
    /** arg1=Minimalwert, arg2=Maximalwert, arg3=Breite gleitendes Fenster, arg4=Anzahl einzutragender Kurven */
    virtual void init_widget_multi(float min_val, float _max_val, float _length, unsigned int);
    /** Wert (arg1) zu (arg2)-ter Kurve hinzufuegen */
    virtual void push(float val, unsigned int =0);
  };
  
}

#endif
