
#include "ValPlotWidget.h"

namespace {

  const QColor colors [] = {
    QColor (255,0,0),
    QColor (0,255,0),
    QColor (0,0,255),
    QColor (0,255,255),
    QColor (255,0,255),
    QColor (255,255,0),
    QColor (255,255,255)
  };
  const unsigned int num_colors=7;
  
}

TribotsTools::ValPlotWidget::ValPlotWidget(QWidget* parent, const char * name, WFlags f) 
  : QFrame(parent, name, f)
{
  max_val = 1;
  min_val = 0;
  length = 10;
}


void TribotsTools::ValPlotWidget::init_widget(float _max_val, float _length)
{
  init_widget_multi (0, _max_val, _length, 1);
}

void TribotsTools::ValPlotWidget::init_widget (float _min_val, float _max_val, float _length) {
  init_widget_multi (_min_val, _max_val, _length, 1);
}

void TribotsTools::ValPlotWidget::init_widget_multi (float _min_val, float _max_val, float _length, unsigned int numc)
{
  max_val = _max_val;
  min_val = _min_val;
  length  = _length;
  values.resize (numc);
}

void TribotsTools::ValPlotWidget::paintEvent(QPaintEvent *)
{
  unsigned int maxx = 0;
  for (unsigned int i=0; i<values.size(); i++)
    if (values[i].size()>maxx)
      maxx=values[i].size();
  QPainter p(this);
  p.setPen(QPen(QColor(0,0,0), 1));
  float range = max_val - min_val;
  p.scale(this->width()/length, (this->height()-1)/100.0) ;

  // oberer, unterer Rand, min_val, max_val, Nulllinie  
  p.drawText(0,100,QString::number(min_val));
  p.drawText(0,10,QString::number(max_val));
  p.drawLine(0, 100, maxx, 100);
  p.drawLine(0,0,maxx,0);
  if (min_val<0 && max_val>0)
    p.drawLine(0,
               (int) (100.0 - (( (- min_val) / range) * 100.0 )),
               maxx,
               (int) (100.0 - (( (- min_val) / range) * 100.0 )));

  // Die Kurven zeichnen
  for (unsigned int j=0; j<values.size(); j++) {
    p.setPen(QPen(colors[j%num_colors], 1));
    int prevscalval=0;
    for (unsigned int i=0; i<values[j].size(); i++)
    {
      int scalval = static_cast<int> (100.0 - (( (values[j][i] - min_val) / range) * 100.0 ));
      if (scalval<0) scalval=0;
      if (scalval>100) scalval=100;
//      p.drawPoint(i,scalval);
      if (i>0)
        p.drawLine (i-1,prevscalval, i, scalval);
      prevscalval = scalval;
    }
  }
}

void  TribotsTools::ValPlotWidget::push(float val, unsigned int index)
{
  if (index>=values.size()) return;
  if (values[index].size() >= length)
    values[index].erase(values[index].begin());
  values[index].push_back(val);
}
