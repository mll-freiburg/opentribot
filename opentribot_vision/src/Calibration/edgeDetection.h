
#ifndef _Tribots_edgeDetection_h_
#define _Tribots_edgeDetection_h_

#include "GrayLevelImage.h"
#include "Pixelset.h"
#include "../../Fundamental/geometry.h"
#include <deque>
#include <cmath>

namespace Tribots {

  /** berechnet den Gradienten von src und liefert die Laenge des Gradienten
      und die Richtung des Gradienten (in rad) zurueck */
  void gradient (GrayLevelImage& gradientMagnitude,
                 GrayLevelImage& gradientDirection,
                 const GrayLevelImage& src);

  /** Canny-Kantenerkennungsoperator. Nimmt ein Grauwertbild in src und
      liefert in dest eine Liste der erkannten Liniezuege, wobei jeder
      Linienzug durch eine Liste seiner Pixel beschrieben wird */
  void canny (std::deque<Pixelset >& dest,
            const GrayLevelImage& src,
            unsigned int gausswidth =5,
            float lowerThreshold =6,
            float upperThreshold =12);

  /** ein Liniensegment mit Anfangs- und Endpunkt ins Bild einzeichnen.
      color ist die Farge der Line, ccolor die Farbe des ersten und letzten Pixels */
  void drawLine (Image& dest, const Vec& p1, const Vec& p2, const RGBTuple& color, const RGBTuple& ccolor);

  /** eine Pixelliste in einzelne Teillinien zerlegen. In breakpoints werden
      die Indeces der Bruchpunkte zurueckgeliefert. maxdist ist die
      maximale tolerierte Abweichung */
  void breakLine (std::deque<unsigned int>& breakpoints, const Pixelset& pixels, float maxdist =3);

  /** Aufbrechen der Pixelliste src an den Stellen breakpoints und absteigende
      Sortierung nach Laenge der Einzelteile */
  void splitLineSorted (std::deque<Pixelset >& dest, const Pixelset& src, const std::deque<unsigned int>& breakpoints) throw (std::bad_alloc);

  /** Aus den Pixeln im Intervall [begin, end) eine Linienschaetzung
      durchfuehren */
  template<class ITERATOR>
  LineSegment estimateLineSegment (const ITERATOR& begin, const ITERATOR& end) throw (std::invalid_argument);
}






// Implementierung von template-Funktion:

template<class ITERATOR>
Tribots::LineSegment Tribots::estimateLineSegment (const ITERATOR& begin, const ITERATOR& end) throw (std::invalid_argument) {
  // Homogenes lineares Gleichungssystem aufstellen und loesen
  double n=0;
  double sx=0;   // sum (x_i)
  double sy=0;   // sum (y_i)
  double sxy=0;  // sum (x_i y_i)
  double sxx=0;  // sum (x_i x_i)
  double syy=0;  // sum (y_i y_i)
  for (ITERATOR it = begin; it!=end; it++) {
    n++;
    sx+=it->x;
    sy+=it->y;
    sxy+=it->x*it->y;
    sxx+=it->x*it->x;
    syy+=it->y*it->y;
  }
  if (n<2)
    throw std::invalid_argument ("Tribots::estimateLineSegment: line cannot be estimated from less than 2 points");

  double a=2*(sxy-sx*sy/n);
  double b=sxx-sx*sx/n-syy+sy*sy/n;
  double c=std::sqrt(a*a+b*b);

  double phi=0;
  if (b+c!=0)
    phi=atan(a/(b+c))+M_PI/2;
  double alpha=cos(phi);
  double beta=sin(phi);
  double gamma=-(sx*alpha+sy*beta)/n;
  // Geradengleichung ist: x alpha+y beta+gamma=0 wobei alpha^2+beta^2=1

  // die beiden aeussersten Punkte an der Geraden berechnen:
  double mintau=1e300;
  double maxtau=-1e300;
  for (ITERATOR it = begin; it!=end; it++) {
    double tau = alpha*it->y-beta*it->x;
    if (tau<mintau) {
      mintau=tau;
    }
    if (tau>maxtau) {
      maxtau=tau;
    }
  }
  // Anfangs und Endpunkte der Strecke:
  Vec p1, p2;
  p1.x=-alpha*gamma-mintau*beta;
  p1.y=-beta*gamma+mintau*alpha;
  p2.x=-alpha*gamma-maxtau*beta;
  p2.y=-beta*gamma+maxtau*alpha;
  try{
    return Tribots::LineSegment (p1, p2);
  }catch(std::invalid_argument&) {
    throw std::invalid_argument ("Tribots::estimateLineSegment: first and last point of line are identical");
  }
}


#endif
