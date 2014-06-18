
#ifndef _Tribots_circleFit_h_
#define _Tribots_circleFit_h_

#include <cmath>
#include <matrix_operations.h>
#include "Vec.h"

namespace Tribots {

  /** Funktion algebraicCircleFit berechnet aus einer Punktemenge die Parameter eines Kreises
      durch Minimierung des algebraischen Abstandes zu den Punkten.
      \arg center (return): brechneter Kreismittelpunkt
      \arg radius (return): berechneter Radius des Kreises
      \arg begin, end: Iteratoren, die den Bereich eines Containers beschreiben, der die Punktemenge enthaelt (Verwendung analog STL-Funktionen)

      Exceptions werden geworfen, wenn die Berechnung fehl schlaegt (d.h. singulaeres oder numerisch schlecht gestelltes Problem) */
  template<class ITERATOR>
  void algebraicCircleFit ( Tribots::Vec& center, double& radius, const ITERATOR& begin, const ITERATOR& end) throw (std::invalid_argument) {
    Martin::SyMatrix xx (3);
    Martin::ColumnVector xt (3);
    Martin::ColumnVector pp (3);
    xx.clear();
    xt.clear();
    for (ITERATOR it = begin; it!=end; it++) {
    double len2 = (it->x*it->x+it->y*it->y);
      xx.r_entry(1,1)+=it->x*it->x;
      xx.r_entry(1,2)+=it->x*it->y;
      xx.r_entry(1,3)+=it->x;
      xx.r_entry(2,2)+=it->y*it->y;
      xx.r_entry(2,3)+=it->y;
      xx.r_entry(3,3)+=1;
      xt.r_entry(1)-=it->x*len2;
      xt.r_entry(2)-=it->y*len2;
      xt.r_entry(3)-=len2;
    }
    try{
      Martin::gauss (pp, xx, xt);
    }catch(std::invalid_argument) {
      throw std::invalid_argument ("algebraicCircleFit: ill posed or singular matrix inversion");
    }
    center.x = -0.5*pp.entry(1);
    center.y = -0.5*pp.entry(2);
    double r2 = 0.25*(pp.entry(1)*pp.entry(1)+pp.entry(2)*pp.entry(2))-pp.entry(3);
    if (r2<0)
      throw std::invalid_argument ("algebraicCircleFit: negative radius.");
    radius=std::sqrt(r2);
  }

}

#endif
