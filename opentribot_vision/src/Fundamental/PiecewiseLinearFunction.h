#ifndef _TRIBOTS_PIECEWISELINEARFUNCTION_H_
#define _TRIBOTS_PIECEWISELINEARFUNCTION_H_

#include <vector>
#include "Vec.h"

namespace Tribots {

  /** 
   * Realizes a partially linear function using several supporting points
   * (vertices) for x > 0. Before the first vertex (x0, f(x0)), the function 
   * increases linearly from 0 to f(x0). After the last vertex (xn, f(xn)),
   * the function increases with constant gradient 1
   * 
   * Examples:
   * 
   * no vertices: f(x) = x for all x
   * 
   * one vertex (x, f(x)) := (1, 2) results in
   * f(0) = 0
   * f(0.5) = 1
   * f(1) = 2
   * f(2) = 3
   * f(5) = 6
   */  
  class PiecewiseLinearFunction
  {
  public:
	  PiecewiseLinearFunction();
  	virtual ~PiecewiseLinearFunction();
    /** add a vertex */
    virtual void addVertex(double x, double y);
    /** get f(x) */
    virtual double getValue(double x);
  protected:
    std::vector<Vec> vertices;
  };
  
}

#endif //_PARTIALLYLINEARFUNCTION_H_
