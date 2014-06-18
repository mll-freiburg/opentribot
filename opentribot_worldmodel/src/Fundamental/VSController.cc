#include "VSController.h"
#include <math.h>
#include <assert.h>

using namespace Tribots;

#define SIGN(__x__) (__x__ < 0 ? -1 : 1)

VSController::VSController( unsigned int  _order , 
			    double        _k ,
			    double        _lambda ,
			    double        _phi)
{
  order	  = _order;   assert(order > 0); 
  k       = _k;
  lambda  = _lambda;
  phi     = _phi;
  variable_with_derivs = new double[order];
  for (unsigned int i=0;i<order; i++) variable_with_derivs[i]=0.0;
  last_variable_with_derivs = new double[order];
  for (unsigned int i=0;i<order; i++) last_variable_with_derivs[i]=0.0;
}

VSController::~VSController( )
{
  delete[] variable_with_derivs;
  delete[] last_variable_with_derivs;
}

double VSController::getAction(const double* _variable_with_derivs,
			       unsigned int  _size ,
			       double        _dt   )
{
  double res = 0.0;
  unsigned int size = order;
  if(_size > 0 && _size <= order) size = _size;
  for (unsigned int i=0; i<size; i++) 
    variable_with_derivs[i]=_variable_with_derivs[i];
  // fill up missing derivatives
  for (unsigned int i=size; i<order; i++)
    variable_with_derivs[i]= (variable_with_derivs[i-1] - last_variable_with_derivs[i-1]) / _dt;

  double error_with_derivs[order];
  error_with_derivs[0] = target - variable_with_derivs[0];
  for (unsigned int i=1; i<order; i++) {
    error_with_derivs[i] = variable_with_derivs[i]*SIGN(error_with_derivs[0])*-1.0;
  }
  
  double s = 0;
  double l = 1;
  for (unsigned int i=0; i<order; i++) {
    s += error_with_derivs[i] * l;
    l*=lambda;
  }
  
  res = k * tanh( s / phi);
  
  for (unsigned int i=0; i<order; i++)
    last_variable_with_derivs[i] = variable_with_derivs[i];
  
  return res;
}

void   VSController::setTarget(double _target)
{
  target = _target;
}
