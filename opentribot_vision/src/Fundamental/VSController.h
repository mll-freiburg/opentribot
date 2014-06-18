#ifndef _TRIBOTS_VSCONTROLLER_
#define _TRIBOTS_VSCONTROLLER_

namespace Tribots {

  class VSController {
  public:
    VSController( unsigned int  _order , 
		  double        _k ,
		  double        _lambda ,
		  double        _phi);
    ~VSController( );

    double getAction(const double* _variable_with_derivs,
		     unsigned int  _size = 0,
		     double        _dt   = 0.033);
    void   setTarget(double _target);
    
  protected:
    unsigned int  order; 
    double        k;
    double        lambda;
    double        phi;
    double        target;

    double*       variable_with_derivs;
    double*       last_variable_with_derivs;
  };

}

#endif
