#ifndef _TRIBOTS_SMOOTHINGFILTER_H_
#define _TRIBOTS_SMOOTHINGFILTER_H_

#include <vector>
#include <stdexcept>

namespace Tribots {

  /** Class OnlineSmoothingFilterI for smoothing successive measured data on a window.
   *  The smoothing is done with an convolution of the data with the parameter vector c.
   *  c[0 ... ((nl+nr+1) = np)-1]
   *  
   *  The smoothing is done on a window on the data 
   *  including nl samples left (in the past),
   *  nr samples right (in the future),
   *  and the  measurement of the last computable data point itself.
   *  c has wrap around order for the parameters of the right hand samples following:
   *
   *  y(k) = c[0] * x(k) + c[1] * x(k-1) + c[2] * x(k-2) + ... + c[nl] * x(k-nl)
   *                     + c[np-1] * x(k+1) + c[np-2] * x(k+2) + ... + c[np-nr] * x(k+nr)
   *
   *  If you added a new measurement x(t) you will be able to read a new smoothed value only for time k=(t-nr).
   *  This is up to date information if nr=0 (causal filter).
   *
   *  Parameters for c can be derived from various sources: 
   *  - mean (c[i] = 1/np for all i from 0 to np-1)
   *  - exponential smoothing 
   *  - Savitzky-Golay
   *  - ...
   */
  class OnlineSmoothingFilterI {  
  public:
    /** Constructor.
     *	@param c  Input; The parameters for the convolution. See description above for details.
     *	@param nl Input; Length of the window to the left.
     *	@param nr Input; Length of the window to the right.
     */
    OnlineSmoothingFilterI(const std::vector< double >& c, int nl, int nr) throw (std::bad_alloc, std::invalid_argument);
    /** Destructor. */
    ~OnlineSmoothingFilterI();

    /** Adds a data point (measurement) to the filter buffer.
     *  @param m Input; The new measurent to add.
     */
    void addMeasurement(double m);

    /** Computes and gets the actual smoothed value.
     *  Dependent on the value of nr this can be older than the last added measurement.
     *  @return the smoothed value.
     */
    double getSmoothedValue();

  protected:
    int nl, nr, np;
    std::vector< double > c;

    // array for buffering measured values (ring buffer)
    double* xrbuf;
    // id of the last measurement written to the buffer [0 .. np-1]
    int    xwid;
    // The last smoothed value.
    double y;
    // flag if all available data has been processed
    bool needsupdate;
    /** Computes the actual smoothing.*/
    void update();
  };


  /** Class for convenience of the user to capsule multiple OnlineSmoothingFilterI for smoothing parallel measured data.
   *  See OnlineSmoothingFilterI for details.
   */
  class  MultiOnlineSmoothingFilterI {
  public:
    MultiOnlineSmoothingFilterI(int n, const std::vector< double >& c, int nl, int nr) throw (std::bad_alloc, std::invalid_argument);
    MultiOnlineSmoothingFilterI(int n, 
				const std::vector< std::vector< double > >& c, 
				std::vector< int > nl, 
				std::vector< int > nr ) throw (std::bad_alloc, std::invalid_argument);
    ~MultiOnlineSmoothingFilterI();
    
    void addMeasurement(const std::vector< double >& m) throw (std::invalid_argument);
    
    void getSmoothedValue(std::vector< double >& s);

    int n;
    OnlineSmoothingFilterI** filters;
    
  };
  
}

#endif
