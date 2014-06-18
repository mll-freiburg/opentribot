// File indicator_array.h:
// contains the declaration of thr class IndicatorArray
// created 02-MAY-01 by Martin Lauer
// ---------------------------------------------

#ifndef indicator_array_h
#define indicator_array_h

#include <stdexcept>
#include <iostream>

namespace Martin {

  // class IndicatorArray:
  // array of bool with a similar declaration to class Martin::ColumnVector
  // indeces run 1..size()
  // it is designed to model missing attributes for incomplete patterns
  // stored in a ColumnVector

  class IndicatorArray {
  public:
    IndicatorArray (unsigned int) throw (std::invalid_argument, std::bad_alloc); 
    // create new IndicatorArray with given size
    IndicatorArray (const IndicatorArray&) throw (std::bad_alloc);
    // copy IndicatorArray
    ~IndicatorArray () throw ();
    
    const IndicatorArray& operator= (const IndicatorArray&) throw (std::invalid_argument);
    // assignment, throw exception when sizes differ

    bool operator== (const IndicatorArray&) const throw ();
    bool operator!= (const IndicatorArray&) const throw ();

    unsigned int size () const throw ();
    // return size
    unsigned int sub_size (bool) const throw ();
    // return number of entries equal to the argument
    // i.e. number of 'true' ('false')-entries.
    bool entry (unsigned int) const throw ();
    bool at (unsigned int) const throw (std::out_of_range);
    // read entry, without (entry) or with (at) range-check
    // indeces run from 1..size()
    bool& r_entry (unsigned int) throw ();
    bool& r_at (unsigned int) throw (std::out_of_range);
    // write an entry

    void set (bool) throw (); 
    // set all entries to arg
    
  private:
    bool* f;
    unsigned int dim;
  };


}


#endif
