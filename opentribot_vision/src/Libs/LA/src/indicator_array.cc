
#include "indicator_array.h"

using namespace Martin;
using namespace std;


IndicatorArray::IndicatorArray (unsigned int d) throw (invalid_argument, bad_alloc) : dim (d) {
  if (d==0) 
    throw invalid_argument ("zero dimenison of IndicatorArray in Martin::IndicatorArray::IndicatorArray");
  f=new bool [d];
}

IndicatorArray::IndicatorArray (const IndicatorArray& li) throw (bad_alloc) : dim (li.dim) {
  f=new bool [dim];
  for (unsigned int i=0; i<dim; i++)
    f [i]=li.f [i];
}

IndicatorArray::~IndicatorArray () throw () {
  delete [] f;
}
    
const IndicatorArray& IndicatorArray::operator= (const IndicatorArray& li) throw (invalid_argument) {
  if (dim!=li.dim)
    throw invalid_argument ("mismatch of indexlist size in Martin::IndicatorArray::operator=");
  for (unsigned int i=0; i<dim; i++)
    f [i]=li.f [i];
  return (*this);
}  

bool IndicatorArray::operator== (const IndicatorArray& li) const throw () {
  if (dim!=li.dim) return false;
  for (unsigned int i=0; i<dim; i++)
    if (f [i]!=li.f [i]) return false;
  return true;
}

bool IndicatorArray::operator!= (const IndicatorArray& li) const throw () {
  return !operator== (li);
}

unsigned int IndicatorArray::size () const throw () {
  return dim;
}

unsigned int IndicatorArray::sub_size (bool b) const throw () {
  unsigned int res=0;
  for (unsigned int i=0; i<dim; i++)
    if (f [i]==b) res++;
  return res;
}

bool IndicatorArray::entry (unsigned int i) const throw () {
  return f [i-1];
}

bool IndicatorArray::at (unsigned int i) const throw (out_of_range) {
  if (i==0 || i>dim)
    throw out_of_range ("index exceeds indexlist dimensions in Martin::IndicatorArray::at");
  return entry (i);
}

bool& IndicatorArray::r_entry (unsigned int i) throw () {
  return f [i-1];
}
  
bool& IndicatorArray::r_at (unsigned int i) throw (out_of_range) {
  if (i==0 || i>dim)
    throw out_of_range ("index exceeds indexlist dimensions in Martin::IndicatorArray::at");
  return r_entry (i);
}

void IndicatorArray::set (bool b) throw () {
  for (unsigned int i=0; i<dim; i++)
    f [i]=b;
}



