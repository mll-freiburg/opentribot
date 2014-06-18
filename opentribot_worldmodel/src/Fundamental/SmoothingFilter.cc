#include "SmoothingFilter.h"

using namespace Tribots;
using namespace std;


// --- OnlineSmoothingFilterI implementation ----------------------------------------------------------------------------

OnlineSmoothingFilterI::OnlineSmoothingFilterI(const std::vector< double >& _c, int _nl, int _nr) throw (std::bad_alloc, std::invalid_argument)
  : nl(_nl), nr(_nr), c(_c), xrbuf(0), xwid(0), y(0), needsupdate(true)
{
  if (nl < 0) nl*=-1;
  if (nr < 0) nr*=-1;
  
  np = nl+nr+1;
  if ((int)c.size() != np) throw std::invalid_argument ("Mismatch in size of parameters for smoothing window.");
  
  xrbuf = new double[np];
  if (xrbuf==0) throw std::bad_alloc ();
}

OnlineSmoothingFilterI::~OnlineSmoothingFilterI()
{
  if (xrbuf!=0) delete[] xrbuf;
}

void OnlineSmoothingFilterI::addMeasurement(double m)
{
  xwid++;
  xwid=xwid%np;
  xrbuf[xwid] = m;
  needsupdate = true;
}

double OnlineSmoothingFilterI::getSmoothedValue()
{
  if (needsupdate) update();
  return y;
}

void OnlineSmoothingFilterI::update()
{
  double sum = 0;
  for (int h=-nl; h<=nr; h++) {
    int hc=(np-h)%np;
    int hx=( ( xwid - nr) + np + h)%np;    // just to stay in positive values
    sum += (c[hc] * xrbuf[hx]);
  }
  y = sum;
  needsupdate = false;
}


// --- MultiOnlineSmoothingFilterI implementation ----------------------------------------------------------------------------

MultiOnlineSmoothingFilterI::MultiOnlineSmoothingFilterI(int _n, const std::vector< double >& _c, int _nl, int _nr) 
  throw (std::bad_alloc, std::invalid_argument)
  : n(_n), filters(0)
{
  filters = new OnlineSmoothingFilterI*[n];
  if (filters==0) throw std::bad_alloc ();
  for (int i=0; i<n; i++) {
    filters[i] = 0;
    filters[i] = new OnlineSmoothingFilterI(_c, _nl, _nr);
    if (filters[i]==0) throw std::bad_alloc ();
  }
}
MultiOnlineSmoothingFilterI::MultiOnlineSmoothingFilterI(int _n, 
							 const std::vector< std::vector< double > >& _c, 
							 std::vector< int > _nl, 
							 std::vector< int > _nr ) throw (std::bad_alloc, std::invalid_argument)
  : n(_n), filters(0)
{
  if (_c.size() != _nl.size() || _c.size() != _nr.size() || (int)_c.size()!=n) 
    throw std::invalid_argument("Length of parameter vectors does not fit to number of filters in constructor.");

  filters = new OnlineSmoothingFilterI*[n];
  if (filters==0) throw std::bad_alloc ();
  for (int i=0; i<n; i++) {
    filters[i] = 0;
    filters[i] = new OnlineSmoothingFilterI(_c[i], _nl[i], _nr[i]);
    if (filters[i]==0) throw std::bad_alloc ();
  }
}

MultiOnlineSmoothingFilterI::~MultiOnlineSmoothingFilterI()
{
  for (int i=0; i<n; i++) {
    if (filters[i] != 0) delete filters[i];
  }
  delete[] filters;
}

void MultiOnlineSmoothingFilterI::addMeasurement(const std::vector< double >& m) throw (std::invalid_argument)
{
  if ((int)m.size()!=n) throw std::invalid_argument("Length of measurement vector does not fit number of filters.");
  for (int i=0; i<n; i++) filters[i]->addMeasurement(m[i]);
}

void MultiOnlineSmoothingFilterI::getSmoothedValue(std::vector< double >& s)
{
  s.clear();
  for (int i=0; i<n; i++) s.push_back(filters[i]->getSmoothedValue());
}

