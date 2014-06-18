
#include "Time.h"

const Tribots::Time Tribots::Time::starting_time;

Tribots::Time::Time () throw () {
  update ();
}

Tribots::Time::Time (const Tribots::Time& src) throw () : sec (src.sec), usec (src.usec) {;}

Tribots::Time::Time (const timeval& tv) throw () {
  set (tv);
}
    
const Tribots::Time& Tribots::Time::operator= (const Tribots::Time& src) throw () {
  sec=src.sec;
  usec=src.usec;
  return *this;
}

void Tribots::Time::set_usec (const long int& d) throw () {
  sec=starting_time.sec;
  usec=starting_time.usec;
  add_usec (d);
}

void Tribots::Time::set_msec (const long int& d) throw () {
  sec=starting_time.sec;
  usec=starting_time.usec;
  add_msec (d);
}

void Tribots::Time::set_sec (const long int& d) throw () {
  sec=starting_time.sec;
  usec=starting_time.usec;
  add_sec (d);
}

void Tribots::Time::update () throw () {
  timeval systime;
  struct timezone no_good;
  gettimeofday (&systime, &no_good);
  sec=systime.tv_sec;
  usec=systime.tv_usec;
}

long int Tribots::Time::elapsed_usec () const throw () {
  Time now;
  return now.diff_usec (*this);
}

long int Tribots::Time::elapsed_msec () const throw () {
  Time now;
  return now.diff_msec (*this);
}

long int Tribots::Time::elapsed_sec () const throw () {
  Time now;
  return now.diff_sec (*this);
}

void Tribots::Time::add_usec (long int n) throw () {
  usec+=n;
  if (usec<0) {
    long int k=-usec/1000000+1;
    sec-=k;
    usec+=1000000*k;
  } else if (usec>=1000000) {
    long int k=usec/1000000;
    sec+=k;
    usec-=1000000*k;
  }
}

void Tribots::Time::add_msec (long int n) throw () {
  add_usec (1000*n);
}

void Tribots::Time::add_sec (long int n) throw () {
  sec+=n;
}

long int Tribots::Time::diff_usec (const Tribots::Time& src) const throw () {
  return 1000000*(sec-src.sec)+(usec-src.usec);
}

long int Tribots::Time::diff_msec (const Tribots::Time& src) const throw () {
  return 1000*(sec-src.sec)+(usec-src.usec)/1000;
}

long int Tribots::Time::diff_sec (const Tribots::Time& src) const throw () {
  return sec-src.sec;
}

bool Tribots::Time::operator== (const Tribots::Time& src) const throw () {
  return ((sec==src.sec) && (usec==src.usec));
}

bool Tribots::Time::operator!= (const Tribots::Time& src) const throw () {
  return !operator==(src);
}

bool Tribots::Time::operator<= (const Tribots::Time& src) const throw () {
  return (diff_usec (src)<=0);
}

bool Tribots::Time::operator< (const Tribots::Time& src) const throw () {
  return (diff_usec (src)<0);
}

bool Tribots::Time::operator>= (const Tribots::Time& src) const throw () {
  return (diff_usec (src)>=0);
}

bool Tribots::Time::operator> (const Tribots::Time& src) const throw () {
  return (diff_usec (src)>0);
}

long int Tribots::Time::get_usec () const throw () {
  return (sec-Time::starting_time.sec)*1000000+usec-Time::starting_time.usec;
}

long int Tribots::Time::get_msec () const throw () {
  return (sec-Time::starting_time.sec)*1000+(usec-Time::starting_time.usec)/1000;
}

long int Tribots::Time::get_sec () const throw () {
  return (sec-Time::starting_time.sec);
}

void Tribots::Time::set (const timeval& tv) throw () {
  sec=tv.tv_sec;
  usec=tv.tv_usec;
}

void Tribots::Time::get (timeval& tv) const throw () {
  tv.tv_sec=sec;
  tv.tv_usec=usec;
}

std::ostream& operator<< (std::ostream& os, const Tribots::Time& tt) throw() {
  os << tt.get_msec();
  return os;
}

  

