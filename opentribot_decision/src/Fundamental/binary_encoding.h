
#ifndef _Tribots_binary_encoding_h_
#define _Tribots_binary_encoding_h_

#include <iostream>

namespace Tribots {

  void wunsigned3 (std::ostream&, unsigned long int) throw ();
  void wunsigned2 (std::ostream&, unsigned long int) throw ();
  void wunsigned1 (std::ostream&, unsigned long int) throw ();

  inline void wsigned2 (std::ostream& s, long int v) throw () { wunsigned2 (s, static_cast<unsigned long int>(v+0x7fff)); }
  inline void wsigned1 (std::ostream& s, long int v) throw () { wunsigned1 (s, static_cast<unsigned long int>(v+0x7f)); }

  inline void wsigned2 (std::ostream& s, int v) throw () { wunsigned2 (s, static_cast<unsigned long int>(v+0x7fff)); }
  inline void wsigned1 (std::ostream& s, int v) throw () { wunsigned1 (s, static_cast<unsigned long int>(v+0x7f)); }

  inline void wsigned2 (std::ostream& s, double v) throw () { wsigned2 (s, static_cast<long int>(v)); }
  inline void wsigned1 (std::ostream& s, double v) throw () { wsigned1 (s, static_cast<long int>(v)); }

  inline void wunsigned3 (std::ostream& s, double v) throw () { wunsigned3 (s, static_cast<unsigned long int>(v)); }
  inline void wunsigned2 (std::ostream& s, double v) throw () { wunsigned2 (s, static_cast<unsigned long int>(v)); }
  inline void wunsigned1 (std::ostream& s, double v) throw () { wunsigned1 (s, static_cast<unsigned long int>(v)); }

  inline void wunsigned3 (std::ostream& s, unsigned int v) throw () { wunsigned3 (s, static_cast<unsigned long int>(v)); }
  inline void wunsigned2 (std::ostream& s, unsigned int v) throw () { wunsigned2 (s, static_cast<unsigned long int>(v)); }
  inline void wunsigned1 (std::ostream& s, unsigned int v) throw () { wunsigned1 (s, static_cast<unsigned long int>(v)); }


  unsigned long int runsigned3 (std::istream&) throw ();
  unsigned long int runsigned2 (std::istream&) throw ();
  unsigned long int runsigned1 (std::istream&) throw ();

  inline long int rsigned2 (std::istream& is) throw () { return static_cast<long int>(runsigned2(is))-0x7fff; }
  inline long int rsigned1 (std::istream& is) throw () { return static_cast<long int>(runsigned1(is))-0x7f; }

}

#endif
