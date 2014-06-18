
#ifndef tribots_time_h
#define tribots_time_h

#include <iostream>
#include <sys/time.h>

namespace Tribots {

  /** Klasse zur Repraesentation von Zeitpunkten */
  class Time {
  private:
    long int sec;   ///< Anzahl Sekunden
    long int usec;  ///< Anzahl Mikrosekunden

    static const Time starting_time;   ///< Startzeit des Programms
  public:
    /** Konstruktor: setze aktuelle Systemzeit */
    Time () throw ();
    /** Copy-Konstruktor */
    Time (const Time&) throw ();
    /** Setzen der Zeit aus timeval */
    Time (const timeval&) throw ();
    /** Zuweisungsoperator */
    const Time& operator= (const Time&) throw ();

    /** Zeit in usec seit dem Programmstart */
    long int get_usec () const throw ();
    /** Zeit in msec seit dem Programmstart */
    long int get_msec () const throw ();
    /** Zeit in sec seit dem Programmstart */
    long int get_sec () const throw ();

    /** Zeit in usec seit Programmstart setzen */
    void set_usec (const long int&) throw ();
    /** Zeit in msec seit Programmstart setzen */
    void set_msec (const long int&) throw ();
    /** Zeit in sec seit Programmstart setzen */
    void set_sec (const long int&) throw ();

    /** aktuelle Systemzeit setzen */
    void update () throw ();

    /** Differenz zwischen jetzt und der gespeicherten Zeit in usec */
    long int elapsed_usec () const throw ();
    /** Differenz zwischen jetzt und der gespeicherten Zeit in msec */
    long int elapsed_msec () const throw ();
    /** Differenz zwischen jetzt und der gespeicherten Zeit in sec */
    long int elapsed_sec () const throw ();

    /** Addieren von Arg1 Mikrosekunden */
    void add_usec (long int) throw ();
    /** Addieren von Arg1 Millisekunden */
    void add_msec (long int) throw ();
    /** Addieren von Arg1 Sekunden */
    void add_sec (long int) throw ();

    /** Zeit-Differenz in Mikrosekunden (*this)-Arg1 */
    long int diff_usec (const Time&) const throw ();
    /** Zeit-Differenz in Millisekunden (*this)-Arg1 */
    long int diff_msec (const Time&) const throw ();
    /** Zeit-Differenz in Sekunden (*this)-Arg1 */
    long int diff_sec (const Time&) const throw ();

    /** Vergleichsoperator == */
    bool operator== (const Time&) const throw ();
    /** Vergleichsoperator != */
    bool operator!= (const Time&) const throw ();
    /** Vergleichsoperator <= */
    bool operator<= (const Time&) const throw ();
    /** Vergleichsoperator < */
    bool operator< (const Time&) const throw ();
    /** Vergleichsoperator >= */
    bool operator>= (const Time&) const throw ();
    /** Vergleichsoperator > */
    bool operator> (const Time&) const throw ();

    /** explizites Setzen der Zeit mit einem Linux timeval */
    void set (const timeval&) throw ();
    /** explizites Holen der Zeit als Linux timeval */
    void get (timeval&) const throw ();
  };

}

std::ostream& operator<< (std::ostream&, const Tribots::Time&) throw();   ///< Zeit seit Programmstart in msec


#endif

