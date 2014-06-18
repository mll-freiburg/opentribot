
#ifndef Tribots_Table_h
#define Tribots_Table_h

#include <stdexcept>

namespace Tribots {

  /** Ein 2-dimensionales Array mit Eintragen vom Typ C */
  template<class C> class Table {
  public:
    /** Konstruktor, Arg1: Anzahl Zeilen, Arg2: Anzahl Spalten */
    Table (unsigned int =1, unsigned int =1) throw (std::bad_alloc);
    /** Copy_konstruktor */
    Table (const Table<C>&) throw (std::bad_alloc);
    /** Destruktor */
    ~Table () throw ();
    const Table<C>& operator= (const Table<C>&) throw (std::bad_alloc);
    /** Groesse der Tabelle aendern (Inhalte gehen verloren), Arg1: Anzahl Zeilen, Arg2: Anzahl Spalten */
    void resize (unsigned int, unsigned int) throw (std::bad_alloc);
    /** Anzahl Zeilen der Tabelle */
    unsigned int rows () const throw ();
    /** Anzahl Spalten der Tabelle */
    unsigned int columns () const throw (); 
    /** Eintrag in Arg1-ter Zeile und Arg2-ter Spalte auswaehlen, ohne Bereichspruefung */
    const C& operator() (unsigned int, unsigned int) const throw ();
    /** Eintrag in Arg1-ter Zeile und Arg2-ter Spalte auswaehlen, ohne Bereichspruefung */
    C& operator() (unsigned int, unsigned int) throw ();
  private:
    unsigned int nr, nc;
    C* array;
  };

}






// Implementierung, wegen template-Deklaration:
template<class C> Tribots::Table<C>::Table (unsigned int r, unsigned int c) throw (std::bad_alloc) : array (NULL) {
  resize (r,c);
}

template<class C> Tribots::Table<C>::~Table () throw () { delete [] array; }

template<class C> void Tribots::Table<C>::resize (unsigned int r, unsigned int c) throw (std::bad_alloc) {
  nr = r;
  nc = c;
  if (array) delete [] array;
  array = new C [nr*nc];
}

template<class C> unsigned int Tribots::Table<C>::rows () const throw () { return nr; }

template<class C> unsigned int Tribots::Table<C>::columns () const throw () { return nc; }

template<class C> const C& Tribots::Table<C>::operator() (unsigned int i, unsigned int j) const throw () { return array[i*nc+j]; }

template<class C> C& Tribots::Table<C>::operator() (unsigned int i, unsigned int j) throw () { return array[i*nc+j]; }

template<class C> const Tribots::Table<C>& Tribots::Table<C>::operator= (const Tribots::Table<C>& src) throw (std::bad_alloc) {
  delete [] array;
  nr = src.nr;
  nc = src.nc;
  array = new C [nr*nc];
  const C* sp = src.array;
  C* dp = array;
  unsigned int n=nr*nc;
  for (unsigned int i=0; i<n; i++)
    (*dp++)=(*sp++);
  return (*this);
}

template<class C> Tribots::Table<C>::Table (const Table<C>& src) throw (std::bad_alloc) {
  nr = src.nr;
  nc = src.nc;
  array = new C [nr*nc];
  const C* sp = src.array;
  C* dp = array;
  unsigned int n=nr*nc;
  for (unsigned int i=0; i<n; i++)
    (*dp++)=(*sp++);
}

#endif

