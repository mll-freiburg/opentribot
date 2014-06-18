
#ifndef Tribots_VisualContainer_h_
#define Tribots_VisualContainer_h_

#include "../../Structures/VisibleObject.h"

namespace Tribots {

  /** Klasse, um Bildinformationen zu sammeln und zu sortieren */
  class VisualContainer {
  private:
    VisibleObjectList lines;        ///< Liste der weisen Linien
    VisibleObjectList goals;        ///< Liste der Torpunkte
    VisibleObjectList balls;        ///< Liste des Balls
    VisibleObjectList obstacles;    ///< Liste der Hindernisse

  public:
    /** Konstruktor */
    VisualContainer () throw (std::bad_alloc);
    /** Destruktor */
    ~VisualContainer () throw ();

    /** Hinzufuegen von Informationen */
    void add (const VisibleObjectList&) throw (std::bad_alloc);
    /** Hinzufuegen von Informationen */
    void add (const VisibleObject&, Time) throw (std::bad_alloc);

    /** Container loeschen */
    void clear () throw ();

    /** liefert die gespeicherten Liniensegmente */
    const VisibleObjectList& get_lines () const throw ();
    /** liefert die gespeicherten Torpunkte */
    const VisibleObjectList& get_goals () const throw ();
    /** liefert die gespeicherten Ballinformation */
    const VisibleObjectList& get_balls () const throw ();
    /** liefert die gespeicherten Hindernisse */
    const VisibleObjectList& get_obstacles () const throw ();

    /** den Zeitstempel bekommen */
    Time get_timestamp () const throw ();
    /** den Zeitstempel explizit setzen */
    void set_timestamp (Time) throw ();

    /** nur zu Testzwecken, Durcheinander-Mischen der Liniensegmente */
    void shuffle_lines () throw (std::bad_alloc);
    /** nur zu Testzwecken, Entfernen der letzten Liniensegmente */
    void prune_lines (unsigned int) throw ();
  };

}

#endif

