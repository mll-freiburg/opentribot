
#ifndef _Tribots_FataMorgana_h_
#define _Tribots_FataMorgana_h_

#include "Types/WorldModelTypeBase.h"

namespace Tribots {

  /** Abstrakte Fata-Morgana Klasse. Dient dazu, bei Benchmarks kuenstliche
      Hindernisse oder Baelle im Weltmodell einfuegen zu koennen.
      Funktionsweise:
        Beim Weltmodell koennen FataMorgana-Objekte angemeldet werden
        (Methode: add_fata_morgana)
        In jeder Iteration ruft das Weltmodell die update-Methoden aller
        angemeldeter FataMorgana-Objekte auf. Diese koennen mit Hilfe des
        uebergebenen Zeigers und den Methodenaufrufen 'add_ball_relative',
        'add_ball_absolute', 'add_obstacle_relative' und
        'add_obstacle_absolute' zusaetzliche Baelle/Hindernisse in globalen
        oder robozentrischen Koordinaten eintragen.
      Der FataMorgana-Mechanismus kann mit dem Config-Eintrag 'tournament_mode'
      ein- (false) und ausgeschaltet (true) werden */
  class FataMorgana {
  public:
    virtual void update (WorldModelTypeBase*) throw () =0;
    virtual ~FataMorgana () {;}
  };

}

#endif
