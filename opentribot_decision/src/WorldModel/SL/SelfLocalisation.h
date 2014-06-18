
#ifndef _Tribots_SelfLocalisation_h_
#define _Tribots_SelfLocalisation_h_

#include "../../Structures/VisibleObject.h"
#include "../../Structures/RobotLocation.h"

namespace Tribots {

  /** Abstrakte Elternklasse fuer Selbstlokalisierungsmodule;
       ACHTUNG: SL verwendet ein eigenes Koordinatensystem unabhaengig von der Spielrichtung.
       Ursprung ist der Spielfeldmittelpunkt,
       die positive y-Achse weist in Richtung des blauen Tores */
  class SelfLocalisation {
  public:
    virtual ~SelfLocalisation () throw () {;}

    /** Position fortschreiben und mit Bildinformation abgleichen;
        arg1: erkannte weise Linien
        arg2: erkannte Hindernisse
        arg3: erkannte Tore
        ret: wurde die interne Repraesentation tatsaechlich veraendert, d.h. wurde Bildinformation eingearbeitet? */
    virtual bool update (const VisibleObjectList&, const VisibleObjectList& , const VisibleObjectList&) throw () =0;

    /** Roboterposition und -geschwindigkeit fuer den Zeitpunkt arg1 prognostizieren; 
        Qualitaetswert bezieht sich nur auf die Qualitaet des letzten Sensorintegrationsschritts
        Angaben im SL-KOORDINATENSYSTEM */
    virtual RobotLocation get (Time) const throw () =0;

    /** Roboterposition zufaellig reinitialisieren */
    virtual void reset () throw () =0;
    /** Roboterposition an Position arg1 reinitialisieren, Angaben im SL-Koordinatensystem */
    virtual void reset (Vec) throw () =0;
    /** Roboterposition an Position arg1 mit Ausrichtung arg2 reinitialisieren; 
        arg2 misst die Verdrehung des Roboterkoordinatensystems vom Weltkoordinatensystem,
        Angaben im SL-Koordinatensyytem */
    virtual void reset (Vec, Angle) throw () =0;

    /** Hinweis geben fuer SL auf Spielfeldsymmetrie, Arg1: ungefaehre Spielerposition */
    virtual void slMirrorHint (Vec) throw () =0;
  };

}

#endif
