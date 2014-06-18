
#ifndef Tribots_SPhysGotoPos_h
#define Tribots_SPhysGotoPos_h

#include "../../Skill.h"
#include "../../../Fundamental/geometry.h"
#include <vector>

namespace Tribots {

  /** Faehrt zu einer vorgegebenen Position und haelt dort an.
      Ignoriert Hindernisse.
      Nutzt eine Modellierung mit physikalischem Bewegungsmodell.
      Arbeitet in der Realitaet ordentlich, schwingt im Simulator ueber,
      vor allem bei Rotationen */
  class SPhysGotoPos : public Skill {
  public:
    /** Konstruktor, liest dynamische Eigeschaften (Geschwindigkeiten, Beschleunigungen) aus den RobotProperties */
    SPhysGotoPos () throw ();
    ~SPhysGotoPos () throw ();
    /** Fahrtbefehl erzeugen */
    DriveVector getCmd(const Time&) throw();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung
      Arg3: soll Roboter am Ziel anhalten?
      Arg4: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg5: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden? */
    void init (Vec, Angle, bool, bool =true, bool =true) throw ();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung (=Durchfahrtsrichtung)
      Arg3: soll Roboter am Ziel anhalten?
      Arg4: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg5: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden? */
    void init (Vec, Vec, bool, bool =true, bool =true) throw ();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung
      Arg3: welche Maximalgeschwindigkeit in m/s soll der Roboter am Ziel haben?
      Arg4: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg5: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden? */
    void init (Vec, Angle, double, bool =true, bool =true) throw ();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung (=Durchfahrtsrichtung)
      Arg3: welche Maximalgeschwindigkeit in m/s soll der Roboter am Ziel haben?
      Arg4: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg5: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden? */
    void init (Vec, Vec, double, bool =true, bool =true) throw ();
    /** eine zusaetzliche Barriere hinter dem Zielpunkt definieren, die vom
      Robotermittelpunkt nicht uberschritten werden soll;
      Einstellung beim naechsten Aufruf von "init" verloren. */
    void init_barrier (const Line&) throw ();
    /** eine zusaetzliche Barriere hinter dem Zielpunkt definieren, die vom
      Robotermittelpunkt nicht uberschritten werden soll;
      Einstellung beim naechsten Aufruf von "init" verloren. */
    void init_barrier (const std::vector<Line>&) throw ();
    /** die translatorische (arg1) Hoechstgeschwindigkeit setzen */
    void set_dynamics (double) throw ();
    /** die translatorische (arg1) und rotatorische (arg2) Hoechstgeschwindigkeit setzen */
    void set_dynamics (double, double) throw ();
    /** die dynamischen Eigenschaften setzen:
      Arg1: maximale translatorische Geschwindigkeit in m/s,
      Arg2: maximale rotationale Geschwindigkeit in rad/s,
      Arg3: maximale translatorische Beschleunigung in m/s^2,
      Arg4: maximale rotationale Beschleunigung in rad/s^2 */
    void set_dynamics (double, double, double, double) throw ();
    /** die dynamischen Eigenschaften erfragen; Rueckgabe ueber Argumentreferenzen:
      Arg1: maximale translatorische Geschwindigkeit in m/s,
      Arg2: maximale rotationale Geschwindigkeit in rad/s,
      Arg3: maximale translatorische Beschleunigung in m/s^2,
      Arg4: maximale rotationale Beschleunigung in rad/s^2,
      Arg5: maximale translatorische Bremsverzoegerung in m/s^2,
      Arg6: maximale rotationale Bremsverzoegerung in rad/s^2 */
    void get_dynamics (double&, double&, double&, double&, double&, double&) throw ();
    /** liefert true, wenn (bis auf eine kleine Toleranz) die Zielposition und -Ausrichtung erreicht wurde */
    bool destination_reached (Time) const throw ();
    /** liefert true, wenn (bis auf eine kleine Toleranz) die Zielausrichtung erreicht wurde */
    bool heading_reached (Time) const throw ();
    /** liefert true, wenn (bis auf eine kleine Toleranz) die Zielposition erreicht wurde */
    bool position_reached (Time) const throw ();
  protected:
    // Zielwerte:
    Vec target_pos;  ///< Zielposition
    Angle target_heading;  ///< Zielausrichtung
    double max_target_velocity;  ///< maximale Geschwindigkeit am Zielpunkt
    bool tolerance_pos;  ///< Ungenauigkeiten in der Position am Zielpunkt zulassen?
    bool tolerance_heading;  ///< Ungenauigkeiten in der Ausrichtung am Zielpunkt zulassen?
    std::vector<Line> barrier;  ///< Barrieren, die nicht ueberschritten werden soll

    // Dynamische Eigenschaften:
    double max_tv;  ///< Maximalgeschwindigkeit
    double max_rv;  ///< Maximale Winkelgeschwindigkeit
    double max_ta;  ///< Maximalbeschleunigung
    double max_ra;  ///< maximale Winkelbeschleunigung
    double max_td;  ///< Bremsverzoegerung, mit der gerechnet wird
    double max_rd;  ///< Winkel-Bremsverzoegerung mit der gerechnet wird

    double recent_vrot;  ///< letzte gesetzte Winkelgeschwindigkeit
  };

}

#endif 
