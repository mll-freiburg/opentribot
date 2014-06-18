
#ifndef Tribots_SPhysGotoPosAvoidObstacles_h
#define Tribots_SPhysGotoPosAvoidObstacles_h

#include "../../Skill.h"
#include "SPhysGotoPos.h"
#include "../../../Fundamental/geometry.h"
#include <vector>

namespace Tribots {

  /** Faehrt zu einer vorgegebenen Position und haelt dort an.
      Umfaehrt Hindernisse und den Ball */
  class SPhysGotoPosAvoidObstacles : public Skill {
  public:
    /** Konstruktor, liest dynamische Eigeschaften (Geschwindigkeiten, Beschleunigungen) aus den RobotProperties,
        wenn arg1 angegeben wird, wird dieses Objekt der Klasse SPhysGotoPos verwendet anstatt ein neues Objekt
        zu erzeugen */
    SPhysGotoPosAvoidObstacles (SPhysGotoPos* = NULL) throw ();
    ~SPhysGotoPosAvoidObstacles () throw () {;}
    /** Fahrtbefehl erzeugen */
    DriveVector getCmd(const Time&) throw();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung
      Arg3: soll Roboter am Ziel anhalten?
      Arg4: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg5: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden?
      Arg6: soll bei stuck eine Ausweichbewegung erfolgen? */
    void init (Vec, Angle, bool, bool =true, bool =true, bool =true) throw ();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung (=Durchfahrtsrichtung)
      Arg3: soll Roboter am Ziel anhalten?
      Arg4: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg5: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden?
      Arg6: soll bei stuck eine Ausweichbewegung erfolgen? */
    void init (Vec, Vec, bool, bool =true, bool =true, bool =true) throw ();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung
      Arg3: welche Maximalgeschwindigkeit in m/s soll der Roboter am Ziel haben?
      Arg4: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg5: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden?
      Arg6: soll bei stuck eine Ausweichbewegung erfolgen? */
    void init (Vec, Angle, double, bool =true, bool =true, bool =true) throw ();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung (=Durchfahrtsrichtung)
      Arg3: welche Maximalgeschwindigkeit in m/s soll der Roboter am Ziel haben?
      Arg4: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg5: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden?
      Arg6: soll bei stuck eine Ausweichbewegung erfolgen? */
    void init (Vec, Vec, double, bool =true, bool =true, bool =true) throw ();
    /** eine zusaetzliche Barriere hinter dem Zielpunkt definieren, die vom
        Robotermittelpunkt nicht uberschritten werden soll;
        Einstellung beim naechsten Aufruf von "init" verloren. */
    void init_barrier (const Line&) throw ();
    /** eine zusaetzliche Barriere hinter dem Zielpunkt definieren, die vom
        Robotermittelpunkt nicht uberschritten werden soll;
        Einstellung beim naechsten Aufruf von "init" verloren. */
    void init_barrier (const std::vector<Line>&) throw ();
    /** Ball als Hindernis betrachten (arg1=true) oder ignorieren (arg1=false), default=true;
        arg2=true sorgt dafuer, dass in Ballnaehe nicht gedreht wird, also besonders vorsichtig
        gefahren wird.
        Einstellung bleibt bis zum naechsten Aufruf von set_ball_as_obstacle erhalten */
    void set_ball_as_obstacle (bool, bool=false) throw ();
    /** die translatorische Hochstgeschwindigkeit setzen */
    void set_dynamics (double) throw ();
    /** die translatorische (arg1) und rotatorische (arg2) Hochstgeschwindigkeit setzen */
    void set_dynamics (double, double) throw ();
    /** die dynamischen Eigenschaften setzen: 
      Arg1: maximale translatorische Geschwindigkeit in m/s,
      Arg2: maximale rotationale Geschwindigkeit in rad/s,
      Arg3: maximale translatorische Beschleunigung in m/s^2,
      Arg4: maximale rotationale Beschleunigung in rad/s^2 */
    void set_dynamics (double, double, double, double) throw ();
    /** die Richtung, in die bei besetztem Zielpunkt ausgewichen werden soll,
      Arg1: Richtung (in Weltkoordinatensystem), in die ausgewichen werden soll */
    void set_target_evade_strategy (Angle) throw ();
    /** die dynamischen Eigenschaften erfragen; Rueckgabe ueber Argumentreferenzen:
      Arg1: maximale translatorische Geschwindigkeit in m/s,
      Arg2: maximale rotationale Geschwindigkeit in rad/s,
      Arg3: maximale translatorische Beschleunigung in m/s^2,
      Arg4: maximale rotationale Beschleunigung in rad/s^2,
      Arg5: maximale translatorische Bremsverzoegerung in m/s^2,
      Arg6: maximale rotationale Bremsverzoegerung in rad/s^2 */
    void get_dynamics (double&, double&, double&, double&, double&, double&) throw ();
    /** der Roboter soll versuchen, die Zielposition auch dann anzufahren, wenn sie
      durch ein Hindernis belegt ist. Ggf.wird geschoben */
    void force_target () throw ();
    /** liefert true, wenn (bis auf eine kleine Toleranz) die Zielposition und -Ausrichtung erreicht wurde */
    bool destination_reached (Time) const throw ();
    /** liefert true, wenn (bis auf eine kleine Toleranz) die Zielausrichtung erreicht wurde */
    bool heading_reached (Time) const throw ();
    /** liefert true, wenn (bis auf eine kleine Toleranz) die Zielposition erreicht wurde */
    bool position_reached (Time) const throw ();

  private:
    // Zielwerte:
    Vec target_pos;
    Angle target_heading;
    double max_target_velocity;
    bool tolerance_pos;
    bool tolerance_heading;
    bool stuckmove; // true, wenn bei stuck in die entgegengesetzte Richtung gefahren werden soll
    bool carefulballmove;  // true, wenn bei Ballnaehe nicht gedreht werden soll

    Vec stop_over_point;
    Time stop_over_time;
    double max_stop_over_velocity;
    std::vector<Line> barrier;

    Vec evade_dir;   // Normalenvektor in Ausweichrichtung (robozentrisch), oder Nullverktor fuer force_target

    double max_tv;
    double robot_width;
    SPhysGotoPos* goto_pos_skill;
    SPhysGotoPos own_goto_pos_skill;

    bool consider_ball_as_obstacle;
    std::vector<Angle> search_directions;  // nur fuer interne Zwecke
    
    std::vector<Line> additional_barriers;
  };

}

#endif 
