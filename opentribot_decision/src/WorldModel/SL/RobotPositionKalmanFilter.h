
#ifndef Tribots_RobotPositionKalmanFilter
#define Tribots_RobotPositionKalmanFilter

#include "../../Fundamental/Vec.h"
#include "../../Structures/RobotLocation.h"

namespace Tribots {

  /** Implementierung eines Kalman-Filters fuer die Roboterposition und -ausrichtung;
      beruecksichtigt nur die Position, nicht die Geschwindigkeit */
  class RobotPositionKalmanFilter {
  protected:
    Vec pos;                ///< Roboterposition in mm
    double heading;     ///< Ausrichtung in rad; Konvention: Ausrichtung stets im Intervall (-pi, pi]
    Vec var_pos;           ///< Varianz der Roboterposition in mm^2
    double var_heading;      ///< Varianz der Ausrichtung in rad^2
  public:
    /** Setzen einer initialen Position; uebergeben wird:
        arg1: Position in WELTKOORDINATEN
        arg2: Ausrichtung
        arg3: Varianz der Position
        arg4: Varianz der Ausrichtung */
    void set (Vec, Angle, Vec, double) throw ();
    /** Aktualisieren des Filters; uebergeben wird:
        arg1: Verschiebungsvektor laut Odometrie/Fahrtvektoren in ROBOTERKOORDINATEN
        arg2: Ausrichtungsaenderung gemaes Odometrie/Fahrtvektoren
        arg3: Positionsschaetzung aus Bildverarbeitung in WELTKOORDINATEN
        arg4: Ausrichtung aus Bildverarbeitung
        arg5: Varianz der Bildverarbeitungsschaetzung (Position)
        arg6: Varianz der Bildverarbeitungsschaetzung (Ausrichtung)
        arg7: Rotation aus Gyroskop (true) oder Odometrie (false) */
    void update (Vec, Angle, Vec, Angle,Vec,double, bool) throw ();
     /** Fortschreiben des Filters ohne neue Bildinformation; uebergeben wird:
        arg1: Verschiebungsvektor laut Odometrie/Fahrtvektoren in ROBOTERKOORDINATEN
        arg2: Ausrichtungsaenderung gemaes Odometrie/Fahrtvektoren
        arg3: Rotation aus Gyroskop (true) oder Odometrie (false)
        arg4: Soll Verschiebungsvektor auf 3/4 seiner Laenge gekuerzt werden?
              -> sinnvoll nur bei nachfolgendem Abgleich mit Bildinformation */
    void update (Vec, Angle, bool, bool =false) throw ();
   /** Position und Ausrichtung abfragen; Rueckgabe ueber die Argumente:
        arg1: Position in WELTKOORDINATEN
        arg2: Ausrichtung
        Rueckgabewert: Guete der Positionsschaetzung zwischen 0 (schlecht) und 1 (gut) */
    double get (Vec&, Angle&) const throw ();
    /** Die Varianzen fuer die Position liefern (in mm^2) */
    Vec get_position_variance () const throw ();
    /** Die Varianzen fuer die Ausrichtung liefern (in rad^2) */
    double get_heading_variance () const throw ();
    /** Die Position spiegeln */
    void mirror () throw();
  };

}

#endif

