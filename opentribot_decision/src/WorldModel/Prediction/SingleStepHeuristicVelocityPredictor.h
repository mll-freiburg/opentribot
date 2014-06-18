
#ifndef _Tribots_SingleStepHeuristicVelocityPredictor_h_
#define _Tribots_SingleStepHeuristicVelocityPredictor_h_

#include "VelocityPredictor.h"
#include "../../Structures/TribotsException.h"
#include "../Orga/OdometryContainer.h"

namespace Tribots {

  /** Geschwindigkeits-Vorhersage durch fortgesetzte Ein-Schritt-Vorhersage */
  class SingleStepHeuristicVelocityPredictor : public VelocityPredictor {
  public:
    SingleStepHeuristicVelocityPredictor (const ConfigReader&, OdometryContainer&) throw (std::bad_alloc, TribotsException);
    ~SingleStepHeuristicVelocityPredictor () throw ();

    /** eine Positionsschaetzung einbinden (zur Zeit nicht benoetigt),
        Arg1: Zeitpunkt, auf den sich die Schaetzung bezieht,
        Arg2: Positionschaetzung */
    void notify_position (const RobotLocation&, Time) throw ();
    /** einen abgesetzten Fahrtvektor bekannt machen,
        Arg1: Zeitpunkt, zu dem der Fahrtvektor mutmasslich erste Auswirkungen hat,
        Arg2: Fahrtvektor */
    void notify_drive_vector (DriveVector, Time) throw ();
    /** neue Geschwindigkeitsschaetzungen berechnen; arg1: soll debug-Information erzeugt werden? */
    void update () throw ();


  protected:
    const unsigned int max_pred_steps;  // Maximale Anzahl Praediktionsschritte in die Zukunft
    const unsigned int max_store_steps;  // Maximale Anzahl gespeicherte historische Schritte
    unsigned int burn_in;  // >0 am Anfang, wenn noch keine Prognosen gestellt werden koennen

    bool do_predict;  // true, wenn Vorhersagen gemacht werden sollen

    OdometryContainer& odobox;  // die Odobox, an die die Vorhersagen geschickt werden
    RingBuffer<OdometryContainer::TimestampDriveVector> drv;  // Liste der letzten Fahrtvektoren
   
    // temporaer verwendete Strukturen, werden bei jedem update() neu gefuellt
    std::vector<OdometryContainer::TimestampDriveVector> pred;  // Liste der letzten Vorhersagen
 
    double max_lacc;  // maximale lineare Beschleunigung
    double max_ldec;  // maximale lineare Bremsverzoegerung
    double max_aacc;  // maximale Winkelbeschleunigung
    double max_lv;  // maximale Geschwindigkeit
    double max_av;  // maximale Winkelgeschwindigkeit
    double ignore_lv;  // Geschwindigkeiten, unterhalb derer der Roboter nicht reagiert
    double ignore_av;  // Geschwindigkeiten, unterhalb derer der Roboter nicht reagiert

    DriveVector latest_basic_dv;   // Bezugspunkt fuer Schaetzung
    Time latest_basic_timestamp;
    
    void drv2vel (Vec&, double&) const throw (); // rechnet Fahrtvektoren in Geschwindigeiten um (heuristisch) unter Beruecksichtigung der von Schlupf etc.
  };

}

#endif
