
#ifndef _Tribots_SingleStepNeuralVelocityPredictor_h_
#define _Tribots_SingleStepNeuralVelocityPredictor_h_

#include "VelocityPredictor.h"
#include "../../Structures/TribotsException.h"
#include "../Orga/OdometryContainer.h"
#include "../SL/SLVelocitySensor.h"
#include "../../Libs/n++/include/n++.h"
#include <fstream>

namespace Tribots {

  /** Geschwindigkeits-Vorhersage durch fortgesetzte Ein-Schritt-Vorhersage */
  class SingleStepNeuralVelocityPredictor : public VelocityPredictor {
  public:
    SingleStepNeuralVelocityPredictor (const ConfigReader&, OdometryContainer&) throw (std::bad_alloc, TribotsException);
    ~SingleStepNeuralVelocityPredictor () throw ();

    /** eine Positionsschaetzung einbinden,
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
    struct TRL { // interne Struktur zum Zwischenspeichern
      Time timestamp;
      RobotLocation rloc;
    };
    struct TP { // interne Struktur zum Zwischenspeichern
      TP ();
      TP (const TP&) throw ();
      ~TP () throw ();
      Time timestamp;
      FTYPE* xpat;
      FTYPE* ypat;
      FTYPE* phipat;
      const TP& operator= (const TP&) throw ();
    };
    
    const unsigned int max_pred_steps;  // Maximale Anzahl Praediktionsschritte in die Zukunft
    const unsigned int max_store_steps;  // Maximale Anzahl gespeicherte historische Schritte
    unsigned int burn_in;  // >0 am Anfang, wenn noch keine Prognosen gestellt werden koennen

    bool save_patterns;  // true, wenn Trainingsmuster gespeichert werden sollen
    bool do_predict;  // true, wenn Vorhersagen gemacht werden sollen

    OdometryContainer& odobox;  // die Odobox, an die die Vorhersagen geschickt werden
    RingBuffer<OdometryContainer::TimestampDriveVector> drv;  // Liste der letzten Fahrtvektoren
    RingBuffer<TRL> pos;  // Liste der letzten Positionsschaetzungen
    Net* net_vx;  // Prognosenetz fuer Geschwindigkeit in x-Richtung
    Net* net_vy;  // Prognosenetz fuer Geschwindigkeit in y-Richtung
    Net* net_vphi;  // Prognosenetz fuer Winkelgeschwindigkeit
    
    // temporaer verwendete Strukturen, werden bei jedem update() neu gefuellt
    std::vector<OdometryContainer::TimestampDriveVector> pred;  // Liste der letzten Vorhersagen
    SLVelocitySensor velo5;  // Schaetzer mit Fensterlaenge 5
    SLVelocitySensor velo10;  // Schaetzer mit Fensterlaenge 10
    TP pattern;
    
    // Im Mustergenerierungsmodus:
    std::ofstream* patstream_x;
    std::ofstream* patstream_y;
    std::ofstream* patstream_phi;
    RingBuffer<TP> recent_patterns;
    unsigned int save_burn_in;
  };

}

#endif
