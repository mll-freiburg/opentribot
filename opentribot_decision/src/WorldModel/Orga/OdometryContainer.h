
#ifndef Tribots_OdometryContainer_h_
#define Tribots_OdometryContainer_h_

#include "../../Structures/DriveVector.h"
#include "../../Structures/GyroData.h"
#include "../../Structures/RobotLocation.h"
#include "../../Fundamental/RingBuffer.h"
#include "../../Fundamental/Time.h"
#include <vector>

namespace Tribots {

  /** Klasse, um Odometrie und Fahrtvektoren einige Zyklen lang aufzubewahren und
      daraus zurueckgelegte Wege zu berechnen; Bei den Berechnungen werden
      geschaetzte Geschwindigkeiten bevorzugt vor Odometrie vor Fahrtvektoren */
  class OdometryContainer {
  public:
    /** Struktur, um Fahrtvektoren mit Zeitstempel abspeichern zu koennen */
    struct TimestampDriveVector {
      Time timestamp;
      DriveVector dv;
    };
    /** Struktur, um Gyrodaten mit Zeitstempel abspeichern zu koennen */
    struct TimestampGyroData {
      Time timestamp;
      GyroData gd;
    };

    /** Konstruktor
        Arg1: Groesse der Puffer
        Arg2: maximale translatorische Beschleunigung
        Arg3: maximale rotatorische Beschleunigung */
    OdometryContainer (unsigned int =20, double =1e6, double =1e6) throw (std::bad_alloc);
    /** Destruktor */
    ~OdometryContainer () throw ();

    /** neue Odometriewerte einarbeiten; Annahme: Information ist neuer als bisherige */
    void add_odometry (DriveVector, Time) throw ();
    /** neuen Fahrtvektor einarbeiten; Annahme: Information ist neuer als bisherige */
    void add_drive_vector (DriveVector, Time) throw ();
    /** neue Gyroskopdaten einarbeiten; Annahme: Information ist neuer als bisherige */
    void add_gyro_data (GyroData, Time) throw ();
    /** Eine Liste mit neuen geschaetzten Geschwindigkeiten in chronologischer Ordnung mitteilen (ersetzt die bisherige Information vollstaendig */
    void set_velocity_estimates (const std::vector<TimestampDriveVector>&) throw ();

    /** Berechne die Bewegung zwischen den Zeitpunkten arg1 und arg2;
        relativ zur Ausgangsposition zum Zeitpunkt arg1;
        mit arg3 kann man einstellen, welche Information verwendet werden
        soll: >=2 verwendet nur Fahrtvektoren, >=1 verwendet Odometrie&Gyro, 
        >=0 verwendet praedizierte Geschwindigkeiten&Gyro */
    RobotLocation movement (Time, Time, unsigned int=0) const throw ();
    /** Berechne die Bewegung zwischen den Zeitpunkten arg2 und arg3;
        und addiere sie zur Ausgangspoition arg1 hinzu;
        mit arg4 kann man einstellen, welche Information verwendet werden
        soll: >=2 verwendet nur Fahrtvektoren, >=1 verwendet Odometrie, 
        >=0 verwendet praedizierte Geschwindigkeiten */
    RobotLocation add_movement (RobotLocation, Time, Time, unsigned int=0) const throw ();
    /** Liefere Odometrie in ROBOTERKOORDINATEN zum Zeitpunkt Arg1 */
    DriveVector get_odometry (Time) const throw ();
    /** Liefere Fahrtvektor in ROBOTERKOORDINATEN zum Zeitpunkt Arg1 */
    DriveVector get_drive_vector (Time) const throw ();
    /** Liefere Gyrodaten ROBOTERKOORDINATEN zum Zeitpunkt Arg1 */
    GyroData get_gyro_data (Time) const throw ();
    /** Liefere geschaetzte/vorhergesagte Geschwindigkeit in ROBOTERKOORDINATEN zum Zeitpunkt Arg1 */
    DriveVector get_velocity_estimate (Time) const throw ();

    RingBuffer<TimestampDriveVector> odo;   ///< Puffer fuer Odometriewerte; Konvention: Anker zeigt immer auf aelteste Information
    RingBuffer<TimestampDriveVector> drv;   ///< Puffer fuer Fahrtvektoren; Konvention: Anker zeigt immer auf aelteste Information
    std::vector<TimestampDriveVector> pred;  ///< Liste mit geschaetzren Robotergeschwindigkeiten in chronologischer Ordnung


  protected:
    RingBuffer<TimestampGyroData> gyro;  ///< Puffer fuer Gyroskopdaten; Konvention: Anker zeigt immer auf aelteste Information
    unsigned int n;        ///< Groesse von odo und drv
    const double max_acc;    ///< maximale translat. Beschleunigung
    const double max_racc;    ///< maximale rotat. Beschleunigung

    double gyro_offset_vrot;    ///< Offset fuer Gyroskop
    std::vector<double> gyro_offset_vrot_candidates;  /// Kandidaten fuer Gyro-Offset, falls Roboter sich nicht in Bewegung setzt
    unsigned int gyro_offset_count;   ///< Zaehler, wie viele Gyrowerte der Roboter bereits still steht
  };

}

#endif

