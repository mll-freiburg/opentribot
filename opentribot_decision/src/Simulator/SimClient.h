
#ifndef _Tribots_SimClient_h_
#define _Tribots_SimClient_h_

#include "../Fundamental/Time.h"
#include "../Fundamental/Vec.h"
#include "../Structures/DriveVector.h"
#include <stdexcept>
#include <vector>


namespace Tribots {

  /** Klasse als Simulatoranbindung, kommuniziert mit dem Simulator */
  class SimClient {
  public:
    /** Aufrufroutine, Singleton; Arg1=Simulator-Konfigurationsdatei */
    static SimClient* get_sim_client (const char*) throw (std::bad_alloc, std::invalid_argument);
    /** entfernt ein ggf. vorhandenes Objekt vom Typ SimClient */
    static void delete_sim_client () throw ();
    /** den Host einstellen (default:localhost). 
        Diese Methode wirkt sich nicht auf bestehende Verbindungen aus, sollte also ggf. 
        vor dem ersten Aufruf von get_sim_client() aufgerufen werden */
    static void set_host (const char*) throw (std::bad_alloc);

  private:
    static SimClient* the_only_sim_client;
    
    /** Konstruktor, privat wegen Singleton */
    SimClient (const char*) throw (std::bad_alloc, std::invalid_argument);
    /** Destruktor */
    ~SimClient () throw ();

    static bool received_anything;  // nur fuer odesim2
    static std::string host;  // nur fuer odesim2
    static unsigned int port;  // nur fuer odesim2
  public:

    /** setzen eines Fahrtvektors */
    void set_drive_vector (DriveVector) throw ();
    /** Kommunizieren mit dem Simulator und Aktualisierung der Weltbeschreibung */
    void update () throw ();

    Time timestamp;        ///< Zeitstempel fuer nachfolgende Attribute
    Vec robot_position;    ///< Roboterposition; Koordinatensystem wie bei Spiel blaues->gelbes Tor
    Angle robot_heading;   ///< Roboterausrichtung; Koordinatensystem wie bei Spiel blaues->gelbes Tor
    Vec ball_position;     ///< Ballposition; Koordinatensystem wie bei Spiel blaues->gelbes Tor
    Vec robot_linear_velocity;
    double robot_angular_velocity;
    std::vector<Vec> obstacle_positions;  ///< Hindernispositionen
    Time latest_obstacle_timestamp;

  };

}

#endif


