
#ifndef _Tribots_JoystickPlayer_h_
#define _Tribots_JoystickPlayer_h_

#include <stdexcept>
#include "SingleRolePlayer.h"
#include "../Fundamental/ConfigReader.h"

#include "../Fundamental/Joystick.h"
#include "../Communication/UDPSocket.h"
#include "../Structures/TribotsException.h"


namespace Tribots {

  /** Joystickanbindung, realisiert als Spielertyp */
  class JoystickPlayerUDP : public SingleRolePlayer {
  protected:
    Joystick *js;
  private:
    JoystickState jstate;
    UDPSocket* serversocket;
    double max_velocity;
    double max_rot_velocity;
    bool currently_changed;
    bool previously_changed;

    unsigned int x_axis, y_axis, phi_axis;     // Nummer der Joystick-Achse, um x,y,phi-Bewegung zu steuern
    double x_diraxis, y_diraxis, phi_diraxis;     // Orientierung der Joystick-Achsen (wo ist +/-)
    unsigned int kick_button, accel_button, decel_button;  // Nummer der Knoepfe, um zu kicken, Geschwindigkeit erhoehen/verringern
  public:
    /** Initialisierung, Name des Devices wird aus dem ConfigReader entnommen */
    JoystickPlayerUDP (const ConfigReader&) throw (InvalidConfigurationException, std::bad_alloc);
    /** Destruktor */
    ~JoystickPlayerUDP () throw ();

    /** Abfrage des Joysticks und Berechnung eines DriveVector */
    DriveVector process_drive_vector (Time) throw ();
  };

}

#endif

