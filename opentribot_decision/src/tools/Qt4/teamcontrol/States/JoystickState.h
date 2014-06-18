
#ifndef TribotsTools_JoystickState_h
#define TribotsTools_JoystickState_h

#include <string>

namespace TribotsTools {

  /** Datentyp, um Joystickdaten zu verwalten */
  struct JoystickState {
    std::string device_name;                    ///< Name des Devices

    bool joystick_okay;                         ///< Joystick ansprechbar?

    unsigned int kick_button;                   ///< Nummer des Kick-Knopfes
    unsigned int x_axis;                        ///< Nummer der x-Achse
    unsigned int y_axis;                        ///< Nummer der y-Achse
    unsigned int phi_axis;                      ///< Nummer der phi-Achse

    unsigned int activate_button;               ///< Nummer des Aktivieren-Knopfes
    unsigned int deactivate_button;             ///< Nummer des Deaktivieren-Knopfes

    int sign_x;                                 ///< Vorzeichen der x-Achse
    int sign_y;                                 ///< Vorzeichen der y-Achse
    int sign_phi;                               ///< Vorzeichen der phi-Achse

    double max_vx;                              ///< Maximalgeschwindigkeit x (m/s)
    double max_vy;                              ///< Maximalgeschwindigkeit y (m/s)
    double max_vphi;                            ///< Maximalgeschwindigkeit phi (rad/s)

    double null_x;                              ///< Nullbereich x
    double null_y;                              ///< Nullbereich y
    double null_phi;                            ///< Nullbereich phi

    double vx;                                  ///< angesteuerte Geschwindigkeit x (m/s)
    double vy;                                  ///< angesteuerte Geschwindigkeit y (m/s)
    double vphi;                                ///< angesteuerte Geschwindigkeit phi (rad/s)
    bool kick;                                  ///< angesteuerter Kick-Befehl
    unsigned int kick_length;                   ///< angesteuerte Dauer der Ausloesung des kickers in ms

    int perspective;                            ///< -1, wenn robozentrisch, ansonsten Winkel in Grad, auf die sich die Steuerung bezieht

    bool activation_request;                    ///< wird Aktivierung der Roboter gewuenscht?
    bool deactivation_request;                  ///< wird Deaktivierung der Roboter gewuenscht?

    JoystickState () :
      device_name ("/dev/input/js0"),
      joystick_okay (false),
      kick_button (5),
      x_axis (0),
      y_axis (1),
      phi_axis (3),
      activate_button (0),
      deactivate_button (1),
      sign_x (1),
      sign_y (-1),
      sign_phi (-1),
      max_vx (3),
      max_vy (3),
      max_vphi (3),
      null_x (0.0),
      null_y (0.0),
      null_phi (0.0),
      vx (0),
      vy (0),
      vphi (0),
      kick(false),
      kick_length(100),
      perspective (-1),
      activation_request(false),
      deactivation_request(false) {;}

  };

}


#endif
