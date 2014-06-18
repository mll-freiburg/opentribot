
#ifndef _Tribots_AddJoystickPlayer_h_
#define _Tribots_AddJoystickPlayer_h_

#include "JoystickPlayer.h"
#include "../Fundamental/Time.h"

namespace Tribots {

  /** Klasse AddJoystickPlayer als zusaetzliche Joystickanbindung zu einer "normalen"
      Playerklasse. D.h. bei GameState::stop_robot werden die Kommandos vom Joystick
      genommen, ansonsten von der normalen Playerklasse. Ausserdem wird Not-Aus am
      Joystick realisiert */
  class AddJoystickPlayer : public JoystickPlayer {
  private:
    PlayerType* the_elementary_player;  ///< die "normale" Playerklasse
    DriveVector remoteCtrDrv;
    unsigned int start_button, stop_button;  // Nummer des Start- und Stop-Knopfes
    unsigned int num_error;
    Time latest_time_nonstop;
  public:
    /** Initialisierung wie bei JoystickPlayer; arg2 ist Zeiger auf "normale" Playerklasse */
    AddJoystickPlayer (const ConfigReader&, PlayerType*) throw (InvalidConfigurationException, std::bad_alloc);
    /** Destruktor */
    ~AddJoystickPlayer () throw ();

    /** Abfrage des Joysticks und Berechnung eines DriveVector */
    DriveVector process_drive_vector (Time) throw ();

    void updateTactics (const TacticsBoard&) throw ();
    const char* get_role () throw ();
    bool set_role (const char*) throw ();
    const std::vector<std::string>& get_list_of_roles () throw ();
  };

}

#endif

