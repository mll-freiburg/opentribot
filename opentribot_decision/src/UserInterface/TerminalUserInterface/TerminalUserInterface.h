
#ifndef tribots_terminal_user_interface_h
#define tribots_terminal_user_interface_h

#include "../UserInterfaceType.h"
#include "../../Player/Player.h"
#include "../../Robot/Robot.h"
#include "../../WorldModel/WorldModel.h"
#include "../../Fundamental/Time.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../ImageProcessing/Formation/Image.h"
#include "../../ImageProcessing/Vision.h"
#include <curses.h>
#include <string>

namespace Tribots {

  using std::string;

  /** einfache MMI auf Grundlage eines Terminalfensters mit Funktionalitaet zum 
      Starten/Stoppen des Roboters und Beenden des Programms 
      Seiteneffekte: greift auf das Weltmodell zu (lesend und schreibend) */
  class TerminalUserInterface : public UserInterfaceType {
  public:
    /** Konstruktor */
    TerminalUserInterface (const ConfigReader&, Player&, Vision&, WorldModel&, Robot&) throw ();

    /** Destruktor */
    virtual ~TerminalUserInterface () throw ();

    /** Tastaturabfrage */
    virtual bool process_messages () throw ();

  protected:
    /** Initialen Bildschirm aktualisieren */
    void init_window () throw ();

    /** Bildschirmanzeige aktualisieren */
    void update_window () throw ();

    /** Bild speichern */
    void save_image(const Image*, int source, int no) const throw();

    /** Spielertyp und -rolle weiterschalten */
    void change_player(const char*) const throw ();
    
    Player& the_player;
    Vision& the_vision;
    WorldModel& the_world_model;
    Robot& the_robot;
    WINDOW* window;
    unsigned int single_step_mode;   ///< Einzelschrittmodus, wieviele Schritte noch?
    Time manual_start_timer;         ///< Timer, um ggf. bei manuellem Start verzoegert loszufahren
    bool wait_for_manual_start;      ///< manueller Start aktiviert?
    int manual_start_sec;            ///< bei manuellem Start wie lange warten?
    bool requestedImage;             ///< wurde ein Bild angefragt?
    string debug_image_filename_base;
    int img_no;                      ///< Zaehler fuer mehrere Bilder

    unsigned int update_frequency;   ///< jeden wievielten Zyklus soll das UserInterface aktualisiert werden?
    unsigned int cycle_counter;      ///< Zyklus-Zaehler modulo update_frequency
  };
  
}

#endif

