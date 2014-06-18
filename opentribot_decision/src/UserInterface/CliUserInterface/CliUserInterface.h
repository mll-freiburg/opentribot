
#ifndef tribots_cli_user_interface_h
#define tribots_cli_user_interface_h

#include "../UserInterfaceType.h"
#include "../../Player/Player.h"

#include "../../WorldModel/WorldModel.h"
#include "../../Fundamental/Time.h"
#include "../../Fundamental/ConfigReader.h"


#include <string>

namespace Tribots {

  using std::string;

  /** einfache MMI auf Grundlage eines CLImit Funktionalitaet zum 
      Starten/Stoppen des Roboters und Beenden des Programms 
      Seiteneffekte: greift auf das Weltmodell zu (lesend und schreibend) */
  class CliUserInterface : public UserInterfaceType {
  public:
    /** Konstruktor */
    CliUserInterface (const ConfigReader&, WorldModel&) throw ();

    /** Destruktor */
    virtual ~CliUserInterface () throw ();

    /** Tastaturabfrage */
    virtual bool process_messages () throw ();

  protected:
    /** Initialen Bildschirm aktualisieren */
    void init_window () throw ();

    /** Bildschirmanzeige aktualisieren */
    void update_window () throw ();


    /** Spielertyp und -rolle weiterschalten */
    void change_player(const char*) const throw ();
    
	/** Spielertyp und -rolle weiterschalten */
    char  readKey_Select();
    fd_set rfds; 


    WorldModel& the_world_model;

    unsigned int single_step_mode;   ///< Einzelschrittmodus, wieviele Schritte noch?
    Time manual_start_timer;         ///< Timer, um ggf. bei manuellem Start verzoegert loszufahren
    bool wait_for_manual_start;      ///< manueller Start aktiviert?
    int manual_start_sec;            ///< bei manuellem Start wie lange warten?
    bool requestedImage;             ///< wurde ein Bild angefragt?
    string debug_image_filename_base;
    int img_no;                      ///< Zaehler fuer mehrere Bilder

    double fps;
	int time_10framesago;
    int framecounter;
    unsigned int update_frequency_redraw;   ///< jeden wievielten Zyklus soll das UserInterface aktualisiert werden?
    unsigned int update_frequency_mode;   ///< jeden wievielten Zyklus soll das UserInterface aktualisiert werden?
    unsigned int update_frequency;   ///< jeden wievielten Zyklus soll das UserInterface aktualisiert werden?
    unsigned int cycle_counter;      ///< Zyklus-Zaehler modulo update_frequency
  };
  
}

#endif

