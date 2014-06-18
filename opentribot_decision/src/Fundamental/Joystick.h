
#ifndef _Tribots_Joystick_h_
#define _Tribots_Joystick_h_

#include <vector>
#include <stdexcept>
#include <string>


namespace Tribots {


   struct JoystickState{
   double axis[8];
   bool buttons[20];
   JoystickState(){
   for (int i=0; i<8;i++)axis[i]=0;
   for (int i=0; i<20;i++)buttons[i]=0;
   
   }
   };

  /** Klasse, um einen Joystick anzubinden, API zu linux/joystick.h */
  class Joystick {
  private:
  public:
    void update () throw ();    ///< Joystick Events abfragen
    int file_descriptor;        ///< File Deskriptor Geraetedatei
    std::vector<double> axis;   ///< Zustand der Achsen (im Intervall [-1,1])
    std::vector<bool> button;   ///< Zustand der Knoepfe; true=gedrueckt
    /** Joystick initialisieren, uebergeben wird der Name der Geraetedatei (z.B. /dev/input/js0) */
    Joystick (const char*) throw (std::invalid_argument, std::bad_alloc);
    /** Destruktor */
    ~Joystick () throw ();
    /** Joystick Typbeschreibung */
    std::string get_type () throw (std::bad_alloc);
    /** Versionsnummer */
    int get_version () throw ();

    /** Joystick abfragen: Achsen */
    const std::vector<double>& get_axis_state () throw ();
    /** Joystick abfragen: Knoepfe */
    const std::vector<bool>& get_button_state () throw ();
  };

}

#endif

