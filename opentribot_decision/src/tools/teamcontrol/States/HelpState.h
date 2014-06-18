
#ifndef _TribotsTools_HelpState_h_
#define _TribotsTools_HelpState_h_

#include <string>
#include <list>
#include "../../../Fundamental/Time.h"

namespace TribotsTools {

  /** Struct, um Pfeile im Teamcontrol anzeigen lassen zu koennen */
  struct DisplayRobotArrow {
    unsigned int source_id;
    unsigned int target_id;
    std::string color;
    std::string text;
    Tribots::Time deadline;
  };
  
  /** Struct, um einem Roboter im Teamcontrol ergaenzenden Text anzeigen lassen zu koennen */
  struct DisplayRobotText {
    unsigned int robot_id;
    std::string color;
    std::string text;
    Tribots::Time deadline;
  };

  /** Datentyp, um hilfreiche Information insbesondere zur Visualisierung abzulegen (nur Daten, die oeffentlich sind) */
  struct HelpState {
    std::list<DisplayRobotArrow> robotarrows;
    std::list<DisplayRobotText> robottext;
  };

}


#endif
