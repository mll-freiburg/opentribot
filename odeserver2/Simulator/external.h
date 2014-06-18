
#ifndef _TribotsSim_external_h_
#define _TribotsSim_external_h_

#include "World.h"

namespace TribotsSim {

  void help ();   ///< Hilfeausgabe

  void applyRules (World&);  ///< Ball im Aus wieder einsetzen

  void setFormation (World&);  ///< Anfangsformation der Roboter einnehmen

  void keyboardCommand (int);   ///< Tastatur abfragen

}

#endif
