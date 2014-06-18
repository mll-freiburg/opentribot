
#ifndef Tribots_DemoGoalie_h
#define Tribots_DemoGoalie_h

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"


namespace Tribots {

  /** Torwartverhalten, vereinfacht als Sparring-Partner */
  class DemoGoalie : public BehaviorPlayer {
  public:
    DemoGoalie (const ConfigReader&) throw ();
    ~DemoGoalie () throw ();

    /** Dirty Hack: This is only implemented to be able to add some 
     *  milliseconds to the given timestamp.
     *  \todo: Clean this up
     */
    DriveVector process_drive_vector (Time) throw ();
  };

}

#endif
