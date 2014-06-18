
#ifndef Tribots_Goalie_h
#define Tribots_Goalie_h

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"
#include "../Behavior/Skills/Goalie/SPhysGotoPos.h"


namespace Tribots {

  /** Torwartverhalten, verhaltsbasiert */
  class Goalie : public BehaviorPlayer {
  public:
    Goalie (const ConfigReader&) throw ();
    ~Goalie () throw ();

    /** Dirty Hack: This is only implemented to be able to add some 
     *  milliseconds to the given timestamp.
     *  \todo: Clean this up
     */
    DriveVector process_drive_vector (Time) throw ();
  protected:
    SPhysGotoPos goto_pos_skill;
  };

}

#endif
