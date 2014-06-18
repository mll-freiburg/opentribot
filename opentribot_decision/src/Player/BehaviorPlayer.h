#ifndef _BEHAVIORPLAYER_H_
#define _BEHAVIORPLAYER_H_

#include "MultiRolePlayer.h"
#include "../Fundamental/ConfigReader.h"
#include "../Behavior/Behavior.h"
#include "../Behavior/BDIBehavior.h"

namespace Tribots {

  class BehaviorPlayer : public MultiRolePlayer, public BDIBehavior {
  public:
    virtual ~BehaviorPlayer() throw();
    virtual DriveVector process_drive_vector(Time) throw();
    virtual void updateTactics (const TacticsBoard&)  throw ();
    virtual bool set_role (const char*) throw ();

  protected:
    BehaviorPlayer(std::string name, const char *roles[] = 0, int size = 0)
      throw(std::bad_alloc);
  private:
    bool first_cycle;
    std::string oldIntention;
  };

}

#endif
