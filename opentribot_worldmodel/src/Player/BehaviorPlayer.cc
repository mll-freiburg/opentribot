#include "BehaviorPlayer.h"
#include "../Behavior/CycleCallBackRegistry.h"
#include "../Structures/TribotsException.h"
#include "../Structures/Journal.h"
#include "../WorldModel/WorldModel.h"
#include "WhiteBoard.h"
#include <iostream>
#include <fstream>
#include <string>

namespace Tribots {

  using namespace std;

  BehaviorPlayer::~BehaviorPlayer() throw ()
  {}

  BehaviorPlayer::BehaviorPlayer(string name, const char *roles[], int size) 
    throw(std::bad_alloc)
    : MultiRolePlayer(roles, size), BDIBehavior(name)
  {
    first_cycle=true; // auf true setzen, wenn eine Behavior-Hierarchie rausgeschrieben werden soll
    setPlayerRole (get_role());
  }

  DriveVector BehaviorPlayer::process_drive_vector(Time time) throw()
  {
    if (first_cycle) {
      first_cycle=false;
      std::string filename = getName();
      filename += ".hierarchy";
      std::ofstream os (filename.c_str());
      printHierarchy (os,0);
      os << flush;
    }
    CycleCallBackRegistry::getRegistry()->callCycleCallBacks(time);
    WBOARD->checkMessageBoard();

    DriveVector dv;
    try {
      dv = getCmd(time);
    } catch (TribotsException& e) {
      JWARNING(e.what());
      dv.vtrans = Vec(0., 0.);
      dv.kick = false;
      dv.vrot = 0.;
    }
    std::string newIntention = getIntentionName();
    MWM.set_active_behavior (newIntention.c_str());
    if (newIntention!=oldIntention){cout << newIntention;
    for (int i=0; i<static_cast<int>(oldIntention.length())-static_cast<int>(newIntention.length()); i++)
      cout << " ";
    cout << "   \n\r";
}
    oldIntention=newIntention;
    return dv;
  }

  void BehaviorPlayer::updateTactics (const TacticsBoard& tb)  throw () {
    this->BDIBehavior::updateTactics (tb);
  }

  bool BehaviorPlayer::set_role (const char* newrole) throw () {
    bool okay = MultiRolePlayer::set_role (newrole);
    if (okay) {
      setPlayerRole (newrole);
      return true;
    } else {
      return false;
    }
  }
}
