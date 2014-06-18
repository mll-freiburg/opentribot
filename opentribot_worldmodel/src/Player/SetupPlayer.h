
#ifndef _Tribots_SetupPlayer_h_
#define _Tribots_SetupPlayer_h_

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"
#include "../Behavior/Behaviors/Setup/BTestOwnsBallCheck.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/Setup/BNewDribbleTest.h"


namespace Tribots {

   /** Behavior, das den Roboter stoppt und das an- und ausgeschaltet werden kann */
  class BStopWhenRequested : public BEmergencyStop {
    bool active;
  public:
    BStopWhenRequested () throw ();
    ~BStopWhenRequested () throw ();
    bool checkInvocationCondition(const Time&) throw();
    bool checkCommitmentCondition(const Time&) throw();
    virtual void setActive (bool b);
  };

  /** SetupPlayer fuer Kalibrier- und Testverhalten vor dem Spiel */
  class SetupPlayer : public BehaviorPlayer {
  public:
    SetupPlayer (const ConfigReader&) throw ();
    ~SetupPlayer () throw ();

    bool set_role (const char*) throw ();
  private:
    BTestOwnsBallCheck* bOwnsBallCheck;
    BStopWhenRequested* bStop;
    std::string cfgFilename;
    double ownsBallDistance;
    double oldOwnsBallDistance;
  };

}

#endif
