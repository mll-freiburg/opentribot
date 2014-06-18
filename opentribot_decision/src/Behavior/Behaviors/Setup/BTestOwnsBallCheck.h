
#ifndef _Tribots_BTestOwnsBallCheck_h_
#define _Tribots_BTestOwnsBallCheck_h_

#include <deque>
#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"
#include "../../../Structures/GameState.h"

namespace Tribots {

  /** EIn Verhalten, das sich innerhalb einer Spielfeldhaelfte bewegt und stets dem
      Ball nachfaehrt. Es beruecksichtigt dabei Hindernisse. Bei kurzer Distanz zum
      Ball wird der Abstand zum Ball gemessen und aus den gemessenen 
      Abstaenden eine Statistik erstellt */
  class BTestOwnsBallCheck : public Behavior {
  private:
    RefereeState testState;
    int checkHalf;
    SPhysGotoPos gotoPos;
    std::deque<double> distances;
    Time startTime;
    bool wasTestState;

    bool ballInCheckHalf (Time);
    bool ballClose (Time);
    bool wayFree (Time);

  public:
    BTestOwnsBallCheck (RefereeState);
    ~BTestOwnsBallCheck () throw ();
    void cycleCallBack(const Time& t) throw ();
    bool checkInvocationCondition(const Time& t) throw();
    bool checkCommitmentCondition(const Time& t) throw();
    DriveVector getCmd(const Time& t) throw();
    void loseControl (const Time&) throw(TribotsException);
    void gainControl (const Time& t) throw(TribotsException);

    virtual void clearCalibration () throw ();  ///< loescht alle bisherigen Abstandsmessungen
    virtual double getCalibration () throw ();  ///< liefert den Abstand fuer ownsBall oder eine negative Zahl, wenn der Abstand nicht berechnet werden konnte
  };

}

#endif
