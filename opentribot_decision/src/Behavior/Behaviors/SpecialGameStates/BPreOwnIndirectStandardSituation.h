#ifndef _TRIBOTS_BPREOWNINDIRECTSTANDARDSITUATION_H_
#define _TRIBOTS_BPREOWNINDIRECTSTANDARDSITUATION_H_

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"
#include "../ApproachingBall/BApproachBallForStandardSituation.h"
#include "../BasicMovements/BEmergencyStop.h"
#include "../../../Fundamental/geometry.h"
#include <vector>
#include "../../Skills/ApproachingBall/SApproachMovingBall.h"
#include "../BallHandling/BStuckOwnsBall.h"


namespace Tribots {

  class BPreOwnIndirectStandardSituation : public Behavior {
  public:

    BPreOwnIndirectStandardSituation(int, int, int, double, double, int);
    ~BPreOwnIndirectStandardSituation() throw();

    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual bool checkInvocationCondition(const Time&) throw();
    
    virtual void gainControl(const Time& t) throw();
    virtual int getAreaID(const Time& t);
    virtual XYRectangle getAreaForAreaID(int);
    virtual void areasPrintln();
    virtual void areaPrintln(XYRectangle);
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
    
    void updateTactics (const TacticsBoard& tb) throw(); 

  protected:
    BStuckOwnsBall stuckBehavior;
    SApproachMovingBall goToBall;
    SPhysGotoPosAvoidObstacles* goToPos;
    BApproachBallForStandardSituation *approach;
    BEmergencyStop* stop;
    DriveVector block(const Time& t, int pos);
    DriveVector safeGuard(const Time& t);
    DriveVector blockGegnerFromBall(const Time& t);
    DriveVector gotoVolleyPosition(const Time& t);
    DriveVector approachBall(const Time& t);
    Vec volleyPosition(Time t);
    enum { POS_LEFT=1, POS_RIGHT };  
    

  private:
    std::vector<XYRectangle> areas;

    bool stuck;
    bool reached;
    bool doStandard;
    bool kickPosition;
    bool atPosition;
    bool finished;
    bool _kick;
    bool _decided;
    bool _ownTurn;
    bool passFirstPhase;
    bool passMessageSent;
    int counter;
    int area;
    int originalRefstate;
    Vec kickerEvadePosition;
    bool checkedCorridor;
    bool breakKickProcedure;
    bool goalKickDirect;
    bool ownHalfAllDirect;

    Time start;

    int shortPassKickDuration;
    int longPassKickDuration;
    double standardPassProbability;
    double standardDribbling;
    int standardPositioning;
    int standardAbstandBlock;

    Vec getTargetForFreePlayer(const Time& t); 
    Vec getPositionForTwoPlayer(const Time& t); 
    DriveVector getCmdBefore(const Time&) throw(TribotsException); // the kicking and the receiving players
    DriveVector getCmdAfter(const Time&) throw(TribotsException); // the kicking player
    
    Vec dangerousOpponentPosition;
    bool doVolley;
    double volleyProbability;
    bool doDynamicBlock;
    int throwInKickDuration;
  };

}

#endif 
