#ifndef _TRIBOTS_DRIBBLE_TEST_PLAYER_H_
#define _TRIBOTS_DRIBBLE_TEST_PLAYER_H_

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"
#include "../Fundamental/geometry.h"
#include "../Behavior/Skills/BallHandling/SDribbleBallToPos.h"
#include "WhiteBoard.h"
#include "../WorldModel/WorldModel.h"
#include <cmath>
#include <vector>


namespace Tribots {
  
  struct wayPoint {
    Vec pos;
    XYRectangle area;
  };
  
  class BDribbleTest : public Tribots::Behavior
  {
  public:
    BDribbleTest(double transVel = 2.3);
    virtual ~BDribbleTest() throw();

    virtual bool	checkCommitmentCondition(const Time&) throw();
    virtual bool	checkInvocationCondition(const Time&) throw();
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
    virtual void	gainControl(const Time&) throw(TribotsException);
    virtual void	loseControl(const Time&) throw(TribotsException);  
    virtual void	updateTactics (const TacticsBoard& tb) throw ();
    
  protected:
    double		dribblevel;
    SDribbleBallToPos*	skill;
    
    int			timer;
    RobotLocation	robot;
    Frame2d		world2robot, robot2world;
    Vec			target;
    std::vector<wayPoint> wayPoints;
    int			currentWayPoint;
    
    void addWayPoint(Vec);
    
  };

  class DribbleTestPlayer : public BehaviorPlayer {
  public:
    DribbleTestPlayer (const ConfigReader&) throw ();
    ~DribbleTestPlayer () throw ();

  };

}



#endif
