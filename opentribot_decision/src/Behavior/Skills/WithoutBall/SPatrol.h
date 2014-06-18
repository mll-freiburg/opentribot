#ifndef _TRIBOTS_SPATROL_H_
#define _TRIBOTS_SPATROL_H_

#include "../../Behavior.h"
#include "../BasicMovements/SGoToPosEvadeObstacles.h"
#include <vector>

namespace Tribots {

  class SPatrol : public Skill {
  public:

    SPatrol(); 
    ~SPatrol() throw();

    virtual void setPatrolPositions(const Vec& pos1, const Vec& pos2);
    virtual void setPatrolPositions(const std::vector<Vec> &positions);
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);

  protected:
    SGoToPosEvadeObstacles* goToPos;
    int aktion;
    Vec pos1, pos2; 
    // patrol many positions
    std::vector<Vec> positions;
    bool forward;
    int positionIndex;
    double patrolSpeed;
    double patrolSpeedToFirstPosition;
    bool firstPosition;
  };

}

#endif 
