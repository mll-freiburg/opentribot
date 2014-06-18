#ifndef _BKICK_H
#define _BKICK_H

#include "../../Behavior.h"
#include "../../Fundamental/geometry.h"
#include "../../Structures/RobotLocation.h"
#include "../../Structures/RobotProperties.h"
#include "../../Structures/ObstacleLocation.h"

namespace Tribots
{

class BKick : public Tribots::Behavior
{

public:
  BKick();
  virtual ~BKick() throw();

  virtual bool checkCommitmentCondition(const Time&) throw();
  virtual bool checkInvocationCondition(const Time&) throw();  
  virtual DriveVector getCmd(const Time& t) throw(TribotsException);

protected:
  static const double DISTANCE = 3500;
  static const double SCATTER_ANGLE = 10;

  double obstacle_distance_to_cone (Vec s, Vec t, Angle alpha, const ObstacleLocation& obstacles, const Time& t) throw ();
};

};

#endif //_BKICK_H
