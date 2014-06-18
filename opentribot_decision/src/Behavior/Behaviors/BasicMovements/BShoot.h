#ifndef _BSHOOT_H_
#define _BSHOOT_H_

#include "BShootImmediately.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Structures/RobotLocation.h"
#include "../../../Structures/RobotProperties.h"
//#include "../../../Structures/FieldGeometry.h"
#include "../../../Structures/ObstacleLocation.h"

namespace Tribots
{

class BShoot : public Tribots::BShootImmediately
{
public:
  BShoot(int hackKickLength=30);
  virtual ~BShoot() throw();

  virtual bool checkCommitmentCondition(const Time&) throw();
  virtual bool checkInvocationCondition(const Time&) throw();  

  virtual void updateTactics (const TacticsBoard& tb) throw ();

protected:
  //bool check_art_invocation(const Time&);
  //bool check_for_shoot(RobotLocation const& robotLocation, Vec target, LineSegment target_segment, ObstacleLocation const& abs_obstacles);
  bool intersects_target_region(Vec robotpos, Vec new_pos, LineSegment const&  target);

  Time firstTimeLaneFree;

  double  obstacles_behind_goal_distance;
  
};

};

#endif //_BSHOOT_H_
