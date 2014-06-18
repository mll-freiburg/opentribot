
 
#include "BTestBehavior.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Structures/Journal.h"
#include <vector>
#include "../../../Fundamental/random.h"
#include "../../../Fundamental/RemoteTune.h"

namespace Tribots {
  
  using namespace std;
  BTestBehavior::BTestBehavior()
    : Behavior("BTestBehavior") 
      {
       
      }

  
  BTestBehavior::~BTestBehavior() throw ()
  {
  }
  
  void 
  BTestBehavior::gainControl(const Time& t) throw()
  {
    
  }

  bool 
  BTestBehavior::checkCommitmentCondition(const Time& t) 
    throw()
  {      return true;
  }


  bool 
  BTestBehavior::checkInvocationCondition(const Time& t)
    throw()
  {
	  start.update();
	  return true;
  }
  

  DriveVector BTestBehavior::getCmd(const Time& t) 
  throw(TribotsException)
  {
	  DriveVector dv;
	  double angle=30;
          double speed=2.0;
	  double ms=1000;
	  
	  TUNABLE("BTBANGLE",&angle)
	  TUNABLE("BTBASPEED",&speed)
	  TUNABLE("BTSPEED",&ms)
			  Angle a;
	  a.set_deg(angle);
	  dv.vtrans=Vec(0,speed).rotate(a);
	  
	  if (start.elapsed_msec()>ms) 
		  dv.vtrans=dv.vtrans.rotate(Angle::half); 
	  if (start.elapsed_msec()>2*ms) 
		  dv.vtrans=Vec(0,0); 
  
	  return dv;
	  
  }
  
  
}
