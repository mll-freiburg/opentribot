#include "BStayInsideArea.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"

namespace Tribots {

  BStayInsideArea::BStayInsideArea(const Area& area, Vec target)
    : Behavior("BStayInsideArea"), area(area.clone()), target(target)
  {}

  BStayInsideArea::~BStayInsideArea() throw ()
  {
    delete area;
  }


  bool 
  BStayInsideArea::checkInvocationCondition(const Time& t) throw()
  { 
    return BStayInsideArea::checkCommitmentCondition(t);
  }

  bool 
  BStayInsideArea::checkCommitmentCondition(const Time& t) throw()
  {
    return ! area->is_inside(MWM.get_robot_location(t).pos);
  }

  DriveVector 
  BStayInsideArea::getCmd(const Time& t) throw(TribotsException)
  {  
    DriveVector dv;
    dv.kick = 0;
    dv.vrot = 0;
    dv.vtrans = (WBOARD->getAbs2RelFrame(t) * target).normalize() * 1.0;
    return dv;
  }





}
