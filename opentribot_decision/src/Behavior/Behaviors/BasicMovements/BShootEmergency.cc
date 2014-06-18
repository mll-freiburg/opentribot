#include "BShootEmergency.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

namespace Tribots
{

using namespace std;

BShootEmergency::BShootEmergency(int hackKickLength) : BShootImmediately("BShootEmergency", hackKickLength)
{}

BShootEmergency::~BShootEmergency() throw()
{}

bool 
BShootEmergency::checkCommitmentCondition(const Time& t) throw()
{
  return BShootEmergency::checkInvocationCondition(t);
}

// ACHTUNG: Bedingungen aus RCPlayerCode übernommen. 
//          Schiesst fast immer ab 1100 mm vor tor, egal ob es frei ist...
bool 
BShootEmergency::checkInvocationCondition(const Time& t) throw()
{
  Time now;         // mit now fragen, da Schuß ziemlich schnell auslöst
  
  if (!BShootImmediately::checkInvocationCondition(now)) { // die müssen
//    LOUT << "Immediately did fail" << endl;
    return false;   // mindestens erfüllt sein (ballbesitz + n zyklen inaktiv)
  }
  
  const RobotLocation& robot = MWM.get_robot_location(now);
  const FieldGeometry& field = MWM.get_field_geometry();
  
  if (robot.pos.y < field.field_length/2. - 1500.) {   // 1300mm von endlinie?
    return false;
  }
  
  if ( - 1500 > robot.pos.x || // zu weit links oder
         1500. < robot.pos.x) { // rechts neben tor?
    return false;
  }

  // nun checken, ob er ins tor schaut (+/- ein paar mm)
  LineSegment goal_segment(Vec(-1100, field.field_length/2), 
                           Vec(1100, field.field_length/2));
  Line shoot_line = Line(robot.pos, WBOARD->getRel2AbsFrame(t)*Vec(0,1));
  std::vector<Vec> res= intersect( goal_segment, shoot_line );
  try {
    if ( res.size() <= 0 ) {
      return false; // schaut nicht aufs tor
    }
  } catch (exception& e) { // parralel?
    return false;
  }

  if (fabs(robot.heading.get_deg_180()) > 70) {
    return false;    // der robbi schaut nicht in richtung der grundlinie
  }                  // bzw. der winkel ist zu spitz

  LOUT << "\% blue circle " << robot.pos.x << " " << robot.pos.y << " 100 \n";
  
  return true;  
}

};
