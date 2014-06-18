
#include "BVolleyApproach.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/stringconvert.h"
#include "../../../Fundamental/random.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BVolleyApproach::BVolleyApproach () throw () 
  : Behavior ("BVolleyApproach"),
    volley_decision(false)
{
  volley_probability=1;
  volley_skill.init (Vec(0,0.5*MWM.get_field_geometry().field_length));
}

void BVolleyApproach::updateTactics (const TacticsBoard& tb) throw () {
  string attr = tb[string("Volley")];
  double newprob;
  if (string2double (newprob, attr) && newprob>=0 && newprob<=1.0)
    volley_probability=newprob;
}

void BVolleyApproach::cycleCallBack(const Time& t) throw ()
{
  if (MWM.get_game_state().refstate==stopRobot)
    volley_decision = (urandom() < volley_probability);
}

bool BVolleyApproach::checkInvocationCondition (const Time& texec) throw () {
  return volley_decision && checkGeometricCondition(texec, true);
}

bool BVolleyApproach::checkCommitmentCondition (const Time& texec) throw () {
  return checkGeometricCondition (texec, false);
}

bool BVolleyApproach::checkGeometricCondition (const Time& texec, bool strict) throw () {
  Vec ipPoint = volley_skill.getInterceptPoint (texec);
  const FieldGeometry& fg = MWM.get_field_geometry();
  if (abs(ipPoint.x)>0.5*fg.field_width+500 || abs(ipPoint.y)>0.5*fg.field_length+500)
    return false;  // Ball wuerde erst im Aus erreicht
  if (ipPoint.y<-0.3*fg.field_length)
    return false;  // nicht vor dem eigenen Tor in dieser Weise an den Ball ranfahren, zu gefaehrlich
  const BallLocation bloc = MWM.get_ball_location (texec);
  if (!bloc.pos_known==BallLocation::known)
    return false;   // Ballposition unbekannt
  if (bloc.velocity.toVec().length()<0.8 && strict)
    return false;  // langsamen/ruhenden Ball nicht so anfahren
  if (bloc.velocity.toVec().length()<0.5)
    return false;  // langsamen/ruhenden Ball nicht so anfahren
  const RobotLocation rloc = MWM.get_robot_location (texec);
  if (rloc.pos.y>0.5*fg.field_length-fg.goal_area_length)
    return false;  // im Bereich der Goal Area und der Ecken diese Bewegung nicht machen
  Vec ipPointgoal = Vec(0,0.5*fg.field_length)-ipPoint;
  Vec robotball = bloc.pos.toVec()-rloc.pos;
  Vec robotipPoint =ipPoint-rloc.pos;
  Vec robotgoal = Vec(0,0.5*fg.field_length)-rloc.pos;
  if (!ipPointgoal.angle().in_between (Angle::deg_angle(30), Angle::deg_angle(150)))
    return false;  // wann Winkel aufs Tor zu steil, nicht diese Bewegung machen
  if (strict && !ipPointgoal.angle().in_between (Angle::eighth, Angle::three_eighth))
    return false;  // wann Winkel aufs Tor zu steil, nicht diese Bewegung machen
  if (strict && 1.0/(robotgoal.length()+1e-10)*(robotipPoint*robotgoal)<300)
    return false;  // wenn der Roboter rueckwarts fahren muesste, nicht fahren
  if ((robotipPoint*robotgoal)<0)
    return false;  // wenn der Roboter rueckwarts fahren muesste, nicht fahren
  Angle ang1 = bloc.velocity.toVec().angle(robotgoal)-Angle::quarter;
  if (ang1.in_between (Angle::quarter, Angle::three_quarters))
    ang1+=Angle::half;
  if (strict && !ang1.in_between (-Angle::eighth, Angle::eighth))
    return false;  // Ball nicht quergespielt
  if (!ang1.in_between (-Angle::sixth, Angle::sixth))
    return false;  // Ball nicht quergespielt
  if (strict && !rloc.heading.in_between (-Angle::eighth, Angle::eighth))
    return false;  // Roboter steht falsch ausgerichtet

  return true;
}

DriveVector BVolleyApproach::getCmd(const Time& texec) throw () {
  DriveVector dv = volley_skill.getCmd (texec);
  dv.kick=false;
  return dv;
}
