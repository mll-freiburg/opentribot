
#include "BGoaliePenaltyRaised.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/random.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoaliePenaltyRaised::BGoaliePenaltyRaised (double pl, double pm, double pr) throw () : Behavior ("BGoaliePenaltyRaised", true) {
  pleft = pl;
  pmiddle = pm;
  pright = pr;
  targetx=0;
}

DriveVector BGoaliePenaltyRaised::getCmd(const Time& texec) throw () {
  const RobotLocation& rloc = MWM.get_robot_location (texec);
  DriveVector ret (Vec(0,0),0,false);
  if (abs(rloc.pos.x-targetx)<100)
    ret.vtrans.x=0;
  else if (rloc.pos.x<targetx)
    ret.vtrans.x=4;
  else
    ret.vtrans.x=-4;
  ret.vtrans/=rloc.heading;  // umrechnen globale->robozentrische Koordinaten
  return ret;
}

bool BGoaliePenaltyRaised::checkInvocationCondition (const Time& t) throw () {
  return (MWM.get_game_state().refstate==opponentPenalty && MWM.get_ball_location (t).pos_known==BallLocation::raised && ball5.y<0.5*MWM.get_field_geometry().field_length-1500 && abs (ball5.x)<500);
}

bool BGoaliePenaltyRaised::checkCommitmentCondition (const Time& t) throw () {
  return checkInvocationCondition(t);
}

void BGoaliePenaltyRaised::gainControl(const Time& t) throw() {
  // entscheide dich fuer eine Ecke
  double r = urandom (0, pleft+pmiddle+pright);
  Angle h = MWM.get_robot_location (t).heading;
  if (r<pleft) {
    targetx=-350;
    LOUT << "GoaliePenaltyRaised: entscheidet sich fuer links\n";
  } else if (r<pleft+pmiddle) {
    targetx=0;
    LOUT << "GoaliePenaltyRaised: entscheidet sich fuer Mitte\n";
  } else {
    targetx=350;
    LOUT << "GoaliePenaltyRaised: entscheidet sich fuer rechts\n";
  }
}

void BGoaliePenaltyRaised::cycleCallBack(const Time& t) throw () {
  // sich die alte Ballposition merken (vor raised)
  if (MWM.get_ball_location (t).pos_known==BallLocation::known) {
    ball5=ball4;
    ball4=ball3;
    ball3=ball2;
    ball2=ball1;
    ball1=MWM.get_ball_location (t).pos.toVec();
  }
}
