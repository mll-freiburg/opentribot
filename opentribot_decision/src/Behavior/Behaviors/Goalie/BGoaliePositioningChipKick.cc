
#include "BGoaliePositioningChipKick.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/stringconvert.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoaliePositioningChipKick::BGoaliePositioningChipKick (SPhysGotoPos* sp) throw () : Behavior ("BGoaliePositioningChipKick") {
  if (sp)
    goto_pos_skill = sp;
  else
    goto_pos_skill = &own_goto_pos_skill;
  const FieldGeometry& fg (MWM.get_field_geometry());
  home.y = -0.5*fg.field_length;
  maxextend = 0.5*fg.goal_width-150;
  workingArea = XYRectangle (Vec(-0.5*fg.goal_width+50, -0.5*fg.field_length-100), Vec(0.5*fg.goal_width-50, -0.5*fg.field_length+1500));
  setOpponentHalf(0);
  wasInToleranceArea2=false;
  wasInToleranceArea2TwoTimes=false;
}

void BGoaliePositioningChipKick::setOpponentHalf (double y) throw () {
  const FieldGeometry& fg (MWM.get_field_geometry());
  opponentHalf = XYRectangle (Vec(-0.5*fg.field_width-500, y), Vec(0.5*fg.field_width+500, 0.5*fg.field_length+500));
  ownHalf = XYRectangle (Vec(-0.5*fg.field_width-500, -0.5*fg.field_length-500), Vec(0.5*fg.field_width+500, y));
  ownHalfFront = XYRectangle (Vec(-0.5*fg.field_width-500, y-1500), Vec(0.5*fg.field_width+500, y));
  toleranceArea1 = XYRectangle (Vec(-0.5*fg.field_width-500, y), Vec(0.5*fg.field_width+500, y+800));
  toleranceArea2 = XYRectangle (Vec(-0.5*fg.field_width-500, y+800), Vec(0.5*fg.field_width+500, y+1200));
}

void BGoaliePositioningChipKick::gainControl (const Time&) throw(TribotsException) {
  goto_pos_skill->set_dynamics (2.0, 2.0, 5.0, 8.0);
  tghead = Angle::zero;
  ballSeenInOwnHalf = false;
  numBallSeenInOpponentHalf = 0;
}

bool BGoaliePositioningChipKick::checkCondition (const Time& texec) throw () {
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));
  bool ball_known = (ball_exec.pos_known == BallLocation::known ||  ball_exec.pos_known == BallLocation::communicated);
  return ( (ball_known && opponentHalf.is_inside (ball_exec.pos.toVec()) || 
          texec.diff_msec (ballSeenInOpponentHalf)<1800) ) && 
          workingArea.is_inside (robot_exec.pos);
}

bool BGoaliePositioningChipKick::checkInvocationCondition (const Time& texec) throw () {
  // wasInToleranceArea2, wasInToleranceArea2TwoTimes wird in cycleCallBack gesetzt
  return checkCondition (texec) && 
    !toleranceArea1.is_inside (MWM.get_ball_location(texec).pos.toVec()) && 
    (wasInToleranceArea2TwoTimes || !toleranceArea2.is_inside (MWM.get_ball_location(texec).pos.toVec()));
}

bool BGoaliePositioningChipKick::checkCommitmentCondition (const Time& texec) throw () {
  return checkCondition (texec);
}

DriveVector BGoaliePositioningChipKick::getCmd(const Time& texec) throw () {
  const BallLocation& ball_exec (MWM.get_ball_location (texec));
  bool ball_own_known = (ball_exec.pos_known == BallLocation::known);
  bool ball_known = ball_own_known || (ball_exec.pos_known == BallLocation::communicated);
  Vec ball_pos = ball_exec.pos.toVec();
  
  if (ball_known && opponentHalf.is_inside (ball_pos)) {
    ballSeenInOpponentHalf=texec;
    latestBallPositionInOpponentHalf=ball_pos;
    tghead = (latestBallPositionInOpponentHalf-home).angle()-Angle::quarter;
    if (tghead.in_between (Angle::deg_angle(15), Angle::half))
      tghead = Angle::deg_angle(15);
    if (tghead.in_between (Angle::half, Angle::deg_angle(-15)))
      tghead = Angle::deg_angle(-15);
    numBallSeenInOpponentHalf++;
    if (numBallSeenInOpponentHalf>=5) {
      ballSeenInOwnHalf=false; // um versehentlich gemerkte Baelle zu loeschen
    }
    numInOwnHalf=0;
    numInOwnHalfFront=0;
  }
  if (ball_known && ownHalfFront.is_inside(ball_pos)) {
    numInOwnHalfFront++;
  }
  if (ball_known && ownHalf.is_inside(ball_pos)) {
    numInOwnHalf++;
    if (numInOwnHalf==5 && numInOwnHalfFront>=4) {
      ballSeenInOpponentHalf.add_sec(-100); // Verhalten ausschalten, offensichtlich ein Dribbling und kein Schuss, da in den letzten 5 Zyklen der Ball mindestens 4 mal in ownHalfFront war
    }
  }

  if (ball_own_known && ownHalf.is_inside(ball_pos)) {
    latestBallPositionInOwnHalf=ball_pos;
    ballSeenInOwnHalf=true;
    numBallSeenInOpponentHalf=0;
  }

  DriveVector dest;
  Vec tp = home;
  if (ballSeenInOwnHalf) {
    try{
      Vec gpl, gpr;
      gpl.y=gpr.y=home.y;
      gpr.x=0.5*MWM.get_field_geometry().goal_width+1500;
      gpl.x=-gpr.x;
      vector<Vec> ip = intersect (LineSegment(gpl, gpr), Line (latestBallPositionInOwnHalf, latestBallPositionInOpponentHalf));
      LOUT << "% dark_red thick arrow " << latestBallPositionInOpponentHalf << latestBallPositionInOwnHalf << '\n';
      if (ip.size()>0)
        tp=ip[0];
      if (tp.x<-maxextend)
        tp.x=-maxextend;
      if (tp.x>maxextend)
        tp.x=maxextend;
    }catch(invalid_argument&){;}  // irgendein geometrisches Problem
  }
  goto_pos_skill->init (tp, tghead, true);
  dest = goto_pos_skill->getCmd (texec);
  dest.kick=false;
  return dest;
}

void BGoaliePositioningChipKick::cycleCallBack (const Time& texec) throw () {
  bool isInToleranceArea2 = toleranceArea2.is_inside (MWM.get_ball_location(texec).pos.toVec());
  wasInToleranceArea2TwoTimes = isInToleranceArea2 && wasInToleranceArea2;
  wasInToleranceArea2=isInToleranceArea2;
}

void BGoaliePositioningChipKick::updateTactics (const TacticsBoard& tb) throw () {
  double hfl = 0.5*MWM.get_field_geometry().field_length;
  double y=hfl+100000;
  string value = tb["GoalieChipKickPositioning"];
  if (value=="nie")
    setOpponentHalf (hfl+100000);
  else if (value=="immer")
    setOpponentHalf (-hfl-100000);
  else if (value=="GegnerHaelfte")
    setOpponentHalf (0);
  else if (value=="GegnerDrittel")
    setOpponentHalf (hfl/6);
  else if (value=="GegnerZweiDrittel")
    setOpponentHalf (-hfl/6);
  else if (value=="GegnerViertel")
    setOpponentHalf (hfl/2);
  else if (value=="GegnerDreiViertel")
    setOpponentHalf (-hfl/2);
  else if (string2double (y, value) && y>-hfl && y<hfl)
    setOpponentHalf (y);
}
