
#include "BOwnPenalty.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/random.h"
#include "../../../Player/WhiteBoard.h"

using namespace Tribots;
using namespace std;

namespace {

  /** Fahre in den Mittelkreis bis Ball freigegeben wird */
  class BOwnPenaltyGotoCenter : public Behavior {
    SPhysGotoPosAvoidObstacles goto_pos;
  public:
    BOwnPenaltyGotoCenter () throw () : Behavior ("BOwnPenaltyGotoCenter") {
      goto_pos.set_dynamics (1.5);
      goto_pos.init (Vec(0,0), Angle::zero, true);
    }
    bool checkInvocationCondition (const Time&) throw () {
      return MWM.get_game_state().refstate==preOwnPenalty;
    }
    bool checkCommitmentCondition (const Time&) throw () {
      return MWM.get_game_state().refstate==preOwnPenalty;
    }
    DriveVector getCmd(const Time& t) throw(TribotsException) {
      return goto_pos.getCmd(t);
    }
  };

  /** Fahre bis kurz vor den Strafstosspunkt bzw. kurz vor den Ball und gebe dich dann auf */
  class BOwnPenaltyGotoSpot : public Behavior {
    SPhysGotoPos goto_pos;
    Vec targetPos;
  public:
    BOwnPenaltyGotoSpot () throw () : Behavior ("BOwnPenaltyGotoSpot") {
      goto_pos.set_dynamics (1.5);
      targetPos.x=0;
      targetPos.y=0.5*MWM.get_field_geometry().field_length-MWM.get_field_geometry().penalty_marker_distance-900;
    }
    bool checkInvocationCondition (const Time& t) throw () {
      return true;
    }
    bool checkCommitmentCondition (const Time& t) throw () {
      return !goto_pos.destination_reached (t) || MWM.get_robot_location(t).vtrans.length()>0.5;
    }
    void gainControl (const Time& t) throw () {
      targetPos.x=0;
      targetPos.y=0.5*MWM.get_field_geometry().field_length-MWM.get_field_geometry().penalty_marker_distance-900;
    }
    DriveVector getCmd(const Time& t) throw(TribotsException) {
      const BallLocation& ball (MWM.get_ball_location(t));
      if (ball.pos_known==BallLocation::known && (targetPos-(ball.pos.toVec()-Vec(0,900))).length()>500)
        targetPos=ball.pos.toVec()-Vec(0,900);
      goto_pos.init (targetPos, Angle::zero, true);
      return goto_pos.getCmd(t);
    }
  };

  /** Warte 15 Zyklen und entscheide dich fuer eine Ecke. Gebe dich danach auf */
  class BOwnPenaltyDecide : public Behavior {
    double& targetPosX;
    unsigned int waitCounter;
    unsigned int numRightDecisions;
    unsigned int numLeftDecisions;
  public:
    BOwnPenaltyDecide (double& tPX) throw () : Behavior ("BOwnPenaltyDecide"), targetPosX(tPX), waitCounter(0) {;}
    bool checkInvocationCondition (const Time&) throw () { return true; }
    bool checkCommitmentCondition (const Time&) throw () { return waitCounter<15; }
    void gainControl (const Time&) throw () { waitCounter=0; numRightDecisions=0; numLeftDecisions=0; }
    DriveVector getCmd (const Time& t) throw (TribotsException) {
      waitCounter++;
      const BallLocation bloc (MWM.get_ball_location(t));
     const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);

      double goal_y = 0.5*MWM.get_field_geometry().field_length;
      double goalarea_y = goal_y-1000;
      double goal_hw = 0.5*MWM.get_field_geometry().goal_width;
      Vec goalie_pos (0,goal_y);
      double goalie_width=-1;
      for (unsigned int i=0; i<obstacles.size(); i++) {
        if (obstacles[i].pos.y<goalarea_y || obstacles[i].pos.y>goal_y+1000)
          continue;  // kein relevantes Hindernis
        if (obstacles[i].pos.x<-goal_hw || obstacles[i].pos.x>goal_hw)
          continue; // kein relevantes Hindernis
        if ((goalie_width<100) || (goalie_pos-Vec(0,goal_y)).squared_length()>(obstacles[i].pos-Vec(0,goal_y)).squared_length()) {
          goalie_pos=obstacles[i].pos;
          goalie_width=obstacles[i].width;
        }
      }
      if (goalie_width<=0) {
        bool do_left = brandom (0.5);
        LOUT << "kein Hindernis gesehen, entscheide fuer " << (do_left ? "links" : "rechts") << "\n";
        if (do_left)
          numLeftDecisions++;
        else
          numRightDecisions++;
      } else {
        LOUT << "% black solid thick circle " << goalie_pos << " " << 0.6*goalie_width << "\n";
        double space_left = goalie_pos.x+goal_hw-0.5*goalie_width-MWM.get_field_geometry().ball_diameter;
        double space_right = goal_hw-goalie_pos.x-0.5*goalie_width-MWM.get_field_geometry().ball_diameter;
        if (space_left<0) space_left=0;
        if (space_right<0) space_right=0;
        double part_space_left = space_left/(space_right+space_left+0.1);
        double prob_left = 1.0/(1.0+exp(50*(0.5-part_space_left)));
        bool do_left = brandom (prob_left);
        LOUT << "Anteil Freiraum links: " << part_space_left*100 << "%, Linkswahrscheinlichkeit: " << prob_left*100 << "%\n";
        LOUT << "Entscheide fuer " << (do_left ? "links" : "rechts") << "\n";
        if (do_left)
          numLeftDecisions++;
        else
          numRightDecisions++;
      }
      targetPosX = (numLeftDecisions>numRightDecisions ? -1 : +1)*2*goal_hw/3;
      DriveVector nullvec (Vec(0,0), 0, false);
      return nullvec;
    }
  };

  /** Fahre an eine Position schraeg hinter dem Ball, auf das Ziel ausgerichtet */
  class BOwnPenaltyGotoKickPos : public Behavior {
    double& targetPosX;
    SPhysGotoPos goto_pos;
    Vec toPos;
    Angle toAngle;
  public:
    BOwnPenaltyGotoKickPos (double& tPx) throw () : Behavior ("BOwnPenaltyGotoKickPos"), targetPosX (tPx) {
      goto_pos.set_dynamics (1.0, 2.0);
    }
    void gainControl (const Time& t) throw () {
      Vec ball = MWM.get_ball_location (t).pos.toVec();
      double goaly = 0.5*MWM.get_field_geometry().field_length;
      toPos.y=ball.y-1500;
      toPos.x=ball.x-1500*(targetPosX-ball.x)/(goaly-ball.y);
      toAngle=(Vec(targetPosX, goaly)-ball).angle()-Angle::quarter;
      goto_pos.init (toPos, toAngle, true);
    }
    bool checkInvocationCondition (const Time&) throw () { return true; }
    bool checkCommitmentCondition (const Time& t) throw () { return !goto_pos.destination_reached(t) || MWM.get_robot_location(t).vtrans.length()>0.5; }
    DriveVector getCmd (const Time& t) throw () {
      LOUT << "% dark_red thick solid arrow " << toPos << Vec(targetPosX,0.5*MWM.get_field_geometry().field_length) << "\n";
      return goto_pos.getCmd(t);
    }
  };

  /** Warte eine kurze Zeitspanne */
  class BOwnPenaltyWaitBeforeKick : public Behavior {
    Time timeout;
  public:
    BOwnPenaltyWaitBeforeKick () throw () : Behavior ("BOwnPenaltyWaitBeforeKick") {;}
    void gainControl (const Time& t) throw () { timeout=t; timeout.add_msec (500); }
    bool checkInvocationCondition (const Time&) throw () { return true; }
    bool checkCommitmentCondition (const Time& t) throw () { return t<timeout; }
    DriveVector getCmd (const Time& t) throw () {
      DriveVector nullvec (Vec(0,0),0,false);
      return nullvec;
    }
  };

  /** Ballanfahrt durch Fahren zur Ballposition (geregelt) */
  class BOwnPenaltyPreApproach : public Behavior {
    SPhysGotoPos goto_pos;
  public:
    BOwnPenaltyPreApproach () throw () : Behavior ("BOwnPrePenaltyApproach") { goto_pos.set_dynamics (0.5); }
    bool checkInvocationCondition (const Time&) throw () { return true; }
    bool checkCommitmentCondition (const Time& t) throw () { return (MWM.get_robot_location(t).pos-MWM.get_ball_location(t).pos.toVec()).length()>500; }
    DriveVector getCmd (const Time& t) throw () {
      goto_pos.init (MWM.get_ball_location(t).pos.toVec(), (MWM.get_ball_location(t).pos.toVec()-MWM.get_robot_location(t).pos).angle()-Angle::quarter, false);
      return goto_pos.getCmd(t);
    }
  };
  
  /** Ballanfahrt auf den letzten Zentimetern durch Fahren nach vorne (ungeregelt),
      um den Ball zwischen die Hoernchen zu kriegen */
  class BOwnPenaltyApproach : public Behavior {
  public:
    BOwnPenaltyApproach () throw () : Behavior ("BOwnPenaltyApproach") {;}
    bool checkInvocationCondition (const Time&) throw () { return true; }
    bool checkCommitmentCondition (const Time& t) throw () { return MWM.get_robot_location(t).pos.y<0.5*MWM.get_field_geometry().field_length-1000; }
    DriveVector getCmd (const Time& t) throw () {
      DriveVector dest (Vec(0,2.0), 0, false);
      return dest;
    }
  };
  
  /** Kicke, sobald Ballbesitz und gebe dich dann sofort auf */
  class BOwnPenaltyKick : public Behavior {
    double shoot_height;
  public:
    BOwnPenaltyKick () throw () : Behavior ("BOwnPenaltyKick"), shoot_height(800) {;}
    bool checkInvocationCondition (const Time& t) throw () { return WBOARD->doPossessBall(t); }
    bool checkCommitmentCondition (const Time&) throw () { return false; }
    DriveVector getCmd (const Time& t) throw () {
      DriveVector dest = MWM.get_recent_drive_vector ();
      dest.kick = true;
      //wenn ich den neuen kicker habe und gerade kicken soll
      if(MWM.get_robot_properties().kickers > 1 && dest.kick) {
        bool reachable = false;
        dest.klength = WBOARD->getKickLength(MWM.get_field_geometry().penalty_marker_distance, shoot_height, &reachable );
        LOUT << "distance to target: " << MWM.get_field_geometry().penalty_marker_distance
                        << "mm, schiesse mit: " << dest.klength << "ms \n";
        LOUT << "versuche das tor auf: " << shoot_height << "mm zu treffen \n";
      }
      LOUT << "KICK " << dest.kick << ' ' << dest.klength << endl;
      return dest;
    }
    void updateTactics (const TacticsBoard& tb) throw () {
      string val="";
      val = tb[string("penalty_shoot_height")];
      if (val!="") {
        shoot_height = atof(val.c_str());
      }
      LOUT << "penalty_shoot_height: " << shoot_height << "\n";
    }
  };

  /** nach dem Penalty abbremsen und stehen bleiben, bis der Gamestate wechselt */
  class BOwnPenaltySlowDown : public Behavior {
  public:
    BOwnPenaltySlowDown () throw () : Behavior ("BOwnPenaltySlowDown") {;}
    bool checkInvocationCondition (const Time&) throw () { return true; }
    bool checkCommitmentCondition (const Time&) throw () { return MWM.get_game_state().refstate==ownPenalty; }
    DriveVector getCmd (const Time& t) throw () {
      DriveVector dest = MWM.get_recent_drive_vector ();
      dest.vrot*=0.7;
      dest.vtrans*=0.7;
      dest.kick=false;
      if (dest.vrot<0.5)
        dest.vrot=0;
      if (dest.vtrans.length()<0.1)
        dest.vtrans=Vec(0,0);
      return dest;
    }
  };

}

BOwnPenalty::BOwnPenalty () : SPBehavior ("BOwnPenalty") {
  appendStage (new BOwnPenaltyGotoCenter, false, false);
  appendStage (new BOwnPenaltyGotoSpot, false, false);
  appendStage (new BOwnPenaltyDecide (targetX), false, false);
  appendStage (new BOwnPenaltyGotoKickPos (targetX), false, false);
  appendStage (new BOwnPenaltyWaitBeforeKick, false, false);
  appendStage (new BOwnPenaltyPreApproach, false, false);
  appendStage (new BOwnPenaltyApproach, false, true);
  appendStage (new BOwnPenaltyKick, true, false);  // Schuss unterbricht die Anfahrt; kann uebersprungen werden, wenn am Ball vorbei gefahren wurde
  appendStage (new BOwnPenaltySlowDown, false, false, 0);  // darf kein vorheriges Behavior unterbrechen
}

bool BOwnPenalty::checkInvocationCondition (const Time& t) throw () {
  return ((MWM.get_game_state ().refstate==preOwnPenalty) || (MWM.get_game_state ().refstate==ownPenalty)) && SPBehavior::checkInvocationCondition (t);
}
bool BOwnPenalty::checkCommitmentCondition (const Time& t) throw () {
  return ((MWM.get_game_state ().refstate==preOwnPenalty) || (MWM.get_game_state ().refstate==ownPenalty)) && SPBehavior::checkCommitmentCondition (t);
}



/* // Alte Version, alles in einem Behavior:
BOwnPenalty::BOwnPenalty() : Behavior("BOwnPenalty"), goto_ball (new SPhysGotoBallCarefully), goto_pos (new SPhysGotoPos), goto_pos_slowly (new SPhysGotoPos), goto_pos_obs (new SPhysGotoPosAvoidObstacles)
{
  was_active = false;
  decision_done = false;
  target_kick_pos = Vec(0,0);
  goto_pos->set_dynamics (1.5);
  goto_pos_slowly->set_dynamics (0.5, 1.0);
  goto_ball->set_dynamics (1.5);
  goto_pos_obs->set_dynamics (1.5);
  goto_pos_obs->init (Vec(0,0), Angle::zero, true);
  shoot_height = 800;					// auf dieser hoehe kommt der ball ins tor (neuer kicker)
}

BOwnPenalty::~BOwnPenalty() throw() {
  delete goto_pos;
  delete goto_pos_slowly;
  delete goto_ball;
  delete goto_pos_obs;
}

void BOwnPenalty::updateTactics (const TacticsBoard& tb) throw ()
{
	 //TODO config files mit einchecken..
  string val="";
  val = tb[string("penalty_shoot_height")];
  if (val!="") {
    shoot_height = atof(val.c_str());
  }
  LOUT << "penalty_shoot_height: "
       << shoot_height << "\n";
}

void BOwnPenalty::gainControl(const Time&) throw() {
  was_active=false;
  decision_done=false;
  point_of_no_return=false;
  ball_touched=false;
}

void BOwnPenalty::loseControl(const Time& t) throw() {
  gainControl(t);
}

bool BOwnPenalty::checkCommitmentCondition(const Time&) throw() {
  return ((was_active && MWM.get_game_state().refstate == ownPenalty) || MWM.get_game_state().refstate == preOwnPenalty);
  // was_active ist ein Mechanismus, um gezielt die Kontrolle abzugeben, wird im Moment kein Gebrauch gemacht
}
  
bool BOwnPenalty::checkInvocationCondition(const Time& t) throw() {
  return checkCommitmentCondition(t);
}

DriveVector BOwnPenalty::getCmd(const Time& t) throw(TribotsException) {
  DriveVector dest;
  was_active = true;
  // vor dem Strafstoss an die Ausgangsposition fahren
  if (MWM.get_game_state().refstate == preOwnPenalty) {
    LOUT << "prePenalty\n";
    decision_done=false;
    goto_pos_obs->init (Vec(0,0),Angle::zero,true);
    dest = goto_pos_obs->getCmd (t);
    dest.kick=false;
    return dest;
  }
  // Strafstoss angepfiffen  
  const FieldGeometry& fgeom = MWM.get_field_geometry();
  Vec ball = Vec (0, 0.5*fgeom.field_length-fgeom.penalty_marker_distance);
  const BallLocation& bloc = MWM.get_ball_location (t);
  const RobotLocation& rloc = MWM.get_robot_location (t);
  if (bloc.pos_known==BallLocation::known)
    ball = bloc.pos.toVec();
  if (wait.elapsed_msec()<5000 && bloc.velocity.toVec().length()>0.8) {
    // Nach erstem Schuss nicht naeher als 1200mm vor das Tor fahren
    if (rloc.pos.y>0.5*fgeom.field_length-1200) {
      LOUT << "nicht in den Torwartbereich hineinfahren\n";
      return dest;
    }
  }
  if (!decision_done) {
    LOUT << "nicht entschieden\n";
    // Erste Phase: Roboter faehrt an den Ball ran und entscheidet sich dann fuer eine Ecke
    if ((rloc.pos-ball).length()>1200 || rloc.vtrans.length()>0.01 || abs(rloc.vrot)>0.01) {
      LOUT << "an den Ball ranfahren\n";
      // an den Ball ranfahren
      goto_pos->init (ball-Vec(0,900), Angle::zero, true);
      dest = goto_pos->getCmd (t);
      dest.kick = false;
      return dest;
    } else {
      // Richtungsentscheidung berechnen
      // Position und Breite des Torwarts in der Hindernisliste suchen
      orig_ball_pos = ball;
      double goal_y = 0.5*fgeom.field_length;
      double marker_y = goal_y-fgeom.penalty_marker_distance;
      double goal_hw = 0.5*fgeom.goal_width;
      Vec goalie_pos (0,goal_y);
      double goalie_width=-1;
      for (unsigned int i=0; i<obstacles.size(); i++) {
        if (obstacles[i].pos.y<marker_y || obstacles[i].pos.y>goal_y+1000)
          continue;  // kein relevantes Hindernis
        if (obstacles[i].pos.x<-goal_hw || obstacles[i].pos.x>goal_hw)
          continue; // kein relevantes Hindernis
        if ((goalie_width<100) || (goalie_pos-Vec(0,goal_y)).squared_length()>(obstacles[i].pos-Vec(0,goal_y)).squared_length()) {
          goalie_pos=obstacles[i].pos;
          goalie_width=obstacles[i].width;
        }
      }
      if (goalie_width<=0) {
        bool do_left = brandom (0.5);
        LOUT << "kein Hindernis gesehen, schiesse " << (do_left ? "links" : "rechts") << "\n";
        target_kick_pos = Vec ((do_left ? -1 : +1)*2*goal_hw/3,goal_y);
        decision_done=true;
        point_of_no_return=false;
      } else {
        double space_left = goalie_pos.x+goal_hw-0.5*goalie_width-fgeom.ball_diameter;
        double space_right = goal_hw-goalie_pos.x-0.5*goalie_width-fgeom.ball_diameter;
        if (space_left<0) space_left=0;
        if (space_right<0) space_right=0;
        double part_space_left = space_left/(space_right+space_left+0.1);
        double prob_left = 1.0/(1.0+exp(50*(0.5-part_space_left)));
        bool d = brandom (prob_left);
        LOUT << "Anteil Freiraum links: " << part_space_left*100 << "%, Linkswahrscheinlichkeit: " << prob_left*100 << "%\n";
        if (d) {
          // links einschiessen
          target_kick_pos = Vec(0.5*(goalie_pos.x-0.5*goalie_width-0.5*goal_hw), goal_y);
          double md = 2.0*goal_hw/3.0;
          if (target_kick_pos.x>-md)
            target_kick_pos.x=-md;
          LOUT << "schiesse links\n";
        } else {
          // rechts einschiessen
          target_kick_pos = Vec(0.5*(goalie_pos.x+0.5*goalie_width+0.5*goal_hw), goal_y);
          double md = 2.0*goal_hw/3.0;
          if (target_kick_pos.x<md)
            target_kick_pos.x=md;
          LOUT << "schiesse rechts\n";
        }
        decision_done=true;
        point_of_no_return=false;
      }
    }
  }
  // Entscheidung wurde gefaellt, jetzt sich um den Ball drehen, anfahren und schiessen, anschliessend Kontrolle abgeben
  LOUT << "% thick red solid arrow " << ball << ' ' << target_kick_pos << '\n';
  Angle target_angle =  (target_kick_pos-ball).angle()-Angle::quarter;
  Angle rbh = (ball-rloc.pos).angle()-Angle::quarter;
  bool ausrichtung1 = (abs((rbh-target_angle).get_deg_180())<3.0);
  bool ausrichtung2 = (abs((rbh-rloc.heading).get_deg_180())<3.0);
  bool langsam = (rloc.vtrans.length()<0.15 && abs(rloc.vrot)<0.15);
  if (point_of_no_return || (ausrichtung1 && ausrichtung2 && langsam)) {
    point_of_no_return=true;
    if (ball_touched) {
      goto_ball->init (target_angle, false);
      dest = goto_ball->getCmd (t);
    } else {
      goto_pos_slowly->init (ball, target_angle, false);
      dest = goto_pos_slowly->getCmd (t);
    }
    Time tkick = t;
//    tkick.add_msec (-200);
    RobotLocation rkick = MWM.get_robot_location (tkick);
    BallLocation bkick = MWM.get_ball_location (tkick);
    dest.kick = false;
    ball_touched = ((ball-orig_ball_pos).length()>300);
    try{
      Line robot_mid (rkick.pos, rkick.pos+Vec::unit_vector_y.rotate (rkick.heading));
      Vec pp = robot_mid.perpendicular_point (bkick.pos.toVec());
      dest.kick = WBOARD->doPossessBall(t);
   // (pp-bkick.pos.toVec()).length()<0.5*MWM.get_robot_properties().kicker_width
//          && (pp-rkick.pos).angle().in_between (rkick.heading, rkick.heading+Angle::half)
//          && (pp-rkick.pos).length()<0.5*fgeom.ball_diameter+MWM.get_robot_properties().kicker_distance+50;
      LOUT << "BOwnPenalty: pBall=" << WBOARD->doPossessBall(t) << "\n";
		//wenn ich den neuen kicker habe und gerade kicken soll
		if(MWM.get_robot_properties().kickers > 1 && dest.kick) {
		  bool reachable = false;
		  dest.klength = WBOARD->getKickLength( (ball-target_kick_pos).length(), shoot_height, &reachable );
		  LOUT << "distance to target: " << (ball - target_kick_pos).length()
				 << "mm, schiesse mit: " << dest.klength << "ms \n";
		  LOUT << "versuche das tor auf: " << shoot_height << "mm zu treffen \n";
		}
    }catch(std::invalid_argument&){;} // Ball- und Roboterposition fallen aufeinander
    LOUT << "GotoBall\n";
  } else {
    ball_touched = false;
    double turn_radius = MWM.get_robot_properties().kicker_distance+0.5*fgeom.ball_diameter+300;
    goto_pos_slowly->init (ball+turn_radius*Vec::unit_vector (target_angle-Angle::quarter), target_angle, true, false, false);
    dest = goto_pos_slowly->getCmd(t);
    dest.kick = false;
    LOUT << "TurnAround\n";
  }
  if (dest.kick) {
    wait.update();
  }
  return dest;
}
*/
