#include "BPreOpponentStandardSituation.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Structures/Journal.h"
#include "../../../Fundamental/stringconvert.h"
#include <cmath>
#include <vector>

using namespace Tribots;
using namespace std;

BPreOpponentStandardSituation::BPreOpponentStandardSituation(bool blockGoal)
  : Behavior("BPreOpponentStandardSituation"),
  skill(new SPhysGotoPosAvoidObstacles()),
  blockGoal(blockGoal),
  lookForBall(false)
{
  ballposknown.set_sec(-100);
  mindestballabstand1=1000;
  mindestballabstand2=2000;
  robotspeed=1.8;
  skill->set_ball_as_obstacle(true, true);
}

BPreOpponentStandardSituation::~BPreOpponentStandardSituation() throw ()
{
  delete skill;
}

void BPreOpponentStandardSituation::updateTactics (const TacticsBoard& tb) 
    throw () 
{
  if (tb[string("AbstossGegner")] == "blockTor") {
    blockGoal = true;
  }
  else {
    blockGoal = false;
  }
  double d;
  if (string2double (d, tb[string("StandardSituationAbstand")])) {
    if (d>=100 && d<=2100)
      mindestballabstand2=d;
  }
/*  if (string2double (d, tb[string("MindestBallAbstand2m")])) {
    if (d>=100 && d<=2100)
      mindestballabstand2=d;
  }*/
// AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
       robotspeed=1.8;
  } else	  
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
       robotspeed=1.4;
  } else
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
       robotspeed=2.3;
  } else { // =normal
       robotspeed=2.0;
  }
  


  if (tb[string("StandardSituationSpionAufstellen")] == "an") {
    lookForBall = true;
  }
  else {
    lookForBall = false;
  }
   
  LOUT << "Tactics changed in BPreOpponentStandardSituation to blockGoal=" 
      << (blockGoal ? "true" : "false");
  LOUT << ", lookForBall=" << (lookForBall ? "true" : "false");  
  LOUT << " MindestBallAbstand1m=" << mindestballabstand1;
  LOUT << " MindestBallAbstand2m=" << mindestballabstand2 << endl;
}

bool BPreOpponentStandardSituation::checkCommitmentCondition(const Time& t) throw()
{
  return
    (MWM.get_game_state().refstate == preOpponentThrowIn ||
     MWM.get_game_state().refstate == preOpponentGoalKick ||
     MWM.get_game_state().refstate == preOpponentCornerKick ||
     MWM.get_game_state().refstate == preOpponentFreeKick ||
     MWM.get_game_state().refstate == preOpponentKickOff ||
     MWM.get_game_state().refstate == preDroppedBall);
}

bool BPreOpponentStandardSituation::checkInvocationCondition(const Time& t) 
    throw()
{
  const BallLocation& ball = MWM.get_ball_location(t);

  return 
    (MWM.get_game_state().refstate == preOpponentThrowIn ||
     MWM.get_game_state().refstate == preOpponentGoalKick ||
     MWM.get_game_state().refstate == preOpponentCornerKick ||
     MWM.get_game_state().refstate == preOpponentFreeKick ||
     MWM.get_game_state().refstate == preOpponentKickOff ||
     MWM.get_game_state().refstate == preDroppedBall) &&
    (ball.pos_known == BallLocation::known || 
     ball.pos_known==BallLocation::communicated || 
     ballposknown.elapsed_msec()<3000);
}

void BPreOpponentStandardSituation::gainControl(const Time&) 
    throw(TribotsException) 
{
  targetEvadeDirArea=9999;
  ballposknown.set_sec(-10);
  prefside=0;
}

void BPreOpponentStandardSituation::setBallPos (const Time& t)
{
  const BallLocation& bloc = MWM.get_ball_location (t);
  Vec ballposwm = bloc.pos.toVec();
  Vec ballvelwm = bloc.velocity.toVec();
  bool validwm = (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::communicated);
  RefereeState refstate = MWM.get_game_state().refstate;
  const FieldGeometry& field = MWM.get_field_geometry();
  
  if (ballposknown.elapsed_msec()<3000 && (ballvelwm.length()>0.5 || (ballpos-ballposwm).length()<300)) {
    if ((ballpos-ballposwm).length()<300 && ballvelwm.length()<0.5 && validwm) {
      ballpos=0.9*ballpos+0.1*ballposwm;
      ballposknown.update();
    }
    return; // alte Ballposition verwenden, da der Ball sich gerade bewegt (entweder Fehlschaetzung oder Ball weggestupst)
  }
  if (validwm) {  
    ballposknown.update();
  }
  
  ballpos = ballposwm;
  // Ball niemals ausserhalb des Feldes
  if (ballpos.y<-0.5*field.field_length) ballpos.y=-0.5*field.field_length;
  if (ballpos.y>0.5*field.field_length) ballpos.y=0.5*field.field_length;
  if (ballpos.x>0.5*field.field_width) ballpos.x=0.5*field.field_width;
  if (ballpos.x<-0.5*field.field_width) ballpos.x=-0.5*field.field_width;
  switch (refstate) {
    case preOpponentKickOff:
      // Ball nicht weiter als 50cm von Mittelpunkt entfernt
      if (ballpos.x>500) ballpos.x=500;
      if (ballpos.x<-500) ballpos.x=-500;
      if (ballpos.y<-500) ballpos.y=-500;
      if (ballpos.y>500) ballpos.y=500;
      break;
    case preOpponentCornerKick:
      // Ball im Bereich der hinteren Ecken
      if (ballpos.y>-0.5*field.field_length+1000) ballpos.y=-0.5*field.field_length+1000;
      // absichtlich kein break, es geht direkt weiter!
    case preOpponentThrowIn:
      // Ball im Bereich der Seitenlinien
      if (ballpos.x>0) {
        if (ballpos.x<0.5*field.field_width-800) ballpos.x=0.5*field.field_width-800;
      } else {
        if (ballpos.x>-0.5*field.field_width+800) ballpos.x=-0.5*field.field_width+800;
      }
      break;
    default:
      break;
  }
}

DriveVector BPreOpponentStandardSituation::getCmd(const Time& t) throw(TribotsException)
{
  setBallPos (t);
  LOUT << "% dark_red solid cross " << ballpos << '\n';
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& field = MWM.get_field_geometry();
  
  Vec position(0., 0.);
  
  double d = 
    (MWM.get_game_state().refstate == preDroppedBall ? 
     mindestballabstand1 : mindestballabstand2) + toleranz;
  
  if ( MWM.get_game_state().refstate == preOpponentKickOff ) {
    d = MWM.get_field_geometry().center_circle_radius + toleranz;
  }

  bool is_ball = false;
  switch (WBOARD->getStandardSituationRole()) { // welche position hat der
    case WhiteBoard::STANDARD_ROLE_AB:
    case WhiteBoard::STANDARD_ROLE_ABCDE:           // bekommen?
    case WhiteBoard::STANDARD_ROLE_A:
    case WhiteBoard::STANDARD_ROLE_B: 
      // der Spieler ist am Ball
      position = getPositionAtBall(t, d, blockGoal);

      // Nun die Spezialfaelle fuer zwei Spieler am Ball (Rollen A und B eigener Roboter)
      // Beim FP07 betrifft dies 4 und mehr Feldspieler
      if (WBOARD->getStandardSituationRole() == WhiteBoard::STANDARD_ROLE_A &&
          ballpos.x > -1000.) {
        is_ball = false;  // 2. Spieler am Ball, ausweichen o.k.
        prefside = POS_LEFT;

        if (lookForBall && 
            MWM.get_game_state().refstate != preOpponentKickOff && 
            ballpos.y > -field.field_length/2. )//  + 500.) wIESO NICHT GANZ BIS
            //IN DIE ECKE?  -> Weil von da niemand direkt in das Tor schiessen kann.
        { 
          position = ballpos+
            ((Vec(0, -field.field_length/2.)-ballpos).normalize()*(d+110.)).rotate(-Angle::quarter);
        } // Ende Spioncode; Bei KickOff Gegner ist _immer_ B der Spion!
      }
      else if (WBOARD->getStandardSituationRole() == WhiteBoard::STANDARD_ROLE_B &&
               ballpos.x < -1100.) {
        is_ball = false;  // 2. Spieler am Ball, ausweichen o.k.
        prefside = POS_RIGHT;
        if (lookForBall && 
            MWM.get_game_state().refstate != preOpponentKickOff && 
            ballpos.y > -field.field_length/2.)// + 500.) s.o.
        { 
          position = ballpos+
            ((Vec(0, -field.field_length/2.)-ballpos).normalize()*(d+110.)).rotate(Angle::quarter);
        } // Ende Spioncode fuer alle Faelle AUSSER KickOff Gegner
        else if(MWM.get_game_state().refstate == preOpponentKickOff  && 
                lookForBall) 
        {
          position = 
            Vec(0.,0.) + (Vec(0,-1.)*(d+120.)).rotate(Angle::sixth);
        } // Ende Spioncode fuer den Fall KickOff Gegner
      }
      else {
        is_ball = true; // dieser Roboter soll das Tor abdecken, daher nicht anderen ausweichen
      }
      break;
    case WhiteBoard::STANDARD_ROLE_C:
    case WhiteBoard::STANDARD_ROLE_CE:
      // Option: "Spion" aufstellen, ausser bei anstoss. Bei blockGoal muss der rechte
      // Blocker auf der rechten Spielfeldseite zum Spion werden!
      if (lookForBall && 
          MWM.get_game_state().refstate != preOpponentKickOff && 
          ballpos.y > -field.field_length/2. + 500. &&
          WBOARD->onlyThreeRobots() && // bei 4 oder mehr bots wird die Rolle A oder B zum Spion
          (ballpos.x > 200 || (ballpos.x <= 200 && blockGoal))) 
      {
        position = ballpos+
          ((Vec(0, -field.field_length/2.)-ballpos).normalize()*(d+110.)).rotate(ballpos.x < 0 ? Angle::quarter : -Angle::quarter);
      } // Ende Spioncode
      else {
        prefside = POS_LEFT;
        position = getPositionBlock(t, prefside, d, blockGoal);
      } 
      break;
    case WhiteBoard::STANDARD_ROLE_D:
    case WhiteBoard::STANDARD_ROLE_DE:
      if (lookForBall && 
          MWM.get_game_state().refstate != preOpponentKickOff && 
          ballpos.y > -field.field_length/2. + 500. &&
          WBOARD->onlyThreeRobots() && // bei 4 oder mehr bots wird die Rolle A oder B zum Spion
          (ballpos.x < 200 || (ballpos.x >= 200 && blockGoal))) 
      {
        position = ballpos+
          ((Vec(0, -field.field_length/2.)-ballpos).normalize()*(d+110.)).rotate(ballpos.x < 0 ? Angle::quarter : -Angle::quarter);
      } // Ende Spioncode
      else {
        prefside = POS_RIGHT;
        position = getPositionBlock(t, prefside, d, blockGoal);
      }
      break;
    case WhiteBoard::STANDARD_ROLE_CD:
    case WhiteBoard::STANDARD_ROLE_CDE:
      if (abs(ballpos.x)<200 && (prefside==POS_LEFT || prefside==POS_RIGHT)) {
        prefside = POS_RIGHT;
      } else if ((MWM.get_game_state().refstate != 
           preOpponentGoalKick && ballpos.x < 0) || 
           (MWM.get_game_state().refstate == 
           preOpponentGoalKick && ballpos.x > 0)) {
        prefside = POS_LEFT;
      } else {
        prefside = POS_RIGHT;
      }
      position = getPositionBlock(t, prefside, d, blockGoal);
      break;
    case WhiteBoard::STANDARD_ROLE_E:
      position = getPositionSafety(t);
    break;
  }
  if (!blockGoal && 
      position.y<-0.5*MWM.get_field_geometry().field_length+500 &&
      fabs(position.x)<0.5*MWM.get_field_geometry().goal_width+500) {
    LOUT << "Zielposition zu Nah am Tor. Korrektur berechnet" << endl;
    position.x = 
      (position.x<0 ? -1.0 : 1.0)*
      (0.5*MWM.get_field_geometry().goal_width+1000);
      // Korrektur, falls Zielposition sehr nahe am oder im Tor
  }

  setTargetEvadeDirection (position, ballpos-robot.pos, prefside, is_ball);

  skill->set_dynamics ((robot.pos-position).length() < 300. ? robotspeed/4. : robotspeed);
  skill->init (position, ballpos-position, true);
  LOUT << "% thick solid yellow cross " << position << " word " 
       << (position+Vec(0.,500.)) << " target pos" << endl;
  LOUT << "% thin dotted yellow line " << position << " " 
       << (position+(ballpos-position).normalize() * 1500.) << endl;
  return skill->getCmd(t);
}

Vec BPreOpponentStandardSituation::getPositionAtBall(const Time& t, double d,
      bool blockGoalOppGoalKick)
{
  const FieldGeometry& field = MWM.get_field_geometry();
  Vec ownGoal = Vec(0., -field.field_length/2.);
  Vec position(ownGoal-ballpos);
  position = position.normalize() * (d + 110.);
  position += ballpos;
  
  if (WBOARD->getStandardSituationRole() == WhiteBoard::STANDARD_ROLE_A &&
      ballpos.x > 500.) {
    position += 
      ((ownGoal-ballpos).normalize()) * 600. +   // weiter hinten als der andere Roboter
      ((ownGoal-ballpos).normalize()) * 300. * Angle::three_quarters;// und mehr zur Mitte
  }
  else if (WBOARD->getStandardSituationRole() == WhiteBoard::STANDARD_ROLE_B &&
           ballpos.x < 500.) {
    position += 
      ((ownGoal-ballpos).normalize()) * 600. +   // weiter hinten als der andere Roboter und
      ((ownGoal-ballpos).normalize()) * 300. * Angle::quarter;       // mehr zur Mitte
  }


  XYRectangle penaltyArea(
      Vec(-field.penalty_area_width/2.+200, -field.field_length / 2. - 2500.), // hinter gl
      Vec(field.penalty_area_width/2.-200, -field.field_length / 2. + field.penalty_area_length-200));
    
  // beim freistoss darf man im strafraum stehen wo man will  // wirklich???
  if (penaltyArea.is_inside(position)) { // im strafraum kein mindestabstand
    LineSegment leftLineOfPenaltyArea(Vec(-field.penalty_area_width/2.+200, 
                                      -field.field_length / 2. - 500.),
    Vec(-field.penalty_area_width/2.+200, 
         -field.field_length / 2. + 
             field.penalty_area_length));
    LineSegment rightLineOfPenaltyArea(Vec(field.penalty_area_width/2.-200, 
                                       -field.field_length / 2. - 500.),
    Vec(field.penalty_area_width/2.-200, 
        -field.field_length / 2. + 
            field.penalty_area_length));
    LineSegment topLineOfPenaltyArea(Vec(-field.penalty_area_width/2., 
                                     -field.field_length / 2. + 
                                         field.penalty_area_length-200),
    Vec(field.penalty_area_width/2., 
        -field.field_length / 2. + 
            field.penalty_area_length-200));
      
    vector<Vec> intersections = intersect(topLineOfPenaltyArea, 
                                          Line(ballpos, position));
    if (intersections.size() == 0) {
      if (ballpos.x < 0) {
        intersections = intersect(leftLineOfPenaltyArea, 
                                  Line(ballpos, position));
      }
      else {
        intersections = intersect(rightLineOfPenaltyArea,
                                  Line(ballpos, position));
      }
    }
    if (intersections.size() == 0) { // assert there is an intersection
      JERROR ("No intersections with penalty area found. This is an "
          "error!");
      return position;
    }
      
    return position = intersections[0];      
  }

  if ((WBOARD->getStandardSituationRole() == WhiteBoard::STANDARD_ROLE_AB ||
       WBOARD->getStandardSituationRole() == WhiteBoard::STANDARD_ROLE_ABCDE) &&
      position.y < -field.field_length / 2 + 250.) 
  { // Breite des Roboters ausnutzen, aber nur, wenn es nur 1
    position.y = -field.field_length / 2 + 250.;
  } // stuermer gibt. sonst rammt der untere den oberen stuermer!
  return position;
}

Vec BPreOpponentStandardSituation::getPositionSafety(const Time& t, double d)
{  // Der Safety steht im Moment mal einfach IM Strafraum
  const FieldGeometry& field = MWM.get_field_geometry();
  Vec protectPos(ballpos.x<0?field.goal_width/2. -400.:-field.goal_width/2.+400., -field.field_length/2.); 
  Vec blockline = ballpos - protectPos;
  Vec target(protectPos + blockline.normalize() * field.penalty_area_length);
  if (blockGoal) {
    target =  Vec(protectPos +((ballpos-protectPos).normalize() * (field.goal_area_length + 1200.))); // etwas vor dem Tor
  } // Ende spezielle MINHO situation
  if (target.y < -field.field_length/2.+700.) {   // Torwart nicht stoeren
    target.y=-field.field_length/2.+700.;
  }
  return target;
}

Vec BPreOpponentStandardSituation::getPositionBlock(
    const Time& t, int pos, double d, bool blockGoalOppGoalKick)
{
  const FieldGeometry& field = MWM.get_field_geometry();
  double x = 650.;
/*  if(ballpos.x > 0 && POS_LEFT == pos){
    x = 750.;
  }
  if(ballpos.x <= 0 && POS_LEFT != pos){
    x = 750.;
  }*/

  //Wenn der Ball in einer der eigenen Ecke liegt sollten die Robies nicht
  //kolledieren // BLUB: DAZU MÜSSTE MAN DIE ECKEN ERSTMAL GROESSER MACHEN; DA DER BALL
                // AUF DIE ENTFERNUNG MEIST AUSSERHALB DES SPIELFELDES GESEHEN WIRD.
  Triangle rechts = Triangle(Vec(field.field_width/2+500., 
                                 -field.field_length/2-500.),
                             Vec(field.field_width/2+500., 
                                 -field.field_length/4+500.),
                             Vec(0.,-field.field_length/2-500.));

  Triangle links =  Triangle(Vec(-field.field_width/2-500.,
                                 -field.field_length/2-500.), 
                             Vec(-field.field_width/2-500., 
                                 -field.field_length/4+500.),
                             Vec(0,-field.field_length/2-500.));

  Vec target = ballpos;
  if (rechts.is_inside(ballpos)){
    XYRectangle corner(Vec(0,-field.field_length/2-500.), 
                       Vec(field.field_width/2+500.,
                           -field.field_length/2-500. + field.penalty_area_length));
    LOUT << "\% red dotted ";
    rechts.draw(LOUT);
    LOUT << "\n";
    if(POS_LEFT == pos){
      target = Vec(field.field_width/2, -field.field_length/4);
    }else if(corner.is_inside(ballpos) && POS_RIGHT == pos){
      target = Vec(field.field_width/2, -field.field_length/2 +
      field.penalty_area_length);
    }
  } else if (links.is_inside(ballpos)){
    XYRectangle corner(Vec(0,-field.field_length/2-500.), 
                       Vec(-field.field_width/2-500.,
                           -field.field_length/2 + field.penalty_area_length));
    LOUT << "\% red dotted ";
    links.draw(LOUT);
    LOUT << "\n";
    if(POS_LEFT != pos){
      target = Vec(-field.field_width/2, -field.field_length/4);
    }else if(corner.is_inside(ballpos) && POS_LEFT == pos){
      target = Vec(-field.field_width/2, -field.field_length/2 +
      field.penalty_area_length);
    }
  }

  Vec protectGoalPos =
      Vec(0., -field.field_length/2.) +
      (pos == POS_LEFT ? Vec(-x, 0.) : Vec(x, 0.));

  Vec position((protectGoalPos-target)*0.33);
  if (position.length() < (d +1000. + 110. + 250.)) { // zu nah dran?
    position = position.normalize() * (d + 1000. + 110. + 250.);
  }
  position += target;

  LOUT << "\% blue thick cross "<<position<<"\n";

  XYRectangle penaltyArea(Vec(-field.penalty_area_width/2., 
                              -field.field_length / 2. 
                              - 1500 - d), // hinter gl
                          Vec(field.penalty_area_width/2.,
                              -field.field_length / 2. +
                              field.penalty_area_length));
  


  
  // Option: Das Tor mit einem Feldspieler blockieren (bei fuenf Spielern steht
  //         der Safety, bzw. Rolle E schon im Tor, deshalb in diesem Fall an der
  //         Strafraumgrenze blocken)
  if (blockGoal && ballpos.y > -field.field_length/2. + 1500.) {
    LineSegment topLine(Vec(-field.penalty_area_width/2. -300, 
                        -field.field_length/2. +
                            field.penalty_area_length ),
                        Vec(field.penalty_area_width/2. + 300,
                            -field.field_length/2. +
                            field.penalty_area_length ));
    if (MWM.get_game_state().refstate != preOpponentGoalKick) {
      if (WBOARD->getActiveRobots() < 5 && ((ballpos.x < -1500 && pos == POS_RIGHT) ||
           (ballpos.x > 1500 && pos == POS_LEFT)) && ballpos.y > -field.field_length/2. + 1500.) {
        return Vec(protectGoalPos + Vec(0.,field.goal_area_length+300.)); // etwas vor dem Tor
      }
      else {
        vector<Vec> intersections = 
        intersect (topLine, LineSegment(protectGoalPos, ballpos));
        if (intersections.size() > 0) {
          return intersections[0];
        }
        else if (ballpos.y > -field.field_length/2. + field.penalty_area_length + 300) {
          Vec blockline = ballpos - protectGoalPos;
          return Vec(protectGoalPos + blockline.normalize() * 1200.);
        }
      }
    }
  } // Ende spezielle MINHO situation
  // else:
  
  // grosser Block: if (penaltyArea.is_inside(position))
  XYRectangle penaltyAreaBroader(
      Vec(-field.penalty_area_width/2. - 300, -field.field_length / 2. - 300), // hinter gl
      Vec(field.penalty_area_width/2. + 300, -field.field_length / 2. + 300 + field.penalty_area_length));
  if (penaltyArea.is_inside(position)) {
    Vec posAlternative = (protectGoalPos-ballpos).normalize() * (d + 110. + 250.) + ballpos;
    if (!penaltyAreaBroader.is_inside(posAlternative)) {
      position = posAlternative;
    } else {
      Circle circle(ballpos, d + 110. + 250.);
      LOUT << "\% thin red line" << circle << endl;
    
      int sgn = pos ==POS_LEFT ? -1 : 1;

      // Checke Schnittpunkte mit erlaubten Aussengrenzen und suche den 
      // auf meiner seite aus
      vector<LineSegment> lines;
      lines.push_back(LineSegment(
          Vec(sgn * (field.penalty_area_width/2. + 300), -field.field_length/2.-300),
          Vec(sgn * (field.penalty_area_width/2. + 300), -field.field_length/2. + field.penalty_area_length + 300)));
      lines.push_back(LineSegment(
          Vec(sgn * (field.goal_area_width/2. + 200), -field.field_length/2. - 300.),
          Vec(sgn * (field.field_width/2. + 400), -field.field_length/2. - 300.)));
      lines.push_back(LineSegment(
          Vec(-field.penalty_area_width/2. -300, -field.field_length/2. + field.penalty_area_length + 300.),
          Vec(field.penalty_area_width/2. + 300, -field.field_length/2. + field.penalty_area_length + 300.)));
        
      LOUT << "\% thin yellow line" << lines[0] << endl;
      LOUT << "\% thin yellow line" << lines[1] << endl;
      LOUT << "\% thin yellow line" << lines[2] << endl;

      for (unsigned int i=0; i < lines.size(); i++) {
        vector<Vec> intersections = 
            intersect(lines[i], circle);
        if (intersections.size() > 0) {
          double bestx = sgn * intersections[0].x;
          LOUT << "\% thin yellow cross" << intersections[i] << endl;
          int best = 0;
          for (unsigned int j=1; j < intersections.size(); j++) {
            LOUT << "\% thin yellow cross" << intersections[i] << endl;
            if (sgn * intersections[j].x > bestx) {
              bestx = fabs(intersections[j].x);
              best = j;
            }
          }
          position = intersections[best];
          LOUT << "\% thick red cross" << position << endl;
          break;
        }
      }
    }
  } // ENDE if (penaltyArea.is_inside(position))
  
  return position;
}


void BPreOpponentStandardSituation::setTargetEvadeDirection (
    Vec tg, Vec tghd, int prefside, bool is_ball)
{
  if (is_ball) {
    skill->force_target ();
    return;
  }
    // berechne, in welche Richtung Roboter ausweichen soll, wenn Zielposition belegt
  double ypen = 0.5*MWM.get_field_geometry().field_length-MWM.get_field_geometry().penalty_area_length-2000;
  double ypen2 = 0.5*MWM.get_field_geometry().field_length-MWM.get_field_geometry().penalty_area_length;
  double xpen2 = 0.5*MWM.get_field_geometry().penalty_area_width-500;
  if ((tg.y<-ypen-500) || (tg.y<-ypen && targetEvadeDirArea!=1)) {
      // sich nach eigenem Strafraum orientieren
    if (tg.x<-xpen2 && (tg.y<-ypen2-200 || (tg.y<-ypen2 && targetEvadeDirArea!=2))) {
      targetEvadeDir = tghd.angle()+Angle::quarter;
      targetEvadeDirArea=3;
    } else if (tg.x>xpen2 && (tg.y<-ypen2-200 || (tg.y<-ypen2 && targetEvadeDirArea!=2))) {
      targetEvadeDir = tghd.angle()-Angle::quarter;
      targetEvadeDirArea=4;
    } else {
      if (tghd.angle().in_between (Angle::three_quarters, Angle::deg_angle (80))) {
        targetEvadeDir = tghd.angle()+Angle::quarter;
      } else if (tghd.angle().in_between (Angle::deg_angle (100), Angle::three_quarters)) {
        targetEvadeDir = tghd.angle()-Angle::quarter;
      }
      targetEvadeDirArea=2;
    }
  } else {
      // sich nach links/rechts-Praeferenz orientieren
    if (prefside==POS_LEFT) {
      targetEvadeDir = tghd.angle()+Angle::quarter;
    } else {
      targetEvadeDir = tghd.angle()-Angle::quarter;
    }
    targetEvadeDirArea=1;
  }

  skill->force_target ();  // diese ganze Methode hier macht im Moment also mal gar nix!
//  skill->set_target_evade_strategy (targetEvadeDir);
}
