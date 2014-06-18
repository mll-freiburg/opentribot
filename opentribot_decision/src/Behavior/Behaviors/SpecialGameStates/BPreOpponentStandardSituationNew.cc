#include "BPreOpponentStandardSituationNew.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Structures/Journal.h"
#include "../../../Fundamental/stringconvert.h"
#include <cmath>
#include <vector>

// Abstand Robotermittelpunkt zur Penalty Area
#define PA_DIST 300.
#define GA_DIST 500.

using namespace Tribots;
using namespace std;

// Diese Implementierung der Standardsituationen rechnet die Positionierung
// der anderen Spieler durch, um Kollisionen der Wunschpositionen zu erkennen
// und Ausweichpositionen zu berechnen. Unschoen ist, dass diese Implentation
// dafuer Kenntnisse benoetigt, welche Rollen die anderen Roboter haben. 
// Daher ist sie vom Feldspieler07 abhaengig und verwendet hier implizites 
// Wissen ueber die Rollenzuweisung in Abhaengigkeit von der Spieleranzahl.
// TODO: Das sollte spaeter mal bereinigt werden.

BPreOpponentStandardSituationNew::BPreOpponentStandardSituationNew(bool blockGoal)
  : Behavior("BPreOpponentStandardSituationNew"),
  skill(new SPhysGotoPosAvoidObstacles()),
  blockGoal(blockGoal),
  lookForBall(false), noMinDistanceInPA(false)
{
  ballposknown.set_sec(-100);
  mindestballabstand2=2000;
  robotspeed=1.8;
  skill->set_ball_as_obstacle(true, true);
}

BPreOpponentStandardSituationNew::~BPreOpponentStandardSituationNew() throw ()
{
  delete skill;
}

void BPreOpponentStandardSituationNew::updateTactics (const TacticsBoard& tb) 
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
    if (d>=100 && d<=4100)
      mindestballabstand2=d;
  }

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
  LOUT << " MindestBallAbstand=" << mindestballabstand2 << endl;
}

bool BPreOpponentStandardSituationNew::checkCommitmentCondition(const Time& t) throw()
{
  return
    (MWM.get_game_state().refstate == preOpponentThrowIn ||
     MWM.get_game_state().refstate == preOpponentGoalKick ||
     MWM.get_game_state().refstate == preOpponentCornerKick ||
     MWM.get_game_state().refstate == preOpponentFreeKick ||
     MWM.get_game_state().refstate == preOpponentKickOff ||
     MWM.get_game_state().refstate == preDroppedBall);
}

bool BPreOpponentStandardSituationNew::checkInvocationCondition(const Time& t) 
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

void BPreOpponentStandardSituationNew::gainControl(const Time&) 
    throw(TribotsException) 
{
  ballposknown.set_sec(-10);
}

void BPreOpponentStandardSituationNew::setBallPos (const Time& t)
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
      // Ball nicht weiter als 40cm von Mittelpunkt entfernt
      if (ballpos.x>400) ballpos.x=400;
      if (ballpos.x<-400) ballpos.x=-400;
      if (ballpos.y<-400) ballpos.y=-400;
      if (ballpos.y>400) ballpos.y=400;
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

DriveVector
BPreOpponentStandardSituationNew::getCmd(const Time& t) throw(TribotsException)
{
  setBallPos (t);
  LOUT << "% dark_red solid cross " << ballpos << '\n';
  const RobotLocation& robot = MWM.get_robot_location(t);
//  const FieldGeometry& field = MWM.get_field_geometry();
  
  Vec position(0.,0.);
  
  double d = 
    (MWM.get_game_state().refstate == preDroppedBall ? 
     1000. : mindestballabstand2) + toleranz;

  if ( MWM.get_game_state().refstate == preOpponentKickOff ) {
    d = MWM.get_field_geometry().center_circle_radius + toleranz;
  }

  switch (WBOARD->getStandardSituationRole()) { // welche position hat der Roboter?
    case WhiteBoard::STANDARD_ROLE_AB:
    case WhiteBoard::STANDARD_ROLE_ABCDE:
      position = getPositionAtBall(t, POS_CENTER, d, blockGoal);
      break;
    case WhiteBoard::STANDARD_ROLE_A:
      position = getPositionAtBall(t, POS_LEFT, d, blockGoal);
      break;
    case WhiteBoard::STANDARD_ROLE_B: 
      position = getPositionAtBall(t, POS_RIGHT, d, blockGoal); 
      break;
    case WhiteBoard::STANDARD_ROLE_C:
    case WhiteBoard::STANDARD_ROLE_CE:
      position = getPositionBlock(t, POS_LEFT, d, blockGoal);
      break;
    case WhiteBoard::STANDARD_ROLE_D:
    case WhiteBoard::STANDARD_ROLE_DE:
      position = getPositionBlock(t, POS_RIGHT, d, blockGoal);
      break;
    case WhiteBoard::STANDARD_ROLE_CD: // Dieser Fall darf nicht eintreten!
      position = getPositionBlock(t, POS_CENTER, d, blockGoal);
      break;
    case WhiteBoard::STANDARD_ROLE_E:
    case WhiteBoard::STANDARD_ROLE_CDE:
      position = getPositionSafety(t, d, blockGoal);
    break;
  }
  skill->force_target();
  skill->set_dynamics ((robot.pos-position).length() < 300. ? robotspeed/4. : robotspeed);
  skill->init (position, ballpos-position, true);
  LOUT << "% thick solid yellow cross " << position << " word " 
       << (position+Vec(0.,500.)) << " target pos" << endl;
  LOUT << "% thin dotted yellow line " << position << " " 
       << (position+(ballpos-position).normalize() * 1500.) << endl;
  return skill->getCmd(t);
}





Vec BPreOpponentStandardSituationNew::getPositionAtBall(const Time& t, int pos,
      double d, bool blockGoal, bool draw)
{
  const FieldGeometry& field = MWM.get_field_geometry();
  Vec ownGoal = Vec(0., -field.field_length/2.);
  Vec position(ownGoal-ballpos);

  // Einer der Stuermer (wenn mehrere) muss den Ball abdecken und hat dabei
  // Prioritaet vor den anderen Spielern. Dies ist immer der Stuermer, 
  // auf dessen Seite der Ball ist.
  bool hasAtBallPriority = 
    (pos == POS_CENTER) ||
    (pos == POS_LEFT && ballpos.x < 500.) ||
    (pos == POS_RIGHT&& ballpos.x >= 500.);

  // Standardposition berechnen
  position = position.normalize() * (d + 110.);
  position += ballpos;          // Standardposition direkt vor dem Ball.

  // Position anpassen, wenn dieser Spieler nicht der Spieler mit Prioritaet ist.
  if (!hasAtBallPriority) {
    // Stelle diesen Roboter etwas weiter hinten und zur Mitte als den Roboter
    // mit Prioritaet auf.
    position += 
      ((ownGoal-ballpos).normalize()) * 700. +   
      ((ownGoal-ballpos).normalize()) * 350. * 
        (pos==POS_LEFT ? -Angle::quarter : Angle::quarter);
  }

  // Nun gucken, ob der Spieler als Spion aufgestellt werden muss. Ein
  // Stuermer wird nicht zum Spion, wenn er die Prioritaet am Ball hat,
  // diesen also unbedingt selbst abdecken muss.
  bool istSpion = false;
  if (lookForBall && !hasAtBallPriority) {
    if (pos == POS_LEFT &&     // linker stuermer
        ballpos.y > -field.field_length/2.-100. && // ball im feld
        MWM.get_game_state().refstate != preOpponentKickOff &&
        ballpos.x > 500.) {   // ball rechts
      position = ballpos+
        ((Vec(0,
              -field.field_length/2.)-ballpos).normalize()*(d+110.)).
              rotate(-Angle::quarter);
      istSpion = true;
    } // Ende Spioncode fuer linken spieler
    else if (pos == POS_RIGHT &&
             ballpos.x < 500. &&                        // Ball links
             ballpos.y > -field.field_length/2-100. &&  // und im Feld
             MWM.get_game_state().refstate != preOpponentKickOff) {
      position = ballpos+
       ((Vec(0, 
             -field.field_length/2.)-ballpos).normalize()*(d+110.)).
             rotate(Angle::quarter);
      istSpion = true;
    } // Ende Spioncode fuer alle Faelle AUSSER KickOff Gegner
    else if(pos == POS_RIGHT &&
            MWM.get_game_state().refstate == preOpponentKickOff) {
      // position nur um ein sechstel drehen, da man in eigener haelfte bleiben
      // muss.
      position = (Vec(0,-1.)*(d+120.)).rotate(Angle::sixth);
      istSpion = true;
    } // Ende Spioncode fuer den Fall KickOff Gegner
  }

  // Nun muss die Position noch angepasst werden, falls sie im eigenen Strafraum 
  // liegen sollte. Das braucht nicht gemacht werden, wenn dieser Spieler
  // reinfahren soll. Das ist der Fall, wenn er Prioritaet am Ball hat und kein
  // Safety da ist (der Safety hat im Strafraum Vorrang).
  XYRectangle penaltyArea(
      Vec(-field.penalty_area_width/2. - PA_DIST, 
          -field.field_length / 2. - 2500.), // hinter GL
      Vec(+field.penalty_area_width/2. + PA_DIST, 
          -field.field_length / 2. + field.penalty_area_length+ PA_DIST));
  XYRectangle goalArea(
      Vec(-field.goal_area_width/2. - GA_DIST, 
          -field.field_length / 2. - 2500.), // hinter GL
      Vec(+field.goal_area_width/2. + GA_DIST, 
          -field.field_length / 2. + field.goal_area_length+ GA_DIST));  

  if (penaltyArea.is_inside(position) &&  
      (hasAtBallPriority && WBOARD->getActiveRobots() < 5 &&
       WBOARD->getActiveRobots() != 2)) {
    // Roboter soll als einziger in den Strafraum. Nun die lange Ecke abdecken.
    Vec protectPos = Vec(ballpos.x >= 500.? -650. : 650., -field.field_length/2.);
    position = (protectPos-ballpos).normalize()*(d+110.) + ballpos;

    if (goalArea.is_inside(position)) {
      // linke Seitenlinie der GA
      LineSegment leftLineOfGA(
        Vec(-field.goal_area_width/2. - GA_DIST, 
            -field.field_length / 2. - 2500.),
        Vec(-field.goal_area_width/2. - GA_DIST, 
            -field.field_length / 2. + field.goal_area_length + GA_DIST));

      // rechte Seitenlinie der GA
      LineSegment rightLineOfGA(
        Vec(+field.goal_area_width/2. + GA_DIST, 
            -field.field_length / 2. - 2500.),
        Vec(+field.goal_area_width/2. + GA_DIST, 
            -field.field_length / 2. + field.goal_area_length + GA_DIST));

      // obere Linie der GA
      LineSegment topLineOfGA(
        Vec(-field.goal_area_width/2. - GA_DIST, 
            -field.field_length / 2. + field.goal_area_length + GA_DIST),
        Vec(+field.goal_area_width/2. + GA_DIST, 
            -field.field_length / 2. + field.goal_area_length + GA_DIST));

      if (draw) {
        LOUT << "% yellow thin solid line " << leftLineOfGA << endl;
        LOUT << "% yellow thin solid line " << topLineOfGA << endl;
        LOUT << "% yellow thin solid line " << rightLineOfGA << endl;
      }
      LineSegment ballLine(protectPos, ballpos);
      if (draw) LOUT << "% grey thin solid line " << ballLine << endl;
      vector<Vec> intersections = intersect(topLineOfGA, ballLine);
      if (!intersections.size()) {
        intersections = intersect(leftLineOfGA, ballLine);
      }
      if (!intersections.size()) {
        intersections = intersect(rightLineOfGA, ballLine);
      }
      if (!intersections.size()) {
        if (draw) LOUT << "FEHLER: Keinen Schnittpunkt gefunden!" << endl;
      }
      else {
        position = intersections[0];
      }
    } // Ende: Roboter in der Goalarea
  }
  else if (penaltyArea.is_inside(position) &&  
           !(hasAtBallPriority && WBOARD->getActiveRobots() < 5 &&
             WBOARD->getActiveRobots() != 2)) { 
    // linke Seitenlinie der PA
    LineSegment leftLineOfPenaltyArea(
      Vec(-field.penalty_area_width/2. - PA_DIST, 
          -field.field_length / 2. - 2500.),
      Vec(-field.penalty_area_width/2. - PA_DIST, 
          -field.field_length / 2. + field.penalty_area_length + PA_DIST));

    // rechte Seitenlinie der PA
    LineSegment rightLineOfPenaltyArea(
      Vec(+field.penalty_area_width/2. + PA_DIST, 
          -field.field_length / 2. - 2500.),
      Vec(+field.penalty_area_width/2. + PA_DIST, 
          -field.field_length / 2. + field.penalty_area_length + PA_DIST));

    // obere Linie der PA
    LineSegment topLineOfPenaltyArea(
      Vec(-field.penalty_area_width/2. - PA_DIST, 
          -field.field_length / 2. + field.penalty_area_length + PA_DIST),
      Vec(+field.penalty_area_width/2. + PA_DIST, 
          -field.field_length / 2. + field.penalty_area_length + PA_DIST));

    if (draw) {
      LOUT << "% yellow thin solid line " << leftLineOfPenaltyArea << endl;
      LOUT << "% yellow thin solid line " << topLineOfPenaltyArea << endl;
      LOUT << "% yellow thin solid line " << rightLineOfPenaltyArea << endl;
    }

    // Zuerst versuchen, den Robter auf den Schnittpunkt mit der 
    // Strafraumgrenze zu setzen. Dies kann schon helfen, wenn es 
    // sich um den Stuermer handelt, der versetzt hinter dem vorderen
    // Stuermer steht.
    if (draw) LOUT << "% grey thin dotted line " << ballpos 
                   << " " << position << endl;
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
      if (draw) LOUT << "No intersections with penalty area found. This is an "
                     << "error!" << endl;
    } 
    else {
      position = intersections[0];
      if (draw) LOUT << "% yellow thin solid cross " << position << endl;
    }

    // Nun muss aber noch ueberprueft werden, ob diese Position nicht ev.
    // zu Nah am Ball ist
    if (!noMinDistanceInPA && (position-ballpos).length() < d+110.) {
      // Regelkonforme Alternativposition
      Circle minCirc(ballpos,d+110.);
      if (draw) LOUT << "% grey thin dotted circle " << minCirc << endl;

      // Schnittpunkte berechnen
      vector<Vec> intersections = intersect(topLineOfPenaltyArea, minCirc);
      vector<Vec> tmp = intersect (leftLineOfPenaltyArea, minCirc);
      intersections.insert(intersections.end(), tmp.begin(), tmp.end());
      tmp = intersect (rightLineOfPenaltyArea, minCirc);
      intersections.insert(intersections.end(), tmp.begin(), tmp.end());

      if (draw) {
        for (unsigned int i=0; i < intersections.size(); i++) {
          LOUT << "% yellow thin solid cross " << intersections[i] << endl;
        }
      }

      // Schnittpunkte auswerten. Es muesste genau zwei geben (einer ist 
      // theoretisch auch moeglich, wenn Kreis Linie nur beruehrt.):
      if (intersections.size() != 2) {
        if (draw) LOUT << "FEHLER: Erwartete 2 Schnittpunkte, es wurden aber " 
                       << intersections.size() << " gefunden!" << endl;
        if (intersections.size() > 0) {
          position = intersections[0];
          if (draw) LOUT << "% yellow thin solid circle " 
                         << position << " 250." << endl;
        }
      }
      else {   // Es wurden zwei Schnittpunkte gefunden
        if (pos == POS_CENTER) {  // fuer POS_CENTER den nehmen, der weiter oben
          if (intersections[0].y > intersections[1].y || // oder naeher in der mitte
              fabs(intersections[0].x) < fabs(intersections[1].x)) { // ist
            position = intersections[0];
          }
          else {
            position = intersections[1];
          }
        }
        else {   // fuer POS_LEFT und POS_RIGHT Schnittpunkte verteilen
          int select = 0;  // zuerst fuer linken Spieler auswaehlen
          if (intersections[0].y == intersections[1].y) {
            if (intersections[0].x > intersections[1].x) {
              select = 1;
            }
          }
          else {  // 0.y != 1.y
            if ((intersections[0].y > intersections[1].y)) { // jetzt nehme ich die
              select = 1;                                    // niedrigere
            }
            if (intersections[select].x > 0) { // rechts muss ich aber die 
              select = !select;                // hoehere position nehmen
            }
          }
          if (pos == POS_RIGHT) { // fuer den rechten spieler einfach den
            select = !select;     // anderen der beiden schnittpunkte nehmen
          }
          position = intersections[select];
        }
        if (draw) LOUT << "% yellow thin circle " << position << " 250." << endl;
      }
    }
    if (position.y < -field.field_length/2.-500.) {
      position.y = -field.field_length/2.-500.;
    }
  } // Ende Berechnung Alternativposition bei Strafraumnaehe

  if ((pos == POS_CENTER) &&
      position.y < -field.field_length / 2 + 250.) 
  { // Breite des Roboters ausnutzen, aber nur, wenn es nur 1
    position.y = -field.field_length / 2 + 250.;
  } // stuermer gibt. sonst rammt der untere den oberen stuermer!

  // Nun muss geschaut werden, ob diese Position mit der eines anderen Spielers
  // kollidiert.
  if (!hasAtBallPriority) {
    Vec otherPos = 
      getPositionAtBall(t, (pos == POS_LEFT ? POS_RIGHT : POS_LEFT), d, blockGoal,  
                        false);
    if ((otherPos-position).length() < 750.) {
      LOUT << "Anpassung der Zielpositon aufgrund von Konflikt "
           << "mit priorisiertem Stuermer vorgenommen." << endl;
      LOUT << "% red thin solid cross " << otherPos << endl;
      LOUT << "% orange thin solid cross " << position << endl;
      if (position == otherPos) { // Einfache Loesung fuer den Fehlerfall
        LOUT << "Warnung: position==otherPosition. Darf nicht passieren!" << endl;
        position = otherPos + Vec(pos==POS_LEFT? -700.:700., -350.);
      }
      else {
        if ((pos == POS_LEFT  && position.x < otherPos.x) ||
            (pos == POS_RIGHT && position.x > otherPos.x)) { 
          //Weiter in die aktuelle Richtung verschieben. Hoffentlich Regelkonform.
          position = otherPos+(position-otherPos).normalize()*750.;
        }
        else { // Linker Roboter steht rechts neben rechtem Roboter!
               // Das kann in Strafraumnaehe passieren, wenn einer der beiden
               // Roboter in den Strafraum fahren darf. In disem Fall wird eine
               // andere Alternative verwendet.
          LOUT << "Wunschpositionen der Stuermer waeren vertauscht. "
               << "Berechne Alternativposition zur Mitte." << endl;
          Vec alt = ((otherPos-Vec(0,-field.field_length/2.)).normalize()*750.);
          position = otherPos + 
            alt.rotate(pos==POS_LEFT ? Angle::quarter : Angle::three_quarters);
          LOUT << "% yellow thin solid line " << otherPos 
               << " " << position << endl;
        }
      }
    }
  }
  return position;
}

Vec BPreOpponentStandardSituationNew::getPositionSafety(const Time& t, double d, 
                                                        bool blockGoal, bool draw)
{  // Der Safety darf grundsaetzlich immer in den Strafraum rein, hat aber die 
   // niedrigste Ausweichprioritaet. Weicht nach hinten Richtung Tor aus.

  // Standardmaessig deckt er die lange Ecke ab und bleibt 
  // dabei innerhalb des Strafraums.
  const FieldGeometry& field = MWM.get_field_geometry();
  Vec protectPos(ballpos.x<500?+field.goal_width/2.-400. :
                               -field.goal_width/2.+400., -field.field_length/2.);
  Vec blockline = ballpos - protectPos;
  Vec target(protectPos + blockline.normalize() * field.penalty_area_length);

  // Wenn er explizit das Tor zustellen soll oder der Ball sehr Nah 
  // am Strafraum liegt, dann muss er noch weiter hinten stehen.
  if (blockGoal || 
      ballpos.y<-field.field_length/2.+field.penalty_area_length+d+110.) {

    // linke Seitenlinie der GA
    LineSegment leftLineOfGA(
      Vec(-field.goal_area_width/2. - GA_DIST, 
          -field.field_length / 2. - 2500.),
      Vec(-field.goal_area_width/2. - GA_DIST, 
          -field.field_length / 2. + field.goal_area_length + GA_DIST));

    // rechte Seitenlinie der GA
    LineSegment rightLineOfGA(
      Vec(+field.goal_area_width/2. + GA_DIST, 
          -field.field_length / 2. - 2500.),
      Vec(+field.goal_area_width/2. + GA_DIST, 
          -field.field_length / 2. + field.goal_area_length + GA_DIST));

    // obere Linie der GA
    LineSegment topLineOfGA(
      Vec(-field.goal_area_width/2. - GA_DIST, 
          -field.field_length / 2. + field.goal_area_length + GA_DIST),
      Vec(+field.goal_area_width/2. + GA_DIST, 
          -field.field_length / 2. + field.goal_area_length + GA_DIST));

    if (draw) {
      LOUT << "% yellow thin solid line " << leftLineOfGA << endl;
      LOUT << "% yellow thin solid line " << topLineOfGA << endl;
      LOUT << "% yellow thin solid line " << rightLineOfGA << endl;
    }
    LineSegment ballLine(protectPos, ballpos);
    if (draw) LOUT << "% grey thin solid line " << ballLine << endl;
    vector<Vec> intersections = intersect(topLineOfGA, ballLine);
    if (!intersections.size()) {
      intersections = intersect(leftLineOfGA, ballLine);
    }
    if (!intersections.size()) {
      intersections = intersect(rightLineOfGA, ballLine);
    }
    if (!intersections.size()) {
      if (draw) LOUT << "FEHLER: Keinen Schnittpunkt gefunden!" << endl;
    }
    else {
      target = intersections[0];
    }

  } // Ende spezielle "Torblockade"

  // Nun muss geschaut werden, ob diese Position mit der eines anderen Spielers
  // kollidiert.
  vector<Vec> otherPos;
  if (WBOARD->getActiveRobots() == 2) {
    otherPos.push_back(getPositionAtBall(t, POS_CENTER, d, blockGoal, false));
  }
  else {
    otherPos.push_back(getPositionAtBall(t, POS_LEFT, d, blockGoal, false));
    otherPos.push_back(getPositionAtBall(t, POS_RIGHT, d, blockGoal, false));
    otherPos.push_back(getPositionBlock(t, POS_LEFT, d, blockGoal, false));
    otherPos.push_back(getPositionBlock(t, POS_RIGHT, d, blockGoal, false));
  }  // Nur in diesen beiden Faellen gibt es einen Safety.

  for (unsigned int p=0; p < otherPos.size(); p++) {
    while ((otherPos[p]-target).length() < 750.) {
      if (draw) {
        LOUT << "% red thin solid cross " << otherPos[p] << endl;
        LOUT << "% orange thin solid cross " << target << endl;
      }
      target = target + (protectPos-target).normalize()*100.;
    }
  }
  return target;
}

Vec BPreOpponentStandardSituationNew::getPositionBlock(
    const Time& t, int pos, double d, bool blockGoalOppGoalKick, bool draw)
{
  if (pos == POS_CENTER) {
    LOUT << "% POS_CENTER nicht implementiert! Sollte auch nicht vorkommen. "
         << "verwende POS_LEFT" << endl;
    pos = POS_LEFT;
  }
  const FieldGeometry& field = MWM.get_field_geometry();
  bool isOppKickOff = MWM.get_game_state().refstate == preOpponentKickOff;
  bool istSpion = (lookForBall &&
                   ballpos.y > -field.field_length/2. + 500 &&
                   WBOARD->onlyThreeRobots() &&
                   ((!isOppKickOff && 
                     ((pos == POS_LEFT  && ballpos.x > 700.) ||
                      (pos == POS_RIGHT && ballpos.x <= 400.))) ||
                    (isOppKickOff && pos == POS_RIGHT))); 
  Vec protectPos(pos==POS_LEFT ? -field.goal_width/2.+350. :
                                 +field.goal_width/2.-350.,
                 -field.field_length/2.);  

  // Standardposition berechnen
  Vec position((protectPos-ballpos)*0.33);
  if (position.length() < (d + 1250. + 110. + 250.)) { // zu nah dran?
    position = position.normalize() * (d + 1250. + 110. + 250.);
  }
  position += ballpos;

  // Nun die Position berechnen, falls dieser Roboter als Spion dient
  if (istSpion) {
    if(!isOppKickOff) {
      position = ballpos+
        ((Vec(0, -field.field_length/2.)-ballpos).normalize()*
          (d+110.)).rotate(pos==POS_LEFT ? -Angle::quarter : Angle::quarter);
    }
    else { // gegenerischer Anstoss
      position = ballpos+
        ((Vec(0, -field.field_length/2.)-ballpos).normalize()*
          (d+110.)).rotate(Angle::sixth);
    }
  } // Ende Spioncode
  if (draw) LOUT << "\% blue thin solid cross " << position << "\n";

  // Wenn der Ball im Strafraum ist oder blockGoal angeschaltet ist,
  // Ausweichposition berechnen.
  // Variante 1: Schnittpunkt der Balllinie
  // Variante 2: Wenn Schnittpunkt zu nah am Ball: Kreis.
  XYRectangle penaltyArea(Vec(-field.penalty_area_width/2. - PA_DIST, 
                              -field.field_length / 2. - 2500), // hinter gl
                          Vec(+field.penalty_area_width/2. + PA_DIST,
                              -field.field_length / 2. +
                              field.penalty_area_length + PA_DIST));
  vector<LineSegment> lines;
  double sgn = ballpos.x > 0 ? 1. : -1.;
  lines.push_back(LineSegment(
    Vec(sgn * (field.penalty_area_width/2. + PA_DIST), 
               -field.field_length/2. - PA_DIST),
    Vec(sgn * (field.penalty_area_width/2. + PA_DIST), 
        -field.field_length/2. + field.penalty_area_length + PA_DIST)));
  lines.push_back(LineSegment(
    Vec(sgn * field.goal_area_width/2., -field.field_length/2. - PA_DIST),
    Vec(sgn * (field.penalty_area_width/2. + PA_DIST),
        -field.field_length/2. - PA_DIST)));
  lines.push_back(LineSegment(
    Vec(-field.penalty_area_width/2. - PA_DIST, 
        -field.field_length/2. + field.penalty_area_length + PA_DIST),
    Vec(+field.penalty_area_width/2. + PA_DIST,
        -field.field_length/2. + field.penalty_area_length + PA_DIST)));

  if ((blockGoal && ballpos.y < field.field_length/6. && !istSpion) ||
      penaltyArea.is_inside(position)) {
    if (draw) {
      for (unsigned int i=0; i < lines.size(); i++) {
        LOUT << "% yellow thin solid line " << lines[i] << endl;
      }
    }
    if (draw) LOUT << "% grey thin dotted line " 
                   << Line(protectPos, position) << endl;
    vector<Vec> intersections;
    for (unsigned int i=0; i < lines.size(); i++) {
      vector<Vec> tmp = intersect(lines[i], Line(protectPos, position));
      if (tmp.size()) {
        intersections.insert(intersections.end(), tmp.begin(), tmp.end());
      }
    }
    if (draw) {
      for (unsigned int i=0; i < intersections.size(); i++) {
        LOUT << "% yellow thin cross " << intersections[i] << endl;
      }
    }
    if (!intersections.size()) {
      if (draw) LOUT << "Genau ein Schnittpunkt erwartet aber " 
                     << intersections.size() << " gefunden." << endl;
    }
    else {
      position = intersections[0];
    }

    // Nun muss noch ueberprueft werden, ob die Position weit genug
    // vom Ball entfernt ist. Andernfalls Position auf Kreis suchen.
    double minDist = d+110.+250.;
    if ((ballpos-position).length() < d + 110. + 250.) { 
      if (draw) LOUT << "% grey thin dotted circle " 
                     << Circle(ballpos, minDist) << endl;
      intersections.clear();
      for (unsigned int i=0; i < lines.size(); i++) {
       vector<Vec> tmp = intersect(lines[i], Circle(ballpos, minDist));
        if (tmp.size()) {
          intersections.insert(intersections.end(), tmp.begin(), tmp.end());
        }
      }
      if (draw) {
        for (unsigned int i=0; i < intersections.size(); i++) {
          LOUT << "% yellow thin cross " << intersections[i] << endl;
        }
      }
      if (!intersections.size()) {
        if (draw) LOUT << "FEHLER: Kein Schnittpunkt mit Kreis gefunden!" << endl;
      }
      else if (intersections.size() == 1) {
        if (draw) LOUT << "FEHLER: Zwei Schnittpunkte erwartet aber nur " 
                       << intersections.size() << " gefunden " << endl;
        position = intersections[0];
      }
      else {
        if (draw && intersections.size() > 2) {
          if (draw) LOUT << "FEHLER: Zwei Schnittpunkte erwartet aber " 
                         << intersections.size() << " gefunden. " 
                         << " Verwende nur die ersten beiden." << endl;
        }
        int select = 0;  // zuerst fuer linken Spieler auswaehlen
        if (intersections[0].y == intersections[1].y) {
          if (intersections[0].x > intersections[1].x) {
            select = 1;
          }
        }
        else {  // 0.y != 1.y
          if ((intersections[0].y > intersections[1].y)) { // jetzt nehme ich die
            select = 1;                                    // niedrigere
          }
          if (intersections[select].x > 0) { // rechts muss ich aber die hoehere nehmen
            select = !select;
          }
        }
        if (pos == POS_RIGHT) { // fuer den rechten spieler einfach den
          select = !select;     // anderen der beiden schnittpunkte nehmen
        }
        position = intersections[select];
      }
      if (draw) LOUT << "% yellow thin circle " << position << " 250." << endl;
    }
  }

  // Nun muss geschaut werden, ob diese Position mit der eines anderen Spielers
  // kollidiert.
  vector<Vec> otherPos;
  if (WBOARD->getActiveRobots() == 3) {
    otherPos.push_back(getPositionAtBall(t, POS_CENTER, d, blockGoal, false));
    if (pos == POS_RIGHT) {
      otherPos.push_back(getPositionBlock(t, POS_LEFT, d, blockGoal, false));
    }
  }
  else if (WBOARD->getActiveRobots() >= 4) {
    otherPos.push_back(getPositionAtBall(t, POS_LEFT, d, blockGoal, false));
    otherPos.push_back(getPositionAtBall(t, POS_RIGHT, d, blockGoal, false));
    if (pos == POS_RIGHT) {
      otherPos.push_back(getPositionBlock(t, POS_LEFT, d, blockGoal, false));
    }
  }  // Nur in diesen beiden Faellen gibt es Rollen C/D.

  for (unsigned int p=0; p < otherPos.size(); p++) {
    Vec moveUnitDir = (otherPos[p]-Vec(0.,-field.field_length/2.)).rotate(
      pos==POS_LEFT?Angle::quarter:-Angle::quarter).normalize();
    while ((otherPos[p]-position).length() < 750.) {
      if (draw) {
        LOUT << "% red thin solid cross " << otherPos[p] << endl;
        LOUT << "% orange thin solid cross " << position << endl;
      }
      position = position + moveUnitDir*100.;
    }
  }
  return position;
}
