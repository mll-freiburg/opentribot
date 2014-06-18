#include "BDribbleBallStraightToGoalEvadeSidewards.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>
#include "../../Predicates/freeCorridor.h"
#include <stdlib.h>
namespace Tribots {
  
using namespace std;
  
enum {LEFT = 0, RIGHT};
  
BDribbleBallStraightToGoalEvadeSidewards::BDribbleBallStraightToGoalEvadeSidewards(double transVel,
                                                                                   const Area* activationArea) 
: Behavior("BDribbleBallStraightToGoalEvadeSidewards"), 
  doDribbleToMiddleFirst(false), 
  transVel(transVel),
  skill(new SDribbleBallStraightToPosEvadeSidewards()), 
  considerObstaclesInGoal(true),
  obstacles_behind_goal_distance(300.0)
{
  if (activationArea) {
    this->activationArea = activationArea->clone();
  }
  else {
    const FieldGeometry& field =
    MWM.get_field_geometry();
    
    // X: Mindestabstand von 1000mm zur Seitenlinie
    // Y: Mindestabstand von 1000mm zur gegnerischen Grundlinie
    this->activationArea =
      new XYRectangle(Vec(-(field.field_width / 2. - 1000.),
                          field.field_length / 2. - 1500.),
                      Vec(field.field_width / 2. - 1000.,
                          -field.field_length / 2.));
  }
}

BDribbleBallStraightToGoalEvadeSidewards::
~BDribbleBallStraightToGoalEvadeSidewards() throw ()
{
  delete activationArea;
  delete skill;
}

void BDribbleBallStraightToGoalEvadeSidewards::updateTactics (const TacticsBoard& tb) throw ()
{
  string key = "SidewardsFahrgeschwindigkeit";
  if (tb[key] == string("schneller")) {
    transVel = 2.3;
  }
  else if (tb[key] == string("langsamer")) {
    transVel = 1.8;
  }
  else if (tb[key] == string("langsam")) {
    transVel = 1.5;
  }
  else {
    transVel = 2.0;
  }

  string val="";
  val = tb[string("obstacles_behind_goal_distance")];
  if (val!="") {
    obstacles_behind_goal_distance = atof(val.c_str());
    considerObstaclesInGoal = obstacles_behind_goal_distance > 101. ? true : false;
  }
  
  LOUT << "obstacles_behind_goal_distance "
       << obstacles_behind_goal_distance << "\n";

  LOUT << "BDribbleBallStraightToGoalEvadeSidewards Taktikupdate transvel="
       << transVel << endl;
  
  if (tb[string("ErstInDieMitteDribbeln")] == "an") {
    doDribbleToMiddleFirst = true;
  }
  else {
    doDribbleToMiddleFirst = false;
  }

	skill->updateTactics(tb);
}

bool 
BDribbleBallStraightToGoalEvadeSidewards::
checkCommitmentCondition(const Time& t) throw()
{
  // 1. ausgehen, wenn keinen ball mehr
  if (! WBOARD->doPossessBall(t)) {
    return false;
  }
  
  Frame2d world2robot =  WBOARD->getAbs2RelFrame(t);
  FieldGeometry const& fgeom= MWM.get_field_geometry();
  Vec oppGoalPos(0., fgeom.field_length / 2.);  
  
  // 2. ausgehen, wenn im Torraum
  if (MWM.get_robot_location(t).pos.y > fgeom.field_length / 2. - fgeom.goal_area_length - 400. &&
      fabs(MWM.get_robot_location(t).pos.x) < fgeom.goal_area_width / 2.) {
    return false;
  }
  
  // 3. ausgehen, wenn roboter nicht mehr richtung tor schaut
  double dir_deg_to_goal= 
    ((world2robot*oppGoalPos).angle() - Angle::quarter ).get_deg_180();
  if  ( fabs( dir_deg_to_goal ) > 45 ) {
    return false;
  }
  return true;    
}

bool 
BDribbleBallStraightToGoalEvadeSidewards::
checkInvocationCondition(const Time& t) throw()
{
  if (! WBOARD->doPossessBall(t)) {       // kein ballbesitz
    return false;
  }
  if (! activationArea->is_inside(MWM.get_ball_location(t).pos.toVec())) {// ausserhalb
    return false;                        // des aktivierungsbereichs
  }
  
  const FieldGeometry& fgeom= MWM.get_field_geometry();
  const RobotLocation& robot = MWM.get_robot_location(t);
  
  // Winkel zum Tor berechnen
  Frame2d world2robot =  WBOARD->getAbs2RelFrame(t);  
  Vec oppGoalPos(0., fgeom.field_length / 2.);  
  double dir_deg_to_goal= ((world2robot*oppGoalPos).angle() - Angle::quarter ).get_deg_180();
  
  if (doDribbleToMiddleFirst) {  // kleinerer aktivierungsbereich und genauerer winkel
    if (robot.pos.y < fgeom.field_length/9. &&
        fabs(robot.pos.x) > fgeom.center_circle_radius) {
      return false;
    }
    if  ( fabs( dir_deg_to_goal ) >20 ) { // im kegel? schmaler als CC!
      return false;
    }    
  }
  
  if  ( fabs( dir_deg_to_goal ) > 25 ) { // im kegel? schmaler als CC!
    return false;
  }
  return true; 
}

DriveVector 
BDribbleBallStraightToGoalEvadeSidewards::getCmd(const Time& t) 
  throw(TribotsException) 
{
  FieldGeometry const& fgeom= MWM.get_field_geometry();
  RobotLocation const& robot = MWM.get_robot_location(t); 
  ObstacleLocation const& obstacles=MWM.get_obstacle_location(t);
  
  // Switchen der anzufahrenden Ecke, wenn zu weit nach aussen abgetrieben
  if(robot.pos.x > 1500. && corner == LEFT)  {  // \note: testhalber geaendert. schien ein bug zu sein, hat nicht zurueck in die mitte gefuehrt.
    corner = RIGHT;
  }
  else if (robot.pos.x < -1500. && corner == RIGHT) {
    corner = LEFT;
  }  
  // Switchen, wenn in Mitte vor Tor und ausgewaehlte Ecke nicht frei
  else if (! changed && robot.pos.y > fgeom.field_length / 2. - 4000. &&
           ((corner == LEFT && robot.pos.x < 0) ||
            (corner == RIGHT && robot.pos.x > 0))) {
    Vec presentTarget((corner == LEFT ? -1.:1.) * (fgeom.goal_width/2. - 230.), 
                      fgeom.field_length / 2. + (considerObstaclesInGoal ? 230. : 0.));   
    if ((!considerObstaclesInGoal && 
	 obstacle_distance_to_line_inside_field(robot.pos+Vec(0., 200.), 
						presentTarget, obstacles) < 300.) ||
	(considerObstaclesInGoal && 
	 obstacle_distance_to_line(robot.pos+Vec(0., 200.), 
				   presentTarget, obstacles) < 300.)) 
      {
      corner = corner == LEFT ? RIGHT : LEFT;
      changed = true;
      LOUT << "Changed target due to blocked shooting line." << endl;
    }
  }
  Vec target((corner == LEFT ? -1. : 1.) * (fgeom.goal_width/2. - 300.), 
             fgeom.field_length / 2.);   
  Vec pointTo = target; // Standardmaessig dorthin zielen, wohin gefahren wird
  
  // Calculate pointing direction
  // Vorgehen:
  // 1. Korridorbreiten fuer Positionen in 10cm Abstand auf der Torgrundlinie
  //    berechnen.
  // 2. Schauen, welche am naechsten zur Anfahrtszielposition ist und diese 
  //    Waehlen, wenn dort genug Platz fuer Schuss vorhanden ist.
  // 3. Ansonsten freieste Position waehlen. Position wechseln, wenn
  //    a) freier werdende Nachbarschaftskette zu neuer Position (nicht ueber
  //       den Torwart wechseln)
  //    b) ueber Nachbarschaftskette keine ausreichend freie Position erreicht 
  //       werden kann und diese Position ist freier.
  Vec left(-fgeom.goal_width / 2. + 300., fgeom.field_length / 2.);
  Vec right(fgeom.goal_width / 2. - 300., fgeom.field_length / 2.);
  int n = static_cast<int>((right-left).length() / 100.);
  Vec step = (right-left).normalize() * 100.;
  vector<double> distances (n+1);
  int min_id=0; double min_d=0.;
  for (int i=0; i <= n; i++) {
    Vec pos = left+(step * i);
    if (!considerObstaclesInGoal) {
      distances[i] = obstacle_distance_to_line_inside_field(robot.pos, pos,
							    obstacles);
    }
    else {
      Vec posInsideGoal = pos + Vec(pos-robot.pos).normalize() * obstacles_behind_goal_distance;
      distances[i] = obstacle_distance_to_line(robot.pos, posInsideGoal,
					       obstacles);		
      LOUT << "\% thin white dotted " << Circle(posInsideGoal, 50) << endl;
    }
    LOUT << "\% white solid" << Circle(pos, 50) << endl;
    if (i == 0 || Vec(left+(step*i) - target).length() < min_d) { // ber. 2.
      min_id = i;
      min_d = (Vec(left+(step*i)) - target).length();
    }
  }
  
  if (distances[min_id] > 300.) {  // 2.: target ist frei ?
    pointTo = target;
    presentPointToPos = min_id;
    LOUT << "Point to target, it is free enough" << endl;
  } else {                         // 3.: wohin wechseln ?
    int newPointToR, newPointToL;
    for (newPointToR = presentPointToPos; newPointToR < n; newPointToR++) {
      if (distances[newPointToR+1] < distances[newPointToR]) {
        break;
      }
    }
    for (newPointToL = presentPointToPos; newPointToL > 0; newPointToL--) {
      if (distances[newPointToL-1] < distances[newPointToL]) {
        break;
      }
    }
    if (distances[newPointToL] > 200. || distances[newPointToR] > 200.) { //3a
      presentPointToPos = distances[newPointToL] > distances[newPointToR] ? newPointToL : newPointToR;
      LOUT << "Swing to more open position" << endl;
    }
    else {                         // 3b
      int max_id=0;
      for (int i = 0; i <= n; i++) {
        if (i == 0 || distances[i] > distances[max_id]) {
          max_id = i;
        }
      }
      if (distances[max_id] - distances[presentPointToPos] > 20.) {
        presentPointToPos = max_id;
        LOUT << "Switch to more open position" << endl;
      }
      else {
        LOUT << "Did not find a better corridor, stay with present decision." 
             << endl;
      }
    }  
  }
  // jetzt pruefen, ob man mehr in die Mitte des Tores Zielen kann, ohne unter
  // 300mm Abstand zu kommen:
  int dir = presentPointToPos < n/2 ? 1 : -1;
  for (int i=presentPointToPos; i != n/2; i += dir) {
    if (distances[i] > 250.) {
      presentPointToPos = i;
    }
    else {
      break;
    }
  }
  pointTo = left + (step * presentPointToPos);    // auf diese Position zielen
  
  
  // falls das tor ganz frei ist nicht genau in die mitte zielen:
  int isFree = true;
  for (int i=0; i < n; i++) {
    if (distances[i] < 330.) {
      isFree = false; break;
    }
  }
  if (isFree && robot.pos.y > fgeom.field_length/4.) pointTo = (robot.pos.x < 0. ? left + Vec(200.,0.) : right-Vec(200.,0)); // wenn tor frei, immer nach links zielen
  
  skill->setTarget(target, pointTo, true);
  skill->setTransVel(transVel);
  return skill->getCmd(t);
}

void 
BDribbleBallStraightToGoalEvadeSidewards::gainControl(const Time& t) 
  throw(TribotsException)
{
  const RobotLocation& robot = MWM.get_robot_location(t);
  // Auswahl der anzufahrenden Ecke. Aussen die kurze Ecke bevorzugen, in 
  // Spielfeldmitte vor dem Tor kreuzen
  if (robot.pos.x < -1000) {
    corner = LEFT;
  }
  else if (robot.pos.x > 1000) {
    corner = RIGHT;
  }
  else if (robot.pos.x > 0) {
    corner = LEFT;
  }
  else {
    corner = RIGHT;
  }
  changed = false;
  presentPointToPos = 0;
  skill->gainControl(t);
}

void 
BDribbleBallStraightToGoalEvadeSidewards::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
}
    

};
