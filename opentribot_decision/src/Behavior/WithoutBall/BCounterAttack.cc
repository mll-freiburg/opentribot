#include "BCounterAttack.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/Frame2D.h"
#include "../../../Fundamental/random.h"
#include "../../../Fundamental/geometry.h"
#include "../../Predicates/freeCorridor.h"
#include <vector>
#include <cmath>
#include <sstream>

namespace Tribots {
  
  using namespace std;
 
  BCounterAttack::BCounterAttack(int maxDuration) 
  : Behavior("BCounterAttack"), activated(Time()), maxDuration(maxDuration), driveVel(2.3), 
    dash(0), gotopos(0), intention(0)
  {
  	dash = new SBoostToPos();
  	gotopos = new SGoToPosEvadeObstaclesOld();
  }
  
  BCounterAttack::~BCounterAttack() throw ()
  {
  	if (dash) delete dash;
  	if (gotopos) delete gotopos;
  }
  
	void BCounterAttack::updateTactics(const TacticsBoard& tb) throw()
  {
    if (tb[string("AllgemeineFahrgeschwindigkeit")]==string("schneller")) {
      driveVel = 3.0;
    }
    else if (tb[string("AllgemeineFahrgeschwindigkeit")]==string("langsamer")) {
    	driveVel = 2.0;
    }
    else if (tb[string("AllgemeineFahrgeschwindigkeit")]==string("langsam")) {
      driveVel = 1.5;
    }
    else { // normal und Rest
      driveVel = 2.3;
    }

    LOUT << "BCounterAttack tactics changed. driveVel=" << driveVel << endl;
  }
  
  bool BCounterAttack::checkCommitmentCondition(const Time& t) throw()
  { 
    const FieldGeometry& field = MWM.get_field_geometry();
    return
      activated.diff_msec(t) < maxDuration &&
      MWM.get_robot_location(t).pos.y < field.field_length/2.-field.penalty_area_length;
  }
  
  bool BCounterAttack::checkInvocationCondition(const Time& t) throw()
  {
    const FieldGeometry& field = MWM.get_field_geometry();
    if (WBOARD->doPossessBall(t)) { // nur, wenn gerade kein Ballbesitz
      return false;
    }
    return 
      MWM.get_robot_location(t).pos.y < field.field_length/2.-field.penalty_area_length;
  }
  
  void BCounterAttack::gainControl(const Time& t) throw()
  {
    activated = t;
    intention = 0;
  }
  
  void BCounterAttack::loseControl(const Time& t) throw()
  {
    if (intention) intention->loseControl(t);
  }
  
  DriveVector BCounterAttack::getCmd(const Time& t) throw(TribotsException)
  {
    const RobotLocation& robot = MWM.get_robot_location(t);
    const BallLocation& ball = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry();
    const ObstacleLocation& obst = MWM.get_obstacle_location(t);			

    Vec target(robot.pos.x, field.field_length/2.-field.penalty_area_length);
    if (activated.diff_msec(t) > 2000) {  // in den ersten 2 Sekunden erstmal gerade aus losfahren
      if ((robot.pos-ball.pos.toVec()).length() < 2500.) {
      	target = ball.pos.toVec();
      }
      else if (target.x-500. > ball.pos.x && ball.velocity.x < +.5) { // ball rollt nicht auf den roboter zu
        target.x = ball.pos.x;
      }
      else if (target.x+500. < ball.pos.x && ball.velocity.x > -.5) { // ball rollt nicht auf den roboter zu
        target.x = ball.pos.x;
      }
    }
    dash->setTargetPos(target);

    Skill* newIntention = dash;  // standardmaessig dash
    double mindist =             // in Fahrtrichtung schauen, dabei nur Gegner beruecksichtigen
      obstacle_distance_to_line_inside_field (robot.pos , 
                                              robot.pos+((target-robot.pos).normalize()*2500.),
                                              obst, true );	  // ignoriere Mitspieler
    double mindistNear =         // im Nahbereich auch Mitspieler berücksichtigen
      obstacle_distance_to_line_inside_field (robot.pos , 
                                              robot.pos+((target-robot.pos).normalize()*800.),
                                              obst, false);   // beruecksichtige Mitspieler
    if (mindist < 600. || mindistNear < 500. ||
        fabs((target-robot.pos).angle(Vec(0,.1)*robot.heading).get_deg_180()) > 10.) 
    { // gotopos Skill fuer Annaehrung verwenden und hier initialisieren
      newIntention = gotopos;
      // Nun berechnen, welche Roboterseite am naechsten an gewuenschter Richtung
      // dran ist und diese zum Ziel ausrichten. Wichtig, um genauen Dash zu
      // ermoeglichen.
      Vec presentDir = Vec(0.,1.) * robot.heading; // Blickrichtungsvek. in Weltkoords
      Vec targetDir = target-robot.pos;
      Vec dir2 = targetDir; dir2.rotate_sixth().rotate_sixth();
      Vec dir3 = dir2; dir3.rotate_sixth().rotate_sixth();
      double minAngle = fabs(targetDir.angle(presentDir).get_deg_180());
      double tmp=0.;
  
      if (minAngle > (tmp=fabs(dir2.angle(presentDir).get_deg_180()))) {
        targetDir = dir2;
        minAngle = tmp; 
      }
      if (minAngle > (tmp=fabs(dir3.angle(presentDir).get_deg_180()))) {
        targetDir = dir3;
        minAngle = tmp;
      }
      LOUT << "% yellow thin dotted line " << robot.pos << " " 
           << (robot.pos+targetDir.normalize()*2000.) << endl;
      gotopos->init(target, driveVel, targetDir);
    }
 
    if (intention != newIntention) {  // nun den eigentlichen Wechsel der Intention 
      if (intention != 0) intention->loseControl(t);   // vornehmen
      intention = newIntention;   
      intention->gainControl(t);
    }
 
    if (intention == dash && dash->checkInvocationCondition(t)) {
      LOUT << "Benutze dash Skill" << endl;
    }
    else if (intention == gotopos) {
      LOUT << "Benutze gotopos Skill" << endl;
    }
    LOUT << "% yellow thick circle " << Circle(target,40) << endl;

    return intention->getCmd(t);
  }
}
