
 
#include "BPreOwnIndirectStandardSituation.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Structures/Journal.h"
#include <vector>
#include <cmath>
#include "../../../Fundamental/random.h"
#include "../../Predicates/freeCorridor.h"
#include "../../../Fundamental/stringconvert.h"
   
namespace Tribots {
  
  using namespace std;
  static string fieldplayer_role = "ball";

  void BPreOwnIndirectStandardSituation::areasPrintln(){    
    for (unsigned int i=0; i < areas.size(); i++) {
      LOUT << "\% red thick dotted" << areas[i] << "\n";
    }
  }
  void BPreOwnIndirectStandardSituation::areaPrintln(XYRectangle area){    
    LOUT <<"\% blue thick dotted " << area << "\n";        
  }

  XYRectangle 
  BPreOwnIndirectStandardSituation::getAreaForAreaID(int areaID){
    return areas[areaID];
  }
      
  int BPreOwnIndirectStandardSituation::getAreaID(const Time& t){
    const BallLocation& ball = MWM.get_ball_location(t);
    for (unsigned int i=0; i < areas.size(); i++) {
      if(areas[i].is_inside(ball.pos.toVec())) {
	      return i;
      }
    }
    return -1;          
  }

  //Konstanten fuer die Areas Blick auf das gegnerische Tor
  const int NOAREA = -1;
  const int EIGENEHAELFTELINKS = 6;
  const int EIGENEHAELFTERECHTS = 7;
  const int GEGNERISCHEECKERECHTS = 2;
  const int GEGNERISCHEECKELINKS = 0;
  const int SEITERECHTS = 5;
  const int SEITELINKS = 3;
  const int MITTELINKS = 1;
  const int MITTERECHTS = 4;
  const int EIGENEHAELFTESEITELINKS = 8;
  const int EIGENEHAELFTESEITERECHTS = 9;
  
  bool isLinks(int areaID){
    return (areaID == EIGENEHAELFTELINKS) ||
           (areaID == GEGNERISCHEECKELINKS) ||
           (areaID == SEITELINKS) ||
           (areaID == EIGENEHAELFTESEITELINKS) ||
           (areaID == MITTELINKS);
  }
  bool isEcke(int areaID){
     return (areaID == GEGNERISCHEECKERECHTS) ||
            (areaID == GEGNERISCHEECKELINKS);
  }
  bool isOwnHalf(int areaID){
      return (areaID == EIGENEHAELFTELINKS) ||
             (areaID == EIGENEHAELFTERECHTS) ||
             (areaID == EIGENEHAELFTESEITELINKS) ||
             (areaID == EIGENEHAELFTESEITERECHTS);
  }
  bool isOwnMiddle(int areaID){
	  return (areaID == EIGENEHAELFTERECHTS ||
		  areaID == EIGENEHAELFTELINKS);
  }
  bool isMitte(int areaID){
      return (areaID == MITTERECHTS) || 
             (areaID == MITTELINKS);
  }
  BPreOwnIndirectStandardSituation::BPreOwnIndirectStandardSituation(
                         int shortPassKickDuration, 
		                 int longPassKickDuration, 
                          int throwInKickDuration,
                         double standardPassProbability,
				         double standardDribbling, int standardPositioning)
    : Behavior("BPreOwnIndirectStandardSituation"), 
      goToPos(new SPhysGotoPosAvoidObstacles()),
      stop(new BEmergencyStop()), reached(false), doStandard(false), 
      kickPosition(false), atPosition(false), finished(false),
      _kick(false), _decided(false), _ownTurn(false), passFirstPhase(true),
      counter(0), area(-1),shortPassKickDuration(shortPassKickDuration),
      longPassKickDuration(longPassKickDuration), 
      standardPassProbability(standardPassProbability), 
      standardDribbling(standardDribbling), 
      standardPositioning(standardPositioning),
      standardAbstandBlock(2300),
      volleyProbability(1.0),
      doDynamicBlock(1), throwInKickDuration(throwInKickDuration)
  {
     goToPos->set_ball_as_obstacle (true, true);
     const FieldGeometry& fgeom = MWM.get_field_geometry();
     areas = vector<XYRectangle>(10);
     areas.at(GEGNERISCHEECKERECHTS) = 
       XYRectangle(Vec(fgeom.field_width/6., 
			  	             fgeom.field_length*2 / 6.),
			             Vec(fgeom.field_width / 2 + 500, 
				               fgeom.field_length / 2 + 500));
     areas.at(GEGNERISCHEECKELINKS) = 
       XYRectangle(Vec(-fgeom.field_width/6., 
			            	    fgeom.field_length*2/ 6.),
			             Vec(-fgeom.field_width / 2 - 500, 
				                fgeom.field_length / 2 + 500));
     areas.at(SEITELINKS) = 
       XYRectangle(Vec(-fgeom.field_width/6., 
				               -1000),
			             Vec(-fgeom.field_width / 2 - 500, 
				               fgeom.field_length*2/6 + 500));
     areas.at(SEITERECHTS) = 
       XYRectangle(Vec(fgeom.field_width/6., 
			         	       -1000),
			             Vec(fgeom.field_width / 2 + 500, 
				               fgeom.field_length*2 / 6 + 500));
     areas.at(EIGENEHAELFTELINKS) =  
         XYRectangle(Vec(500,
                         -fgeom.field_length / 2 - 500.),
			               Vec(-fgeom.field_width *2/ 6 - 500,
                         -500));
     areas.at(EIGENEHAELFTERECHTS) =  
         XYRectangle(Vec(0,
                         -fgeom.field_length / 2 - 500.),
			               Vec(fgeom.field_width *2/ 6 + 500,
                         -500)); 
     areas.at(MITTERECHTS) =  
         XYRectangle(Vec(0,
                         -1000),
		                 Vec(fgeom.field_width / 6 + 500, 
			                   fgeom.field_length / 2 + 500));
     areas.at(MITTELINKS) =  
         XYRectangle(Vec(500,
                         -1000),
			               Vec(-fgeom.field_width / 6 - 500, 
			                   fgeom.field_length / 2 + 500));
     areas.at(EIGENEHAELFTESEITELINKS) = 
         XYRectangle(Vec(-fgeom.field_width*2/6., 
				                 -fgeom.field_length/2 - 500),
			               Vec(-fgeom.field_width / 2 - 500, 
				                 -500));
     areas.at(EIGENEHAELFTESEITERECHTS) = 
         XYRectangle(Vec(fgeom.field_width*2/6., 
				                 -fgeom.field_length/2 - 500),
			               Vec(fgeom.field_width / 2 + 500, 
				                 -500));
     goalKickDirect = false;
     ownHalfAllDirect = false;
  }

  
  BPreOwnIndirectStandardSituation::~BPreOwnIndirectStandardSituation() throw ()
  {
    delete goToPos;
    delete stop;
  }
  
  
  void 
  BPreOwnIndirectStandardSituation::gainControl(const Time& t) throw()
  {
    stuck = false;
    passFirstPhase = true;
    finished = false;
    reached = false;
    kickPosition = false;
    doStandard = false;
    atPosition = false;
    counter = 0;
    area = getAreaID(t);
    _decided = false;
    _kick = false;
    _ownTurn = false;
    originalRefstate = errorState;
    passMessageSent = false;
    checkedCorridor = false;
    breakKickProcedure = false;
    dangerousOpponentPosition = Vec (1e300, 1e300);
    doVolley = (urandom()<volleyProbability);
  }

  bool 
  BPreOwnIndirectStandardSituation::checkCommitmentCondition(const Time& t) 
    throw()
  {
    const BallLocation& ball = MWM.get_ball_location(t);
    if(MWM.get_game_state().refstate == preOwnThrowIn  || // noch vor 
       MWM.get_game_state().refstate == preOwnGoalKick || // Ausfuehrung
       MWM.get_game_state().refstate == preOwnFreeKick ||
       MWM.get_game_state().refstate == preOwnKickOff  ||
       MWM.get_game_state().refstate == preOwnCornerKick) {
      if (ball.pos_known == BallLocation::unknown ||
          ball.pos_known == BallLocation::raised) {
        return false;
      }
       
      return true;
    }

    if (MWM.get_game_state().refstate == freePlay &&  // nach spaetestens 15s
	     t.diff_sec(start) > 15) {                  // normal weiterspielen!
      LOUT << "BPreOwnIndirectStandardSituation::checkCommitmentCondition: "
	   << "Verhalten zu lange aktiv geblieben!" << endl; 
      return false;                                // (zur Sicherheit)
    }

    if (doStandard && ball.velocity.toVec().length() > .2 
            && MWM.get_game_state().refstate == freePlay
            && atPosition && 0){
       LOUT << "Ball beruehrt(1)!" <<endl;
       MWM.get_message_board().publish("touchedBall:");
       return false; 
    }
    
    if (WBOARD->getStandardSituationRole() ==      // wenn alleine auf dem feld
	     WhiteBoard::STANDARD_ROLE_ABCDE &&           // aufhoeren, wenn "LOS"
	     MWM.get_game_state().refstate == freePlay && // und entweder an zielpos
	     (t.diff_sec(start) > 7 || atPosition)) {   // oder 7s verstrichen
      return false;
    }
    if (doStandard) {  // Spezialfall für ankickenden / einwerfenden Spieler
      if (finished) {  // schon alles fertig
	    return false;
      }
      else if (MWM.get_game_state().refstate == freePlay && 
	       (t.diff_sec(start) < 7 || atPosition)) { // maximal 7s nach "LOS"
	    return true;        // um an Startpos zu kommen, beliebig viel danach 
      }
      else {
	return false;
      }
    }                                          
    if (MWM.get_game_state().refstate == freePlay &&   // maximal 8 Sekunden
	   t.diff_sec(start) < 8 &&                       // zur Ausfuehrung
	   ! WBOARD->receivePass() &&
           ! WBOARD->receiveSetPlayShortPass() &&
	   ! WBOARD->touchedBall()) {
      return true;
    }
    return false;    
  }


  bool 
  BPreOwnIndirectStandardSituation::checkInvocationCondition(const Time& t)
    throw()
  {
    const BallLocation& ball = MWM.get_ball_location(t);
    if (ball.pos_known == BallLocation::unknown ||
        ball.pos_known == BallLocation::raised) {
      return false;
    }
    return 
      MWM.get_game_state().refstate == preOwnThrowIn ||
      MWM.get_game_state().refstate == preOwnGoalKick ||
      MWM.get_game_state().refstate == preOwnFreeKick ||
      MWM.get_game_state().refstate == preOwnKickOff ||
      MWM.get_game_state().refstate == preOwnCornerKick;
  }
  
  //Freistossvariante Zwei Spieler sthen fast nebeneinander auf den Weg zum Tor
  Vec
  BPreOwnIndirectStandardSituation::getPositionForTwoPlayer(const Time& t){
    const FieldGeometry& field = MWM.get_field_geometry();
    const BallLocation& ball3d = MWM.get_ball_location(t);
    Vec ball(ball3d.pos.x, ball3d.pos.y);
    return Vec((area == MITTELINKS) ? 
	       field.field_width/4:field.field_width/4*-1,ball.y-500);
  }
  
  /*
   * Methode fuer die Position des freien Spielers
   * */
  Vec
  BPreOwnIndirectStandardSituation::getTargetForFreePlayer(const Time& t){
    const FieldGeometry& field = MWM.get_field_geometry();
    const BallLocation& ball3d = MWM.get_ball_location(t);
    //    const RobotLocation& robot = MWM.get_robot_location(t);
    Vec ball(ball3d.pos.x, ball3d.pos.y);
    switch(area){
      case GEGNERISCHEECKELINKS:
      case GEGNERISCHEECKERECHTS:
	      return Vec(0,field.field_length/2 - field.penalty_marker_distance);
      case MITTELINKS:
	    return Vec(field.field_width*2/6,ball.y);
      case MITTERECHTS:
        return Vec(-1*field.field_width*2/6,ball.y);
      case EIGENEHAELFTESEITELINKS:
      case SEITELINKS:
        return Vec(field.field_width*2/6,ball.y);
      case EIGENEHAELFTERECHTS:
	      return Vec(field.field_width/-6, ball.y); 
      case EIGENEHAELFTELINKS:
	      return Vec(field.field_width/6, ball.y);
      case EIGENEHAELFTESEITERECHTS:
      case SEITERECHTS:
      default:
        return Vec(-1*field.field_width*2/6,ball.y);
    }
  }

  DriveVector 
  BPreOwnIndirectStandardSituation::getCmd(const Time& t) 
  throw(TribotsException)
  {
    const BallLocation& ball = MWM.get_ball_location(t);
    if(ball.pos_known != BallLocation::unknown &&
      ball.pos_known != BallLocation::raised){
      stop->getCmd(t);
    }
  
    if(MWM.get_game_state().refstate != freePlay) { // setup timer for 
      start = t;                                    // additional positioning
      originalRefstate = MWM.get_game_state().refstate; // time during freeplay
    }
    cerr << "BPreOwnStandardSituation::getCmd originalrefstate"
         << originalRefstate << "\n\r";
    
    // update active area (with hysteresis)
    if (area == -1 || ! getAreaForAreaID(area).is_inside(ball.pos.toVec())) {
      area = getAreaID(t);
    }
    areaPrintln(getAreaForAreaID(area));
    areasPrintln();
    if (MWM.get_game_state().refstate != freePlay) {
      atPosition = false;
    }    
    if(! atPosition ){
      LOUT << "BPreOwnStandardSituation::getCmdBefore" << endl;
      cerr << "BPreOwnStandardSituation::getCmdBefore\n\r";
      if (!stuck && stuckBehavior.checkInvocationCondition(t)) {
	    stuck = true;
	    stuckBehavior.gainControl(t);
	    return stuckBehavior.getCmd(t);
      }
      else if (stuckBehavior.checkCommitmentCondition(t)) {
	    return stuckBehavior.getCmd(t);
      }
      else if (stuck) {
	    stuck = false;
	    stuckBehavior.loseControl(t);
      }
      return getCmdBefore(t);
    }else if (MWM.get_game_state().refstate == freePlay && doStandard) {
      LOUT << "BPreOwnStandardSituation::getCmdAfter" << endl;
      cerr << "BPreOwnStandardSituation::getCmdAfter\r\n";
      return getCmdAfter(t);
    }else{
      LOUT << "BPreOwnStandardSituation: At position waiting for freeplay." 
	       << endl;
      cerr << "BPreOwnStandardSituation: At position waiting for freeplay.\r\n";
      return stop->getCmd(t); 
    } 
  }

  DriveVector 
  BPreOwnIndirectStandardSituation::getCmdAfter(const Time& t) 
  throw(TribotsException)
  {
    DriveVector dv;
    dv.vrot = 0.;
    dv.kick = 0;
    
    static Time passTime;
    static Time tut;
    
    const RobotLocation& robot = MWM.get_robot_location(t);
    const BallLocation& ball3d = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry();
    const vector<TeammateLocation>& teammates = MWM.get_teammate_location();

    if (!kickPosition && ! WBOARD->doPossessBall(t)){// nahe Startpos anfahren
      if (t.diff_sec(start) > 5) { // wenn nicht schnell genug da, dann weiter
	    finished = true;  // mit freiem spiel im naechsten zyklus
	    return dv;
      }

      //letzter anfahrtppunkt
      Vec position;
      if(preOwnKickOff == originalRefstate){
        position = Vec(ball3d.pos.x -350,ball3d.pos.y-200);
      }else if((originalRefstate == preOwnGoalKick && goalKickDirect &&
      _ownTurn) || (ownHalfAllDirect && isOwnMiddle(area) && _ownTurn)){
        position = Vec(ball3d.pos.x,ball3d.pos.y -500);
      }else if(isOwnMiddle(area)){
        position = Vec(ball3d.pos.x + 500 * (isLinks(area)?1:-1),
                       ball3d.pos.y);
      }else if (isEcke(area)) { // bei Eckstoss anders
	      position = ball3d.pos.toVec()
                   + 600 * 
                      (ball3d.pos.toVec() - getTargetForFreePlayer(t)).normalize();
      }else if (isMitte(area)){
        position = Vec(ball3d.pos.x + (isLinks(area) ? 300 : -300),ball3d.pos.y+300);
      }else if (ball3d.pos.x >  field.field_width/2 - 100 || 
                ball3d.pos.x < -field.field_width/2 + 100){
        LOUT << "Ball liegt auf der Linie, muss näher ran" << endl;
        position = Vec(ball3d.pos.x + 350*   // standard
		                    (!(isLinks(area))?1:-1),
		                   ball3d.pos.y);
      }else{
        position = Vec(ball3d.pos.x + 500*   // standard
		                    (!(isLinks(area))?1:-1),
		                   ball3d.pos.y);
      }
      Vec target;
      if(originalRefstate == preOwnKickOff){
        target = Vec(800,455);
      }else if((originalRefstate == preOwnGoalKick && 
               goalKickDirect && _ownTurn) ||
                (ownHalfAllDirect && _ownTurn 
                 && isOwnMiddle(area))){
        target = Vec(ball3d.pos.x, 0);
      }else if(isOwnMiddle(area)){
        target = Vec(((isLinks(area)?-1:1)*field.field_width/2), ball3d.pos.y);
      }else if(isMitte(area)){
        target = Vec(ball3d.pos.x + (isLinks(area) ? -1000 : 1000), ball3d.pos.y-1000);      
      }else{
        target = getTargetForFreePlayer(t);
      }
      Vec targetHeading = ((_kick || _ownTurn)? target-robot.pos:(target-robot.pos).rotate_sixth().rotate_sixth()); // (MWM.get_robot_properties().kickers >= 2 ? target-robot.pos : ((_kick || _ownTurn)? target-robot.pos:(target-robot.pos).rotate_sixth().rotate_sixth()));
      LOUT << endl << "\% blue cross " << target.x <<" "<<target.y <<endl;
      LOUT << "Abstand: " << (position - robot.pos).length() << endl;
      if((position - robot.pos).length() < 100 && (targetHeading.angle()-robot.heading-Angle::quarter).in_between(-Angle::deg_angle(10), Angle::deg_angle(10))) {
        kickerEvadePosition  = position;
        kickPosition = true;
      }
      tut = t;
      goToPos->init(position, targetHeading, (_kick || _ownTurn), true);
      goToPos->set_dynamics(1.0);
      return goToPos->getCmd(t);
    } else if (kickPosition) {
      if (t.diff_sec(tut) < .33) return dv; // kurz anhalten
    }

    // hier wurde die kickposition erreicht oder aber der ballbesitzt
    // ist bereits angegangen 

    counter++;
    if(!doStandard){
      return dv;
    }else{ 
      if (_kick && !checkedCorridor) {
        LOUT << "% dark_red solid cross " << getTargetForFreePlayer(t) << " cross " << volleyPosition (t) << '\n';
        bool passWayOkay=true;
        checkedCorridor = true;
        Vec receiver = getTargetForFreePlayer(t);
        unsigned int robotSelfId = MWM.get_robot_id(); 

        int closest = -1;
        double closestDistance = 9999999.;
        // Suche potentiellen Langpassempfaenger
        for (unsigned int i=0; i < teammates.size(); i++) {
          if (teammates[i].number == robotSelfId ||                // selber
              fabs((double)teammates[i].timestamp.diff_msec(t)) > 2000.) {
            continue;
          }
          if ((teammates[i].pos-receiver).length() < closestDistance) {
            closestDistance = (teammates[i].pos-receiver).length();
            closest = i;
          }
        }
        if (closest>=0 && closestDistance<1500) {
          LOUT << "% circle " << teammates[closest].pos << " 200\n";
        }
        if ((closest == -1 || closestDistance>1500) && (area==SEITELINKS || area==SEITERECHTS)) {
          // kein Langpassempfaenger, suche moeglichen Volleyempfaenger
          Vec vpos = volleyPosition (t);
          int minIndex=-1;
          double minDistance=1e300;
          for (unsigned int i=0; i < teammates.size(); i++) {
            if (teammates[i].number != robotSelfId && abs(teammates[i].timestamp.diff_msec(t)) < 2000) {
              if ((teammates[i].pos-vpos).length() < minDistance) {
                minDistance = (teammates[i].pos-vpos).length();
                minIndex = i;
              }
            }
          }
          if (minIndex<0 || minDistance>1500) {
            passWayOkay=false;
            LOUT << "Keinen Langpassempfaenger gefunden\n";
          } else { // untersuche Volleypasslinie
            LOUT << "% circle " << teammates[minIndex].pos << " 200\n";
            Vec vbpos = vpos;
            vbpos.y=ball3d.pos.y;
            vbpos.x=vbpos.x+(vbpos.x<0 ? 200 : -200);  // damit der Volleyempfaenger nicht als Hindernis gesehen wird
            Vec bxpos=ball3d.pos.toVec();
            bxpos.x+=(bxpos.x<0 ? 1500 : -1500);
            LOUT << "% arrow " << bxpos << vbpos << '\n';
            double odist = obstacle_distance_to_line_inside_field (bxpos, vbpos, MWM.get_obstacle_location(t));
            if (odist<700) {
              LOUT << "Hindernisse auf der Passlinie zum Volleyspieler\n";
              passWayOkay=false;
            } else {
              LOUT << "Erwarte Langpass-Volleyabnahme\n";
            }
          }
        } else {  // untersuche Langpasslinie
          Vec bp1 = ball3d.pos.toVec()+(receiver-ball3d.pos.toVec()).normalize()*800.;
          Vec bp2 = receiver-(receiver-ball3d.pos.toVec()).normalize()*1500.;
          LOUT << "\% red solid " << Circle(teammates[closest].pos, 200) << endl; 
          // Occupancy grid auswerten
          int freeProximity= teammates[closest].occupancy_grid.occupied((ball3d.pos.toVec()-teammates[closest].pos).angle());
          if (freeProximity == 0) {
            LOUT << "\% white dotted "  << LineSegment(ball3d.pos.toVec(), teammates[closest].pos) << endl;
          }
          else if (freeProximity == 2) {
            LOUT << "\% yellow dotted " << LineSegment(ball3d.pos.toVec(), teammates[closest].pos) << endl;
          }
          else {
            LOUT << "\% red dotted "    << LineSegment(ball3d.pos.toVec(), teammates[closest].pos) << endl;
          }
          LOUT << "% arrow " << bp1 << bp2 << '\n';
          if (freeProximity != 0) { // Nahbereich belegt, kein Pass moeglich
            LOUT << "Umgebung des Langpassempfaengers nicht frei\n";
            passWayOkay=false;
          } else {  // untersuche Passlinie
            double dist = obstacle_distance_to_line_inside_field(bp1, bp2, MWM.get_obstacle_location(t));
            if (dist < 700) {
              LOUT << "Langpasslinie blockiert durch HIndernisse mit Abstand " << dist << "\n";
              passWayOkay=false;
            } else {
              LOUT << "Erwarte Langpass-Stopabnahme\n";
            }
          }
        }
        if (!passWayOkay) {
          LOUT << "Umgeschaltet auf kurzen Pass wegen fehlendem Empfaenger oder Hindernissen" << endl;
          _kick = false;
          breakKickProcedure = true;
        }
      }
      //wegen zu dollem kick rausgenommen      
      /*
      if (0 && originalRefstate == preOwnKickOff) {
        goToBall.setParameters(getTargetForFreePlayer(t)-robot.pos, 1.0);
	      dv = goToBall.getCmd(t);
        if (WBOARD->doPossessBall(t)){
          dv.kick = 1;
          dv.klength = shortPassKickDuration;
          passFirstPhase = false;
        }
        if(!passMessageSent && ((counter > 2 && counter%3 ==0))){
          MWM.get_message_board().publish("short_pass:");
          cerr << "-----anstoss Pass Quer\r\n";
          LOUT << "anstoss Pass Quer!" <<endl;     // nur einmal senden!
          passMessageSent = true;
        }
        if(!passFirstPhase && t.diff_msec(passTime)< 500){
          dv.vtrans = Vec(0.,0.);
        }else if(passFirstPhase){
          passTime = t;
        }else{
          finished = true;
        }
      }else */
      if(_ownTurn){// || isOwnHalf(area)){
        cerr << "Dribbling\r\n";
        LOUT << "Dribbling" << endl;
        Vec target;
        if((goalKickDirect && originalRefstate == preOwnGoalKick) ||
           (isOwnMiddle(area) && ownHalfAllDirect)){
          target = Vec(ball3d.pos.x,0);
        }else{
          target = getTargetForFreePlayer(t);
        }
        goToBall.setParameters(target-robot.pos, 1.5);
        dv = goToBall.getCmd(t);
        if (WBOARD->doPossessBall(t)){
          finished = true;
          LOUT << "Ball beruehrt(2)!" <<endl;
          MWM.get_message_board().publish("touchedBall:");
        }
      } else if (_kick){
        goToBall.setParameters(getTargetForFreePlayer(t)-robot.pos, 1.0);
        dv = goToBall.getCmd(t);
        if (WBOARD->doPossessBall(t)) {
          dv.kick = 2;
          dv.klength = longPassKickDuration;    
          passFirstPhase = false;
          LOUT << "Ball beruehrt(3)!" <<endl;
          MWM.get_message_board().publish("touchedBall:");
        }
        if(!passMessageSent && (finished || counter==5)){
          MWM.get_message_board().publish("pass:");
          cerr << "Langer Pass Quer\r\n";
          LOUT << "Langer Pass Quer!" <<endl;     // nur einmal senden!
          passMessageSent = true;
        }
        if(!passFirstPhase && t.diff_msec(passTime)< 500){
          dv.vtrans = Vec(0.,0.);
        }else if(passFirstPhase){
          passTime = t;
        }else{
          finished = true;
        }
      }
      else { // short pass mit anschieben/kicken
             // in der eigene haelfte will man nach aussen schieben daher nur dafuer ein anderes ziel
        LOUT << "Will Kurzpass spielen\n";
        Vec target;
        if(originalRefstate == preOwnKickOff){
          target = Vec(800,455);
        }else if(isOwnMiddle(area)){
           target = Vec(((isLinks(area)?-1:1)*field.field_width/2),
           ball3d.pos.y) - robot.pos;
	      }else if (isMitte(area)){
          target = Vec(ball3d.pos.x + (isLinks(area) ? -1000 : 1000)-robot.pos.x, ball3d.pos.y - 1000-robot.pos.y);
        }else{
          target = getTargetForFreePlayer(t)-robot.pos;
	      }
        LOUT << endl << "\% blue cross " << target.x <<" "<<target.y <<endl;
        LOUT << "\% thick blue line " << target << endl;
/*        if (MWM.get_robot_properties().kickers >= 2)
          goToPos->init(ball3d.pos.toVec(), target, true);
        else*/
          goToPos->init(ball3d.pos.toVec(), breakKickProcedure? target : target.rotate_sixth().rotate_sixth(), true);
        goToPos->set_dynamics (1.5);
        if(passFirstPhase){
          passFirstPhase = false;
          passTime = t;
        }
        LOUT << "Time passed: " << t.diff_msec(passTime) << '\n';
        bool doKick=false;
        if (/*WBOARD->doPossessBall(t) || t.diff_msec(passTime)>700 ||
            (MWM.get_robot_properties().kickers<2 &&*/ t.diff_msec(passTime)>550/*)*/){  // Schreizeitpunkt
          doKick= false; //(MWM.get_robot_properties().kickers>=2);
          cerr << "Kurzer Pass Quer\r\n";
          LOUT << "Kurzer Pass Quer!" <<endl;
          MWM.get_message_board().publish("short_pass:");
        }

        if(t.diff_msec(passTime)>900 && t.diff_msec(passTime)<1800){  // Rueckwaertsfahren
          dv.vtrans = (-(ball3d.pos.toVec()-robot.pos).normalize())/robot.heading;
          dv.vrot = 0;
          dv.kick=false;
//        dv=goToPos->getCmd(t);
//        dv.vtrans=dv.vtrans.mirror(Vec(0,0));
        if (doKick) {
            dv.kick=LOWKICK;
            dv.klength = throwInKickDuration; // TODO: eigenen wert im configfile vergeben. Allerdings _nicht_ shortpass verwenden, da das einen leicht anderen wert haben muss (der short-pass ist wegen der spielfeldgrš§e inzwishcen lŠnger als der anschubkick).
            LOUT << "KURZPASS-GESCHOSSEN\n";
        }
                /*  if(isMitte(area) || isOwnMiddle(area)){
            target = Vec((isLinks(area)?-500:500)+robot.pos.x, robot.pos.y);
          }else{
            target = Vec((isLinks(area)?500:-500)+robot.pos.x, robot.pos.y);
          }
          goToPos->init(target, 1.5, ball3d.pos.toVec(),true); */
                
        }else if(t.diff_msec(passTime)>1800){ // Ende
          dv.vtrans.x = 0;
          dv.vtrans.y = 0;
          finished= true;
          LOUT << "Ball beruehrt(4)!" <<endl;
          MWM.get_message_board().publish("touchedBall:");
        }else{        
          dv = goToPos->getCmd(t);
        }
      }
          
/*        goToBall.setParameters(target, 1.5);
        dv = goToBall.getCmd(t);
        MWM.get_message_board().publish("short_pass:");
        cerr << "Kurzer Pass Quer\r\n";
        LOUT << "Kurzer Pass Quer!" <<endl;
        if (WBOARD->doPossessBall(t) && passFirstPhase){
	      passFirstPhase = false;
	      kickerEvadePosition = kickerEvadePosition + 
	        (kickerEvadePosition - ball3d.pos.toVec()).normalize() * 300.;
          if (dv.vtrans.length() < 1.5) {
            dv.vtrans = dv.vtrans.normalize() * 1.5;
          }
        } else if (passFirstPhase) {
          passTime = t;
	    }
        if (!passFirstPhase && t.diff_msec(passTime) < 200) { // vorfahren
          dv = goToBall.getCmd(t);
          if (dv.vtrans.length() < 1.5) dv.vtrans = dv.vtrans.normalize() * 1.5;
        } else if (!passFirstPhase && 
             (robot.pos-ball3d.pos.toVec()).length() < 1300.) { // freimachen
          goToPos->init(kickerEvadePosition, 1.5, 
          ball3d.pos.toVec() - robot.pos, true);
          LOUT << "thin red line " << kickerEvadePosition << endl;
          dv = goToPos->getCmd(t);
        } else if (!passFirstPhase &&                 // aufhoeren
		   (WBOARD->teamPossessBall() || 
		    t.diff_msec(passTime) > 3300)) {
          finished = true;
        } else if (!passFirstPhase) {  // weit genug weg, einfach abwarten
          dv.vtrans = Vec(0.,0.);
        }
      }*/
      return dv;
    }
  }

  DriveVector 
  BPreOwnIndirectStandardSituation::getCmdBefore(const Time& t) 
    throw(TribotsException)
  {
    const BallLocation& ball = MWM.get_ball_location(t);

    LOUT << "\% red circle " << ball.pos.x << " "<< ball.pos.y << " 2000 \n";
    

    DriveVector dv;
    double wskt = urandom();
    switch (WBOARD->getStandardSituationRole()) { // welche position hat der
    case WhiteBoard::STANDARD_ROLE_ABCDE:
      doStandard = true;
      _kick = false;
      dv = approachBall(t);
      break;
    case WhiteBoard::STANDARD_ROLE_AB: // TODO: gesonderte startposition 
    case WhiteBoard::STANDARD_ROLE_A:
    case WhiteBoard::STANDARD_ROLE_B:
      if ((WBOARD->getStandardSituationRole() == WhiteBoard::STANDARD_ROLE_A && !isLinks(area)) ||
          (WBOARD->getStandardSituationRole() == WhiteBoard::STANDARD_ROLE_B &&  isLinks(area))) {  // auf anderer Spielfeldseite
        dv = blockGegnerFromBall(t);
      }
      else {                 // auf eigener Seite (oder beide Seiten weil AB)
        if(isEcke(area)){
          doStandard = true;
          _ownTurn = true;
          _kick = false;
          dv = approachBall(t);
        }else if((goalKickDirect && originalRefstate == preOwnGoalKick) ||
                 (ownHalfAllDirect && isOwnMiddle(area))){
          doStandard = true;
          _ownTurn = true;
          _kick = false;
          if(isLinks(area)){
            dv = block(t,POS_LEFT);
          }else{
            dv = block(t,POS_RIGHT);
          }
        }else if (isLinks(area)) {
          doStandard = false;
          dv = block(t, POS_LEFT);
        }else{
          doStandard = false;
          dv = block(t, POS_RIGHT);
        }
      }
      break;
    case WhiteBoard::STANDARD_ROLE_C:
    case WhiteBoard::STANDARD_ROLE_CE:
      if (isLinks(area) && !isEcke(area)) {
	      if(!_decided){
	        doStandard = true;
	        if(!(area == MITTELINKS) && !isOwnHalf(area)){
	          _kick = wskt <= standardPassProbability;
	          if(_kick){
		          cerr << "Langer Pass\r\n";
		          LOUT << "Langer Pass" << endl;
	          }
	        }else{
	          _kick=false;
	        }
           _ownTurn = ((wskt > standardPassProbability) && wskt <
           (standardPassProbability + standardDribbling));
           if(ownHalfAllDirect && !isOwnMiddle(area)){
             _ownTurn = true;
           }
           if((originalRefstate == preOwnGoalKick && goalKickDirect) ||
           (ownHalfAllDirect && isOwnMiddle(area))){
             doStandard = false;
           }
           if(_ownTurn){
             _kick=false;
             cerr << "Dribbling: " << _ownTurn << "\r\n";
             LOUT << "Dribbling: " << _ownTurn << endl;
           }
	        _decided = true;
	      }
	      dv = approachBall(t);
      }else{
        doStandard = false;
	     dv = block(t, POS_LEFT);
      }
      break;
    case WhiteBoard::STANDARD_ROLE_D:
    case WhiteBoard::STANDARD_ROLE_DE:
      if (isLinks(area)||isEcke(area)) {
        doStandard = false;
	     dv = block(t, POS_RIGHT);
      }else{
	      if(!_decided){
	        doStandard = true;
	        if(!(area == MITTERECHTS) && !isOwnHalf(area)){
	          _kick = wskt <= standardPassProbability;
	          if(_kick){
		         cerr << "Langer Pass\r\n";
               LOUT << "Langer Pass" << endl;
	          }/*else{
              if(area == GEGNERISCHEECKERECHTS){
                _ownTurn = brandom(standardDribbling);
                cerr << "Dribbling: " << _ownTurn << "\r\n";
                LOUT << "Dribbling: " << _ownTurn << endl;
              }
	          }*/
	        }else{
	          _kick = false;
	        }
           _ownTurn = (wskt > standardPassProbability && (wskt <
           standardPassProbability + standardDribbling));
           if(ownHalfAllDirect && !isOwnMiddle(area)){
             _ownTurn = true;
           }
           if((originalRefstate == preOwnGoalKick && goalKickDirect) ||
           (ownHalfAllDirect && isOwnMiddle(area))){
             doStandard = false;
           }
           if(_ownTurn){
             _kick=false;
             cerr << "Dribbling: " << _ownTurn << "\r\n";
             LOUT << "Dribbling: " << _ownTurn << endl;
           }
	        _decided = true;
	      }	
	      dv = approachBall(t);
      }
      break;
    case WhiteBoard::STANDARD_ROLE_CD:
    case WhiteBoard::STANDARD_ROLE_CDE:
    default:
      if(isEcke(area)){
        doStandard = false;
        dv = block(t, isLinks(area)?POS_LEFT:POS_RIGHT);
      }else{
        if((originalRefstate == preOwnGoalKick && goalKickDirect) ||
           (ownHalfAllDirect && isOwnMiddle(area))){
          doStandard = false;
        }else{
          doStandard = true;
        }
        if(!_decided){
           _ownTurn = (wskt > standardPassProbability && (wskt <
           standardPassProbability + standardDribbling));
           if(ownHalfAllDirect && !isOwnMiddle(area)){
             _ownTurn = true;
           }
           if(_ownTurn){
             cerr << "Dribbling: " << _ownTurn << "\r\n";
             LOUT << "Dribbling: " << _ownTurn << endl;
           }
           _decided = true;
         }
        _kick = false;
        dv = approachBall(t);
      }
      break;
    case WhiteBoard::STANDARD_ROLE_E:
      //Steht immer hinter den ausführenden als Block
      dv = safeGuard(t);
    break;
    }
    return dv;
  }  
 
  DriveVector
  BPreOwnIndirectStandardSituation::safeGuard(const Time& t){
    const RobotLocation& robot = MWM.get_robot_location(t);
    const BallLocation& ball3d = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry();
    
    Vec protectPos = Vec(0,-field.field_length/2);
    Vec target;
    
    double dist = 0.;
    XYRectangle penaltyArea(Vec(-field.penalty_area_width/2. - 200, 
                                -field.field_length / 2. - 200), // hinter gl
                            Vec(field.penalty_area_width/2. + 200,
                                -field.field_length / 2. + dist +  // 50cm vor Strafraum
                                field.penalty_area_length));
    
    LineSegment leftLineOfPenaltyArea(Vec(-field.penalty_area_width/2.-200, 
                                          -field.field_length / 2. - 200.),
                                      Vec(-field.penalty_area_width/2.-200, 
                                          -field.field_length / 2. + 
                                          field.penalty_area_length+dist));
    LineSegment rightLineOfPenaltyArea(Vec(field.penalty_area_width/2.+200, 
                                           -field.field_length / 2. - 200.),
                                       Vec(field.penalty_area_width/2.+200, 
                                           -field.field_length / 2. + 
                                           field.penalty_area_length+dist));
    LineSegment topLineOfPenaltyArea(Vec(-field.penalty_area_width/2.-200, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length+dist),
                                     Vec(field.penalty_area_width/2.+200, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length+dist));
    
    if (penaltyArea.is_inside(ball3d.pos.toVec())) {
      target = Vec(0, -field.field_length/2.+field.penalty_area_length);
    }
    else {
    
      // sucht den Punkt an der Strafraumgrenze, der auf der abzudeckenden Linie
      // Ball->Tor liegt.
      LineSegment ballLine(ball3d.pos.toVec(), protectPos);
      
      vector<Vec> intersections;
      intersections = intersect(leftLineOfPenaltyArea, ballLine);
      if (! intersections.size()) {
        intersections = intersect(rightLineOfPenaltyArea, ballLine);
      }
      if (! intersections.size()) {
        intersections = intersect(topLineOfPenaltyArea, ballLine);
      }
      if (intersections.size() >= 1) {
        target = intersections[0];
      }
      else {
        JERROR("Keine Freistossposition fuer Safety gefunden.");
        target = ball3d.pos.toVec() + Vec((ball3d.pos.x > 0 ? -1. : .1) * 2500., 0);
      }
    }
    if((target-robot.pos).length() < 300.){
      atPosition = true;
      return stop->getCmd(t);
    }
    goToPos->init(target, ball3d.pos.toVec()-robot.pos, true);
    goToPos->set_dynamics(2.0);
    return goToPos->getCmd(t);
  }

  DriveVector
  BPreOwnIndirectStandardSituation::blockGegnerFromBall(const Time& t){
    const RobotLocation& robot = MWM.get_robot_location(t);
    const BallLocation& ball3d = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry();
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
    Vec position = Vec(0,field.field_length/2);
    position = ball3d.pos.toVec() + standardAbstandBlock * (-ball3d.pos.toVec()+position).normalize();
    
    if (ball3d.pos.x > -1000 && ball3d.pos.x < 1000) { // ball in Mitte, keinen Block
      position = ball3d.pos.toVec() + Vec(0., ball3d.pos.y > -field.field_length/2. + 4500. ? -2500. : 2500.);
    } else if (doDynamicBlock) {   // dynamische Blockposition berechnen, falls dieses Feature angeschaltet wurde
      Vec newDangerousOpponentPosition (1e300, 1e300);
      double opponentDistance=1e300;
      Angle goaldir = (Vec(0,field.field_length/2)-ball3d.pos.toVec()).angle();
      Angle rightangle (goaldir-Angle::deg_angle(30));
      Angle leftangle (goaldir+Angle::deg_angle(30));
      for (unsigned int i=0; i<obstacles.size(); i++) {
        double d=(obstacles[i].pos-ball3d.pos.toVec()).length();
        Angle a=(obstacles[i].pos-ball3d.pos.toVec()).angle();
        if (a.in_between (rightangle, leftangle) && d<opponentDistance && obstacles[i].player<0) {
          opponentDistance=d;
          newDangerousOpponentPosition=obstacles[i].pos;
        }
      }
      if (abs(opponentDistance-(dangerousOpponentPosition-ball3d.pos.toVec()).length())>200)
        dangerousOpponentPosition=newDangerousOpponentPosition;
      if ((dangerousOpponentPosition-ball3d.pos.toVec()).length()<3000) {
        // Gegner in der Naehe, davorstellen
        LOUT << "Gefaehrlichen Gegner gefunden und blocken\n";
        LOUT << "% solid red cross " << dangerousOpponentPosition << '\n';
        position = dangerousOpponentPosition+400*(ball3d.pos.toVec()-dangerousOpponentPosition).normalize();
      }
    }
    
    double distance = (position - robot.pos).length();
    if(distance < 400. && ((ball3d.pos.toVec()-robot.pos).angle()-robot.heading-Angle::quarter).in_between(-Angle::deg_angle(10), Angle::deg_angle(10))) {
      atPosition = true;
      return stop->getCmd(t);
    }
    
    goToPos->init(position, (ball3d.pos.toVec()-robot.pos), true);
    goToPos->set_dynamics (2.0);
    return goToPos->getCmd(t);
  }

  Vec BPreOwnIndirectStandardSituation::volleyPosition (Time t) {
    const BallLocation& ball3d = MWM.get_ball_location(t);
    Vec position (ball3d.pos.x>0 ? -1500 : 1500, ball3d.pos.y-1500);
    return position;
  }

  DriveVector
  BPreOwnIndirectStandardSituation::gotoVolleyPosition(const Time& t){
    const RobotLocation& robot = MWM.get_robot_location(t);
    const BallLocation& ball3d = MWM.get_ball_location(t);
    Vec position = volleyPosition (t);
    double distance = (position - robot.pos).length();
    if(distance < 400. && robot.heading.in_between (Angle::rad_angle(-0.08), Angle::rad_angle(0.08))) {
      atPosition = true;
      LOUT << "Standard: volley position reached\n";
      return stop->getCmd(t);
    }
    LOUT << "Standard: Goto volley position\n";
    goToPos->init(position, Angle::zero, true);
    goToPos->set_dynamics (2.0);
    goToPos->set_target_evade_strategy (ball3d.pos.x>0 ? Angle::zero : Angle::half);
    DriveVector dv = goToPos->getCmd(t);
    goToPos->force_target ();
    return dv;
  }
  
  DriveVector
  BPreOwnIndirectStandardSituation::approachBall(const Time& t)
  {
    const RobotLocation& robot = MWM.get_robot_location(t);
    const BallLocation& ball3d = MWM.get_ball_location(t);
    
    Vec position;
    Vec target;
    
    if(originalRefstate == preOwnKickOff) {  // Anstoss
      position = Vec(-800,-488);
      target = Vec(800,488);
    } else if (isEcke(area)) {          // Eckstossbereich
      position = Vec(ball3d.pos.x
                      + 300*((area==GEGNERISCHEECKELINKS)?-1:1),
                     ball3d.pos.y - 500);
      target = ball3d.pos.toVec();
    }else if(isOwnMiddle(area) || isMitte(area)){
      position = Vec(ball3d.pos.x+700*
		     ((isLinks(area))?1:-1), 
		     ball3d.pos.y+700); 
      target = ball3d.pos.toVec(); 
    }else{                                               // Rest
      position = Vec(ball3d.pos.x+700*
		     (!(isLinks(area))?1:-1), 
		     ball3d.pos.y+600); 
      // TODO: Hindernis betrachten und wenn frei naeher an ball
      target = ball3d.pos.toVec();
    }
    double distance = (position - robot.pos).length();
    if(distance > 200){
      goToPos->init(position, (isEcke(area))?(target-robot.pos):(target - robot.pos).rotate_sixth(), true);
      goToPos->set_dynamics (distance < 2000. ? 1.5 : 2.5);
      return goToPos->getCmd(t);
    }
    else {
      atPosition = true;
      return stop->getCmd(t);
    }
  }

  DriveVector
  BPreOwnIndirectStandardSituation::block(const Time& t, int pos)
  {
    const FieldGeometry& field = MWM.get_field_geometry();
    const BallLocation& ball3d = MWM.get_ball_location(t);
    const RobotLocation& robot = MWM.get_robot_location(t);
    Vec goal = Vec(0,field.field_length / 2);
    
    Vec position;
    Vec ball(ball3d.pos.x, ball3d.pos.y);

    
    //freier Spieler auf der anderen seite
    if ((!isLinks(area) && pos == POS_LEFT) ||
	      (isLinks(area) && pos == POS_RIGHT)) {
      if(originalRefstate == preOwnKickOff){
        Vec protectGoalPos = 
          Vec(0., -field.field_length/2.) + Vec(700, 0.);
    
        position = Vec((protectGoalPos-ball)*0.33);
        if (position.length() < (3500. + 110. + 250.)) { // zu nah dran?
	        position = position.normalize() * (3500. + 110. + 250.);
        }
        position += ball;
        goToPos->init(position, ball-robot.pos, true);
        goToPos->set_dynamics (true, true);
      }else{
        //für jede Area anders
        switch(area){
          case MITTELINKS:
          case MITTERECHTS:
            position = getPositionForTwoPlayer(t);
            goToPos->init(position, ball-robot.pos, true);
            goToPos->set_dynamics (1.9);
            break;
          default:
            if (doVolley && (area==SEITELINKS || area==SEITERECHTS)) {
              return gotoVolleyPosition(t);
            } else {
              position = getTargetForFreePlayer(t); 
            }
            goToPos->init(position, ball-robot.pos, true);
            goToPos->set_dynamics(1.9);
            break;
        }
      }
    }
    else {  // der "ball"spieler stellt sich in die passende achse zum tor
        //für jede area anders
        if(preOwnKickOff == originalRefstate){
          position = Vec(0, -1000);
          goToPos->init(position, ball-robot.pos, true);
          goToPos->set_dynamics(1.4);
        }else if(isEcke(area)){
          //wenn in der Ecke eine Standardsituation entsteht soll der Spieler
          //passen dfür nen eigenmove stehen
          if(area!=GEGNERISCHEECKELINKS){
            position = Vec(field.field_width/6,field.field_length/6);
          }else{
            position = Vec(-field.field_width/6,field.field_length/6);
          }
          Vec target = ball3d.pos.toVec();
          goToPos->init(position, target-robot.pos, true);
          goToPos->set_dynamics(2.0);
        }else if(isMitte(area)){
          position = ball + (isLinks(area) ? Vec(-1000,-1000) : Vec(+1000,-1000)); 
          goToPos->init(position, ball-robot.pos, true);
          goToPos->set_dynamics(2.0);
        }else if(isOwnMiddle(area)){
          position = ball + (isLinks(area) ? Vec(-300,-900) : Vec(300,-900)); 
          goToPos->init(position, Vec(0,1000), true);
          goToPos->set_dynamics(2.0);
        }else{        
          Vec neu = ((isLinks(area)) ?
                                     Vec(+800,-800) :
                                      Vec(-800,-800));
	        position = ball + neu;
          goToPos->init(position, Vec(0,field.field_length/2)-robot.pos, true);
          goToPos->set_dynamics(2.0);
        }
    }
    if ((position - robot.pos).length() < 200) {
      atPosition = true;
    }
    return goToPos->getCmd(t);
  }
  
  
  void BPreOwnIndirectStandardSituation::updateTactics (const TacticsBoard& tb) throw () {
    double longPassProb = 5.;
    double shortPassProb = 5.;
    string key = "StandardLangPassWk";
    if (tb[key] == string("nie")) {
      longPassProb = 0.;
    }
    else if (tb[key] == string("drittel")) {
      longPassProb = .33;
    }
    else if (tb[key] == string("zweidrittel")) {
      longPassProb = .66;
    }
    else { // "immer"
      longPassProb = 1.;
    }
    
    key = "StandardKurzPassWk";
    if (tb[key] == string("nie")) {
      shortPassProb = 0.;
    }
    else if (tb[key] == string("drittel")) {
      shortPassProb = .33;
    }
    else if (tb[key] == string("zweidrittel")) {
      shortPassProb = .66;
    }
    else { // "immer"
      shortPassProb = 1.;
    }
    
    standardPassProbability = longPassProb;
    standardDribbling = 1-longPassProb-shortPassProb;
    if (standardDribbling < 0) {
      standardDribbling = 0;
    }

    key = "StandardAbstandBlock";
    int ab;
    if (string2int (ab, tb[key])) {
      if (ab >= 1499 && ab <= 3001) {
         standardAbstandBlock = ab;
      }
    }
    key = "GoalKickDirect";
    if(tb[key] == string("an")){
      goalKickDirect = true;
    } else if(tb[key] == string("aus")){
      goalKickDirect = false;
    }
    key = "OwnHalfAllDirect";
    if(tb[key] == string("an")){
      ownHalfAllDirect = true;
    }else if(tb[key] == string("aus")){
      ownHalfAllDirect = false;
    }
    key = "StandardMitDynamischemBlock";
    if (tb[key] == string("an")) {
      doDynamicBlock = true;
    }
    else {
      doDynamicBlock = false;
    }
    double dbx;
    if (string2double (dbx, tb["Volley"])) {
      volleyProbability=dbx;
    }
  }
}
