#include "WhiteBoard.h"
#include "../WorldModel/WorldModel.h"
#include "../Fundamental/stringconvert.h"
#include "../Fundamental/geometry.h"
#include "../Behavior/Predicates/freeCorridor.h"
#include <cmath>

using namespace std;
using namespace Tribots;

#define LEFT 0
#define MIDDLE 1
#define RIGHT 2

#define DISPLAY_AREAS 1
#define DISPLAY_DRIBBLE_CORRIDOR 1
#define DISPLAY_FREEGOALPOS_CORRIDORS 1
#define DISPLAY_SHOOTCORRIDOR 1

#define MIN(_x_,_y_) ((_x_) < (_y_) ? (_x_) : (_y_))
#define MAX(_x_,_y_) ((_x_) > (_y_) ? (_x_) : (_y_))


namespace Tribots {
  
  //////////////////////// singleton implementation ///////////////////////////
  
  WhiteBoard* WhiteBoard::instance = 0;
  
  WhiteBoard* 
  WhiteBoard::getTheWhiteBoard()
  {
    if (instance == 0) {
      instance = new WhiteBoard();
    }
    return instance;
  }
  
  
  ////////////////////////// init and deinit code /////////////////////////////
  
  WhiteBoard::WhiteBoard()
    : m_bDoNotGetBallPossession(false),ballInOppGoalData(0),possesBallData(0),
      abs2relData(0),rel2absData(0),shootCorridorData(0), freeGoalPosData(0),
      dribbleCorridorData(0),passPlayedData(0), cycles_without_team_posses_ball(10),
      cycles_without_team_possess_ball_extended(10),
      cycles_without_advanced_team_posses_ball(10), 
      cycles_without_active_robots(1000),
      cycles_without_receive_pass(1000),
      cycles_without_receive_spsp(1000),
      cycles_without_nach_vorne(1000),
      number_active_robots(1),
      standardSituationRole(STANDARD_ROLE_A),
      lastMessageBoardCheck(0)
  {
    frames_ball_owned=0;
    m_bFreeWayToTarget=1;
    m_iEvasiveManeuverMainDirection=LEFT;

    ms_x_a = 1.8443;
    ms_x_b =-13.9908;
    ms_x_c = -5.20274;
    ms_y_a = 1.09432;
    ms_y_b = 5.70011;
    ms_y_c = -2.14541;

  }
  
  
  WhiteBoard::~WhiteBoard()
  {
    if (instance == this) {       // cleanup singleton
      instance = 0;
    }
    
    // <<<<<<<<< put your clean up code here
    
    if (ballInOppGoalData) delete ballInOppGoalData;
    if (possesBallData) delete possesBallData;
    if (abs2relData) delete abs2relData;
    if (rel2absData) delete rel2absData;
    if (shootCorridorData) delete shootCorridorData;
    if (dribbleCorridorData) delete dribbleCorridorData;
    if (passPlayedData) delete passPlayedData;
  }
  
  void WhiteBoard::readConfigs(const ConfigReader& cfg)
  {
    //Read in Playerspecific Data
    cfg.get ("WhiteBoard::OwnsBallPixelDistance", owns_ball_pixel_distance);
    
    

    cfg.get("WhiteBoard::ms_x_a",ms_x_a);
    cfg.get("WhiteBoard::ms_x_b",ms_x_b);
    cfg.get("WhiteBoard::ms_x_c",ms_x_c);
    cfg.get("WhiteBoard::ms_y_a",ms_y_a);
    cfg.get("WhiteBoard::ms_y_b",ms_y_b);
    cfg.get("WhiteBoard::ms_y_c",ms_y_c);
  }

  
  bool 
  WhiteBoard::onlyTwoRobots() const
  {
    return number_active_robots == 2  && cycles_without_active_robots < 60;
  }
  
  bool 
  WhiteBoard::onlyOneRobot() const
  {
    return number_active_robots == 1 || cycles_without_active_robots >= 60; // geht an, wenn nur einer da ist oder nix uebermittelt wird!
  }  

  bool 
    WhiteBoard::onlyThreeRobots() const
    {
      return number_active_robots == 3 && cycles_without_active_robots < 60;
    } 
  
  bool 
    WhiteBoard::onlyFourRobots() const
    {
      return number_active_robots == 4 && cycles_without_active_robots < 60;
    } 
  
  unsigned int
  WhiteBoard::getActiveRobots() const
  {
    return cycles_without_active_robots >= 60 ? 1 : number_active_robots;  // wenn lange nix bekommen, dann nimm an, dass du alleine bist
  }


  //////////////////////////// detectPassedBall ////////////////////////////////
   bool WhiteBoard::detectPassedBall (const Time& texec) {

      BoolData* data;
      if (passPlayedData) {
         data = dynamic_cast<BoolData*>(passPlayedData);
         if (data->t == texec &&
             data->cycle == 
             WorldModel::get_main_world_model().get_game_state().cycle_num) 
         {
          return data->b;          // Zwischengespeichertes Ergebnis zurck
         }
      } else {                       // erste Anfrage berhaupt
          data = new BoolData();     // Zwischenspeicher erzeugen
          passPlayedData = data;
      }
      const RobotLocation& robot_exec (MWM.get_robot_location (texec));
      const BallLocation& ball_exec (MWM.get_ball_location (texec));
      const ObstacleLocation& obstacles = MWM.get_obstacle_location(texec);
      Frame2d world2robot = WBOARD->getAbs2RelFrame(texec);

      // hard conditions
      bool passHardCondition = true;

      // only recognize passed balls if closer than 2.5 m, 
      // because otherwise there could no obstacles get detected behind
      passHardCondition =  passHardCondition && ((ball_exec.pos.toVec()-robot_exec.pos).length()/1000 < 2.5);

      // only recognize passed balls if farer than 1.2 m, 
      // because otherwise there is no time to react
      passHardCondition =  passHardCondition && ((ball_exec.pos.toVec()-robot_exec.pos).length()/1000 > 1.2);

      // check if an obstacle (or teammate) is near ball
      // then it is probably no pass
      const double MIN_OBSTACLE_BALL_DIST = 1500;
      bool obsBehind = false;
      Vec bpos = ball_exec.pos.toVec();
      for (unsigned int o=0; o < obstacles.size(); o++) { 
         // print the ostacle on screen
         if ((bpos - obstacles[o].pos).length() < MIN_OBSTACLE_BALL_DIST) {
            obsBehind = true;
            break;
         }
      }
      passHardCondition = passHardCondition && !obsBehind;

      // check if the ball is rolling towards the robot
      Vec ball_rob = (robot_exec.pos - ball_exec.pos.toVec());
      double length_prod = (ball_rob.length() * ball_exec.velocity.toVec().length());
      bool rollingTowards = false;
      if (length_prod != 0) {
         // LOUT << "ballrolling with cos of angle: " << (ball_rob * ball_exec.velocity.toVec()) / length_prod << endl;
         const double epsilon = 0.05;
         if( fabs((ball_rob * ball_exec.velocity.toVec()) / length_prod - 1) < epsilon) {
            rollingTowards = true;
         }
      }
      passHardCondition = passHardCondition && rollingTowards ;

      // check if ball is rolling towards goal of opponent
      bool safeBallMovement = false;
      if (ball_exec.velocity.toVec().y > 0) {
         safeBallMovement = true;
      }
      passHardCondition = passHardCondition && safeBallMovement;
        
      // soft conditions (also hard, if MIN_GOOD_CONDITIONS is equal to the amount of conditions)
      const int MIN_GOOD_CONDITIONS = 1;
      int goodConditions = 0;

      // need a higher ball speed
      if (ball_exec.velocity.toVec().length() > 2.0) goodConditions++;
         
      return passHardCondition && (goodConditions >= MIN_GOOD_CONDITIONS);
  }




  bool
  WhiteBoard::receiveSetPlayShortPass() const
  {
    return cycles_without_receive_spsp < 60;
  }

    
  bool
  WhiteBoard::receivePass() const
  {
    return cycles_without_receive_pass < 60;
  }

  // pass spiel 10 sekunden lang signalisieren
  bool
  WhiteBoard::receivePlannedPass(unsigned int playerId) const
  {
    return (cycles_without_planned_pass < 300) && (planned_pass_receiver == playerId);
  }
  
  // passannahme ausschalten (zb nach annahme)
  void
  WhiteBoard::resetPlannedPass()
  {
    cycles_without_planned_pass = 300;
  }

  bool
  WhiteBoard::touchedBall() const
  {
    return cycles_without_touched_ball < 60;
  }
  
	bool
	WhiteBoard::doCounterAttack() const
	{
	  return cycles_without_nach_vorne < 3;
	}

  ///////////////////////// standardSituationRole /////////////////////////////

  int
  WhiteBoard::getStandardSituationRole() const
  {
    return standardSituationRole;
  }

  void
  WhiteBoard::setStandardSituationRole(int role)
  {
    this->standardSituationRole = role;
  }
  
  
  void WhiteBoard::setZonePressureRole(string role)
  { zonePressureRole = role; }
  string WhiteBoard::getZonePressureRole() const 
  { return zonePressureRole; }

  //////////////////////////// isBallInOppGoal ////////////////////////////////

  bool
  WhiteBoard::isBallInOppGoal(const Time& t)
  {
    BoolData* data;
    if (ballInOppGoalData) {
      data = dynamic_cast<BoolData*>(ballInOppGoalData);
    
      if (data->t     == t &&
	  data->cycle == 
	  WorldModel::get_main_world_model().get_game_state().cycle_num) {
	return data->b;          // Zwischengespeichertes Ergebnis zurck
      }
    }
    else {                       // erste Anfrage berhaupt
      data = new BoolData();     // Zwischenspeicher erzeugen
      ballInOppGoalData = data;
    }

    data->t = t;
    data->cycle = 
      WorldModel::get_main_world_model().get_game_state().cycle_num;

    BallLocation ballLocation =
      WorldModel::get_main_world_model().get_ball_location(t);
    FieldGeometry field = 
      WorldModel::get_main_world_model().get_field_geometry(); 

    if (ballLocation.pos.y < field.field_length / 2. ||
	ballLocation.pos.x < -field.goal_width ||
	ballLocation.pos.x > field.goal_width ||
	ballLocation.pos.y > field.field_length / 2. + field.goal_length * 2) {
      return data->b =  false;
    }
    else {
      return data->b = true;
    }
  }

  ////////////////////////////// doPossessBall ////////////////////////////////

  void
  WhiteBoard::resetPossessBall()
  {
    frames_ball_owned = 0;
  }
  
  /** wenn der ball maximal 40cm abstand vom roboter hat
  und nicht weiter als 5cm seitlich ist, kann er gedribbelt werden **/
  bool
  WhiteBoard::canDribbleBall() {
    Vec relBall = MWM.get_ball_relative().pos;
    if(relBall.y > 400 || fabs(relBall.x) > 50){
      return false;  
    } else {
      return true;
    }
  }
  
  bool
  WhiteBoard::doPossessBall(const Time& /*t*/) // Zeitpunkt, nicht beruecksichtigen, immer Bildzeit nehmen
  {
    Time t;
    RobotLocation rloc_dummy = MWM.get_slfilter_robot_location (t);  // Zeitstempel des Bildes
    BoolData* data;
    if (possesBallData) {
      data = dynamic_cast<BoolData*>(possesBallData);
    
      if ( data->cycle == 
	  WorldModel::get_main_world_model().get_game_state().cycle_num) {
	return data->b;          // Zwischengespeichertes Ergebnis zurueck
      }
    }
    else {                       // erste Anfrage berhaupt
      data = new BoolData();     // Zwischenspeicher erzeugen
      possesBallData = data;
      
    }

    data->t = t;
    data->cycle = 
      WorldModel::get_main_world_model().get_game_state().cycle_num;

    BallLocation ballLocation =
      WorldModel::get_main_world_model().get_ball_location(t);
    const RobotLocation& robot = MWM.get_robot_location(t);

    if (ballLocation.pos_known == BallLocation::unknown ||
        ballLocation.pos_known == BallLocation::communicated) {
      frames_ball_owned--;
    }
    else { // ball_pos == known || ball_pos == raised
      Vec relBall = MWM.get_ball_relative().pos; //getAbs2RelFrame(t) * ballLocation.pos;
      //LOUT << "% yellow dotted thin circle " << (robot.pos+(relBall*robot.heading)) << " 100 " << endl;

      bool ballclose=true;
      if ((relBall.y> owns_ball_pixel_distance|| relBall.y<0)  // nicht schiessen, wenn ball zu weit weg
          || fabs(relBall.x) > 80) {  
        ballclose=false;
      }
      if (ballclose){
        frames_ball_owned++;
      }
      else { 
        frames_ball_owned--;
      }
    }
    if (frames_ball_owned<0) {
      frames_ball_owned=0;
    }
    if (frames_ball_owned>16)  {
      frames_ball_owned=16;
    }


    if (frames_ball_owned < 8){
      data->b = false; 
    }
    else {
      data->b = true;

      if(!m_bDoNotGetBallPossession) {
        data->b = true; 
      }
    }

    if (data->b) { 
      MWM.get_message_board().publish ("pB!");
      LOUT << "\% red line" << robot.pos << " " << (robot.pos + (Vec(0,1400)*robot.heading))        << endl;    
      bool extendedPossessBall = true;
      if (obstacle_distance_to_line(robot.pos,
                                    robot.pos + (Vec(0,1000.)*robot.heading), 
                                    MWM.get_obstacle_location(t), true) < 750.) {
        extendedPossessBall = false;
      }
      if (extendedPossessBall) {    // nicht zwischenspeichern, interessiert nur die anderen
        MWM.get_message_board().publish("extendedPB!");
      }
    }
    
    return data->b;
  }

  ///////////////////////////// getRel2AbsFrame ///////////////////////////////

  const Frame2d&
  WhiteBoard::getRel2AbsFrame(const Time& t)
  {
    Frame2dData* data;
    if (rel2absData) {
      data = dynamic_cast<Frame2dData*>(rel2absData);
    
      if (data->t     == t &&
	  data->cycle == 
	  WorldModel::get_main_world_model().get_game_state().cycle_num) {
	return data->frame;      // Zwischengespeichertes Ergebnis zurck
      }
    }
    else {                       // erste Anfrage berhaupt
      data = new Frame2dData();  // Zwischenspeicher erzeugen
      rel2absData = data;
    }
    
    RobotLocation robotLocation = 
      WorldModel::get_main_world_model().get_robot_location(t);
    
    data->frame.set_position(robotLocation.pos);
    data->frame.set_angle(robotLocation.heading);

    return data->frame;
  }

  ///////////////////////////// getAbs2RelFrame ///////////////////////////////

  const Frame2d&
  WhiteBoard::getAbs2RelFrame(const Time& t)
  {
    Frame2dData* data;
    if (abs2relData) {
      data = dynamic_cast<Frame2dData*>(abs2relData);
    
      if (data->t     == t &&
	  data->cycle == 
	  WorldModel::get_main_world_model().get_game_state().cycle_num) {
	return data->frame;      // Zwischengespeichertes Ergebnis zurck
      }
    }
    else {                       // erste Anfrage berhaupt
      data = new Frame2dData();  // Zwischenspeicher erzeugen
      abs2relData = data;
    }

    data->frame = getRel2AbsFrame(t);    // rel2abs holen
    data->frame.invert();                // und invertieren

    return data->frame;
  }
  
  void WhiteBoard::checkMessageBoard()
  {
    if (MWM.get_game_state().cycle_num == lastMessageBoardCheck) {
      return;
    }
    lastMessageBoardCheck = MWM.get_game_state().cycle_num;
    
    // nach TeamBallbesitz suchen
    if (MWM.get_message_board().scan_for_prefix ("OwnsBall!").length()>0)
      cycles_without_team_posses_ball=0;
    else
      cycles_without_team_posses_ball++;
    
    if (MWM.get_message_board().scan_for_prefix ("OwnsBallExtended!").length()>0) {
      cycles_without_team_possess_ball_extended=0;
    }
    else
      cycles_without_team_possess_ball_extended++;
      
    // nach erweitertem TeamBallbesitz suchen
    if (MWM.get_message_board().scan_for_prefix ("NearBall!").length()>0)
      cycles_without_advanced_team_posses_ball=0;
    else
      cycles_without_advanced_team_posses_ball++;

    // nach knuts pass suchen
    string message;
    string temp;
    if ((message = MWM.get_message_board().scan_for_prefix ("knutpass")).length() > 0) {
      LOUT << "WhiteBoard: Geplanter Pass gefunden:" << message << endl;
      unsigned int receiverId = 0;
      istringstream stream(message);
      stream >> temp >> receiverId;
      LOUT << "WhiteBoard: Pass zu Spieler: " << receiverId << endl; 
      planned_pass_receiver = receiverId;
      cycles_without_planned_pass=0;
    } else {
      cycles_without_planned_pass++;
    }
    if (MWM.get_message_board().scan_for_prefix ("receiveSetPlayShortPass!").length()>0)
      cycles_without_receive_spsp =0;
    else 
      cycles_without_receive_spsp++;  
      
    
    if (MWM.get_message_board().scan_for_prefix ("receivePass!").length()>0)
      cycles_without_receive_pass=0;
    else 
      cycles_without_receive_pass++;  
    
    if (MWM.get_message_board().scan_for_prefix ("touchedBall:").length()>0)
      cycles_without_touched_ball=0;
    else
      cycles_without_touched_ball++;
             
    if (MWM.get_message_board().scan_for_prefix ("NachVorne").length()>0)
      cycles_without_nach_vorne=0;
    else 
      cycles_without_nach_vorne++;                   

    /*wieviele Robots sind aktiv*/
    std::string s = MWM.get_message_board().scan_for_prefix ("Robots: ");
    if (s.length()>0) {
      std::stringstream ss;
      ss << s;
      ss >> s;
      ss >> number_active_robots; 
      LOUT << "NUMBER OF ACTIVE ROBOTS ACCORDING TO TEAMCONTROL: " << number_active_robots << "     " << endl;
      cycles_without_active_robots = 0;
    }
    else {
      cycles_without_active_robots++;
    }
  }
  
  bool WhiteBoard::teamPossessBall() 
  {
    return (cycles_without_team_posses_ball<6);
  }
  
  bool WhiteBoard::teamPossessBallExtended() {
    return (cycles_without_team_possess_ball_extended<6);
  }
  
  bool WhiteBoard::teammateNearBall() 
  {
    return (cycles_without_advanced_team_posses_ball<6);
  }

  //////////////////// calculate evasive movement vector //////////////////////

  Vec 
  WhiteBoard::calculateEvasiveMovement(const Vec& desiredVtrans, 
                                       const Vec& targetPos, const Time& t)
  {
    const RobotLocation& robotLocation = 
      WorldModel::get_main_world_model().get_robot_location(t);
  
    Vec robotPos = robotLocation.pos;
    Angle robotHeading = robotLocation.heading;
  
    bool obstacle_in_drivedir;
    obstacle_in_drivedir=false;
    ObstacleLocation obstacles= 
      WorldModel::get_main_world_model().get_obstacle_location(t);
  
    Frame2d r2a = getRel2AbsFrame(t);
    Frame2d a2r = getAbs2RelFrame(t);
    
    Vec left_of_drivedir=desiredVtrans.rotate(0.35)*2000; // Winkel aufspannen, 
    Vec right_of_drivedir=desiredVtrans.rotate(-0.35)*2000;//indem gesucht wird
    // \TODO : Keine Probleme mit Mittelpunkt von sehr nahen Hindernissen?

    LOUT << "\% blue line " <<robotLocation.pos<<r2a*left_of_drivedir<<'\n';
    LOUT << "\% red line " <<robotLocation.pos<<r2a*right_of_drivedir<<'\n';

    Vec relobs;  
    Vec mostdangerousobstacle(Vec(999999,99999));
     
    // suche in der Liste aller Hindernisse nach dem n�hsten Hinderniss in 
    // Fahrtrichtung, dass maximal 2000mm entfernt ist
    for (unsigned int i=0;i<obstacles.size();i++) {
      relobs=a2r*obstacles[i].pos;  // relative Hindernisposition
   
      // wenn das hinderniss innerhalb des aufgespannten Winkels oder
      // im Bereich von 70cm in einem gr�eren Bereich (+- 1/4 PI) ist,
      // dann untersuche das Hindernis
      if (relobs.angle(right_of_drivedir).get_rad_pi() < 0 && 
          relobs.angle(left_of_drivedir).get_rad_pi() > 0 && 
          relobs.length() < 2000 ||  
          (relobs.length() < 700 && 
           fabs(relobs.angle(desiredVtrans).get_rad_pi())< M_PI / 4)) {
        LOUT << "% black dotted circle "<< r2a*relobs << endl;
        if ((robotPos-relobs).length() < 
            (robotPos-mostdangerousobstacle).length()) {
          mostdangerousobstacle=relobs;
          obstacle_in_drivedir=true;          
        }
      }
    }
   
    if (!obstacle_in_drivedir) {  // no obstacle =>
      return desiredVtrans;       // do not have to change the drive vector
    }
    
    Vec evadepoint; // calculate evadepoint

    LOUT << "% black thick dotted circle "<< r2a*mostdangerousobstacle 
         << " 100 word " << r2a*mostdangerousobstacle+Vec(200,200) 
         << " nearest obstacle \n"; 
    
    // berechne einen punkt 1000mm links oder rechts vom hindernis
    if (desiredVtrans.angle(mostdangerousobstacle).get_rad_pi() < 0) {
      evadepoint = (r2a*mostdangerousobstacle) +
        (((r2a*mostdangerousobstacle)-robotPos).rotate(Angle::quarter).normalize() * 1000.);
    }
    else {
      evadepoint = (r2a*mostdangerousobstacle) +
        (((r2a*mostdangerousobstacle)-robotPos).rotate(-1*Angle::quarter).normalize() * 1000.);
    }
     
    // Nur wenn die Zielposition weiter als das nahegelgenste Hindernis
    // entfernt ist, muss der Fahrvektor modifiziert werden
    if  ((robotPos-targetPos).length() > 
         (robotPos-mostdangerousobstacle).length()) {
      LOUT << "Fahrtvektor zur Hindernisvermeidung modifiziert" << endl;
      LOUT << "% white line " << robotLocation.pos 
           << r2a * (desiredVtrans*500.) // seltsamerweise wird der gesetzte dv
           << " word "  << r2a * (desiredVtrans*500.) // in tribotsview
           << " original vtrans" << endl;// halbiert angezeigt, also hier auch
      return (a2r*evadepoint).normalize()*desiredVtrans.length();           
    }
    else {
      return desiredVtrans;
    }    
  }


  ///////////////////////////// from RCPlayer /////////////////////////////////

  Vec 
  WhiteBoard::getEvasiveManeuverTarget(Vec targetPos, const Time& tt) {
    CorridorData* data;
    if (dribbleCorridorData) {
      data = dynamic_cast<CorridorData*>(dribbleCorridorData);
    
      if (data->t      == tt &&
          data->refPos == targetPos &&
          data->cycle  == 
            WorldModel::get_main_world_model().get_game_state().cycle_num) {
        return data->pos;      // Zwischengespeichertes Ergebnis zurck
      }
    }
    else {                    // erste Anfrage berhaupt
      data = new CorridorData();  // Zwischenspeicher erzeugen
      dribbleCorridorData = data;
    }
    data->refPos = targetPos;
    checkDribbleCorridor(data, targetPos, tt);
    return data->pos;
  }

  bool WhiteBoard::isFreeShootCorridor(const Time& tt) {
    BoolData* data;
    if (shootCorridorData) {
      data = dynamic_cast<BoolData*>(shootCorridorData);

      if (data->t     == tt &&
          data->cycle ==
          WorldModel::get_main_world_model().get_game_state().cycle_num) {
        return data->b;      // Zwischengespeichertes Ergebnis zurck
      }
    }
    else {                    // erste Anfrage berhaupt
      data = new BoolData();  // Zwischenspeicher erzeugen
      shootCorridorData = data;
    }
    // berechnen, ob Schusskorridor bis zum Spielfeldrand frei ist
    const RobotLocation& rloc = MWM.get_robot_location (tt);
    const FieldGeometry& fgeom = MWM.get_field_geometry ();
    Vec s = rloc.pos;
    Vec t = s+3500*Vec::unit_vector (rloc.heading+Angle::quarter);  // maximal 3,5m weit Hindernisse beruecksichtigen
    double obstacle_distance = obstacle_distance_to_line_inside_field (s, t, MWM.get_obstacle_location (tt));
    LOUT << "Korridor-Margin: " << obstacle_distance << '\n';
    data->b = (obstacle_distance>0.5*fgeom.ball_diameter);
    if (data->b)
      LOUT << "% dark_green dotted thin arrow " << s << ' ' << t << '\n';
    else
      LOUT << "% dark_red dotted thin arrow " << s << ' ' << t << '\n';
    return data->b;
  }

  Vec WhiteBoard::getFreeGoalPos(const Time& tt) {
    VecData* data;
    if (freeGoalPosData) {
      data = dynamic_cast<VecData*>(freeGoalPosData);

      if (data->t     == tt &&
          data->cycle ==
          WorldModel::get_main_world_model().get_game_state().cycle_num) {
        return data->v;      // Zwischengespeichertes Ergebnis zurck
          }
    }
    else {                    // erste Anfrage berhaupt
      data = new VecData();  // Zwischenspeicher erzeugen
      freeGoalPosData = data;
    }
    // Freie Torposition berechnen
    const FieldGeometry& fgeom = MWM.get_field_geometry();
    Vec goalMiddle (0, 0.5*fgeom.field_length);
    Vec goalLeft (-600, 0.5*fgeom.field_length);
    Vec goalRight (600, 0.5*fgeom.field_length);
    const RobotLocation& rloc = MWM.get_robot_location (tt);
    const ObstacleLocation& oloc = MWM.get_obstacle_location (tt);
    bool freeMiddle = (obstacle_distance_to_line_inside_field (rloc.pos, goalMiddle, oloc)>=0.5*fgeom.ball_diameter);
    bool freeLeft = (obstacle_distance_to_line_inside_field (rloc.pos, goalLeft, oloc)>=0.5*fgeom.ball_diameter);
    bool freeRight = (obstacle_distance_to_line_inside_field (rloc.pos, goalRight, oloc)>=0.5*fgeom.ball_diameter);
    unsigned int freeState = (freeLeft ? 1 : 0)+2*(freeMiddle ? 1 : 0)+4*(freeRight ? 1 : 0);
    Vec target;
    double anglediffMiddle = abs(((goalMiddle-rloc.pos)/rloc.heading).angle().get_rad_pi());
    double anglediffLeft = abs(((goalLeft-rloc.pos)/rloc.heading).angle().get_rad_pi());
    double anglediffRight = abs(((goalRight-rloc.pos)/rloc.heading).angle().get_rad_pi());
    switch (freeState) {
      case 1: case 4: target = goalRight; break;
      case 2: target = goalMiddle; break;
      case 3: case 6: target = goalLeft; break;
      case 0: case 7: target = (anglediffLeft<anglediffMiddle ? goalLeft : (anglediffRight<anglediffMiddle ? goalRight : goalMiddle)); break;
      case 5: target = (anglediffLeft<anglediffRight ? goalLeft : goalRight);
    }
    data->v=target;
    return data->v;
  }

  bool WhiteBoard::isFreeDribbleCorridor(Vec targetPos, const Time& tt) {
    CorridorData* data;
    if (dribbleCorridorData) {
      data = dynamic_cast<CorridorData*>(dribbleCorridorData);

      if (data->t      == tt &&
          data->refPos == targetPos &&
          data->cycle  ==
          WorldModel::get_main_world_model().get_game_state().cycle_num) {
        return data->isFree;      // Zwischengespeichertes Ergebnis zurck
      }
    }
    else {                    // erste Anfrage berhaupt
      data = new CorridorData();  // Zwischenspeicher erzeugen
      dribbleCorridorData = data;
    }
    data->refPos = targetPos;
    checkDribbleCorridor(data, targetPos, tt);
    return data->isFree;
  }


  void 
  WhiteBoard::checkDribbleCorridor(CorridorData* data,
                                   Vec p_targetPos, const Time& t) {  
    // get wm-data
    Vec _robotPos = MWM.get_robot_location(t).pos;
    ObstacleLocation _obstacles = MWM.get_obstacle_location(t);

    Frame2d world2robot =  getAbs2RelFrame(t);
    Frame2d robot2world =  getRel2AbsFrame(t);

    Vec _relevantObstacleRel, _relevantObstacle, 
        _relevantObstacleRelToDriveCorridor;

    _relevantObstacle = Vec(10000.0,10000.0);    // damit die Abfrage unten funktioniert

    bool _bFreeWayToTargetTemp = 1;

    // Frame, in dem der Vektor zwischen Robotermittelpunkt und Ziel das Koordinatensystem bildet
    Frame2d driveCorridor2world;
    driveCorridor2world.set_position(_robotPos);
    driveCorridor2world.set_angle((p_targetPos-_robotPos).angle());
    Frame2d world2driveCorridor=driveCorridor2world;
    world2driveCorridor.invert();

    Vec _targetRelToDriveCorridor = world2driveCorridor*p_targetPos;

    double m_dDriveCorridorInnerWidth = 200.;  // \TODO: config file
    double m_dDriveCorridorOuterWidth = 300.;  // \TODO: config file
    double m_dRecognizeObstacleDistance = 3500.; 
    
    const FieldGeometry& fgeom = MWM.get_field_geometry();
  
    Quadrangle guestGoalieArea(
      Vec(-MWM.get_field_geometry().goal_area_width/2,
           MWM.get_field_geometry().field_length/2-MWM.get_field_geometry().goal_area_length),
      Vec(MWM.get_field_geometry().goal_area_width/2,
          MWM.get_field_geometry().field_length/2));
      
    Quadrangle _driveCorridorInner = Quadrangle(Vec(0.,0.),_targetRelToDriveCorridor,m_dDriveCorridorInnerWidth);
    Quadrangle _driveCorridorOuterRight = Quadrangle(
      Vec(0.,-((m_dDriveCorridorInnerWidth/2)+(m_dDriveCorridorOuterWidth/2))),
      Vec(_targetRelToDriveCorridor.x,-((m_dDriveCorridorInnerWidth/2)+(m_dDriveCorridorOuterWidth/2))),m_dDriveCorridorOuterWidth);
    Quadrangle _driveCorridorOuterLeft = Quadrangle(
      Vec(0.,((m_dDriveCorridorInnerWidth/2)+(m_dDriveCorridorOuterWidth/2))),
      Vec(_targetRelToDriveCorridor.x,((m_dDriveCorridorInnerWidth/2)+(m_dDriveCorridorOuterWidth/2))),m_dDriveCorridorOuterWidth);
    
      
    if (DISPLAY_DRIBBLE_CORRIDOR) {
      Quadrangle quad = driveCorridor2world*_driveCorridorInner;    
      LOUT<<"% dark_green thin dotted line "<<quad.p1.x<<" "<<quad.p1.y<<" "
          <<quad.p2.x<<" "<<quad.p2.y<<" "
          <<quad.p3.x<<" "<<quad.p3.y<<" "
          <<quad.p4.x<<" "<<quad.p4.y<<" "
          <<quad.p1.x<<" "<<quad.p1.y<<"\n";
      
      quad = driveCorridor2world*_driveCorridorOuterLeft;    
      LOUT<<"% dark_green thin dotted line "<<quad.p1.x<<" "<<quad.p1.y<<" "
          <<quad.p2.x<<" "<<quad.p2.y<<" "
          <<quad.p3.x<<" "<<quad.p3.y<<" "
          <<quad.p4.x<<" "<<quad.p4.y<<" "
          <<quad.p1.x<<" "<<quad.p1.y<<"\n"; 
          
      quad = driveCorridor2world*_driveCorridorOuterRight;    
      LOUT<<"% dark_green thin dotted line "<<quad.p1.x<<" "<<quad.p1.y<<" "
          <<quad.p2.x<<" "<<quad.p2.y<<" "
          <<quad.p3.x<<" "<<quad.p3.y<<" "
          <<quad.p4.x<<" "<<quad.p4.y<<" "
          <<quad.p1.x<<" "<<quad.p1.y<<"\n"; 
    } 
    
    // --------------------------------------------------------------------------------------
    // das naheliegenste Hinderniss ausw�len
    bool _bEvasiveManeuverNewObstacle = 0;
    vector<Vec> wayBlockingObstacles;
    wayBlockingObstacles.clear();
    for (int i=0;i<(int)_obstacles.size();i++)
    {
      Vec _obstacleRel                = world2robot*_obstacles[i].pos;
      Vec _obstacleRelToDriveCorridor = world2driveCorridor*_obstacles[i].pos;
      
      if ( (_obstacleRel.length() < m_dRecognizeObstacleDistance)&&
           (!guestGoalieArea.is_inside(_obstacles[i].pos)) ) // evtl. auf extendedGuestGoalieArea testen...
      {
        // Hinderniss im Korridor zwischen Roboter und Ziel?
        if ( _driveCorridorInner.is_inside(_obstacleRelToDriveCorridor) ||
             _driveCorridorOuterLeft.is_inside(_obstacleRelToDriveCorridor) ||
             _driveCorridorOuterRight.is_inside(_obstacleRelToDriveCorridor) )
        {
          // falls Hinderniss noch n�er als bisheriges, austauschen
          if ( _obstacleRel.length() < (_robotPos - _relevantObstacle).length() )
          {
            _relevantObstacle                   = _obstacles[i].pos;
            _relevantObstacleRel                = _obstacleRel;
            _relevantObstacleRelToDriveCorridor = _obstacleRelToDriveCorridor;

            // letzten Zustand berprfen
            if (! data->isFree)
            {
              _bEvasiveManeuverNewObstacle = 0;
            }
            else
            {
              _bEvasiveManeuverNewObstacle = 1;
            }

            _bFreeWayToTargetTemp = 0;

            wayBlockingObstacles.push_back(_relevantObstacle);
          }
        }
      }
    }

    int _iUsedWay;

    // --------------------------------------------------------------------------------------
    // Ausweichpunkt festlegen  
    if (!_bFreeWayToTargetTemp)
    {
      LOUT<<"% black thick dotted circle "<<_relevantObstacle.x<<" "<<_relevantObstacle.y<<" 100 word "<<_relevantObstacle.x+200<<" "<<_relevantObstacle.y+100<<" wbo\n"; // way blocking obstacle

      data->isFree = m_bFreeWayToTarget = 0;

      // Hinderniss im ��ren Korridorbereich
      if ( !_driveCorridorInner.is_inside(_relevantObstacleRelToDriveCorridor) )
      {
        // Hinderniss halbrechts
        if ( _driveCorridorOuterRight.is_inside(_relevantObstacleRelToDriveCorridor) )
        {
          if (_bEvasiveManeuverNewObstacle)
          {
            m_iEvasiveManeuverMainDirection = LEFT;
          }

          // letzte Richtungkorrektur wie diese
          if (m_iEvasiveManeuverMainDirection == LEFT)
          {
            // Ausweichpunkt links vom Hinderniss
            _iUsedWay = LEFT;
          }
          else
          {
            // Ausweichpunkt rechts vom Hinderniss
            _iUsedWay = RIGHT;
          }
        }
        // Hinderniss halblinks
        else
        {
          if (_bEvasiveManeuverNewObstacle)
          {
            m_iEvasiveManeuverMainDirection = RIGHT;
          }

          // letzte Richtungkorrektur wie diese
          if (m_iEvasiveManeuverMainDirection == RIGHT)
          {
            // Ausweichpunkt rechts vom Hinderniss
            _iUsedWay = RIGHT;
          }
          // letzte Richtungkorrektur anders als diese
          else
          {
            // Ausweichpunkt links vom Hinderniss
            _iUsedWay = LEFT;
          }
        }
      }
      // Hinderniss im inneren Korridorbereich
      else
      {
        // Robbie steht in der rechten Feldh�fte
        if ( _robotPos.x > 0 )      // RIGHT?
        {
          if (_bEvasiveManeuverNewObstacle)
          {
            m_iEvasiveManeuverMainDirection = LEFT;
          }

          // letzte Richtungkorrektur wie diese
          if (m_iEvasiveManeuverMainDirection == LEFT)
          {
            // Ausweichpunkt links vom Hinderniss
            _iUsedWay = LEFT;
          }
          // letzte Richtungkorrektur anders als diese
          else
          {
            // Ausweichpunkt rechts vom Hinderniss
            _iUsedWay = RIGHT;
          }
        }
        // Robbie steht in der linken Feldh�fte
        else
        {
          if (_bEvasiveManeuverNewObstacle)
          {
            m_iEvasiveManeuverMainDirection = RIGHT;
          }

          // letzte Richtungkorrektur wie diese
          if (m_iEvasiveManeuverMainDirection == RIGHT)
          {
            // Ausweichpunkt rechts vom Hinderniss
            _iUsedWay = RIGHT;
          }
          // letzte Richtungkorrektur anders als diese
          else
          {
            // Ausweichpunkt links vom Hinderniss
            _iUsedWay = LEFT;
          }
        }
      }

  //     if (_iUsedWay == RIGHT)
  //     {
  //       m_EPEvasiveManeuverTarget = _EPRelevantObstacleAbs+V2d(m_dEvasiveManeuverTargetXPos,-m_dEvasiveManeuverTargetYPos);
  //     }
  //     else
  //     {
  //       m_EPEvasiveManeuverTarget = _EPRelevantObstacleAbs+V2d(m_dEvasiveManeuverTargetXPos,m_dEvasiveManeuverTargetYPos);
  //     }

      double _dEvasiveManeuverTargetXPos = 2000.0;//m_dEvasiveManeuverTargetXPos;
      if (_relevantObstacleRel.length()<1500.0) _dEvasiveManeuverTargetXPos = 3000.0;

      if (_iUsedWay == RIGHT)
      {
        data->pos = _relevantObstacle+(p_targetPos-_relevantObstacle).rotate_quarter().normalize()*(-_dEvasiveManeuverTargetXPos);
      }
      else
      {
        data->pos = _relevantObstacle+(p_targetPos-_relevantObstacle).rotate_three_quarters().normalize()*(-_dEvasiveManeuverTargetXPos);
      }
      if ((_robotPos-_relevantObstacle).length() > 2000. &&  
          fabs(_robotPos.x) > fgeom.field_width/2.-2000. &&
          fabs(data->pos.x) > fgeom.field_width/2.) {
        if (_iUsedWay == RIGHT) {
          _iUsedWay = LEFT;
          data->pos = _relevantObstacle+(p_targetPos-_relevantObstacle).rotate_three_quarters().normalize()*(-_dEvasiveManeuverTargetXPos);
        }
        else {
          _iUsedWay = RIGHT;
          data->pos = _relevantObstacle+(p_targetPos-_relevantObstacle).rotate_quarter().normalize()*(-_dEvasiveManeuverTargetXPos);          
        }
      }
    }
    else
    {
      data->isFree = m_bFreeWayToTarget = 1;
    }
  }
	  
 double WhiteBoard::getKickTargetHeight(double distance_mm, unsigned char klength_ms) {
   distance_mm -= 300;
   double x_max_params[3];
   double y_max_params[3];

   /**   
	 x_max_params[0] = 1.8443;
	 x_max_params[1] = -13.9908;
	 x_max_params[2] = -5.20274;
	 y_max_params[0] = 1.09432;
	 y_max_params[1] = 5.70011;
	 y_max_params[2] = -2.14541;
   **/

   x_max_params[0] = ms_x_a;
   x_max_params[1] = ms_x_b;
   x_max_params[2] = ms_x_c;
   y_max_params[0] = ms_y_a;
   y_max_params[1] = ms_y_b;
   y_max_params[2] = ms_y_c;

   unsigned char min_dur = 15;
   min_dur = MAX ( min_dur , (int) ( MAX(x_max_params[1]  , y_max_params[1]))+1 );

   klength_ms = klength_ms > 60 ? 60 : klength_ms;
   klength_ms = klength_ms < min_dur ? min_dur : klength_ms;
   

   distance_mm /= 1000;
   LOUT << x_max_params[0] << "  "<< x_max_params[1] << "  " << x_max_params[2] << "  ";
   LOUT << y_max_params[0] << "  "<< y_max_params[1] << "  " << y_max_params[2] << "  ";
   double x_max = x_max_params[0] * log( (double) klength_ms - x_max_params[1]) + x_max_params[2];
   double y_max = y_max_params[0] * log( (double) klength_ms - y_max_params[1]) + y_max_params[2];
   
   
   
#if 1
   // Annahme: symmetrische parabel, Zeit bis zum Maximum (bzw. vom Maximum bis 0)
   double t_dx = sqrt( 2.0 * y_max / 9.81 ); // in Sekunden
//   Time	now;
//   RobotLocation robot = MWM.get_robot_location( now );
	Time tt_sl;
   RobotLocation            robot         = MWM.get_slfilter_robot_location(tt_sl);

   double rob_y_vel = (robot.vtrans / robot.heading).y; // in m/s (mm/ms)
   
   double s_dx = rob_y_vel * t_dx; // in m
	
	LOUT << "geschwindigkeit: " << rob_y_vel << " korrektur: " << s_dx << "\r\n";

   // verschiebe Maximalpunkt der Parabel um zur�ckgelegte Strecke
	// Unterscheidung aufsteigend / absteigend
	if (distance_mm < (x_max + 0.2 * s_dx) )	
		x_max += 0.2 * s_dx; // in m
	else
		x_max += 0.4 * s_dx; // in m 
	
#endif

   double res =  -(y_max / (x_max * x_max)) * (distance_mm - x_max)*(distance_mm - x_max) + y_max;
   res = (res < 0) ? 0 : res;
   LOUT << "Xmax, Ymax, dist: " << x_max << " " << y_max << " " << distance_mm  << " " << res << "\n";
   return res*1000.0; // in mm
 }
	  
 unsigned char WhiteBoard::getKickLength(double distance_mm, double height_mm, bool* reachable) {
   
   unsigned char min		= 255;
   double min_error		= 1000000;
   unsigned char min_time	= 17;
   unsigned char max_time	= 100;
   unsigned char time;
   double error			= 0;
   
   for(time = max_time; time >= min_time; time--) {
     error = getKickTargetHeight(distance_mm, time) - height_mm;
     if( fabs(error) < fabs(min_error)) {
       min_error = error;
       min = time;
     }
     LOUT << "time: " << (int) time << "  error: " << error << "\n";
   }

   if ( min > min_time && min_error > 0 ) min-=1; // nimm lieber zu kleine Werte
   
   if ( (min <= min_time || min >= max_time) && ( fabs(min_error) > 200) )
     {
       *reachable = false;
     }
   else
     {
       *reachable = true;
     }
   
   return min;
 }

}
