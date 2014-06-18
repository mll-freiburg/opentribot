#include "RLDemoPlayer.h"
#include "PlayerFactory.h"
#include "WhiteBoard.h"
#include "../Fundamental/geometry.h"
#include "../WorldModel/WorldModel.h"
#include "../WorldModel/FataMorgana.h"
#include "../Behavior/Behaviors/ApproachingBall/BComplexApproachBallFreePlay.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallFromBehindPointingToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallStraightToGoalEvadeSidewards.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BEigenMove.h"
#include "../Behavior/Behaviors/BasicMovements/BShootEmergency.h"
#include "../Behavior/Behaviors/BasicMovements/BShoot.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Libs/n++/include/n++.h"
#include "../privat/Sascha/CLSquareBot/CLSquareBot.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (
        string("RLDemoPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, 
			    PlayerType*) 
      throw (TribotsException,std::bad_alloc) 
    {
      return new RLDemoPlayer (reader);
    }
  };
  Builder the_builder;
}

static const char *_all_roles[8] = { "netz1", "netz2", "netz3", "netz4", 
                                     "netz5", "netz6", "netz7", "ball" };
static string _role = "netz1";

class BallStack : public BDIBehavior {
public:
  BallStack() throw () : BDIBehavior("BallStack") {
    options.push_back (new BShootEmergency());
    options.push_back (new BShoot());
    options.push_back (new BEigenMove());
    options.push_back (new BDribbleBallStraightToGoalEvadeSidewards());
    options.push_back (new BDribbleBallToGoal());
    options.push_back (new BComplexApproachBallFreePlay());
    options.push_back (new BEmergencyStop());
  }
  virtual ~BallStack() throw() {};
  
  virtual bool checkInvocationCondition(const Time& t) throw() {
    // const FieldGeometry& fgeom = MWM.get_field_geometry();
    // const RobotLocation& robot = MWM.get_robot_location(t);
    const BallLocation&  ball  = MWM.get_ball_location(t);
    
    if (ball.pos_known == BallLocation::unknown || 
        ball.pos_known == BallLocation::raised) {
      return false;
    }
    if (_role != string("ball")) {
      return false;
    }
    return true;
  }
  
  virtual bool checkCommitmentCondition(const Time& t) throw() {
    const BallLocation&  ball  = MWM.get_ball_location(t);
    if (ball.pos_known == BallLocation::unknown || 
        ball.pos_known == BallLocation::raised) {
      return false;
    }
    return true;
  }
  
};

class SGoToPosNFQ : public Skill {
public:
  SGoToPosNFQ() : Skill("SGoToPosNFQ"), net(0) {
    stateEncoder = new GoToPos3DimEncoder(Vec(0,0));
  }
  ~SGoToPosNFQ() throw() { if (net!=0) delete net; delete stateEncoder; }
  
  void setTarget(const Vec& target, const Vec& heading) {
    if (target != this->target) {
      this->target = target;
      delete stateEncoder;
      stateEncoder = new GoToPos3DimEncoder(target);
    }
    this->heading = heading;
  }
  void loadNet(const string& filename) {
    if (net) {
      delete net;
    }
    net = new Net();
    net->load_net(const_cast<char*>(filename.c_str()));
  }
  virtual DriveVector getCmd(const Time& t) throw (Tribots::TribotsException) {
    const RobotLocation& robot = MWM.get_robot_location(t);
    double state[3];
    double actions[5] = { -2., -1., 0., 1., 2. };
    
    float qMin = 999999.;  // maximale netzausgabe 
    int iMin = 0;
    stateEncoder->encodeState(state, t);
    net->in_vec[0] = state[0];
    net->in_vec[1] = state[1];
    net->in_vec[2] = state[2];

    for (int i=0; i<5; i++) {
      net->in_vec[3] = actions[i];
      net->forward_pass(net->in_vec, net->out_vec);
      LOUT << "Action " << i << " value: \t" << net->out_vec[0] << "\r\n";      
      if (net->out_vec[0] < qMin) {
        qMin = net->out_vec[0];
        iMin = i;
      }      
    }       
    double action = actions[iMin];
    DriveVector dv = stateEncoder->decodeAction(&action, t);
    if ((target-robot.pos).length() > 2300.) {
      dv.vtrans = (WBOARD->getAbs2RelFrame(t) * target).normalize() * 2.;
    }	    
    return dv;
  }
protected:
  Vec target;
  Vec heading;
  
  double lastAction;
  
  Net* net;
  StateActionEncoder* stateEncoder;
};

class BRLDemo: public Behavior {
public:
  BRLDemo(const vector<string>& netFilenames,
          const vector<Vec>& waypoints) 
  : Behavior("BRLDemo", true), nextWaypoint(0), waypoints(waypoints),
    netFilenames(netFilenames), useNet(0)
  {
      skill = new SGoToPosNFQ();
  }

  virtual ~BRLDemo() throw() {
    delete skill;
  }
  
  virtual DriveVector getCmd(const Time& t) throw (Tribots::TribotsException) {
    const RobotLocation& robot = MWM.get_robot_location(t);
  
    if (lastRole != _role) {
      useNet = 0;
      if (_role == "netz1") {
        useNet = 0;
      }
      else if (_role == "netz2") {
        useNet = 1;
      }
      else if (_role == "netz3") {
        useNet = 2;
      }
      else if (_role == "netz4") {
        useNet = 3;
      }
      else if (_role == "netz5") {
        useNet = 4;
      }
      else if (_role == "netz6") {
        useNet = 5;
      }
      else if (_role == "netz7") {
	  useNet = 7;
      }
      else { // ball
        useNet = 9999;
      }
      if (useNet >= netFilenames.size()) { // netze 0-7
        useNet = netFilenames.size()-1;
      }
      lastRole = _role;
      
      skill->loadNet(netFilenames[useNet]);
    }
    
    if ((waypoints[nextWaypoint]-robot.pos).length() < 100. &&
        (robot.vtrans.length() < 300. || useNet >= 6)) {
      nextWaypoint = (nextWaypoint + 1) % waypoints.size();
    }
    skill->setTarget(waypoints[nextWaypoint],
                     waypoints[nextWaypoint]-robot.pos);
    return skill->getCmd(t);
  }
  
protected:
  int nextWaypoint;
  
  vector<Vec> waypoints;
  vector<string> netFilenames;
  
  SGoToPosNFQ* skill;
  
  string lastRole;
  unsigned int useNet;
};


class VirtObstacles : public FataMorgana {
  virtual void update(WorldModelTypeBase* wm) throw() {
    LOUT << "Virtual obstacles added" << endl;
    wm->add_obstacle_absolute(Vec(0, 1000.), 500.);
    wm->add_obstacle_absolute(Vec(-500, 3000.), 500);
    wm->add_obstacle_absolute(Vec(700, 2000.), 500);
  }
};

bool
RLDemoPlayer::set_role(const char* role) throw ()
{
  if (MultiRolePlayer::set_role(role)) {
    _role = std::string(role);      // fuer "interne" behaviors
    return true;
  }
  return false;
}

RLDemoPlayer::RLDemoPlayer (const ConfigReader& cfg) throw () 
  : BehaviorPlayer ("FieldPlayer", _all_roles, 7) 
{
//  MWM.add_fata_morgana(new VirtObstacles());
    
  // Das WhiteBoard veranlassen, die Configs auzuwerten ///////////////////////
  WhiteBoard::getTheWhiteBoard()->readConfigs(cfg);
  const FieldGeometry& fgeom = MWM.get_field_geometry();
  
  // Area, die der Roboter nicht verlassen darf festlegen (Training) //////////
  XYRectangle area(Vec( fgeom.field_width/2.  - 500., 
                        fgeom.field_length/2.),
                   Vec(-fgeom.field_width/2.  + 500.,
                       -fgeom.field_length/2.)); 
  Vec target(0., 0.);

  vector<string> netFilenames;
  if (cfg.get(("RLDemoPlayer::net_filenames"), netFilenames) <= 0) {
    throw InvalidConfigurationException("RLDemoPlayer::net_filenames");
  }
  
  vector<double> waypointsTmp;
  if (cfg.get(("RLDemoPlayer::waypoints"), waypointsTmp) <= 0) {
    throw InvalidConfigurationException("RLDemoPlayer::waypoints");
  }     
  vector<Vec> waypoints(waypointsTmp.size()/2);
  for (unsigned int i=0; i < waypoints.size(); i++) {
    waypoints[i] = Vec(waypointsTmp[i*2], waypointsTmp[i*2+1]);
  }

  // Optionsstack fuellen /////////////////////////////////////////////////////

  options.push_back (new BGameStopped());
  options.push_back (new BStayInsideArea(area, target)); // Training
  options.push_back (new BRLDemo(netFilenames, waypoints));
  options.push_back (new BallStack());
  options.push_back (new BEmergencyStop());
}

RLDemoPlayer::~RLDemoPlayer () throw () 
{}
