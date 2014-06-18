#include "FieldPlayerPolicy.h"
#include "PolicyFactory.h"
#include "../States/RemoteBlackboard.h"
#include <map>

namespace {

class FPPBuilder : public TribotsTools::PolicyBuilder {
  static FPPBuilder the_builder;
public:
  FPPBuilder () {
    TribotsTools::PolicyFactory::get_policy_factory ()->sign_up (std::string("FeldspielerPolicy"), this);
  }
  TribotsTools::Policy* get_policy (const std::string&, const Tribots::ConfigReader& reader) throw (Tribots::TribotsException,std::bad_alloc) {
    return new TribotsTools::FieldPlayerPolicy(reader);
  }
};
FPPBuilder FPPBuilder::the_builder = FPPBuilder();
}

namespace TribotsTools {


using namespace std;
using namespace Tribots;


FieldPlayerPolicy::FieldPlayerPolicy(const ConfigReader& config) 
  : StaticPolicy(3, config), dynamicChangeCycleCounter(0)
{
  policy_name = "FeldspielerPolicy";
  playertype = "Feldspieler";

  roles(0,0) = "ball";

  roles(1,0) = "ball";
  roles(1,1) = "left";

  roles(2,0) = "ball";
  roles(2,1) = "left";
  roles(2,2) = "right";
}

void 
FieldPlayerPolicy::update () throw () 
{
  if (dynamicChangeCycleCounter <= 0) {
    StaticPolicy::update();
  }
  else {
    dynamicChangeCycleCounter--;
  }
  unsigned int num_robots = REMBB.robot_state.size();
  unsigned int num_active_robots = 0;
  vector<bool> is_active(num_robots);
  for (unsigned int i=0; i < num_robots; i++) {
    if (REMBB.robot_state[i].playertype==playertype &&
        REMBB.robot_state[i].in_game) {
      num_active_robots++;
      is_active[i] = true;
      if (REMBB.robot_state[i].playerrole !=
          REMBB.robot_state[i].desired_playerrole) { // solange noch ein 
        return;                // anderer wechsel vollzogen wird, keinen 
      }                        // neuen wechsel anstossen
    }
    else {
      is_active[i] = false;
    }
  }
  for (unsigned int i=0; i < num_robots; i++) {
    if (is_active[i] ){
      if(REMBB.robot_state[i].message_board.scan_for_prefix("short_pass:").length() > 0) {
        for (unsigned int r=0; r < num_robots; r++) {
          if (is_active[r] && (REMBB.robot_state[r].playerrole=="ball")) {
            REMBB.robot_state[r].message_board.publish("receivePass!");
          }
        }
      }
    }
  }  
  if (num_active_robots < max_num_players) { // nicht genug Spieler da, 
    string fewPlayers;
    if (num_active_robots == 2) {
      fewPlayers = "Only2Robots!";
    }
    if (num_active_robots == 1) {
      fewPlayers = "Only1Robot!";
    }
    for (unsigned int i=0; i < num_robots; i++) {
      if (is_active[i]) {
        REMBB.robot_state[i].message_board.publish(fewPlayers);
      }
    }    
    return;                    // dynamische Wechsel machen keinen Sinn
  } 

  // assert: drei (max_num_players) roboter da, alle haben desired rolle
  //         angenommen

  bool passFound = false;
  for (unsigned int i=0; i < num_robots; i++) {
    bool change = false;
    if (is_active[i] ) {
      const MessageBoard& mb = REMBB.robot_state[i].message_board;
/*      if (mb.scan_for_prefix("pass:") != "") {
        cerr << "pass gefunden " << endl;
        cerr << "ball.pos.x " << REMBB.robot_state[i].ball_pos.pos.x*REMBB.robot_state[i].own_half << endl;
        cerr << "rolle " << REMBB.robot_state[i].playerrole << endl; 
     }*/
      
      map<string, string> nr;
      if(REMBB.robot_state[i].playerrole=="ball") {
        string msg = mb.scan_for_prefix("pass:");
        if (msg != "") {
          cerr << "pass gefunden: " << msg << endl;
          istringstream str(msg);
          string tmp;
          unsigned int number;
          str >> tmp >> number;
          if (str) {              // passempfaenger wurde kommuniziert
            cerr << "Empfaenger: " << number << endl;
            int receiverId = -1;
            for (unsigned int r=0; r < num_robots; r++) { // empfaenger raussuchen
              if (is_active[r] && REMBB.robot_state[r].local_id == number) {
                receiverId = r;
                break;            // braucht nicht weitersuchen
              }
            }
            if (receiverId < 0) {
              cerr << "Empfaenger konnte nicht gefunden werden!" << endl;
            }
            else {
              if (REMBB.robot_state[i].robot_pos.pos.x * REMBB.robot_state[i].own_half > 0.) {
                if (REMBB.robot_state[receiverId].playerrole=="left") {
                  nr["ball"]  = "right"; nr["left"]  = "ball"; nr["right"] = "left";
                }
                else { // playerrole == "right"
                  nr["ball"]  = "right"; nr["left"]  = "left"; nr["right"] = "ball";   
                }
              }
              else {
                if (REMBB.robot_state[receiverId].playerrole=="left") {
                  nr["ball"]  = "left"; nr["left"]  = "ball"; nr["right"] = "right";
                }
                else { // playerrole == "right"
                  nr["ball"]  = "left"; nr["left"]  = "right"; nr["right"] = "ball";   
                }
              }
            }
          }                      // ende passempfaenger wurde kommuniziert
          else if (REMBB.robot_state[i].robot_pos.pos.x * REMBB.robot_state[i].own_half > 0.) {
            nr["ball"]  = "right";
            nr["left"]  = "ball"; 
            nr["right"] = "left";
          }
          else {
            nr["ball"]  = "left";
            nr["left"]  = "right";
            nr["right"] = "ball";          
          }
          change = true;
          passFound = true;
        }
      }else{
        if(REMBB.robot_state[i].playerrole=="left" && REMBB.robot_state[i].ball_pos.pos.x*REMBB.robot_state[i].own_half < 700){
          if (mb.scan_for_prefix("pass:") != "") {
            cerr << "pass gefunden" << endl;
            nr["ball"] = "right";
            for (unsigned int s = 0; s < num_robots; s++) {
              if (is_active[s] && REMBB.robot_state[s].playerrole == "ball") {
                REMBB.robot_state[s].message_board.publish("NearBall!"); // dafür sorgen, dass der neue rechte verteidiger nicht sofort zum Ball fährt
              }
            }
            nr["left"] = "left";
            nr["right"] = "ball";
            change = true;
          }
        } else if(REMBB.robot_state[i].playerrole=="right" && REMBB.robot_state[i].ball_pos.pos.x*REMBB.robot_state[i].own_half >= 700){
          if (mb.scan_for_prefix("pass:") != "") {
            cerr << "pass gefunden" << endl;
            nr["ball"] = "left";
            for (unsigned int s = 0; s < num_robots; s++) {
              if (is_active[s] && REMBB.robot_state[s].playerrole == "ball") {
                REMBB.robot_state[s].message_board.publish("NearBall!"); // dafür sorgen, dass der neue linke verteidiger nicht sofort zum Ball fährt
              }
            }
            nr["left"] = "ball";
            nr["right"] = "right";
            change = true;
            passFound = true;
          }
        }
        if (!change && dynamicChangeCycleCounter <= 0) {  // noch kein Wechsel angesagt, dann test ob ballbesitz
          if (mb.scan_for_prefix("request_role_ball") != "") {
            if(REMBB.robot_state[i].playerrole=="left") {
              cerr << "request_role_ball gefunden" << endl;
              nr["ball"] = "left";
              nr["left"] = "ball";
              nr["right"] = "right";
              change = true;	      
            }
            else if(REMBB.robot_state[i].playerrole=="right") {
              cerr << "request_role_ball gefunden" << endl;
              nr["ball"] = "right";
              nr["left"] = "left";
              nr["right"] = "ball";
              change = true;
            }
          }
        }
      }
      if(change){
        for (unsigned int i=0; i < num_robots; i++) {
          if (is_active[i]) {
            cerr << "ersetze " << i << " mit " << nr[REMBB.robot_state[i].playerrole] << endl;
            REMBB.robot_state[i].desired_playerrole =
              nr[REMBB.robot_state[i].playerrole];
            if (nr[REMBB.robot_state[i].playerrole] == "ball" && passFound) {
              REMBB.robot_state[i].message_board.publish("receivePass!");            }
          }
        }
        dynamicChangeCycleCounter = 10;
        return ;
      }
    }
  }
}


};
