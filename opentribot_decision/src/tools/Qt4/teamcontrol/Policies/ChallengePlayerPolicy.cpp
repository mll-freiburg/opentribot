#include "ChallengePlayerPolicy.h"
#include "PolicyFactory.h"
#include "../States/RemoteBlackboard.h"
#include <map>

namespace {

class FPPBuilder : public TribotsTools::PolicyBuilder {
  static FPPBuilder the_builder;
public:
  FPPBuilder () {
    TribotsTools::PolicyFactory::get_policy_factory ()->sign_up (std::string("ChallengePlayerPolicy"), this);
  }
  TribotsTools::Policy* get_policy (const std::string&, const Tribots::ConfigReader& reader) throw (Tribots::TribotsException,std::bad_alloc) {
    return new TribotsTools::ChallengePlayerPolicy;
  }
};
FPPBuilder FPPBuilder::the_builder = FPPBuilder();
}

namespace TribotsTools {


using namespace std;
using namespace Tribots;


ChallengePlayerPolicy::ChallengePlayerPolicy() 
  : StaticPolicy(3), dynamicChangeCycleCounter(0)
{
  policy_name = "ChallengePlayerPolicy";
  playertype = "ChallengePlayer";

  roles(0,0) = "shooter";

  roles(1,0) = "shooter";
  roles(1,1) = "receiver";
  
  roles(2,0) = "shooter";
  roles(2,1) = "receiver";
  roles(2,2) = "hold";
  
  ballActualHalf = 1; // wir gehen mal davon aus, daß der ball zu beginn auf der shooter-seite liegt
}

void 
ChallengePlayerPolicy::update () throw () 
{
  /*
  if (dynamicChangeCycleCounter <= 0) {
    StaticPolicy::update();
  }
  else {
    dynamicChangeCycleCounter--;
  }
  */
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
  
  // passspiel kommunizieren
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
   
  if (num_active_robots < max_num_players-1) { // nicht genug Spieler da, 
    string fewPlayers;
    if (num_active_robots == 1) {
      fewPlayers = "Only1Robot!";
    }
    for (unsigned int i=0; i < num_robots; i++) {
      if (is_active[i]) {
        REMBB.robot_state[i].message_board.publish(fewPlayers);
      }
    }    
    cerr << "Zu wenig Spieler!"<<endl;
    return;                    // dynamische Wechsel machen keinen Sinn
  } 
  
  
  // assert: 2 (max_num_players) roboter da, alle haben desired rolle
  //         angenommen

  for (unsigned int i=0; i < num_robots; i++) {
    bool change = false;
    if (is_active[i] ) {
/*      const MessageBoard& mb = REMBB.robot_state[i].message_board;
        if (mb.scan_for_prefix("pass:") != "") {
        cerr << "pass gefunden " << endl;
        cerr << "ball.pos.x " << REMBB.robot_state[i].ball_pos.pos.x*REMBB.robot_state[i].own_half << endl;
        cerr << "rolle " << REMBB.robot_state[i].playerrole << endl; 
     }*/
      
      map<string, string> nr;

      // wenn der ball die mittellinie überrollt, wird der receiver zum shooter und umgekehrt,
      // d.h. beim receiver geht approachball (bzw. interceptball) an und der shooter geht auf ?      
      int _stacksize=10; // anzahl frames, die der ball das vorzeichen gewechselt haben muß
      bool ballSwitchedHalf=0; // generell auf 0, nur die ballposition des aktuellen shooters wird abgefragt und bestimmt den wechsel                     
      if (REMBB.robot_state[i].playerrole=="shooter") {
        if (static_cast<int>(ballstack.size())>=_stacksize) {
          ballstack.erase(ballstack.begin());
        }
        ballstack.push_back(REMBB.robot_state[i].ball_pos.pos.y*REMBB.robot_state[i].own_half);
        
        ballSwitchedHalf=1;
        for (int j=0;j<static_cast<int>(ballstack.size());j++) {
          if (ballActualHalf>0) {
            if (ballstack[j]>0) ballSwitchedHalf=0;
          }
          else {
            if (ballstack[j]<0) ballSwitchedHalf=0;
          }
          cerr << ballstack[j] << " ";
        }
        cerr << endl;
        
        if (ballSwitchedHalf) {
          //ballstack.clear();
          ballActualHalf*=-1;
          cerr << "Ball hat die Seite gewechselt!"<<endl;      
        }
      } 
      
      //if ( ballSwitchedHalf ) {
      //  nr["shooter"] = "hold";
      //  nr["receiver"] = "shooter";
      //  change = true;
      //}
      
      //if ( (REMBB.robot_state[i].playerrole=="receiver")&&ballSwitchedHalf ) {
      //  nr["shooter"] = "receiver";
      //  nr["receiver"] = "shooter";
      //  change = true;
      //}
      
      /*
      if(REMBB.robot_state[i].playerrole=="shooter") {
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
              if (REMBB.robot_state[receiverId].playerrole=="receiver") {
                  nr["shooter"]  = "receiver"; nr["receiver"]  = "shooter";
              }
            }
          }                      // ende passempfaenger wurde kommuniziert
          change = true;
        }
      }else{
        if (mb.scan_for_prefix("pass:") != "") {
          cerr << "pass gefunden" << endl;
          nr["shooter"] = "receiver";
          nr["receiver"] = "shooter";
          change = true;
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
      */
      
      if(change){
        for (unsigned int i=0; i < num_robots; i++) {
          if (is_active[i]) {
            cerr << "ersetze " << i << " mit " << nr[REMBB.robot_state[i].playerrole] << endl;
            REMBB.robot_state[i].desired_playerrole =
              nr[REMBB.robot_state[i].playerrole];
       //     if (nr[REMBB.robot_state[i].playerrole] == "ball") {
       //       REMBB.robot_state[i].message_board.publish("receivePass!"); // ups, das kriegt er ja auch in den faellen, wo es gar nicht um einen pass ging...
       //     }
          }
        }
        dynamicChangeCycleCounter = 10;
        return ;
      }
    }
  }
}


};
