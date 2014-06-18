#include "FieldPlayer07Policy.h"
#include "PolicyFactory.h"
#include "../States/RemoteBlackboard.h"
#include <map>
#include "../Logic/CoachLogger.h"

namespace {

class FPPBuilder : public TribotsTools::PolicyBuilder {
  static FPPBuilder the_builder;
public:
  FPPBuilder () {
    TribotsTools::PolicyFactory::get_policy_factory ()->sign_up
    (std::string("Feldspieler07Policy"), this);
  }
  TribotsTools::Policy* get_policy (const std::string&, const Tribots::ConfigReader& reader) throw (Tribots::TribotsException,std::bad_alloc) {
    return new TribotsTools::FieldPlayer07Policy(reader);
  }
};
FPPBuilder FPPBuilder::the_builder = FPPBuilder();
}

namespace TribotsTools {


using namespace std;
using namespace Tribots;


static map<string,string> rotateLeft(int numRobots, bool includeSafety=false)
{
  map<string,string> nr;
  nr["ballL"] = "ballL"; nr["ballR"] = "ballR"; nr["left"] = "left";
  nr["right"] = "right"; nr["safety"] = "safety";
  if (numRobots == 3) {
    nr["ballL"] = "left";
    nr["left"] = "right";
    nr["right"] = "ballL";
  }
  if (numRobots == 4) {
    nr["ballL"] = "left";
    nr["ballR"] = "ballL";
    nr["left"] = "right";
    nr["right"] = "ballR";
  }
  if (numRobots == 5) {
    nr["ballL"] = "left";
    nr["ballR"] = "ballL";
    nr["right"] = "ballR";
    if (includeSafety) {
      nr["left"] = "safety";
      nr["safety"] = "right";
    }
    else {
      nr["left"] = "right";
    }
  }
  return nr;
}

static map<string,string> rotateRight(int numRobots, bool includeSafety=false)
{
  map<string,string> nr;
  nr["ballL"] = "ballL"; nr["ballR"] = "ballR"; nr["left"] = "left";
  nr["right"] = "right"; nr["safety"] = "safety";
  if (numRobots == 3) {
    nr["ballL"] = "right";
    nr["left"] = "ballL";
    nr["right"] = "left";
  }
  if (numRobots == 4) {
    nr["ballL"] = "ballR";
    nr["ballR"] = "right";
    nr["left"] = "ballL";
    nr["right"] = "left";
  }
  if (numRobots == 5) {
    nr["ballL"] = "ballR";
    nr["ballR"] = "right";
    nr["left"] = "ballL";
    if (includeSafety) {
      nr["right"] = "safety";
      nr["safety"] = "left";
    }
    else {
      nr["right"] = "left";
    }
  }
  return nr;
}



FieldPlayer07Policy::FieldPlayer07Policy(const ConfigReader& reader) 
  : StaticPolicy(5, reader), dynamicChangeCycleCounter(0)
{
  for (unsigned int i=0; i < 100; i++) {  // mehr als 100 roboter darfs beim Feldspieler nicht geben
    passMessageCounter.push_back(-1);
  }
  passReceived.update();
  passReceived.add_sec(-2);
  
  policy_name = "Feldspieler07Policy";
  playertype = "Feldspieler07";

  roles(0,0) = "ballL";

  roles(1,0) = "ballL";
  roles(1,1) = "safety";

  roles(2,0) = "ballL";
  roles(2,1) = "left";
  roles(2,2) = "right";

  roles(3,0) = "ballL";
  roles(3,1) = "ballR";
  roles(3,2) = "left";
  roles(3,3) = "right";
  
  roles(4,0) = "ballL";
  roles(4,1) = "ballR";
  roles(4,2) = "left";
  roles(4,3) = "right";
  roles(4,4) = "safety";
}

void 
FieldPlayer07Policy::update () throw () 
{
  int sender= -1;
  int receiver = -1;
  bool alreadyChanging = false;
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
        REMBB.robot_state[i].in_game && !REMBB.robot_state[i].comm_interrupted) {
      num_active_robots++;
      is_active[i] = true;
      if (REMBB.robot_state[i].playerrole !=
          REMBB.robot_state[i].desired_playerrole) { // solange noch ein 
        alreadyChanging = true;                      // anderer wechsel vollzogen wird, keinen 
      }                                              // neuen wechsel anstossen
    }
    else {
      is_active[i] = false;
    }
  }
  if (passReceived.elapsed_msec() < 1000) {         // eine Sekunde lang Nachrichten wiederholen und an den Empfaenger schicken
    if (num_robots > lastReceiverID && is_active[lastReceiverID]) {
      REMBB.robot_state[lastReceiverID].message_board.publish("receivePass!");
      cerr << "DRAWING_AGAIN" << endl;
      REMBB.drawRobotArrow(REMBB.robot_state[lastSenderID].id,REMBB.robot_state[lastReceiverID].id,"yellow","Pass",2000);
      cout << "repeated sending recesivePass to robot number: " << REMBB.robot_state[lastReceiverID].id << endl;
      LOUT << "repeated sending recesivePass to robot number: " << REMBB.robot_state[lastReceiverID].id << endl;
    }
  }
  
  for (unsigned int i=0; i < num_robots; i++) {
    if (is_active[i] ){
      if(REMBB.robot_state[i].message_board.scan_for_prefix("short_pass:").length() > 0) {
        string shortPassReceiver = "";
        if (REMBB.robot_state[i].ball_pos.pos.x * REMBB.robot_state[i].own_half < 1000. ||
            num_active_robots <= 3) {
          shortPassReceiver = "ballL";
        }
        else {
          shortPassReceiver = "ballR"; 
        }
        for (unsigned int r=0; r < num_robots; r++) {            
          if (is_active[r] && (REMBB.robot_state[r].playerrole==shortPassReceiver)) {
            REMBB.robot_state[r].message_board.publish("receiveSetPlayShortPass!");
            REMBB.drawRobotArrow(REMBB.robot_state[i].id,REMBB.robot_state[r].id,"blue","Kurzpass",2000);	            
          }
        }
      }
    }
  } 
  stringstream playerCounter;
  playerCounter << "Robots: " << num_active_robots;
  for (unsigned int i=0; i < num_robots; i++) {
    if (is_active[i]) {
      REMBB.robot_state[i].message_board.publish(playerCounter.str());
    }
  }
  if(num_active_robots < 3 || alreadyChanging){   // bei weniger als 3 Spieler machen 
    return;                                       // dynamische Wechsel keinen Sinn
  } 

  // assert: mindestens drei (max_num_players) roboter da, alle aktiven haben desired rolle
  //         angenommen

  string destReceiver="";  // new role of the pass receiving robot
  bool passFound = false;
  bool querPassFound = false;
  bool befreiungsschlagFound = false;
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
      nr["ballL"] = "ballL"; nr["ballR"] = "ballR"; nr["left"] = "left";
      nr["right"] = "right"; nr["safety"] = "safety";
      string bfmsg = "";
      if((bfmsg = mb.scan_for_prefix("BefreiungsschlagZu:")) != "") {
      	cerr << "Befreiungsschlag gefunden: " << bfmsg << endl;
        LOUT << "Befreiungsschlag gefunden: " << bfmsg << endl;
      	sender = i;
        istringstream str(bfmsg);
        string tmp;
        int receiverId = -1;
        unsigned int number;
        str >> tmp >> number;
        for (unsigned int r=0; r < num_robots; r++) { // empfaenger raussuchen
          if (is_active[r] && REMBB.robot_state[r].local_id == number) {
            receiverId = r;
            receiver = r;
            break;            // braucht nicht weitersuchen
          }
        }
        if (receiverId < 0) {
          cerr << "Empfaenger konnte nicht gefunden werden!" << endl;
          LOUT << "Empfaenger konnte nicht gefunden werden!" << endl;
        }
        else { // ACHTUNG: destReceiver im Juni 08 geaendert. (vorher destReceiver = nr["X"] = "Y";  was falsche Zuweisung war)
          if (REMBB.robot_state[receiverId].playerrole=="left") {
            nr["ballL"]  = "left"; nr["left"]  = "ballL"; 
          }
          else if (REMBB.robot_state[receiverId].playerrole=="right") { // playerrole == "right"
            if (num_active_robots == 3) {
              nr["ballL"]  = "right"; nr["right"] = "ballL";   
            }
            else { // num_active_robots > 3
              nr["ballR"]  = "right"; nr["right"]  = "ballR"; 
            }
          }   // keine paesse zum safety, ballL und ballR keine Rotation noetig!
          destReceiver = REMBB.robot_state[receiverId].playerrole;
          change = true;
          befreiungsschlagFound = true;
        }
      }
      else if(REMBB.robot_state[i].playerrole=="ballL" || REMBB.robot_state[i].playerrole=="ballR") {  
        // wer benutzt diese Kommunikation? ( ) freistoesse    ( X ) Spontaneous Pass  ( X ) PassBeforeGoal  
        string msg = mb.scan_for_prefix("pass:");
        string tmp = mb.scan_for_prefix("querpass:");
        bool querPassStringFound = false;
        if (tmp != "") { msg = tmp; querPassStringFound = true; } // beide passarten verwenden die gleiche Rotation
        if (msg != "") {
          cerr << "pass gefunden: " << msg << endl;
          LOUT << "pass gefunden: " << msg << endl;
          sender = i;
          istringstream str(msg);
          string tmp;
          unsigned int number;
          str >> tmp >> number;
          if (str) {              // passempfaenger wurde kommuniziert
            cerr << "Empfaenger: " << number << endl;
            LOUT << "Empfaenger: " << number << endl;
            int receiverId = -1;
            for (unsigned int r=0; r < num_robots; r++) { // empfaenger raussuchen
              if (is_active[r] && REMBB.robot_state[r].id == number) {
                receiverId = r;
                receiver = r;
                break;            // braucht nicht weitersuchen
              }
            }
            int messageID = -1;
            if (str)  {
              str >> messageID;
            }
            if (receiverId < 0) {
              cerr << "Empfaenger konnte nicht gefunden werden!" << endl;
              LOUT << "Empfaenger konnte nicht gefunden werden!" << endl;
            }
            else if (messageID > 0 && messageID == passMessageCounter[i] ) { // falls die nachricht mit einer ID versehen war:
                                   // nicht noch einmal verarbeiten, wenn sie schon zu einem frueheren Zeitpunkt verarbeitet
                                   // wurde.
              cerr << "Diese Nachricht habe ich schon gehoert und weitergeleitet!" << endl;
              LOUT << "Diese Nachricht habe ich schon gehoert und weitergeleitet!" << endl;
            }
            else {
              if (messageID > 0) { // Merken, dass die akutelle Nachricht verarbeitet wurde und nicht noch mal verarbeitet werden
                passMessageCounter[i] = messageID;  // muss, wenn sie zu einem spaeteren Zeitpunkt ein weiteres Mal ankommt
              }
              if (REMBB.robot_state[i].robot_pos.pos.x * REMBB.robot_state[i].own_half > 0.) {
                if (REMBB.robot_state[receiverId].playerrole=="left") {
                  if (num_active_robots == 3) {
                    nr["ballL"]  = "right"; destReceiver = "left"; nr["left"]  = "ballL"; nr["right"] = "left"; 
                  }
                  else { // 2er rotation (spaeter vielleicht 4er?)
                    nr["ballL"]  = "left"; destReceiver = "left"; nr["left"]  = "ballL";                   }
                }
                else if (REMBB.robot_state[receiverId].playerrole=="right") { // playerrole == "right"
                  if (num_active_robots == 3) {
                    nr["ballL"]  = "right"; destReceiver = "right";  nr["right"] = "ballL";   
                  }
                  else {
                    // 2er rotation (spaeter vielleicht 4er?)
                    nr["ballR"]  = "right"; destReceiver = "right";  nr["right"]  = "ballR"; 
                  }
                }   // keine paesse zum safety, ballL und ballR keine Rotation noetig!
              }
              
              else {  
                if (REMBB.robot_state[receiverId].playerrole=="left") {
                  nr["ballL"]  = "left"; destReceiver = "left"; nr["left"]  = "ballL";
                }
                else if (REMBB.robot_state[receiverId].playerrole=="right") { 
                  if (num_active_robots == 3) {
                    nr["ballL"]  = "left"; nr["left"]  = "right"; destReceiver = "right";  nr["right"] = "ballL";   
                  }
                  else {
                    nr["ballR"]  = "right";  destReceiver = "right"; nr["right"] = "ballR";   
                  }
                }
              }
            }
          }                     
          else if (REMBB.robot_state[i].robot_pos.pos.x * REMBB.robot_state[i].own_half > 0.) {
            if (num_active_robots == 3) {
              nr["ballL"]  = "right";
              destReceiver = "left";
              nr["left"]  = "ballL"; 
              nr["right"] = "left";
            }
            else {
              nr["ballL"]  = "ballR";
              nr["ballR"]  = "right";
              destReceiver = "left";
              nr["left"]  = "ballL"; 
              nr["right"] = "left";
            }
          }
          else {
            if (num_active_robots == 3) {
              nr["ballL"]  = "left";
              nr["left"]  = "right";
              destReceiver = "right";
              nr["right"] = "ballL";       
            }
            else {
              nr["ballL"]  = "left";
              nr["ballR"]  = "ballL";
              nr["left"]  = "right";
              destReceiver = "right";
              nr["right"] = "ballR";    
            }
          }
          change = true;
          passFound = !querPassStringFound;
          querPassFound = querPassStringFound;
        }   // ende passempfaenger wurde kommuniziert
		
		
        //ballL oder ballR wurden ueberspielt
        if (!change && dynamicChangeCycleCounter <= 0) {
          if (mb.scan_for_prefix("help") != "") {
            REMBB.drawRobotText(REMBB.robot_state[i].id,"red","Help!",4000);
            cerr << "Help gehoert." << endl;
            LOUT << "Help gehoert." << endl;
            if (REMBB.robot_state[i].ball_pos.pos.y < REMBB.team_state.field_geometry.penalty_area_length * 1.3) {
              cerr << "Keine sinnvolle Rotation moeglich, Ball ist zu nah an der Grundlinie" << endl;
              LOUT << "Keine sinnvolle Rotation moeglich, Ball ist zu nah an der Grundlinie" << endl;
            }
            else {
              if (num_active_robots == 3 && REMBB.robot_state[i].playerrole == "ballL") {
                if (REMBB.robot_state[i].robot_pos.pos.x * REMBB.robot_state[i].own_half >
                    REMBB.robot_state[i].ball_pos.pos.x*REMBB.robot_state[i].own_half) { 
                  //ballL auf linker seite: CW
                  nr = rotateRight(3);
                } else { //ballL auf rechter seite: CCW
                  nr = rotateLeft(3);
                }
                change = true;
              } else if (num_active_robots >= 4) {
                if(REMBB.robot_state[i].playerrole == "ballL" &&
                   REMBB.robot_state[i].robot_pos.pos.x * REMBB.robot_state[i].own_half >
                   REMBB.robot_state[i].ball_pos.pos.x*REMBB.robot_state[i].own_half    ) {
                  nr = rotateRight(num_active_robots, true);
                } else if (REMBB.robot_state[i].playerrole == "ballR" &&
                   REMBB.robot_state[i].robot_pos.pos.x * REMBB.robot_state[i].own_half <
                   REMBB.robot_state[i].ball_pos.pos.x*REMBB.robot_state[i].own_half    ) {
                  nr = rotateLeft(num_active_robots, true); 
                }
                change = true;
              }
            }
          }
        }

      }else{
        // dieser block ist fuer standards, oder?? -> Jau scheint so, weil das quasi der einzige fall ist, wo der passgeber nicht ballbesitzer ist....
        if(REMBB.robot_state[i].playerrole=="left" && REMBB.robot_state[i].ball_pos.pos.x*REMBB.robot_state[i].own_half < 700){  
          if (mb.scan_for_prefix("pass:") != "") {
            cerr << "pass gefunden" << endl;
            LOUT << "pass gefunden" << endl;
            sender = i;
            for (unsigned int s = 0; s < num_robots; s++) {
              if (is_active[s] && REMBB.robot_state[s].playerrole == "ballL") {
                REMBB.robot_state[s].message_board.publish("NearBall!"); // dafuer sorgen, dass der neue rechte verteidiger nicht sofort zum Ball f�hrt
              }
            }
            if (num_active_robots==3) {
              nr["ballL"] = "right";
              nr["left"] = "left";
              destReceiver = "right";
              nr["right"] = "ballL"; // ballL, weil kein ballR und ballL fuer beide seiten zustaendig.
            }
            else {
              nr["ballL"] = "right";
              nr["ballR"] = "ballL";
              nr["left"] = "left";
              destReceiver = "right";
              nr["right"] = "ballR";
            }
            change = true;
          }
        } else if(REMBB.robot_state[i].playerrole=="right" && REMBB.robot_state[i].ball_pos.pos.x*REMBB.robot_state[i].own_half >= 700){
          if (mb.scan_for_prefix("pass:") != "") {
            cerr << "pass gefunden" << endl;
            LOUT << "pass gefunden" << endl;
            sender = i;
            for (unsigned int s = 0; s < num_robots; s++) {
              if (is_active[s] && (REMBB.robot_state[s].playerrole == "ballR" ||
                                   (REMBB.robot_state[s].playerrole == "ballL" && num_active_robots==3))) {
                REMBB.robot_state[s].message_board.publish("NearBall!"); // dafuer sorgen, dass der neue linke verteidiger nicht sofort zum Ball f�hrt
              }
            }
            if (num_active_robots==3) {
              nr["ballL"] = "left";
              nr["right"] = "right";
              destReceiver = "left";
              nr["left"] = "ballL";
            }
            else {
              nr["ballL"] = "ballR";
              nr["ballR"] = "left";
              nr["right"] = "right";
              destReceiver = "left";
              nr["left"] = "ballL";
            }
            change = true;
            passFound = true; // ACHTUNG: Auf welchem Wert steht hier receiver und receiverID??? Im Moment werden die Werte wohl nicht korrekt gesetzt. Das scheint mir ein Bug zu sein...
          }
        }
        if (!change && dynamicChangeCycleCounter <= 0) {  // noch kein Wechsel angesagt, dann test ob ballbesitz
          if (mb.scan_for_prefix("request_role_ball") != "") {
            if(REMBB.robot_state[i].playerrole=="left") {
              cerr << "request_role_ball gefunden" << endl;
              LOUT << "request_role_ball gefunden" << endl;
              nr["ballL"] = "left";
              nr["left"] = "ballL";
              change = true;	      
            }
            else if(REMBB.robot_state[i].playerrole=="right") {
              cerr << "request_role_ball gefunden" << endl;
              LOUT << "request_role_ball gefunden" << endl;
              if (num_active_robots == 3) {
                nr["ballL"] = "right";
                nr["right"] = "ballL";
              }
              else {
                nr["ballR"] = "right";
                nr["right"] = "ballR";
              }
              change = true;
            }
            else if(REMBB.robot_state[i].playerrole=="safety") {
              cerr << "request_role_ball gefunden" << endl;
              LOUT << "request_role_ball gefunden" << endl;
              nr["ballL"] = "left";
              nr["left"] = "safety";
              nr["safety"] = "ballL";
              change = true;
            }
          }
          else if (num_active_robots >= 4 && mb.scan_for_prefix("request_switch_lr") != "") {
            if (REMBB.robot_state[i].playerrole=="ballL") {
              cerr << "ballL moechte ballR werden" << endl;
              LOUT << "ballL moechte ballR werden" << endl;
              nr["ballL"] = "ballR";
              nr["ballR"] = "right";
              nr["right"] = "left";
              nr["left"] = "ballL";
              change = true;
            } else {
              cerr << "ballR moechte ballL werden" << endl;
              LOUT << "ballR moechte ballL werden" << endl;
              nr["ballR"] = "ballL";  
              nr["ballL"] = "left";
              nr["left"]  = "right";
              nr["right"] = "ballR";
              change = true;
            }
          }
        }
      }
      if (change) {   // Rollen werden gewechselt, fuer 10 Zyklen nix anderes mehr machen
        dynamicChangeCycleCounter = 10;
      }
      if(change || passFound || befreiungsschlagFound){ // es gibt etwas an einen oder mehrere Roboter mitzuteilen
        for (unsigned int i=0; i < num_robots; i++) {
          if (is_active[i]) {
            if (change) {
              cerr << "ersetze " << i << " mit " << nr[REMBB.robot_state[i].playerrole] << endl;
              LOUT << "ersetze " << i << " mit " << nr[REMBB.robot_state[i].playerrole] << endl;
              if (REMBB.robot_state[i].desired_playerrole != nr[REMBB.robot_state[i].playerrole]) {
                REMBB.drawRobotText(REMBB.robot_state[i].id, "yellow", nr[REMBB.robot_state[i].playerrole].c_str(), 2000);
                REMBB.robot_state[i].desired_playerrole = nr[REMBB.robot_state[i].playerrole];
              }
            }
            if (nr[REMBB.robot_state[i].playerrole] == destReceiver && (passFound || querPassFound)) {
              passReceived.update();
              lastReceiverID = i;
              lastSenderID = sender;
              REMBB.robot_state[i].message_board.publish("receivePass!");
              cout << "sending receivePass to robot: " << i << endl;
              LOUT << "sending receivePass to robot: " << i << endl;
              REMBB.drawRobotArrow(REMBB.robot_state[sender].id,REMBB.robot_state[i].id,"yellow","Pass",2000);
              if (receiver != i) {
                cout << "!!!! ACHTUNG FEHLER: receiver != i :::: " << receiver << "!=" << i << endl;
                LOUT << "!!!! ACHTUNG FEHLER: receiver != i :::: " << receiver << "!=" << i << endl;
              }
            }
            else if (nr[REMBB.robot_state[i].playerrole] != destReceiver && querPassFound) {
              REMBB.robot_state[i].message_board.publish("stayAwayFromPassingLane!");  // andere roboter duerfen passempfaenger nicht in die quere kommen
            }
            if (nr[REMBB.robot_state[i].playerrole] == destReceiver && befreiungsschlagFound) {
              REMBB.robot_state[i].message_board.publish("NachVorne!");
              REMBB.drawRobotArrow(REMBB.robot_state[sender].id,REMBB.robot_state[i].id,"red","Befreiungsschlag",2000);
            }
          }
        }
        return ;
      }
    }
  }
}


}
