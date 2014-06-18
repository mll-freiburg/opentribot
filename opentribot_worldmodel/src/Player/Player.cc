
#include "Player.h"
#include "PlayerFactory.h"
#include "../Structures/Journal.h"
#include "../Structures/TacticsBoardDescription.h"
#include <cstring>

using namespace std;

const char* Tribots::Player::get_player_type () const throw () { return player_descriptor; }

Tribots::Player::~Player () throw () {
  delete the_player;
  delete [] player_descriptor;
}

Tribots::Player::Player (const ConfigReader& vread) throw (TribotsException, bad_alloc) : the_player(NULL), configuration_list(vread) {
  string confline;
  if (vread.get("player_type", confline)<=0) {
    JERROR("no config line \"player_type\" found");
    throw Tribots::InvalidConfigurationException ("player_type");
  }
  std::vector<TacticsAttribute> tacattr;
  read_tactics (tacattr, vread);
  make_default_tactics_board (tactics, tacattr);
  try{
    really_change_player_type (confline.c_str(), vread);
  }catch(TribotsException& e){
    JERROR((std::string ("creating player of type ")+confline+std::string(" failed due to: ")+std::string(e.what())).c_str());
    throw (e);
  }
}

bool Tribots::Player::change_player_type (const char* pt) throw () {
  return change_player_type (pt, configuration_list);
}

bool Tribots::Player::change_player_type (const char* pt, const ConfigReader& vread) throw () {
  try{
    really_change_player_type (pt, vread);
  }catch(bad_alloc&){
    JWARNING("Change of player type failed due to lack of memory");
    return false;
  }catch(TribotsException&){
    JWARNING("Change of player type failed");
    return false;
  }
  return true;
}

void Tribots::Player::really_change_player_type (const char* pt, const ConfigReader& vread) throw (TribotsException, bad_alloc) {
  PlayerType* new_player;
  char* new_descriptor;
  try{
    string plt (pt);
    new_player = PlayerFactory::get_player_factory()->get_player (plt, vread);
  }catch(invalid_argument){
    throw Tribots::InvalidConfigurationException ("player_type");
  }

  new_descriptor = new char [strlen(pt)+1];
  strcpy(new_descriptor,pt);
  if (the_player!=NULL) {
    delete the_player;
    delete [] player_descriptor;
  }

  the_player=new_player;
  player_descriptor=new_descriptor;
  the_player->updateTactics (tactics);
  MWM.set_player_type (player_descriptor);
  MWM.set_player_role (the_player->get_role());
}

void Tribots::Player::getPlayerTypeList(std::vector<std::string> &ptl)
{
  PlayerFactory::get_player_factory()->player_list(ptl);
}

void Tribots::Player::updateTactics (const Tribots::TacticsBoard& tb) throw (std::bad_alloc) {
  tactics = tb;
  the_player->updateTactics (tactics);
}

const Tribots::TacticsBoard& Tribots::Player::getTactics () const throw () {
  return tactics;
}

bool Tribots::Player::set_role (const char* s) throw () {
  bool success = the_player->set_role(s);
  MWM.set_player_role (the_player->get_role());
  return success;
}
