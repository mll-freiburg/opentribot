
#include "GameStateManager.h"
#include "../WorldModel.h"

using namespace Tribots;
using namespace std;

GameStateManager::GameStateManager (const ConfigReader& reader, const FieldGeometry& fg) throw () : rsm (fg) {
  gs.refstate = stopRobot;
  rsm.set_state (gs.refstate);
  gs.in_game = false;
  gs.own_score = gs.opponent_score = 0;
  reader.get ("loop_time", gs.intended_cycle_time);
  gs.actual_cycle_time = gs.intended_cycle_time;
  gs.cycle_num = 0;
}

GameStateManager::~GameStateManager() throw () {;}

const GameState& GameStateManager::get () const throw () {
  return gs;
}

void GameStateManager::update () throw () {
  Time now;
  now.add_msec (-30);
  const BallLocation& bp = MWM.get_ball_location (now, false);
  const RobotLocation& rp = MWM.get_robot_location (now, false);
  const ObstacleLocation& oloc = MWM.get_obstacle_location (now, false);
  rsm.update (bp, rp.pos, rp.vtrans, oloc);
  gs.refstate = rsm.get_state ();
}

void GameStateManager::init_cycle (Time now, Time exp_exec) throw () {
  // Mittlere Zykluszeit anpassen und Iterationsnummer inkrementieren
  if (gs.cycle_num++>0)
    gs.actual_cycle_time=0.98*gs.actual_cycle_time+0.02*now.diff_msec(gs.cycle_start_time);
  gs.cycle_start_time = now;
  gs.expected_execution_time = exp_exec;
}

void GameStateManager::update (RefboxSignal sig) throw () {
  Time now;
  const BallLocation& bp = MWM.get_ball_location (now, false);
  const RobotLocation& rp = MWM.get_robot_location (now, false);
  RefboxSignal sig2 = rsm.update (sig, bp, rp.pos, rp.vtrans, rp.vrot);
  if (sig2==SIGownGoalScored)
    gs.own_score++;
  if (sig2==SIGopponentGoalScored)
    gs.opponent_score++;
  gs.latest_update.update();
  gs.refstate = rsm.get_state ();
}

bool GameStateManager::get_in_game () throw () {
  return gs.in_game;
}
void GameStateManager::set_in_game (bool b) throw () {
  gs.in_game=b;
}

void GameStateManager::set_score (unsigned int own, unsigned int opp, unsigned int yell) throw () {
  gs.own_score = own;
  gs.opponent_score = opp;
  gs.yellow_cards = yell;
}
