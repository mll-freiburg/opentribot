
#include "GameState.h"

using namespace Tribots;
using namespace std;

GameState::GameState () throw () {
  refstate = stopRobot;
  in_game=false;
  own_score=0;
  opponent_score=0;
  yellow_cards=0;
  intended_cycle_time=1;
  actual_cycle_time=1;
  cycle_num=0;
}

GameState::GameState (const GameState& gs) throw () {
  (*this) = gs;
}

const GameState& GameState::operator= (const GameState& gs) throw () {
  refstate = gs.refstate;
  latest_update = gs.latest_update;
  in_game = gs.in_game;
  own_score = gs.own_score;
  opponent_score = gs.opponent_score;
  yellow_cards = gs.yellow_cards;
  intended_cycle_time = gs.intended_cycle_time;
  actual_cycle_time = gs.actual_cycle_time;
  cycle_num = gs.cycle_num;
  cycle_start_time = gs.cycle_start_time;
  expected_execution_time = gs.expected_execution_time;
  return (*this);
}
