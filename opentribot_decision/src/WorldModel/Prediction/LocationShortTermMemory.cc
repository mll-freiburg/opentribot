
#include "LocationShortTermMemory.h"

using namespace Tribots;
using namespace std;

// Konvention zur Speicherung der Informationen in den Ringpuffern:
// - der Anker weist stets auf den neuesten Eintrag
// - Anker+1 zeigt auf den zweit-neuesten Eintrag, Anker+2 auf den dritt-neuesten usw.

namespace {
  const unsigned int num=15;  // Groesse der Ringpuffer

  template<class Triple>
  int search_ringbuffer (const RingBuffer<Triple>& buffer, unsigned long int cycle, Time t) throw() {
    // Ringpuffer nach Zyklus und Zeitstempel untersuchen, -1 bei Misserfolg 
    for (unsigned int i=0; i<num; i++)
      if (buffer[i].cycle==cycle && buffer[i].timestamp.get_msec()==t.get_msec())
        return i;
    return -1;
  }
}

LocationShortTermMemory::LocationShortTermMemory (const WorldModelTypeBase& w) throw (std::bad_alloc) : wm(w), robots_pure(num), robots_interacted(num), balls_pure(num), balls_interacted(num) {
  for (unsigned int i=0; i<num; i++)
    robots_pure[i].cycle=robots_interacted[i].cycle=balls_pure[i].cycle=balls_interacted[i].cycle=0;
  obstacles.cycle=0;
}

const RobotLocation& LocationShortTermMemory::get_robot_location (Time t, bool b) throw () {
  if (b) {
    int index = search_ringbuffer (robots_interacted, wm.get_game_state().cycle_num, t);
    if (index>=0)
      {
       return robots_interacted[index].value;
      }
    else {
      get_interacted (t);
       if ((robots_interacted.get().value.pos.x!=robots_interacted.get().value.pos.x) || (robots_interacted.get().value.pos.y!=robots_interacted.get().value.pos.y) || (robots_interacted.get().value.heading.get_rad()!=robots_interacted.get().value.heading.get_rad()))
       {
         robots_interacted.get().value.pos=Vec(0,0);
         robots_interacted.get().value.heading=Angle(0);
       }
       return robots_interacted.get().value;
    }
  } else {
    int index = search_ringbuffer (robots_pure, wm.get_game_state().cycle_num, t);
    if (index>=0)
      {

       return robots_pure[index].value;
      }
    else {
      robots_pure.step(-1);
      robots_pure.get().value = wm.get_robot(t);
      robots_pure.get().cycle = wm.get_game_state().cycle_num;
      robots_pure.get().timestamp = t;
      if ((robots_pure.get().value.pos.x!=robots_pure.get().value.pos.x) || (robots_pure.get().value.pos.y!=robots_pure.get().value.pos.y) || (robots_pure.get().value.heading.get_rad()!=robots_pure.get().value.heading.get_rad()))
      {
        robots_pure.get().value.pos=Vec(0,0);
        robots_pure.get().value.heading=Angle(0);
      }
      return robots_pure.get().value;
    }
  }
}

const BallLocation& LocationShortTermMemory::get_ball_location (Time t, bool b) throw () {
  if (b) {
    int index = search_ringbuffer (balls_interacted, wm.get_game_state().cycle_num, t);
    if (index>=0)
      return balls_interacted[index].value;
    else {
      get_interacted (t);
      // check for ball position NaN
     if ((balls_interacted.get().value.pos.x!=balls_interacted.get().value.pos.x) ||
           (balls_interacted.get().value.pos.y!=balls_interacted.get().value.pos.y) ||
           (balls_interacted.get().value.pos.z!=balls_interacted.get().value.pos.z)
         ) 
         balls_interacted.get().value.pos=Vec3D(0,0,0);
      // check for velocity position NaN 
      if ((balls_interacted.get().value.velocity.x!=balls_interacted.get().value.velocity.x) ||
           (balls_interacted.get().value.velocity.y!=balls_interacted.get().value.velocity.y) ||
           (balls_interacted.get().value.velocity.z!=balls_interacted.get().value.velocity.z)
         )
         balls_interacted.get().value.velocity=Vec3D(0,0,0);
      return balls_interacted.get().value;
    }
  } else {
    int index = search_ringbuffer (balls_pure, wm.get_game_state().cycle_num, t);
    if (index>=0)
      return balls_pure[index].value;
    else {
      balls_pure.step(-1);
      balls_pure.get().value = wm.get_ball(t);
      balls_pure.get().cycle = wm.get_game_state().cycle_num;
      balls_pure.get().timestamp = t;
      // check for ball position NaN
      if ((balls_pure.get().value.pos.x!=balls_pure.get().value.pos.x) ||
           (balls_pure.get().value.pos.y!=balls_pure.get().value.pos.y) ||
           (balls_pure.get().value.pos.z!=balls_pure.get().value.pos.z)
         ) 
         balls_pure.get().value.pos=Vec3D(0,0,0);
      // check for velocity position NaN 
      if ((balls_pure.get().value.velocity.x!=balls_pure.get().value.velocity.x) ||
           (balls_pure.get().value.velocity.y!=balls_pure.get().value.velocity.y) ||
           (balls_pure.get().value.velocity.z!=balls_pure.get().value.velocity.z)
         )
         balls_pure.get().value.velocity=Vec3D(0,0,0);
       return balls_pure.get().value;
    }
  }
}

const ObstacleLocation& LocationShortTermMemory::get_obstacle_location (Time t, bool b) throw () {
  if (obstacles.cycle != wm.get_game_state().cycle_num) {
    obstacles.value = wm.get_obstacles (t);
    obstacles.cycle = wm.get_game_state().cycle_num;
    obstacles.timestamp = t;
    obstacle_teammate_assignment ();
  }
  return obstacles.value;
}

void LocationShortTermMemory::get_interacted (Time t) {
  Time tvis = wm.get_timestamp_latest_update ();
  const RobotLocation& robot_vis = get_robot_location (tvis, false);
  const BallLocation& ball_vis = get_ball_location (tvis, false);

  get_obstacle_location (t, false);
  RobotLocation robot_exec = get_robot_location (t, false);
  BallLocation ball_exec = get_ball_location (t,false);

  if (static_cast<unsigned int>(t.diff_msec(tvis))<1000) {
    interaction_manager.get (robot_exec, ball_exec, obstacles.value, robot_vis, ball_vis, static_cast<unsigned int>(t.diff_msec(tvis)));
  }
  robots_interacted.step(-1);
  robots_interacted.get().value = robot_exec;
  robots_interacted.get().cycle = wm.get_game_state().cycle_num;
  robots_interacted.get().timestamp = t;
  balls_interacted.step(-1);
  balls_interacted.get().value = ball_exec;
  balls_interacted.get().cycle = wm.get_game_state().cycle_num;
  balls_interacted.get().timestamp = t;
}

void LocationShortTermMemory::obstacle_teammate_assignment () {
  // Einfache Heuristik zur Zuorndung: "nimm immer das naechstliegende Hindernis"
  // Problem: wenn zwei Roboter sehr nahe kommen, kann ein und das selbe Hindernis
  // beiden Robotern das naechstliegende sein.
  const std::vector<TeammateLocation>& tl (wm.get_teammate_location ());
  vector<int> closest_obstacle_index (tl.size());
  vector<int> snd_closest_obstacle_index (tl.size());
  vector<double> closest_obstacle_dist (tl.size());
  vector<double> snd_closest_obstacle_dist (tl.size());
  for (unsigned int i=0; i<tl.size(); i++) {
    Vec searchpos = tl[i].pos+0.6*tl[i].vtrans*tl[i].timestamp.elapsed_msec();
    int closest_index=-1;
    int snd_closest_index=-1;
    double mindist = 1e+300;
    double snd_mindist = 1e+300;
    for (unsigned int j=0; j<obstacles.value.size(); j++) {
      double dist = (obstacles.value[j].pos-searchpos).length();
      if (dist>1000) continue;  // zu grosser Abstand, nicht mehr beruecksichtigen
      if (dist<mindist) {
        snd_mindist=mindist;
        snd_closest_index=closest_index;
        mindist=dist;
        closest_index=j;
      } else if (dist<snd_mindist) {
        snd_mindist=dist;
        snd_closest_index=j;
      }
    }
    closest_obstacle_index[i]=closest_index;
    snd_closest_obstacle_index[i]=snd_closest_index;
    closest_obstacle_dist[i]=mindist;
    snd_closest_obstacle_dist[i]=snd_mindist;
  }
  // erste Heuristik: jeden Spieler dem naechstliegenden Hindernis zuordnen, bei Konfliken Heuristik 2 aktivieren
  const double bignumber=100000;
  for (unsigned int i=0; i<tl.size(); i++) {
    if (closest_obstacle_index[i]>=0) {  // ueberhaupt einem bekannten Hindernis nahe
      if (obstacles.value[closest_obstacle_index[i]].player<0) {
        // (vorerst) kein Konflikt => einfache Zuweisung
        obstacles.value[closest_obstacle_index[i]].player=i;
      } else {
        // Konflikt mit anderem Spieler, einfache Loesungsheuristik
        unsigned int confl=obstacles.value[closest_obstacle_index[i]].player;
        double own_snd = closest_obstacle_dist[confl];
        if (snd_closest_obstacle_index[i]>=0 && obstacles.value[snd_closest_obstacle_index[i]].player<0)  // lazy evaluation!
          own_snd += snd_closest_obstacle_dist[i];  // hier steckt eine Annahme drin, dass confl durch erste Wahl diesem Hindernis zugeordnet wurde. Dies ist allerdings nicht garantiert.
        else
          own_snd += bignumber;
        double other_snd = closest_obstacle_dist[i];
        if (snd_closest_obstacle_index[confl]>=0 && obstacles.value[snd_closest_obstacle_index[confl]].player<0)  // lazy evaluation!
          other_snd += snd_closest_obstacle_dist[confl]; // dito. Evtl. mal genauer machen. Bleibt aber dennoch eine Heuristik.
        else
          other_snd += bignumber;
        if (own_snd<other_snd) {
          if (snd_closest_obstacle_index[i]>=0)
            obstacles.value[snd_closest_obstacle_index[i]].player=i; // andernfalls: kein zweitnaechstes Hindernis, bleibt ohne Zuordnung
        } else {
          obstacles.value[closest_obstacle_index[i]].player=i;
          if (snd_closest_obstacle_index[confl]>=0) {
            obstacles.value[snd_closest_obstacle_index[confl]].player=confl; // andernfalls: kein zweitnaechstes Hindernis, bleibt ohne Zuordnung
          }
        }
      }
    }
  }
  // Indeces durch Nummern ersetzen
  for (unsigned int i=0; i<obstacles.value.size(); i++) {
    if (obstacles.value[i].player>=0) {
      obstacles.value[i].player=tl[obstacles.value[i].player].number;
    }
  }
}

void LocationShortTermMemory::clear () throw () {
  for (unsigned int i=0; i<robots_pure.size(); i++)
    robots_pure[i].cycle=0;
  for (unsigned int i=0; i<robots_interacted.size(); i++)
    robots_interacted[i].cycle=0;
  for (unsigned int i=0; i<balls_pure.size(); i++)
    balls_pure[i].cycle=0;
  for (unsigned int i=0; i<balls_interacted.size(); i++)
    balls_interacted[i].cycle=0;
  obstacles.cycle=0;
}
