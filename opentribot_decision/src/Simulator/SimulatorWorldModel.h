
#ifndef _Tribots_SimulatorWorldModel_h_
#define _Tribots_SimulatorWorldModel_h_

#include "../WorldModel/Types/WorldModelTypeBase.h"
#include "../WorldModel/Ball/DynamicSlidingWindowBallFilter3D.h"
#include "../WorldModel/Obstacles/EMAObstacleFilter.h"
#include "../WorldModel/Orga/OdometryContainer.h"
#include "SimClient.h"

namespace Tribots {

  /** Klasse implementiert die Weltmodell-Seite der Simulatoranbindung */
  class SimulatorWorldModel : public WorldModelTypeBase {
  private:
    SimClient* the_sim_client;         ///< pointer auf den SimClient
    DynamicSlidingWindowBallFilter3D ball_filter;
    EMAObstacleFilter obstacle_filter;
    RobotLocation cpos;
    Time cpos_time;

    double ball_vision_radius;
    double obstacle_vision_radius;

  public:
    SimulatorWorldModel (const ConfigReader&) throw (std::bad_alloc, Tribots::InvalidConfigurationException);
    ~SimulatorWorldModel () throw ();

    unsigned int update_localisation () throw ();
    RobotLocation get_robot (Time) const throw ();
    BallLocation get_ball (Time) const throw ();
    ObstacleLocation get_obstacles (Time) const throw ();
    Time get_timestamp_latest_update () const throw ();
    void init_cycle (Time, Time) throw ();

    // folgende Funktionen tun nichts:
    void reset () throw ();
    void reset (const Vec) throw ();
  };

}

#endif

