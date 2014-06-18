
#ifndef _ODE_SIM_PACK_WORLD_MODEL_H_
#define _ODE_SIM_PACK_WORLD_MODEL_H_

#include "../../WorldModel/Types/WorldModelTypeBase.h"
//#include "../../WorldModel/Ball/DynamicSlidingWindowBallFilter.h"
#include "../../WorldModel/Ball/DynamicSlidingWindowBallFilter3D.h"
#include "../../WorldModel/Obstacles/ObstacleContainer.h"
#include "../../WorldModel/Orga/OdometryContainer.h"
#include "OdeSimPackClient.h"

namespace Tribots {

  /** Klasse implementiert die Weltmodell-Seite der Simulatoranbindung */
  class OdeSimPackWorldModel : public WorldModelTypeBase {
  private:
    OdeSimPackClient* client;         ///< pointer auf den SimClient

    DynamicSlidingWindowBallFilter3D ball_filter;
    ObstacleContainer              obstacle_filter;
    RobotLocation                  cpos;
    Time                           cpos_time;

  public:
    OdeSimPackWorldModel (const ConfigReader&) throw (std::bad_alloc, Tribots::InvalidConfigurationException);
    ~OdeSimPackWorldModel () throw ();

    unsigned int update_localisation () throw ();
    RobotLocation get_robot (Time) const throw ();
    BallLocation get_ball (Time) const throw ();
    ObstacleLocation get_obstacles (Time) const throw ();
    Time get_timestamp_latest_update () const throw ();

    // folgende Funktionen tun nichts:
    void reset () throw ();
    void reset (const Vec) throw ();
    void init_cycle (Time, Time) throw ();

  };

}

#endif

