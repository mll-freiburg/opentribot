
#include "MultiCameraBallFilter.h"
//#include "DynamicSlidingWindowBallFilter.h"
#include "DynamicSlidingWindowBallFilter3D.h"
//#include "OdometrySlidingWindowBallFilter.h"
#include "../WorldModel.h"

using namespace Tribots;
using namespace std;

namespace {

  BallFilter* create_ball_filter (const ConfigReader& cfg, const OdometryContainer& odobox) {
//    std::string temp;
//    cfg.get ("BallFilter::filter_type", temp);
//    if (temp=="odometry")
//      return new OdometrySlidingWindowBallFilter (cfg, odobox);
//    else if (temp=="world3d")
      return new DynamicSlidingWindowBallFilter3D (cfg);
//    else
//      return new DynamicSlidingWindowBallFilter (cfg);
  }

}

MultiCameraBallFilter::MultiCameraBallFilter (const ConfigReader& reader, const OdometryContainer& odo) throw (std::bad_alloc) : common_mode (false), cfg(reader), odobox (odo) {
  cfg.get ("BallFilter::imagesource", imagesources);
  if (imagesources.size()==0)
    imagesources.push_back (0);
  cfg.get ("BallFilter::common_mode", common_mode);
  common_ball_filter = create_ball_filter (cfg, odobox);
}

MultiCameraBallFilter::~MultiCameraBallFilter () throw () {
  delete common_ball_filter;
  for (unsigned int i=0; i<individual_ball_filter.size(); i++)
    delete individual_ball_filter[i];
}

bool MultiCameraBallFilter::update (const std::vector<VisibleObjectList>& vis, const std::vector<RobotLocation>& pos) throw () {
  bool success = false;
  if (common_mode) {
    vector<unsigned int> copy_imagesources = imagesources;
    unsigned int ii=0;
    while (ii<copy_imagesources.size())
      if (copy_imagesources[ii]>=vis.size())
        copy_imagesources.erase (copy_imagesources.begin()+ii);
    else
      ii++;
    while (copy_imagesources.size()>0) {
      unsigned int oldest_source=0;
      for (unsigned int i=1; i<copy_imagesources.size(); i++)
        if (vis[copy_imagesources[i]].timestamp<vis[copy_imagesources[oldest_source]].timestamp)
          oldest_source=i;
      success |= common_ball_filter->update (vis[copy_imagesources[oldest_source]], pos[copy_imagesources[oldest_source]]);
      copy_imagesources.erase (copy_imagesources.begin()+oldest_source);
    }
  } else {
    while (individual_ball_filter.size()<vis.size()) {
      individual_ball_filter.push_back (create_ball_filter (cfg, odobox));
    }
    for (unsigned int i=0; i<vis.size(); i++)
      success |= individual_ball_filter [i]->update (vis[i], pos[i]);
  }
  return success;
}

void MultiCameraBallFilter::comm_ball (Vec p) throw () {
  comm_ball_pos = MWM.get_own_half()*p;
  comm_ball_pos_time.update();
}

BallLocation MultiCameraBallFilter::get (const Time t) const throw () {
  BallLocation result;
  if (common_mode)
    result=common_ball_filter->get (t);
  else {
    bool found=false;
    BallLocation best;
    unsigned int index=0;
    Time lastSeen;
    lastSeen.set_msec(0);
    while (index<individual_ball_filter.size()) {
      BallLocation bloc = individual_ball_filter[index]->get(t);
      if (bloc.pos_known==BallLocation::known || bloc.pos_known==BallLocation::raised) {
        if (bloc.lastly_seen>lastSeen) {
          best = bloc;
          lastSeen=bloc.lastly_seen;
          lastSeen.add_msec(34);  // den Kameras mit niedrigem Index einen Vorsprung einraeumen
          found=true;
          break;
        }
      }
      index++;
    }
    if (found)
      return best;
    if (individual_ball_filter.size()>0)
      result=individual_ball_filter[0]->get(t);
    else
      result=common_ball_filter->get(t);
  }
  if (result.pos_known==BallLocation::unknown)
    if (t.diff_msec (comm_ball_pos_time)<1000) {
    result.pos=comm_ball_pos;
    result.velocity=Vec::zero_vector;
    result.pos_known=BallLocation::communicated;
  }
  return result;
}
