#ifndef __SIMPLE_VEC_CLUSTER_H__
#define __SIMPLE_VEC_CLUSTER_H__

#include "../../Fundamental/Vec.h"
#include <vector>

namespace Tribots {

class VecCluster {
 public:
  std::vector<Vec> samples;
  Vec    com;
  double comWidth;
  Vec    support;
  double minRadAng, maxRadAng;

  double width;

  void update();

  VecCluster();

  bool add(const Tribots::Vec& v);
  double dist(const Tribots::Vec& v) const;
  double dist(const Tribots::VecCluster& vc);
  VecCluster glue(const Tribots::VecCluster &other);
  
  void clear() {samples.clear(); support.x=0; support.y=0; width=0;};
};


class SimpleVecCluster {
 public:
  double thresh;
  std::vector<VecCluster> cluster;

  SimpleVecCluster(double _thresh);
  void add(const Tribots::Vec& v);
  void prune();
  
};
}

#endif
