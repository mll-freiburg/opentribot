#include "SimpleCluster.h"
#include <cmath>

using namespace Tribots;

VecCluster::VecCluster() : samples(0), com(0,0), support(0,0), width(0)
{
  ;
}


void VecCluster::update()
{
  support.x = support.y = 0;
  com.x = com.y = 0;
 
  for (unsigned int i=0;i<samples.size(); i++)
    {
      com.x += samples[i].x;
      com.y += samples[i].y;
    }
  com.x/=(double) samples.size();
  com.y/=(double) samples.size();

  double maxdist = 0;
  for (unsigned int i=0;i<samples.size(); i++)
    {
      double dist = fabs((samples[i] - com).length());
      if (dist  > maxdist) maxdist = dist;
    }
  comWidth = 2*maxdist;

  // test for min and max angle of points in local koordinate system
  minRadAng = 2*M_PI;
  maxRadAng = 0;
  for (unsigned int i=0;i<samples.size(); i++)
    {
      double a = samples[i].angle().get_rad();
      if (a < minRadAng) minRadAng = a;
      if (a > maxRadAng) maxRadAng = a;
    }
  // if the cluster is distributed over the 0 axis correct
  bool corrected = false;
  if (minRadAng < 0.5 && maxRadAng > (0.9 *2 * M_PI))
    {
      minRadAng = 2 * M_PI;
      maxRadAng = 0;
      for (unsigned int i=0;i<samples.size(); i++)
	{
	  double a = samples[i].angle().get_rad();
	  if (a < minRadAng && a > M_PI) minRadAng = a;
	  if (a > maxRadAng && a < M_PI) maxRadAng = a;
	}
      corrected = true;
    }
  
  Angle minA(minRadAng);
  Angle maxA(maxRadAng);

  double mindist = 10000;
  for (unsigned int i=0;i<samples.size(); i++)
    {
      double dist = fabs((samples[i] - support).length());
      if (dist  < mindist) mindist = dist;
    }
 
  Vec minV(mindist,0.0);
  Vec maxV(mindist,0.0);
  minV*=minA;
  maxV*=maxA;
  support = Vec(mindist,0)*Angle(0.5*(minRadAng+maxRadAng)) ;
  if (corrected) support*=Angle(M_PI);

  width = (minV - maxV).length();
}

bool VecCluster::add(const Tribots::Vec& v)
{
  samples.push_back(v);
  update();
  return true;
}

double VecCluster::dist(const Tribots::Vec& v) const
{
  //double dist = fabs((v-com).length())-(comWidth/2.0);
  double dist = 10000;
  for (unsigned int i=0; i<samples.size(); i++)
    {
      if (((v-samples[i]).length()) < dist) dist = ((v-samples[i]).length());
      //std::cerr << "Dist of sample " << i << " : " << samples[i] << " with " << v << "\n";
    }
  
  return (dist > 0 ? dist : 0);
}


double VecCluster::dist(const VecCluster& vc)
{
  //double dist = (support - vc.support).length() - (width/2.0) - (vc.width/2.0);
  double dist = 10000;
  for (unsigned int i=0; i<samples.size(); i++)
    {
      if (vc.dist(samples[i]) < dist) dist = vc.dist(samples[i]);
    }
  return dist < 0 ? 0 : dist;
}


VecCluster VecCluster::glue(const VecCluster &other)
{
  VecCluster new_vc;
  for (unsigned int i=0; i< samples.size(); i++) new_vc.samples.push_back(samples[i]);
  for (unsigned int i=0; i< other.samples.size(); i++) new_vc.samples.push_back(other.samples[i]);
  new_vc.update();
  return new_vc;
}

SimpleVecCluster::SimpleVecCluster(double _thresh)
{
  thresh = _thresh;
}

void SimpleVecCluster::add(const Tribots::Vec& v)
{
  bool newClust = true;
  for (unsigned int i=0; i<cluster.size(); i++)
    {
      if (cluster[i].dist(v) < thresh) {
	cluster[i].add(v);
	newClust = false;
	break;
      }
    }
  if (newClust) {
    VecCluster vc;
    vc.add(v);
    cluster.push_back(vc);
  }
}


void SimpleVecCluster::prune()
{
  bool changed = false;
  do {
    std::vector<VecCluster> nvc;
    changed = false;
    std::vector<VecCluster>::iterator s;
    std::vector<VecCluster>::iterator p;
    for (s=cluster.begin(); s!= cluster.end(); s++)
      {
	for (p=s+1; p!= cluster.end(); p++)
	  {
	    if (s->dist(*(p)) < thresh ) 
	      {
		//VecCluster nc = s->glue(*(p));
		//cluster.erase(p);
		//cluster.erase(s);
		//cluster.push_back(nc);
		changed = true;
		break;
	      }
	  }
	if (changed) break;
	else nvc.push_back(*p);
      }
    if (changed) {
      for(std::vector<VecCluster>::iterator i=p+1; i!=cluster.end(); i++)
	if (i!=s) nvc.push_back(*i);
      nvc.push_back(s->glue(*(p)));
    }
    cluster = nvc;
  } while (changed);
}

