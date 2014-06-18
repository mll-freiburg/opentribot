
#include "CondensationFilter.h"
#include "../../Fundamental/random.h"
#include "../../Structures/Journal.h"
#include "../WorldModel.h"    // zum loggen/debuggen
#include <cmath>
#include <algorithm>

using namespace Tribots;
using namespace std;

#define DEBUG_CFILTER 0

Particle::Particle () throw () {;}

Particle::Particle (const Particle& p) throw () : pos(p.pos), heading(p.heading), probability(p.probability) {;}

const Particle& Particle::operator= (const Particle& p) throw () {
  pos = p.pos;
  heading = p.heading;
  probability = p.probability;
  return (*this);
}



CondensationFilter::CondensationFilter (const ConfigReader& reader, const OdometryContainer& ob, const FieldGeometry& fg) throw (std::bad_alloc, InvalidConfigurationException) : odobox(ob), field_geometry (fg), fieldlut (fg, 100), trimrate (0), ignore_goals(false), plog(NULL), max_lines(1000) {
  // Parameter auslesen und Variablen setzen
  vector<unsigned int> uival (4);
  if (reader.get ("CondensationFilter::number_particles", uival)>=1) {
    min_set_size = uival[0];
    if (uival.size()>=2)
      max_set_size = uival[1];
    else
      max_set_size = min_set_size;
    unsigned int set_size = max_set_size;
    if (uival.size()>=3)
      set_size = uival[2];
    num_new_particles=0;
    if (uival.size()>=4)
      num_new_particles = uival[3];    

    particle_set = new vector<Particle> (max_set_size);
    reserve_set = new vector<Particle> (max_set_size);
    select_numbers = new vector<double> (max_set_size);

    create_random_particleset(set_size);
  } else
    throw InvalidConfigurationException ("CondensationFilter::number_particles");

  vector<double> dval (3);
  if (reader.get ("CondensationFilter::spread_factors", dval)>=2) {
    spread_dev = abs(dval[0]);
    spread_rot_dev = abs(dval[1]*M_PI/180);
  } else 
    throw InvalidConfigurationException ("CondensationFilter::spread_factors");

  if (reader.get ("CondensationFilter::sensor_probabilities", dval)>=2) {
    probability_line_sdev = abs(dval[0]);
    probability_line_min = abs(dval[1]);
  } else
    throw InvalidConfigurationException ("CondensationFilter::sensor_probabilities");
  double dd;
  if (reader.get ("CondensationFilter::trimrate", dd)>0)
    trimrate = ( dd>1 ? 1.0 : dd );

  bool b;
  if (reader.get ("CondensationFilter::consider_goals", b)>0)
    ignore_goals = !b;
  reader.get ("CondensationFilter::max_lines", max_lines);

  max_cycle_max_particle=0;
  reader.get ("CondensationFilter::auto_reinit_waiting_time", max_cycle_max_particle);
#if DEBUG_CFILTER
  std::string fname;
  if (!reader.get ("write_world_model_info", fname)) 
    fname = "wminfo";
  fname+=".ppos";
  plog = new std::ofstream (fname.c_str());
  if (!plog) {
    JERROR("could not open particle log file");
    delete plog;
    plog=NULL;
  }
#endif
}

CondensationFilter::~CondensationFilter () throw () {
  delete particle_set;
  delete reserve_set;
  delete select_numbers;
  if (plog)
    delete plog;
}

void CondensationFilter::create_random_particleset (unsigned int n) {
  double max_x = 0.5*field_geometry.field_width+field_geometry.side_band_width;
  double max_y = 0.5*field_geometry.field_length+field_geometry.goal_band_width;
  particle_set->resize (n);
  double eins_n = 1.0/static_cast<double>(n);
  vector<Particle>::iterator pit = particle_set->begin();
  vector<Particle>::iterator pit_end = particle_set->end();
  while (pit<pit_end) {
    pit->pos.x = urandom (-max_x, max_x);
    pit->pos.y = urandom (-max_y, max_y);
    pit->heading = Angle (urandom (0, 6.2831853071));
    pit->probability = eins_n;
    pit++;
  }
}

void CondensationFilter::create_local_particleset (const Vec pos, unsigned int n) {
  double sdev = 500;
  particle_set->resize(n);
  double eins_n = 1.0/static_cast<double>(n);
  vector<Particle>::iterator pit = particle_set->begin();
  vector<Particle>::iterator pit_end = particle_set->end();
  while (pit<pit_end) {
    pit->pos.x = pos.x+nrandom (0,sdev);
    pit->pos.y = pos.y+nrandom (0,sdev);
    pit->heading = Angle (urandom (0, 6.2831853071));
    pit->probability = eins_n;
    pit++;
  }
}

void CondensationFilter::create_local_particleset (const Vec pos, const Angle h, unsigned int n) {
  double sdev = 500;
  particle_set->resize(n);
  double eins_n = 1.0/static_cast<double>(n);
  vector<Particle>::iterator pit = particle_set->begin();
  vector<Particle>::iterator pit_end = particle_set->end();
  while (pit<pit_end) {
    pit->pos.x = pos.x+nrandom (0,sdev);
    pit->pos.y = pos.y+nrandom (0,sdev);
    pit->heading = h+Angle::deg_angle (urandom (-20, 20));
    pit->probability = eins_n;
    pit++;
  }
}

double CondensationFilter::sum_particle_probabilities () const {
  double sum=0;
  vector<Particle>::iterator pit = particle_set->begin();
  vector<Particle>::iterator pit_end = particle_set->end();
  while (pit<pit_end) {
    sum += pit->probability;
    pit++;
  }
  return sum;
}

void CondensationFilter::resample_particleset (unsigned int n) {
  vector<Particle>* new_particle_set = reserve_set;
  new_particle_set->resize (n);
  double sum_prob = sum_particle_probabilities ();

  // erzeugen von n U(0,sum_prob)-verteilten, aufsteigend sortiereten Zufallszahlen
  select_numbers->resize(n);
  double factor = 1.0;
  vector<double>::iterator sit = select_numbers->end()-1;
  for (int i=n; i>0; i--) {
    factor = factor*std::pow(urandom(),1.0/static_cast<double>(i));
    *(sit--) = factor*sum_prob;
  }
  
  // Particle auswaehlen gemaess select_numbers
  double eins_n =1.0/static_cast<double>(n);
  sit = select_numbers->begin();
  vector<double>::iterator sit_end = select_numbers->end();
  vector<Particle>::iterator pit = particle_set->begin();
  vector<Particle>::iterator pit_end = particle_set->end();
  vector<Particle>::iterator npit = new_particle_set->begin();
  double sum_to=0.0;
  while ((sit<sit_end) && (pit<pit_end)) {
    sum_to += pit -> probability;
    int i=0;
    while ((sit<sit_end) && ((*sit) < sum_to)) {
      i++;
      sit++;
    }
    while (i>0) {
      *npit = *pit;
      (npit++) -> probability = eins_n;  // gleich die Wahrscheinlichkeiten zuruecksetzen
      i--;
    }
    pit++;
  }

  reserve_set = particle_set;
  particle_set = new_particle_set;
}

void CondensationFilter::move_and_spread_particleset (const Vec trans, const Angle rot, double sdev_p, double sdev_h, unsigned int num_new, const Angle& prior_heading) {
  vector<Particle>::iterator pit = particle_set->begin();
  vector<Particle>::iterator pit_end = particle_set->end()-num_new;
  double dev = (1+trans.length()/100)*sdev_p;
  while (pit<pit_end) {
    pit->pos += trans.rotate (pit->heading)+dev*n2random ();
    pit->heading += rot+Angle(sdev_h*nrandom());
    pit++;
  }
  pit_end = particle_set->end();
  while (pit<pit_end) {
    // noch ein paar zufaellige Partikel hinzufuegen
    pit->pos.x=urandom (-0.5*field_geometry.field_width-field_geometry.side_band_width, 0.5*field_geometry.field_width+field_geometry.side_band_width);
    pit->pos.y=urandom (-0.5*field_geometry.field_length-field_geometry.goal_band_width, 0.5*field_geometry.field_length+field_geometry.goal_band_width);
    pit->heading = prior_heading+Angle::rad_angle (urandom (-1.5,1.5));  // eine Richtung, die der bisherigen Orientierung entspricht
    pit++;
  }
}

Particle CondensationFilter::average_particle (double& det, double& len, unsigned int n2) {
#if 1
  Vec sum_pos (0,0);
  Vec sum_unit (0,0);
  double sum_sxx = 0;
  double sum_syy = 0;
  double sum_sxy = 0;
  double n = 0;
  vector<Particle>::iterator pit = particle_set->begin();
  vector<Particle>::iterator pit_end = particle_set->end()-n2;
  while (pit<pit_end) {
    n += pit->probability;
    sum_pos += pit->probability*pit->pos;
    sum_unit += pit->probability*Vec::unit_vector (pit->heading);
    sum_sxx += pit->probability*pit->pos.x*pit->pos.x;
    sum_syy += pit->probability*pit->pos.y*pit->pos.y;
    sum_sxy += pit->probability*pit->pos.x*pit->pos.y;
    pit++;
  }
  Particle res;
  res.pos = (1.0/n)*sum_pos;
  res.heading = sum_unit.angle();
  len = sum_unit.length()/n;
  double cxx = (sum_sxx-sum_pos.x*sum_pos.x/n);
  double cyy = (sum_syy-sum_pos.y*sum_pos.y/n);
  double cxy = (sum_sxy-sum_pos.x*sum_pos.y/n);
  det = (cxx*cyy-cxy*cxy)/(n*n);
  return res;
#else
  vector<Particle>::iterator pit = particle_set->begin();
  vector<Particle>::iterator pit_end = particle_set->end()-n2;
  Particle res;
  res.probability=0;
  while (pit<pit_end) {
    if (pit->probability>=res.probability)
      res=*pit;
    pit++;
  }
  return res;
#endif
}

namespace {

  // Torausrichtung pruefen; world ist die Position des Tores in der Welt, sensor die gesehende Position in Roboterkoordinaten
  void check_goal_direction (vector<Particle>& particle_set, Vec world, Vec sensor) {
    vector<Particle>::iterator pit = particle_set.begin();
    vector<Particle>::iterator pit_end = particle_set.end();
    Angle sensor_direction = sensor.angle();
    while (pit<pit_end) {
      if ((world-pit->pos).squared_length()>=2.25e+06) {  // Berechnung nur bei Partikeln durchfuehren, die weit genug (1.5m) vom Tor entfernt sind
        Angle particle_angle = (world-pit->pos).angle()-pit->heading;  // particle_angle und sensor_direction sollten uebereinstimmen
        if ((particle_angle-sensor_direction).in_between (Angle::quarter, Angle::three_quarters)) {
          // Partikel spiegeln
          pit->pos *=1;
          pit->heading+=Angle::half;
#if DEBUG_CFILTER
          WorldModel::get_main_world_model().log_stream() << "CFilter: Partikel spiegeln\n";
#endif
        }
      }
      pit++;
    }
  }

}

void CondensationFilter::evaluate_lines (const VisibleObjectList& lines) {
  if (trimrate<=0) {
    vector<VisibleObject>::const_iterator vit = lines.objectlist.begin();
    vector<VisibleObject>::const_iterator vit_end = lines.objectlist.end();
    if ((vit_end-vit)>static_cast<int>(max_lines))
      vit_end=vit+max_lines;
    vector<Particle>::iterator pit;
    vector<Particle>::iterator pit_end = particle_set->end();
    while (vit<vit_end) {
      pit = particle_set->begin();
      while (pit < pit_end) {
        double d = fieldlut.distance (pit->pos+vit->pos.rotate (pit->heading));
        double p = (d>=probability_line_sdev ? probability_line_min : 1.0-d/probability_line_sdev+probability_line_min);
        (pit++)->probability *= p;
      }
      vit++;
    }
  } else { // trimrate >0
    vector<VisibleObject>::const_iterator vit;
    vector<VisibleObject>::const_iterator vit_end = lines.objectlist.end();
    if ((vit_end-vit)>static_cast<int>(max_lines))
      vit_end=vit+max_lines;
    vector<Particle>::iterator pit;
    vector<Particle>::iterator pit_end = particle_set->end();
    vector<double> probs (lines.objectlist.size());
    pit = particle_set->begin();
    while (pit<pit_end) {
      probs.clear();
      vit=lines.objectlist.begin();
      while (vit<vit_end) {
        double d = fieldlut.distance (pit->pos+vit->pos.rotate (pit->heading));
        double p = (d>=probability_line_sdev ? probability_line_min : 1.0-d/probability_line_sdev+probability_line_min);
        probs.push_back (p);
        vit++;
      }
      sort (probs.begin(), probs.end());  // hier wird mehr Aufwand spendiert als wirklich notwendig
      vector<double>::const_iterator evalit = probs.begin()+=static_cast<int>(floor(probs.size()*trimrate));
      vector<double>::const_iterator evalit_end = probs.end();
      while (evalit<evalit_end)
        pit->probability*=(*(evalit++));
      pit++;
    }
  }
}

void CondensationFilter::evaluate_goals (const VisibleObjectList& goals) {
  vector<VisibleObject>::const_iterator vit = goals.objectlist.begin();
  vector<VisibleObject>::const_iterator vit_end = goals.objectlist.end();
  unsigned blue_goal_found = 0;
  unsigned yellow_goal_found = 0;
  Vec blue_goal (0,0);
  Vec yellow_goal (0,0);
    
  while (vit<vit_end) {
    if (vit->object_type==VisibleObject::blue_goal || vit->object_type==VisibleObject::blue_goal_post_left || vit->object_type==VisibleObject::blue_goal_post_right) {
      blue_goal_found++;
      blue_goal += vit->pos;
    } else if (vit->object_type==VisibleObject::yellow_goal || vit->object_type==VisibleObject::yellow_goal_post_left || vit->object_type==VisibleObject::yellow_goal_post_right ) {
      yellow_goal_found++;
      yellow_goal += vit->pos;
    }
    if (!ignore_goals) {
      if (blue_goal_found>0)
        check_goal_direction (*particle_set, Vec(0, 0.5*field_geometry.field_length+field_geometry.goal_length), (1.0/static_cast<double>(blue_goal_found))*blue_goal);
      if (yellow_goal_found>0)
        check_goal_direction (*particle_set, Vec(0, -0.5*field_geometry.field_length-field_geometry.goal_length), (1.0/static_cast<double>(yellow_goal_found))*yellow_goal);
    }
    vit++;
  }
}

unsigned int CondensationFilter::adapt_particle_number (double det, double len) {
  unsigned int ret=particle_set->size();
  if (len>0.6 && sqrt(det)<2*spread_dev*spread_dev) {
    ret = static_cast<unsigned int>(0.9*ret);
    if (ret<min_set_size)
      ret=min_set_size;
  }
  if (len<0.6 || sqrt(det)>4*spread_dev*spread_dev) {
    ret = static_cast<unsigned int>(1.2*ret);
    if (ret>max_set_size)
      ret=max_set_size;
  }
  return ret;
}


void CondensationFilter::reset () throw () {
  create_random_particleset (max_set_size);
}

void CondensationFilter::reset (Vec p) throw () {
  create_local_particleset (p, max_set_size);
}

void CondensationFilter::reset (Vec p, Angle h) throw () {
  create_local_particleset (p, h, max_set_size);
}

bool CondensationFilter::update (const VisibleObjectList& lines, const VisibleObjectList&, const VisibleObjectList& goals) throw () {
  RobotLocation egomotion = odobox.movement (latest_update, lines.timestamp);
  latest_update=lines.timestamp;
  move_and_spread_particleset (egomotion.pos, egomotion.heading, spread_dev, spread_rot_dev, num_new_particles, latest_average_pos.heading);
  evaluate_lines (lines);
  evaluate_goals (goals);
  double det, len;
  latest_average_pos = average_particle (det, len, num_new_particles);
  unsigned int set_size =  adapt_particle_number (det, len);
  if (set_size==max_set_size && min_set_size!=max_set_size) {
    max_cycle_counter++;
    if (max_cycle_counter>=max_cycle_max_particle && max_cycle_max_particle>0) {
      max_cycle_counter=0;
      create_random_particleset(set_size);
      JWARNING ("CondensationFilter: AutoReInit");
    }
  } else
    max_cycle_counter=0;

#if DEBUG_CFILTER
  // Partikel rausschreiben:
  if (plog) {
    Time now;
    vector<Particle>::const_iterator pit = particle_set->begin();
    vector<Particle>::const_iterator pit_end = particle_set->end();
    double dir = WorldModel::get_main_world_model().get_own_half();
    while (pit<pit_end) {
      (*plog) << now << '\t' << dir*pit->pos.x << '\t' << dir*pit->pos.y << '\t' << (pit->heading+ (dir>0 ? Angle::zero : Angle::half)).get_rad() << '\t' << pit->probability << '\n';
      pit++;
    }
  }
#endif

  resample_particleset (set_size);

#if DEBUG_CFILTER
  WorldModel::get_main_world_model().log_stream() << "CFilter: delta_s= " << egomotion.pos.x << ", " << egomotion.pos.y << ", " << (egomotion.heading.get_deg()>180 ? 360-egomotion.heading.get_deg() : egomotion.heading.get_deg()) << '\n';
  WorldModel::get_main_world_model().log_stream() << "CFilter: num_particle= " << particle_set->size() << '\n';
  WorldModel::get_main_world_model().log_stream() << "CFilter: quality= " << len << ", " << det << '\n';
#endif
  return true;
}

RobotLocation CondensationFilter::get (Time t) const throw() {
  RobotLocation pnow;
  pnow.pos = latest_average_pos.pos;
  pnow.heading = latest_average_pos.heading;
  pnow.quality = 1;
  return odobox.add_movement (pnow, latest_update, t);
}

