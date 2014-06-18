
#include "TemporalDifferenceSL.h"
#include "../WorldModel.h"
#include "../Prediction/update_robot_location.h"
#include "../../Structures/Journal.h"
#include "../../Fundamental/random.h"
#include <cmath>
#include <sstream>
#include <algorithm>

using namespace std;
using namespace Tribots;

#define DEBUG_INTERNAL_SL 0   // Debugen der Hauptpositionshypothesen
#define DEBUG_EXTERNAL_SL 0   // Debuggen der alternativen Hypothesen

namespace {
  double maxx = 10;  // Hilfsgroesse: halbe Spielfeldbreite+Rand, wird in TemporalDifferenceSL-Konstruktor gesetzt
  double maxy = 10;  // Hilfsgroesse: halbe Spielfelblaenge+Rand, dito
  double gsx = 10; // Hilfsgroesse: halbe Torbreite, dito
  double gsy = 10; // Hilfsgroesse: halbe Spielfeldlaenge, dito

  VisibleObjectList selectedlines1;  // um den Speicher nicht immer wieder neu allokieren zu muessen
  VisibleObjectList selectedlines2;  // dito
  vector<double> obstaclerays;  // dito
  vector<bool> alternative_erase;  // dito

  inline void chosemin (double& a1, const double& a2) {
    a1 = (a1<a2 ? a1 : a2);
  }

  inline double hoch6 (const double& x) {
    double y=x*x*x;
    return y*y;
  }

  const double max_way_alive = 40000;   // maximal zu beruecksichtigender Weg
  const double ema_quality = 0.2;   // EMA-Parameter

  string format8 (double x) {
    string res;
    stringstream inout;
    inout << x << '\n';
    getline (inout, res);
    while (res.length()<8)
      res+=" ";
    return res;
  }

}

TemporalDifferenceSL::AlternativePose::AlternativePose () : latestSLMirrorHintTimes (50) {
  init_randomly ();
  for (unsigned int i=0; i<latestSLMirrorHintTimes.size(); i++)
    latestSLMirrorHintTimes[i].add_sec (-100);
  magic_number=1e300;
}

void TemporalDifferenceSL::AlternativePose::init_randomly () {
  kfilter.set (Vec(urandom(-maxx,maxx),urandom(-maxy,maxy)), Angle::rad_angle (urandom(0,2*M_PI)), Vec(1e10, 1e10), 400);
  num_iterations_alive=0;
  num_iterations_visual_input=0;
  way_alive=0;
  reset_hypothesis=false;
  visual_error=1e300;
  odo_vis_mismatch_linear=1e300;
  odo_vis_mismatch_angular=1e300;
  heading_checked=false;
  rotation_since_heading_check=0;
  ema_right_goal=0.51;
  ema_wrong_goal=0.5;
  magic_number=1e300;
}

void TemporalDifferenceSL::AlternativePose::init_position (Vec p) {
  kfilter.set (p, Angle::rad_angle (urandom(0,2*M_PI)), Vec(500, 500), 400);
  num_iterations_alive=0;
  num_iterations_visual_input=0;
  way_alive=0;
  reset_hypothesis=false;
  visual_error=1e300;
  odo_vis_mismatch_linear=1e300;
  odo_vis_mismatch_angular=1e300;
  heading_checked=false;
  rotation_since_heading_check=0;
  ema_right_goal=0.51;
  ema_wrong_goal=0.5;
  magic_number=1e300;
}

void TemporalDifferenceSL::AlternativePose::init_pose (Vec p, Angle h) {
  kfilter.set (p, h, Vec(500, 500), 0.1);
  num_iterations_alive=0;
  num_iterations_visual_input=0;
  way_alive=0;
  reset_hypothesis=false;
  visual_error=1e300;
  odo_vis_mismatch_linear=1e300;
  odo_vis_mismatch_angular=1e300;
  heading_checked=false;
  rotation_since_heading_check=0;
  ema_right_goal=0.51;
  ema_wrong_goal=0.5;
  magic_number=1e300;
}



TemporalDifferenceSL::TemporalDifferenceSL (const ConfigReader& reader, const OdometryContainer& odo, const FieldGeometry& fg1) throw (std::bad_alloc) : fg (fg1), odobox (odo) , alternative_hypothesis (1), average_vision_delay (40) {
  maxx = 0.5*fg.field_width+fg.side_band_width;
  maxy = 0.5*fg.field_length+fg.goal_band_width;
  gsx = 0.5*fg.goal_width;
  gsy = 0.5*fg.field_length;

  vector<double> an_array;
  double err_width;
  if (!reader.get ("TemporalDifferenceSL::error_width", err_width))
    err_width=300;
  double dist_param;
  if (!reader.get ("TemporalDifferenceSL::distance_weight_parameter", dist_param))
    dist_param=2500;   // default: praktisch gleiche Gewichtung unabhaengig von Entfernung
  consider_yellow_goal=consider_blue_goal=false;
  reader.get ("TemporalDifferenceSL::consider_yellow_goal", consider_yellow_goal);
  reader.get ("TemporalDifferenceSL::consider_blue_goal", consider_blue_goal);
  num_internal_alternatives=4;
  reader.get ("TemporalDifferenceSL::number_internal_alternatives", num_internal_alternatives);
  if (num_internal_alternatives>8)
    num_internal_alternatives=8;
  counter_internal_alternatives=0;
  internal_random_direction=true;
  reader.get ("TemporalDifferenceSL::internal_random_direction", internal_random_direction);
  num_alternatives = 1;
  reader.get ("TemporalDifferenceSL::number_external_alternatives", num_alternatives);
  num_alternative_updates = num_alternatives;
  reader.get ("TemporalDifferenceSL::number_alternative_updates", num_alternative_updates);
  max_lines = 100;
  reader.get ("TemporalDifferenceSL::max_lines", max_lines);
  do_remove_occluded_lines = false;
  do_remove_occluded_lines_far = true;
  string dmy;
  reader.get ("TemporalDifferenceSL::remove_occluded_lines", dmy);
  if (dmy=="none" || dmy=="false" || dmy=="0")
    do_remove_occluded_lines=do_remove_occluded_lines_far=false;
  else if (dmy=="far") {
    do_remove_occluded_lines=false;
    do_remove_occluded_lines_far=true;
  } else if (dmy=="all" || dmy=="true" || dmy=="1") {
    do_remove_occluded_lines=true;
    do_remove_occluded_lines_far=false;
  }
  do_remove_lines_outside_field = true;
  reader.get ("TemporalDifferenceSL::remove_lines_outside_field", do_remove_lines_outside_field);
  max_line_distance = 6000;
  reader.get ("TemporalDifferenceSL::remove_lines_from_distance", max_line_distance);
  has_gyro=false;
  reader.get ("TemporalDifferenceSL::gyroscope", has_gyro);
  main_to_reset_hysteresis_lower=0;
  main_to_reset_hysteresis_upper=1000000;
  reader.get ("TemporalDifferenceSL::reset_hysteresis", an_array);
  if (an_array.size()>=1)
    main_to_reset_hysteresis_lower = an_array[0];
  if (an_array.size()>=2)
    main_to_reset_hysteresis_upper = an_array[1];
  prefered_enter=0;
  string esd;
  reader.get ("TemporalDifferenceSL::enter_side", esd);
  if (esd=="Right" || esd=="right" || esd=="+1" || esd=="1" || esd=="+")
    prefered_enter=+1;
  else if (esd=="Left" || esd=="left" || esd=="-1" || esd=="-")
    prefered_enter=-1;

  alternative_hypothesis.resize(num_alternatives);
  field_lut = new FieldLUT (fg, 50);
  vis_optimiser = new VisualPositionOptimiser (*field_lut, err_width, dist_param);

  next_alternative=0;

  reset ();
}

TemporalDifferenceSL::~TemporalDifferenceSL () throw () {
  delete field_lut;
  delete vis_optimiser;
}



void TemporalDifferenceSL::remove_occluded_lines (VisibleObjectList& select, const VisibleObjectList& all, const VisibleObjectList& obs, double maxdist, double minod) {
  select.objectlist.clear();
  select.timestamp = all.timestamp;

  // obstaclerray aufbauen
  obstaclerays.assign (360, maxdist*maxdist);

  vector<VisibleObject>::const_iterator obsit = obs.objectlist.begin();
  vector<VisibleObject>::const_iterator obsitend = obs.objectlist.end();
  while (obsit!=obsitend) {
    double d2 = (obsit->pos).squared_length()+1e-10;
    double d1= sqrt(d2);
    if (d1>=minod) {
      Vec ortho = ((1.0/d1)*(obsit->pos)).rotate_quarter();
      Vec p1 = (obsit->pos)-0.5*obsit->width*ortho;
      Vec p2 = (obsit->pos)+0.5*obsit->width*ortho;
      int index1 = static_cast<int>(floor(p1.angle().get_deg()));
      int index2 = static_cast<int>(ceil(p2.angle().get_deg()))+1;
      int index_m1 = index2+1;
      int index_m2 = index2+1;
      if (index1>index2) {
        index_m1 = 360;
        index_m2 = 0;
      }
      for (int i=index1; i<index_m1; i++)
        chosemin (obstaclerays [i], d2);  // nehme an, die Hindernisse sind nicht sehr breit
      for (int i=index_m2; i<index2; i++)
        chosemin (obstaclerays [i], d2);
    }
    obsit++;
  }

  vector<VisibleObject>::const_iterator linesit = all.objectlist.begin();
  vector<VisibleObject>::const_iterator linesitend = all.objectlist.end();
  while (linesit!=linesitend) {
    int index = static_cast<int>((linesit->pos).angle().get_deg()+0.5);
    if ((linesit->pos).squared_length()<obstaclerays [index]) {
      select.objectlist.push_back (*linesit);
    }
    linesit++;
  }
}

void TemporalDifferenceSL::remove_lines_outside_field (VisibleObjectList& select, const VisibleObjectList& all, Vec p, Angle h) {
  select.objectlist.clear();
  select.timestamp = all.timestamp;

  double cutx = maxx-200;
  double cuty = maxy-200;
  double sinh=sin(h.get_rad());
  double cosh=cos(h.get_rad());

  vector<VisibleObject>::const_iterator linesit = all.objectlist.begin();
  vector<VisibleObject>::const_iterator linesitend = all.objectlist.end();
  while (linesit!=linesitend) {
    double x = abs(p.x+cosh*linesit->pos.x-sinh*linesit->pos.y);
    double y = abs(p.y+sinh*linesit->pos.x+cosh*linesit->pos.y);
    if (x<cutx && y<cuty) {
      select.objectlist.push_back (*linesit);
    }
    linesit++;
  }
}


void TemporalDifferenceSL::reset () throw () {
  main_hypothesis.init_randomly();
  main_hypothesis.reset_hypothesis=false;
  for (unsigned int i=0; i<alternative_hypothesis.size(); i++) {
    alternative_hypothesis[i].init_randomly(); 
    alternative_hypothesis[i].reset_hypothesis=false;
  }

  double d = (prefered_enter>=0 ? +1.0 : -1.0);
  Angle a0 = (prefered_enter<0 ? Angle::quarter : Angle::three_quarters);
  main_hypothesis.kfilter.set (Vec(d*(0.5*fg.field_width+0.5*fg.side_band_width), 0), a0, Vec(1e10, 1e10), 400);
  main_hypothesis.reset_hypothesis=(prefered_enter==0 ? false : true);
  if (alternative_hypothesis.size()>=1) {
    alternative_hypothesis[0].kfilter.set (Vec(d*(0.5*fg.field_width+0.5*fg.side_band_width), 0), a0+Angle::half, Vec(1e10, 1e10), 400);
    alternative_hypothesis[0].reset_hypothesis=(prefered_enter==0 ? false : true);
  }
  JWARNING ("SL: manual random reset");
}

void TemporalDifferenceSL::reset (Vec p) throw () {
  main_hypothesis.init_position(p);
  main_hypothesis.reset_hypothesis=false;
  for (unsigned int i=0; i<alternative_hypothesis.size(); i++) {
    alternative_hypothesis[i].init_position(p); 
    alternative_hypothesis[i].reset_hypothesis=false;
  }
  JWARNING ("SL: manual position reset");
}

void TemporalDifferenceSL::reset (Vec p, Angle h) throw () {
  main_hypothesis.init_pose(p,h);
  main_hypothesis.reset_hypothesis=true;
  main_hypothesis.heading_checked=true;
  for (unsigned int i=0; i<alternative_hypothesis.size(); i++) {
    alternative_hypothesis[i].init_randomly(); 
    alternative_hypothesis[i].reset_hypothesis=false;
  }
  JWARNING ("SL: manual position and heading reset");
}


RobotLocation TemporalDifferenceSL::get (Time t) const throw () {
  RobotLocation rloc;
  main_hypothesis.kfilter.get (rloc.pos, rloc.heading);
  rloc.valid = valid_hypothesis;
  return odobox.add_movement (rloc, latest_main_update, t);
}


void TemporalDifferenceSL::update_alternative (AlternativePose& alternative, const VisibleObjectList& lines, Vec odo_linear, Angle odo_angular, bool internal_alternatives, unsigned int cyc) {
  bool first_update=(alternative.num_iterations_alive==0);
  alternative.num_iterations_alive++;
  if (first_update) {
    odo_angular=Angle::zero;
    odo_linear=Vec::zero_vector;
  }

  Vec old_pos;
  Angle old_heading;
  alternative.kfilter.get (old_pos, old_heading);
  Vec trans_welt = odo_linear.rotate(old_heading);
  Vec odometry_pos = old_pos+trans_welt;
  Angle odometry_heading = old_heading+odo_angular;
  Vec visual_pos = odometry_pos;
  Angle visual_heading = odometry_heading;
  double ref_error = vis_optimiser->optimise (visual_pos, visual_heading, lines, lines.objectlist.size()>20 ? 10 : 20, max_lines);

  // Zwischenstand:
  //  (odometry_pos, odometry_heading) ist die fortgeschriebene Odometrieposition in Weltkoordinaten
  //  (visual_pos, visual_heading) ist die Bildposition in Weltkoordinaten

#if DEBUG_INTERNAL_SL
    LOUT << "% dark_blue solid thick circle " << visual_pos << " 20 line " << visual_pos << " " << visual_pos+30*Vec::unit_vector (visual_heading+Angle::quarter) << '\n';
    LOUT << "SL Interne Suchposition: " << ref_error << ' ' << visual_pos.x << ' ' << visual_pos.y << ' ' << visual_heading.get_deg() << '\n';
#endif

  // interne Alternativen berechnen und vergleichen:
  if (internal_alternatives) {
    Vec offsets [] = { Vec(600,0), Vec(0,600), Vec(-600,0), Vec(0,-600), Vec(0, 1200), Vec(0, -1200), Vec(1200, 0), Vec(-1200, 0) };
    double min_error = ref_error;
    int min_index=-1;
    double std_head = sqrt(alternative.kfilter.get_heading_variance ());
    const double sfac = 1.7;   // Faktor, der die Bevorzugung der zentralen internen Hypothese regelt: 1=keine Bevorzugung, >>1=starke Bevorzugung
    for (unsigned int j=0; j<num_internal_alternatives; j++) {
      unsigned int i=counter_internal_alternatives;
      counter_internal_alternatives=(counter_internal_alternatives+1)%8;
      Vec trial_pos = odometry_pos+offsets[i];
      bool not_near_center = odometry_pos.length()>2000;
      Angle trial_heading = odometry_heading+Angle::rad_angle(internal_random_direction && not_near_center ? urandom(0,2*M_PI) : std_head*nrandom());
      double err = vis_optimiser->optimise (trial_pos, trial_heading, lines, lines.objectlist.size()>20 ? 10 : 20, max_lines);
#if DEBUG_INTERNAL_SL
      LOUT << "% light_blue solid thin circle " << trial_pos << " 15 line " << trial_pos << " " << trial_pos+25*Vec::unit_vector (trial_heading+Angle::quarter) << '\n';
      LOUT << "SL Interne Suchposition: " << err << ' ' << trial_pos.x << ' ' << trial_pos.y << ' ' << trial_heading.get_deg() << '\n';
#endif
      if (err<min_error && sfac*err<ref_error) {
        min_index=i;
        min_error=err;
        visual_pos=trial_pos;
        visual_heading=trial_heading;
      }
    }
#if DEBUG_INTERNAL_SL
    LOUT << "SL waehle interne Suchposition: " << min_index+2 << '\n'; 
#endif
  }

  // Zwischenstand:
  //  (visual_pos, visual_heading) ist die beste Bildposition unter den internen Alternativen in Weltkoordinaten

  // die neue Position an den Kalmanfilter uebergeben, dabei Apertur-Problem beachten
  double ddphi;
  Vec ddxy;
  double num_lines = (lines.objectlist.size()>max_lines ? lines.objectlist.size() : max_lines);
  double latest_error = vis_optimiser->analyse (ddxy, ddphi, visual_pos, visual_heading, lines, max_lines)/(num_lines+1e-300);
  double fnum = 16.0/(lines.objectlist.size()+4.0)+0.7;   // Faktor, um die Anzahl gesehener Linien zu beruecksichtigen
  Vec var_xy (trans_welt.y*trans_welt.y+fnum*225*hoch6(log(abs(ddxy.x+1e-6))+7), trans_welt.x*trans_welt.x+fnum*225*hoch6(log(abs(ddxy.y)+1e-6)+7));  // aus den Kruemmungen Varianzen ableiten (heuristische Formel); 1e-6 um numerische Singularitaet zu vermeiden; trans_welt.x^2 bzw. trans_welt.y^2, um eine groessere Ungtenauigkeit quer zur Fahrtrichtung zu modellieren (Schaukeln des Roboters)
  double var_phi = fnum*3.0461741978670859865/(ddphi*ddphi+1-6);  // dito
  bool visual_update = false;
  if (lines.objectlist.size()>3) {
    double disc=abs((odometry_heading-visual_heading).get_rad_pi());
    if (disc<0.1*lines.objectlist.size()*sqrt(alternative.kfilter.get_heading_variance()) || disc<0.15) {
      alternative.kfilter.update (odo_linear, odo_angular, visual_pos, visual_heading, var_xy, var_phi, has_gyro);
      alternative.num_iterations_visual_input++;
      visual_update=true;
    } else {
      alternative.kfilter.update (odo_linear, odo_angular, has_gyro);
    }
  } else {
    alternative.kfilter.update (odo_linear, odo_angular, has_gyro);
  }
  Vec filter_pos;
  Angle filter_heading;
#if DEBUG_INTERNAL_SL
  double latest_quality = alternative.kfilter.get (filter_pos, filter_heading);
#else
  alternative.kfilter.get (filter_pos, filter_heading);
#endif

  // die Qualitaet der Hypothese berechnen:
  if (visual_update) {
  double error_nonlin=latest_error*(1.0/(1.0+exp(0.06*(latest_error-30)))+0.5);
  double f = 1.0/static_cast<double>(alternative.num_iterations_visual_input);
    if (f<ema_quality)
      f=ema_quality;
    if (alternative.num_iterations_visual_input==2 && alternative.visual_error > error_nonlin)
      f=1.0; // Sonderregelung, um ein im ersten Zyklus noch nicht richig positionierte Alternative zu ignorieren
    alternative.visual_error = (1-f)*alternative.visual_error+f*error_nonlin;
  }
  if (alternative.num_iterations_alive>=2) {
    double f = 1.0/(static_cast<double>(alternative.num_iterations_alive)-1);
    if (f<ema_quality)
      f=ema_quality;
    alternative.odo_vis_mismatch_linear = (1-f)*alternative.odo_vis_mismatch_linear+f*0.05*pow((odometry_pos-visual_pos).length()/cyc, 1.5);
    alternative.odo_vis_mismatch_angular = (1-f)*alternative.odo_vis_mismatch_angular+f*pow(abs((odometry_heading-visual_heading).get_rad_pi())/cyc, 1.5);
    alternative.way_alive+=odo_linear.length();
    if (alternative.way_alive>max_way_alive)
      alternative.way_alive=max_way_alive;
  }
  magic_alternative_evaluation (alternative);

#if DEBUG_INTERNAL_SL
  LOUT << "% dark_green solid thin circle " << old_pos << " 15 line " << old_pos << " " << old_pos+25*Vec::unit_vector (old_heading+Angle::quarter) << " black word " << old_pos << " old\n";
  LOUT << "% dark_green solid thin circle " << odometry_pos << " 15 line " << odometry_pos << " " << odometry_pos+25*Vec::unit_vector (odometry_heading+Angle::quarter) << " black word " << odometry_pos << " odo\n";
  LOUT << "% dark_green solid thin circle " << visual_pos << " 15 line " << visual_pos << " " << visual_pos+25*Vec::unit_vector (visual_heading+Angle::quarter) << " black word " << visual_pos << " vis\n";
  LOUT << "% dark_green solid thin circle " << filter_pos << " 15 line " << filter_pos << " " << filter_pos+25*Vec::unit_vector (filter_heading+Angle::quarter) << " black word " << filter_pos << " filter\n";
  LOUT << "SL Odometrie-Weg: " << odo_linear.x << ' ' << odo_linear.y << ' ' << odo_angular.get_deg() << '\n';
  LOUT << "SL Bildverarbeitung Fehler und 2. Ableitungen: " << latest_error << ' ' << ddxy.x << ' ' << ddxy.y << ' ' << ddphi << '\n';
  LOUT << "SL Filterqualitaet: " << latest_quality << '\n';
#endif
}


bool TemporalDifferenceSL::check_goals (AlternativePose& alternative, const VisibleObjectList& goals, bool consider_yellow_goal, bool consider_blue_goal, double turnplus) {
  bool return_value=false;
  alternative.rotation_since_heading_check+=abs(turnplus);
  if (consider_yellow_goal || consider_blue_goal) {
    unsigned int num_true_dir=0;   // Anzahl Bestaetigungen der aktuellen Position
    unsigned int num_false_dir=0;  // Anzahl Bestaetigungen der gespiegelten Position
    unsigned int num_fail=0;   // Anzahl Torpunkte, die gar nicht passen
    vector<VisibleObject>::const_iterator visit = goals.objectlist.begin();
    vector<VisibleObject>::const_iterator visend = goals.objectlist.end();
    int c;  // Farbe des Tors +1=blau, -1=gelb
    Vec rpos;
    Angle rhead;
    alternative.kfilter.get (rpos, rhead);
    while (visit<visend) {
      if (visit->object_type==VisibleObject::blue_goal || visit->object_type==VisibleObject::blue_goal_post_left || visit->object_type==VisibleObject::blue_goal_post_right)
        c=+1;
      else
        c=-1;
      Angle goal_angle = rhead+visit->pos.angle();
      double goal_distance = visit->pos.length();
      if ((gsx+gsy-(rpos.x+rpos.y)<0) || (gsx+gsy+(rpos.x-rpos.y)<0) || (gsy-rpos.y<0) || (gsx+gsy+(rpos.x+rpos.y)<0) || (gsx+gsy-(rpos.x-rpos.y)<0) || (gsy+rpos.y<0)) {
        num_fail++;
      // Roboter befindet sich hinter der Torlinie oder in flachem Winkel zum Tor, daher nicht auswerten
      } else {
        if ((c==+1 && consider_blue_goal) || (c==-1 && consider_yellow_goal)) {
          Angle blue_goal_right = (Vec (gsx,gsy)-rpos).angle();
          Angle blue_goal_left = (Vec (-gsx,gsy)-rpos).angle();
          Angle yellow_goal_right = (Vec (-gsx,-gsy)-rpos).angle();
          Angle yellow_goal_left = (Vec (gsx,-gsy)-rpos).angle();
          double blue_goal_distance = (Vec (0,gsy)-rpos).length();
          double yellow_goal_distance = (Vec (0,-gsy)-rpos).length();
          if (goal_angle.in_between (blue_goal_right, blue_goal_left) && goal_distance+1000>blue_goal_distance) {
            // Beobachtung passt zum blauen Tor
            if (c==1)
              num_true_dir++;
            else
              num_false_dir++;
          } else if (goal_angle.in_between (yellow_goal_right, yellow_goal_left) && goal_distance+1000>yellow_goal_distance) {
            // Beobachtung passt zum gelben Tor
            if (c==1)
              num_false_dir++;
            else
              num_true_dir++;
          } else
            num_fail++;
        }
      }
      visit++;
    }
    alternative.ema_right_goal*=0.75;
    alternative.ema_wrong_goal*=0.75;
    if (num_true_dir>0 && num_false_dir==0) 
      alternative.ema_right_goal+=0.25*num_true_dir;
    if (num_true_dir==0 && num_false_dir>0) {
      alternative.ema_wrong_goal+=0.25*num_false_dir;
      if (alternative.ema_wrong_goal>alternative.ema_right_goal+0.2 && alternative.ema_wrong_goal>0.45) {
        alternative.kfilter.mirror();
        alternative.heading_checked=true;
        alternative.rotation_since_heading_check=0;
        double sw=alternative.ema_wrong_goal;
        alternative.ema_wrong_goal=alternative.ema_right_goal;
        alternative.ema_right_goal=sw;
        return_value=true;
      }
    }
#if DEBUG_INTERNAL_SL
    LOUT << "SL Goalcheck: fail=" << num_fail << " true=" << num_true_dir << " false=" << num_false_dir << '\n';
#endif
  }
  return return_value;
}


bool TemporalDifferenceSL::update (const VisibleObjectList& alllines, const VisibleObjectList& obstacles, const VisibleObjectList& goals) throw () {
  bool return_value=false;
  const VisibleObjectList* lines_ptr=&alllines;
  if (do_remove_occluded_lines || do_remove_occluded_lines_far) {
    remove_occluded_lines (selectedlines1, alllines, obstacles, max_line_distance, do_remove_occluded_lines_far ? 500 : 1);
  } else {
    selectedlines1=alllines;
  }
  lines_ptr=&selectedlines1;
  VisibleObjectList& linesx (selectedlines1);  // Linien ohne Verdeckungen
  if (alllines.objectlist.size()==0) {
    linesx.timestamp.update();
    linesx.timestamp.add_msec(-static_cast<long int>(average_vision_delay));
  } else {
    average_vision_delay = 0.95*average_vision_delay+0.05*alllines.timestamp.elapsed_msec();
  }
  RobotLocation delta_xv_main = odobox.movement (latest_main_update, linesx.timestamp, 1);
  if (linesx.timestamp.diff_msec (latest_main_update)>1000) {
    Time t2 = latest_main_update;
    t2.add_msec(200);
    delta_xv_main = odobox.movement (latest_main_update, t2, 1);
  }
  if (do_remove_lines_outside_field) {
    Vec pos_robot_imagetime_expected;
    Angle angle_robot_imagetime_expected;
    main_hypothesis.kfilter.get (pos_robot_imagetime_expected, angle_robot_imagetime_expected);
    remove_lines_outside_field (selectedlines2, selectedlines1, pos_robot_imagetime_expected, angle_robot_imagetime_expected);
    lines_ptr=&selectedlines2;
  }
  const VisibleObjectList& lines (*lines_ptr); // Linien ohne Verdeckungen und ausserhalb bzgl. Haupthypothese

  // pruefen, ob alle Alternativen einmal aktualisiert wurden; wenn ja, Aehnlichkeit der Hypothesen ueberpruefen und Qualitaet vergleichen
  if (next_alternative>=alternative_hypothesis.size() && linesx.objectlist.size()>10) {
    next_alternative=0;
    lines_all_update = linesx;
    RobotLocation delta_xv_update = odobox.movement (latest_all_update, linesx.timestamp, 1);
    odo_linear_all_update = delta_xv_update.pos;
    odo_angular_all_update = delta_xv_update.heading;
    latest_all_update = linesx.timestamp;
    cycweight_all_update=(cycweight>0 ? cycweight : 1);
    cycweight=0;

    alternative_competition ();
  }

  vis_optimiser->calculate_distance_weights (lines, max_lines);
  update_alternative  (main_hypothesis, lines, delta_xv_main.pos, delta_xv_main.heading, num_internal_alternatives>0);
  cycweight++;
  return_value=true;
  latest_main_update=lines.timestamp;
  if (main_hypothesis.magic_number<main_to_reset_hysteresis_lower)
    main_hypothesis.reset_hypothesis=true;
  if (main_hypothesis.magic_number>main_to_reset_hysteresis_upper && main_hypothesis.num_iterations_alive>=5)
    main_hypothesis.reset_hypothesis=false;

  // fuer einige Alternativen einen Update durchfuehren:
  if (linesx.objectlist.size()>10) {
    vis_optimiser->calculate_distance_weights (lines_all_update, max_lines);
    unsigned int maxi = next_alternative+num_alternative_updates;
    if (maxi>alternative_hypothesis.size())
      maxi=alternative_hypothesis.size();
    for ( ; next_alternative<maxi; next_alternative++) {
      update_alternative (alternative_hypothesis[next_alternative], lines_all_update, odo_linear_all_update, odo_angular_all_update, false, cycweight_all_update);
    }
  }

  // Nach Toren schauen
  if (check_goals (main_hypothesis, goals, consider_yellow_goal, consider_blue_goal, delta_xv_main.heading.get_rad_pi()))
    JWARNING ("SL: switching global orientation");
  for (unsigned int i=0; i<alternative_hypothesis.size(); i++)
    check_goals (alternative_hypothesis[i], goals, consider_yellow_goal, consider_blue_goal, delta_xv_main.heading.get_rad_pi());

#if DEBUG_EXTERNAL_SL
  // Debug-Ausgabe
  int dir = WorldModel::get_main_world_model().get_own_half();
  Vec p;
  Angle h;
  Time now;
  for (unsigned int i=0; i<alternative_hypothesis.size(); i++) {
    alternative_hypothesis[i].kfilter.get (p,h);
    LOUT << "% white solid cross " << dir*p.x << ' ' << dir*p.y << " word " << dir*p.x << ' ' << dir*p.y << ' ' << i+1 << '\n';
    LOUT << "SL Alternative " << i+1 << (alternative_hypothesis[i].reset_hypothesis ? " (R)" : " (-)") << ": \t" 
        << alternative_hypothesis[i].num_iterations_alive << '\t'
        << alternative_hypothesis[i].num_iterations_visual_input << '\t'
        << format8(alternative_hypothesis[i].way_alive) << '\t'
        << format8(alternative_hypothesis[i].visual_error) << '\t'
        << format8(alternative_hypothesis[i].odo_vis_mismatch_linear) << '\t'
        << format8(alternative_hypothesis[i].odo_vis_mismatch_angular) << '\t'
        << ">> " << format8(alternative_hypothesis[i].magic_number) << " <<" << '\n';
  }
  main_hypothesis.kfilter.get (p,h);
  LOUT << "% dark_blue solid cross " << dir*p.x << ' ' << dir*p.y  << " word " << dir*p.x << ' ' << dir*p.y << " H\n";
  LOUT << "SL Haupthyp. " << (main_hypothesis.reset_hypothesis ? " (R)" : " (-)") << ": \t"
      << main_hypothesis.num_iterations_alive << '\t'
      << main_hypothesis.num_iterations_visual_input << '\t'
      << format8(main_hypothesis.way_alive) << '\t'
      << format8(main_hypothesis.visual_error) << '\t'
      << format8(main_hypothesis.odo_vis_mismatch_linear) << '\t'
      << format8(main_hypothesis.odo_vis_mismatch_angular) << '\t'
      << ">> " << format8(main_hypothesis.magic_number) << " <<" << '\n';
#endif
  valid_hypothesis = (main_hypothesis.magic_number<50) || (valid_hypothesis && main_hypothesis.magic_number<=80);
  return return_value;  // Update der Haupthypothese
}


void TemporalDifferenceSL::alternative_competition () {
  Vec main_pos, alter1_pos, alter2_pos;
  Angle main_head, alter1_head, alter2_head;
  main_hypothesis.kfilter.get (main_pos, main_head);
  alternative_erase.resize (alternative_hypothesis.size());  // wird auf true gesetzt, wenn Alternative entfernt werden soll

  // pruefen, ob Alternativen zu aehnlich zur Haupthypothese oder zu alt sind:
  for (unsigned int i=0; i<alternative_hypothesis.size(); i++) {
    alternative_hypothesis[i].kfilter.get (alter1_pos, alter1_head);
    bool replace = false;
    replace |= (abs(alter1_pos.x)>maxx) || (abs(alter1_pos.y)>maxy);   // liegt auserhalb des Feldes
    replace |= ((alter1_pos-main_pos).squared_length()<40000 && (alter1_head-main_head).in_between (Angle::eleven_twelvth, Angle::twelvth));  // liegt der Hauptposition zu nahe
    replace |= ((alter1_pos+main_pos).squared_length()<40000 && (alter1_head+main_head).in_between (Angle::eleven_twelvth, Angle::twelvth)); // liegt der Hauptposition gespiegelt zu nahe
    replace |= (alternative_hypothesis[i].num_iterations_alive>=30);  // gaenzlich zu alt, um noch interessant zu sein
    replace |= (alternative_hypothesis[i].num_iterations_alive>=10 && alternative_hypothesis[i].visual_error>10*main_hypothesis.visual_error);
    unsigned int num_crit=0;
    if (alternative_hypothesis[i].num_iterations_alive>=15 && alternative_hypothesis[i].visual_error>3*main_hypothesis.visual_error)
      num_crit++;
    if (alternative_hypothesis[i].num_iterations_alive>=15 && alternative_hypothesis[i].way_alive>1000 && alternative_hypothesis[i].odo_vis_mismatch_linear>3*main_hypothesis.odo_vis_mismatch_linear)
      num_crit++;
    if (alternative_hypothesis[i].num_iterations_alive>=15 && alternative_hypothesis[i].way_alive>1000 && alternative_hypothesis[i].odo_vis_mismatch_angular>3*main_hypothesis.odo_vis_mismatch_angular)
      num_crit++;
    replace |= (num_crit>=2);
    alternative_erase[i]=replace;
  }

  // pruefen, ob sich zwei Alternativen zu aehnlich sind:
  for (unsigned int i=0; i<alternative_hypothesis.size(); i++) {
    if (!alternative_erase[i]) {
      alternative_hypothesis[i].kfilter.get (alter1_pos, alter1_head);
      for (unsigned int j=i+1; j<alternative_hypothesis.size(); j++) {
        if (!alternative_erase[j]) {
          alternative_hypothesis[j].kfilter.get (alter2_pos, alter2_head);
          if (((alter1_pos-alter2_pos).squared_length()<40000 && (alter1_head-alter2_head).in_between (Angle::eleven_twelvth, Angle::twelvth)) || ((alter1_pos+alter2_pos).squared_length()<40000 && (alter1_head+alter2_head).in_between (Angle::eleven_twelvth, Angle::twelvth))) {
            // Alternativen i und j sind sich zu aehnlich, eine davon loeschen
            if (alternative_hypothesis[i].magic_number<alternative_hypothesis[j].magic_number)
              alternative_erase[j]=true;
            else
              alternative_erase[i]=true;
          }
        }
      }
    }
  }

  // Alle zu loeschenden Alternativen entfernen/reinitialisieren
  for (int i=alternative_hypothesis.size()-1; i>=0; i--) {
    if (alternative_erase[i]) {
      if (alternative_hypothesis.size()>num_alternatives)
        alternative_hypothesis.erase (alternative_hypothesis.begin()+i);
      else
        alternative_hypothesis[i].init_randomly();
    }
  }

  // Alternativen mit Haupthypothese vergleichen
  double q=main_hypothesis.magic_number;
  int index=-1;
  for (unsigned int i=0; i<alternative_hypothesis.size(); i++) {
    double q1=alternative_hypothesis[i].magic_number;
    if (q1<q) {
      q=q1;
      index=i;
    }
  }

  if (index>=0) {
    // Haupthypothese ersetzen
    AlternativePose sw = main_hypothesis;
    Vec p1, p2;
    Angle h1, h2;
    main_hypothesis.kfilter.get (p1,h1);
    main_hypothesis = alternative_hypothesis[index];
    main_hypothesis.kfilter.get (p2,h2);
    if (!main_hypothesis.heading_checked) {
      main_hypothesis.ema_right_goal=0.5;
      main_hypothesis.ema_wrong_goal=0.5;
      if ((h1-h2).in_between (Angle::quarter, Angle::three_quarters))
        main_hypothesis.kfilter.mirror();        // Orientierung ggf. anpassen; trotzdem kann was schief gehen
    }
    main_hypothesis.reset_hypothesis=true;
    sw.reset_hypothesis=false;
    sw.num_iterations_alive=1;
    alternative_hypothesis[index] = sw;  // bisherige Haupthypothese darf weiterleben, aber nicht als reset_hypothese um staendiges Wechseln zu vermeiden
#if DEBUG_EXTERNAL_SL
    LOUT << "SL externe Ersetzung durch Alternative " << index+1 << '\n';
#endif
    JWARNING ("SL: auto robot displacement");
  }
}

double TemporalDifferenceSL::magic_alternative_evaluation (AlternativePose& a) {
  double unexperience_factor = (1.0+27.0/(8.0+a.num_iterations_visual_input)) * (1.0+500.0/(500.0+a.way_alive));
  double reset_factor = (a.reset_hypothesis ? 0.6 : 1.0);
  double reset_add = (a.reset_hypothesis ? 0 : 10);
  Vec pos;
  Angle head;
  a.kfilter.get (pos,head);
  double out_of_field1 = max(0.0,(abs(pos.x)-0.5*fg.field_width-fg.side_band_width-200));
  double out_of_field2 = max(0.0,(abs(pos.y)-0.5*fg.field_length-fg.goal_band_width-200));
  a.magic_number=unexperience_factor*reset_factor*(reset_add+600*a.visual_error+0.1*a.odo_vis_mismatch_linear+(has_gyro ? 500 : 100)*a.odo_vis_mismatch_angular+out_of_field1+out_of_field2);
  return a.magic_number;
}

void TemporalDifferenceSL::slMirrorHint (Vec v) throw () {
  Vec rpos;
  Angle rhead;
  main_hypothesis.kfilter.get (rpos, rhead);
  if (((rpos-v).length()>((rpos+v).length()+1500)) &&
        ((rpos+v).length()<1000) &&
      (!main_hypothesis.heading_checked || (main_hypothesis.rotation_since_heading_check>20))) {
    main_hypothesis.latestSLMirrorHintTimes.get().update();
    main_hypothesis.latestSLMirrorHintTimes.step(1);
    JMESSAGETS ("receive SL mirror hint");
    unsigned int numHints=0;
    int oldestElapsed=0;
    for (unsigned int i=0; i<main_hypothesis.latestSLMirrorHintTimes.size(); i++) {
      int elapsed = main_hypothesis.latestSLMirrorHintTimes[i].elapsed_msec();
      if (elapsed<5000) {
        numHints++;
        if (oldestElapsed<elapsed)
          oldestElapsed=elapsed;
      }
    }
    if (numHints>=5 && oldestElapsed>2000) {
      JWARNING ("switching sides due to SL mirror hint");
      main_hypothesis.kfilter.mirror();
//      main_hypothesis.heading_checked=true;
//      main_hypothesis.rotation_since_heading_check=0;
      double sw=main_hypothesis.ema_wrong_goal;
      main_hypothesis.ema_wrong_goal=main_hypothesis.ema_right_goal;
      main_hypothesis.ema_right_goal=sw;
    }
  }
}
