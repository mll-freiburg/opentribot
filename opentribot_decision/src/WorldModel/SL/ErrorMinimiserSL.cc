
#include "ErrorMinimiserSL.h"
#include "../WorldModel.h"
#include "../Prediction/update_robot_location.h"
#include "../../Structures/Journal.h"
#include "../../Fundamental/random.h"
#include <cmath>

using namespace std;
using namespace Tribots;

#define DEBUG_INTERNAL_SL 0   // Debugen der Hauptpositionshypothesen
#define DEBUG_EXTERNAL_SL 0   // Debuggen der alternativen Hypothesen


namespace {
  double maxx = 10;  // Hilfsgroesse: halbe Spielfeldbreite+Rand, wird in ErrorMinimiserSL-Konstruktor gesetzt
  double maxy = 10;  // Hilfsgroesse: halbe Spielfelblaenge+Rand, dito
  double gsx = 10; // Hilfsgroesse: halbe Torbreite, dito
  double gsy = 10; // Hilfsgroesse: halbe Spielfeldlaenge, dito

  VisibleObjectList selectedlines1;  // um den Speicher nicht immer wieder neu allokieren zu muessen
  VisibleObjectList selectedlines2;  // dito
  vector<double> obstaclerays;  // dito


  inline void chosemin (double& a1, const double& a2) {
    a1 = (a1<a2 ? a1 : a2);
  }
}

ErrorMinimiserSL::ErrorMinimiserSL (const ConfigReader& reader, const OdometryContainer& odo, const FieldGeometry& fg) throw (std::bad_alloc) : odobox (odo) , external_alternatives (1) {
  maxx = 0.5*fg.field_width+fg.side_band_width;
  maxy = 0.5*fg.field_length+fg.goal_band_width;
  gsx = 0.5*fg.goal_width;
  gsy = 0.5*fg.field_length;

  field_lut = new FieldLUT (fg, 50);
  double err_width;
  if (!reader.get ("ErrorMinimiserSL::error_width", err_width))
    err_width=250;
  double dist_param;
  if (!reader.get ("ErrorMinimiserSL::distance_weight_parameter", dist_param))
    dist_param=1e50;   // default: praktisch gleiche Gewichtung unabhaengig von Entfernung
  consider_yellow_goal=consider_blue_goal=true;
  reader.get ("ErrorMinimiserSL::consider_yellow_goal", consider_yellow_goal);
  reader.get ("ErrorMinimiserSL::consider_blue_goal", consider_blue_goal);
  use_internal_alternatives=true;
  reader.get ("ErrorMinimiserSL::use_internal_alternatives", use_internal_alternatives);
  num_external_alternatives = 1;
  reader.get ("ErrorMinimiserSL::number_external_alternatives", num_external_alternatives);
  max_lines = 1000;
  reader.get ("ErrorMinimiserSL::max_lines", max_lines);
  external_alternatives.resize(num_external_alternatives);
  do_remove_occluded_lines = true;
  reader.get ("ErrorMinimiserSL::remove_occluded_lines", do_remove_occluded_lines);
  do_remove_lines_outside_field = true;
  reader.get ("ErrorMinimiserSL::remove_lines_outside_field", do_remove_lines_outside_field);
  max_line_distance = 6000;
  reader.get ("ErrorMinimiserSL::remove_lines_from_distance", max_line_distance);
  has_gyro=false;
  reader.get ("ErrorMinimiserSL::gyroscope", has_gyro);
  
  vis_optimiser = new VisualPositionOptimiser (*field_lut, err_width, dist_param);

  reset ();
}

ErrorMinimiserSL::~ErrorMinimiserSL () throw () {
  delete field_lut;
  delete vis_optimiser;
}

namespace {
  inline double hoch6 (double x){
    double y=x*x*x;
    return y*y;
  }
}


void ErrorMinimiserSL::update_alternative (AltPos& altpos, const VisibleObjectList& lines, const VisibleObjectList& goals, bool internal_alternatives, bool is_main_alternative) {
  Time current_update = lines.timestamp;
  RobotLocation delta_xv = odobox.movement (latest_update, current_update, 1);

  Vec old_pos;
  Angle old_heading;
  altpos.rpos_filter.get (old_pos, old_heading);
  Vec trans_welt = delta_xv.pos.rotate(old_heading);
  Vec odometry_pos = old_pos+delta_xv.pos.rotate(old_heading);
  Angle odometry_heading = old_heading+delta_xv.heading;
  Vec visual_pos = odometry_pos;
  Angle visual_heading = odometry_heading;
  double ref_error = vis_optimiser->optimise (visual_pos, visual_heading, lines, lines.objectlist.size()>20 ? 10 : 20, max_lines);
  
  if (internal_alternatives && lines.objectlist.size()>=10) {
#if DEBUG_INTERNAL_SL
    LOUT << "% dark_blue solid thick circle " << visual_pos << " 20 line " << visual_pos << " " << visual_pos+30*Vec::unit_vector (visual_heading+Angle::quarter) << '\n';
    LOUT << "SL Interne Suchposition: " << ref_error << ' ' << visual_pos.x << ' ' << visual_pos.y << ' ' << visual_heading.get_deg() << '\n';
#endif
    Vec offsets [] = { Vec(600,0), Vec(0,600), Vec(-600,0), Vec(0,-600) };
    double min_error = ref_error;
    int min_index=-1;
    double std_head = sqrt(altpos.rpos_filter.get_heading_variance ());
    double sfac = 1.7-0.7/(static_cast<double>(cycles_since_reset)+1.0);   // direkt nach Reset die Odometrieposition nicht bevorzugen
    for (unsigned int i=0; i<4; i++) {
      Vec trial_pos = odometry_pos+offsets[i];
      Angle trial_heading = odometry_heading+Angle::rad_angle(std_head*nrandom());;
      double err = vis_optimiser->optimise (trial_pos, trial_heading, lines, lines.objectlist.size()>20 ? 10 : 20, max_lines);
      if (err<min_error && sfac*err<ref_error) {
        min_index=i;
        min_error=err;
        visual_pos=trial_pos;
        visual_heading=trial_heading;
      }
#if DEBUG_INTERNAL_SL
      LOUT << "% dark_blue solid thin circle " << trial_pos << " 15 line " << trial_pos << " " << visual_pos+25*Vec::unit_vector (trial_heading+Angle::quarter) << '\n';
      LOUT << "SL Interne Suchposition: " << ref_error << ' ' << trial_pos.x << ' ' << trial_pos.y << ' ' << trial_heading.get_deg() << '\n';
#endif
    }
#if DEBUG_INTERNAL_SL
    LOUT << "SL waehle interne Suchposition: " << min_index+2 << '\n'; 
#endif
  }

  double ddphi;
  Vec ddxy;
  altpos.latest_error = vis_optimiser->analyse (ddxy, ddphi, visual_pos, visual_heading, lines, max_lines);
  double fnum = 16.0/(lines.objectlist.size()+4.0)+0.7;   // Faktor, um die Anzahl gesehener Linien zu beruecksichtigen
  Vec var_xy (trans_welt.y*trans_welt.y+fnum*225*hoch6(log(abs(ddxy.x+1e-6))+7), trans_welt.x*trans_welt.x+fnum*225*hoch6(log(abs(ddxy.y)+1e-6)+7));  // aus den Kruemmungen Varianzen ableiten (heuristische Formel); 1e-6 um numerische Singularitaet zu vermeiden; trans_welt.x^2 bzw. trans_welt.y^2, um eine groessere Ungtenauigkeit quer zur Fahrtrichtung zu modellieren (Schaukeln des Roboters)
  double var_phi = fnum*3.0461741978670859865/(ddphi*ddphi+1-6);  // dito
  if (lines.objectlist.size()>3)
    altpos.rpos_filter.update (delta_xv.pos, delta_xv.heading, visual_pos, visual_heading, var_xy, var_phi, has_gyro);
  else
    altpos.rpos_filter.update (delta_xv.pos, delta_xv.heading, has_gyro);
  Vec filter_pos;
  Angle filter_heading;
#if DEBUG_INTERNAL_SL
  double latest_quality = altpos.rpos_filter.get (filter_pos, filter_heading);
#else
  altpos.rpos_filter.get (filter_pos, filter_heading);
#endif

#if DEBUG_INTERNAL_SL
  LOUT << "% dark_green solid thin circle " << old_pos << " 15 line " << old_pos << " " << old_pos+25*Vec::unit_vector (old_heading+Angle::quarter) << " black word " << old_pos << " old\n";
  LOUT << "% dark_green solid thin circle " << odometry_pos << " 15 line " << odometry_pos << " " << odometry_pos+25*Vec::unit_vector (odometry_heading+Angle::quarter) << " black word " << odometry_pos << " odo\n";
  LOUT << "% dark_green solid thin circle " << visual_pos << " 15 line " << visual_pos << " " << visual_pos+25*Vec::unit_vector (visual_heading+Angle::quarter) << " black word " << visual_pos << " vis\n";
  LOUT << "% dark_green solid thin circle " << filter_pos << " 15 line " << filter_pos << " " << filter_pos+25*Vec::unit_vector (filter_heading+Angle::quarter) << " black word " << filter_pos << " filter\n";
  LOUT << "SL Odometrie-Weg: " << delta_xv.pos.x << ' ' << delta_xv.pos.y << ' ' << delta_xv.heading.get_deg() << '\n';
  LOUT << "SL Bildverarbeitung Fehler und 2. Ableitungen: " << altpos.latest_error << ' ' << ddxy.x << ' ' << ddxy.y << ' ' << ddphi << '\n';
  LOUT << "SL Filterqualitaet: " << latest_quality << '\n';
#endif
  
  // noch ergaenzen: globale Lokalisierung mit Torinformation
  if (consider_yellow_goal || consider_blue_goal) {
    unsigned int num_true_dir=0;   // Anzahl Bestaetigungen der aktuellen Position
    unsigned int num_false_dir=0;  // Anzahl Bestaetigungen der gespiegelten Position
    unsigned int num_fail=0;   // Anzahl Torpunkte, die gar nicht passen
    vector<VisibleObject>::const_iterator visit = goals.objectlist.begin();
    vector<VisibleObject>::const_iterator visend = goals.objectlist.end();
    int c;  // Farbe des Tors +1=blau, -1=gelb
    Vec rpos;
    Angle rhead;
    altpos.rpos_filter.get (rpos, rhead);
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
    altpos.ema_right_goal*=0.75;
    altpos.ema_wrong_goal*=0.75;
    if (num_true_dir>0 && num_false_dir==0) 
      altpos.ema_right_goal+=0.25*num_true_dir;
    if (num_true_dir==0 && num_false_dir>0) {
      altpos.ema_wrong_goal+=0.25*num_false_dir;
      if (altpos.ema_wrong_goal>altpos.ema_right_goal+0.2 && altpos.ema_wrong_goal>0.45) {
        altpos.rpos_filter.mirror();
        altpos.heading_checked=true;
        double sw=altpos.ema_wrong_goal;
        altpos.ema_wrong_goal=altpos.ema_right_goal;
        altpos.ema_right_goal=sw;
        if (is_main_alternative)
          JWARNING ("SL: switching global orientation");
      }
    }
#if DEBUG_INTERNAL_SL
    LOUT << "SL Goalcheck: fail=" << num_fail << " true=" << num_true_dir << " false=" << num_false_dir << '\n';
#endif
  }
}

bool ErrorMinimiserSL::update (const VisibleObjectList& alllines, const VisibleObjectList& obstacles, const VisibleObjectList& goals) throw () {
  const VisibleObjectList* lines_ptr=&alllines;
  if (do_remove_occluded_lines) {
    remove_occluded_lines (selectedlines1, alllines, obstacles, max_line_distance);
    lines_ptr=&selectedlines1;
  }
  if (do_remove_lines_outside_field) {
    RobotLocation delta_xv = odobox.movement (latest_update, alllines.timestamp, 1);
    Vec pos_robot_imagetime_expected;
    Angle angle_robot_imagetime_expected;
    main_position.rpos_filter.get (pos_robot_imagetime_expected, angle_robot_imagetime_expected);
    remove_lines_outside_field (selectedlines2, selectedlines1, pos_robot_imagetime_expected, angle_robot_imagetime_expected);
    lines_ptr=&selectedlines2;
  }
      
  const VisibleObjectList& lines (*lines_ptr);
    
  // fuer jede Alternive einen Update durchfuehren
  if (lines.objectlist.size()>=10) {
    vis_optimiser->calculate_distance_weights (lines, max_lines);
    update_alternative (main_position, lines, goals, use_internal_alternatives, true);
    for (unsigned int i=0; i<num_external_alternatives; i++)
      update_alternative (external_alternatives[i], lines, goals, false, false);
    cycles_since_reset++;
    latest_update = lines.timestamp;

    // pruefen, ob Alternativen zu aehnlich sind, auserhalb des Feldes liegen oder zu alt sind. wenn ja, ersetzen
    Vec p_main, p1, p2;
    Angle h_main, h1, h2;
    Time now;
    main_position.rpos_filter.get (p_main, h_main);
    for (unsigned int i=0; i<num_external_alternatives; i++) {
      external_alternatives[i].rpos_filter.get (p1,h1);
      double dt = now.diff_msec(external_alternatives[i].init_time);
      bool replace = false;
      replace |= (abs(p1.x)>maxx) || (abs(p1.y)>maxy);   // liegt auserhalb des Feldes
      replace |= ((p1-p_main).squared_length()<40000 && (h1-h_main).in_between (Angle::eleven_twelvth, Angle::twelvth));  // liegt der Hauptposition zu nahe
      replace |= ((p1+p_main).squared_length()<40000 && (h1+h_main).in_between (Angle::eleven_twelvth, Angle::twelvth)); // liegt der Hauptposition gespiegelt zu nahe
      replace |= (dt>=2000);  // gaenzlich zu alt
      replace |= (dt>=1000 &&  external_alternatives[i].winning_coefficient<0.2);
      replace |= (dt>=500 && external_alternatives[i].winning_coefficient<0.1);
      replace |= (dt>=150 && external_alternatives[i].winning_coefficient<0.001);  // in 120 ms nie gewonnen -> hoffnungslose Positionen
      if (replace) {
        AltPos z;
        external_alternatives[i]=z;
        z.rpos_filter.get (p1,h1);
      }
      for (unsigned int j=i+1; j<num_external_alternatives; j++) {
        external_alternatives[j].rpos_filter.get (p2,h2);
        if (((p1-p2).squared_length()<40000 && (h1-h2).in_between (Angle::eleven_twelvth, Angle::twelvth)) || ((p1+p2).squared_length()<40000 && (h1+h2).in_between (Angle::eleven_twelvth, Angle::twelvth))) {
          if (external_alternatives[j].winning_coefficient < external_alternatives[i].winning_coefficient) {
            AltPos z;
            external_alternatives[j]=z;
          } else {
            external_alternatives[i]=external_alternatives[i];
            AltPos z;
            external_alternatives[j]=z;
            external_alternatives[i].rpos_filter.get (p1,h1);
          }
        }
      }
    }

    // Gewinner bestimmen und Erfolgskoeffizienten anpassen:
    double min_error = main_position.latest_error-3-10.0*(max_lines<lines.objectlist.size() ? max_lines : lines.objectlist.size())/200;  // der Hauptposition einen Vorsprung einraeumen
    int min_index=-1;
    for (unsigned int i=0; i<num_external_alternatives; i++)
      if (external_alternatives[i].latest_error<min_error) {
      min_error=external_alternatives[i].latest_error;
      min_index=i;
    }
    main_position.winning_coefficient=0.95*main_position.winning_coefficient+(min_index==-1 ? 0.05 : 0.0);
    for (unsigned int i=0; i<num_external_alternatives; i++)
      external_alternatives[i].winning_coefficient=0.95*external_alternatives[i].winning_coefficient+(min_index==static_cast<int>(i) ? 0.05 : 0.0); 
    
    // nach Alternativen suchen:
    double max_win = main_position.winning_coefficient;
    int max_index=-1;
    for (unsigned int i=0; i<num_external_alternatives; i++)
      if (external_alternatives[i].winning_coefficient>max_win) {
        max_win=external_alternatives[i].winning_coefficient;
        max_index=i;
      }
    if (max_index!=-1 && max_win>=0.3) {
      Vec p1, p2;
      Angle h1, h2;
      main_position.rpos_filter.get (p1,h1);
      main_position = external_alternatives[max_index];
      main_position.rpos_filter.get (p2,h2);
      if (!main_position.heading_checked) {
        main_position.ema_right_goal=0.5;
        main_position.ema_wrong_goal=0.5;
        if ((h1-h2).in_between (Angle::quarter, Angle::three_quarters))
          main_position.rpos_filter.mirror();        // Orientierung ggf. anpassen; trotzdem kann was schief gehen
      }
      AltPos z;
      external_alternatives[max_index]=z;
#if DEBUG_EXTERNAL_SL
      LOUT << "SL externe Ersetzung durch Alternative " << max_index+1 << '\n';
#endif
      JWARNING ("SL: auto robot displacement");
    }
    
#if DEBUG_EXTERNAL_SL
    int dir = WorldModel::get_main_world_model().get_own_half();
    Vec p;
    Angle h;
    for (unsigned int i=0; i<num_external_alternatives; i++) {
      external_alternatives[i].rpos_filter.get (p,h);
      LOUT << "% dark_blue solid cross " << dir*p.x << ' ' << dir*p.y << " word " << dir*p.x << ' ' << dir*p.y << ' ' << i << '\n';
      LOUT << "SL Alternative " << i+1 << ": " << now.diff_msec(external_alternatives[i].init_time) << '\t' << external_alternatives[i].winning_coefficient << '\t' << external_alternatives[i].latest_error << '\t' << dir*p.x << '\t' << dir*p.y << '\t' << (h+(dir>0 ? Angle::zero : Angle::half)).get_deg() << '\n';
    }
    main_position.rpos_filter.get (p,h);
    LOUT << "% dark_blue solid cross " << dir*p.x << ' ' << dir*p.y  << " word " << dir*p.x << ' ' << dir*p.y << " H\n";
    LOUT << "SL Hypothese: " << now.diff_msec(main_position.init_time) << '\t' << main_position.winning_coefficient << '\t' << main_position.latest_error << '\t' << dir*p.x << '\t' << dir*p.y << '\t' << (h+(dir>0 ? Angle::zero : Angle::half)).get_deg() << '\n';
#endif
    return true;   // update hat tatsaechlich stattgefunden
  } else
    return false;
}

RobotLocation ErrorMinimiserSL::get (Time t) const throw () {
  RobotLocation rloc;
  main_position.rpos_filter.get (rloc.pos, rloc.heading);
  return odobox.add_movement (rloc, latest_update, t);
}

void ErrorMinimiserSL::reset () throw () {
  AltPos z;
  main_position = z;
  latest_update.update();
  cycles_since_reset=0;
  JWARNING ("SL: manual random reset");
}

void ErrorMinimiserSL::reset (Vec p) throw () {
  AltPos z (p);
  main_position = z;
  latest_update.update();
  cycles_since_reset=0;
  JWARNING ("SL: manual position reset");
}

void ErrorMinimiserSL::reset (Vec p, Angle h) throw () {
  AltPos z (p,h);
  main_position = z;
  latest_update.update();
  cycles_since_reset=0;
  JWARNING ("SL: manual position and heading reset");
}

ErrorMinimiserSL::AltPos::AltPos  () {
  rpos_filter.set (Vec(urandom(-maxx,maxx),urandom(-maxy,maxy)), Angle::zero, Vec(1e10, 1e10), 400);
  winning_coefficient=0;
  latest_error=1e100;
  heading_checked=false;
  ema_right_goal=0.51;
  ema_wrong_goal=0.5;
}

ErrorMinimiserSL::AltPos::AltPos  (Vec p) {
  rpos_filter.set (p, Angle::zero, Vec(500,500), 400);
  winning_coefficient=0;
  latest_error=0;
  heading_checked=false;
  ema_right_goal=0.5;
  ema_wrong_goal=0.5;
}

ErrorMinimiserSL::AltPos::AltPos  (Vec p, Angle h) {
  rpos_filter.set (p, h, Vec(500,500), 0.1);
  winning_coefficient=0;
  latest_error=0;
  heading_checked=true;
  ema_right_goal=0.5;
  ema_wrong_goal=0.5;
}

void ErrorMinimiserSL::remove_occluded_lines (VisibleObjectList& select, const VisibleObjectList& all, const VisibleObjectList& obs, double maxdist) {
  select.objectlist.clear();
  select.timestamp = all.timestamp;
  
  // obstacleray aufbauen
  obstaclerays.assign (360, maxdist*maxdist);
  
  vector<VisibleObject>::const_iterator obsit = obs.objectlist.begin();
  vector<VisibleObject>::const_iterator obsitend = obs.objectlist.end();
  while (obsit!=obsitend) {
    double d2 = (obsit->pos).squared_length()+1e-10;
    Vec ortho = ((1.0/sqrt(d2))*(obsit->pos)).rotate_quarter();
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
      chosemin (obstaclerays [i%360], d2);  // nehme an, die Hindernisse sind nicht sehr breit
    for (int i=index_m2; i<index2; i++)
      chosemin (obstaclerays [i%360], d2);
    obsit++;
  }
  
  // die ersten max_lines filtern
  vector<VisibleObject>::const_iterator linesit = all.objectlist.begin();
  vector<VisibleObject>::const_iterator linesitend = all.objectlist.end();
  while (linesit!=linesitend) {
    int index = static_cast<int>((linesit->pos).angle().get_deg()+0.5) % 360;
    if ((linesit->pos).squared_length()<obstaclerays [index]) {
      select.objectlist.push_back (*linesit);
    }
    linesit++;
  }
}

void ErrorMinimiserSL::remove_lines_outside_field (VisibleObjectList& select, const VisibleObjectList& all, Vec p, Angle h) {
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
