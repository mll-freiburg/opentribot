
#include "DynamicSlidingWindowBallFilter.h"
#include "../WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

#define DEBUG_BALLFILTER 0   // Debug-Info nach LOUT schreiben

Tribots::DynamicSlidingWindowBallFilter::DynamicSlidingWindowBallFilter (const ConfigReader& reader) throw (std::bad_alloc) : pos_list (30), pos_for_raised (6) {
  rescale_ring_buffer(1);

  vector<unsigned int> vals;
  if (reader.get ("BallFilter::history_length", vals)>=2) {
    max_buffer_size=vals[1];
    min_buffer_size=vals[0];
  } else {
    max_buffer_size=10;
    min_buffer_size=3;
  }
  if (min_buffer_size<2)
    min_buffer_size=2;
  if (max_buffer_size<min_buffer_size)
    max_buffer_size=min_buffer_size;
  if (!reader.get ("BallFilter::max_error", max_error))
    max_error=500;
  if (reader.get ("BallFilter::raised_threshold", vals)>=2) {
    raised_hysterese_lower2 = vals[0]*vals[0];
    raised_hysterese_upper2 = vals[1]*vals[1];
  } else {
    raised_hysterese_lower2 = 49;
    raised_hysterese_upper2 = 100;
  }
  
  // vernuenftige Anfangswerte setzen:
  first_call=true;
  ref_bpos=Vec::zero_vector;
  ref_bvel=Vec::zero_vector;
  ball_raised=false;
}

Tribots::DynamicSlidingWindowBallFilter::~DynamicSlidingWindowBallFilter () throw () {;}

bool Tribots::DynamicSlidingWindowBallFilter::update (const VisibleObjectList& vis, const RobotLocation& cr) throw () {
  if (vis.objectlist.size()>0) {
    Vec p = cr.pos+vis.objectlist[0].pos.rotate(cr.heading);
    update (p, vis.timestamp, cr.pos);
    return true;
  }
  return false;
}

void Tribots::DynamicSlidingWindowBallFilter::update (const Vec p, const Time t, const Vec r) throw () {
#if DEBUG_BALLFILTER
  LOUT << "Ball gesehen an Position " << p << '\n';
#endif
  if  (!inside_field (p)) {
#if DEBUG_BALLFILTER
    LOUT << "Ballfilter: Ball auserhalb gesehen\n";
#endif
    ball_raised=true;
    return;
  }
  // neue Messung in Ringpuffer einfuegen
  PairTimePos newp;
  newp.pos=p;
  newp.timestamp=t;

  pos_for_raised.get()=newp;
  pos_for_raised.step();

  // pruefen, ob alte Informationen ueberhaupt beruecksichtigt werden sollen
  if (t.diff_msec(ref_time)>1000) { // juengste Informationen aelter als eine Sekunde -> loeschen
    rescale_ring_buffer (1);
    first_call=true;
  }

  if (first_call) {
    // alle Eintrage der Liste auf den neuen Wert setzen -> vernuenftige Initialisierung
    for (unsigned int i=0; i<pos_list.size(); i++)
      pos_list.set (newp, i);
    first_call=false;
  } else {
    // einfuegen oder aeltesten Eintrag ueberschreiben
    if (pos_list.size()<max_buffer_size) 
      try{
      pos_list.insert (newp);
      }catch(std::bad_alloc&){
        pos_list.set (newp);
      }
    else
      pos_list.set (newp);
    pos_list.step();  // einen Eintrag weitergehen
  }


  // Zwischenstand hier: 
  //   -- die Ringpuffereintraege sind aktualisiert
  //   -- das Prognosemodell ist noch das alte

    
  // pruefen, ob das Modell noch passt
  Vec p_predict = ref_bpos+t.diff_msec(ref_time)*ref_bvel;
  double error = (p_predict-p).length();
  double max_allowed_error = max_error*(0.5+sqrt((r-p_predict).length()+100)/100);   // der maximal erlaubte Fehler soll von Ballabstand abhaengen

#if DEBUG_BALLFILTER
  LOUT << "BallFilter error: " << error << ' ' << latest_error << ' ' << max_allowed_error << '\n';
#endif

  // wenn Modell nicht mehr passt, die aeltesten Beobachtungen entfernen
  if ((error>max_allowed_error && latest_error>max_allowed_error) || (error>2500)) {
    rescale_ring_buffer (min_buffer_size);
    latest_error = 0;  // um einen erneuten Reset zu unterbinden
  } else
    latest_error = error;
    
  // neue Referenz-Roboterposition und -zeitpunkt setzen
  Time old_time = ref_time;
  Vec old_bpos = ref_bpos;
  ref_time=t;

  bool successfull_update = update_motion_model ();    // Bewegungsmodell neu berechnen, berechnet ref_bpos und ref_bvel
  if (successfull_update && (r-ref_bpos).squared_length()<1e6)
    ref_bpos=p;        // wenn Ball nahe Roboter, der Bildverarbeitung voll vertrauen wegen Dribbling
#if DEBUG_BALLFILTER
  LOUT << "Ballfilter LS |v|: " << ref_bvel.length() << '\n';
#endif
  if (successfull_update) {     // wenn die Berechnung numerisch moeglich war
    if (ref_bvel.squared_length()<raised_hysterese_upper2) {   // Plausibilitaetscheck
      if (pos_list.size()>=min_buffer_size) {
        if (!ball_raised)
          latest_motion_model = ref_time;     // Modell zaehlt nur dann als Bewegungsmodell, wenn die Geschwindigkeit aus genuegend Beobachtungen gelernt wurde
        else
          if (ref_bvel.squared_length()<raised_hysterese_lower2) {  // Ball soll erst wieder als nicht hochgehalten gelten, wenn Geschwindigkeit < 7m/s
          latest_motion_model = ref_time;
          ball_raised=false;
          } else {
            rescale_ring_buffer (min_buffer_size);
            ref_bvel=Vec::zero_vector;
          }
      } else {
        ref_bvel=Vec::zero_vector;   // bei zu wenigen Bewegungen setze die Geschwindigkeit besser auf 0; ball_raised Attribut beibehalten
      }
    } else {
      // Puffer bis auf letztes Element verkleinern, da irrefuehrende Beobachtungen vorliegen
      rescale_ring_buffer (1);
      first_call=true;
      ball_raised = true;  // Ball wird hochgehoben, wenn er mit groser Geschwindigkeit vom Roboter weggeht
      ref_bvel=Vec::zero_vector;
      ref_bpos=old_bpos;
      ref_time=old_time;
#if DEBUG_BALLFILTER
      LOUT << "BallFilter Plausibilitaetscheck: massive Probleme v>10m/s\n";
#endif
    }
  } else {
    // !successfull_update
    ref_bpos=p;
    ref_bvel=Vec::zero_vector;
  }
  
#if DEBUG_BALLFILTER
  LOUT << "BallFilter length " << pos_list.size() << '\n';
  Time now;
  unsigned int nn=pos_list.size();
  for (unsigned int i=0; i<nn; i++)
    LOUT << "\% blue solid cross " << pos_list.get(i).pos.x << ' ' << pos_list.get(i).pos.y << " word " 
        << pos_list.get(i).pos.x << ' ' << pos_list.get(i).pos.y << ' ' << i+1 << '\n';
#endif
}

BallLocation Tribots::DynamicSlidingWindowBallFilter::get (const Time t) const throw () {
  // einfaches lineares Bewegungsmodell ohne Beschleunigung/Bremsung
  BallLocation dest;
  double td = static_cast<double>(t.diff_msec (ref_time));
  dest.pos=ref_bpos+td*ref_bvel;
  dest.velocity=ref_bvel;
  dest.velocity_known=true;
//  double n = static_cast<double>(pos_list.size());
//  double f = (n>9.5 ? 1.0 : 1.0-0.0046875*(n-10.0)*(n-10.0));
//  dest.quality = f*2000.0/(2000.0+std::fabs(td));  // eine plausible Zuverlaessigkeitskurve
  dest.lastly_seen = ref_time;
  dest.pos_known = (t.diff_msec (latest_motion_model)<2000 ?  (ball_raised ? BallLocation::raised : BallLocation::known) : BallLocation::unknown);
  if (dest.pos_known==BallLocation::unknown || (dest.pos_known==BallLocation::raised && t.diff_msec (latest_motion_model)>=1000))
    if (t.diff_msec (comm_ball_pos_time)<1000) {
      dest.pos=comm_ball_pos;
      dest.velocity=Vec::zero_vector;
      dest.velocity_known=false;
      dest.pos_known=BallLocation::communicated;
    }
  if (dest.pos_known==BallLocation::raised || dest.pos_known==BallLocation::unknown) {
    dest.pos = pos_for_raised.get().pos;
    dest.velocity = Vec::zero_vector;
    dest.velocity_known=false;
  }
  return dest;
}

bool Tribots::DynamicSlidingWindowBallFilter::update_motion_model () throw () {
  // kleinste Quadrate-Schaetzung fuer lineares Bewegungsmodell; Annahme: ref_time bereits gesetzt
  const unsigned int n = pos_list.size();
  double sum_ts = 0;     // Summe der Zeitstempel in ms
  double sum_ts2 = 0;    // Summe der Zeitstempel^2 in ms^2
  Vec sum_pos (0,0);     // Summe der gemessenen Positionen in (mm x mm)
  Vec sum_ts_pos (0,0);  // Summe der gemessenen Positionen mal Zeitstempel in ms mal (mm x mm)
  double n_eff=0;      // Die Anzahl beruecksichtigter Eintraege
  for (unsigned int i=0; i<n; i++) {
    const PairTimePos& tmp = pos_list.get();
    pos_list.step();
    double tau = static_cast<double>(tmp.timestamp.diff_msec (ref_time));
    if (tau>-2000) {
      Vec x = tmp.pos;
      sum_ts+=tau;
      sum_ts2+=tau*tau;
      sum_pos+=x;
      sum_ts_pos+=tau*x;
      n_eff++;
    }
  }
  if (n_eff>1.5) {  // n_eff ist auf jeden Fall >= 1
    // LGS loesen
    const double weight_decay=10000;
    sum_ts2+=weight_decay;
    double det = n_eff*sum_ts2-sum_ts*sum_ts;
    if (std::fabs(det)<1e-5) {
      // Singulaere Matrix, setze Default-Werte
      // sollte nur am Anfang auftreten, wenn alle timestamps gleich sind
      ref_bpos = sum_pos/n_eff;
      ref_bvel = Vec::zero_vector;
      return false;
    } else {
      ref_bpos = (sum_ts2*sum_pos-sum_ts*sum_ts_pos)/det;
      ref_bvel = (-sum_ts*sum_pos+static_cast<double>(n_eff)*sum_ts_pos)/det;
      return true;
    }
  } else {
    return false;
  }
}

void DynamicSlidingWindowBallFilter::rescale_ring_buffer (unsigned int n) throw () {
  while (pos_list.size()>n)
    pos_list.erase();
}

bool DynamicSlidingWindowBallFilter::inside_field (Vec p) const throw () {
  const FieldGeometry& fg (WorldModel::get_main_world_model().get_field_geometry());
  if (abs(p.x)>0.5*fg.field_width+fg.side_band_width+400)   // Sicherheitsmarge 400 mm
    return false;
  if (abs(p.y)>0.5*fg.field_length+fg.goal_band_width+400)     // Sicherheitsmarge 400 mm
    return false;
  return true;
}

void DynamicSlidingWindowBallFilter::comm_ball (Vec p) throw () {
  comm_ball_pos = MWM.get_own_half()*p;
  comm_ball_pos_time.update();
}
