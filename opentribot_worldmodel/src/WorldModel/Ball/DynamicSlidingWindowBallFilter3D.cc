
#include "DynamicSlidingWindowBallFilter3D.h"
#include "../WorldModel.h"
#include <algorithm>
#include <cmath>

using namespace Tribots;
using namespace std;

#define dout(text) // LOUT << text;  // Debug-Ausgabe auf LOUT

namespace {
  const double gravity=5;
  const double ballFlection=0.85;
}

unsigned int LinearBallMotionModel2D::estimate (Time& reftime, Vec& refpos, Vec& refvel, const std::vector<BallObservation>& observations, bool use2d, bool use3d) throw () {
  if (observations.size()<2)
    return 0;  // unterhalb von 2 Beobachtungen erst gar nicht anfangen

  // Referenzzeitpunkt bestimmen:
  reftime=observations[0].time;
  for (unsigned int i=0; i<observations.size(); i++) {
    if (reftime<observations[i].time) {
      reftime=observations[i].time;
    }
  }

  // die Parameter des LGS berechnen:
  double sum_t = 0;  // Summe der Zeitstempel in ms
  double sum_t2 = 0;  // Summe der Zeitstempel^2 in ms^2
  Vec sum_pos (0,0);  // Summe der Positionen in mm
  Vec sum_t_pos (0,0);  // Summe Positionen mal Zeitstempel in ms mal mm
  unsigned int n_eff = 0;  // Anzahl beruechsichtigter Eintraege
  for (unsigned int i=0; i<observations.size(); i++) {
    double delta_t = observations[i].time.diff_msec (reftime);
    if (delta_t>-2000 && ((use2d && !observations[i].true3d) || (use3d && observations[i].true3d))) {
      sum_t+=delta_t;
      sum_t2+=delta_t*delta_t;
      sum_pos+=observations[i].pos.toVec();
      sum_t_pos+=delta_t*observations[i].pos.toVec();
      n_eff++;
    }
  }

  // Das LGS loesen:
  if (n_eff>1.5) {
    const double weight_decay=10000;
    sum_t2+=weight_decay;
    double det = n_eff*sum_t2-sum_t*sum_t;
    if (std::abs(det)<1e-5) {
      // Singulaere Matrix, sollte nur am Anfang auftreten, wenn alle timestamps gleich sind
      return 0;
    } else {
      refpos = (sum_t2*sum_pos-sum_t*sum_t_pos)/det;
      refvel = (-sum_t*sum_pos+static_cast<double>(n_eff)*sum_t_pos)/det;
      return n_eff;
    }
  } else {
    return 0;
  }
}

bool LinearBallMotionModel2D::reestimate (const std::vector<BallObservation>& observations, bool use2d, bool use3d) throw () {
  Time reftime;
  Vec refpos;
  Vec refvel;
  if (estimate (reftime, refpos, refvel, observations, use2d, use3d)<3)
    return false;  // unterhalb von 3 Beobachtungen kein Modell liefern

  referenceTime=reftime;
  referencePosition=refpos;
  referenceVelocity=refvel;
  dout ("% thin solid");
  for (unsigned int i=0; i<observations.size(); i++)
    dout (" color 0 0 " << 255-10*i <<  " cross " << observations[i].pos.toVec());
  dout (" color 100 100 255 Tarrow " << referencePosition << referencePosition+300*referenceVelocity << "\n");
  return true;
}

bool LinearBallMotionModel2D::reestimateEM (const std::vector<BallObservation>& observations, Vec robot) throw () {
  Time reftime;
  Vec refpos;
  Vec refvel;
  if (estimate (reftime, refpos, refvel, observations, false, true)<2) // initiale Schaetzung nur aus 3D-Positionen
    return false;  // 2 3D-Positionen muessen es schon sein, um etwas Vernuenftiges schaetzen zu koennen
  vector<BallObservation> observations2 (observations);
  for (unsigned int k=0; k<4; k++) {  // mehrere Iteration des EM-Algorithmus' durchfuehren
    for (unsigned int i=0; i<observations2.size(); i++) {
      if (!observations2[i].true3d) {
        // 2D-Positionen auf die Position auf dem Sichtstrahl verschieben, die am besten zum Geschwindigkeitsmodell passt
        try{
          Vec positionPredict = refpos+reftime.diff_msec(observations2[i].time)*refvel;
          Line ray (observations2[i].robot, observations2[i].pos.toVec());
          observations2[i].pos=ray.perpendicular_point (positionPredict);
        }catch(invalid_argument&) {;}  // Geometrieausnahme wuerde auftreten, wenn pos=robot
      }
    }
    // Bewegungsmodell erneut schaetzen
    estimate (reftime, refpos, refvel, observations2, true, true);
  }
  referencePosition=refpos;
  referenceVelocity=refvel;
  referenceTime=reftime;
  dout ("% thin solid");
  for (unsigned int i=0; i<observations.size(); i++)
    dout (" color 0 0 " << 255-10*i << " line " << observations[i].pos.toVec() << observations2[i].pos.toVec() << " cross " << observations[i].pos.toVec());
  dout (" color 100 100 255 Tarrow " << referencePosition << referencePosition+300*referenceVelocity << "\n");
  return true;
}


void BallMotionModelZ::predict (Time t) throw () {
  if (t==predictTime)
    return;
  double t1=referenceTime.get_msec();
  double z1=referencePosition;
  double v1=referenceVelocity;
  double dt=0;
  double tt=t.get_msec();
  do {
    double r = v1*v1+2e-3*gravity*z1;
    if (r>=1e-10) {
      dt=1e3*((v1+sqrt(r))/gravity);
      if (dt>(tt-t1)) {
        z1+=v1*(tt-t1)-0.5e-3*gravity*(tt-t1)*(tt-t1);
        v1-=1e-3*gravity*(tt-t1);
        break;
      }
      z1=0;
      t1+=dt;
      v1=-ballFlection*(v1-gravity*1e-3*dt);
    } else {
      z1=0;
      v1=0;
      break;
    }
  }while(true);
  predictPosition=z1;
  predictVelocity=v1;
  predictTime=t;
}

bool BallMotionModelZ::reestimate (const std::vector<BallObservation>& observations) throw () {
  // kleinste Quadrate-Schaetzung fuer lineares Bewegungsmodell
  const unsigned int n = observations.size();
  double sum_ts = 0;     // Summe der Zeitstempel in ms
  double sum_ts2 = 0;    // Summe der Zeitstempel^2 in ms^2
  double sum_pos = 0;     // Summe der gemessenen Positionen in (mm x mm)
  double sum_ts_pos = 0;  // Summe der gemessenen Positionen mal Zeitstempel in ms mal (mm x mm)
  double n_eff=0;      // Die Anzahl beruecksichtigter Eintraege
  unsigned int missing=0;
  bool first=true;
  double zbefore=-99;
  int trend=0;
  Time tref;
  tref.set_msec(0);
  for (unsigned int i=0; i<n; i++) {
    if (observations[i].true3d && observations[i].time>tref) {
      tref=observations[i].time;
    }
  }
  for (unsigned int i=0; i<n & n_eff<=4; i++) {
    const BallObservation& tmp = observations[n-i-1];
    double tau = static_cast<double>(tmp.time.diff_msec (tref));
    if (tau>-2000) {
      if (tmp.true3d) {
        if ((trend>0 && tmp.pos.z<zbefore+20) || (trend<0 && tmp.pos.z>zbefore-20) || (trend==0)) {
          sum_ts+=tau;
          sum_ts2+=tau*tau;
          sum_pos+=tmp.pos.z;
          sum_ts_pos+=tau*tmp.pos.z;
          n_eff++;
          missing=0;
          first=false;
          if (trend==0) {
            if (zbefore!=-99) {
              if (zbefore>tmp.pos.z)
                trend=+1;
              else if (zbefore<tmp.pos.z)
                trend=-1;
            }
          }
          zbefore=tmp.pos.z;
        } else {
          break;  // Wechsel in der Auf-/Abbewegung
        }
      } else {
        missing++;
      }
      if (!first && missing>=2) {
        break;
      }
    } else {
      break;
    }
  }
  if (n_eff>1.5) {  // n_eff ist auf jeden Fall >= 1
    // LGS loesen
    const double weight_decay=0;
    sum_ts2+=weight_decay;
    double det = n_eff*sum_ts2-sum_ts*sum_ts;
    if (std::fabs(det)<1e-5) {
      // Singulaere Matrix, setze Default-Werte
      // sollte nur am Anfang auftreten, wenn alle timestamps gleich sind
      return false;
    } else {
      referencePosition = (sum_ts2*sum_pos-sum_ts*sum_ts_pos)/det;
      referenceVelocity = (-sum_ts*sum_pos+static_cast<double>(n_eff)*sum_ts_pos)/det;
      referenceTime = tref;
      return true;
    }
  } else {
    return false;
  }
}

Time BallMotionModelZ::getTimeTouchingGround () const throw () {
  if (referencePosition<=0)
    return referenceTime;
  double dt = (referenceVelocity+sqrt(referenceVelocity*referenceVelocity+2e-3*gravity*referencePosition))/gravity;
  Time dest (referenceTime);
  dest.add_msec (static_cast<long int>(dt*1000));
  return dest;
}

double BallMotionModelZ::getHighestPoint () const throw () { 
  if (referenceVelocity<0) 
    return referencePosition;
  return referencePosition-0.5e3*referenceVelocity*referenceVelocity/gravity;
}


Tribots::DynamicSlidingWindowBallFilter3D::DynamicSlidingWindowBallFilter3D (const ConfigReader& reader) throw (std::bad_alloc) : observations (30) {
  rescaleRingBuffer(0);

  vector<unsigned int> vals;
  if (reader.get ("BallFilter::history_length", vals)>=2) {
    maxBufferSize=vals[1];
    minBufferSize=vals[0];
  } else {
    maxBufferSize=10;
    minBufferSize=3;
  }
  if (minBufferSize<2)
    minBufferSize=2;
  if (maxBufferSize<minBufferSize)
    maxBufferSize=minBufferSize;
  if (!reader.get ("BallFilter::max_error", maxError))
    maxError=500;

  if (reader.get ("BallFilter::raised_threshold", vals)>=2) {
    raised.setHysteresis (vals[0], vals[1]);
  }

  // vernuenftige Anfangswerte setzen:
  largeErrorObserved=false;
  lowBall=true;
}

Tribots::DynamicSlidingWindowBallFilter3D::~DynamicSlidingWindowBallFilter3D () throw () {;}

bool Tribots::DynamicSlidingWindowBallFilter3D::update (const VisibleObjectList& vis, const RobotLocation& cr) throw () {
  if (vis.objectlist.size()>0) {
    Vec p = cr.pos+vis.objectlist[0].pos.rotate(cr.heading);
    Vec3D p3 (p.x,p.y,-10000);
    if (vis.objectlist[0].object_type==VisibleObject::ball3d)
      p3.z=vis.objectlist[0].z;
    update (p3, vis.timestamp, cr.pos);
    return true;
  }
  return false;
}

void Tribots::DynamicSlidingWindowBallFilter3D::addToRingbuffer (const BallObservation& obs) {
  // pruefen, ob alte Informationen ueberhaupt beruecksichtigt werden sollen
  if (latestObservation.time.diff_msec(observations.get(-1).time)>1000) { // juengste Informationen aelter als eine Sekunde -> loeschen
    rescaleRingBuffer (0);
  }

  // einfuegen oder aeltesten Eintrag ueberschreiben
  if (observations.size()<maxBufferSize) {
    try{
      observations.insert (latestObservation);
    }catch(std::bad_alloc&){
      observations.set (latestObservation);  // sollte eigentlich nicht auftreten
    }
  } else {
    observations.set (latestObservation);
  }
  observations.step();  // einen Eintrag weitergehen
}

void Tribots::DynamicSlidingWindowBallFilter3D::update (const Vec3D p1, const Time t1, const Vec robot) throw () {
  dout ("Ballfilter: Ball gesehen: " << t1 << ' ' << p1 << '\n');
  if  (!insideField (p1.toVec())) {
    // Beobachtungen ausserhalb ignorieren und raised setzen
    dout ("Ballfilter: Ball auserhalb gesehen\n");
    raised.set (true);
    rescaleRingBuffer (0);
    largeErrorObserved=false;
    return;
  }

  // latestObservation setzen:
  latestObservation.pos=p1;
  latestObservation.time=t1;
  latestObservation.true3d=(p1.z>-100);
  latestObservation.robot=robot;
  if (!latestObservation.true3d)
    latestObservation.pos.z=0;

  // Beobachtung in Ringpuffer einfuegen:
  addToRingbuffer (latestObservation);
  dout("Ballfilter: Ringspeicher:");
  for (unsigned int i=0; i<observations.size(); i++)
    dout((observations.get(i).true3d ? (observations.get(i).pos.z<200 ? " 3-" : " 3+") : " 2"));
  dout('\n');

  // die Prognoseguete des aktuellen 2D-Bewegungsmodell auf neuer Beobachtung bestimmen:
  Vec predictPos = ballMotion2d.getPosition (latestObservation.time);
  double error = (predictPos-latestObservation.pos.toVec()).length();
  if (!lowBall && !latestObservation.true3d) {
    try{
      // wenn 3D-Modell, aber 2D-Beobachtung, den Trick mit Verschieben der Beobachtung auf dem Sichtstrahl durchfuehren
      Line ray (latestObservation.robot, latestObservation.pos.toVec());
      error = (ray.perpendicular_point (predictPos)-predictPos).length();
    }catch(invalid_argument&) {;}  // Geometrieproblem wenn latestObservation.pos=robot; ignorieren
  }
  double maxErrorAllowed = min (2500.0, maxError*(0.5+sqrt((robot-predictPos).length()+100)/100)+(lowBall ? 0 : 300));  // der maximal erlaubte Fehler soll von Ballabstand abhaengen
  dout ("Ballfilter: Vorhersagefehler: " << error << " <? " << maxErrorAllowed << "; war: " << largeErrorObserved << ' ' << (lowBall ? "low_ball" : "high_ball") << '\n');
  bool largeError = (error>maxErrorAllowed);
  bool modelDoesNotFitReality = largeError && largeErrorObserved;
  largeErrorObserved = largeError;

  if (modelDoesNotFitReality && observations.size()>minBufferSize) {
    // wenn Modell nicht mehr passt, die aeltesten Beobachtungen entfernen
    rescaleRingBuffer (minBufferSize-1);
    largeErrorObserved=false;
    lowBall=false;
    dout ("Ballfilter: Ringpuffer loeschen wegen zu grossem Fehler\n");
  }

  // Bewegungsmodelle neu schaetzen:
  vector<BallObservation> observationsArray;
  observations.toVector(observationsArray);
  bool success3d = false;  // Erfolg bei Z-Schaetzung
  bool success2d = false;  // Erfolg bei XY-Schaetzung

  // Z-Modell schaetzen
  success3d = ballMotionZ.reestimate (observationsArray);
  lowBall = !success3d;

  // und in Abhaengigkeit dessen das XY-Modell schaetzen
  double zmax=0;
  for (unsigned int i=0; i<observationsArray.size(); i++) {
    if (observationsArray[i].pos.z>zmax)
      zmax=observationsArray[i].pos.z;
  }
  if (success3d && ballMotionZ.getTimeTouchingGround().diff_msec(latestObservation.time)>-70) {
    if (zmax>200) {
      dout ("Ballfilter: 3D-EM-Schaetzung\n");
      success2d = ballMotion2d.reestimateEM (observationsArray, robot);
      if (!success2d)
        dout ("Ballfilter: Fehler bei Schaetzung 2D, aber 3D richtig geschaetzt\n");
    } else {
      dout ("Ballfilter: 2D-Schaetzung gemischt\n");
      success2d = ballMotion2d.reestimate (observationsArray, true, true);
      if (!success2d)
        dout ("Ballfilter: Kein Bewegungsmodell schaetzbar\n");
    }
  } else {
    dout ("Ballfilter: 2D-Schaetzung\n");
    success2d = ballMotion2d.reestimate (observationsArray, true, false);
    if (!success2d)
      dout ("Ballfilter: Kein Bewegungsmodell schaetzbar\n");
  }
  if (success2d && (robot-ballMotion2d.getReferencePosition()).length()<1000 && (robot-latestObservation.pos.toVec()).length()<1000) {
    dout ("Ballfilter: Ball in Roboternaehe, verschiebe Referenzposition " << '\n');
    ballMotion2d.setReferencePosition (latestObservation.pos.toVec());  // wenn Ball nahe Roboter, der Bildverarbeitung voll vertrauen wegen Dribbling
  }

  // Raised aktualisieren
  raised.update (latestObservation.pos.toVec(), ballMotion2d.getReferenceVelocity().length());  // raised aktualisieren
  if (raised.raised()) {
    rescaleRingBuffer (minBufferSize-1);
  }
  if (latestObservation.true3d)
    raised.set(false);  // eine echte 3D-Position setzt raised sofort zurueck

}  // Ende DynamicSlidingWindowBallFilter3D::update()

BallLocation Tribots::DynamicSlidingWindowBallFilter3D::get (const Time t) const throw () {
  BallLocation dest;
  dest.pos=latestObservation.pos;
  dest.velocity=Vec3D(0,0,0);
  dest.velocity_known=false;
  dest.lastly_seen=latestObservation.time;
  dest.pos_known=BallLocation::unknown;

  bool validLatestObservation = (t.diff_msec(latestObservation.time)<1000);
  bool validMotionModel2d = (validLatestObservation && (ballMotion2d.getReferenceTime()==latestObservation.time));
  bool validMotionModel3d =(validMotionModel2d && (ballMotionZ.getTimeTouchingGround ().diff_msec(latestObservation.time)>-70));
  bool validComm = (t.diff_msec (ballCommunicated.time)<1000);

  if (raised.raised() && validLatestObservation) {
    dest.pos.x=raised.raisedPosition().x;
    dest.pos.y=raised.raisedPosition().y;
    dest.pos.z=1000;
    dest.velocity=Vec3D(0,0,0);
    dest.pos_known=BallLocation::raised;
    dest.velocity_known=false;
    dout ("Ballfilter: get raised " << t.get_msec() << ' ' << dest.pos << "\n");
  } else if (validMotionModel3d) {
    dest.pos.x=ballMotion2d.getPosition(t).x;
    dest.pos.y=ballMotion2d.getPosition(t).y;
    dest.pos.z=ballMotionZ.getPosition(t);
    dest.velocity.x=ballMotion2d.getVelocity(t).x;
    dest.velocity.y=ballMotion2d.getVelocity(t).y;
    dest.velocity.z=ballMotionZ.getVelocity(t);
    dest.pos_known=BallLocation::known;
    dest.velocity_known=true;
    dout ("Ballfilter: get model3d " << t.get_msec() << ' ' << dest.pos << "\n");
  } else if (validMotionModel2d) {
    dest.pos.x=ballMotion2d.getPosition(t).x;
    dest.pos.y=ballMotion2d.getPosition(t).y;
    dest.pos.z=0;
    dest.velocity.x=ballMotion2d.getVelocity(t).x;
    dest.velocity.y=ballMotion2d.getVelocity(t).y;
    dest.velocity.z=0;
    dest.pos_known=BallLocation::known;
    dest.velocity_known=true;
    dout ("Ballfilter: get model2d " << t.get_msec() << ' ' << dest.pos << "\n");
  } else if (validLatestObservation) {
    dest.pos=latestObservation.pos;
    dest.velocity=Vec3D(0,0,0);
    dest.pos_known=BallLocation::known;
    dest.velocity_known=false;
    dout ("Ballfilter: get latest " << t.get_msec() << ' ' << dest.pos << "\n");
  } else if (validComm) {
    dest.pos=ballCommunicated.pos;
    dest.velocity=Vec3D(0,0,0);
    dest.pos_known=BallLocation::communicated;
    dest.velocity_known=false;
    dout ("Ballfilter: get comm " << t.get_msec() << ' ' << dest.pos << "\n");
  } else {
    dout ("Ballfilter: get unknown " << t.get_msec() << ' ' << dest.pos << "\n");
  }

  return dest;
}  // DynamicSlidingWindowBallFilter3D::Ende get()

void DynamicSlidingWindowBallFilter3D::rescaleRingBuffer (unsigned int n) throw () {
  while (observations.size()>n)
    observations.erase();
}

bool DynamicSlidingWindowBallFilter3D::insideField (Vec p) const throw () {
  const FieldGeometry& fg (WorldModel::get_main_world_model().get_field_geometry());
  if (abs(p.x)>0.5*fg.field_width+fg.side_band_width+400)   // Sicherheitsmarge 400 mm
    return false;
  if (abs(p.y)>0.5*fg.field_length+fg.goal_band_width+400)     // Sicherheitsmarge 400 mm
    return false;
  return true;
}

void DynamicSlidingWindowBallFilter3D::comm_ball (Vec p) throw () {
  ballCommunicated.pos = MWM.get_own_half()*p;
  ballCommunicated.time.update();
}
