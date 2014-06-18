
#include "OdometryContainer.h"
#include "../Prediction/update_robot_location.h"
#include <cmath>

using namespace std;
using namespace Tribots;

namespace {

  const int min_time_odo = 70;

  // Ungenuegende Heuristiken, um die Ungenauigkeiten in der Roboteransteuerung zu modellieren
  // Soll ueber kurz oder lang entbehrlich werden, wenn bessere, geschaetzte Geschwindigkeiten vorliegen
  inline double rectified_velocity (const double& vrot) {
    return (abs(vrot)>1.2 ? 0.8 : (abs(vrot)>0.7 ? 0.9 : 1.0))*vrot;
  }

  inline Vec rectified_velocity (const Vec& vtrans) {
    return (vtrans.length()>1.2 ? 0.7 : (vtrans.length()>0.7 ? 0.8 : 1.0))*vtrans;
  }
  inline double nulltol (const double& d) {
    return (d>0.2 ? d : (d<-0.2 ? d : 0));
  }

  // Klasse, um innerhalb eines Ringpuffers fuer GyroDaten den zeitlich passenden Eintrag zu finden und in einem RobotLocation vrot zu ersetzen
  // Annahme: Anker des Ringpuffers zeigt stets auf aeltesten Eintrag
   class GyroSearcher {
   private:
     const Tribots::RingBuffer<OdometryContainer::TimestampGyroData>& buffer;
     const unsigned int bufsize;
     unsigned int index;
     unsigned int index2;
     double offset;
   public:
     GyroSearcher (const Tribots::RingBuffer<OdometryContainer::TimestampGyroData>& buffer1, double off) : buffer(buffer1), bufsize (buffer1.size()), index(0), index2(0), offset(off) {;}
     void replace (RobotLocation& rloc, const Time& t, int td = 50) { // td: Toleranz in ms bezueglich der Zeitstempel
      double vrot;
      if (find (vrot, t, td, index))
        rloc.vrot=vrot;
    } // Methode replace
     void replace (DriveVector& dv, const Time& t, int td = 50) { // td: Toleranz in ms bezueglich der Zeitstempel
      double vrot;
      if (find (vrot, t, td, index))
        dv.vrot=vrot;
    } // Methode replace
    bool integrate (Angle& ret, const Time& t1, const Time& t2, int td = 50) {
      double vrot=0;
      Time t=t1;
      double ang=0;
      bool succ=find(vrot, t1, td, index2);
      if (succ) {
        vrot = nulltol(buffer[index2].gd.vrot-offset);
        while (index2+1<bufsize && buffer[index2+1].timestamp<=t2) {
          index2++;
          double dt = buffer[index2].timestamp.diff_msec(t);
          double vr2 = nulltol(buffer[index2].gd.vrot-offset);
          ang+=0.0005*dt*(vrot+vr2);
          vrot=vr2;
          t=buffer[index2].timestamp;
        }
        if (t2.diff_msec(t)<=td) {
          ang+=0.001*t2.diff_msec(t)*vrot;
          ret.set_rad (ang);
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    } // Methode integrate
    bool find (double& vrot, const Time& t, int td, unsigned int& idx) {
      if (buffer[idx].timestamp>t)
        idx=0;
      if (idx==0 && buffer.get().timestamp>t) {
        // Zeitpunkt angefragt, der vor den zur Verfuegung stehenden Werten liegt
        if (buffer.get().timestamp.diff_msec(t)<td) {
          vrot=buffer.get().gd.vrot;
          return true;
        } else {
          return false;
        }
      }
      while (idx+1<bufsize && buffer[idx+1].timestamp<=t)
        idx++;
      // Index zeigt jetzt auf den letzten Eintrag vor dem Zeitpunkt t
      if (idx+1==bufsize) {
        if (t.diff_msec(buffer[idx].timestamp)<td) {
          vrot=buffer[idx].gd.vrot;
          return true;
        } else {
          return false;
        }
      }
      vrot=nulltol(static_cast<double>(t.diff_msec(buffer[idx].timestamp))/static_cast<double>(buffer[idx+1].timestamp.diff_msec(buffer[idx].timestamp))*(buffer[idx+1].gd.vrot-buffer[idx].gd.vrot)+buffer[idx].gd.vrot-offset); // linear Interpolation
      return true;
    } // Methode find
   }; // Klasse GyroSearcher

   void heun_integration_step (RobotLocation& start, const DriveVector& dv1, const DriveVector& dv2, Time t1, Time t2, GyroSearcher& gs, bool use_gyro) {
     // einen Heun-Integrationsschritt durchfuehren:
     // rloc (Argument und Rueckgabe): Roboterposition zu Beginn und Ende des Integrationsschritts
     // dv1: Gescheindigkeit zu Beginn
     // dv2: Geschwindigkeit am Ende
     // t1: Anfangszeitpunkt
     // t2: Endzeitpunkt
     // gs: Gyrosearcher
     // use_gyro: Gyroskopinformationen nutzen?
     double dt = t2.diff_msec(t1);
     start.vtrans = dv1.vtrans.rotate (start.heading);
     start.vrot = dv1.vrot;
     if (use_gyro)
       gs.replace (start, t1);
     start.kick = dv1.kick;

     DriveVector dv2x = dv2;
     if (use_gyro)
       gs.replace (dv2x, t2);
     Angle deltah;
     bool gyro_int = gs.integrate (deltah, t1, t2);
     Angle h2=start.heading+deltah;

     start = update_robot_location (start, dt, dv2x);
     if (use_gyro && gyro_int)
       start.heading=h2;  // ggf. Ausrichtung durch gyro-integrierte Ausrichtung ersetzen
   }

} // namespace



// Konvention fuer Stellung des Ankers in odo und drv: der Anker steht stets auf dem aeltesten Eintrag;
// In aufsteigender Reihenfolge werden die Eintraege immer aktueller

OdometryContainer::OdometryContainer (unsigned int s, double mta, double mra) throw (std::bad_alloc) : odo(s), drv(s), gyro(3*s), n(s), max_acc (mta), max_racc (mra), gyro_offset_vrot(0), gyro_offset_vrot_candidates (9), gyro_offset_count(0) {
  for (unsigned int i=0; i<n; i++) {
    odo[i].dv.vtrans=drv[i].dv.vtrans=Vec::zero_vector;
    odo[i].dv.vrot=drv[i].dv.vrot=0;
    odo[i].dv.kick=drv[i].dv.kick=false;
    odo[i].timestamp.set_msec(0);
    drv[i].timestamp.set_msec(0);
  }
}

OdometryContainer::~OdometryContainer () throw () {;}

void OdometryContainer::add_odometry (DriveVector dv, Time t) throw () {
  Time latestOdoTime = odo[-1].timestamp;
  odo.get().timestamp=t;
  odo.get().dv = dv;
  odo.step();
}

void OdometryContainer::add_gyro_data (GyroData gd, Time t) throw () {
  gyro.get().timestamp=t;
  gyro.get().gd = gd;
  gyro.step();

  // gyro_offset anpassen, wenn Roboter steht:
  DriveVector odo = get_odometry(t);
  DriveVector dv = get_drive_vector(t);  
  bool gyro_vrot_null=(abs(gd.vrot-gyro_offset_vrot)<0.3);
  bool dv_vrot_null=(abs(dv.vrot)<1e-10);
  bool odo_vrot_null=(abs(odo.vrot)<0.1);
  bool dv_vtrans_null=(dv.vtrans.length()<1e-10);
  bool odo_vtrans_null=(odo.vtrans.length()<0.1);
  if (gyro_vrot_null && dv_vrot_null && odo_vrot_null && dv_vtrans_null && odo_vtrans_null) {
    if (gyro_offset_count++>9) {
      gyro_offset_vrot = gyro_offset_vrot_candidates[0];
      for (unsigned int i=1; i<gyro_offset_vrot_candidates.size(); i++)
        gyro_offset_vrot_candidates[i-1]=gyro_offset_vrot_candidates[i];
      gyro_offset_vrot_candidates[gyro_offset_vrot_candidates.size()-1]=0.95*gyro_offset_vrot_candidates[gyro_offset_vrot_candidates.size()-1]+0.05*gd.vrot;
    }
  } else {
    gyro_offset_count=0;
    for (unsigned int i=0; i<gyro_offset_vrot_candidates.size(); i++)
      gyro_offset_vrot_candidates[i]=gyro_offset_vrot;
  }
}

void OdometryContainer::add_drive_vector (DriveVector dv, Time t) throw () {
  if (dv.vtrans.length()>3.0) {
    dv.vtrans = 3.0*dv.vtrans.normalize();
  }
  
  const OdometryContainer::TimestampDriveVector& latest (drv[-1]);
  double dt = t.diff_msec (latest.timestamp);
  double dvtrans = (latest.dv.vtrans-dv.vtrans).length();
  double dvrot = abs(latest.dv.vrot-dv.vrot);
  
  // evtl. unmoegliche Veraenderungen der DriveVectoren korrigieren
  // kann ueber kurz oder lang verschwinden, wenn eine vernuenftige Geschwindigkeits-Vorhersage existiert
  double mt = max_acc*dt*1e-3;
  if (mt<1e-3) mt=1e-3;
  if (dvtrans>mt)
    dv.vtrans = (1-mt/dvtrans)*latest.dv.vtrans+(mt/dvtrans)*dv.vtrans;
  mt = max_racc*dt*1e-3;
  if (mt<1e-3) mt=1e-3;
  if (dvrot>mt)
    dv.vrot = (1-mt/dvrot)*latest.dv.vrot+(mt/dvrot)*dv.vrot;

  drv.get().timestamp=t;
  drv.get().dv=dv;
  drv.step();
}

RobotLocation OdometryContainer::movement (Time t1, Time t2, unsigned int mode) const throw () {
  RobotLocation start;
  start.pos=Vec::zero_vector;
  start.heading=Angle::zero;
  start.vtrans=Vec::zero_vector;
  start.vrot=0;
  start.kick=false;
  start.stuck.robot_stuck=false;
  start.stuck.msec_since_stuck=0;
  return add_movement (start, t1, t2, mode);
}

RobotLocation OdometryContainer::add_movement (RobotLocation start, Time t1, Time t2, unsigned int mode) const throw () {
  GyroSearcher gs (gyro, gyro_offset_vrot);
  if (t1>=t2) {
    // nur Geschwindigkeiten zum Zeitpunkt t2 und kick setzen
    // zuerst in pred nach passenden Eintraegen suchen:
    unsigned int predindex = 0;
    while ( predindex+1 < pred.size() ) {
      if (pred[predindex+1].timestamp<=t2)
        predindex++;
      else
        break;
    }
    if (mode<=0) {
      if (predindex+1<pred.size() || (predindex<pred.size() && t2.diff_msec (pred[predindex].timestamp)<min_time_odo)) {
        start.kick = pred[predindex].dv.kick;
        start.vrot = pred[predindex].dv.vrot;
        gs.replace (start, t2);
        start.vtrans = pred[predindex].dv.vtrans.rotate (start.heading);
        return start;
      }
    }
    if (mode<=1) {
      // wenn in pred nichts vernuenftiges gefunden wurde, in odo nachsehen
      unsigned int odoindex = 0;
      while (odoindex+1<n) {
        if (odo[odoindex+1].timestamp<=t2)
          odoindex++;
        else
          break;
      }
      if (odoindex+1<n || t2.diff_msec (odo[odoindex].timestamp)<min_time_odo) {
        start.kick = odo[odoindex].dv.kick;
        start.vrot = odo[odoindex].dv.vrot;
        gs.replace (start, t2);
        start.vtrans = odo[odoindex].dv.vtrans.rotate (start.heading);
        return start;
      }
    }
    // wenn in odo nichts vernuenftiges gefunden wurde, in drv nachsehen
    unsigned int drvindex = 0;
    while (drvindex+1<n)
      if (drv[drvindex+1].timestamp<=t2)
        drvindex++;
      else
        break;
    start.kick = drv[drvindex].dv.kick;
    start.vrot = drv[drvindex].dv.vrot;
    if (mode<=1)
      gs.replace (start, t2);
    start.vtrans = drv[drvindex].dv.vtrans.rotate (start.heading);
    return start;
  }


  // eigentlich Interessanter Fall: t1<t2
  // zunaechst pred, dann odo, dann drv durchgehen und aufintegrieren
  TimestampDriveVector cdv;
  Time tstart = t1;
  long int dt;

  // zunaechst nach einem Anfangspunkt in pred suchen:
  unsigned int predindex = 0;
  if (mode>=1) {
    predindex=pred.size();
  } else {
    while (predindex+1<pred.size()) {
      if (pred[predindex+1].timestamp<=t1)
        predindex++;
      else
        break;
    }
  }
  if (predindex<pred.size())
    cdv = pred[predindex];
  if (! (predindex+1<pred.size() || (predindex<pred.size() && t1.diff_msec (pred[predindex].timestamp)<min_time_odo))) {
    // kein passender Anfangspunkt in pred gefunden, also in odo nachschauen:
    unsigned int odoindex = 0;
    if (mode>=2) {
      predindex=pred.size();
    } else {
      while (odoindex+1<n) {
        if (odo[odoindex+1].timestamp<=t1)
          odoindex++;
        else
          break;
      }
    }
    if (odoindex+1<n || t1.diff_msec (odo[odoindex].timestamp)<min_time_odo) {
      cdv = odo[odoindex];
    } else {
      // kein passender Anfangspunkt in pred und odo gefunden, also in drv nachschauen:
      unsigned int drvindex = 0;
      while (drvindex+1<n) {
        if (drv[drvindex+1].timestamp<=t1)
          drvindex++;
        else
          break;
      }
      if (drvindex+1<n || t1.diff_msec (drv[drvindex].timestamp)<min_time_odo) {
        cdv = drv[drvindex];
      }
    }
  }

  // hier ist der Anfangspunkt der Integration gefunden.
  // die aktuell zu verwendende Geschwindigkeit steht in drv
  // die aktuelle Position in start
  // predindex zeigt auf den letzten pred-Eintrag vor t1
  // zu Beginn eines Integrationsschritts steht in:
  //    tstart: der Startzeitpunkt des naechsten Integrationsschritts
  //    cdv: der Fahrtvektor/Odometrie zu Beginn des naechsten Integrationsschritts
  //    start: Roboterposition- und Geschwindigkeit zu Beginn des Integrationsschritts

  // schrittweise ueber pred integrieren:
  while (predindex+1<pred.size()) {
    if (pred[predindex+1].timestamp<t2) {
      // einen Integrationsschritt durchfuehren
      heun_integration_step (start, cdv.dv, pred[predindex+1].dv, tstart, pred[predindex+1].timestamp, gs, true);
      cdv=pred[predindex+1];
      tstart=cdv.timestamp;
      predindex++;
    } else {
      break;
    }
  }

  if (! (t2.diff_msec (tstart)<min_time_odo)) {
    // letzter Eintrag aus pred liegt schon lange zurueck, evtl. existiert aktuellere Information aus odo
    unsigned int odoindex=0;
    if (mode>=2) {
      predindex=pred.size();
    } else {
      while (odoindex+1<n) {
        if (odo[odoindex+1].timestamp<t2) {
          if (odo[odoindex+1].timestamp>tstart) {
            // einen Integrationsschritt durchfuehren
            heun_integration_step (start, cdv.dv, odo[odoindex+1].dv, tstart, odo[odoindex+1].timestamp, gs, true);
            cdv=odo[odoindex+1];
            tstart=cdv.timestamp;
          }
          odoindex++;
        } else {
          break;
        }
      }
    }

    if (! (t2.diff_msec (tstart)<min_time_odo)) {
      // letzter Eintrag aus pred und odo liegt schon lange zurueck, evtl. existiert aktuellere Information aus drv
      unsigned int drvindex=0;
      while (drvindex+1<n) {
        if (drv[drvindex+1].timestamp<t2) {
          if (drv[drvindex+1].timestamp>tstart) {
            // einen Integrationsschritt durchfuehren
            heun_integration_step (start, cdv.dv, drv[drvindex+1].dv, tstart, drv[drvindex+1].timestamp, gs, mode<=1);
            cdv=drv[drvindex+1];
            tstart=cdv.timestamp;
          }
          drvindex++;
        } else {
          break;
        }
      }
    }
  }

  // den letzten Teilschritt bis zum Zeitpunkt t2 machen
  dt = t2.diff_msec (tstart);
  start.vtrans = cdv.dv.vtrans.rotate (start.heading);
  start.vrot = cdv.dv.vrot;
  if (mode<=1)
    gs.replace (start, tstart);
  start.kick = cdv.dv.kick;

  Angle deltah;
  bool gyro_int = gs.integrate (deltah, t1, t2);
  Angle h2=start.heading+deltah;

  start = update_robot_location (start, dt);
  if (mode<=1 && gyro_int) {
    start.heading=h2;
  }

  return start;
}

DriveVector OdometryContainer::get_odometry (Time t) const throw () {
  DriveVector dest (odo[0].dv);
  unsigned int i=1;
  while (i<n && odo[i].timestamp<=t) {
    dest = odo[i].dv;
    i++;
  }
  return dest;
}

DriveVector OdometryContainer::get_drive_vector (Time t) const throw () {
  DriveVector dest (drv[0].dv);
  unsigned int i=1;
  while (i<n && drv[i].timestamp<=t) {
    dest = drv[i].dv;
    i++;
  }
  return dest;
}

GyroData OdometryContainer::get_gyro_data (Time t) const throw () {
  GyroData dest;
  dest.vrot=0;
  unsigned int i=1;
  while (i<n && gyro[i].timestamp<=t) {
    dest = gyro[i].gd;
    i++;
  }
  return dest;
}

DriveVector OdometryContainer::get_velocity_estimate (Time t) const throw () {
  DriveVector dest;
  unsigned int i=1;
  while (i<pred.size() && pred[i].timestamp<=t) {
    dest = pred[i].dv;
    i++;
  }
  return dest;
}

void OdometryContainer::set_velocity_estimates (const std::vector<OdometryContainer::TimestampDriveVector>& p2) throw () {
  pred = p2;
}

