
#include "SingleStepNeuralVelocityPredictor.h"
#include "update_robot_location.h"
#include "../WorldModel.h"

using namespace Tribots;
using namespace std;

SingleStepNeuralVelocityPredictor::TP::TP () {
  xpat = new FTYPE [10];
  ypat = new FTYPE [10];
  phipat = new FTYPE [10];
}

SingleStepNeuralVelocityPredictor::TP::TP (const SingleStepNeuralVelocityPredictor::TP& tp) throw () {
  xpat = new FTYPE [10];
  ypat = new FTYPE [10];
  phipat = new FTYPE [10];
  (*this) = tp;
}

SingleStepNeuralVelocityPredictor::TP::~TP () throw () {
  delete [] xpat;
  delete [] ypat;
  delete [] phipat;
}

const SingleStepNeuralVelocityPredictor::TP& SingleStepNeuralVelocityPredictor::TP::operator= (const SingleStepNeuralVelocityPredictor::TP& tp) throw () {
  for (unsigned int i=0; i<10; i++) {
    xpat[i]=tp.xpat[i];
    ypat[i]=tp.ypat[i];
    phipat[i]=tp.phipat[i];
  }
  timestamp = tp.timestamp;
  return (*this);
}


SingleStepNeuralVelocityPredictor::SingleStepNeuralVelocityPredictor (const ConfigReader& cfg, OdometryContainer& odobox1) throw (std::bad_alloc, TribotsException) :
    max_pred_steps (10),
    max_store_steps (15),
    burn_in (max_store_steps+20),
    save_patterns (false),
    do_predict (true),
    odobox (odobox1),
    drv (max_pred_steps+max_store_steps),
    pos (max_pred_steps+max_store_steps),
    net_vx (NULL),
    net_vy (NULL),
    net_vphi (NULL),
    pred (max_store_steps+max_pred_steps-7),
    velo5 (5),
    velo10 (10),
    patstream_x (NULL),
    patstream_y (NULL),
    patstream_phi (NULL),
    recent_patterns (max_store_steps),
    save_burn_in (max_store_steps+20)
{
  cfg.get ("VelocityPredictor::do_predict", do_predict);
  cfg.get ("VelocityPredictor::collect_patterns", save_patterns);
  if (do_predict) {
    string filename_x, filename_y, filename_phi;
    if (cfg.get ("VelocityPredictor::NetX", filename_x)==0)
      throw InvalidConfigurationException ("VelocityPredictor::NetX");
    if (cfg.get ("VelocityPredictor::NetY", filename_y)==0)
      throw InvalidConfigurationException ("VelocityPredictor::NetY");
    if (cfg.get ("VelocityPredictor::NetPHI", filename_phi)==0)
      throw InvalidConfigurationException ("VelocityPredictor::NetPHI");
    net_vx = new Net;
    if (net_vx->load_net (filename_x.c_str())!=0)
      throw TribotsException ((string("Neural Network corrupted in file: ")+filename_x).c_str());
    net_vy = new Net;
    if (net_vy->load_net (filename_y.c_str())!=0)
      throw TribotsException ((string("Neural Network corrupted in file: ")+filename_y).c_str());
    net_vphi = new Net;
    if (net_vphi->load_net (filename_phi.c_str())!=0)
      throw TribotsException ((string("Neural Network corrupted in file: ")+filename_phi).c_str());
  }
  if (save_patterns) {
    patstream_x = new ofstream ("velocity_predict_x.pat");
    patstream_y = new ofstream ("velocity_predict_y.pat");
    patstream_phi = new ofstream ("velocity_predict_phi.pat");
    if (!(*patstream_x) || !(*patstream_y) || !(*patstream_phi))
      throw TribotsException ("SingleStepNeuralVelocityPredictor: Musterdateien velocity_predict_x.pat konnten nicht erzeugt werden");
    (*patstream_x) << "SNNS pattern definition file V3.2\ngenerated at Sat Apr 21 14:00:28 MEST 2001\nNo. of patterns : 0\nNo. of input units : 10\nNo. of output units : 1\n\n";  // n++ ignoriert Anzahl Muster um Header, Datum ist sowieso irrelevant, kann aber nicht weggelassen werden
    (*patstream_y) << "SNNS pattern definition file V3.2\ngenerated at Sat Apr 21 14:00:28 MEST 2001\nNo. of patterns : 0\nNo. of input units : 10\nNo. of output units : 1\n\n";
    (*patstream_phi) << "SNNS pattern definition file V3.2\ngenerated at Sat Apr 21 14:00:28 MEST 2001\nNo. of patterns : 0\nNo. of input units : 10\nNo. of output units : 1\n\n";
  }
}

SingleStepNeuralVelocityPredictor::~SingleStepNeuralVelocityPredictor () throw () {
  if (net_vx) delete net_vx;
  if (net_vy) delete net_vy;
  if (net_vphi) delete net_vphi;
  if (patstream_x) { (*patstream_x) << flush; delete patstream_x; }
  if (patstream_y) { (*patstream_y) << flush; delete patstream_y; }
  if (patstream_phi) { (*patstream_phi) << flush; delete patstream_phi; }
}


void SingleStepNeuralVelocityPredictor::notify_position (const RobotLocation& rl, Time t) throw () {
  TRL trl;
  trl.timestamp=t;
  trl.rloc = rl;
  pos.get()=trl;
  pos.step();
}

void SingleStepNeuralVelocityPredictor::notify_drive_vector (DriveVector dv, Time t) throw () {
  OdometryContainer::TimestampDriveVector tdv;
  tdv.timestamp = t;
  tdv.dv = dv;
  drv.get()=tdv;
  drv.step();
}

void SingleStepNeuralVelocityPredictor::update () throw () {
  if (!(do_predict || save_patterns))
    return;

  if (burn_in>0) {
    burn_in--;
    return;
  }
  // Sicherheitshalber die drv-Liste auffuellen
  OdometryContainer::TimestampDriveVector tdv = drv[-1];
  for (unsigned int i=0; i<max_pred_steps; i++) {
    tdv.timestamp.add_msec (33);
    drv[i]=tdv;
  }
  
  Time t0, t1, t2, tno_good;
  // erst mal die VelocitySensors fuellen
  for (unsigned int i=0; i<9; i++) {
    const TRL& trl = pos[max_pred_steps+i];
    velo5.update (trl.rloc, trl.timestamp);
    velo10.update (trl.rloc, trl.timestamp);
    t0 = t1;
    t1 = t2;
    t2 = trl.timestamp;
  }
  // jetzt alle Geschwindigkeiten der Vergangenheit schaetzen, fuer die keine Fahrtvektoren benoetigt werden
  for (unsigned int i=9; i<max_store_steps; i++) {
    const TRL& trl = pos[max_pred_steps+i];
    velo5.update (trl.rloc, trl.timestamp);
    velo10.update (trl.rloc, trl.timestamp);
    pred[i-9].timestamp = t1;
    RobotLocation rv5 = velo5.get (tno_good);
    pred[i-9].dv.vtrans = rv5.vtrans/rv5.heading;
    pred[i-9].dv.vrot = rv5.vrot;
    pred[i-9].dv.kick = false;   // TODO: richtigen Kickbefehl setzen!
    t0 = t1;
    t1 = t2;
    t2 = trl.timestamp;
  }
  // zwei fehlende Werte:
  pred[max_store_steps-9].timestamp = t1;
  pred[max_store_steps-9].dv = pred[max_store_steps-10].dv;
  pred[max_store_steps-8].timestamp = t2;
  pred[max_store_steps-8].dv = pred[max_store_steps-10].dv;
  // jetzt den zum Zeitpunkt t2 passenden Fahrtvektor suchen mit  t_dv >= t2
  unsigned int drvindex=max_pred_steps;
  while (drvindex+1<max_store_steps) {
    if (drv[drvindex+1].timestamp<t2) {
      drvindex++;
    } else {
      break;
    }
  }
  // jetzt sich Prognose fuer Prognose nach vorne hangeln
  Time newtime_pos = t2;
  Time newtime_dv = drv[drvindex].timestamp;
  for (unsigned int i=0; i<max_pred_steps; i++) {
    newtime_pos.add_msec (33);
    newtime_dv.add_msec (33);
    
    RobotLocation rv5 = velo5.get (tno_good);
    RobotLocation rv10 = velo10.get (tno_good);
    Vec vtrans5 = rv5.vtrans/rv5.heading;
    Vec vtrans10 = rv10.vtrans/rv10.heading;
    
    pattern.xpat[0]=vtrans5.x;  // Schaetzung ueber 5 Zeitschritte
    pattern.xpat[1]=vtrans10.x;  // Schaetzung ueber 10 Zeitschritte
    pattern.xpat[2]=drv[drvindex+1].dv.vtrans.x;  // aktuellster Fahrtvektor, der noch einen Einfluss haben koennte
    pattern.xpat[3]=drv[drvindex].dv.vtrans.x;
    pattern.xpat[4]=drv[drvindex-1].dv.vtrans.x;
    pattern.xpat[5]=drv[drvindex-2].dv.vtrans.x;
    pattern.xpat[6]=drv[drvindex-3].dv.vtrans.x;
    pattern.xpat[7]=drv[drvindex-4].dv.vtrans.x;  // aeltester Fahrtvektor, der noch beruecksichtigt wird
    pattern.xpat[8]=vtrans5.y;  // Schaetzung fuer andere Korrdinatenrichtung wegen Abhaengigkeit
    pattern.xpat[9]=rv5.vrot;  // Schaetzung fuer Rotation wegen Abhaengigkeit
    
    pattern.ypat[0]=vtrans5.y;
    pattern.ypat[1]=vtrans10.y;
    pattern.ypat[2]=drv[drvindex+1].dv.vtrans.y;
    pattern.ypat[3]=drv[drvindex].dv.vtrans.y;
    pattern.ypat[4]=drv[drvindex-1].dv.vtrans.y;
    pattern.ypat[5]=drv[drvindex-2].dv.vtrans.y;
    pattern.ypat[6]=drv[drvindex-3].dv.vtrans.y;
    pattern.ypat[7]=drv[drvindex-4].dv.vtrans.y;
    pattern.ypat[8]=vtrans5.x;
    pattern.ypat[9]=rv5.vrot;

    pattern.phipat[0]=rv5.vrot;
    pattern.phipat[1]=rv10.vrot;
    pattern.phipat[2]=drv[drvindex+1].dv.vrot;
    pattern.phipat[3]=drv[drvindex].dv.vrot;
    pattern.phipat[4]=drv[drvindex-1].dv.vrot;
    pattern.phipat[5]=drv[drvindex-2].dv.vrot;
    pattern.phipat[6]=drv[drvindex-3].dv.vrot;
    pattern.phipat[7]=drv[drvindex-4].dv.vrot;
    pattern.phipat[8]=vtrans5.x;
    pattern.phipat[9]=vtrans5.y;

    if (i==0) {
      if (save_patterns) {
        // Im Mustererzuegungsmodus Muster zwischenspeichern
        pattern.timestamp = newtime_dv;
        recent_patterns.get()=pattern;
        recent_patterns.step();
      }
      if (save_burn_in>0)
        save_burn_in--;
      if (save_patterns && save_burn_in==0 && MWM.get_game_state().in_game) {
        // Muster in Dateien schreiben
        // in recent_patterns nach demjenigen suchen mit Zeitstempel <= t0
        unsigned int index = 0;
        while (index+1<recent_patterns.size()) {
          if (recent_patterns[index].timestamp>t0)
            index++;
          else
            break;
        }
        const TP& pp = recent_patterns[index];
        (*patstream_x) << "# Pattern time " << pp.timestamp << '\n';
        (*patstream_y) << "# Pattern time " << pp.timestamp << '\n';
        (*patstream_phi) << "# Pattern time " << pp.timestamp << '\n';
        for (unsigned int i=0; i<10; i++) {
          (*patstream_x) << pp.xpat[i] << ' ';
          (*patstream_y) << pp.ypat[i] << ' ';
          (*patstream_phi) << pp.phipat[i] << ' ';
        }
        (*patstream_x) << '\n' << vtrans5.x << '\n';
        (*patstream_y) << '\n' << vtrans5.y << '\n';
        (*patstream_phi) << '\n' << rv5.vrot << '\n';
      }
    }

    if (do_predict) {
      FTYPE xr, yr, pr;
      net_vx->forward_pass (pattern.xpat, &xr);
      net_vy->forward_pass (pattern.ypat, &yr);
      net_vphi->forward_pass (pattern.phipat, &pr);

      DriveVector newdv;
      newdv.vtrans.x = xr;
      newdv.vtrans.y = yr;
      newdv.vrot = pr;
      newdv.kick = drv[drvindex].dv.kick;
      pred[max_store_steps-7+i].timestamp = newtime_dv;
      pred[max_store_steps-7+i].dv = newdv;
      RobotLocation pos1 = pos[i-1].rloc;
      pos1.vtrans = pred[max_store_steps-8+i].dv.vtrans*pos1.heading;
      pos1.vrot = pred[max_store_steps-8+i].dv.vrot;
      RobotLocation pos2 = update_robot_location (pos1, 33);
      pos[i].timestamp = newtime_pos;
      pos[i].rloc = pos2;
      velo5.update (pos2, newtime_pos);
      velo10.update (pos2, newtime_pos);
    }
    
    drvindex++;
  }

  if (do_predict)
    odobox.set_velocity_estimates (pred);
}
