
#include "TribotsviewMainWidget.h"
#include <QtGui/QApplication>
#include <QtGui/QStatusBar>

#include <string>
#include <sstream>
#include <cmath>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

TribotsviewMainWidget::TribotsviewMainWidget (QWidget* parent, Qt::WindowFlags flags) : QMainWindow (parent, flags) {
  setupUi (this);

  imageviewDialog=NULL;
  slwidget=NULL;

  connect( fileExitAction, SIGNAL( triggered() ), this, SLOT( fileExit() ) );
  connect( PlayStepRew, SIGNAL( triggered() ), this, SLOT( prevCycle() ) );
  connect( PlayFastFwd, SIGNAL( triggered() ), this, SLOT( start_ffw() ) );
  connect( PlayFastRew, SIGNAL( triggered() ), this, SLOT( start_frew() ) );
  connect( PlayFwd, SIGNAL( triggered() ), this, SLOT( start_play() ) );
  connect( StopPlay, SIGNAL( triggered() ), this, SLOT( stop_play() ) );
  connect( cycle_num, SIGNAL( returnPressed() ), this, SLOT( setCycleNum() ) );
  connect( display_rate, SIGNAL( valueChanged(int) ), this, SLOT( change_display_frequency(int) ) );
  connect( display_rate, SIGNAL( sliderMoved(int) ), this, SLOT( change_display_frequency(int) ) );
  connect( cycle_slider, SIGNAL( valueChanged(int) ), this, SLOT( cycle_slider_value_changed(int) ) );
  connect( Revert, SIGNAL( triggered() ), this, SLOT( revert_file() ) );
  connect( Reload, SIGNAL( triggered() ), this, SLOT( reload_file() ) );
  connect( PlayStepFwd, SIGNAL( triggered() ), this, SLOT( nextCycle() ) );
  connect( PlayRew, SIGNAL( triggered() ), this, SLOT( start_rew() ) );
  connect( ReplaceCycleInfo, SIGNAL( triggered() ), this, SLOT( replaceCycleInfo() ) );
  connect( prog_time, SIGNAL( returnPressed() ), this, SLOT( setTime() ) );
  connect( DisplayImages, SIGNAL( toggled(bool) ), this, SLOT( toggleImageView (bool) ) );
  connect( LoadImages, SIGNAL( triggered() ), this, SLOT( loadImages() ) );
  connect( goto_end_button, SIGNAL( clicked() ), this, SLOT( goto_end() ) );
  connect( goto_begin_button, SIGNAL( clicked() ), this, SLOT( goto_start() ) );
  connect( AddLogfile, SIGNAL( triggered() ), this, SLOT( loadAdditionalLogfile() ) );

  first_show=true;
}

TribotsviewMainWidget::~TribotsviewMainWidget () {
  if (imageviewDialog)
    delete imageviewDialog;
  if (slwidget)
    delete slwidget;
  for (unsigned int i=0; i<cycle_container.size(); i++)
    delete cycle_container[i];
}

void TribotsviewMainWidget::showEvent ( QShowEvent *) {
  if (!first_show)
    return;

  QList<int> sz = splitter->sizes();
  int sum = sz[0]+sz[1];
  sz[1]=20;
  sz[0]=sum-sz[0];
  splitter->setSizes(sz);
  first_show=false;
}

void TribotsviewMainWidget::init_field_and_streams( const std::deque<std::string>& info_praefix, const std::string& config_file, bool use_synch_signals1 )
{
  use_synch_signals = use_synch_signals1;
  for (unsigned int i=0; i<info_praefix.size(); i++)
    cycle_container.push_back (new TribotsTools::CycleContainer (info_praefix[i].c_str()));
  synchronize ();

  Tribots::FieldGeometry fgeometry;
  Tribots::ConfigReader reader;
  bool fg_found=false;
  try{
    if (config_file.length()>0) {
      reader.append_from_file (config_file.c_str());
      fgeometry = Tribots::FieldGeometry (reader);
      fg_found=true;
    }
  }catch(std::exception&){
    // keine Config-File Feldgeometrie-> versuche die Geometrie aus dem .log-File zu lesen
    fg_found=false;
  }
  play_mode = 0;
  wait_msec = 100;

  field_of_play->init (this, fgeometry);
  field_of_play->robot_names().resize (26);
  std::string xcv = "A";
  for (unsigned int i=0; i<26; i++) {
    xcv[0]=static_cast<char>('A'+i);
    field_of_play->robot_names()[i] = xcv;
  }
  buildCycleInfo ();
  if (!fg_found)
    fgeometry = read_field_geometry ();
  rotate_view->init (field_of_play);

  slwidget = new TribotsTools::SLErrorWidget;
  slwidget->init_pointers (reader, fgeometry);

  imageviewDialog = new TribotsTools::ImageviewWidget;

  connect (&play_control, SIGNAL(timeout()), this, SLOT(play_on()));
  connect (field_of_play, SIGNAL(unresolvedKeyPressEvent(QKeyEvent*)), this, SLOT(unresolvedKeyPressEvent(QKeyEvent*)));
  connect (field_of_play, SIGNAL(unresolvedMouseRect(Tribots::Vec, Tribots::Vec)), this, SLOT(showSLError(Tribots::Vec, Tribots::Vec)));
  connect (field_of_play, SIGNAL(vectorMessage(QString)), this, SLOT(displayStatusMessage(QString)));
  connect (field_of_play, SIGNAL(refrobotChanged()), this, SLOT(refrobotChanged())); 
  connect (slwidget, SIGNAL(robot_update()),this,SLOT(sl_pos_changed()));

  cycleChanged();
  setWindowTitle (QString("Tribotsview - ")+info_praefix[0].c_str());
}

void TribotsviewMainWidget::cycleChanged()
{
  if (cycle_container.size()>0) {
    buildCycleInfo ();
    field_of_play->next_cycle (cycle_info);
    imageviewDialog->showImage(cycle_info.cycle_num);
  }

  displayChanged();
}

void TribotsviewMainWidget::sl_pos_changed()
{
  field_of_play->next_cycle (cycle_info);
  displayChanged();
}

void TribotsviewMainWidget::displayChanged()
{
  statusBar()->showMessage ("");
  unsigned int reference_robot = field_of_play->get_preferences().reference_robot;
  QString s;

  // Info-Felder ausfuellen: Zyklusnummer, Programmzeit
  cycle_num->setText(s.setNum(cycle_info.cycle_num));
  prog_time->setText(s.setNum(cycle_info.time_msec));
  // Ballinformationsfelder ausfuellen
  const std::vector<Tribots::BallLocation>& bloc (field_of_play->get_preferences().use_exec_time ? cycle_info.bloc_exec : cycle_info.bloc_vis );
  if (bloc.size()>reference_robot) {
    ball_fuzzyness->setText(s.setNum(bloc[reference_robot].quality, 'f', 2));
    if (bloc[reference_robot].pos_known==Tribots::BallLocation::known)
      ball_known->setText("ja");
    else if (bloc[reference_robot].pos_known==Tribots::BallLocation::unknown)
      ball_known->setText("nein");
    else if (bloc[reference_robot].pos_known==Tribots::BallLocation::raised)
      ball_known->setText("hoch");
    else if (bloc[reference_robot].pos_known==Tribots::BallLocation::communicated)
      ball_known->setText("komm.");    
    ball_velocity->setText(s.setNum(bloc[reference_robot].velocity.toVec().length(),'f',2)+"; "+s.setNum(bloc[reference_robot].velocity.z,'f',2));
    ball_height->setText(s.setNum((bloc[reference_robot].pos.z<0 ? 0 : bloc[reference_robot].pos.z),'f',2));
  } else {
    ball_velocity->setText(" ----- ");
    ball_height->setText(" ----- ");
    ball_known->setText(" ----- ");
    ball_fuzzyness->setText(" ----- ");
  }
  // Roboterinformationsfelder ausfuellen
  const std::vector<Tribots::RobotLocation>& rloc (field_of_play->get_preferences().use_exec_time ? cycle_info.rloc_exec : cycle_info.rloc_vis );
  QString s1, s2, s3, s4;
  if (rloc.size()>reference_robot) {
    s1.setNum (rloc[reference_robot].vtrans.length(),'f',2);
    s2.setNum (rloc[reference_robot].vrot,'f',2);
    if (reference_robot<cycle_info.dv.size()) {
      s3.setNum (cycle_info.dv[reference_robot].vtrans.length(),'f',2);
      s4.setNum (cycle_info.dv[reference_robot].vrot,'f',2);
      robot_velocity->setText (s1+QString("; ")+s2 + QString(" / ") + s3 + QString("; ") + s4);
    } else {
      robot_velocity->setText (s1+QString("; ")+s2);
    }
    textfeld->setText (cycle_info.logmsg.c_str());
    stuck_led->setOn (rloc[reference_robot].stuck.robot_stuck);
  } else {
    textfeld->setText (" ----- ");
    stuck_led->setOn (false);
  }
  if (static_cast<double>(cycle_container[reference_robot]->size())/static_cast<double>(cycle_slider->maximum())>0.95)
    cycle_slider->setMaximum(static_cast<int>(cycle_container[reference_robot]->size()/0.95));
  cycle_slider->setValue (cycle_info.cycle_num);
  refereeState->setText(Tribots::referee_state_names [cycle_info.gs.refstate]);
  in_game_edit->setText(cycle_info.gs.in_game ? "Roboter aktiviert" : "Roboter deaktiviert");
  refereeState->setCursorPosition (0);
  std::stringstream io;
  io << field_of_play->get_preferences().imagesource_id+1 << "/" << cycle_info.vloc.size() << '\n';
  std::string txt;
  std::getline (io, txt);
  lineEditImageSource->setText(txt.c_str());
}


// Slots zum Vorwaerts- und Rueckwaertsschalten: ------------------------------------
void TribotsviewMainWidget::nextCycle()  // Einzelschritt vor
{
  play_mode=0;
  play_control.stop();
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size()) {
    cycle_container[refrobot]->step(1);
  }
  cycleChanged();
}

void TribotsviewMainWidget::prevCycle()  // Einzelschritt zurueck
{
  play_mode=0;
  play_control.stop();
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size()) 
    cycle_container[refrobot]->step(-1);
  cycleChanged();
}

void TribotsviewMainWidget::play_on()  // einen Schritt automatisch weiterschalten
{
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size()) {
    long int cn = cycle_container[refrobot]->cycle_num();
    if (cn == cycle_container[refrobot]->step(play_mode)) 
      play_mode=0;  // am Anfang/Ende angekommen, stoppen
    else
      cycleChanged();
  }
}

void TribotsviewMainWidget::start_play() // vorwaerts abspielen
{
  play_mode=1;
  play_control.start(wait_msec);
}

void TribotsviewMainWidget::start_rew() // rueckwaerts abspielen
{
  play_mode=-1;
  play_control.start(wait_msec);  
}

void TribotsviewMainWidget::start_ffw()  // schnell vorwaerts
{
  play_mode=10;
  play_control.start(wait_msec);
}

void TribotsviewMainWidget::start_frew() // schnell rueckwaerts
{
  play_mode=-10;
  play_control.start(wait_msec);
}

void TribotsviewMainWidget::stop_play()  // anhalten
{
  play_control.stop();
  play_mode=0;
}

void TribotsviewMainWidget::setCycleNum()  // Iteration direkt setzen
{
  play_mode=0;
  play_control.stop();
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size())
    cycle_container[refrobot]->set (cycle_num->text().toUInt());
  cycleChanged();
}

void TribotsviewMainWidget::setTime() // Programmzeit direkt setzen
{
  play_mode=0;
  play_control.stop();
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size())
    cycle_container[refrobot]->set_time (prog_time->text().toUInt());
  cycleChanged();  
}

void TribotsviewMainWidget::goto_start() { // an Start springen
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size())
    cycle_container[refrobot]->set (0);
  cycleChanged();
}


void TribotsviewMainWidget::goto_end() { // an Ende springen
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size())
    cycle_container[refrobot]->set (1000000);
  cycleChanged();
}

// sonstige Slots ------------------------------------------------
void TribotsviewMainWidget::toggleImageView( bool b )
{
  if (b) {
    imageviewDialog->show();
    imageviewDialog->showImage( cycle_info.cycle_num );
  }
  else {
    imageviewDialog->hide();
  }
}

void TribotsviewMainWidget::change_display_frequency( int v )
{
  int g [] = { 200, 120, 70, 50, 35, 25, 20, 15, 10 };
  wait_msec = g[v-1];
  play_control.setInterval (wait_msec);
}

void TribotsviewMainWidget::cycle_slider_moved(int i)
{
  play_mode=0;
  play_control.stop();
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size())
    cycle_container[refrobot]->set (i);
  cycleChanged();
}

void TribotsviewMainWidget::cycle_slider_value_changed( int i )
{
  if (i!=static_cast<int>(cycle_info.cycle_num))
    cycle_slider_moved (i);
}

void TribotsviewMainWidget::fileExit()
{
  qApp->quit();
}

void TribotsviewMainWidget::revert_file()
{
  play_mode=0;
  play_control.stop();
  for (unsigned int i=0; i<cycle_container.size(); i++)
    if (!cycle_container[i]->revert())
      QMessageBox::warning (this, "tribotsview", "Fehler beim Dateizugriff\nBehalte alte Datei bei", QMessageBox::Ok,  QMessageBox::NoButton);
  synchronize();
  buildCycleInfo();
  read_field_geometry();
  cycleChanged();
}

void TribotsviewMainWidget::reload_file()
{
  play_mode=0;
  play_control.stop();
  QString filterstring= "*.log";
  QString fn = QFileDialog::getOpenFileName( this, "", "", filterstring);
  if ( !fn.isEmpty() ) {
    std::string pp (fn.toAscii());
    std::string ppp = pp.substr (0,pp.length()-4);
    if (!cycle_container[0]->revert(ppp.c_str()))
      QMessageBox::warning (this, "tribotsview", "Fehler beim Dateizugriff\nBehalte alte Dateien bei", QMessageBox::Ok,  QMessageBox::NoButton);
    else {
      for (unsigned int i=1; i<cycle_container.size(); i++)
        delete cycle_container[i];
      cycle_container.erase (cycle_container.begin()+1, cycle_container.end());
    }
  }
  synchronize();
  buildCycleInfo();
  read_field_geometry();
  cycleChanged();
  setWindowTitle (QString("Tribotsview - ")+fn);
}

void TribotsviewMainWidget::loadAdditionalLogfile()
{
  play_mode=0;
  play_control.stop();
  QString filterstring= "*.log";
  QString fn = QFileDialog::getOpenFileName( this, "", "", filterstring);
  if ( !fn.isEmpty() ) {
    std::string pp (fn.toAscii());
    std::string ppp = pp.substr (0,pp.length()-4);
    try{
      cycle_container.push_back (new TribotsTools::CycleContainer (ppp.c_str()));
      synchronize();
      buildCycleInfo();
      cycleChanged();
    }catch(std::invalid_argument&){
      QMessageBox::warning (this, "tribotsview", "Fehler beim Dateizugriff\nBehalte alte Dateien bei", QMessageBox::Ok,  QMessageBox::NoButton);
    }
  }
}

void TribotsviewMainWidget::loadImages()
{
  play_mode=0;
  play_control.stop();
  QString filterstring = "*.log";
  QString filename = QFileDialog::getOpenFileName(this,"","",filterstring);
  if ( ! filename.isEmpty() ) {
    imageviewDialog->loadImages(filename, cycle_info.cycle_num);
  }
}

void TribotsviewMainWidget::showSLError( Tribots::Vec p1, Tribots::Vec p2 )
{
  slwidget->update_error (cycle_info, p1, p2, field_of_play->get_preferences().zoom.own_half);
  slwidget->show();
}

void TribotsviewMainWidget::replaceCycleInfo()
{
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size())
    cycle_container[refrobot]->replace (field_of_play->get_cycle_info ());
}

void TribotsviewMainWidget::keyPressEvent( QKeyEvent * ev)
{
  unresolvedKeyPressEvent (ev);
}

void TribotsviewMainWidget::unresolvedKeyPressEvent( QKeyEvent * event )
{
  switch (event->key()) {
  case Qt::Key_N:
    nextCycle ();
    break;
  case Qt::Key_P:
    prevCycle ();
    break;
  case Qt::Key_G:
    start_play ();
    break;
  case Qt::Key_B:
    start_rew ();
    break;
  case Qt::Key_Space:
    stop_play ();
    break;
  case Qt::Key_F:
    start_ffw ();
    break;
  case Qt::Key_R:
    start_frew ();
    break;
  }
}

Tribots::FieldGeometry TribotsviewMainWidget::read_field_geometry()
{
  // versuche die Geometrie aus dem .log-File zu lesen
  Tribots::FieldGeometry fgeometry;
  std::vector<std::string> parts;
  Tribots::split_string (parts, cycle_info.logmsg);
  if (parts.size()>=14 && parts[0]=="FieldGeometry:") {
    Tribots::string2double (fgeometry.field_length, parts[1]);
    Tribots::string2double (fgeometry.field_width, parts[2]);
    Tribots::string2double (fgeometry.side_band_width, parts[3]);
    Tribots::string2double (fgeometry.goal_band_width, parts[4]);
    Tribots::string2double (fgeometry.goal_area_length, parts[5]);
    Tribots::string2double (fgeometry.goal_area_width, parts[6]);
    Tribots::string2double (fgeometry.penalty_area_length, parts[7]);
    Tribots::string2double (fgeometry.penalty_area_width, parts[8]);
    Tribots::string2double (fgeometry.center_circle_radius, parts[9]);
    Tribots::string2double (fgeometry.corner_arc_radius, parts[10]);
    Tribots::string2double (fgeometry.penalty_marker_distance, parts[11]);
    Tribots::string2double (fgeometry.line_thickness, parts[12]);
    Tribots::string2double (fgeometry.border_line_thickness, parts[13]);
    Tribots::string2double (fgeometry.goal_width, parts[14]);
    Tribots::string2double (fgeometry.goal_length, parts[15]);
    Tribots::string2double (fgeometry.goal_height, parts[16]);
    Tribots::string2double (fgeometry.pole_height, parts[17]);
    Tribots::string2double (fgeometry.pole_diameter, parts[18]);
    Tribots::string2double (fgeometry.pole_position_offset_x, parts[19]);
    Tribots::string2double (fgeometry.pole_position_offset_y, parts[20]);
    Tribots::string2double (fgeometry.ball_diameter, parts[21]);
    Tribots::string2uint (fgeometry.lineset_type, parts[22]);
    field_of_play->set_field_geometry (fgeometry);
  }
  return fgeometry;
}

void TribotsviewMainWidget::show()
{
  QMainWindow::show();
  field_of_play->zoom_all();
}

void TribotsviewMainWidget::displayStatusMessage (QString s) {
  statusBar()->showMessage(s);
}


void TribotsviewMainWidget::buildCycleInfo()
{
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot>=cycle_container.size())
    refrobot=0;
  TribotsTools::CycleInfo new_cycle_info = cycle_container[refrobot]->get();
  Tribots::RobotLocation no_good_rloc;
  no_good_rloc.pos = Tribots::Vec (1e10, 1e10);
  no_good_rloc.vtrans = Tribots::Vec (0,0);
  no_good_rloc.vrot = 0;
  no_good_rloc.heading = Tribots::Angle::zero;
  Tribots::BallLocation no_good_bloc;
  no_good_bloc.pos = Tribots::Vec (1e10, 1e10);
  no_good_bloc.velocity = Tribots::Vec (0,0);
  no_good_bloc.pos_known = Tribots::BallLocation::unknown;
  new_cycle_info.rloc_vis.resize (cycle_container.size());
  new_cycle_info.bloc_vis.resize (cycle_container.size());
  new_cycle_info.rloc_exec.resize (cycle_container.size());
  new_cycle_info.bloc_exec.resize (cycle_container.size());
  for (unsigned int i=0; i<cycle_container.size(); i++) {
    long int citime = cycle_container[i]->shift_time (new_cycle_info.time_msec, cycle_container[refrobot]->get_reference_time());
    cycle_container[i]->set_time (citime>=0 ? citime : 0);
    TribotsTools::CycleInfo ci = cycle_container[i]->get();
    if (static_cast<long int>(ci.time_msec)-citime<100 && static_cast<long int>(ci.time_msec)-citime>-100) {
      new_cycle_info.rloc_vis[i]=(ci.rloc_vis.size()>0 ? ci.rloc_vis[0] : (cycle_info.rloc_vis.size()>i ? cycle_info.rloc_vis[i] : no_good_rloc));
      new_cycle_info.rloc_exec[i]=(ci.rloc_exec.size()>0 ? ci.rloc_exec[0] : (cycle_info.rloc_exec.size()>i ? cycle_info.rloc_exec[i] : no_good_rloc));
      new_cycle_info.bloc_vis[i]=(ci.bloc_vis.size()>0 ? ci.bloc_vis[0] : (cycle_info.bloc_vis.size()>i ? cycle_info.bloc_vis[i] : no_good_bloc));
      new_cycle_info.bloc_exec[i]=(ci.bloc_exec.size()>0 ? ci.bloc_exec[0] : (cycle_info.bloc_exec.size()>i ? cycle_info.bloc_exec[i] : no_good_bloc));
    } else {
      new_cycle_info.rloc_vis[i]=no_good_rloc;
      new_cycle_info.rloc_exec[i]=no_good_rloc;
      new_cycle_info.bloc_vis[i]=no_good_bloc;
      new_cycle_info.bloc_exec[i]=no_good_bloc;
    }
  }
  cycle_info = new_cycle_info;
}

void TribotsviewMainWidget::refrobotChanged()
{
  buildCycleInfo();
  displayChanged();
}


void TribotsviewMainWidget::synchronize()
{
  if (!use_synch_signals) return;
  if (cycle_container.size()==0) return;
  int time_sig1 = cycle_container[0]->get_synchronisation_signals ().front().first;
  long int sig1 = cycle_container[0]->get_synchronisation_signals ().front().second;
  long int delta_msec = (cycle_container[0]->get_synchronisation_signals ().back().first-time_sig1);
  int delta_sig = (cycle_container[0]->get_synchronisation_signals ().back().second-sig1);
  if (delta_sig==0) return; // nur ein Synchronisationssignal
  long int sec_per_sig = ((delta_msec+500)/delta_sig)/1000;  // +500 wegen korrekter Rundung auf ganze Sekunden
  timeval reftime = cycle_container[0]->get_reference_time();
  reftime.tv_sec += time_sig1/1000;
  reftime.tv_usec += (time_sig1%1000)*1000;
  if (reftime.tv_usec>=1000000) {
    reftime.tv_sec++;
    reftime.tv_usec-=1000000;
  }
  reftime.tv_sec-=sig1*sec_per_sig;
  // jetzt enthaelt reftime eine ungefaehre Teamcontrol-Referenzzeit und sec_per_sig das Sendeintervall in Sekunden
  for (unsigned int i=0; i<cycle_container.size(); i++) {
    const std::deque<std::pair<unsigned long int, unsigned int> >& synch_sigs = cycle_container[i]->get_synchronisation_signals ();
    if (synch_sigs.size()>0) {
      long int delta_sig=0;
      for (unsigned int j=0; j<synch_sigs.size(); j++) {
        delta_sig += static_cast<long int>(synch_sigs[j].second*sec_per_sig*1000)-static_cast<long int>(synch_sigs[j].first);
      }
      delta_sig /= static_cast<long int>(synch_sigs.size());
      timeval iref = reftime;
      iref.tv_sec+=delta_sig/1000;
      iref.tv_usec+=(delta_sig%1000)*1000;
      if (iref.tv_usec>=1000000) {
        iref.tv_usec-=1000000;
        iref.tv_sec++;
      }
      if (iref.tv_usec<0) {
        iref.tv_usec+=1000000;
        iref.tv_sec--;
      }
      cycle_container[i]->set_reference_time (iref);
    }
  }
}
