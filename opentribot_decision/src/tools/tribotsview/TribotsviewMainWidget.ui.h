/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you want to add, delete, or rename functions or slots, use
** Qt Designer to update this file, preserving your code.
**
** You should not define a constructor or destructor in this file.
** Instead, write your code in functions called init() and destroy().
** These will automatically be called by the form's constructor and
** destructor.
*****************************************************************************/

#include "../../Structures/GameState.h"
#include "../../Fundamental/stringconvert.h"
#include <algorithm>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

void TribotsviewMainWidget::init_field_and_streams( const std::deque<std::string>& info_praefix, const std::string& config_file, bool use_sync_signals1, bool use_colored_goals1, bool use_attr )
{
  use_sync_signals = use_sync_signals1;
  use_attributes = use_attr;
  useSyncSignalsAction->setOn (use_sync_signals);
  useColoredGoalsAction->setOn (use_colored_goals1);
  useGreyRobotsAction->setOn (use_attr);
  for (unsigned int i=0; i<info_praefix.size(); i++) {
    TribotsTools::CycleContainer* ncc = NULL;
    try{
      ncc = new TribotsTools::CycleContainer (info_praefix[i].c_str());
    }catch(invalid_argument& e){
      if (get_fileextension (info_praefix[i]).length()>0) {
        ncc = new TribotsTools::CycleContainer (remove_fileextension (info_praefix[i]).c_str());
        cerr << "Verwende Logfile " << remove_fileextension (info_praefix[i]) << endl;
      } else
        throw e;
    }
    cycle_container.push_back (ncc);
  }
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

  field_of_play->init (this, fgeometry, Qt::DockTop);
  field_of_play->show_colored_goals (use_colored_goals1);
  field_of_play->show_direction (!use_colored_goals1);
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

  imageviewDialog = new ImageviewWidget;

  connect (&play_control, SIGNAL(timeout()), this, SLOT(play_on()));
  connect (field_of_play, SIGNAL(unresolvedKeyPressEvent(QKeyEvent*)), this, SLOT(unresolvedKeyPressEvent(QKeyEvent*)));
  connect (field_of_play, SIGNAL(unresolvedMouseRect(Tribots::Vec, Tribots::Vec)), this, SLOT(showSLError(Tribots::Vec, Tribots::Vec)));
  connect (field_of_play, SIGNAL(vectorMessage(QString)), this, SLOT(displayStatusMessage(QString)));
  connect (field_of_play, SIGNAL(refrobotChanged()), this, SLOT(refrobotChanged())); 
  connect (slwidget, SIGNAL(robot_update()),this,SLOT(sl_pos_changed()));

  cycleChanged();
  setCaption (QString("Tribotsview - ")+info_praefix[0].c_str());
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
  statusBar()->message ("");
  unsigned int reference_robot = field_of_play->get_preferences().reference_robot;
  QString s;

  // Info-Felder ausfuellen: Zyklusnummer, Programmzeit
  if (reference_robot==0 && std::find (bookmarks.begin(), bookmarks.end(), cycle_info.cycle_num)!=bookmarks.end()) {
    cycle_num->setPaletteBackgroundColor (QColor(214,221,255));
  } else {
    cycle_num->setPaletteBackgroundColor (Qt::white);
  }
  cycle_num->setText(s.setNum(cycle_info.cycle_num));
  std::stringstream inout;
  inout << std::floor(cycle_info.time_msec/1000.0) << '.' << std::fmod(cycle_info.time_msec,1000.0) << '\n';
  std::string line;
  std::getline (inout, line);
  if (line.length()>=2 && line[line.length()-2]=='.')
    line=line.substr(0,line.length()-1)+"00"+line[line.length()-1];
  else if (line.length()>=2 && line[line.length()-3]=='.')
    line=line.substr(0,line.length()-2)+"0"+line[line.length()-2]+line[line.length()-1];
  prog_time->setText(line.c_str());
  if (cycle_info.cycle_time>=100) {
    prog_time->setPaletteBackgroundColor (QColor(255,127,127));
  } else {
    prog_time->setPaletteBackgroundColor (Qt::white);
  }
  // Ballinformationsfelder ausfuellen
  const std::vector<Tribots::BallLocation>& bloc (field_of_play->get_preferences().use_exec_time ? cycle_info.bloc_exec : cycle_info.bloc_vis );
  std::stringstream ballsstream;
  if (bloc.size()>reference_robot) {
    if (bloc[reference_robot].pos_known==Tribots::BallLocation::unknown)
      ballsstream << "unbekannt";
    else if (bloc[reference_robot].pos_known==Tribots::BallLocation::raised)
      ballsstream << "hoch";
    else if (bloc[reference_robot].pos_known==Tribots::BallLocation::communicated)
      ballsstream << "kommuniziert";
    else { // Ball ist bekannt
      double heightmm = std::floor(bloc[reference_robot].pos.z);
      if (heightmm<0) heightmm=0;
      ballsstream << "h=" <<heightmm;
      if (bloc[reference_robot].velocity_known) {
        double ballvelXY = floor(bloc[reference_robot].velocity.toVec().length()*100)/100;
        ballsstream << ",  |v|=" << ballvelXY;
      }
    }
  } else {
    ballsstream << " --- ";
  }
  ballsstream << std::endl;
  std::string bline;
  std::getline (ballsstream, bline);
  ball_known->setText (bline.c_str());
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
    std::string outgoingText;
    for (unsigned int i=0; i<cycle_info.mboard_outgoing.size(); i++)
      outgoingText+=cycle_info.mboard_outgoing[i]+"\n";
    std::string incomingText;
    for (unsigned int i=0; i<cycle_info.mboard_incoming.size(); i++)
      incomingText+=cycle_info.mboard_incoming[i]+"\n";
    if (incomingText.length()==0)
      incomingText="[Keine eingehenden Nachrichten]";
    else
      incomingText=incomingText.substr(0,incomingText.length()-1);
    if (outgoingText.length()==0)
      outgoingText="[Keine ausgehenden Nachrichten]";
    else
      outgoingText=outgoingText.substr(0,outgoingText.length()-1);
    textEditOutgoing->setText (outgoingText.c_str());
    textEditIncoming->setText (incomingText.c_str());
//    stuck_led->setOn (rloc[reference_robot].stuck.robot_stuck);
  } else {
    textfeld->setText (" ----- ");
//    stuck_led->setOn (false);
  }
  if (static_cast<double>(cycle_container[reference_robot]->size())/static_cast<double>(cycle_slider->maxValue())>0.95)
    cycle_slider->setMaxValue(static_cast<int>(cycle_container[reference_robot]->size()/0.95));
  cycle_slider->setValue (cycle_info.cycle_num);
  std::string game_state_string=Tribots::referee_state_names [cycle_info.gs.refstate];
  while (game_state_string.length()>0 && game_state_string[game_state_string.length()-1]==' ')
    game_state_string = game_state_string.substr(0, game_state_string.length()-1);
  if (!cycle_info.gs.in_game)
    game_state_string = std::string("[[")+game_state_string+std::string("]]");
  refereeState->setText(game_state_string.c_str());
  refereeState->setCursorPosition (0);
  std::stringstream io;
  io << "Bildquelle " << field_of_play->get_preferences().imagesource_id+1 << "/" << cycle_info.vloc.size() << '\n';
  std::string txt;
  std::getline (io, txt);
  lineEditImageSource->setText(txt.c_str());
  // Verhaltensinformation anzeigen:
  lineEditPlayertype->setText((cycle_info.playertype+"/"+cycle_info.playerrole).c_str());
  if (old_behavior!=cycle_info.behavior) {
    old_behavior=cycle_info.behavior;
    std::string behaviortext="";
    std::string::size_type ind=0;
    unsigned int indent=0;
    while (ind<old_behavior.length()) {
      std::string::size_type ind1=old_behavior.find ("::", ind);
      if (ind1>=old_behavior.length())
        ind1=old_behavior.length();
      behaviortext+=old_behavior.substr (ind, ind1-ind)+"\n";
      ind=ind1+2;
      indent++;
    }
    textEditBehavior->setText (behaviortext.c_str());
  }
}


// Slots zum Vorwaerts- und Rueckwaertsschalten: ------------------------------------
void TribotsviewMainWidget::prevRefStateCycle()
{
  play_mode=0;
  play_control.stop();
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size()) {
      Tribots::GameState oldGS = cycle_info.gs;
      do {
         cycle_container[refrobot]->step(-1);
      } while ( (oldGS.refstate == cycle_container[refrobot]->get().gs.refstate) && 
                (cycle_container[refrobot]->cycle_num()>1));
   }
  cycleChanged();
}

void TribotsviewMainWidget::nextRefStateCycle()
{ 
  play_mode=0;
  play_control.stop();
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size()) {
    Tribots::GameState oldGS = cycle_info.gs;
    long int old_cycle=0;
    long int new_cycle=0;
    do {
      old_cycle=new_cycle;
      new_cycle=cycle_container[refrobot]->step(1);
    } while ( (oldGS.refstate == cycle_container[refrobot]->get().gs.refstate) &&
                (new_cycle!=old_cycle) );
  }
  cycleChanged();
}

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
  play_mode=30;
  play_control.start(wait_msec);
}

void TribotsviewMainWidget::start_frew() // schnell rueckwaerts
{
  play_mode=-30;
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
  field_of_play->setFocus();
  cycleChanged();
}

void TribotsviewMainWidget::setTime() // Programmzeit direkt setzen
{
  play_mode=0;
  play_control.stop();
  unsigned int refrobot = field_of_play->get_preferences().reference_robot;
  if (refrobot<cycle_container.size())
    cycle_container[refrobot]->set_time (static_cast<long int>(1000*prog_time->text().toDouble()));
  field_of_play->setFocus();
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
void TribotsviewMainWidget::toogleImageView( bool b )
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
  int g [] = { 200, 120, 70, 40, 25, 15, 10, 5, 3 };
  wait_msec = g[v-1];
  play_control.changeInterval (wait_msec);
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
  QString fn = QFileDialog::getOpenFileName( QString::null, filterstring, this);
  if ( !fn.isEmpty() ) {
    std::string pp (fn.latin1());
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
  setCaption (QString("Tribotsview - ")+fn);
}

void TribotsviewMainWidget::loadAdditionalLogfile()
{
  play_mode=0;
  play_control.stop();
  QString filterstring= "*.log";
  QString fn = QFileDialog::getOpenFileName( QString::null, filterstring, this);
  if ( !fn.isEmpty() ) {
    std::string pp (fn.latin1());
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
  QString filename = QFileDialog::getOpenFileName(QString::null, filterstring, this);
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
  case Key_N:
    nextCycle ();
    break;
  case Key_P:
    prevCycle ();
    break;
  case Key_G:
    start_play ();
    break;
  case Key_B:
    start_rew ();
    break;
  case Key_Space:
    stop_play ();
    break;
  case Key_F:
    start_ffw ();
    break;
  case Key_R:
    start_frew ();
    break;
  }
}

Tribots::FieldGeometry TribotsviewMainWidget::read_field_geometry()
{
  // versuche die Geometrie aus dem .log-File zu lesen
  Tribots::FieldGeometry fgeometry;
  if (Tribots::prefix ("FieldGeometry:", cycle_info.logmsg))
    if (fgeometry.deserialize (cycle_info.logmsg.substr (14,cycle_info.logmsg.length())))
      field_of_play->set_field_geometry (fgeometry);
  return fgeometry;
}

void TribotsviewMainWidget::show()
{
  QMainWindow::show();
  field_of_play->zoom_all();
}

void TribotsviewMainWidget::displayStatusMessage (QString s) {
  statusBar()->message(s);
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
  new_cycle_info.robot_attr.resize (cycle_container.size());
  for (unsigned int i=0; i<cycle_container.size(); i++) {
    long int citime = cycle_container[i]->shift_time (new_cycle_info.time_msec, cycle_container[refrobot]->get_reference_time());
    cycle_container[i]->set_time (citime>=0 ? citime : 0);
    TribotsTools::CycleInfo ci = cycle_container[i]->get();
    if (static_cast<long int>(ci.time_msec)-citime<100 && static_cast<long int>(ci.time_msec)-citime>-100) {
      new_cycle_info.rloc_vis[i]=(ci.rloc_vis.size()>0 ? ci.rloc_vis[0] : (cycle_info.rloc_vis.size()>i ? cycle_info.rloc_vis[i] : no_good_rloc));
      new_cycle_info.rloc_exec[i]=(ci.rloc_exec.size()>0 ? ci.rloc_exec[0] : (cycle_info.rloc_exec.size()>i ? cycle_info.rloc_exec[i] : no_good_rloc));
      new_cycle_info.bloc_vis[i]=(ci.bloc_vis.size()>0 ? ci.bloc_vis[0] : (cycle_info.bloc_vis.size()>i ? cycle_info.bloc_vis[i] : no_good_bloc));
      new_cycle_info.bloc_exec[i]=(ci.bloc_exec.size()>0 ? ci.bloc_exec[0] : (cycle_info.bloc_exec.size()>i ? cycle_info.bloc_exec[i] : no_good_bloc));
      if (use_attributes) {
        new_cycle_info.robot_attr[i].assign(3,0);
        if (ci.gs.in_game) {
          new_cycle_info.robot_attr[i][2]=1;
        }
      }
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
  if (!use_sync_signals) return;
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

void TribotsviewMainWidget::toggleBookmark()
{
  field_of_play->set_reference_robot (0);
  buildCycleInfo();
  std::vector<unsigned long int>::iterator it = std::lower_bound (bookmarks.begin(), bookmarks.end(), cycle_info.cycle_num);
  if (it==bookmarks.end() || (*it)!=cycle_info.cycle_num)
    bookmarks.insert (it, cycle_info.cycle_num);
  else
    bookmarks.erase (it);
  cycleChanged();
}

void TribotsviewMainWidget::gotoNextBookmark()
{
  field_of_play->set_reference_robot (0);
  buildCycleInfo();
  std::vector<unsigned long int>::iterator it = std::lower_bound (bookmarks.begin(), bookmarks.end(), cycle_info.cycle_num);
  if (it!=bookmarks.end() && (*it)==cycle_info.cycle_num)
    it++;
  if (it==bookmarks.end())
    statusBar()->message ("Kein nächstes Lesezeichen", 2000);
  else {
    unsigned int refrobot = field_of_play->get_preferences().reference_robot;
    if (refrobot<cycle_container.size())
      cycle_container[refrobot]->set (*it);
    cycleChanged();
  }
}

void TribotsviewMainWidget::gotoPrevBookmark()
{
  field_of_play->set_reference_robot (0);
  buildCycleInfo();
  std::vector<unsigned long int>::iterator it = std::lower_bound (bookmarks.begin(), bookmarks.end(), cycle_info.cycle_num);
  if (it==bookmarks.begin())
    statusBar()->message ("Kein vorheriges Lesezeichen", 2000);
  else {
    unsigned int refrobot = field_of_play->get_preferences().reference_robot;
    if (refrobot<cycle_container.size())
      cycle_container[refrobot]->set (*(it-1));
    cycleChanged();
  }
}

void TribotsviewMainWidget::clearBookmarks()
{
  bookmarks.clear();
  cycleChanged();
}


void TribotsviewMainWidget::toggleColoredGaols( bool b )
{
  field_of_play->show_colored_goals (b);
  field_of_play->show_direction (!b);
}


void TribotsviewMainWidget::toggleGreyRobots( bool b )
{
  use_attributes = b;
}


void TribotsviewMainWidget::toggleSyncSignals( bool b )
{
  use_sync_signals = b;
  synchronize();
}
