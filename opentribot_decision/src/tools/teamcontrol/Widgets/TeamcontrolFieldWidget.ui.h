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

using namespace Tribots;
using namespace TribotsTools;
using namespace std;

#include "../../../Fundamental/random.h"
#include <algorithm>

namespace {
  Tribots::Time pftaletime;
}

void TeamcontrolFieldWidget::init()
{
  TribotsTools::PaintPreferences ppref;
  TribotsTools::PaintActionSelect psel;

  ppref.show_wm_robot=ppref.show_wm_ball=ppref.show_robot_ids=ppref.show_robot_ball_links=true;
  ppref.show_wm_obstacles=ppref.show_vis_lines=ppref.show_vis_ball=ppref.show_vis_obstacles=false;
  ppref.show_vis_goals=ppref.show_sl_pos=false;
  ppref.show_aux_lines=true;
  ppref.show_robot_trace=ppref.show_ball_trace=false;
  ppref.use_exec_time=false;
  ppref.reference_robot=0;
  ppref.imagesource_id=0;
  ppref.zoom.scaling=ppref.zoom.own_half=ppref.field_heading=1;
  ppref.show_direction=true;
  ppref.show_colored_goals=false;

  psel.show_wm_robot=psel.show_wm_ball=psel.show_robot_ids=psel.show_robot_ball_links=true;
  psel.show_wm_obstacles=true;
  psel.show_vis_lines=psel.show_vis_ball=psel.show_vis_obstacles= psel.show_vis_goals=true;
  psel.show_sl_pos=false;
  psel.show_aux_lines=true;
  psel.show_robot_trace=psel.show_ball_trace=true;
  psel.use_exec_time=false;
  psel.clear_lines=true;
  psel.zoom_in=psel.zoom_out=psel.zoom_all=psel.zoom_undo=psel.zoom_redo=psel.flip_sides=true;
  psel.flip_goals=false;
  psel.next_refrobot=true;
  psel.next_imagesource=true;

  field->init (NULL, REMBB.team_state.field_geometry, ppref, psel);

  std::vector<std::string> robot_names (REMBB.robot_state.size());
  for (unsigned int i=0; i<REMBB.robot_state.size(); i++)
    robot_names[i]=REMBB.robot_state[i].name;
  field->robot_names () = robot_names;

  connect (field, SIGNAL(slDisplacement ()), this, SLOT(slhint()));
  connect (field, SIGNAL(robotDisplacement (Tribots::Vec, Tribots::Angle)), this, SLOT(gotoPos( Tribots::Vec, Tribots::Angle )));
  connect (field, SIGNAL(unresolvedKeyPressEvent(QKeyEvent*)), this, SLOT(unresolvedKeyPressEvent(QKeyEvent*)));

  field->zoom_all();  // Befehl wirkt noch nicht, da Fenster noch nicht sichtbar
}


void TeamcontrolFieldWidget::destroy()
{
  // im Moment muss hier nichts gemacht werden
}


void TeamcontrolFieldWidget::updateRequests()
{
  // Requests setzen
  unsigned int ref = field->get_preferences().reference_robot;
  for (unsigned int i=0; i<REMBB.robot_state.size(); i++) {
    if (i==ref) {
      REMBB.robot_state[i].obstacle_request = field->get_preferences().show_wm_obstacles;
      REMBB.robot_state[i].visible_object_request = field->get_preferences().show_vis_lines || field->get_preferences().show_vis_ball || field->get_preferences().show_vis_obstacles || field->get_preferences().show_vis_goals;
    } else {
      REMBB.robot_state[i].obstacle_request = false;
      REMBB.robot_state[i].visible_object_request = false;
    }
  }
}


void TeamcontrolFieldWidget::update()
{
  unsigned int ref = field->get_preferences().reference_robot;

  // Anzeige aktualisieren
  TribotsTools::CycleInfo ci;
  ci.rloc_vis.resize(REMBB.robot_state.size());
  ci.bloc_vis.resize(REMBB.robot_state.size());
  ci.robot_attr.resize(REMBB.robot_state.size());
  for (unsigned int i=0; i<REMBB.robot_state.size(); i++) {
    if (REMBB.robot_state[i].comm_okay) {
      ci.rloc_vis[i]=REMBB.robot_state[i].robot_pos;
      ci.bloc_vis[i]=REMBB.robot_state[i].ball_pos;
      ci.robot_attr[i].push_back (0);
      ci.robot_attr[i].push_back (0);
      ci.robot_attr[i].push_back (0);
      if (REMBB.robot_state[i].message_board.scan_for_prefix ("pB!")=="pB!")
        ci.robot_attr[i][0]=1;  // do PossessBall gesetzt
      const std::vector<std::string>& outg = REMBB.robot_state[i].message_board.get_outgoing();
      std::vector<std::string>::const_iterator fd = std::find (outg.begin(), outg.end(), std::string("NearBall!"));
      if (fd!=outg.end())
        ci.robot_attr[i][1]=1;  // NearBall gesendet
      if (REMBB.robot_state[i].in_game)
        ci.robot_attr[i][2]=1;  // in_game gesendet
    } else {
      ci.rloc_vis[i].pos=Tribots::Vec (100000,100000);
      ci.rloc_vis[i].vtrans=Tribots::Vec(0,0);
      ci.rloc_vis[i].stuck.robot_stuck=false;
      ci.bloc_vis[i].pos=Tribots::Vec (100000,100000);
      ci.bloc_vis[i].velocity=Tribots::Vec(0,0);
      ci.bloc_vis[i].pos_known = Tribots::BallLocation::unknown;
    }
  }
  ci.oloc = REMBB.robot_state[ref].obs_pos;
  ci.vloc = REMBB.robot_state[ref].visible_objects;

  // Hilfslinien (Pfeile, Texte eintragen) und Liste aktualisieren
  {
    std::list<TribotsTools::DisplayRobotArrow>::iterator it1 = REMBB.help_state.robotarrows.begin();
    double offsetx [] = { -3, +5, +3, -5 };
    double offsety [] = { +5, +3, -5, -3 };
    unsigned int index = (field->get_preferences().zoom.own_half>0 ? 0 : 1)*2+(field->get_preferences().zoom.orientation>0 ? 0 : 1);
    Vec offset = 1.0/field->get_preferences().zoom.scaling*Vec (offsetx[index], offsety[index]);
    Tribots::Time now;
    while (it1!=REMBB.help_state.robotarrows.end()) {
      if (it1->deadline>now) {
        int source_index=-1;
        int target_index=-1;
        for (unsigned int i=0; i<REMBB.robot_state.size(); i++) {
          if (REMBB.robot_state[i].id==it1->source_id)
            source_index=i;
          if (REMBB.robot_state[i].id==it1->target_id)
            target_index=i;
        }
        if (source_index>=0 && target_index>=0 && target_index!=source_index && !REMBB.robot_state[source_index].comm_interrupted && !REMBB.robot_state[target_index].comm_interrupted) { 
          Vec sp = REMBB.robot_state[source_index].robot_pos.pos;
          Vec tp = REMBB.robot_state[target_index].robot_pos.pos;
          Vec txp = 0.5*(sp+tp);
          stringstream inout;
          inout << " " << it1->color << " arrow " << sp << tp;
          if (it1->text.length()>0) 
            inout << " word " << txp+offset << it1->text;
          inout << '\n';
          string line;
          getline (inout, line);
          ci.paintmsg += line;
        } else {
          if (source_index<0 || REMBB.robot_state[source_index].comm_interrupted)
            cerr << "Achtung: Coach referenziert nicht existenten Roboter " << it1->source_id << '\n';
          if (target_index<0 || REMBB.robot_state[target_index].comm_interrupted) 
            cerr << "Achtung: Coach referenziert nicht existenten Roboter " << it1->target_id << '\n';
          if (target_index==source_index && target_index>=0) 
            cerr << "Achtung: Coach will Pfeil von Roboter zu sich selbst zeichnen: " << it1->target_id << '\n';
        }
        it1++;
      } else {
        it1=REMBB.help_state.robotarrows.erase (it1);
      }
    }
  }

  {
    std::list<TribotsTools::DisplayRobotText>::iterator it2 = REMBB.help_state.robottext.begin();
    double offsetx [] = { +15, 0, -15, 0 };
    double offsety [] = { 0, -15, 0, +15 };
    unsigned int index = (field->get_preferences().zoom.own_half>0 ? 0 : 1)*2+(field->get_preferences().zoom.orientation>0 ? 0 : 1);
    Vec offset = 1.0/field->get_preferences().zoom.scaling*Vec (offsetx[index], offsety[index]);
    Tribots::Time now;
    while (it2!=REMBB.help_state.robottext.end()) {
      if (it2->deadline>now && it2->text.length()>0) {
        int source_index=-1;
        for (unsigned int i=0; i<REMBB.robot_state.size(); i++) {
          if (REMBB.robot_state[i].id==it2->robot_id)
            source_index=i;
        }
        if (source_index>=0 && !REMBB.robot_state[source_index].comm_interrupted) { 
          Vec sp = REMBB.robot_state[source_index].robot_pos.pos;
          stringstream inout;
          inout << " " << it2->color << " word " << sp+offset << it2->text << '\n';
          string line;
          getline (inout, line);
          ci.paintmsg += line;
        } else {
          if (source_index<0 || REMBB.robot_state[source_index].comm_interrupted)
            cerr << "Achtung: Coach referenziert nicht existenten Roboter " << it2->robot_id << '\n';
        }
        it2++;
      } else {
        it2=REMBB.help_state.robottext.erase (it2);
      }
    }
  }
  if (pftaletime.elapsed_sec()>30 && REMBB.team_state.refstate==Tribots::stopRobot && REMBB.team_state.refbox_connected && urandom()<0.05) {
    pftaletime.update();
    field->say ();
  }

  field->next_cycle (ci);

  QWidget::update();
}


void TeamcontrolFieldWidget::slhint()
{
  const TribotsTools::CycleInfo& ci = field->get_cycle_info ();
  unsigned int ref = field->get_preferences().reference_robot;
  REMBB.robot_state[ref].slhint_request=true;
  REMBB.robot_state[ref].slhint_pos=ci.rloc_vis[ref].pos;
  REMBB.robot_state[ref].slhint_angle=ci.rloc_vis[ref].heading;
}


void TeamcontrolFieldWidget::gotoPos( Tribots::Vec p, Tribots::Angle h)
{
  unsigned int ref = field->get_preferences().reference_robot;
  REMBB.robot_state[ref].drive_to_request=true;
  REMBB.robot_state[ref].drive_to_pos=p;
  REMBB.robot_state[ref].drive_to_angle=h;
}


void TeamcontrolFieldWidget::show()
{
  QWidget::show();
  field->zoom_all();
}

void TeamcontrolFieldWidget::unresolvedKeyPressEvent( QKeyEvent * event )
{
  switch (event->key()) {
  case Key_G: //go tribots
    REMBB.team_state.refbox_signal = Tribots::SIGstart;
    break;
  case Key_Space: //stop playing
    REMBB.team_state.refbox_signal = Tribots::SIGstop;
    break;
  }
}
