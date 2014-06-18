
#include "CoachWidget.h"
#include "../States/RemoteBlackboard.h"

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

namespace {
  void set_combo_text (QComboBox& cb, const char* s) {
    int index= cb.findText (s);
    if (index<0) {
      cb.addItem (s);
      index=cb.findText (s);
    }
    cb.setCurrentIndex (index);
  }
}


CoachWidget::CoachWidget(QWidget* p, Qt::WindowFlags f) : QWidget (p,f)
{
  setupUi(this);

  connect (policy_combo,SIGNAL(activated(int)),this,SLOT(valueChanged(int)));
  connect (message_combo,SIGNAL(activated(const QString&)),this,SLOT(extraMessage(const QString&)));
  connect (ball_position_mode_check,SIGNAL(toggled(bool)),this,SLOT(valueChanged(bool)));
  connect (ball_posession_mode_check,SIGNAL(toggled(bool)),this,SLOT(valueChanged(bool)));
  connect (broadcast_mode_check,SIGNAL(toggled(bool)),this,SLOT(valueChanged(bool)));
  connect (teammate_mode_check,SIGNAL(toggled(bool)),this,SLOT(valueChanged(bool)));
  connect (checkBoxTacticsOnOff,SIGNAL(toggled(bool)),this,SLOT(tacticsOnOffChanged(bool)));
  connect (comboBoxTacticsVariants,SIGNAL(activated(const QString&)),this,SLOT(tacticsVariantsSelected(const QString&)));
  connect (sl_mirror_hint_check,SIGNAL(toggled(bool)),this,SLOT(valueChanged(bool)));


  display_locked=true;
  policy_combo->clear();
  for (unsigned int i=0; i<REMBB.coach_state.list_policies.size(); i++)
  set_combo_text (*policy_combo, REMBB.coach_state.policy_name.c_str());
  latest_policy_name=REMBB.coach_state.policy_name;
  ball_position_mode_check->setChecked (REMBB.coach_state.ball_position_mode);
  ball_posession_mode_check->setChecked (REMBB.coach_state.ball_posession_mode);
  broadcast_mode_check->setChecked (REMBB.coach_state.broadcast_mode);
  teammate_mode_check->setChecked (REMBB.coach_state.teammate_mode);
  sl_mirror_hint_check->setChecked (REMBB.coach_state.sl_mirror_hint_mode);
  display_locked=false;
  tacticsWidget->init();
}

CoachWidget::~CoachWidget ()
{
  // leer, da keine dynamische Objekte erzeugt wurden
}

void CoachWidget::valueChanged( int )
{
  REMBB.coach_state.policy_name = std::string(policy_combo->currentText().toAscii());
}

void CoachWidget::valueChanged( bool )
{
  if (!display_locked) {
    REMBB.coach_state.ball_position_mode = ball_position_mode_check->isChecked();
    REMBB.coach_state.ball_posession_mode = ball_posession_mode_check->isChecked();
    REMBB.coach_state.broadcast_mode = broadcast_mode_check->isChecked();
    REMBB.coach_state.teammate_mode = teammate_mode_check->isChecked();
    REMBB.coach_state.sl_mirror_hint_mode = sl_mirror_hint_check->isChecked();
  }
}

void CoachWidget::update ()
{
  // das Uebersichtsfenster aktualisieren
  std::string content = "";
  for (unsigned int i=0; i<REMBB.robot_state.size(); i++)
    if (REMBB.robot_state[i].comm_started) {
      content += REMBB.robot_state[i].name;
      if (REMBB.robot_state[i].in_game)
        content+=std::string("[aktiv]");
      else
        content+=std::string("[inaktiv]");
      content += std::string(": ") + REMBB.robot_state[i].playertype;
      content += std::string("/") + REMBB.robot_state[i].playerrole;
      content += std::string ("\n");
    }
  summary_edit->setText (content.c_str());

  // die Checkboxen aktualisieren, falls Remote veraendert wurde
  display_locked=true;
  ball_position_mode_check->setChecked(REMBB.coach_state.ball_position_mode);
  ball_posession_mode_check->setChecked(REMBB.coach_state.ball_posession_mode);  broadcast_mode_check->setChecked(REMBB.coach_state.broadcast_mode);
  teammate_mode_check->setChecked(REMBB.coach_state.teammate_mode);
  sl_mirror_hint_check->setChecked(REMBB.coach_state.sl_mirror_hint_mode);
  checkBoxTacticsOnOff->setChecked(REMBB.coach_state.tactics_mode);
  if (latest_policy_name!=REMBB.coach_state.policy_name) {
    latest_policy_name=REMBB.coach_state.policy_name;
    set_combo_text (*policy_combo, latest_policy_name.c_str());
  }
  display_locked=false;
  tacticsWidget->update();

  QWidget::update();
}


void CoachWidget::extraMessage( const QString & s)
{
  if (s!="") {
    REMBB.coach_state.extra_message = std::string(s.toAscii());
    set_combo_text (*message_combo, "");
  }
}


void CoachWidget::tacticsOnOffChanged( bool b )
{
  REMBB.coach_state.tactics_mode=b;
}


void CoachWidget::tacticsVariantsSelected( const QString & sel )
{
  REMBB.coach_state.tactics_board = tactics_variants[std::string(sel.toAscii())];
  tacticsWidget->updateTactics (tactics_variants[std::string(sel.toAscii())]);
}


void CoachWidget::init( const Tribots::ConfigReader & cfg )
{
  std::vector<std::string> variants;
  std::vector<std::string> attributes;
  cfg.get ("Coach::tactics_variants", variants);
  cfg.get ("Coach::tactics_attributes", attributes);
  variants.insert (variants.begin(), "default");
  for (unsigned int i=0; i<variants.size(); i++) {
    Tribots::TacticsBoard tb;
    for (unsigned int j=0; j<attributes.size(); j++) {
      cfg.get ((variants[i]+"::"+attributes[j]).c_str(), tb [attributes[j]]);
      if (tb[attributes[j]].length()==0) {
        tb[attributes[j]] = REMBB.coach_state.tactics_attributes[j].default_value;
      }
    }
    tactics_variants[variants[i]] = tb;
    comboBoxTacticsVariants->addItem (variants[i].c_str());
  }
}
