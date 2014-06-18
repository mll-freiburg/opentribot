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


void CoachWidget::init()
{
  display_locked=true;
  policy_combo->clear();
  for (unsigned int i=0; i<REMBB.coach_state.list_policies.size(); i++)
    policy_combo->insertItem (REMBB.coach_state.list_policies[i].c_str());
  policy_combo->setCurrentText (REMBB.coach_state.policy_name.c_str());
  latest_policy_name=REMBB.coach_state.policy_name;
  ball_position_mode_check->setChecked (REMBB.coach_state.ball_position_mode);
  ball_posession_mode_check->setChecked (REMBB.coach_state.ball_posession_mode);
  broadcast_mode_check->setChecked (REMBB.coach_state.broadcast_mode);
  teammate_mode_check->setChecked (REMBB.coach_state.teammate_mode);
  sl_mirror_hint_check->setChecked (REMBB.coach_state.sl_mirror_hint_mode);
  display_locked=false;
  tacticsWidget->init();
}

void CoachWidget::destroy()
{
  // leer, da keine dynamische Objekte erzeugt wurden
}

void CoachWidget::valueChanged( int )
{
  REMBB.coach_state.policy_name = policy_combo->currentText().ascii();
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

void CoachWidget::update()
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
    policy_combo->setCurrentText (latest_policy_name.c_str());
  }
  display_locked=false;
  tacticsWidget->update();

  QWidget::update();
}


void CoachWidget::extraMessage( const QString & s)
{
  if (s!="") {
    REMBB.coach_state.extra_message = s.ascii();
    message_combo->setCurrentText ("");
  }
}


void CoachWidget::tacticsOnOffChanged( bool b )
{
  REMBB.coach_state.tactics_mode=b;
}


void CoachWidget::tacticsVariantsSelected( const QString & sel )
{
  const Tribots::TacticsBoard& defaultTB (tactics_variants[std::string("default")]);
  const Tribots::TacticsBoard& desiredTB (tactics_variants[std::string(sel.ascii())]);
  for (std::map<std::string, std::string>::const_iterator it=defaultTB.begin(); it!=defaultTB.end(); it++) {
    REMBB.coach_state.tactics_board[it->first] = ((desiredTB[it->first].length()>0) ? desiredTB[it->first] : defaultTB[it->first]);
  }
  tacticsWidget->updateTactics (tactics_variants[std::string(sel.ascii())]);
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
    comboBoxTacticsVariants->insertItem (variants[i].c_str());
  }
}
