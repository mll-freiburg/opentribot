
#ifndef _TribotsTools_CoachWidget_h_
#define _TribotsTools_CoachWidget_h_

#include "UI/CoachWidget.h"
#include <QtGui/QWidget>
#include <map>
#include <string>
#include "../../../../Fundamental/ConfigReader.h"
#include "../../../../Structures/TacticsBoard.h"


namespace TribotsTools {

  class CoachWidget : public QWidget, private Ui::CoachWidget {
    Q_OBJECT

  public:
    CoachWidget(QWidget* =0, Qt::WindowFlags =0);
    ~CoachWidget ();

  public slots:
    void init( const Tribots::ConfigReader & cfg );
    void valueChanged( int );
    void valueChanged( bool );
    void extraMessage( const QString & s);
    void tacticsOnOffChanged( bool b );
    void tacticsVariantsSelected( const QString & sel );
    void update ();

  private:
    bool display_locked;
    std::map<std::string, Tribots::TacticsBoard> tactics_variants;
    std::string latest_policy_name;
  };

}

#endif
