
#ifndef _TribotsTools_TacticsWidget_h_
#define _TribotsTools_TacticsWidget_h_

#include <qscrollview.h>
#include <vector>
#include "../../../Structures/TacticsBoard.h"
#include "../../components/IDQObjects.h"

namespace TribotsTools {

  /** Widget, um Taktikeinstellungen vorzunehmen */
  class TacticsWidget : public QScrollView {
    Q_OBJECT

  public:
    TacticsWidget ( QWidget* =0, const char* = 0, WFlags = 0 );
    ~TacticsWidget ();

    void init ();
    /** setzt die angezeigten Werte auf die uebergebenen Werte */
    void updateTactics (const Tribots::TacticsBoard&);
    void update();

  protected slots:
    /** eine ComboBox mit ID (arg1) hat sich veraendert auf den Wert (arg2) */
    void comboBoxChanged (unsigned int, const QString&);

  protected:
    std::vector<IDComboBox*> comboBoxes;
  };

}

#endif
