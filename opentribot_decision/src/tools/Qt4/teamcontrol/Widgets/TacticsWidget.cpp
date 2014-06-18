
#include "TacticsWidget.h"
#include <QtGui/QFrame>
#include <QtGui/QLabel>
#include "../States/RemoteBlackboard.h"
#include "../../components/IDQObjects.h"

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

TacticsWidget::TacticsWidget ( QWidget* parent, Qt::WindowFlags) : QScrollArea (parent) {
  // die ganze Initialisierung wird in init() gemacht
}

TacticsWidget::~TacticsWidget () {;}

void TacticsWidget::init () {
  // das Widget aufbauen
  unsigned int y0=0;  // die aktuelle y0-Position des naechsten Frames
  QFrame* cframe = new QFrame;
  for (unsigned int i = 0; i<REMBB.coach_state.tactics_attributes.size(); i++) {
    QFrame* frame = new QFrame (cframe);
    frame->setFrameShape (QFrame::NoFrame);
    frame->setFrameShadow (QFrame::Plain);
    frame->setGeometry (0, y0, 500, 30);
    QLabel* label = new QLabel (frame);
    label->setText (REMBB.coach_state.tactics_attributes[i].name.c_str());
    label->setGeometry (140, 5, 350, 20);
    IDComboBox* comboBox = new IDComboBox (frame, i);
    if (REMBB.coach_state.tactics_attributes[i].editable)
      comboBox->setEditable(true);
    comboBox->setGeometry (5,5,130,20);
    for (unsigned int j=0; j<REMBB.coach_state.tactics_attributes[i].options.size(); j++)
      comboBox->addItem (REMBB.coach_state.tactics_attributes[i].options[j].c_str());
    comboBox->setCurrentText (REMBB.coach_state.tactics_board [REMBB.coach_state.tactics_attributes[i].name].c_str());
    connect (comboBox,SIGNAL(comboBoxChanged (unsigned int, const QString&)), this, SLOT(comboBoxChanged (unsigned int, const QString&)));
    y0+=30;
    comboBoxes.push_back (comboBox);
  }

  cframe->adjustSize();
  setWidget(cframe);
}

void TacticsWidget::comboBoxChanged (unsigned int index, const QString& qtext) {
  REMBB.coach_state.tactics_board[REMBB.coach_state.tactics_attributes[index].name]=string(qtext.toAscii());
}

void TacticsWidget::updateTactics (const Tribots::TacticsBoard& tb) {
  for (unsigned int i=0; i<comboBoxes.size(); i++) {
    std::string value = tb [REMBB.coach_state.tactics_attributes[i].name];
    if (value!=std::string(comboBoxes[i]->lastValidText().toAscii())) {
      bool is_in=false;
      for (int j=0; j<comboBoxes[i]->count(); j++) {
        if (std::string(comboBoxes[i]->itemText (j).toAscii())==value) {
          is_in=true;
          break;
        }
      }
      if (!is_in)
        comboBoxes[i]->addItem (value.c_str());
      comboBoxes[i]->setCurrentText (value.c_str());
    }
  }
}

void TacticsWidget::update() {
  updateTactics (REMBB.coach_state.tactics_board);
  QWidget::update();
}
