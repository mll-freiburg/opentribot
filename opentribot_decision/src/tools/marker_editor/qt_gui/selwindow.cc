#include "selwindow.h"
#include "paper.h"

#include <qcheckbox.h>


selWindow::selWindow(Paper *paper, QWidget *parent, const char *name) : QWidget( parent, name ){
    this->paper = paper;
    this->setFixedSize(300, 200);
    baselayout = new QVBox(this, "baselayout");
    baselayout->setGeometry(0,0, 300, 200);

    // build legend
    QVBox *selectionbox = new QVBox(baselayout, "selectionbox");
    QLabel *label;
    label = new QLabel(selectionbox, "title");
    label->setText("Selektion der markierbaren Featuretypen:");
    QHBox* legendEntry;
    // blue white
    legendEntry = new QHBox(selectionbox, "entry1");
    label = new QLabel(legendEntry, "color1");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_BW());
    label->setText("+++");
    QCheckBox * checkbox_bw = new QCheckBox("markierbar", legendEntry);
    checkbox_bw->setChecked(true);
    connect( checkbox_bw, SIGNAL(clicked()), paper, SLOT(toogleBW()));
    // white blue 
    legendEntry = new QHBox(selectionbox, "entry2");
    label = new QLabel(legendEntry, "color2");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_WB());
    label->setText("+++");
    QCheckBox * checkbox_wb = new QCheckBox("markierbar", legendEntry);
    checkbox_wb->setChecked(true);
    connect( checkbox_wb, SIGNAL(clicked()), paper, SLOT(toogleWB()));
    // Ballanfang
    legendEntry = new QHBox(selectionbox, "entry3");
    label = new QLabel(legendEntry, "color3");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_WR());
    label->setText("+++");
    QCheckBox * checkbox_wr = new QCheckBox("markierbar", legendEntry);
    checkbox_wr->setChecked(true);
    connect( checkbox_wr, SIGNAL(clicked()), paper, SLOT(toogleWR()));
    // Ballmitte
    legendEntry = new QHBox(selectionbox, "entry4");
    label = new QLabel(legendEntry, "color4");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_RM());
    label->setText("+++");
    QCheckBox * checkbox_rm = new QCheckBox("markierbar", legendEntry);
    checkbox_rm->setChecked(true);
    connect( checkbox_rm, SIGNAL(clicked()), paper, SLOT(toogleRM()));
    // Ballende
    legendEntry = new QHBox(selectionbox, "entry5");
    label = new QLabel(legendEntry, "color5");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_RW());
    label->setText("+++");
    QCheckBox * checkbox_rw = new QCheckBox("markierbar", legendEntry);
    checkbox_rw->setChecked(true);
    connect( checkbox_rw, SIGNAL(clicked()), paper, SLOT(toogleRW()));

    // close button
    closeButton = new QPushButton("Close", baselayout, "closebutton");
    connect(closeButton, SIGNAL(clicked()), this, SLOT(close()));
}
