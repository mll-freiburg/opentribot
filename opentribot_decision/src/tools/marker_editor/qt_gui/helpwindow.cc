#include "helpwindow.h"
#include "paper.h"


helpWindow::helpWindow( QWidget *parent, const char *name ) : QWidget( parent, name ){
    this->setFixedSize(200, 300);
    baselayout = new QVBox(this, "baselayout");
    baselayout->setGeometry(0,0, 200, 300);

    helptext = new QTextEdit(baselayout, "helptext");
    helptext->setText("Lade marker.log und editere die Datei mit der Maus. "
                      "Punkte mit der linken Maustaste markieren "
                      "und mit der rechten entfernen.");
    helptext->setReadOnly(true);
    

    // build legend
    QVBox *legendbox = new QVBox(baselayout, "legend");
    QLabel *label;
    label = new QLabel(legendbox, "title");
    label->setText("Legende:");
    QHBox* legendEntry;
    // blue white
    legendEntry = new QHBox(legendbox, "entry1");
    label = new QLabel(legendEntry, "color1");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_BW());
    label->setText("+++");
    label = new QLabel(legendEntry, "color1-desc");
    label->setText("blue white (bw)");
    // white blue 
    legendEntry = new QHBox(legendbox, "entry2");
    label = new QLabel(legendEntry, "color2");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_WB());
    label->setText("+++");
    label = new QLabel(legendEntry, "color2-desc");
    label->setText("white blue (wb)");
    // Ballanfang
    legendEntry = new QHBox(legendbox, "entry3");
    label = new QLabel(legendEntry, "color3");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_WR());
    label->setText("+++");
    label = new QLabel(legendEntry, "color3-desc");
    label->setText("Ballanfang (wr)");
    // Ballmitte
    legendEntry = new QHBox(legendbox, "entry4");
    label = new QLabel(legendEntry, "color4");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_RM());
    label->setText("+++");
    label = new QLabel(legendEntry, "color4-desc");
    label->setText("Ballmitte (rm)");
    // Ballende
    legendEntry = new QHBox(legendbox, "entry5");
    label = new QLabel(legendEntry, "color5");
    label->setPaletteBackgroundColor(Paper::COLOR_BACK());
    label->setPaletteForegroundColor(Paper::COLOR_RW());
    label->setText("+++");
    label = new QLabel(legendEntry, "color5-desc");
    label->setText("Ballende (rw)");

    // close button
    closeButton = new QPushButton("Close", baselayout, "closebutton");
    connect(closeButton, SIGNAL(clicked()), this, SLOT(close()));
}
