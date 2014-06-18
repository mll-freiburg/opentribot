
#include "VisionToolMainWindow.h"
#include "ImageCenterWidget.h"
#include "BalanceAreaWidget.h"
#include "ImageMaskWidget.h"
#include "CameraDefinitionWidget.h"
#include "SettingsWidget.h"
#include "ColorCalibrationWidget.h"
#include <QtGui/QApplication>
#include <QtGui/QStatusBar>
#include <QtGui/QMenuBar>
#include <cstdlib>
#include <fstream>

using namespace TribotsTools;
using namespace std;

VisionToolMainWindow::VisionToolMainWindow (VisionToolImageSource& is, Tribots::ConfigReader& cfg, QWidget* parent, Qt::WindowFlags flags) : QMainWindow (parent, flags), imageSource(is), config (cfg), continueLoop(true) {
  setWindowTitle ("VisionTool");
  setGeometry (0,0,800,600);
  QStatusBar* stb = new QStatusBar;
  setStatusBar(stb);
  QMenuBar* mb = new QMenuBar;
  setMenuBar (mb);
  QMenu* mainMenu = mb->addMenu ("Datei");
  connect (mainMenu->addAction ("Speichern und beenden"), SIGNAL(triggered()), this, SLOT(saveExit()));

  tabWidget = new QTabWidget(this);

  addTab ("Kameraauswahl", new CameraDefinitionWidget (imageSource, config, *stb, tabWidget));
  addTab ("Bildmittelpunkt", new ImageCenterWidget (imageSource, config, *stb, tabWidget));
  addTab ("Balancebereich", new BalanceAreaWidget (imageSource, config, *stb, tabWidget));
  addTab ("Maske", new ImageMaskWidget (imageSource, config, *stb, tabWidget));
  addTab ("Kamerafeatures", new SettingsWidget (imageSource, config, *stb, tabWidget));
  addTab ("Farbkalibrierung", new ColorCalibrationWidget (imageSource, config, *stb, tabWidget));

  // HIER WEITERE WIDGETS EINFUEGEN (BALANCE-AREA, BILDMASKE, FARBEN, KAMERAAUSWAHL, ...)

  activeWindow=0;
  tabWidget->setCurrentIndex (activeWindow);
  tabs[activeWindow]->start();
  connect (tabWidget, SIGNAL(currentChanged (int)), this, SLOT(tabChanged(int)));
}


VisionToolMainWindow::~VisionToolMainWindow() {
  delete tabWidget;
  // kein delete auf tabs[i]; wird bereits von tabWidget ausgefuehrt
}

void VisionToolMainWindow::addTab (const char* n, VisionToolWidget* w) {
  tabWidget->addTab(w, n);
  tabs.push_back (w);
}

void VisionToolMainWindow::resizeEvent ( QResizeEvent *) {
  int mbh = menuBar()->height();
  if (mbh<30)
    mbh=30;
  tabWidget->setGeometry (3,3+mbh, width()-6, height()-6-statusBar()->height()-mbh);
}

void VisionToolMainWindow::closeEvent ( QCloseEvent *) {
  continueLoop=false;
  emit (widgetClosed());
}

void VisionToolMainWindow::saveExit () {
  continueLoop=false;
}

void VisionToolMainWindow::tabChanged ( int indexNew ) {
  tabs[activeWindow]->stop();
  activeWindow=indexNew;
  saveConfig();
  statusBar()->clearMessage();
  tabs[activeWindow]->start();
}

void VisionToolMainWindow::loop () {
  // PROVISORISCH:
  while (continueLoop) {
    qApp->processEvents();
    tabs[activeWindow]->loop();
  }
  tabs[activeWindow]->stop();
  saveConfig();
}

void VisionToolMainWindow::saveConfig () {
  std::string filename=getenv ("HOME");
  filename+="/.visionToolBackup.cfg";
  std::string section;
  config.get ("VisionTool::Section", section);
  ofstream file (filename.c_str());
  if (file) {
    config.write_section (file, section.c_str());
  }
  file << flush;
}
