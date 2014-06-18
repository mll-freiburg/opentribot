#include "topwindow.h"
#include <qmessagebox.h>
#include <qfiledialog.h>
#include <qinputdialog.h>
#include <qhbox.h>
#include <qvbox.h>
#include <qpushbutton.h>
#include <qfont.h>

#include <iostream>
#include <math.h>
#include <sstream>
#include <stdlib.h>

#include "../../CalibrationTool/MarkerLog.h"

const std::string APPNAME = "Post Calibration Processor";
const std::string CONFIGFILE = "../../../../config_files/vision.cfg"; 
const std::string section = "OmniDistanceCalibration";

topWindow::topWindow( QWidget *parent, const char *name ) : QWidget( parent, name )
{
   this->setCaption(APPNAME);
   paper = new Paper(this, "paper");
   createMENU(); // important create menu after paper, as the menu uses a slot from paper
   // create layout
   helpWnd = new helpWindow();
   helpWnd->setCaption("Help");
   selWnd = new selWindow(paper);
   selWnd->setCaption("Selection options");

   //QPushButton *deleteMarkers = new QPushButton( "D", baselayout, "deleteMarkers" );
   //deleteMarkers->setFont( QFont( "Times", 9, QFont::Bold ) );
   //baselayout->setSpacing(0);
   //baselayout->setGeometry(0,0,paper->getWidth() + deleteMarkers->width()+2,paper->getHeight());

   //connect( deleteMarkers, SIGNAL(clicked()), paper, SLOT(deleteMarkedMarkers() ));

   char * workingdir;
   workingdir = getenv("PWD");
   std::string path(workingdir);

   autoload(path); // load marker.log and dist_marker.cfg

   // find location vision.cfg
   std::string configfile = CONFIGFILE;
   size_t lastpos = path.rfind("robotcontrol");
   if (lastpos != std::string::npos) {
     configfile = path.substr(0,lastpos) + "robotcontrol/config_files/vision.cfg";
   }

   Tribots::ConfigReader config(1); // 3 -> more debug-output
   config.append_from_file(configfile.c_str());

   distMarkerBuilder = 0;
   distMarkerBuilder = new TribotsTools::DistMarkerBuilder(config, section);
}


void topWindow::autoload(std::string path) {
   // try to autoload
   // 1st marker.log
   std::ifstream markersrc;
   std::string markerlog = path+"/marker.log";
   markersrc.open(markerlog.c_str(), std::ios::in);
   if (markersrc) { 
      std::cout << "autoloading marker.log\n";
      loadMarkerlog(markersrc);
      markerFileName = markerlog;
      markersrc.close();
   } else {
      std::cout << "not autoloading marker.log, wrong position: " << markerlog << " ?\n";
   }
   // 2nd dist_marker.cfg
   std::ifstream distsrc;
   char * homedir;
   homedir = getenv("HOME");
   std::string plainhomedir(homedir);
   std::string distmarkercfg = plainhomedir+"/.robotcontrol/dist_marker.cfg";
   distsrc.open(distmarkercfg.c_str(), std::ios::in);
   if (distsrc) { 
      std::cout << "autoloading ~/.robotcontrol/dist_marker.cfg\n";
      loadDistcfg(distsrc);
      distMarkerFileName = distmarkercfg;
      distsrc.close();
   } else {
      std::cout << "not autoloading ~/.robotcontrol/dist_marker.cfg, not there: " << distmarkercfg << " ?\n";
   }
}


void topWindow::createMENU() {
    QPopupMenu *file = new QPopupMenu( this );
    Q_CHECK_PTR( file );
    file->insertItem( "&Open",  this, SLOT(open()), CTRL+Key_O );
    file->insertItem( "&Generate lines", this, SLOT(generate_lines()), CTRL+Key_G );
    file->insertItem( "&Save marker", this, SLOT(save_markerDialog()), CTRL+Key_S );
    file->insertItem( "Save &linedistances", this, SLOT(save_lineDialog()), CTRL+Key_L );
    file->insertSeparator();
    file->insertItem( "Exi&t",  this, SLOT(terminateApplication()), CTRL+Key_T );
    file->insertItem( "Save and E&xit",  this, SLOT(saveAndExit()), CTRL+Key_X );

    QPopupMenu *settings = new QPopupMenu( this );
    Q_CHECK_PTR( settings );
    settings->insertItem( "&Selection", this, SLOT(showSelection()), CTRL+Key_E );

    QPopupMenu *help = new QPopupMenu( this );
    Q_CHECK_PTR( help );
    help->insertItem( "&Help", this, SLOT(showHelp()), CTRL+Key_H );
    help->insertItem( "&About", this, SLOT(about()), CTRL+Key_A );

    QPopupMenu *deleteMenu = new QPopupMenu( this );
    Q_CHECK_PTR( deleteMenu );
    deleteMenu->insertItem( "&Delete!", paper, SLOT(deleteMarkedMarkers()), CTRL+Key_D );

    // If we used a QMainWindow we could use its built-in menuBar().
    menu = new QMenuBar( this );
    Q_CHECK_PTR( menu );
    menu->insertItem( "&File", file );
    menu->insertItem( "&Settings", settings );
    menu->insertSeparator();
    menu->insertItem( "&Help", help );
    menu->insertItem( "&Delete marked points", deleteMenu );
    menu->setSeparator( QMenuBar::InWindowsStyle );
}


void topWindow::open() {
   QString filename = QFileDialog::getOpenFileName(
                    "marker.log",
                    "markerfile (*.log *.cfg)",
                    this,
                    "open file dialog",
                    "Choose a markerfile" );
    
   std::ifstream src;
   src.open(filename.latin1(), std::ios::in);
    
   if (!src) { 
       QMessageBox::warning( this, APPNAME,
                            "Could not open " + filename + "!\n");
       return;
   }
    
   if (filename.endsWith(".log")) {
      loadMarkerlog(src);
      markerFileName = filename;
   }
   if (filename.endsWith(".cfg")) {
      loadDistcfg(src);
      distMarkerFileName = filename;
   }

   src.close();

   paper->repaint();
}


void topWindow::loadMarkerlog(std::ifstream &src) {
   double angle = 0;
   double dist = 0;
   std::string type = "";

   paper->initFeatures();

   // try to read triples
   while ( src.good() ) {
      src >> angle;
      if (!src.good()) break;
      src >> dist;
      if (!src.good()) break;
      src >> type;
      //std::cout << "read from file: " << angle << "; " << dist << "; " << type << "\n";
      paper->addFeature(angle, dist, type);
   }
}


void topWindow::loadDistcfg(std::istream &src) {
   // read header
   src >> loadedDistFile.imagewidth;
   src >> loadedDistFile.imageheight;
   src >> loadedDistFile.center_x;
   src >> loadedDistFile.center_y;
   src >> loadedDistFile.exponent;
   double angle_multiplicator = pow(2.0,loadedDistFile.exponent);
   src >> loadedDistFile.linecount; // amount of lines
   // read carpet magic (real distances on the carpet)
   double carpetfoo;
   std::vector<double> *carpetparam = new std::vector<double>();
   for (unsigned int i = 0; i<loadedDistFile.linecount; i++) {
      src >> carpetfoo;
      carpetparam->push_back(carpetfoo);
   }
   int foo;
   double angle;
   feature f;
   f.type = "undefined";
   std::vector<std::vector<feature> > *distLines;
   distLines = new std::vector<std::vector<feature> >(loadedDistFile.linecount);
   while(src.good()){
      // read new line
      src >> foo; // read current angle
      if (!src.good()) // catch last read
         break;
      angle = foo*angle_multiplicator;
      src >> foo; // read a marking 0
      if (foo != 0) {
         std::cout << "WARNING: read a non-0, aborting fileparsing!\n";
         return;
      }
      for (unsigned int i = 0; i<loadedDistFile.linecount; i++) { // read data
         f.angle = angle;
         src >> f.dist;
         (*distLines)[i].push_back(f); 
      }
   }

   paper->setLines(distLines, carpetparam);
}

bool topWindow::isMarkerfileLoaded() {
   std::vector<feature> * featList = paper->getFeatureList();
   if (featList->size() == 0) {
       QMessageBox::warning( this, APPNAME,
                             "There are no markers loaded! "
                             "Please load marker.log first.\n");
       return false;
   }
   return true;
}


void topWindow::save_markerDialog() {
   if (!isMarkerfileLoaded()) return;

   markerFileName =  QFileDialog::getSaveFileName(
                    markerFileName,
                    "markerfile (*.log)",
                    this,
                    "save markerfile dialog",
                    "Choose a location for the markerfile." );

   save_marker();
}


void topWindow::save_marker() {
   if (!isMarkerfileLoaded()) return;
   std::vector<feature> * featList = paper->getFeatureList();

   std::ofstream save;
   save.open(markerFileName.latin1(), std::ios::out);
    
   if (!save) { 
       QMessageBox::warning( this, APPNAME,
                            "Could not open " + markerFileName + "!\n");
       return;
   }
   for (unsigned int i = 0; i < featList->size(); i++) {
        save << ((*featList)[i]).angle << " " << ((*featList)[i]).dist << " " << ((*featList)[i]).type << "\n";
    }
    save.close();
}


void topWindow::save_lineDialog() {
   std::vector<std::vector<feature> > *lines = paper->getLines();
   if ( (lines == 0) || (lines->size() == 0) ) {
      QMessageBox::warning( this, APPNAME,
                              "There are no lines loaded! Please "
                              "load dist_marker.cfg first.\n");
      return;
   }
   distMarkerFileName = QFileDialog::getSaveFileName(
                    distMarkerFileName,
                    "markerfile (dist_marker.cfg)",
                    this,
                    "save distmarker dialog",
                    "Choose a location for the distmarkerfile." );
   save_lines();
}



void topWindow::save_lines() {
   std::vector<std::vector<feature> > *lines = paper->getLines();
   if ( (lines == 0) || (lines->size() == 0) ) {
      QMessageBox::warning( this, APPNAME,
                              "There are no lines loaded! Please "
                              "load dist_marker.cfg first.\n");
      return;
   }
   if ( lines->size() != loadedDistFile.linecount ) {
      QMessageBox::warning( this, APPNAME,
                              "Dataintegrity can not be guaranteed: "
                              "linecount does not fit!\n");
      return;
   }

   std::vector<double> *carpetparam = paper->getCarpetparam();
   if ( carpetparam->size() != loadedDistFile.linecount ) {
      QMessageBox::warning( this, APPNAME,
                              "Dataintegrity can not be guaranteed: "
                              "carpetparams do not fit!\n");
      return;
   }

   std::ofstream save;
   save.open(distMarkerFileName.latin1(), std::ios::out);
    
   if (!save) { 
      QMessageBox::warning( this, APPNAME,
                           "Could not open " + distMarkerFileName + "!\n");
      return;
   }
   save << loadedDistFile.imagewidth << " " 
        << loadedDistFile.imageheight << " "
        << loadedDistFile.center_x << " "
        << loadedDistFile.center_y << "\n"
        << loadedDistFile.exponent << " "
        << loadedDistFile.linecount << "\n";
   for (unsigned int i=0; i<carpetparam->size(); i++) {
     save << carpetparam->at(i) << " ";
   }
   save << "\n";
   double angle_multiplicator = pow(2.0,loadedDistFile.exponent);
   double angle;
   for (unsigned int point=0; point<(*lines)[0].size(); point++) {
      angle = (*lines)[0][point].angle;
      save << (int)(angle/angle_multiplicator) << " 0";
      for (unsigned int i=0; i<lines->size(); i++) {
         if (angle != (*lines)[i][point].angle) {
            QMessageBox::warning( this, APPNAME,
                           "Data corrupted: angles have to be equal!\n");
            save.close();
            return;
         }
         save << " " << (*lines)[i][point].dist;
      }
      save << "\n";
   }
   save.close();
}


void topWindow::generate_lines() {
   if (!isMarkerfileLoaded()) return;
   if (distMarkerBuilder == 0) return;
   std::cout << "generating lines\n";

   // convert dataformats (string -> ENUM)
   std::vector<feature> * featList = paper->getFeatureList();
   std::vector<TribotsTools::MarkerLog> markerlist;
   for (unsigned int i = 0; i < featList->size(); i++) {
      TribotsTools::MarkerLog marker;
      marker.angle = Tribots::Angle::deg_angle(((*featList)[i]).angle);
      marker.distance = ((*featList)[i]).dist;
      marker.type = TribotsTools::MarkerLog::NL; 
      std::string type = ((*featList)[i]).type;
      if (type == "bw")
         marker.type = TribotsTools::MarkerLog::BW; 
      if (type == "wb")
         marker.type = TribotsTools::MarkerLog::WB; 
      if (type == "wr")
         marker.type = TribotsTools::MarkerLog::WR; 
      if (type == "rm")
         marker.type = TribotsTools::MarkerLog::RM; 
      if (type == "rw")
         marker.type = TribotsTools::MarkerLog::RW; 

      markerlist.push_back(marker);
   }

   // create table
   distMarkerBuilder->createTable(markerlist);

   //load lines
   std::stringstream adapterstream;
   distMarkerBuilder->writeTable(adapterstream, 640, 480,0,0);
   loadDistcfg(adapterstream); 
   paper->repaint();
}


void topWindow::showSelection() {
    selWnd->move(this->pos());
    selWnd->show();
}


void topWindow::showHelp() {
    helpWnd->move(this->pos());
    helpWnd->show();
}


void topWindow::about() {
    QMessageBox::about( this, APPNAME,
                        "First version: 2008/March/4th\n"
                        "Author: Enrico Kochon\n\n");
}


void topWindow::terminateApplication() {
   qApp->quit();
}


void topWindow::saveAndExit() {
   save_marker();
   save_lines();
   terminateApplication();
}


topWindow::~topWindow() {
}

