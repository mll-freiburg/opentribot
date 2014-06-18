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

#include "../../ImageProcessing/Formation/ImageSourceFactory.h"
#include "../../ImageProcessing/Formation/IIDC.h"
#include "../../ImageProcessing/Formation/Painter.h"
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <qstatusbar.h>
#include <qbitmap.h>
#include <qfiledialog.h>
#include <typeinfo>

#define NEWROBOT__

namespace {
  double square (double x){
    return x*x;
  }
}

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

void DistanceCalibrationCollectionWidget::setCenterActionClicked()
{
  mouseMode = TribotsTools::SetCenterMode;
  setCursor (Qt::ArrowCursor);
}


void DistanceCalibrationCollectionWidget::setMaskAddActionClicked()
{
  unsigned char largeball_bits[] = { 0x3c, 0x7e, 0xff, 0xff, 0xff, 0xff, 0x7e, 0x3c };
  QBitmap largeball_bm ( 8, 8, largeball_bits, false );
  unsigned char smallball_bits[] = { 0x00, 0x3c, 0x7e, 0x7e, 0x7e, 0x7e, 0x3c, 0x00 };
  QBitmap smallball_bm ( 8, 8, smallball_bits, false );
  QCursor ballcursor ( smallball_bm, largeball_bm );
  setCursor (ballcursor);
  mouseMode = TribotsTools::SetAddMaskMode;
}


void DistanceCalibrationCollectionWidget::setBlueActionClicked()
{
  mouseMode = TribotsTools::SetBlueMode;
  setCursor (Qt::ArrowCursor);
}


void DistanceCalibrationCollectionWidget::setMaskSubActionClicked()
{
  unsigned char largeball_bits[] = { 0x3c, 0x7e, 0xff, 0xff, 0xff, 0xff, 0x7e, 0x3c };
  QBitmap largeball_bm ( 8, 8, largeball_bits, false );
  unsigned char smallball_bits[] = { 0x00, 0x3c, 0x7e, 0x7e, 0x7e, 0x7e, 0x3c, 0x00 };
  QBitmap smallball_bm ( 8, 8, smallball_bits, false );
  QCursor ballcursor ( smallball_bm, largeball_bm );
  setCursor (ballcursor);
  mouseMode = TribotsTools::SetSubMaskMode;
}


void DistanceCalibrationCollectionWidget::setRedActionClicked()
{
  mouseMode = TribotsTools::SetRedMode;
  setCursor (Qt::ArrowCursor);
}


void DistanceCalibrationCollectionWidget::setDirectionActionClicked()
{
  mouseMode = TribotsTools::SetDirectionMode;
  setCursor (Qt::ArrowCursor);
}


void DistanceCalibrationCollectionWidget::setBalanceActionClicked()
{
  mouseMode=TribotsTools::SetBalanceMode;
  setCursor (Qt::PointingHandCursor);
}


void DistanceCalibrationCollectionWidget::generateImageMaskStart()
{
  modeStarttime=imageTimestamp;
  mode=TribotsTools::DCMaskGeneration;
  statusBar()->message ("H�nde weg, Roboter f�ngt in K�rze an, sich zu drehen");
}


void DistanceCalibrationCollectionWidget::generateMarkerLogStart()
{
  modeStarttime=imageTimestamp;
  mode=TribotsTools::DCMarkerCollection;
  markers.clear();
  statusBar()->message ("H�nde weg, Roboter f�ngt in K�rze an, sich zu drehen");
}


void DistanceCalibrationCollectionWidget::stopActions()
{
  modeStarttime=imageTimestamp;
  mode=TribotsTools::DCNormal;
}


void DistanceCalibrationCollectionWidget::init()
{
  std::string start_message="";

  lastImage = NULL;
  imageSource = NULL;
  robotMask = NULL;
  robotMask2 = NULL;
  robot = NULL;
  updateDirection = false;
  wm=NULL;
  debug=false;

  cfg.add_command_line_shortcut ("d","debug",false);
  cfg.add_command_line_shortcut ("h","help",false);
  cfg.add_command_line_shortcut ("o","use_omni_cam",false);
  cfg.add_command_line_shortcut ("p","use_perspective_cam",false);
  cfg.add_command_line_shortcut ("f","use_file_cam",true);
  cfg.append_from_command_line (qApp->argc(), qApp->argv());
  configfile = "../config_files/robotcontrol.cfg";
  cfg.get ("ConfigReader::unknown_argument_1", configfile);
  cfg.append_from_file (configfile.c_str());
  cfg.append_from_command_line (qApp->argc(), qApp->argv());

  cfg.get ("debug", debug);
  bool help=false;
  cfg.get ("help", help);
  if (help) {
    commandLineHelp ();
    exit (-1);
  }

  cfg.set ("world_model_type", "Dummy");
  cfg.set ("add_write_world_model", false);
  wm = new Tribots::WorldModel (cfg);

  try{
    robot = new Tribots::Robot (cfg);
  }catch(Tribots::TribotsException& e){
    std::cerr << "Roboter kann nicht gestartet werden.\n";
    std::cerr << e.what() << '\n';
    start_message += "Roboter kann nicht gestartet werden. ";
    robot = NULL;
  }catch(std::exception& e){
    std::cerr << "Roboter kann nicht gestartet werden.\n";
    std::cerr << e.what() << std::endl;
    start_message += "Roboter kann nicht gestartet werden. ";
    robot = NULL;
  }

  bool dummy;
  std::string imageProducerSection;
  std::string filename;
  cfg.get ("image_sources", imageProducerSection);
  if (cfg.get ("use_omni_cam", dummy))
    imageProducerSection = "ImageProducer_Omni";
  else if (cfg.get ("use_perspective_cam", dummy))
    imageProducerSection = "ImageProducer_Perspective";
  else if (cfg.get ("use_file_cam", filename)) {
    imageProducerSection = "ImageProducer_File";
    if (filename.length()>0) {
      cfg.set ("FileSource::filename", filename);
    }
  }

  std::string homedir = std::getenv ("HOME");
  imageSourceSection = "";
  std::string imageSourceType;
  std::vector<int> array;
  robotMaskFile=homedir+".robotcontrol/image_mask.cfg";
  cfg.get ((imageProducerSection+"::image_source").c_str(),imageSourceSection);
  cfg.get ((imageSourceSection+"::image_source_type").c_str(),imageSourceType);
  cfg.get ((imageSourceSection+"::robot_mask_file").c_str(),robotMaskFile);
  cfg.get ((imageSourceSection+"::image_center").c_str(),array);
  if (array.size()>=2) {
    imageAnalysis.setCenter (array[0], array[1]);
  }
  cfg.get ((imageSourceSection+"::balance_area").c_str(),array);
  if (array.size()>=4) {
    balanceX1=array[0];
    balanceY1=array[1];
    balanceX2=balanceX1+array[2];
    balanceY2=balanceY1+array[3];
  }

  try{
    imageSource = Tribots::ImageSourceFactory::get_image_source_factory ()->get_image_source (imageSourceType, cfg, imageSourceSection);
  }catch(std::exception& e){
    std::cerr << "Bildquelle (Kamera) kann nicht gestartet werden. Stop.\n";
    std::cerr << "Exception was: " << e.what() << std::endl;
    exit (-1);
  }

  try{
    robotMask = new Tribots::RobotMask (robotMaskFile.c_str());
  }catch(Tribots::TribotsException& e) {
    std::cerr << "Laden der Bildmaske " << robotMaskFile << " gescheitert";
    std::cerr << e.what() << '\n';
    start_message += "Laden der Bildmaske gescheitert.";
    robotMask=NULL;
  }catch(std::exception&) {
    std::cerr << "Laden der Bildmaske " << robotMaskFile << " gescheitert";
    start_message += "Laden der Bildmaske gescheitert.";
    robotMask=NULL;
  }

  distMarkerBuilder = new TribotsTools::DistMarkerBuilder (cfg, "OmniDistanceCalibration");
  distMarkerBuilderUsed=false;

  imageAnalysis.setMask (robotMask);
  radioButtonSetRed->setChecked (true);
  setCenterActionClicked();
  connect (imageWidget, SIGNAL(mousePressed(QMouseEvent*)), this, SLOT(mouseInImagePressed(QMouseEvent*)));
  connect (imageWidget, SIGNAL(mouseMoved(QMouseEvent*)), this, SLOT(mouseInImageMoved(QMouseEvent*)));
  colorSample.resize (3);
  classifierChanged=false;
  mode=TribotsTools::DCNormal;

  autoinfo.useAuto=true;
  autoinfo.defaultCameraWhiteBalance=false;
  autoinfo.defaultCameraShutter=false;
  autoinfo.defaultCameraGain=false;
  autoinfo.defaultSoftwareWhiteBalance=false;
  autoinfo.defaultSoftwareExposure=false;
  if (typeid (*imageSource)==typeid(Tribots::IIDC)) {
    Tribots::IIDC* cam = dynamic_cast<Tribots::IIDC*>(imageSource);
    autoinfo.defaultCameraWhiteBalance |= (cam->getFeatureMode (Tribots::IIDC::whiteBalance)==Tribots::IIDC::featureAuto);
    autoinfo.defaultCameraShutter |= (cam->getFeatureMode (Tribots::IIDC::shutter)==Tribots::IIDC::featureAuto);
    autoinfo.defaultCameraGain |= (cam->getFeatureMode (Tribots::IIDC::gain)==Tribots::IIDC::featureAuto);
    autoinfo.defaultSoftwareWhiteBalance |= cam->isSoftwareWhiteBalance ();
    autoinfo.defaultSoftwareExposure |= cam->isSoftwareExposure ();
  }
  if (start_message.length()>0)
    statusBar()->message (start_message.c_str());
}


void DistanceCalibrationCollectionWidget::destroy()
{
  if (lastImage)
    delete lastImage;
  if (imageSource)
    delete imageSource;
  if (robotMask)
    delete robotMask;
  if (robotMask2)
    delete robotMask2;
  if (robot)
    delete robot;
  if (wm)
    delete wm;
  if (distMarkerBuilder)
    delete distMarkerBuilder;
}


void DistanceCalibrationCollectionWidget::loop()
{
  unsigned int samplesize=0;
  for (unsigned int i=0; i<colorSample.size(); i++)
    samplesize+=colorSample[i].size();
  if (autoinfo.useAuto && samplesize!=0)
    switchAutoFeatures (false);
  else if (!autoinfo.useAuto && samplesize==0)
    switchAutoFeatures (true);
  Tribots::Image* image=NULL;
  if (checkBoxShowFreeze->isChecked() && lastImage && mode==TribotsTools::DCNormal) {
    image = lastImage->clone();
  } else {
    try{
    image = new Tribots::YUVImage (imageSource->getImage ());
    }catch(std::exception& e){ std::cerr << e.what() << std::endl; }
  }
  imageTimestamp = image->getTimestamp();
  if (classifierChanged) {
    classifier.createFromExamples (colorSample);
    classifierChanged=false;
  }
  image->setClassifier (&classifier);
  if (lastImage) {
    delete lastImage;
    lastImage=NULL;
  }
  Tribots::DriveVector dv;
  switch (mode) {
    case TribotsTools::DCMaskGeneration:
      if (image->getTimestamp().diff_msec (modeStarttime)<2000) {
        if (robot) robot->set_drive_vector (dv);
        imageAnalysis.setRotation (0);
        imageMaskBuilder.init (image->getWidth(), image->getHeight());
      } else if (image->getTimestamp().diff_msec (modeStarttime)<10000) {
        dv.vrot=1.0;
        if (robot) robot->set_drive_vector (dv);
        imageAnalysis.setRotation (1.0);
        imageMaskBuilder.addImage (*image);
      } else {
        mode=TribotsTools::DCNormal;
        checkBoxShowMask->setChecked(true);
        statusBar()->message ("Maskenerzeugung beendet", 5000);
        if (robot) robot->set_drive_vector (dv);
        imageAnalysis.setRotation (0);
        recalculateMask (0);
      }
      break;
    case TribotsTools::DCMarkerCollection:
      if (image->getTimestamp().diff_msec (modeStarttime)<2000) {
        if (robot) robot->set_drive_vector (dv);
        imageAnalysis.setRotation (0);
        imageAnalysis.nextImage (*image, true, true, false, false, false);
        imageWidget->setImage (*image);
      } else if (image->getTimestamp().diff_msec (modeStarttime)<32000) {
        dv.vrot=0.6;
        if (robot) robot->set_drive_vector (dv);
        imageAnalysis.setRotation (0.6);
        std::vector<TribotsTools::MarkerLog> mk = imageAnalysis.nextImage (*image, debug, debug, debug, debug, debug);
        if (debug)
          imageWidget->setImage (*image);
        markers.insert (markers.end(), mk.begin(), mk.end());
      } else {
        mode=TribotsTools::DCNormal;
        distMarkerBuilder->createTable (markers);
        distMarkerBuilderUsed=true;
        statusBar()->message ("Markersammlung beendet", 5000);
        if (robot) robot->set_drive_vector (dv);
        imageAnalysis.setRotation (0);
      }
      break;
    case TribotsTools::DCNormal:
    default:
      //if a red pixel was clicked two seconds ago
      if (updateDirection && lastClick.elapsed_msec() >2000 ) {
        updateDirection = false;
        int xc,yc;
        imageAnalysis.getCenter (xc,yc);
        imageAnalysis.setSearchDirection (Tribots::Angle::rad_angle (std::atan2 (static_cast<double>(recentMouseClickY-yc), static_cast<double>(recentMouseClickX-xc))));
      }

      if (robot) robot->set_drive_vector (dv);
      imageAnalysis.setRotation (0);
      lastImage = image->clone();
      imageAnalysis.setEdgeDetectionSensitivity (sliderCalibrationThreshold->value());
      imageAnalysis.nextImage (*image, checkBoxShowCenter->isChecked(), checkBoxShowDirection->isChecked(), checkBoxShowMask->isChecked(), checkBoxShowSegmentation->isChecked(), checkBoxShowTransitions->isChecked());
      if (checkBoxShowBalance->isChecked()) {
        Tribots::Painter paint (*image);
        Tribots::RGBTuple red = { 196,0,0 };
        paint.setPen (Tribots::Painter::PEN_SOLID);
        paint.setColor (red);
        if (balanceX1<0) balanceX1=0;
        if (balanceX2<0) balanceX2=0;
        if (balanceY1<0) balanceY1=0;
        if (balanceY2<0) balanceY2=0;
        if (balanceX1>=image->getWidth()) balanceX1=image->getWidth()-1;
        if (balanceX2>=image->getWidth()) balanceX2=image->getWidth()-1;
        if (balanceY1>=image->getHeight()) balanceY1=image->getHeight()-1;
        if (balanceY2>=image->getHeight()) balanceY2=image->getHeight()-1;
        paint.drawLine (balanceX1, balanceY1, balanceX2, balanceY1);
        paint.drawLine (balanceX1, balanceY2, balanceX2, balanceY2);
        paint.drawLine (balanceX1, balanceY1, balanceX1, balanceY2);
        paint.drawLine (balanceX2, balanceY1, balanceX2, balanceY2);
        paint.drawXYEllipse (0.5*Vec(balanceX1+balanceX2, balanceY1+balanceY2), 0.5*abs(balanceX1-balanceX2), 0.5*abs(balanceY1-balanceY2));
      }
      if (checkBoxShowMarker->isChecked()) {
        int xc, yc;
        imageAnalysis.getCenter(xc,yc);
        Tribots::Painter paint (*image);
        Tribots::RGBTuple yellow = { 148,148,0 };
        paint.setPen (Tribots::Painter::PEN_SOLID);
        paint.setColor (yellow);
        for (unsigned int i=0; i<markers.size(); i++) {
          paint.drawPoint (static_cast<int>(xc+markers[i].distance*std::cos(markers[i].angle.get_rad())), static_cast<int>(yc+markers[i].distance*std::sin(markers[i].angle.get_rad())));
        }
      }
      imageWidget->setImage (*image);
      break;
  }
  delete image;
}


void DistanceCalibrationCollectionWidget::mouseInImagePressed(QMouseEvent* ev)
{
  {
    int centerX, centerY;
    imageAnalysis.getCenter (centerX, centerY);
    Tribots::RGBTuple rgb;
    lastImage->getPixelRGB (ev->x(),ev->y(), &rgb);
    Tribots::YUVTuple yuv;
    lastImage->getPixelYUV (ev->x(),ev->y(), &yuv);
    std::stringstream inout;
    inout << "(x,y)=" << ev->x() << "," << ev->y();
    inout << "   (phi,d)=" << atan2(static_cast<double>(ev->x()-centerX), static_cast<double>(ev->y()-centerY))*180/M_PI << "," << sqrt(square(static_cast<double>(ev->x()-centerX))+square(static_cast<double>(ev->y()-centerY)));
    inout << "   RGB=" << (int)rgb.r << ',' << (int)rgb.g << ',' << (int)rgb.b << "   Y=" << (int)yuv.y << std::endl;
    std::string line;
    std::getline (inout, line);
    statusBar()->message (line.c_str(), 3000);
  }

  //remember position of click
  recentMouseClickX=ev->x();
  recentMouseClickY=ev->y();
  Tribots::RGBTuple rgb;
  lastImage->getPixelRGB (recentMouseClickX,recentMouseClickY, &rgb);
  switch (mouseMode) {
    case TribotsTools::SetCenterMode:
      imageAnalysis.setCenter (recentMouseClickX,recentMouseClickY);
      break;
    case TribotsTools::SetDirectionMode:
    {
      int xc,yc;
      imageAnalysis.getCenter (xc,yc);
      imageAnalysis.setSearchDirection (Tribots::Angle::rad_angle (std::atan2 (static_cast<double>(recentMouseClickY-yc), static_cast<double>(recentMouseClickX-xc))));
    }
    break;
    case TribotsTools::SetBlueMode:
      colorSample[TribotsTools::DistanceCalibrationImageAnalysis::ColorBluePatch-1].push_back (rgb);
      classifierChanged=true;
      break;
    case TribotsTools::SetRedMode:
      colorSample[TribotsTools::DistanceCalibrationImageAnalysis::ColorRedPatch-1].push_back (rgb);
      classifierChanged=true;

      //remember time of last red pixel clicked
      lastClick.update();
      updateDirection=true;

      break;
    case TribotsTools::SetAddMaskMode:
    case TribotsTools::SetSubMaskMode:
      updateMaskManually (mouseMode==TribotsTools::SetAddMaskMode, recentMouseClickX, recentMouseClickY);
      break;
    case TribotsTools::SetBalanceMode:
      balanceX1=balanceX2=recentMouseClickX;
      balanceY1=balanceY2=recentMouseClickY;
      break;
    default:
      std::cerr << "unbekannter Maus-Modus\n";
  }
}


void DistanceCalibrationCollectionWidget::mouseInImageMoved(QMouseEvent* ev)
{
  switch (mouseMode) {
    case TribotsTools::SetAddMaskMode:
    case TribotsTools::SetSubMaskMode:
      updateMaskManually (mouseMode==TribotsTools::SetAddMaskMode, ev->x(), ev->y());
      break;
    case TribotsTools::SetBalanceMode:
      balanceX2=ev->x();
      balanceY2=ev->y();
      break;
    default:
      break;
  }
}


void DistanceCalibrationCollectionWidget::updateMaskManually( bool doAdd, int x, int y )
{
  if (!robotMask && lastImage)
    robotMask = new Tribots::RobotMask (lastImage->getWidth(), lastImage->getHeight());
  int numX = abs (x-recentMouseClickX);
  int numY = abs (y-recentMouseClickY);
  int num = (numX<numY ? numY : numX)+1;
  int ballstart [] = { -1, -2, -3, -3, -3, -3, -2, -1 };
  int ballend [] = { 2, 3, 4, 4, 4, 4, 3, 2 };
  int zeilen = 8;
  for (int k=0; k<=num; k++) {
    double theta = static_cast<double>(k)/static_cast<double>(num);
    int x1=static_cast<int>(theta*recentMouseClickX+(1-theta)*x);
    int y1=static_cast<int>(theta*recentMouseClickY+(1-theta)*y);
    for (int i=0; i<zeilen; i++) {
      for (int j=ballstart[i]; j<=ballend[i]; j++) {
        robotMask->set(x1+i-zeilen/2,y1+j-1,!doAdd);
      }
    }
  }
  recentMouseClickX=x;
  recentMouseClickY=y;
  imageAnalysis.setMask (robotMask);
}


void DistanceCalibrationCollectionWidget::openMaskClicked()
{
  std::string dotrcdir = std::string(getenv ("HOME"))+"/.robotcontrol";
  robotMaskFile = QFileDialog::getOpenFileName (dotrcdir.c_str(), "*.ppm", this, "Maske laden", "Maske laden").ascii();
  try{
    Tribots::RobotMask* newmask = new Tribots::RobotMask (robotMaskFile.c_str());
    statusBar()->message ("Neue Maske geladen", 3000);
    delete robotMask;
    robotMask=newmask;
    imageAnalysis.setMask (robotMask);
  }catch(Tribots::TribotsException& e){
    statusBar()->message ("Fehler beim Laden der Bildmaske", 3000);
  }
}


void DistanceCalibrationCollectionWidget::saveMaskClicked()
{
  if (robotMaskFile.length()==0) {
    saveMaskAsClicked();
    return;
  }
  saveMask (robotMaskFile);
}


void DistanceCalibrationCollectionWidget::saveMaskAsClicked()
{
  std::string dotrcdir = std::string(getenv ("HOME"))+"/.robotcontrol";
  robotMaskFile = QFileDialog::getSaveFileName (dotrcdir.c_str(), "*.ppm", this, "Maske speichern", "Maske speichern").ascii();
  saveMask (robotMaskFile);
}


void DistanceCalibrationCollectionWidget::saveMask( const std::string & filename )
{
  std::ofstream dest (filename.c_str());
  std::string message;
  if (!dest) {
    message = std::string("Bildmaske konnte nicht in der Datei ")+filename+std::string(" gespeichert werden");
  } else if (!robotMask) {
    message = "Keine Bildmaske vorhanden";
  } else {
    robotMask->writeToStream (dest);
    dest << std::flush;
    message = std::string("Bildmaske gespeichert in Datei ")+filename;
  }
  statusBar()->message(message.c_str(),5000);
}


void DistanceCalibrationCollectionWidget::saveMarker( const std::string & filename )
{
  std::ofstream dest (filename.c_str());
  std::string message;
  if (!dest) {
    message = std::string("Marker konnten nicht in der Datei ")+filename+std::string(" gespeichert werden");
  } else if (markers.size()==0) {
    message = "Keine Marker vorhanden";
  } else {
    for (unsigned int i=0; i<markers.size(); i++) {
      dest << markers[i].angle.get_deg() << ' ' << markers[i].distance << ' ';
      switch (markers[i].type) {
        case TribotsTools::MarkerLog::WB: dest << "wb"; break;
        case TribotsTools::MarkerLog::BW: dest << "bw"; break;
        case TribotsTools::MarkerLog::WR: dest << "wr"; break;
        case TribotsTools::MarkerLog::RM: dest << "rm"; break;
        case TribotsTools::MarkerLog::RW: dest << "rw"; break;
        case TribotsTools::MarkerLog::NL: dest << "nl"; break;
      }
      dest << '\n';
    }
    dest << std::flush;
    message = std::string("Marker gespeichert in Datei ")+filename;
  }
  statusBar()->message(message.c_str(),5000);
}


void DistanceCalibrationCollectionWidget::saveMarkerClicked()
{
  saveMarker (std::string("marker.log"));
}


void DistanceCalibrationCollectionWidget::saveMarkerAsClicked()
{
  std::string filename = QFileDialog::getSaveFileName (".", "*.log", this, "Marker speichern", "Marker speichern").ascii();
  saveMarker (filename);
}


void DistanceCalibrationCollectionWidget::resetRedClicked()
{
  for (unsigned int i=0; i<colorSample.size(); i++)
    colorSample[i].clear();
  classifierChanged=true;
}


void DistanceCalibrationCollectionWidget::switchAutoFeatures( bool on )
{
  Tribots::IIDC::CameraFeatureMode fm (on ? Tribots::IIDC::featureAuto : Tribots::IIDC::featureMan);
  if (autoinfo.defaultCameraWhiteBalance)
    dynamic_cast<Tribots::IIDC*>(imageSource)->setFeatureMode(Tribots::IIDC::whiteBalance, fm);
  if (autoinfo.defaultCameraShutter)
    dynamic_cast<Tribots::IIDC*>(imageSource)->setFeatureMode(Tribots::IIDC::shutter, fm);
  if (autoinfo.defaultCameraGain)
    dynamic_cast<Tribots::IIDC*>(imageSource)->setFeatureMode(Tribots::IIDC::gain, fm);
  if (autoinfo.defaultSoftwareWhiteBalance)
    dynamic_cast<Tribots::IIDC*>(imageSource)->toggleSoftwareWhiteBalance (on);
  if (autoinfo.defaultSoftwareExposure)
    dynamic_cast<Tribots::IIDC*>(imageSource)->toggleSoftwareExposure (on);
  autoinfo.useAuto=on;
}


void DistanceCalibrationCollectionWidget::saveCenterClicked()
{
  std::vector<int> array (2);
  imageAnalysis.getCenter (array[0], array[1]);
  cfg.set ((imageSourceSection+"::image_center").c_str(), array);
  array.resize(4);
  array[0]=(balanceX1<balanceX2 ? balanceX1 : balanceX2);
  array[1]=(balanceY1<balanceY2 ? balanceY1 : balanceY2);
  array[2]=abs(balanceX1-balanceX2);
  array[3]=abs(balanceY1-balanceY2);
  cfg.set ((imageSourceSection+"::balance_area").c_str(),array);
  std::vector<std::string> keys;
  keys.push_back (imageSourceSection+"::image_center");
  keys.push_back (imageSourceSection+"::balance_area");
  cfg.replace_config_file (configfile.c_str(),keys);
}


void DistanceCalibrationCollectionWidget::recalculateMask( int )
{
  if (imageMaskBuilder.numSamples()>10) {
    if (robotMask2)
      delete robotMask2;
    robotMask2 = imageMaskBuilder.generateMask (sliderMaskThreshold->value());
    dilateMask (0);
  }
}


void DistanceCalibrationCollectionWidget::saveExitActionClicked()
{
  saveCenterClicked();
  saveMaskClicked();
  saveMarkerClicked();
  if (distMarkerBuilderUsed)
    saveDistlinesClicked();
  close();
}


void DistanceCalibrationCollectionWidget::commandLineHelp()
{
  std::cerr << "Programm zum Kalibrieren der Tribots.\n";
  std::cerr << " - Erzeugen der Bildmaske\n";
  std::cerr << " - Setzen des Bildmittelpunktes und des Balancebereichs\n";
  std::cerr << " - Distanzkalibrierung der Omnikamera\n\n";
  std::cerr << "Kommandozeilenoptionen:\n";
  std::cerr << " Konfigurationsdatei: eine Datei a la robotcontrol.cfg\n";
  std::cerr << "   als default wird ../config_files/robotcontrol.cfg verwendet\n";
  std::cerr << " --help, -h: Anzeigen diese Hilfe\n";
  std::cerr << " --debug, -d: Debugausgabe waehrend des Sammelns der Kalibriermerkmale\n";
  std::cerr << " -o: Omnidirektionale Kamera verwenden\n";
  std::cerr << " -p: Perspektivische Kamera verwenden\n";
  std::cerr << " -f FILE: Bilder aus einer Datei lesen\n\n";
  std::cerr << "Funktionsweise der Bildmaskenerzeugung:\n";
  std::cerr << " 1. Roboter an kontrastreichen Ort stellen\n";
  std::cerr << " 2. 'Maskengenerierung' anklicken. Der Roboter dreht sich.\n";
  std::cerr << " 3. Ergebnis korrigieren. Mit den beiden Schiebern oder manuell korrigieren\n\n";
  std::cerr << "Funktionsweise der Distanzkalibrierung:\n";
  std::cerr << " 1. Kalibriertapete ausrollen, Roboter an die Nullposition stellen\n";
  std::cerr << " 2. Bildmittelpunkt korrekt waehlen\n";
  std::cerr << " 3. Rote Pixel aus dem roten Tapentenbereich anklicken, so dass der rote\n";
  std::cerr << "   segmentiert wird. Andere mit segmentierte Bereiche sind okay, so lange\n";
  std::cerr << "   sie nicht direkt hinter der Tapete liegen oder direkt benachbart sind\n";
  std::cerr << " 4. Die Richtung vorgeben, in der die Tapete ausliegt\n";
  std::cerr << " 5. 'Distanzkalibrierung' anklicken. Der Roboter dreht sich langsam und sammelt\n";
  std::cerr << "   dabei die Kalibriermerkmale auf.\n";
  std::cerr << " 6. Ergebnis ueberpruefen und speichern.\n";
  std::cerr << " 7. Mit dem MarkerEditor nochmals pruefen\n";
}


void DistanceCalibrationCollectionWidget::dilateMask( int )
{
  if (robotMask2) {
    if (robotMask)
      delete robotMask;
    robotMask = imageMaskBuilder.dilateMask (robotMask2, sliderMaskDilation->value());
    imageAnalysis.setMask (robotMask);
  }
}


void DistanceCalibrationCollectionWidget::generateCenterBalanceArea()
{
  if (!lastImage) {
    statusBar()->message ("Kein Bild verf�gbar zum Bestimmen des Bildmittelpunktes und Balancebereichs");
    return;
  }
  double ccx, ccy, ccrmin, ccrmax;
  Tribots::RGBImage dummyOut (lastImage->getWidth(), lastImage->getHeight());
  if (Tribots::findCenterRing (ccx, ccy, ccrmin, ccrmax, dummyOut, *lastImage, false)) {
    imageAnalysis.setCenter (static_cast<int>(ccx), static_cast<int>(ccy));
    unsigned int x1, x2, y1, y2;
    Tribots::determineBalanceArea (x1, y1, x2, y2, ccx, ccy, ccrmin, ccrmax);
    balanceX1=x1;
    balanceX2=x2;
    balanceY1=y1;
    balanceY2=y2;
    return;
  }
  statusBar()->message ("Bestimmen des Bildmittelpunktes und Balancebereichs fehlgeschlagen. Gelber Ring vorhanden?");
}


void DistanceCalibrationCollectionWidget::saveDistlinesClicked()
{
  if (distMarkerBuilderUsed) {
    std::string filename = std::string(std::getenv("HOME"))+"/.robotcontrol/dist_marker.cfg";
    saveDistlines (filename.c_str());
  } else {
    statusBar()->message ("Speicher von dist_marker.cfg nicht m�glich, da noch keine Kalibrierung durchgef�hrt");
  }
}


void DistanceCalibrationCollectionWidget::saveDistlinesAsClicked()
{
  if (distMarkerBuilderUsed) {
    std::string filename = QFileDialog::getSaveFileName (".", "*.cfg", this, "dist_marker.cfg speichern", "dist_marker.cfg speichern").ascii();
    saveDistlines (filename);
  } else {
    statusBar()->message ("Speicher von dist_marker.cfg nicht m�glich, da noch keine Kalibrierung durchgef�hrt");
  }
}


void DistanceCalibrationCollectionWidget::saveDistlines( const std::string & filename )
{
  std::ofstream dest (filename.c_str());
  if (!dest) {
    statusBar()->message ((std::string("Distanzlinien konnten nicht in der Datei ")+filename+std::string(" gespeichert werden")).c_str());
    return;
  }
  unsigned int iw=22;
  unsigned int ih=22;
  if (lastImage) {
    iw=lastImage->getWidth();
    ih=lastImage->getHeight();
  }
  int centerX, centerY;
  imageAnalysis.getCenter (centerX, centerY);
  distMarkerBuilder->writeTable (dest, iw, ih, centerX, centerY);
}
