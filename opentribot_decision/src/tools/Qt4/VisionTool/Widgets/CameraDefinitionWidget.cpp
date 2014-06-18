
#include "CameraDefinitionWidget.h"
#include <cmath>
#include "../../../../ImageProcessing/Formation/Painter.h"
#include <QtGui/QFileDialog>
#include <cstdlib>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

CameraDefinitionWidget::CameraDefinitionWidget(VisionToolImageSource& is, Tribots::ConfigReader& cfg, QStatusBar& stb, QWidget* p, Qt::WindowFlags f) : VisionToolWidget (cfg,stb,p,f),  imageSource(is) {
  setupUi (this);
  connect(pushButton_start,SIGNAL(clicked()),this,SLOT(startPressed()));
  connect(pushButton_stop,SIGNAL(clicked()),this,SLOT(stopPressed()));
  connect(pushButton_restart,SIGNAL(clicked()),this,SLOT(restartPressed()));
  connect(pushButton_device,SIGNAL(clicked()),this,SLOT(selectDevicePressed()));
  setFocusPolicy (Qt::StrongFocus);

  vector<string> uids = imageSource.getUIDs ();
  for (unsigned int i=0; i<uids.size(); i++) {
    comboBox_uid->addItem (uids[i].c_str());
  }
}

CameraDefinitionWidget::~CameraDefinitionWidget () {;}

void CameraDefinitionWidget::takeArguments () {
  device = std::string(lineEdit_device->text().toAscii());
  mode = std::string(lineEdit_mode->text().toAscii());
  name = std::string(lineEdit_name->text().toAscii());
  uid = std::string(comboBox_uid->currentText().toAscii());
  delay = spinBox_delay->value();
  do_selftest = checkBox_selftest->isChecked();
  do_blocking = checkBox_blocking->isChecked();
  std::string section;
  config.get ("VisionTool::Section", section);
  section+="::";
  is_fileSource=(device.substr(0,5)!="/dev/");
  if (is_fileSource) {
    config.set ((section+"image_source_type").c_str(), std::string("FileSource"));
    config.set ((section+"filename").c_str(), device);
  } else {
    config.set ((section+"image_source_type").c_str(), std::string("CameraSource"));
    config.set ((section+"uid").c_str(), uid);
    config.set ((section+"device_name").c_str(), device);
    config.set ((section+"port").c_str(), 0);
    config.set ((section+"mode").c_str(), mode);
    config.set ((section+"blocking").c_str(), do_blocking);
    config.set ((section+"delay").c_str(), delay);
    config.set ((section+"self_test").c_str(), do_selftest);
  }
}

void CameraDefinitionWidget::startPressed() {
  if (imageSource.isStarted()) {
    textEdit_messages->append ("Fehler: Bildquelle bereits gestartet.\n");
    return;
  }
  takeArguments();
  std::string message = "";
  try{
    if (device.substr(0,5)!="/dev/") {
      message+="Versuche, Bilder aus Datei zu lesen: "+device+"\n";
      imageSource.startFileSource ();
      is_fileSource=true;
    } else {
      message+="Versuche, Kamera zu starten mit UID "+uid+" auf Device "+device+"\n";
      imageSource.startCameraSource ();
      is_fileSource=false;
    }
    message+="Bildquelle erfolgreich gestartet.\n";
  }catch(Tribots::TribotsException& e) {
    message+="Fehler beim Starten der Bildquelle: ";
    message+=e.what();
    message+="\n";
  }
  textEdit_messages->append(message.c_str());
  imageWidget->centerImage();
  return;
}

void CameraDefinitionWidget::stopPressed() {
  imageSource.stop();
  textEdit_messages->append("Bildquelle gestoppt\n");
}

void CameraDefinitionWidget::restartPressed() {
  stopPressed();
  startPressed();
}

void CameraDefinitionWidget::selectDevicePressed() {
  std::string home=getenv ("HOME");
  QString file = QFileDialog::getOpenFileName (this, "Device oder Datei wählen", home.c_str(), "Logfiles (*.log)");
  if (file!="") {
    lineEdit_device->setText (file);
  }
}



void CameraDefinitionWidget::start () {
  std::string section;
  config.get ("VisionTool::Section", section);
  name = section;
  section+="::";

  std::string s="CameraSource";
  config.get ((section+"image_source_type").c_str(), s);
  is_fileSource=(s=="FileSource");
  if (is_fileSource) {
    config.get ((section+"filename").c_str(), device);
    mode=uid="";
    do_selftest=do_blocking=true;
    delay=60;
  } else {
    device = "/dev/video1394/0";
    config.get ((section+"device_name").c_str(), device);
    mode = "Format0 640x480 YUV422 30fps";
    config.getline ((section+"mode").c_str(), mode);
    uid = "";
    config.get ((section+"uid").c_str(), uid);
    delay=60;
    config.get ((section+"delay").c_str(), delay);
    do_selftest=true;
    config.get ((section+"self_test").c_str(), do_selftest);
    do_blocking=true;
    config.get ((section+"blocking").c_str(), do_blocking);
  }

  lineEdit_name->setText(name.c_str());
  lineEdit_device->setText(device.c_str());
  lineEdit_mode->setText(mode.c_str());
  spinBox_delay->setValue(delay);
  checkBox_selftest->setChecked(do_selftest);
  checkBox_blocking->setChecked(do_blocking);
  int index=-1;
  for (int i=0; i<comboBox_uid->count(); i++) {
    if (uid==std::string(comboBox_uid->itemText(i).toAscii())) {
      index=i;
    }
  }
  if (index==-1) {
    comboBox_uid->addItem (uid.c_str());
    index=comboBox_uid->count()-1;
  }
  comboBox_uid->setCurrentIndex(index);
  imageWidget->centerImage();
}

void CameraDefinitionWidget::stop () {
  takeArguments();
  imageSource.setMode ();  // default-Modus, tut nichts
}

void CameraDefinitionWidget::loop () {
  Tribots::Image& image (imageSource.getImage());
  imageWidget->setImage (image);
}
