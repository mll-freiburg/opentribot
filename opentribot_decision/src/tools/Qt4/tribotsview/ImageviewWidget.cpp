
#include "ImageviewWidget.h"
#include <iostream>
#include <fstream>
#include <string>
#include <QtGui/QMessageBox>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

ImageviewWidget::ImageviewWidget (QWidget * parent, Qt::WindowFlags flags) : QWidget(parent, flags) {
  setupUi (this);
  connect (freeze,SIGNAL(toggled(bool)),this,SLOT(toggleFreeze(bool)));
  imgSource = 0;
  freezed = false;
}

void ImageviewWidget::showImage(int cycle)
{
  if (! imgSource) {  // keine Bildquelle ausgewählt
    return ;
  }
  if (! isVisible()) {  // wird eh nicht angezeigt
    return ;
  }
  if (freezed) {       // aktuelles Bild eingefroren
    return;
  }
  ImageBuffer* image = imgSource->getImageNumber(cycle, 0);
  imageWidget->setImage(*image);
  filenameInfo->setText(imgSource->getFilename().c_str());
  timestampInfo->setText(QString::number(image->timestamp.get_msec()));
  delete image;
}

void ImageviewWidget::toggleFreeze(bool b)
{
  freezed = b;
}

void ImageviewWidget::loadImages( QString filename, int cycle )
{
  ifstream in(filename.toAscii());
  if (!in) {
    QMessageBox::information(this, "File access error", "Could not open " + filename + " for reading");
  }

  try{
    imgSource = new FileSource(std::string(filename.toAscii()));
  }catch(Tribots::BadHardwareException& e){
    QMessageBox::information(this, "Wrong format", e.what());
    return;
  }

  showImage(cycle);
}
