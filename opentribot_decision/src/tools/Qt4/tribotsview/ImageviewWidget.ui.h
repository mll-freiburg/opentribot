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
#include <iostream>
#include <fstream>
#include <qmessagebox.h>
#include <string>

using namespace std;
using namespace Tribots;

void ImageviewWidget::init()
{
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
  filenameInfo->setText(imgSource->getFilename());
  timestampInfo->setText(QString::number(image->timestamp.get_msec()));
  delete image;
}

void ImageviewWidget::toggleFreeze(bool b)
{
  freezed = b;
}

void ImageviewWidget::loadImages( QString filename, int cycle )
{
  ifstream in(filename);
  if (!in) {
    QMessageBox::information(this, "File access error", "Could not open " + filename + " for reading");
  }

  try{
    imgSource = new FileSource(filename);
  }catch(Tribots::BadHardwareException& e){
    QMessageBox::information(this, "Wrong format", e.what());
    return;
  }
  
  showImage(cycle);
}
