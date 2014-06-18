/*
 * Description: Implementation of the ImageWidget.
 * Created:     2004-05-10
 * Author:      Sascha Lange
 * Mail:        Sascha.Lange@Uni-Osnabrueck.De
 *
 * Copyright (C) 2001, 2004 Sascha Lange
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 
 * 02111-1307, USA.
 *
 * --------------------------------------------------------------------
 */

#include "ImageWidget.h"
#include <QtGui/QPainter>
#include "../../../ImageProcessing/Formation/RGBImage.h"

TribotsTools::ImageWidget::ImageWidget(QWidget* parent,
				    Qt::WFlags f)
  : QWidget(parent, f), qt_image (40,40,QImage::Format_RGB32)
{
  setFocusPolicy(Qt::StrongFocus); // um Key-Events zu bekommen
  Tribots::RGBImage image(640,480);  // empty image
  setMinimumSize(40, 40);
// QT3-4-PROBLEM?  setBackgroundMode(Qt::NoBackground);
  setImage(image);
}

TribotsTools::ImageWidget::ImageWidget(const Tribots::Image& image, 
				    QWidget* parent,
				    Qt::WFlags f)
  : QWidget(parent, f), qt_image (40,40,QImage::Format_RGB32)
{
  setFocusPolicy(Qt::StrongFocus); // um Key-Events zu bekommen
  setMinimumSize(60, 40);
// QT3-4-PROBLEM?  setBackgroundMode(NoBackground);
  setImage(image);
}

void TribotsTools::ImageWidget::mousePressEvent(QMouseEvent* ev)
{
  emit mousePressed(ev);
}

void TribotsTools::ImageWidget::mouseMoveEvent(QMouseEvent* ev)
{
  emit mouseMoved(ev);
}

void TribotsTools::ImageWidget::keyPressEvent(QKeyEvent* ev)
{
  emit keyPressed(ev);
}

void TribotsTools::ImageWidget::paintEvent(QPaintEvent*)
{
  QPainter p(this);
  p.drawImage(0,0,qt_image);
}

void TribotsTools::ImageWidget::setImage(const Tribots::ImageBuffer& new_image) {
  Tribots::RGBImage img (new_image);
  setImage (img);
}

void TribotsTools::ImageWidget::setImage(const Tribots::Image& new_image)
{
  bool resized = false;
  unsigned int w = new_image.getWidth();
  unsigned int h = new_image.getHeight();
  if (w!=static_cast<unsigned int>(qt_image.width()) || h!=static_cast<unsigned int>(qt_image.height())) {
    resized=true;
    qt_image = QImage (w,h,QImage::Format_RGB32);
  }

  Tribots::RGBTuple rgb;
  unsigned char* dest = qt_image.scanLine(0);
  for (unsigned int y=0; y<h; y++) {
    for (unsigned int x=0; x<w; x++) {
      new_image.getPixelRGB (x,y, &rgb);
      (*reinterpret_cast<QRgb*>(dest)) = qRgb (rgb.r, rgb.g, rgb.b);
      dest+=4;
    }
  }

  if (resized) {
    setMaximumSize(w, h);
    resize(w, h);
  } else {
    repaint();            // unfortunately update's to slow!
  }
}
