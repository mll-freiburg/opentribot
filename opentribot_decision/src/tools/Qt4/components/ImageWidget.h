
#ifndef _TribotsTools_ImageWidget_h_
#define _TribotsTools_ImageWidget_h_

/*
 * Description: Qt widget, that holds and displays an image.
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

#include <QtGui/QApplication>
#include <QtGui/QWidget>
#include <QtGui/QImage>
#include <QtGui/QPaintEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QKeyEvent>
#include "../../../ImageProcessing/Formation/Image.h"

namespace TribotsTools{

  using Tribots::Image;
  using Tribots::ImageBuffer;

  /** Qt-Widget zur Darstellung eines Bildes im Tribots-Format */
  class ImageWidget : public QWidget
  {
  Q_OBJECT

  public:
    /** Konstruktor erzeugt ein leeres bild */
    ImageWidget(QWidget* parent = 0, Qt::WFlags f=0);

    /** Konstruktor uebernimmt als Bild arg1 */
    ImageWidget(const Tribots::Image& image, QWidget* parent = 0, 
		 Qt::WFlags f = Qt::Window);

  signals:
    void mousePressed(QMouseEvent*);
    void mouseMoved(QMouseEvent*);
    void keyPressed(QKeyEvent*);

  public slots:
    /** den Bildinhalt durch arg1 ersetzen */
    virtual void setImage(const Image& new_image);
    /** den Bildinhalt durch arg1 ersetzen */
    virtual void setImage(const ImageBuffer& new_image);

  protected:
    void paintEvent(QPaintEvent *ev);
    void mousePressEvent(QMouseEvent*);
    void mouseMoveEvent(QMouseEvent*);
    void keyPressEvent(QKeyEvent*);

  protected:
    QImage qt_image;
  };
}

#endif
