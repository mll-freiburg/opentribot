
#ifndef _TribotsTools_ScrollImageWidget_h_
#define _TribotsTools_ScrollImageWidget_h_

#include "ImageWidget.h"
#include <QtGui/QScrollArea>

namespace TribotsTools{

  /** Qt-Widget zur Darstellung eines Bildes im Tribots-Format, mit Scrollbalken */
  class ScrollImageWidget : public QScrollArea
  {
  Q_OBJECT

  public:
    /** Konstruktor erzeugt ein leeres Bild */
    ScrollImageWidget(QWidget* parent = 0, Qt::WFlags f=0);

    /** Konstruktor uebernimmt als Bild arg1 */
    ScrollImageWidget(const Tribots::Image& image, QWidget* parent = 0, Qt::WFlags f = Qt::Window);

    ~ScrollImageWidget();

  signals:
    void mousePressed(QMouseEvent*);
    void mouseMoved(QMouseEvent*);
    void keyPressed(QKeyEvent*);

  public slots:
    /** den Bildinhalt durch arg1 ersetzen */
    virtual void setImage(const Image& new_image);
    /** den Bildinhalt durch arg1 ersetzen */
    virtual void setImage(const ImageBuffer& new_image);
    /** Bild zentrieren */
    virtual void centerImage ();

  protected slots:
    void mousePressEvent(QMouseEvent*);
    void mouseMoveEvent(QMouseEvent*);
    void keyPressEvent(QKeyEvent*);

  protected:
    ImageWidget* imgWidget;
    void init ();
  };
}

#endif
