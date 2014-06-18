
#ifndef _TribotsTools_ImageviewWidget_h_
#define _TribotsTools_ImageviewWidget_h_

#include "UI/ImageviewWidget.h"
#include <QtGui/QWidget>
#include <QtGui/QPixmap>
#include <QtCore/QString>
#include "../../../ImageProcessing/Formation/FileSource.h"

namespace TribotsTools {

  class ImageviewWidget : public QWidget, private Ui::ImageviewWidget {
    Q_OBJECT

  public:
    ImageviewWidget(QWidget* =0, Qt::WindowFlags =0);
    virtual ~ImageviewWidget () {;}

  public slots:
    virtual void showImage(int);
    virtual void toggleFreeze(bool);
    virtual void loadImages( QString, int);

  private:
    Tribots::FileSource *imgSource;
    int freezed;
    QPixmap image0;
  };

}

#endif
