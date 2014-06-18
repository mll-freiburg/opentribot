/****************************************************************************
** Form interface generated from reading ui file 'ImageviewWidget.ui'
**
** Created: Mon Aug 31 14:45:19 2009
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#ifndef IMAGEVIEWWIDGET_H
#define IMAGEVIEWWIDGET_H

#include <qvariant.h>
#include <qpixmap.h>
#include <qwidget.h>
#include "../../ImageProcessing/Formation/FileSource.h"

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
namespace TribotsTools {
class ImageWidget;
}
class QLabel;
class QCheckBox;
class QLineEdit;

class ImageviewWidget : public QWidget
{
    Q_OBJECT

public:
    ImageviewWidget( QWidget* parent = 0, const char* name = 0, WFlags fl = 0 );
    ~ImageviewWidget();

    QLabel* textLabel2;
    TribotsTools::ImageWidget* imageWidget;
    QCheckBox* freeze;
    QLabel* textLabel3;
    QLineEdit* timestampInfo;
    QLineEdit* filenameInfo;

public slots:
    virtual void showImage( int cycle );
    virtual void toggleFreeze( bool b );
    virtual void loadImages( QString filename, int cycle );

protected:
    Tribots::FileSource *imgSource;
    int freezed;


protected slots:
    virtual void languageChange();

private:
    QPixmap image0;

    void init();

};

#endif // IMAGEVIEWWIDGET_H
