/****************************************************************************
** Form implementation generated from reading ui file 'ImageviewWidget.ui'
**
** Created: Mon Aug 31 14:45:22 2009
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#include "ImageviewWidget.h"

#include <qvariant.h>
#include <qlabel.h>
#include <qcheckbox.h>
#include <qlineedit.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qimage.h>
#include <qpixmap.h>

#include "../components/ImageWidget.h"
#include "ImageviewWidget.ui.h"
/*
 *  Constructs a ImageviewWidget as a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'.
 */
ImageviewWidget::ImageviewWidget( QWidget* parent, const char* name, WFlags fl )
    : QWidget( parent, name, fl )
{
    if ( !name )
	setName( "ImageviewWidget" );

    textLabel2 = new QLabel( this, "textLabel2" );
    textLabel2->setGeometry( QRect( 0, 490, 74, 20 ) );

    imageWidget = new TribotsTools::ImageWidget( this, "imageWidget" );
    imageWidget->setGeometry( QRect( 0, 0, 640, 480 ) );
    imageWidget->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, 0, 0, imageWidget->sizePolicy().hasHeightForWidth() ) );

    freeze = new QCheckBox( this, "freeze" );
    freeze->setGeometry( QRect( 540, 490, 96, 22 ) );

    textLabel3 = new QLabel( this, "textLabel3" );
    textLabel3->setGeometry( QRect( 350, 490, 80, 20 ) );

    timestampInfo = new QLineEdit( this, "timestampInfo" );
    timestampInfo->setGeometry( QRect( 430, 490, 90, 22 ) );
    timestampInfo->setReadOnly( TRUE );

    filenameInfo = new QLineEdit( this, "filenameInfo" );
    filenameInfo->setEnabled( TRUE );
    filenameInfo->setGeometry( QRect( 80, 490, 250, 22 ) );
    QPalette pal;
    QColorGroup cg;
    cg.setColor( QColorGroup::Foreground, black );
    cg.setColor( QColorGroup::Button, QColor( 238, 234, 222) );
    cg.setColor( QColorGroup::Light, white );
    cg.setColor( QColorGroup::Midlight, QColor( 255, 255, 252) );
    cg.setColor( QColorGroup::Dark, QColor( 85, 85, 82) );
    cg.setColor( QColorGroup::Mid, QColor( 198, 198, 191) );
    cg.setColor( QColorGroup::Text, black );
    cg.setColor( QColorGroup::BrightText, white );
    cg.setColor( QColorGroup::ButtonText, black );
    cg.setColor( QColorGroup::Base, white );
    cg.setColor( QColorGroup::Background, QColor( 238, 238, 230) );
    cg.setColor( QColorGroup::Shadow, black );
    cg.setColor( QColorGroup::Highlight, QColor( 255, 221, 118) );
    cg.setColor( QColorGroup::HighlightedText, black );
    cg.setColor( QColorGroup::Link, QColor( 0, 0, 192) );
    cg.setColor( QColorGroup::LinkVisited, QColor( 128, 0, 128) );
    pal.setActive( cg );
    cg.setColor( QColorGroup::Foreground, black );
    cg.setColor( QColorGroup::Button, QColor( 238, 234, 222) );
    cg.setColor( QColorGroup::Light, white );
    cg.setColor( QColorGroup::Midlight, QColor( 255, 255, 252) );
    cg.setColor( QColorGroup::Dark, QColor( 85, 85, 82) );
    cg.setColor( QColorGroup::Mid, QColor( 198, 198, 191) );
    cg.setColor( QColorGroup::Text, black );
    cg.setColor( QColorGroup::BrightText, white );
    cg.setColor( QColorGroup::ButtonText, black );
    cg.setColor( QColorGroup::Base, white );
    cg.setColor( QColorGroup::Background, QColor( 238, 238, 230) );
    cg.setColor( QColorGroup::Shadow, black );
    cg.setColor( QColorGroup::Highlight, QColor( 255, 221, 118) );
    cg.setColor( QColorGroup::HighlightedText, black );
    cg.setColor( QColorGroup::Link, QColor( 0, 0, 192) );
    cg.setColor( QColorGroup::LinkVisited, QColor( 128, 0, 128) );
    pal.setInactive( cg );
    cg.setColor( QColorGroup::Foreground, QColor( 128, 128, 128) );
    cg.setColor( QColorGroup::Button, QColor( 238, 234, 222) );
    cg.setColor( QColorGroup::Light, white );
    cg.setColor( QColorGroup::Midlight, QColor( 255, 255, 252) );
    cg.setColor( QColorGroup::Dark, QColor( 85, 85, 82) );
    cg.setColor( QColorGroup::Mid, QColor( 198, 198, 191) );
    cg.setColor( QColorGroup::Text, QColor( 198, 198, 191) );
    cg.setColor( QColorGroup::BrightText, white );
    cg.setColor( QColorGroup::ButtonText, QColor( 128, 128, 128) );
    cg.setColor( QColorGroup::Base, white );
    cg.setColor( QColorGroup::Background, QColor( 238, 238, 230) );
    cg.setColor( QColorGroup::Shadow, black );
    cg.setColor( QColorGroup::Highlight, QColor( 212, 184, 98) );
    cg.setColor( QColorGroup::HighlightedText, white );
    cg.setColor( QColorGroup::Link, QColor( 0, 0, 192) );
    cg.setColor( QColorGroup::LinkVisited, QColor( 128, 0, 128) );
    pal.setDisabled( cg );
    filenameInfo->setPalette( pal );
    filenameInfo->setBackgroundOrigin( QLineEdit::WidgetOrigin );
    filenameInfo->setFrameShadow( QLineEdit::Sunken );
    filenameInfo->setFrame( TRUE );
    filenameInfo->setReadOnly( TRUE );
    languageChange();
    resize( QSize(640, 518).expandedTo(minimumSizeHint()) );
    clearWState( WState_Polished );

    // signals and slots connections
    connect( freeze, SIGNAL( toggled(bool) ), this, SLOT( toggleFreeze(bool) ) );
    init();
}

/*
 *  Destroys the object and frees any allocated resources
 */
ImageviewWidget::~ImageviewWidget()
{
    // no need to delete child widgets, Qt does it all for us
}

/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void ImageviewWidget::languageChange()
{
    setCaption( tr( "ImageviewWidget" ) );
    QToolTip::add( this, tr( "Zeigt aufgezeichnete Bilder an" ) );
    textLabel2->setText( tr( "Dateiname:" ) );
    freeze->setText( tr( "einfrieren" ) );
    QToolTip::add( freeze, tr( "Anzeige einfieren" ) );
    textLabel3->setText( tr( "Zeitstempel:" ) );
}

