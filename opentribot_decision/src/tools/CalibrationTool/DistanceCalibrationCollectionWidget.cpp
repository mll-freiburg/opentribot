/****************************************************************************
** Form implementation generated from reading ui file 'DistanceCalibrationCollectionWidget.ui'
**
** Created: Tue Mar 30 12:43:43 2010
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#include "DistanceCalibrationCollectionWidget.h"

#include <qvariant.h>
#include <qpushbutton.h>
#include <qgroupbox.h>
#include <qcheckbox.h>
#include <qbuttongroup.h>
#include <qradiobutton.h>
#include <qlabel.h>
#include <qslider.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qaction.h>
#include <qmenubar.h>
#include <qpopupmenu.h>
#include <qtoolbar.h>
#include "../components/ScrollImageWidget.h"
#include "DistanceCalibrationCollectionWidget.ui.h"

/*
 *  Constructs a DistanceCalibrationCollectionWidget as a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'.
 *
 */
DistanceCalibrationCollectionWidget::DistanceCalibrationCollectionWidget( QWidget* parent, const char* name, WFlags fl )
    : QMainWindow( parent, name, fl )
{
    (void)statusBar();
    if ( !name )
	setName( "DistanceCalibrationCollectionWidget" );
    setMinimumSize( QSize( 714, 462 ) );
    QFont f( font() );
    f.setFamily( "Andale Mono" );
    setFont( f ); 
    setCentralWidget( new QWidget( this, "qt_central_widget" ) );
    DistanceCalibrationCollectionWidgetLayout = new QGridLayout( centralWidget(), 1, 1, 11, 6, "DistanceCalibrationCollectionWidgetLayout"); 

    imageWidget = new TribotsTools::ScrollImageWidget( centralWidget(), "imageWidget" );
    imageWidget->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, 0, 0, imageWidget->sizePolicy().hasHeightForWidth() ) );
    imageWidget->setMinimumSize( QSize( 200, 200 ) );

    DistanceCalibrationCollectionWidgetLayout->addMultiCellWidget( imageWidget, 0, 0, 0, 4 );

    pushButtonAutoCenter = new QPushButton( centralWidget(), "pushButtonAutoCenter" );

    DistanceCalibrationCollectionWidgetLayout->addWidget( pushButtonAutoCenter, 1, 0 );

    pushButtonResetRed = new QPushButton( centralWidget(), "pushButtonResetRed" );
    pushButtonResetRed->setCursor( QCursor( 0 ) );

    DistanceCalibrationCollectionWidgetLayout->addWidget( pushButtonResetRed, 1, 1 );

    pushButtonBuildMarker = new QPushButton( centralWidget(), "pushButtonBuildMarker" );
    pushButtonBuildMarker->setCursor( QCursor( 0 ) );

    DistanceCalibrationCollectionWidgetLayout->addWidget( pushButtonBuildMarker, 1, 2 );

    pushButtonBuildMask = new QPushButton( centralWidget(), "pushButtonBuildMask" );
    pushButtonBuildMask->setCursor( QCursor( 0 ) );
    pushButtonBuildMask->setFlat( FALSE );

    DistanceCalibrationCollectionWidgetLayout->addWidget( pushButtonBuildMask, 1, 3 );

    pushButtonStop = new QPushButton( centralWidget(), "pushButtonStop" );
    pushButtonStop->setCursor( QCursor( 0 ) );

    DistanceCalibrationCollectionWidgetLayout->addWidget( pushButtonStop, 1, 4 );

    layout1 = new QVBoxLayout( 0, 0, 0, "layout1"); 

    groupBox1 = new QGroupBox( centralWidget(), "groupBox1" );
    groupBox1->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)1, (QSizePolicy::SizeType)5, 0, 0, groupBox1->sizePolicy().hasHeightForWidth() ) );
    groupBox1->setCursor( QCursor( 0 ) );
    groupBox1->setColumnLayout(0, Qt::Vertical );
    groupBox1->layout()->setSpacing( 0 );
    groupBox1->layout()->setMargin( 5 );
    groupBox1Layout = new QGridLayout( groupBox1->layout() );
    groupBox1Layout->setAlignment( Qt::AlignTop );

    checkBoxShowMask = new QCheckBox( groupBox1, "checkBoxShowMask" );
    checkBoxShowMask->setChecked( TRUE );

    groupBox1Layout->addWidget( checkBoxShowMask, 2, 0 );

    checkBoxShowSegmentation = new QCheckBox( groupBox1, "checkBoxShowSegmentation" );
    checkBoxShowSegmentation->setChecked( TRUE );

    groupBox1Layout->addWidget( checkBoxShowSegmentation, 3, 0 );

    checkBoxShowTransitions = new QCheckBox( groupBox1, "checkBoxShowTransitions" );

    groupBox1Layout->addWidget( checkBoxShowTransitions, 4, 0 );

    checkBoxShowBalance = new QCheckBox( groupBox1, "checkBoxShowBalance" );
    checkBoxShowBalance->setChecked( TRUE );

    groupBox1Layout->addWidget( checkBoxShowBalance, 5, 0 );

    checkBoxShowMarker = new QCheckBox( groupBox1, "checkBoxShowMarker" );

    groupBox1Layout->addWidget( checkBoxShowMarker, 6, 0 );

    checkBoxShowFreeze = new QCheckBox( groupBox1, "checkBoxShowFreeze" );

    groupBox1Layout->addWidget( checkBoxShowFreeze, 7, 0 );

    checkBoxShowDirection = new QCheckBox( groupBox1, "checkBoxShowDirection" );
    checkBoxShowDirection->setChecked( TRUE );

    groupBox1Layout->addWidget( checkBoxShowDirection, 1, 0 );

    checkBoxShowCenter = new QCheckBox( groupBox1, "checkBoxShowCenter" );
    checkBoxShowCenter->setChecked( TRUE );

    groupBox1Layout->addWidget( checkBoxShowCenter, 0, 0 );
    layout1->addWidget( groupBox1 );

    buttonGroup1 = new QButtonGroup( centralWidget(), "buttonGroup1" );
    buttonGroup1->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)1, (QSizePolicy::SizeType)5, 0, 0, buttonGroup1->sizePolicy().hasHeightForWidth() ) );
    buttonGroup1->setCursor( QCursor( 0 ) );
    buttonGroup1->setProperty( "selectedId", 3 );
    buttonGroup1->setColumnLayout(0, Qt::Vertical );
    buttonGroup1->layout()->setSpacing( 0 );
    buttonGroup1->layout()->setMargin( 5 );
    buttonGroup1Layout = new QGridLayout( buttonGroup1->layout() );
    buttonGroup1Layout->setAlignment( Qt::AlignTop );

    radioButtonSetCenter = new QRadioButton( buttonGroup1, "radioButtonSetCenter" );

    buttonGroup1Layout->addWidget( radioButtonSetCenter, 0, 0 );

    radioButtonSetDirection = new QRadioButton( buttonGroup1, "radioButtonSetDirection" );

    buttonGroup1Layout->addWidget( radioButtonSetDirection, 1, 0 );

    radioButtonSetBalance = new QRadioButton( buttonGroup1, "radioButtonSetBalance" );

    buttonGroup1Layout->addWidget( radioButtonSetBalance, 5, 0 );

    radioButtonSetRed = new QRadioButton( buttonGroup1, "radioButtonSetRed" );
    radioButtonSetRed->setChecked( TRUE );

    buttonGroup1Layout->addWidget( radioButtonSetRed, 2, 0 );

    radioButtonSetMaskAdd = new QRadioButton( buttonGroup1, "radioButtonSetMaskAdd" );

    buttonGroup1Layout->addWidget( radioButtonSetMaskAdd, 3, 0 );

    radioButtonSetMaskSub = new QRadioButton( buttonGroup1, "radioButtonSetMaskSub" );

    buttonGroup1Layout->addWidget( radioButtonSetMaskSub, 4, 0 );
    layout1->addWidget( buttonGroup1 );

    groupBox2 = new QGroupBox( centralWidget(), "groupBox2" );
    groupBox2->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)1, (QSizePolicy::SizeType)5, 0, 0, groupBox2->sizePolicy().hasHeightForWidth() ) );
    groupBox2->setCursor( QCursor( 0 ) );
    groupBox2->setColumnLayout(0, Qt::Vertical );
    groupBox2->layout()->setSpacing( 0 );
    groupBox2->layout()->setMargin( 5 );
    groupBox2Layout = new QGridLayout( groupBox2->layout() );
    groupBox2Layout->setAlignment( Qt::AlignTop );

    textLabel3 = new QLabel( groupBox2, "textLabel3" );

    groupBox2Layout->addWidget( textLabel3, 2, 0 );

    textLabel1 = new QLabel( groupBox2, "textLabel1" );

    groupBox2Layout->addWidget( textLabel1, 0, 0 );

    sliderMaskThreshold = new QSlider( groupBox2, "sliderMaskThreshold" );
    sliderMaskThreshold->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)1, (QSizePolicy::SizeType)0, 0, 0, sliderMaskThreshold->sizePolicy().hasHeightForWidth() ) );
    sliderMaskThreshold->setMinimumSize( QSize( 100, 0 ) );
    sliderMaskThreshold->setMaxValue( 30 );
    sliderMaskThreshold->setPageStep( 5 );
    sliderMaskThreshold->setValue( 10 );
    sliderMaskThreshold->setOrientation( QSlider::Horizontal );

    groupBox2Layout->addWidget( sliderMaskThreshold, 0, 1 );

    sliderMaskDilation = new QSlider( groupBox2, "sliderMaskDilation" );
    sliderMaskDilation->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)1, (QSizePolicy::SizeType)0, 0, 0, sliderMaskDilation->sizePolicy().hasHeightForWidth() ) );
    sliderMaskDilation->setMinimumSize( QSize( 100, 0 ) );
    sliderMaskDilation->setMaxValue( 30 );
    sliderMaskDilation->setPageStep( 5 );
    sliderMaskDilation->setValue( 6 );
    sliderMaskDilation->setOrientation( QSlider::Horizontal );

    groupBox2Layout->addWidget( sliderMaskDilation, 1, 1 );

    sliderCalibrationThreshold = new QSlider( groupBox2, "sliderCalibrationThreshold" );
    sliderCalibrationThreshold->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)1, (QSizePolicy::SizeType)0, 0, 0, sliderCalibrationThreshold->sizePolicy().hasHeightForWidth() ) );
    sliderCalibrationThreshold->setMinimumSize( QSize( 100, 0 ) );
    sliderCalibrationThreshold->setMaxValue( 40 );
    sliderCalibrationThreshold->setPageStep( 5 );
    sliderCalibrationThreshold->setValue( 15 );
    sliderCalibrationThreshold->setOrientation( QSlider::Horizontal );
    sliderCalibrationThreshold->setTickmarks( QSlider::NoMarks );
    sliderCalibrationThreshold->setTickInterval( 5 );

    groupBox2Layout->addWidget( sliderCalibrationThreshold, 2, 1 );
    layout1->addWidget( groupBox2 );

    DistanceCalibrationCollectionWidgetLayout->addMultiCellLayout( layout1, 0, 1, 5, 5 );

    // actions
    openMaskAction = new QAction( this, "openMaskAction" );
    saveMaskAction = new QAction( this, "saveMaskAction" );
    saveMaskAsAction = new QAction( this, "saveMaskAsAction" );
    saveMarkerAction = new QAction( this, "saveMarkerAction" );
    saveMarkerAsAction = new QAction( this, "saveMarkerAsAction" );
    saveCenterAction = new QAction( this, "saveCenterAction" );
    exitAction = new QAction( this, "exitAction" );
    saveExitAction = new QAction( this, "saveExitAction" );
    dateiMaskeAction = new QAction( this, "dateiMaskeAction" );
    dateiMittelpunkt_und_BalancebereichAction = new QAction( this, "dateiMittelpunkt_und_BalancebereichAction" );
    dateiDistanzmarkerAction = new QAction( this, "dateiDistanzmarkerAction" );
    dateiDistanzlinien = new QAction( this, "dateiDistanzlinien" );
    saveDistlinesAction = new QAction( this, "saveDistlinesAction" );
    saveDistlinesAsAction = new QAction( this, "saveDistlinesAsAction" );


    // toolbars


    // menubar
    MenuBarEditor = new QMenuBar( this, "MenuBarEditor" );

    MenuBarEditor->setCursor( QCursor( 0 ) );

    Datei = new QPopupMenu( this );
    popupMenu = new QPopupMenu( this );
    Datei->insertItem( dateiMaskeAction->iconSet(), tr( "Maske" ), popupMenu );
    openMaskAction->addTo( popupMenu );
    saveMaskAction->addTo( popupMenu );
    saveMaskAsAction->addTo( popupMenu );
    popupMenu_5 = new QPopupMenu( this );
    Datei->insertItem( dateiMittelpunkt_und_BalancebereichAction->iconSet(), tr( "Mittelpunkt und Balancebereich" ), popupMenu_5 );
    saveCenterAction->addTo( popupMenu_5 );
    popupMenu_7 = new QPopupMenu( this );
    Datei->insertItem( dateiDistanzmarkerAction->iconSet(), tr( "Distanzmarker" ), popupMenu_7 );
    saveMarkerAction->addTo( popupMenu_7 );
    saveMarkerAsAction->addTo( popupMenu_7 );
    popupMenu_10 = new QPopupMenu( this );
    Datei->insertItem( dateiDistanzlinien->iconSet(), tr( "Distanzlinien" ), popupMenu_10 );
    saveDistlinesAction->addTo( popupMenu_10 );
    saveDistlinesAsAction->addTo( popupMenu_10 );
    Datei->insertSeparator();
    exitAction->addTo( Datei );
    saveExitAction->addTo( Datei );
    MenuBarEditor->insertItem( QString(""), Datei, 2 );

    languageChange();
    resize( QSize(714, 462).expandedTo(minimumSizeHint()) );
    clearWState( WState_Polished );

    // signals and slots connections
    connect( radioButtonSetCenter, SIGNAL( clicked() ), this, SLOT( setCenterActionClicked() ) );
    connect( radioButtonSetDirection, SIGNAL( clicked() ), this, SLOT( setDirectionActionClicked() ) );
    connect( radioButtonSetMaskAdd, SIGNAL( clicked() ), this, SLOT( setMaskAddActionClicked() ) );
    connect( radioButtonSetMaskSub, SIGNAL( clicked() ), this, SLOT( setMaskSubActionClicked() ) );
    connect( radioButtonSetRed, SIGNAL( clicked() ), this, SLOT( setRedActionClicked() ) );
    connect( openMaskAction, SIGNAL( activated() ), this, SLOT( openMaskClicked() ) );
    connect( saveMarkerAction, SIGNAL( activated() ), this, SLOT( saveMarkerClicked() ) );
    connect( saveMarkerAsAction, SIGNAL( activated() ), this, SLOT( saveMarkerAsClicked() ) );
    connect( saveMaskAction, SIGNAL( activated() ), this, SLOT( saveMaskClicked() ) );
    connect( saveMaskAsAction, SIGNAL( activated() ), this, SLOT( saveMaskAsClicked() ) );
    connect( saveCenterAction, SIGNAL( activated() ), this, SLOT( saveCenterClicked() ) );
    connect( radioButtonSetBalance, SIGNAL( clicked() ), this, SLOT( setBalanceActionClicked() ) );
    connect( exitAction, SIGNAL( activated() ), this, SLOT( close() ) );
    connect( saveExitAction, SIGNAL( activated() ), this, SLOT( saveExitActionClicked() ) );
    connect( sliderMaskThreshold, SIGNAL( valueChanged(int) ), this, SLOT( recalculateMask(int) ) );
    connect( sliderMaskDilation, SIGNAL( valueChanged(int) ), this, SLOT( dilateMask(int) ) );
    connect( pushButtonResetRed, SIGNAL( clicked() ), this, SLOT( resetRedClicked() ) );
    connect( pushButtonBuildMask, SIGNAL( clicked() ), this, SLOT( generateImageMaskStart() ) );
    connect( pushButtonBuildMarker, SIGNAL( clicked() ), this, SLOT( generateMarkerLogStart() ) );
    connect( pushButtonStop, SIGNAL( clicked() ), this, SLOT( stopActions() ) );
    connect( pushButtonAutoCenter, SIGNAL( clicked() ), this, SLOT( generateCenterBalanceArea() ) );
    connect( saveDistlinesAction, SIGNAL( activated() ), this, SLOT( saveDistlinesClicked() ) );
    connect( saveDistlinesAsAction, SIGNAL( activated() ), this, SLOT( saveDistlinesAsClicked() ) );
    init();
}

/*
 *  Destroys the object and frees any allocated resources
 */
DistanceCalibrationCollectionWidget::~DistanceCalibrationCollectionWidget()
{
    destroy();
    // no need to delete child widgets, Qt does it all for us
}

/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void DistanceCalibrationCollectionWidget::languageChange()
{
    setCaption( tr( "CalibrationTool" ) );
    pushButtonAutoCenter->setText( tr( "Auto-Center" ) );
    pushButtonResetRed->setText( trUtf8( "\x52\x6f\x74\x65\x20\x50\x69\x78\x65\x6c\x20\x6c\xc3\xb6\x73\x63\x68\x65\x6e" ) );
    pushButtonBuildMarker->setText( tr( "Distanz" ) );
    pushButtonBuildMask->setText( tr( "Maske" ) );
    pushButtonStop->setText( tr( "Stop" ) );
    groupBox1->setTitle( tr( "Anzeige" ) );
    checkBoxShowMask->setText( tr( "Maske" ) );
    checkBoxShowSegmentation->setText( tr( "Segmentierung" ) );
    checkBoxShowTransitions->setText( trUtf8( "\x46\x61\x72\x62\xc3\xbc\x62\x65\x72\x67\xc3\xa4\x6e\x67\x65" ) );
    checkBoxShowBalance->setText( tr( "Balancebereich" ) );
    checkBoxShowMarker->setText( tr( "Marker" ) );
    checkBoxShowFreeze->setText( tr( "Standbild" ) );
    checkBoxShowDirection->setText( tr( "Richtung" ) );
    checkBoxShowCenter->setText( tr( "Mittelpunkt" ) );
    buttonGroup1->setTitle( tr( "Maus-Aktion" ) );
    radioButtonSetCenter->setText( tr( "Mittelpunkt" ) );
    radioButtonSetDirection->setText( tr( "Richtung" ) );
    radioButtonSetBalance->setText( tr( "Balancebereich" ) );
    radioButtonSetRed->setText( tr( "Rote Pixel" ) );
    radioButtonSetMaskAdd->setText( trUtf8( "\x4d\x61\x73\x6b\x65\x20\x65\x72\x67\xc3\xa4\x6e\x7a\x65\x6e" ) );
    radioButtonSetMaskSub->setText( tr( "Maske radieren" ) );
    groupBox2->setTitle( tr( "Feintuning" ) );
    textLabel3->setText( tr( "Kalibrierung" ) );
    textLabel1->setText( tr( "Maske" ) );
    openMaskAction->setText( trUtf8( "\xc3\x96\x66\x66\x6e\x65\x6e" ) );
    openMaskAction->setMenuText( trUtf8( "\xc3\x96\x66\x66\x6e\x65\x6e" ) );
    saveMaskAction->setText( tr( "Speichern" ) );
    saveMaskAction->setMenuText( tr( "Speichern" ) );
    saveMaskAsAction->setText( tr( "Speichern unter" ) );
    saveMaskAsAction->setMenuText( tr( "Speichern unter" ) );
    saveMarkerAction->setText( tr( "Speichern" ) );
    saveMarkerAction->setMenuText( tr( "Speichern" ) );
    saveMarkerAsAction->setText( tr( "Speichern unter" ) );
    saveMarkerAsAction->setMenuText( tr( "Speichern unter" ) );
    saveCenterAction->setText( tr( "Speichern" ) );
    saveCenterAction->setMenuText( tr( "Speichern" ) );
    exitAction->setText( tr( "Beenden" ) );
    exitAction->setMenuText( tr( "Beenden" ) );
    exitAction->setAccel( tr( "Ctrl+C" ) );
    saveExitAction->setText( tr( "Alles speichern und beenden" ) );
    saveExitAction->setMenuText( tr( "Alles speichern und beenden" ) );
    saveExitAction->setAccel( tr( "Ctrl+X" ) );
    dateiMaskeAction->setText( tr( "Maske" ) );
    dateiMaskeAction->setMenuText( tr( "Maske" ) );
    dateiMittelpunkt_und_BalancebereichAction->setText( tr( "Mittelpunkt und Balancebereich" ) );
    dateiMittelpunkt_und_BalancebereichAction->setMenuText( tr( "Mittelpunkt und Balancebereich" ) );
    dateiDistanzmarkerAction->setText( tr( "Distanzmarker" ) );
    dateiDistanzmarkerAction->setMenuText( tr( "Distanzmarker" ) );
    dateiDistanzlinien->setText( tr( "Distanzlinien" ) );
    saveDistlinesAction->setText( tr( "Speichern" ) );
    saveDistlinesAsAction->setText( tr( "Speichern als" ) );
    Datei->changeItem( Datei->idAt( 0 ), tr( "Maske" ) );
    Datei->changeItem( Datei->idAt( 1 ), tr( "Mittelpunkt und Balancebereich" ) );
    Datei->changeItem( Datei->idAt( 2 ), tr( "Distanzmarker" ) );
    Datei->changeItem( Datei->idAt( 3 ), tr( "Distanzlinien" ) );
    if (MenuBarEditor->findItem(2))
        MenuBarEditor->findItem(2)->setText( tr( "Datei" ) );
}

