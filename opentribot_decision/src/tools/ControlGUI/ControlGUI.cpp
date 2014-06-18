/****************************************************************************
** Form implementation generated from reading ui file 'ControlGUI.ui'
**
** Created: Mon Aug 31 14:47:14 2009
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#include "ControlGUI.h"

#include <qvariant.h>
#include <qpushbutton.h>
#include <qgroupbox.h>
#include <qcombobox.h>
#include <qcheckbox.h>
#include <qlabel.h>
#include <qlcdnumber.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qaction.h>
#include <qmenubar.h>
#include <qpopupmenu.h>
#include <qtoolbar.h>
#include <qimage.h>
#include <qpixmap.h>

#include "../components/FieldOfPlay.h"
#include "../tribotsview/MonoLED.h"
#include "ControlGUI.ui.h"
static const char* const image1_data[] = { 
"16 18 2 1",
". c #08a400",
"# c #ffffff",
"................",
"................",
"................",
".......###......",
"........##......",
"....##.....#....",
".....#....##.#..",
"..##......##.#..",
".##.......##....",
"................",
".##.......##....",
"..#.......##.#..",
".....#.....#.#..",
"....##...#......",
"........###.....",
"........##......",
"................",
"................"};

static const unsigned char image2_data[] = { 
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
    0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10,
    0x08, 0x06, 0x00, 0x00, 0x00, 0x1f, 0xf3, 0xff, 0x61, 0x00, 0x00, 0x02,
    0xef, 0x49, 0x44, 0x41, 0x54, 0x38, 0x8d, 0x4d, 0x92, 0x5f, 0x68, 0xd6,
    0x65, 0x14, 0xc7, 0x3f, 0xe7, 0x9c, 0xe7, 0xf9, 0xed, 0xf7, 0xee, 0x7d,
    0xdf, 0xd9, 0xe6, 0x7a, 0x97, 0x6e, 0xf6, 0x6e, 0x54, 0x14, 0x1b, 0x6e,
    0xe5, 0x1a, 0x84, 0x16, 0xd9, 0x85, 0x74, 0x95, 0x77, 0xe1, 0x45, 0x7a,
    0xd7, 0x06, 0x16, 0x44, 0x90, 0x57, 0x61, 0x5d, 0x04, 0x45, 0x05, 0x45,
    0x90, 0x45, 0xcc, 0xa8, 0xac, 0x20, 0xc2, 0x6e, 0xa2, 0x14, 0x8c, 0x02,
    0x29, 0xfa, 0xb3, 0x44, 0x4b, 0x13, 0x5d, 0x34, 0x69, 0xe9, 0x98, 0x73,
    0x36, 0x5b, 0x6e, 0x6e, 0x7b, 0x7f, 0xdb, 0xef, 0x79, 0xba, 0x78, 0xc7,
    0xe8, 0x70, 0x0e, 0x9c, 0x9b, 0xcf, 0x97, 0x2f, 0xe7, 0x7c, 0x05, 0xe0,
    0x50, 0x5b, 0x3a, 0x18, 0xb3, 0x7c, 0xb8, 0xb9, 0xaf, 0x87, 0xb0, 0x54,
    0x23, 0x02, 0x92, 0xe7, 0xe4, 0x44, 0x88, 0x01, 0xf2, 0x08, 0x04, 0x42,
    0x0e, 0x9a, 0x08, 0x73, 0x63, 0x13, 0xf8, 0xc6, 0x86, 0xa1, 0xdd, 0x93,
    0xf3, 0x07, 0xe5, 0x50, 0x5b, 0x3a, 0xd8, 0xd4, 0xd5, 0x35, 0xdc, 0xfb,
    0xdc, 0x13, 0x5c, 0xf9, 0xfa, 0x18, 0x92, 0xa6, 0x88, 0x01, 0xea, 0x50,
    0x83, 0x88, 0x60, 0x4e, 0x08, 0xaa, 0x60, 0x8a, 0xf3, 0x9e, 0x86, 0xca,
    0x7a, 0xce, 0xbc, 0xf0, 0x21, 0xf9, 0xdc, 0x8d, 0x21, 0xf9, 0xa0, 0xd9,
    0xc7, 0xfb, 0x3f, 0x7a, 0x8d, 0x73, 0xaf, 0xbe, 0x44, 0x72, 0x73, 0x05,
    0xf1, 0x86, 0x39, 0x8f, 0x38, 0xa5, 0x72, 0xcf, 0x36, 0x54, 0x8d, 0xab,
    0xa3, 0x27, 0x51, 0xf3, 0xa8, 0x33, 0x30, 0x43, 0x9d, 0xa7, 0x7c, 0x47,
    0x95, 0x93, 0xfb, 0xde, 0xc4, 0xdd, 0xd4, 0xdb, 0xcd, 0xdf, 0x3f, 0x7d,
    0x4f, 0xd2, 0xda, 0x8a, 0x15, 0x0b, 0x14, 0xca, 0x2d, 0xf4, 0xef, 0xdd,
    0x4f, 0xda, 0xb3, 0x95, 0xff, 0xd7, 0xfc, 0xe9, 0xe3, 0x9c, 0xff, 0xfc,
    0x3d, 0x56, 0xb2, 0x45, 0xc4, 0x8c, 0x7c, 0x61, 0x81, 0x74, 0x43, 0x2b,
    0x2e, 0xd6, 0x32, 0xe2, 0xf2, 0x0a, 0x92, 0x78, 0x0a, 0xe5, 0x16, 0xb6,
    0x1d, 0x38, 0x0a, 0x40, 0x6d, 0x74, 0x84, 0x89, 0x13, 0xc7, 0x69, 0x68,
    0x5a, 0x47, 0xc7, 0x83, 0x3b, 0x29, 0xf5, 0x6d, 0x67, 0xa0, 0xfb, 0x5e,
    0x4e, 0xbd, 0xfe, 0x34, 0xcb, 0x59, 0x8d, 0x88, 0x10, 0x6b, 0x19, 0x1a,
    0x01, 0xf1, 0x82, 0x39, 0x4f, 0xff, 0xde, 0xfd, 0x00, 0x5c, 0xfa, 0xec,
    0x00, 0x3f, 0xbf, 0xfd, 0x3c, 0x95, 0xcd, 0x03, 0x14, 0x2b, 0x1d, 0xfc,
    0xf0, 0xf2, 0x93, 0x4c, 0x1e, 0x7d, 0x1f, 0x7c, 0x89, 0xdb, 0x76, 0xec,
    0x42, 0x9c, 0x47, 0x45, 0x21, 0x46, 0x54, 0xf2, 0x1c, 0xd4, 0x51, 0x6a,
    0xbf, 0x95, 0xb4, 0x67, 0x2b, 0xb5, 0xd1, 0x11, 0xc6, 0xbf, 0xfd, 0x02,
    0x71, 0x8e, 0x72, 0xe7, 0x5d, 0x34, 0x77, 0xf7, 0x23, 0xce, 0x71, 0x71,
    0xe4, 0x18, 0xcc, 0x5d, 0x61, 0xdd, 0x96, 0x1d, 0x88, 0x29, 0x51, 0x95,
    0x10, 0x02, 0x9a, 0x13, 0x51, 0x83, 0xe2, 0xc6, 0x2a, 0xc0, 0x1a, 0x2c,
    0x66, 0x20, 0x02, 0x08, 0x62, 0x86, 0x98, 0x31, 0xf3, 0xdb, 0x08, 0x00,
    0xe5, 0xb6, 0x0e, 0xc4, 0x79, 0x08, 0x01, 0x25, 0x06, 0x22, 0x82, 0xaa,
    0xd5, 0xaf, 0xa5, 0xb6, 0x06, 0x80, 0xd4, 0xdb, 0x0c, 0x71, 0x09, 0xa2,
    0x02, 0x80, 0x2f, 0x35, 0x23, 0xea, 0x56, 0x05, 0xf2, 0x88, 0x39, 0x61,
    0xf1, 0xda, 0x14, 0x00, 0xed, 0x03, 0x0f, 0xad, 0x02, 0x0e, 0x04, 0x10,
    0xa9, 0xc3, 0xa6, 0xb4, 0xf4, 0xd6, 0x3f, 0xb3, 0x38, 0x73, 0x19, 0xaf,
    0x46, 0x1e, 0x72, 0x14, 0x02, 0x41, 0x95, 0x85, 0x99, 0x69, 0x98, 0x9d,
    0xa4, 0xd4, 0xb7, 0x9d, 0x4a, 0xcf, 0x00, 0x62, 0xc6, 0xf2, 0xb5, 0x69,
    0xae, 0x5f, 0x38, 0x8b, 0x98, 0x52, 0xbd, 0xef, 0x61, 0x68, 0x5c, 0x4f,
    0xf6, 0xe7, 0x69, 0xd4, 0xa7, 0xe4, 0xab, 0x0e, 0x5c, 0xc8, 0x59, 0x0d,
    0x87, 0xf1, 0xeb, 0xbb, 0x2f, 0x72, 0xf7, 0xbe, 0xb7, 0xb8, 0x7d, 0xcf,
    0xb3, 0x54, 0xc7, 0xcf, 0x30, 0x75, 0x76, 0x04, 0x51, 0x61, 0xcb, 0xee,
    0x67, 0xb0, 0xf6, 0x6e, 0x20, 0x63, 0xfc, 0xbb, 0x23, 0xe0, 0x1c, 0xe6,
    0xb4, 0x2e, 0xa0, 0x89, 0xa2, 0x80, 0x38, 0xc7, 0xe2, 0xec, 0x34, 0xa7,
    0x5e, 0x79, 0x9c, 0xcd, 0xbb, 0x9e, 0xc2, 0x77, 0xf6, 0xb2, 0xa9, 0xb3,
    0x77, 0x2d, 0x48, 0x61, 0xea, 0x0f, 0xc6, 0xbe, 0x3a, 0xcc, 0x72, 0x9e,
    0x91, 0x14, 0x9b, 0xc8, 0x57, 0x96, 0x40, 0xc1, 0xcd, 0x8d, 0x5d, 0xa2,
    0xd0, 0x71, 0x0b, 0xd9, 0x3f, 0xf3, 0x75, 0xdb, 0x59, 0x8d, 0x5f, 0x3e,
    0x79, 0x83, 0xa4, 0x54, 0xa6, 0xa5, 0x7a, 0x27, 0x38, 0xcf, 0xf5, 0x89,
    0x0b, 0xa8, 0x4f, 0xc1, 0x39, 0x7c, 0x63, 0x89, 0x62, 0xb5, 0x8b, 0x8b,
    0x5f, 0x7e, 0x4a, 0xbe, 0x04, 0xf6, 0x68, 0x4b, 0xe3, 0xe4, 0xd5, 0x1f,
    0xcf, 0x3f, 0xb2, 0x69, 0xe7, 0x03, 0x58, 0xe2, 0xb0, 0x42, 0x03, 0x3e,
    0x4d, 0xc0, 0x09, 0xb5, 0xc5, 0x7f, 0xa9, 0xdd, 0x98, 0xc5, 0x3b, 0x23,
    0x98, 0x60, 0x4e, 0xb1, 0xc6, 0x12, 0x97, 0xbf, 0x39, 0xc2, 0xef, 0xef,
    0x1c, 0x26, 0x69, 0x2a, 0x0f, 0x09, 0xc0, 0xc7, 0x1b, 0xcb, 0x83, 0x2b,
    0x4b, 0x4b, 0xc3, 0xe9, 0x86, 0x56, 0x62, 0x2d, 0x83, 0x18, 0x09, 0x21,
    0xc0, 0xea, 0xe4, 0x21, 0x5f, 0xdb, 0x23, 0x81, 0x90, 0x29, 0x49, 0x53,
    0x71, 0xe8, 0xb1, 0xbf, 0x66, 0x0f, 0xfe, 0x07, 0x7c, 0x8e, 0x0b, 0x2f,
    0x4e, 0xbc, 0x6a, 0x07, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44,
    0xae, 0x42, 0x60, 0x82
};

static const char* const image3_data[] = { 
"16 18 2 1",
". c #08a400",
"# c #ff0000",
"................",
"................",
"................",
".......###......",
"........##......",
"....##.....#....",
".....#....##.#..",
"..##......##.#..",
".##.......##....",
"................",
".##.......##....",
"..#.......##.#..",
".....#.....#.#..",
"....##...#......",
"........###.....",
"........##......",
"................",
"................"};


/*
 *  Constructs a ControlGUI as a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'.
 *
 */
ControlGUI::ControlGUI( QWidget* parent, const char* name, WFlags fl )
    : QMainWindow( parent, name, fl ),
      image1( (const char **) image1_data ),
      image3( (const char **) image3_data )
{
    (void)statusBar();
    QImage img;
    img.loadFromData( image2_data, sizeof( image2_data ), "PNG" );
    image2 = img;
    if ( !name )
	setName( "ControlGUI" );
    setMinimumSize( QSize( 486, 494 ) );
    setCentralWidget( new QWidget( this, "qt_central_widget" ) );
    ControlGUILayout = new QGridLayout( centralWidget(), 1, 1, 11, 6, "ControlGUILayout"); 

    field = new TribotsTools::FieldOfPlay( centralWidget(), "field" );
    field->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)3, 0, 0, field->sizePolicy().hasHeightForWidth() ) );
    field->setMinimumSize( QSize( 0, 200 ) );

    ControlGUILayout->addWidget( field, 0, 0 );

    layout7 = new QVBoxLayout( 0, 0, 6, "layout7"); 

    groupBox1 = new QGroupBox( centralWidget(), "groupBox1" );
    groupBox1->setColumnLayout(0, Qt::Vertical );
    groupBox1->layout()->setSpacing( 2 );
    groupBox1->layout()->setMargin( 2 );
    groupBox1Layout = new QGridLayout( groupBox1->layout() );
    groupBox1Layout->setAlignment( Qt::AlignTop );

    pushButtonExecColorToolOmni = new QPushButton( groupBox1, "pushButtonExecColorToolOmni" );

    groupBox1Layout->addWidget( pushButtonExecColorToolOmni, 0, 0 );

    pushButtonExecColorToolPerspective = new QPushButton( groupBox1, "pushButtonExecColorToolPerspective" );

    groupBox1Layout->addWidget( pushButtonExecColorToolPerspective, 1, 0 );

    pushButtonExecCalibrationTool = new QPushButton( groupBox1, "pushButtonExecCalibrationTool" );

    groupBox1Layout->addWidget( pushButtonExecCalibrationTool, 2, 0 );

    pushButtonExecMarkerEditor = new QPushButton( groupBox1, "pushButtonExecMarkerEditor" );

    groupBox1Layout->addWidget( pushButtonExecMarkerEditor, 3, 0 );

    pushButtonExecCoriander = new QPushButton( groupBox1, "pushButtonExecCoriander" );

    groupBox1Layout->addWidget( pushButtonExecCoriander, 4, 0 );

    pushButtonExecConfigEditor = new QPushButton( groupBox1, "pushButtonExecConfigEditor" );

    groupBox1Layout->addWidget( pushButtonExecConfigEditor, 5, 0 );

    pushButtonExecJournalViewer = new QPushButton( groupBox1, "pushButtonExecJournalViewer" );

    groupBox1Layout->addWidget( pushButtonExecJournalViewer, 6, 0 );

    pushButtonExecTribotsview = new QPushButton( groupBox1, "pushButtonExecTribotsview" );

    groupBox1Layout->addWidget( pushButtonExecTribotsview, 7, 0 );

    pushButtonDebugImage = new QPushButton( groupBox1, "pushButtonDebugImage" );

    groupBox1Layout->addWidget( pushButtonDebugImage, 8, 0 );
    layout7->addWidget( groupBox1 );
    spacer3 = new QSpacerItem( 21, 16, QSizePolicy::Minimum, QSizePolicy::Expanding );
    layout7->addItem( spacer3 );

    groupBox2 = new QGroupBox( centralWidget(), "groupBox2" );
    groupBox2->setColumnLayout(0, Qt::Vertical );
    groupBox2->layout()->setSpacing( 2 );
    groupBox2->layout()->setMargin( 2 );
    groupBox2Layout = new QGridLayout( groupBox2->layout() );
    groupBox2Layout->setAlignment( Qt::AlignTop );

    pushButtonExecRobotcontrol = new QPushButton( groupBox2, "pushButtonExecRobotcontrol" );

    groupBox2Layout->addWidget( pushButtonExecRobotcontrol, 0, 0 );

    pushButtonExecRestartCaller = new QPushButton( groupBox2, "pushButtonExecRestartCaller" );

    groupBox2Layout->addWidget( pushButtonExecRestartCaller, 1, 0 );

    pushButtonExitRobotcontrol = new QPushButton( groupBox2, "pushButtonExitRobotcontrol" );

    groupBox2Layout->addWidget( pushButtonExitRobotcontrol, 2, 0 );
    layout7->addWidget( groupBox2 );

    ControlGUILayout->addMultiCellLayout( layout7, 0, 2, 1, 1 );

    layout12 = new QGridLayout( 0, 1, 1, 0, 6, "layout12"); 

    comboBoxPlayertype = new QComboBox( FALSE, centralWidget(), "comboBoxPlayertype" );
    comboBoxPlayertype->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)3, (QSizePolicy::SizeType)0, 0, 0, comboBoxPlayertype->sizePolicy().hasHeightForWidth() ) );

    layout12->addWidget( comboBoxPlayertype, 0, 1 );

    checkBoxBlackScreen = new QCheckBox( centralWidget(), "checkBoxBlackScreen" );

    layout12->addWidget( checkBoxBlackScreen, 1, 2 );

    comboBoxRefereeState = new QComboBox( FALSE, centralWidget(), "comboBoxRefereeState" );
    comboBoxRefereeState->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)3, (QSizePolicy::SizeType)0, 0, 0, comboBoxRefereeState->sizePolicy().hasHeightForWidth() ) );

    layout12->addWidget( comboBoxRefereeState, 0, 2 );

    comboBoxPlayerrole = new QComboBox( FALSE, centralWidget(), "comboBoxPlayerrole" );
    comboBoxPlayerrole->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)3, (QSizePolicy::SizeType)0, 0, 0, comboBoxPlayerrole->sizePolicy().hasHeightForWidth() ) );

    layout12->addWidget( comboBoxPlayerrole, 1, 1 );

    pushButtonActivate = new QPushButton( centralWidget(), "pushButtonActivate" );

    layout12->addWidget( pushButtonActivate, 0, 0 );

    pushButtonDeactivate = new QPushButton( centralWidget(), "pushButtonDeactivate" );

    layout12->addWidget( pushButtonDeactivate, 1, 0 );

    ControlGUILayout->addLayout( layout12, 1, 0 );

    layout11 = new QHBoxLayout( 0, 0, 6, "layout11"); 

    monoLEDActivated = new TribotsTools::MonoLED( centralWidget(), "monoLEDActivated" );
    monoLEDActivated->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)1, (QSizePolicy::SizeType)1, 0, 0, monoLEDActivated->sizePolicy().hasHeightForWidth() ) );
    monoLEDActivated->setMinimumSize( QSize( 21, 21 ) );
    monoLEDActivated->setMaximumSize( QSize( 21, 21 ) );
    layout11->addWidget( monoLEDActivated );

    textLabel2 = new QLabel( centralWidget(), "textLabel2" );
    layout11->addWidget( textLabel2 );
    spacer4 = new QSpacerItem( 40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum );
    layout11->addItem( spacer4 );

    monoLEDConnection = new TribotsTools::MonoLED( centralWidget(), "monoLEDConnection" );
    monoLEDConnection->setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)1, (QSizePolicy::SizeType)1, 0, 0, monoLEDConnection->sizePolicy().hasHeightForWidth() ) );
    monoLEDConnection->setMinimumSize( QSize( 21, 21 ) );
    monoLEDConnection->setMaximumSize( QSize( 21, 21 ) );
    layout11->addWidget( monoLEDConnection );

    textLabel3 = new QLabel( centralWidget(), "textLabel3" );
    layout11->addWidget( textLabel3 );
    spacer5 = new QSpacerItem( 40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum );
    layout11->addItem( spacer5 );

    textLabel1 = new QLabel( centralWidget(), "textLabel1" );
    layout11->addWidget( textLabel1 );

    lCDNumberVoltage = new QLCDNumber( centralWidget(), "lCDNumberVoltage" );
    lCDNumberVoltage->setMaximumSize( QSize( 32767, 20 ) );
    lCDNumberVoltage->setNumDigits( 2 );
    layout11->addWidget( lCDNumberVoltage );

    ControlGUILayout->addLayout( layout11, 2, 0 );

    // actions
    SLHintAction = new QAction( this, "SLHintAction" );
    SLHintAction->setToggleAction( TRUE );
    SLHintAction->setIconSet( QIconSet( image1 ) );
    QuitAction = new QAction( this, "QuitAction" );
    QuitAction->setIconSet( QIconSet( image2 ) );
    GotoAction = new QAction( this, "GotoAction" );
    GotoAction->setToggleAction( TRUE );
    GotoAction->setIconSet( QIconSet( image3 ) );


    // toolbars
    ToolbarMouseAction = new QToolBar( QString(""), this, DockTop ); 

    QuitAction->addTo( ToolbarMouseAction );
    SLHintAction->addTo( ToolbarMouseAction );
    GotoAction->addTo( ToolbarMouseAction );

    languageChange();
    resize( QSize(629, 507).expandedTo(minimumSizeHint()) );
    clearWState( WState_Polished );

    // signals and slots connections
    connect( pushButtonActivate, SIGNAL( clicked() ), this, SLOT( activate() ) );
    connect( pushButtonDeactivate, SIGNAL( clicked() ), this, SLOT( deactivate() ) );
    connect( pushButtonDebugImage, SIGNAL( clicked() ), this, SLOT( debugImage() ) );
    connect( comboBoxPlayertype, SIGNAL( activated(const QString&) ), this, SLOT( playerType(const QString&) ) );
    connect( comboBoxPlayerrole, SIGNAL( activated(const QString&) ), this, SLOT( playerRole(const QString&) ) );
    connect( comboBoxRefereeState, SIGNAL( activated(const QString&) ), this, SLOT( refereeState(const QString&) ) );
    connect( SLHintAction, SIGNAL( activated() ), this, SLOT( slHintActivated() ) );
    connect( QuitAction, SIGNAL( activated() ), this, SLOT( close() ) );
    connect( pushButtonExecRobotcontrol, SIGNAL( clicked() ), this, SLOT( execRobotcontrol() ) );
    connect( pushButtonExitRobotcontrol, SIGNAL( clicked() ), this, SLOT( quitRobotcontrol() ) );
    connect( GotoAction, SIGNAL( activated() ), this, SLOT( gotoActivated() ) );
    connect( checkBoxBlackScreen, SIGNAL( toggled(bool) ), this, SLOT( toggleBlackScreen(bool) ) );
    connect( pushButtonExecRestartCaller, SIGNAL( clicked() ), this, SLOT( execRestartCaller() ) );
    connect( pushButtonExecColorToolOmni, SIGNAL( clicked() ), this, SLOT( execColorToolOmni() ) );
    connect( pushButtonExecColorToolPerspective, SIGNAL( clicked() ), this, SLOT( execColorToolPerspective() ) );
    connect( pushButtonExecCalibrationTool, SIGNAL( clicked() ), this, SLOT( execCalibrationTool() ) );
    connect( pushButtonExecMarkerEditor, SIGNAL( clicked() ), this, SLOT( execMarkerEditor() ) );
    connect( pushButtonExecCoriander, SIGNAL( clicked() ), this, SLOT( execCoriander() ) );
    connect( pushButtonExecConfigEditor, SIGNAL( clicked() ), this, SLOT( execConfigEditor() ) );
    connect( pushButtonExecJournalViewer, SIGNAL( clicked() ), this, SLOT( execJournalViewer() ) );
    connect( pushButtonExecTribotsview, SIGNAL( clicked() ), this, SLOT( execTribotsview() ) );
    init();
}

/*
 *  Destroys the object and frees any allocated resources
 */
ControlGUI::~ControlGUI()
{
    destroy();
    // no need to delete child widgets, Qt does it all for us
}

/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void ControlGUI::languageChange()
{
    setCaption( tr( "ControlGUI" ) );
    groupBox1->setTitle( tr( "Tools" ) );
    pushButtonExecColorToolOmni->setText( tr( "ColorTool(Omni)" ) );
    pushButtonExecColorToolPerspective->setText( tr( "ColorTool(Persp)" ) );
    pushButtonExecCalibrationTool->setText( tr( "CalibrationTool" ) );
    pushButtonExecMarkerEditor->setText( tr( "MarkerEditor" ) );
    pushButtonExecCoriander->setText( tr( "Coriander" ) );
    pushButtonExecConfigEditor->setText( tr( "ConfigEditor" ) );
    pushButtonExecJournalViewer->setText( tr( "Journal" ) );
    pushButtonExecTribotsview->setText( tr( "Tribotsview" ) );
    pushButtonDebugImage->setText( tr( "DebugImage" ) );
    groupBox2->setTitle( tr( "Robotcontrol" ) );
    pushButtonExecRobotcontrol->setText( tr( "Start (testing)" ) );
    pushButtonExecRestartCaller->setText( tr( "Start (match)" ) );
    pushButtonExitRobotcontrol->setText( tr( "Quit" ) );
    comboBoxPlayertype->clear();
    comboBoxPlayertype->insertItem( tr( "---" ) );
    checkBoxBlackScreen->setText( tr( "black screen" ) );
    comboBoxRefereeState->clear();
    comboBoxRefereeState->insertItem( tr( "---" ) );
    comboBoxPlayerrole->clear();
    comboBoxPlayerrole->insertItem( tr( "---" ) );
    pushButtonActivate->setText( tr( "activate" ) );
    pushButtonDeactivate->setText( tr( "deactivate" ) );
    textLabel2->setText( tr( "active" ) );
    textLabel3->setText( tr( "program running" ) );
    textLabel1->setText( tr( "batteries [Volt]" ) );
    SLHintAction->setText( tr( "Action" ) );
    QuitAction->setText( tr( "Action" ) );
    GotoAction->setText( tr( "Action" ) );
    ToolbarMouseAction->setLabel( tr( "Toolbar" ) );
}

