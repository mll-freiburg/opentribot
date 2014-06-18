#include "NewDialog.h"
#include "MainWindow.h"
#include "MainWidget.h"
#include "../projectmanagement/HardDiscAccess.h"
#include "../projectmanagement/ProjectManager.h"

#include <qvariant.h>
#include <qframe.h>
#include <qgroupbox.h>
#include <qlabel.h>
#include <qlineedit.h>
#include <qpushbutton.h>
#include <qlistbox.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qimage.h>
#include <qpixmap.h>
#include <qfiledialog.h>
#include <qdir.h>
#include <qmessagebox.h>

#include <iostream>
#include <fstream>

/*
 *  Constructs a NewDialog as a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'.
 *
 *  The dialog will by default be modeless, unless you set 'modal' to
 *  TRUE to construct a modal dialog.
 */
NewDialog::NewDialog( QWidget* parent, const char* name, bool modal, WFlags fl )
    : QDialog( parent, name, modal, fl )
{
	
    if ( !name )
	setName( "NewDialog" );
    setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, 0, 0, sizePolicy().hasHeightForWidth() ) );
    setMinimumSize( QSize( 400, 370 ) );
    setMaximumSize( QSize( 400, 370 ) );
    setSizeGripEnabled( TRUE );
    
    NewDialogLayout = new QGridLayout( this, 1, 1, 11, 6, "NewDialogLayout"); 
	NewDialogLayout->setResizeMode(QLayout::FreeResize);
    line = new QFrame( this, "line" );
    line->setFrameShape( QFrame::HLine );
    line->setFrameShadow( QFrame::Sunken );
    line->setFrameShape( QFrame::HLine );

    NewDialogLayout->addMultiCellWidget( line, 4, 4, 0, 3 );

    directoryGroupBox1 = new QGroupBox( this, "directoryGroupBox1" );

    directoryLabel = new QLabel( directoryGroupBox1, "directoryLabel" );
    directoryLabel->setGeometry( QRect( 11, 32, 82, 30 ) );

    dirPath = new QLineEdit( directoryGroupBox1, "dirPath" );
    dirPath->setGeometry( QRect( 99, 33, 218, 28 ) );

    browse = new QPushButton( directoryGroupBox1, "browse" );
    browse->setGeometry( QRect( 323, 32, 46, 30 ) );

    pictureListBox = new QListBox( directoryGroupBox1, "pictureListBox" );
    pictureListBox->setGeometry( QRect( 11, 68, 358, 122 ) );

    NewDialogLayout->addMultiCellWidget( directoryGroupBox1, 2, 2, 0, 3 );

    captionLabel = new QLabel( this, "captionLabel" );

    NewDialogLayout->addMultiCellWidget( captionLabel, 0, 0, 0, 3 );

    projectLabel = new QLabel( this, "projectLabel" );

    NewDialogLayout->addWidget( projectLabel, 1, 0 );

    projectName = new QLineEdit( this, "projectName" );

    NewDialogLayout->addMultiCellWidget( projectName, 1, 1, 1, 3 );
    Horizontal_Spacing2 = new QSpacerItem( 203, 20, QSizePolicy::Expanding, QSizePolicy::Minimum );
    NewDialogLayout->addMultiCell( Horizontal_Spacing2, 5, 5, 0, 1 );

    finish = new QPushButton( this, "finish" );
    finish->setAutoDefault( TRUE );
    finish->setDefault( TRUE );

    NewDialogLayout->addWidget( finish, 5, 2 );

    cancel = new QPushButton( this, "cancel" );
    cancel->setAutoDefault( TRUE );

    NewDialogLayout->addWidget( cancel, 5, 3 );

    statusLabel = new QLabel( this, "statusLabel" );

    NewDialogLayout->addMultiCellWidget( statusLabel, 3, 3, 0, 3 );
    languageChange();
    resize( QSize(400, 370).expandedTo(minimumSizeHint()) );
    clearWState( WState_Polished );

    // signals and slots connections
    connect( finish, SIGNAL( clicked() ), this, SLOT( createProject() ) );
    connect( cancel, SIGNAL( clicked() ), this, SLOT( reject() ) );
    connect( browse, SIGNAL( clicked() ), this, SLOT( openFileChooser() ) );
    
     // tab order
    setTabOrder( projectName, dirPath );
    setTabOrder( dirPath, browse );
    setTabOrder( browse, finish );
    setTabOrder( finish, cancel );
}

/*
 *  Destroys the object and frees any allocated resources
 */
NewDialog::~NewDialog()
{
    // no need to delete child widgets, Qt does it all for us
}

/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void NewDialog::languageChange()
{
    setCaption( tr( "New CalibTool Project" ) );
    directoryGroupBox1->setTitle( tr( "Choose a Directory" ) );
    directoryLabel->setText( tr( "Directory:" ) );
    browse->setText( tr( "..." ) );
    browse->setAccel( QKeySequence( QString::null ) );
    pictureListBox->clear();
    //pictureListBox->insertItem( QPixmap::fromMimeSource( "empty.png" ), tr( "New Item" ) );
    captionLabel->setText( tr( "<b>Create a new CalibTool Project.</b>" ) );
    projectLabel->setText( tr( "Project Name:" ) );
    finish->setText( tr( "&Finish" ) );
    finish->setAccel( QKeySequence( tr( "Alt+F" ) ) );
    cancel->setText( tr( "&Cancel" ) );
    cancel->setAccel( QKeySequence( QString::null ) );
    statusLabel->setText( tr( "<font color=\"#ff0000\" size=\"-3\">\nStatus Informationen\n</font>" ) );
}

/**  */
void NewDialog::openFileChooser()
{
	// aus listbox alle items entfernen
	for(unsigned int i = pictureListBox->count(); i > 0; i--) {
		pictureListBox->removeItem(i-1);
	}
	// dirPath auf null setzen
	dirPath->setText("");
	
	// öffne einen FileChooser
	QString dir = QFileDialog::getExistingDirectory(
                    "/home/",
                    this,
                    "Get a project directory!",
                    "Choose a directory",
                    TRUE );	
	
	// Ordner nach txt Datei durchsuchen
	QDir caproDir(dir, "*.capro", QDir::Name | QDir::IgnoreCase , QDir::All);
	if(caproDir.count() == 1) {		
		statusMessage(dir, 4);
		return;
	}
	
	// Ordner nach txt Datei durchsuchen
	QDir txtDir(dir, "*.txt", QDir::Name | QDir::IgnoreCase , QDir::All);
	if(txtDir.count() == 0) {		
		statusMessage(dir, 0);
		return;
	} else if(txtDir.count() > 1) {
		statusMessage(dir, 1);
		return;
	} else {
		QString txtFile = txtDir.operator[](0);
		pictureListBox->insertItem( tr( txtFile ) );		
	} 
	
	// Ordner nach Bildern  durchsuchen	
	QDir picDir(dir, "*.ppm *.jpeg *.jpg", QDir::Name | QDir::IgnoreCase , QDir::All);
	if(picDir.count() == 0) {
		statusMessage(dir,2);		
		return;
	} else {		
		for(unsigned int i = 0; i < picDir.count(); i++ ) {
			QString picFile = picDir.operator[](i);
			pictureListBox->insertItem( tr ( picFile ) );
		}
		statusMessage(dir,3);
		dirPath->setText(dir);
	}
	
			
	
}

/**  */
void NewDialog::createProject()
{
	// überprüfe ob Project Name und Directory angegeben wurde
	if(projectName->text().isEmpty()) {
		int messageId = QMessageBox::information ( this, "Enter Project Name", 
						"Please enter a name for your project!", QMessageBox::Ok );
    	return;	
	}
	if(dirPath->text().isEmpty()) {
		int messageId = QMessageBox::information ( this, "Choose a Directory", 
						"Please choose a directory!", QMessageBox::Ok );
    	return;	
	}
	
	// Project anlegen
	// testen ob ProjectName schon vorhanden -> wenn ja Project intern anders benennen 
	ProjectManager::newProject(projectName->text(), dirPath->text());
	

	done(QDialog::Accepted);
}

void NewDialog::statusMessage(QString dir, int statuscode)
{
	QDir picDir(dir, "*.ppm *.jpeg *.jpg", QDir::Name | QDir::IgnoreCase , QDir::All);
	char slash = '/';
	int posslash = dir.findRev ( slash );
	dir = dir.remove(0, posslash+1);
	
	if(0 == statuscode) {
		statusLabel->setText(QString("<font color=\"#ff0000\" size=\"-4\">Es ist keine Text-Datei im Ordner %1 vorhanden, \nwaehlen Sie bitte einen anderen Ordner!</font>").arg(dir));
	} else 	if(1 == statuscode) {
		statusLabel->setText(QString("<font color=\"#ff0000\" size=\"-4\">Es ist mehr als eine Text-Datei im Ordner %1 vorhanden, \nwaehlen Sie bitte einen anderen Ordner!</font>").arg(dir));
	} else 	if(2 == statuscode) {
		statusLabel->setText(QString("<font color=\"#ff0000\" size=\"-4\">Es sind keine Bilder im Ordner %1 vorhanden, \nwaehlen Sie einen anderen Ordner!</font>").arg(dir));
	} else 	if(3 == statuscode) {
		statusLabel->setText(QString("<font color=\"#ff0000\" size=\"-3\">Es sind %1 Bilder im Ordner %2 vorhanden!\n </font>").arg(picDir.count()).arg(dir));
	} else 	if(4 == statuscode) {
		statusLabel->setText(QString("<font color=\"#ff0000\" size=\"-3\">Es ist eine Projekt-Datei im Ordner %2 vorhanden!\n </font>").arg(dir));
	}
}

