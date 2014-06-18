#include "AddPictureDialog.h"

#include "../projectmanagement/ProjectManager.h"
#include "../projectmanagement/HardDiscAccess.h"
#include "ImageClickWidget.h"
#include <iostream>
#include <fstream>
#include "../../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../../ImageProcessing/Formation/JPEGIO.h"
#include "../../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../../ImageProcessing/Formation/ImageIO.h"

#include <qvariant.h>
#include <qpushbutton.h>
#include <qframe.h>
#include <qgroupbox.h>
#include <qlineedit.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qimage.h>
#include <qpixmap.h>
#include <qfiledialog.h>
#include <qmessagebox.h>
#include <qfile.h>


AddPictureDialog::AddPictureDialog( QWidget* parent, const char* name, bool modal, WFlags fl )
    : QDialog( parent, name, modal, fl )
{
    if ( !name )
	setName( "AddPictureDialog" );
    setSizePolicy( QSizePolicy( (QSizePolicy::SizeType)7, (QSizePolicy::SizeType)7, 0, 0, sizePolicy().hasHeightForWidth() ) );
    setMinimumSize( QSize( 400, 200 ) );
    setMaximumSize( QSize( 400, 200 ) );

    pictureGroupBox = new QGroupBox( this, "pictureGroupBox" );
    pictureGroupBox->setGeometry( QRect( 10, 50, 378, 80 ) );

    picturePath = new QLineEdit( pictureGroupBox, "picturePath" );
    picturePath->setGeometry( QRect( 67, 33, 250, 28 ) );

    directoryLabel = new QLabel( pictureGroupBox, "directoryLabel" );
    directoryLabel->setGeometry( QRect( 11, 32, 53, 30 ) );

    browse = new QPushButton( pictureGroupBox, "browse" );
    browse->setGeometry( QRect( 323, 32, 46, 30 ) );

    captionLabel = new QLabel( this, "captionLabel" );
    captionLabel->setGeometry( QRect( 11, 11, 378, 23 ) );

    add = new QPushButton( this, "add" );
    add->setGeometry( QRect( 216, 151, 84, 28 ) );
    add->setAutoDefault( TRUE );
    add->setDefault( TRUE );

    cancel = new QPushButton( this, "cancel" );
    cancel->setGeometry( QRect( 306, 151, 83, 28 ) );
    cancel->setAutoDefault( TRUE );
    languageChange();
    resize( QSize(400, 200).expandedTo(minimumSizeHint()) );
    clearWState( WState_Polished );

    // signals and slots connections
    connect( browse, SIGNAL( clicked() ), this, SLOT( openFileChooser() ) );
    connect( cancel, SIGNAL( clicked() ), this, SLOT( reject() ) );
    connect( add, SIGNAL( clicked() ), this, SLOT( addPicture() ) );

    // tab order
    setTabOrder( picturePath, browse );
    setTabOrder( browse, add );
    setTabOrder( add, cancel );
}

/*
 *  Destroys the object and frees any allocated resources
 */
AddPictureDialog::~AddPictureDialog()
{
    // no need to delete child widgets, Qt does it all for us
}

/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void AddPictureDialog::languageChange()
{
    setCaption( tr( "Add a Picture" ) );
    pictureGroupBox->setTitle( tr( "Choose a Picture" ) );
    directoryLabel->setText( tr( "Picture:" ) );
    browse->setText( tr( "..." ) );
    browse->setAccel( QKeySequence( QString::null ) );
    captionLabel->setText( tr( "<b>Add a new picture to active project.</b>" ) );
    add->setText( tr( "&Add" ) );
    add->setAccel( QKeySequence( tr( "Alt+A" ) ) );
    cancel->setText( tr( "&Cancel" ) );
    cancel->setAccel( QKeySequence( QString::null ) );
}


void AddPictureDialog::addPicture()
{
	if(picturePath->text().isEmpty()) {
		int messageId = QMessageBox::information ( this, "Choose a Picture", 
						"Please choose a picture to add!", QMessageBox::Ok );
    	return;	
	}
	
	// aktueller Projectpfad besorgen
	QString newPath = ProjectManager::getProject(ProjectManager::getActProjectID()).getProjectPath();
	
	// Pfad des zu kopierenden Bildes
	QString oldPath = picturePath->text();
	
	// Bildnamen extrahieren
	QString pictureName = picturePath->text();
	char slash = '/';
	int posslash = pictureName.findRev ( slash );
	pictureName = pictureName.remove(0, posslash+1);
	
	// Bildprefix extrahieren
	QString picturePrefix = pictureName;
	char point = '.';
	int pospoint = picturePrefix.findRev ( point );
	picturePrefix = picturePrefix.remove(0, pospoint+1);
	
	// Pfad des kopierten Bildes
	newPath = (newPath.append("/")).append(pictureName);
	
	int width = ((ProjectManager::getProject(ProjectManager::getActProjectID())).getCalibImages()).at(0).getOriginalImage()->getWidth();
	int height = ((ProjectManager::getProject(ProjectManager::getActProjectID())).getCalibImages()).at(0).getOriginalImage()->getHeight();
	
	// testen, ob schon ein Bild mit ddem selben Namen vorhanden ist
	bool sameName = false;
	pictureName = pictureName.remove(pospoint, pictureName.length());
	for(unsigned int i = 0; i < (ProjectManager::getProject(ProjectManager::getActProjectID())).getImageCount(); i++) {
		if(pictureName.compare((ProjectManager::getProject(ProjectManager::getActProjectID())).getCalibImage(i).getImageFileName()) == 0) {
			int messageId = QMessageBox::information ( this, "Same name error!", 
							"Please choose a picture to add!", QMessageBox::Ok );
	    	return;	
		}
	}
	
	// Image kopieren
	if(picturePrefix.compare("jpeg") == 0 || picturePrefix.compare("jpg") == 0) {
		
		// JPEG behandeln
		
		std::ifstream tempin(oldPath.ascii()); 
		std::ifstream in(oldPath.ascii()); 	
   		Tribots::JPEGIO tempio;
   		Tribots::JPEGIO io;
   		Tribots::RGBImage* tempImage =  new Tribots::RGBImage( *tempio.read(NULL, tempin) );
   		if(tempImage->getWidth() == width && tempImage->getHeight() == height) {
   			std::ofstream out(newPath.ascii()); 
			io.write(*io.read(NULL, in), out);
   			in.close();
   			out.close();
   		} else {
   			int messageId = QMessageBox::information ( this, "Wrong size error!", 
						"Please choose a picture to add!", QMessageBox::Ok );
    		return;	
   		}   	
   		tempin.close();
   		delete tempImage;	
   		

	} else if(picturePrefix.compare("ppm") == 0) {
		
		// PPM behandeln
		
		std::ifstream tempin(oldPath.ascii()); 
		std::ifstream in(oldPath.ascii()); 
   		Tribots::PPMIO tempio;
   		Tribots::PPMIO io;
   		Tribots::RGBImage* tempImage =  new Tribots::RGBImage( *tempio.read(NULL, tempin) );
   		if(tempImage->getWidth() == width && tempImage->getHeight() == height) {
			std::ofstream out(newPath.ascii());
   			io.write(*io.read(NULL, in), out);
   			in.close();
   			out.close();
   		} else {
   			int messageId = QMessageBox::information ( this, "Wrong size error!", 
						"Please choose a picture to add!", QMessageBox::Ok );
    		return;	
   		} 
   		tempin.close();
   		delete tempImage;
   		   				
	} else {		
		// throw exception
	}

	// Bild dem Project zufügen
	(ProjectManager::getProject(ProjectManager::getActProjectID())).addPicture(newPath);
		
	
	done(QDialog::Accepted);
	
}

/**  */
void AddPictureDialog::openFileChooser()
{
	picturePath->setText("");
	// öffne einen FileChooser
	QString pictureFileURL = QFileDialog::getOpenFileName(
                    "/home/",
                    "Picture (*.ppm *.jpeg *.jpg)",
                    this,
                    "Add a Picture",
                    "Choose a picture to open" );
    picturePath->setText(pictureFileURL);
}

