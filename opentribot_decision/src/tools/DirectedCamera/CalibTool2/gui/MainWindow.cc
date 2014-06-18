#include "MainWindow.h"
#include "MainWidget.h"
#include "ImageClickWidget.h"
#include "ModelView.h"
#include "NewDialog.h"
#include "AddPictureDialog.h"

#include "../projectmanagement/ProjectManager.h"
#include "../projectmanagement/HardDiscAccess.h"
#include "ImageClickWidget.h"
#include "../../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../../ImageProcessing/Formation/Image.h"
#include "../../../../ImageProcessing/Formation/YUVImage.h"
#include "../../../../ImageProcessing/Formation/ImageIO.h"

#include <iostream>
#include <fstream>

#include <qvariant.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qaction.h>
#include <qmenubar.h>
#include <qpopupmenu.h>
#include <qtoolbar.h>
#include <qimage.h>
#include <qpixmap.h>
#include <qtabwidget.h>
#include <qlabel.h>
#include <qfiledialog.h>
#include <qmessagebox.h>
#include <qfile.h>
#include <qtextstream.h>


/**
 *  Constructs a mainForm as a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'.	 *
 */
MainWindow::MainWindow( QWidget* parent, const char* name, WFlags fl )
    : QMainWindow( parent, name, fl )
{
    (void)statusBar();
    if ( !name )
	setName( "MainWindow" );

    // actions
    fileOpenAction = new QAction( this, "fileOpenAction" );
    fileOpenAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "images/open.png" ) ) );
    fileOpenAction->setEnabled(true);
    
    fileCloseAction = new QAction( this, "fileCloseAction" );
    fileCloseAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "images/close.png" ) ) );
    fileCloseAction->setEnabled(false);
    
    fileExitAction = new QAction( this, "fileExitAction" );
    fileExitAction->setEnabled(true);
    
    fileNewAction = new QAction( this, "fileNewAction" );
    fileNewAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "images/new.png" ) ) );
    
    
    fileSaveAction = new QAction( this, "fileSaveAction" );
    fileSaveAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "images/save.png" ) ) );	    
    fileSaveAction->setEnabled(false);
    
    projectAddPictureAction = new QAction( this, "projectAddPictureAction" );
    projectAddPictureAction->setEnabled(false);
    
    projectProjectSettingsAction = new QAction( this, "projectProjectSettingsAction" );
	projectProjectSettingsAction->setIconSet( QIconSet( QPixmap::fromMimeSource( "images/settings2.png" ) ) );
	projectProjectSettingsAction->setEnabled(false);
	
	toggleWindowAction = new QAction( this, "toggleWindowAction" );
	toggleWindowAction->setEnabled(false);
	
    // toolbars
    Toolbar = new QToolBar( QString(""), this, DockTop ); 
    fileNewAction->addTo( Toolbar );
	fileOpenAction->addTo( Toolbar );
	fileCloseAction->addTo( Toolbar );
	Toolbar->addSeparator();
	fileSaveAction->addTo( Toolbar );
	
    // menubar
    menuBar = new QMenuBar( this, "menuBar" );

	// file
    File = new QPopupMenu( this );
    fileNewAction->addTo( File );
    fileOpenAction->addTo( File );
    fileCloseAction->addTo( File );
    File->insertSeparator();
    fileSaveAction->addTo( File );
    File->insertSeparator();
    fileExitAction->addTo( File );
    menuBar->insertItem( QString(""), File, 1 );

	// project
    Project = new QPopupMenu( this );	    
    ActiveProjects = new QPopupMenu( this );
    Project->insertItem( QIconSet( QPixmap::fromMimeSource( "images/projects.png" ) ), "Active Projects", ActiveProjects);   
    projectGroup = new QActionGroup( this );
    connect( projectGroup, SIGNAL( selected(QAction*) ), this, SLOT( changeProject(QAction*) ) );
    Project->insertSeparator();
    projectAddPictureAction->addTo( Project );
    projectProjectSettingsAction->addTo( Project );
    menuBar->insertItem( QString(""), Project, 2 );
    
	// window
    Window = new QPopupMenu( this );
    toggleWindowAction->addTo(Window);
    menuBar->insertItem( QString(""), Window, 3 );
	
	mainwidget = new MainWidget(this, "mainwidget");
	setCentralWidget ( mainwidget );
	
	modelview = new ModelView(this,"ModelView", false);

    languageChange();
    //resize( QSize(608, 577).expandedTo(minimumSizeHint()) );
    clearWState( WState_Polished );

    // signals and slots connections
    connect( fileNewAction, SIGNAL( activated() ), this, SLOT( fileNewProject() ) );
    connect( fileOpenAction, SIGNAL( activated() ), this, SLOT( fileOpenProject() ) );
    connect( fileCloseAction, SIGNAL( activated() ), this, SLOT( fileCloseProject() ) );
    connect( fileSaveAction, SIGNAL( activated() ), this, SLOT( fileSaveProject() ) );
    connect( fileExitAction, SIGNAL( activated() ), this, SLOT( fileExitProject() ) );
    //connect( fileExitAction, SIGNAL( activated() ), qApp, SLOT( closeAllWindows() ) );
    connect( projectAddPictureAction, SIGNAL( activated() ), this, SLOT( projectAddPicture() ) );
    connect( toggleWindowAction, SIGNAL( activated() ), this, SLOT( toggleModelView() ) );

}

/*
 *  Destroys the object and frees any allocated resources
 */
MainWindow::~MainWindow()
{
    // no need to delete child widgets, Qt does it all for us
}


void MainWindow::showProject() {
	
	// ProjectID bekommen
	int id = ProjectManager::getActProjectID();
		
	// es sind schon Projekte geöffnet 
	// mainwidget neu anlegen
	if(id > 1) {
			
		// erstmal QTabWidget leeren, um neue Bilder einfügen zu können
		delete mainwidget;
		mainwidget = new MainWidget(this, "mainwidget");
		setCentralWidget(mainwidget);
		mainwidget->show();
		
		modelview->show();
		modelview->setCaption((ProjectManager::getProject(id)).getProjectName());
		
		// Grosse der Bilder besorgen um TabWidget anzupassen
		int width = ((ProjectManager::getProject(id)).getCalibImages()).at(0).getOriginalImage()->getWidth();
		int height = ((ProjectManager::getProject(id)).getCalibImages()).at(0).getOriginalImage()->getHeight();
		mainwidget->tabwidget->setMinimumSize(width, height);
		mainwidget->tabwidget->setMaximumSize(width, height);
			
		// Bilder in einzelne Tabs einfügen
		for(int i = 0; i < (ProjectManager::getProject(id)).getImageCount(); i++) {   	
		   	Tribots::Image* image = ((ProjectManager::getProject(id)).getCalibImages()).at(i).getTempImage();
	  		ImageClickWidget* click = new ImageClickWidget(mainwidget->tabwidget, ((ProjectManager::getProject(id)).getCalibImages()).at(i).getImageFileName().ascii());
	   		click->setImage(*image);
	   		mainwidget->tabwidget->insertTab( click, ((ProjectManager::getProject(id)).getCalibImages()).at(i).getImageFileName().ascii() );
	   		click->markSelections();	
		}
			      
		mainwidget->tabwidget->setCurrentPage(0);
		// dann den Rest aufbauen
		modelview->updateModelView(id, 0);
		
		delete projectGroup;
		projectGroup = new QActionGroup( this );
    	
		
		for(int i = ProjectManager::getProjectCounter(); i > 0; i--) {
			QAction* openProjectAction = new QAction( projectGroup, ProjectManager::getProject(i).getProjectName()+" <"+QString::number(i,10)+">" );
	    	openProjectAction->setText( tr( ProjectManager::getProject(i).getProjectName()+" <"+QString::number(i,10)+">" ) );
			openProjectAction->setMenuText( tr( ProjectManager::getProject(i).getProjectName()+" <"+QString::number(i,10)+">" ) );
	    	openProjectAction->setEnabled(true);
	    	openProjectAction->setToggleAction( TRUE );
	    	if(i == ProjectManager::getActProjectID()) {
	    		openProjectAction->setOn(true);
	    	}
		}					
    	projectGroup->addTo(ActiveProjects);
    	connect( projectGroup, SIGNAL( selected(QAction*) ), this, SLOT( changeProject(QAction*) ) );

		
	} else if (id == 1) {			
		modelview->show();
		modelview->setCaption((ProjectManager::getProject(id)).getProjectName());
		
		// Grosse der Bilder besorgen um TabWidget anzupassen
		int width = ((ProjectManager::getProject(id)).getCalibImages()).at(0).getOriginalImage()->getWidth();
		int height = ((ProjectManager::getProject(id)).getCalibImages()).at(0).getOriginalImage()->getHeight();
		mainwidget->tabwidget->setMinimumSize(width, height);
		mainwidget->tabwidget->setMaximumSize(width, height);				
		// Bilder in einzelne Tabs einfügen
		for(int i = 0; i < (ProjectManager::getProject(id)).getImageCount(); i++) {
		   	Tribots::Image* image = ((ProjectManager::getProject(id)).getCalibImages()).at(i).getTempImage();
		  	ImageClickWidget* click = new ImageClickWidget(mainwidget->tabwidget, ((ProjectManager::getProject(id)).getCalibImages()).at(i).getImageFileName().ascii());
		   	click->setImage(*image);
		   	mainwidget->tabwidget->insertTab( click, ((ProjectManager::getProject(id)).getCalibImages()).at(i).getImageFileName().ascii() );
		   	click->markSelections();	
		}   
		mainwidget->tabwidget->setCurrentPage(0);
		modelview->updateModelView(id, 0);
		
		delete projectGroup;
		projectGroup = new QActionGroup( this );
    	
		
		for(int i = ProjectManager::getProjectCounter(); i > 0; i--) {
			QAction* openProjectAction = new QAction( projectGroup, ProjectManager::getProject(i).getProjectName()+" <"+QString::number(i,10)+">" );
	    	openProjectAction->setText( tr( ProjectManager::getProject(i).getProjectName()+" <"+QString::number(i,10)+">" ) );
			openProjectAction->setMenuText( tr( ProjectManager::getProject(i).getProjectName()+" <"+QString::number(i,10)+">" ) );
	    	openProjectAction->setEnabled(true);
	    	openProjectAction->setToggleAction( TRUE );
	    	if(i == ProjectManager::getActProjectID()) {
	    		openProjectAction->setOn(true);
	    	}
		}					
    	projectGroup->addTo(ActiveProjects);
    	connect( projectGroup, SIGNAL( selected(QAction*) ), this, SLOT( changeProject(QAction*) ) );
		
	}
	
}

void MainWindow::enableButtons()
{
	fileCloseAction->setEnabled(true);
	fileSaveAction->setEnabled(true);
	projectAddPictureAction->setEnabled(true);
	toggleWindowAction->setEnabled(true);
}
void MainWindow::disableButtons()
{
	fileCloseAction->setEnabled(false);
	fileSaveAction->setEnabled(false);
	projectAddPictureAction->setEnabled(false);
	toggleWindowAction->setEnabled(false);
}


void MainWindow::switchProject() 
{

	// ProjectID bekommen
	int id = ProjectManager::getActProjectID();
	
	// erstmal QTabWidget leeren, um neue Bilder einfügen zu können
	delete mainwidget;
	mainwidget = new MainWidget(this, "mainwidget");
	setCentralWidget(mainwidget);
	mainwidget->show();
	
	modelview->show();
	modelview->setCaption((ProjectManager::getProject(id)).getProjectName());
	
	// Grosse der Bilder besorgen um TabWidget anzupassen
	int width = ((ProjectManager::getProject(id)).getCalibImages()).at(0).getOriginalImage()->getWidth();
	int height = ((ProjectManager::getProject(id)).getCalibImages()).at(0).getOriginalImage()->getHeight();
	mainwidget->tabwidget->setMinimumSize(width, height);
	mainwidget->tabwidget->setMaximumSize(width, height);
		
	// Bilder in einzelne Tabs einfügen
	for(int i = 0; i < (ProjectManager::getProject(id)).getImageCount(); i++) {   	
	   	Tribots::Image* image = ((ProjectManager::getProject(id)).getCalibImages()).at(i).getTempImage();
  		ImageClickWidget* click = new ImageClickWidget(mainwidget->tabwidget, ((ProjectManager::getProject(id)).getCalibImages()).at(i).getImageFileName().ascii());
   		click->setImage(*image);
   		mainwidget->tabwidget->insertTab( click, ((ProjectManager::getProject(id)).getCalibImages()).at(i).getImageFileName().ascii() );
   		click->markSelections();	
	}
		      
	mainwidget->tabwidget->setCurrentPage(0);
	// dann den Rest aufbauen
	modelview->updateModelView(id, 0);

}


/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void MainWindow::languageChange()
{
    setCaption( tr( "CalibTool" ) );
    fileOpenAction->setText( tr( "Open a Project!" ) );
    fileOpenAction->setMenuText( tr( "&Open" ) );
    fileOpenAction->setAccel( tr( "Ctrl+O" ) );
    
    fileCloseAction->setText( tr( "Close the current Project!" ) );
    fileCloseAction->setMenuText( tr( "&Close" ) );
    
    fileExitAction->setText( tr( "Exit CalibTool" ) );
    fileExitAction->setMenuText( tr( "&Exit" ) );
    fileExitAction->setAccel( tr( "Alt+F4" ) );
    
    fileNewAction->setText( tr( "Create a new Project!" ) );
    fileNewAction->setMenuText( tr( "&New" ) );
    fileNewAction->setAccel( tr( "Ctrl+N" ) );
    
    fileSaveAction->setText( tr( "Save the current Project!" ) );
    fileSaveAction->setMenuText( tr( "&Save" ) );
    fileSaveAction->setAccel( tr( "Ctrl+S" ) );
    
    projectAddPictureAction->setText( tr( "Add Picture" ) );
    projectAddPictureAction->setMenuText( tr( "Add Picture" ) );
    
    projectProjectSettingsAction->setText( tr( "Project Settings" ) );
    projectProjectSettingsAction->setMenuText( tr( "Project Settings" ) );
    
    toggleWindowAction->setText( tr( "ModelView" ) );
    toggleWindowAction->setMenuText( tr( "ModelView" ) );
    
    Toolbar->setLabel( tr( "Toolbar" ) );
    
    if (menuBar->findItem(1))
        menuBar->findItem(1)->setText( tr( "&File" ) );
    if (menuBar->findItem(2))
        menuBar->findItem(2)->setText( tr( "Pr&oject" ) );
    if (menuBar->findItem(3))
        menuBar->findItem(3)->setText( tr( "Win&dow" ) );
}

void MainWindow::fileNewProject()
{

	NewDialog* newdialog = new NewDialog(this, "New", true);  
	newdialog->show();
	
	if(newdialog->exec() == QDialog::Accepted) {
		showProject();
		enableButtons();
	}		
}

void MainWindow::fileOpenProject()
{
	// öffne einen FileChooser
	QString projectFileURL = QFileDialog::getOpenFileName(
                "/home/",
                "CalibTool-Projects (*.capro)",
                this,
                "Open an existing Project",
                "Choose a file to open" );	

    
    QString name,cpointsFileName,mpointsFileName,projectDir;
    int picCount;
    std::vector<QString> pics;
    
    HardDiscAccess::parseProjectFile(projectFileURL, projectDir, name, picCount, cpointsFileName, mpointsFileName, pics);
   

	// es worden schon Punkte geklickt
	// Reihenfolge und Anzahl der Bilder ist wichtig
	// Anzahl der .txt Daten ist wichtig
	// zu ueberpruefen sind: - Anzahl der Bilder und .txt Datein
	//						 - Dateinamen der Bilder und txt Dateien (muessen uebereinstimmen Ordner <-> CAPRO Datei)
	
	unsigned int ok = 0;
	bool txtOK = false;
	bool picOK = false;
	
	// Ordner nach txt Datei durchsuchen
	QDir txtDir(projectDir, "*.txt", QDir::Name | QDir::IgnoreCase , QDir::All);
	// Anzahl der Text Dateien ueberpruefen
	if(txtDir.count() == 2) {	
		// Namen der Text Dateien ueberpruefen
		for(unsigned int i = 0; i < 2; i++) {
			if(txtDir.operator[](i) == cpointsFileName) {
				ok++;	
			}	
		}
		if(ok == 0) {
			int messageId = QMessageBox::information ( this, "Error!", "Die Datei "+cpointsFileName+" ist nicht vorhanden!", QMessageBox::Ok );
			return;
		} else if(ok == 1) {
			txtOK = true;
		}
	} else {
		int messageId = QMessageBox::information ( this, "Error!", "Falsche Anzahl .txt Dateien!", QMessageBox::Ok );
		return;
	}
	
	// Ordner nach Bildern  durchsuchen	
	QDir picDir(projectDir, "*.ppm *.jpeg *.jpg", QDir::Name | QDir::IgnoreCase , QDir::All);
	std::vector<QString> addedPics;
	ok = 0;
	
	// Anzahl Bilder im Ordner gleich der Anzahl der Bilder in CAPRO-Datei
	// -> ueberpruefen, ob Dateinamen uebereinstimmen
	if(picDir.count() == picCount) {
		for(unsigned int i = 0; i < picDir.count(); i++) {
			for(unsigned int j = 0; j < pics.size(); j++) {
				if(picDir.operator[](i) == pics.at(j)) {
					ok++;	
				}			
			}	
		}
		if(ok == picDir.count()) {
			picOK = true;	
		}			
	}
	
	// Anzahl Bilder im Ordner groesser der Anzahl der Bilder in CAPRO-Datei
	// -> oeffnen abbrechen
	if(picDir.count() > picCount) {
		int messageId = QMessageBox::information ( this, "Error!", "Bildanzahl ist zu hoch!", QMessageBox::Ok );
		return;
	}
	
	// Anzahl Bilder im Ordner kleiner der Anzahl der Bilder in CAPRO-Datei
	// -> oeffnen abbrechen
	if(picDir.count() < picCount) {
		int messageId = QMessageBox::information ( this, "Error!", "Bildanzahl ist zu niedrig!", QMessageBox::Ok );
		return;
	}
		
	
	// wenn alles OK dann neues Project anlegen und anzeigen	
	if(txtOK && picOK) {
		ProjectManager::openProject(name, projectDir);
		showProject();
		enableButtons();
		
	} else {
		int messageId = QMessageBox::information ( this, "Error!", "Es ist ein unbekannter Fehler aufgetreten!", QMessageBox::Ok );	
		return;
	}    
}


void MainWindow::fileSaveProject()
{
	QString project = ProjectManager::getProject(ProjectManager::getActProjectID()).getProjectName();
	
	// Projectpfad besorgen
	QString path = ProjectManager::getProject(ProjectManager::getActProjectID()).getProjectPath();
	
	// cpointsFile schreiben
	HardDiscAccess::writeModelWorldPointsFile(path);
	
	// CAPRO Datei schreiben
	HardDiscAccess::writeProjectFile(path);

	int messageId = QMessageBox::information ( this, "Information", 
					"Project: "+project+" and cpoints.txt saved!", QMessageBox::Ok );
}	

void MainWindow::fileCloseProject()
{
	// ProjectID bekommen
	int id = ProjectManager::getActProjectID();
	QString project = ProjectManager::getProject(id).getProjectName();
	
	int messageId = QMessageBox::question ( this, "Project "+project+" speichern?", 
					"Moechten Sie das Projekt: "+project+" vor dem Schliessen speichern?", QMessageBox::Yes,  QMessageBox::No);
	if(messageId == QMessageBox::Yes) {
		fileSaveProject();
	}
		
	// anderes Project anzeigen falls noch eines offen ist
	if(id == 1) {
		delete mainwidget;
		mainwidget = new MainWidget(this, "mainwidget");
		setCentralWidget(mainwidget);
		mainwidget->show();
		// Project aus Map entfernen
		ProjectManager::closeProject(id);
		if(ProjectManager::getProjectCounter() >= 1) {
			showProject();
		} else {
			modelview->hide();	
			disableButtons();
		}
	} 
	
	else if(id > 1) {
		delete mainwidget;
		mainwidget = new MainWidget(this, "mainwidget");
		setCentralWidget(mainwidget);
		mainwidget->show();
		// Project aus Map entfernen
		ProjectManager::closeProject(id);
		showProject();
	} 
}

void MainWindow::fileExitProject()
{
	int count = ProjectManager::getProjectCounter();
	while(count != 0)
	{
		fileCloseProject();
		count--;	
	}
	qApp->closeAllWindows();
}

void MainWindow::projectAddPicture()
{
	AddPictureDialog* addpicturedialog = new AddPictureDialog(this, "New", true);  
	addpicturedialog->show();
	
	if(addpicturedialog->exec() == QDialog::Accepted) {
		switchProject();
		fileSaveProject();
	}
}

void MainWindow::changeProject(QAction* action)
{
	QString temp = action->text();
	int pos1 = temp.find('<');
	int pos2 = temp.find('>');
	temp.remove(pos2, temp.length());
	temp.remove(0, pos1+1);
	bool ok;
	int id = temp.toInt(&ok,10);
	
	if(ok) {
		ProjectManager::setActProjectID(id);
		switchProject();
	}
}

void MainWindow::toggleModelView()
{
	if(!modelview->isVisible()) {
		modelview->show();	
	} else if(modelview->isVisible()) {
		modelview->hide();
	}		
}


void MainWindow::keyPressEvent ( QKeyEvent * e ) {  }

void MainWindow::keyReleaseEvent ( QKeyEvent * e )
{	
	if (e->key() == Key_Up) { 
		modelview->showNextPoint();
	}
	if (e->key() == Key_Down) { 
		modelview->showPreviousPoint();
	}
	if (e->key() == Key_Delete) { 
		//modelview->deleteActPoint();
	}
}


