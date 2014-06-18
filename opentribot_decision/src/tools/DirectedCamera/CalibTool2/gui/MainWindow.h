#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <qvariant.h>
#include <qmainwindow.h>

#include "MainWidget.h"
#include "ModelView.h"
#include "NewDialog.h"

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
class QAction;
class QActionGroup;
class QToolBar;
class QPopupMenu;
class QLabel;
class QTabWidget;
class QActionGroup;

class MainWindow : public QMainWindow
{
    Q_OBJECT

	public:
	    MainWindow( QWidget* parent = 0, const char* name = 0, WFlags fl = WType_TopLevel );
	    ~MainWindow();
	    
	    QMenuBar *menuBar;
	    QPopupMenu *File;
	    QPopupMenu *Project;
	    QPopupMenu *Window;
	    QPopupMenu *ActiveProjects;
	    QActionGroup *projectGroup;
	    QToolBar *Toolbar;
	    QAction *fileOpenAction;
	    QAction *fileCloseAction;
	    QAction *fileExitAction;
	    QAction *fileNewAction;
	    QAction *fileSaveAction;
	    QAction *projectActiveProjectsAction;
	    QAction *projectAddPictureAction;
	    QAction *projectProjectSettingsAction;
	    QAction *toggleWindowAction;
	    QLabel *label;
	    QTabWidget *tabwidget;
	    MainWidget *mainwidget;
	    ModelView *modelview;
	        
	public slots:
	    virtual void fileNewProject();
	    virtual void fileOpenProject();
	    virtual void fileSaveProject();
	    virtual void fileCloseProject();
	    virtual void fileExitProject();
	    virtual void projectAddPicture();
	    virtual void toggleModelView();	    
	    virtual void changeProject(QAction* action);
	    void keyPressEvent ( QKeyEvent * e );
		void keyReleaseEvent ( QKeyEvent * e );
	
	protected:
		QGridLayout* gridLayout;
	    QHBoxLayout* boxLayout;
	
	protected slots:
	    virtual void languageChange();
	    
	private:
		void showProject();
		void switchProject();
		
		void enableButtons();
		void disableButtons();

};

#endif /*MAINWINDOW_H_*/
