#ifndef NEWDIALOG_H
#define NEWDIALOG_H

#include "MainWindow.h"

#include <qvariant.h>
#include <qdialog.h>

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
class QFrame;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QListBox;
class QListBoxItem;

class NewDialog : public QDialog
{
    Q_OBJECT

	public:
	    NewDialog( QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
	    ~NewDialog();
	
	    QFrame* line;
	    QGroupBox* directoryGroupBox1;
	    QLabel* directoryLabel;
	    QLineEdit* dirPath;
	    QPushButton* browse;
	    QListBox* pictureListBox;
	    QLabel* captionLabel;
	    QLabel* projectLabel;
	    QLineEdit* projectName;
	    QPushButton* finish;
	    QPushButton* cancel;
	    QLabel* statusLabel;
	    
	    void statusMessage(QString dir, int statuscode);
	
	public slots:
	    virtual void openFileChooser();
	    virtual void createProject();
	
	protected:
	    QGridLayout* NewDialogLayout;
	    QSpacerItem* Horizontal_Spacing2;
	
	protected slots:
	    virtual void languageChange();

};

#endif // NEWDIALOG_H
