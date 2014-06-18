#ifndef ADDPICTUREDIALOG_H
#define ADDPICTUREDIALOG_H

#include <qvariant.h>
#include <qdialog.h>

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
class QGroupBox;
class QLineEdit;
class QLabel;
class QPushButton;

class AddPictureDialog : public QDialog
{
    Q_OBJECT

	public:
	    AddPictureDialog( QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
	    ~AddPictureDialog();
	
	    QGroupBox* pictureGroupBox;
	    QLineEdit* picturePath;
	    QLabel* directoryLabel;
	    QPushButton* browse;
	    QLabel* captionLabel;
	    QPushButton* add;
	    QPushButton* cancel;
	
	public slots:
	    virtual void addPicture();
	    virtual void openFileChooser();
	
	protected:
	
	protected slots:
	    virtual void languageChange();

};

#endif // ADDPICTUREDIALOG_H