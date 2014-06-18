#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <qvariant.h>
#include <qwidget.h>


class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
class QLabel;
class QPushButton;
class QTabWidget;

class MainWidget : public QWidget
{
    Q_OBJECT

	public:
	    MainWidget( QWidget* parent = 0, const char* name = 0, WFlags fl = 0 );
	    ~MainWidget();
		
	    QTabWidget* tabwidget;
	    QGridLayout* mainlayout;
	 
	 	void removeAllPages();
	
	public slots:
		virtual void tabchanged();
	 	
	protected:
	    QHBoxLayout* boxlayout;
	    QSpacerItem* spacer;
	
	protected slots:
	    virtual void languageChange();

};

#endif // MAINWIDGET_H
