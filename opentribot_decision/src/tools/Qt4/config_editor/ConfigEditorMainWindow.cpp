
#include "ConfigEditorMainWindow.h"

#include <QtGui/QFileDialog>
#include <QtGui/QStatusBar>
#include <QtGui/QMessageBox>
#include <QtGui/QApplication>
#include <QtCore/QTextStream>
#include <vector>

using namespace Tribots;
using namespace TribotsTools;
using namespace std;

ConfigEditorMainWindow::ConfigEditorMainWindow (QWidget * parent, Qt::WindowFlags flags) : QMainWindow(parent, flags) {
  setupUi (this);
  connect (fileOpenAction,SIGNAL(activated()),this,SLOT(chooseOpenFile()));
  connect (fileSaveAction,SIGNAL(activated()),this,SLOT(slotFileSave()));
  connect (helpAboutAction,SIGNAL(activated()),this,SLOT(aboutSlot()));
  connect (fileExitAction,SIGNAL(activated()), qApp,SLOT(quit()));
  connect (loadButton,SIGNAL(clicked()),this,SLOT(slotReloadMainConfig()));
  connect (lineEditMainConfig,SIGNAL(returnPressed()),this,SLOT(slotReloadMainConfig()));
  connect (saveFileButton,SIGNAL(clicked()), this,SLOT(slotFileSave()));
  connect (listView1,SIGNAL(currentTextChanged(const QString&)),this,SLOT(slotClickedListViewItem(const QString&)));
  tribotssyntax=NULL;

  setWindowTitle( tr( "TribotsConfigEditor" ) );
  tribotssyntax=new TribotsSyntax(textEditor);
  //  listView1->setSorting(-1 );
  if (qApp->argc() == 2)
  {
    QFile f(qApp->argv()[1]);
    if (f.exists())
    {
      slotLoadMainConfig(f.fileName());
      lineEditMainConfig->setText(f.fileName());
    }
  }
  else
  {
    QFile f("../config_files/robotcontrol.cfg");
    if (f.exists())
    {
      slotLoadMainConfig(f.fileName());
      lineEditMainConfig->setText(f.fileName());
    }
    else
    {
      QMessageBox::warning( this, f.fileName(),
          "The config File doesn't exist\n","Ok","Quit", 0, 0, 1 );
    }
  }
  listView1->setFocus();
}

void ConfigEditorMainWindow::slotLoadMainConfig(const QString &fileName)
{
  usedFile.setFileName( fileName );
  if ( !usedFile.open( QIODevice::ReadOnly ) )
    return;
  QTextStream ts( &usedFile );
  textEditor->setText( ts.readAll() );
  textEditor->document()->setModified( FALSE );
  setWindowTitle( fileName );
  statusBar()->showMessage( tr("Loaded document %1").arg(fileName), 2000 );

  Tribots::ConfigReader config(0);
  config.append_from_file(fileName.toAscii());
  this->updateListView(&config);
  usedFile.close();
  lineEditMainConfig->setText(fileName);
}

void ConfigEditorMainWindow::chooseOpenFile()
{
  QString fn = QFileDialog::getOpenFileName(this, QString::null, QString::null);
  if ( !fn.isEmpty() )
    slotLoadMainConfig( fn );
  else
    statusBar()->showMessage( tr("Loading aborted"), 2000 );
}

void ConfigEditorMainWindow::updateListView( Tribots::ConfigReader * cr)
{
  listView1->clear();
  const std::vector<std::string>& files = cr->list_of_sources();
  for (unsigned int i=0;i< files.size();i++ )
  {
    QString itemText = get_pathname(files[i]).c_str();
    itemText+=get_filename(files[i]).c_str();
    new QListWidgetItem( itemText, listView1 );
  }
}

void ConfigEditorMainWindow::slotClickedListViewItem( const QString& lv)
{
  if (textEditor->document()->isModified() &&
    QMessageBox::question(
            this,
            tr("Write Modified File? -- "),
            tr("The current file has been Modified !! Do you want to save?"),
            QMessageBox::Yes ||QMessageBox::Default,QMessageBox::No, 0 )==QMessageBox::Ok  ) {
    slotFileSave();
  }
  usedFile.setFileName( lv );

  if ( !usedFile.open( QIODevice::ReadOnly ) )
    return;

  QTextStream ts( &usedFile );
  textEditor->setText( ts.readAll() );
  textEditor->document()->setModified( FALSE );
  setWindowTitle( lv );
  statusBar()->showMessage( tr("Loaded document %1").arg(lv), 2000 );
  usedFile.close();
}


void ConfigEditorMainWindow::slotFileSave()
{
  if ( usedFile.fileName().isEmpty() )
  {
    return;
  }

  QString text = textEditor->document()->toPlainText();

  if ( !usedFile.open( QIODevice::WriteOnly ) )
  {
    statusBar()->showMessage( tr("Could not write to %1").arg(usedFile.fileName()),
                          2000 );
    return;
  }

  QTextStream t( &usedFile );
  t << text;
  usedFile.close();

  textEditor->document()->setModified( FALSE );

  setWindowTitle( usedFile.fileName());

  statusBar()->showMessage( tr( "File %1 saved" ).arg( usedFile.fileName() ), 2000 );
  listView1->setFocus();
}


void ConfigEditorMainWindow::aboutSlot()
{
  QMessageBox::about(this,
                     "About Tribots Config Editor",
                     "Tool to read in all used config Files \n"
                     "and edit them "
                     "(c) 2005 Stefan Welker\nEmail: stefan.welker@gmx.de");
}


void ConfigEditorMainWindow::slotReloadMainConfig()
{
  slotLoadMainConfig(lineEditMainConfig->text());
}

std::string ConfigEditorMainWindow::get_filename( const std::string & src)
{
  std::string dest="";
  if (src.length()>0) {
    unsigned int i=src.length()-1;
    while (i>0)
      if (src[i]=='/')
        break;
      else
        i--;
    if (src[i]=='/') {
      dest = src.substr (i+1,src.length());
    }
  }
  return dest;
}


std::string ConfigEditorMainWindow::get_pathname( const std::string & src )
{
  std::string dest="";
  if (src.length()>0) {
    unsigned int i=src.length()-1;
    while (i>0)
      if (src[i]=='/')
        break;
      else
        i--;
    if (src[i]=='/') {
      dest = src.substr (0,i+1);
    }
  }
  return dest;
}
