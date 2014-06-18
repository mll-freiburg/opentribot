/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you want to add, delete, or rename functions or slots, use
** Qt Designer to update this file, preserving your code.
**
** You should not define a constructor or destructor in this file.
** Instead, write your code in functions called init() and destroy().
** These will automatically be called by the form's constructor and
** destructor.
*****************************************************************************/

#include "qfile.h"

#include "qfiledialog.h"
#include <qstatusbar.h>
#include <qmessagebox.h>
#include "../../Fundamental/ConfigReader.h"
#include <vector>
#include "TribotsSyntax.h"
#include "qapplication.h"

using namespace Tribots;

void mainWidget::init()
{



  setCaption( tr( "TribotsConfig" ) );
  tribotssyntax=new TribotsSyntax(textEditor);
  listView1->setSorting(-1 );
  
  
  if (qApp->argc() == 2)
  {
    QFile f(qApp->argv()[1]);
    if (f.exists())
    {
      slotLoadMainConfig(f.name());
      lineEditMainConfig->setText(f.name());

    }


  }
  else {
    QFile f("../config_files/robotcontrol.cfg");
    if (f.exists())
    {
      slotLoadMainConfig(f.name());
      lineEditMainConfig->setText(f.name());
    }
    else 
	    QMessageBox::warning( this, f.name(),
			            "The config File doesn't exist\n","Ok","Quit", 0, 0, 1 );
	    
  
  
  }



  // check arguments




  /*    listView1->clear();
      QListViewItem * item_2 = new QListViewItem( listView1, 0 );
      item_2->setOpen( TRUE );
      QListViewItem * item_3 = new QListViewItem( item_2, 0 );
      item_3->setOpen( TRUE );
      QListViewItem * item = new QListViewItem( item_3, 0 );
      item->setText( 0, tr( "imageproducer_camera.cfg" ) );
      item_3->setOpen( TRUE );
      item = new QListViewItem( item_3, item );
      item->setText( 0, tr( "imageprocessing.cfg" ) );
      item_3->setText( 0, tr( "global_image.cfg" ) );
      item_2->setText( 0, tr( "robotctrl.cfg" ) );   */
  listView1->setFocus();


}
void mainWidget::destroy()
{




}






void mainWidget::slotLoadMainConfig(const QString &fileName)
{

  usedFile.setName( fileName );
  if ( !usedFile.open( IO_ReadOnly ) )
    return;
  QTextStream ts( &usedFile );
  textEditor->setText( ts.read() );
  textEditor->setModified( FALSE );
  setCaption( fileName );
  statusBar()->message( tr("Loaded document %1").arg(fileName), 2000 );

  Tribots::ConfigReader config(0);
  config.append_from_file(fileName);
  this->updateListView(&config);
  usedFile.close();
  lineEditMainConfig->setText(fileName);

}


void mainWidget::slotFileOpen()
{
}


void mainWidget::chooseOpenFile()
{

  QString fn = QFileDialog::getOpenFileName( QString::null, QString::null,
               this);
  if ( !fn.isEmpty() )
    slotLoadMainConfig( fn );
  else
    statusBar()->message( tr("Loading aborted"), 2000 );

}






void mainWidget::updateListView( Tribots::ConfigReader * cr)
{
  //cr->
  listView1->clear();
  // element = new QListViewItem( listView, qName, namespaceURI );
  //
  //           for ( int i = 0 ; i < attributes.length(); i++ ) {
  //                 new QListViewItem( element, attributes.qName(i), attributes.uri(i) );
  //             }
  const std::vector<std::string>& files = cr->list_of_sources();
  for (unsigned int i=0;i< files.size();i++ )
  {

    new QListViewItem( listView1, get_filename(files[i]),get_pathname(files[i]), 0 );
    //item->setOpen( TRUE );
    //item->setText( 0,    );
  }




}

void mainWidget::slotClickedListViewItem( QListViewItem * lv)
{
  
  if (textEditor->isModified() &&
    QMessageBox::question(
            this,
            tr("Write Modified File? -- "),
            tr("The current file has been Modified! Do you want to save?"),
	    QMessageBox::Yes ||QMessageBox::Default,QMessageBox::No, 0 )==QMessageBox::Ok  ) {
  
  slotFileSave();
  
  
  }
  
  
  QString fileName=lv->text(0);
  QString filePath=lv->text(1);

  usedFile.setName( filePath+fileName);

  if ( !usedFile.open( IO_ReadOnly ) )
    return;

  QTextStream ts( &usedFile );
  textEditor->setText( ts.read() );
  textEditor->setModified( FALSE );
  setCaption( fileName );
  statusBar()->message( tr("Loaded document %1").arg(filePath+fileName), 2000 );
  usedFile.close();


}


void mainWidget::slotFileSave()
{
  if ( usedFile.name().isEmpty() )
  {

    return;
  }

  QString text = textEditor->text();

  if ( !usedFile.open( IO_WriteOnly ) )
  {
    statusBar()->message( tr("Could not write to %1").arg(usedFile.name()),
                          2000 );
    return;
  }

  QTextStream t( &usedFile );
  t << text;
  usedFile.close();

  textEditor->setModified( FALSE );

  setCaption( usedFile.name());

  statusBar()->message( tr( "File %1 saved" ).arg( usedFile.name() ), 2000 );
  listView1->setFocus();
  

}


void mainWidget::aboutSlot()
{
  QMessageBox::about(this,
                     "About Tribots Config Editor",
                     "Tool to read in all used config Files \n"
                     "and edit them "
                     "(c) 2005 Stefan Welker\nEmail: stefan.welker@gmx.de");
}


void mainWidget::slotReloadMainConfig()
{

slotLoadMainConfig(lineEditMainConfig->text());


    
}


void mainWidget::slotAskForSave()
{



}


std::string mainWidget::get_filename( const std::string & src)
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


std::string mainWidget::get_pathname( const std::string & src )
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


void mainWidget::commentLine()
{
  int zeile;
  int spalte;
  textEditor->getCursorPosition (&zeile, &spalte);
  bool isCommentLine=false;
  unsigned int linePosition=0;
  unsigned int lineNumber=0;
  QString txt = textEditor->text();
  for (unsigned int i=0; i<txt.length(); i++) {
    QChar c = txt[i];
    if (c==QChar('\n')) {
      if (lineNumber==static_cast<unsigned int>(zeile))
        break;
      lineNumber++;
      linePosition=i+1;
      isCommentLine=false;
    }
    if (c==QChar('#') && i==linePosition) {
      isCommentLine=true;
    }
  }
  if (isCommentLine) {
    textEditor->setText (txt.left(linePosition)+txt.right(txt.length()-linePosition-1));
    textEditor->setCursorPosition (zeile, spalte>0 ? spalte-1: 0);
  } else {
    textEditor->setText (txt.left(linePosition)+QString("#")+txt.right(txt.length()-linePosition));
    textEditor->setCursorPosition (zeile, spalte+1);
  }
}


void mainWidget::closeClicked()
{
  if (textEditor->isModified() &&
    QMessageBox::question(
            this,
            tr("Write Modified File? -- "),
            tr("The current file has been Modified! Do you want to save?"),
	    QMessageBox::Yes ||QMessageBox::Default,QMessageBox::No, 0 )==QMessageBox::Ok  ) {
  
  slotFileSave();
  
  
  }
  close();  

}
