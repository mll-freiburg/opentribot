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


void CalibrationRecorderControlPanel::nextClicked()
{
  doNext=true;
  doContinue=false;
}


void CalibrationRecorderControlPanel::continueClicked()
{
  doContinue=true;
}


void CalibrationRecorderControlPanel::quitClicked()
{
  doQuit=true;
}


void CalibrationRecorderControlPanel::init()
{
  doQuit=doContinue=doNext=doBack=doSkip=false;
  sensitivity=sliderSensitivity->value();
}


void CalibrationRecorderControlPanel::setMessage( const QString & text )
{
  textEditMessage->setText(text);
}


void CalibrationRecorderControlPanel::backClicked()
{
  doBack=true;
}


void CalibrationRecorderControlPanel::sensitivityChanged( int val )
{
  sensitivity=val;
}


void CalibrationRecorderControlPanel::skipClicked()
{
  doSkip=true;
}
