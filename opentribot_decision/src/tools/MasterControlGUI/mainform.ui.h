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


void MainForm::helpIndex()
{

}


void MainForm::helpContents()
{

}


void MainForm::helpAbout()
{

}


void MainForm::init()
{
    rc_ctr = new RobotControl_CtrWidget(this);
    ToolTab->addTab ( rc_ctr , "robotcontrol" );
    ToolTab->removePage(tmp_tab);
}


void MainForm::destroy()
{
    if (rc_ctr!=0) delete rc_ctr;
}


void MainForm::show_debug_image()
{
 QString image_name = QFileDialog::getOpenFileName(
                    "./",
                    "debugImages (debug_image_*)",
                    this,
                    "open file dialog",
                    "Choose a debug file for viewing with display (imagemagick)" );
 if (image_name!="") {
   char cmdbuf[100];
   sprintf(cmdbuf, "display %s &", image_name.ascii ());
   system(cmdbuf);  
 }
}


void MainForm::open_simulator()
{
    char cmdbuf[100];
    sprintf(cmdbuf, "xterm -e \"../../odeserver2/odeserver\" &");
    system(cmdbuf);  
}


void MainForm::open_coriander()
{
    char cmdbuf[100];
    sprintf(cmdbuf, "coriander &");
    system(cmdbuf);
}


void MainForm::closeEvent( QCloseEvent * ce )
{
    if (!rc_ctr->isRunning()) {
	ce->accept();
	return;
    }
    
    switch( QMessageBox::information( this, "Master Control GUI",
                                      "There is still an robotcontrol running\n close anyway?",
				      "Close Anyway", "Cancel", QString::null,
                                      1, 1 ) ) {
    case 0:
        ce->accept();
        break;
    case 1:
    default: // just for sanity
        ce->ignore();
        break;
    }
}


void MainForm::open_colortool()
{
    char cmdbuf[100];
    sprintf(cmdbuf, "colorTool &");
    system(cmdbuf);  
}


void MainForm::open_calibrationtool()
{
    char cmdbuf[100];
    sprintf(cmdbuf, "CalibrationTool &");
    system(cmdbuf);  
}


void MainForm::open_tribotsview()
{
    char cmdbuf[100];
    sprintf(cmdbuf, "tribotsview &");
    system(cmdbuf);  
}


void MainForm::open_configeditor()
{
    char cmdbuf[100];
    sprintf(cmdbuf, "config_editor &");
    system(cmdbuf);  
}


void MainForm::open_controlGUI()
{
    char cmdbuf[100];
    sprintf(cmdbuf, "ControlGUI &");
    system(cmdbuf);  
}
