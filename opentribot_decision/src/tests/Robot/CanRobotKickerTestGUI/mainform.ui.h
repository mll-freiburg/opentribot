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


void MainForm::init()
{
    com = 0;
    cmd_frame->setEnabled(false);
}


void MainForm::destroy()
{
    if (com!=0) disconnect_tmc();
}


void MainForm::connect_tmc()
{    
    if (com != 0) return;

    RobotCtr2::TmcCanCom_Params p;
    //p.DevNodeStr = "/dev/pcan32";
    p.DevNodeStr = lineEditDev->text().ascii();
    p.canbaud    = CAN_BAUD_1M;
  
    try {
	msgout->append("Try to connect on: " + p.DevNodeStr );
	com = new RobotCtr2::TmcCanCom(p);
	
	//unsigned int confec = 0;
	//confec |= com->setCyclicSendMode(CSM_ALL);
	//if (confec!=0) msgout->append(QString(com->resolveCONFEC(confec)));
    }	
    catch (RobotCtr2::TmcCanException& e) {
	msgout->append( "Connection failed: " + QString(e.what()) );
	com = 0;
	connectButton->setPaletteBackgroundColor(yellow);
	connectButton->toggle();
	return;
    }
    
    cmd_frame->setEnabled(true);
    connectButton->setPaletteBackgroundColor(green);
    lineEditDev->setEnabled(false);
}


void MainForm::disconnect_tmc()
{
    if (com==0) return;
    
    //unsigned int confec = 0;
    //confec |= com->setCyclicSendMode(CSM_NOTHING);
    //if (confec!=0) msgout->append(QString(com->resolveCONFEC(confec)));
    
    com->sendMotorVel( 0 , 0, 0, MOTOR_CTR_MODE_PWM, 0, 0);
    
    delete com;
    com = 0;
  
    cmd_frame->setEnabled(false);
    connectButton->setPaletteBackgroundColor(gray);
    lineEditDev->setEnabled(true);
}


void MainForm::connection( bool on )
{
    if (on) connect_tmc();
    if (!on) disconnect_tmc();
}


void MainForm::kick()
{
    int id=0;
    unsigned char duration[2];
    duration[0]=duration[1]=0;
    
    if (radioButtonKickerId2->isChecked()) id = 1;
    else id =0;
    
    duration[id] = spinBoxKickDuration1->value();
    
    com->sendMotorVel( 0 , 0 , 0 , MOTOR_CTR_MODE_PWM, duration[0],duration[1] );
}
