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

#define LOG( __x__ ) *log << "[ " << QDateTime::currentDateTime().toString("yyyy-MM-dd  hh:mm:ss:zzz") << " ] \t " << __x__ << "\n"

void RobotControl_CtrWidget::init()
{
    log = new std::ofstream("mastercontrol_robotcontrol.log");
    
    LOG("MasterControlGUI started");
    restart_flag = 0;
    rc_process = new QProcess(this);
    label_status->setText(" --- ");
    
    stdcerr->setMaxLogLines(100);
    stdcout->setMaxLogLines(100);
    
    connect( rc_process , SIGNAL(processExited()), this, SLOT(rc_exited()) );
    connect( rc_process , SIGNAL(readyReadStdout()), this, SLOT(read_cout()) );
    connect( rc_process , SIGNAL(readyReadStderr()), this, SLOT(read_cerr()) );
    
    connect( button_I , SIGNAL(clicked()), this, SLOT(button_I_slot()));
}


void RobotControl_CtrWidget::destroy()
{
    if (rc_process->isRunning()) {
	stop();
	rc_process->kill();
    }
    LOG("MasterControl ended. Bye.");
    if (log!=0) delete log;
    delete rc_process;
}


void RobotControl_CtrWidget::start()
{
    if (rc_process->isRunning()) return;
    
    stdcout->clear();
    stdcerr->clear();
    
    label_status->setText(" ... starting ...");
    label_status->setPaletteBackgroundColor(Qt::yellow);
    rc_process->clearArguments ();
    
    rc_process->addArgument ("./robotcontrol");
    rc_process->addArgument("--stream_interface");
    if (restart_flag > 0) {
	rc_process->addArgument("--restart");
	LOG("robotcontrol restart no: " << restart_flag);
    }
    if (!param_keeplogs->isChecked()) rc_process->addArgument ("--no_rotate_log");
    bool res = rc_process->start();
    if (res) {
	LOG("robotcontrol started - success ("<< rc_process->arguments().join(" ") << ")");
	label_status->setText("running");
	label_status->setPaletteBackgroundColor(Qt::green);	
    }
    else {
	LOG("failed to start robotcontrol ("<< rc_process->arguments().join(" ") << ")");
	label_status->setText("failed starting");
	label_status->setPaletteBackgroundColor(Qt::red);	
    }
    
}


void RobotControl_CtrWidget::rc_exited()
{
    if (rc_process->normalExit()) {
	LOG("robotcontrol exit regular");
	label_status->setText("Finished");
	label_status->setPaletteBackgroundColor(Qt::gray);
	restart_flag=0;
    }
    else {
	LOG("robotcontrol exit irregular (crash or killed)");
	label_status->setText("crashed or killed");
	label_status->setPaletteBackgroundColor(Qt::red);
	
	if (param_autorestart->isChecked()) {
	    restart_flag++;
	    start();
	}
    }
}


void RobotControl_CtrWidget::stop()
{
     if (!rc_process->isRunning()) return;
     rc_process->writeToStdin("q");
     cycle_info->clear();
}


void RobotControl_CtrWidget::kill()
{
    if (!rc_process->isRunning()) return;
    rc_process->kill();
}


void RobotControl_CtrWidget::read_cout()
{
    QString all_cout(rc_process->readStdout ());
    QString cyc_info_text;
    
    int cyc_info_begin =0;
    do {
	cyc_info_begin = all_cout.find("[BEGIN_CYCLE_INFO]");
	if (cyc_info_begin>=0) {	
	    int cyc_info_end = all_cout.find("[END_CYCLE_INFO]", cyc_info_begin);
	    if (cyc_info_end<0) {
		std::cerr << "Error in finding cyc end string\n";
		return;
	    }
	    cyc_info_text=all_cout.mid(cyc_info_begin+18, cyc_info_end-cyc_info_begin-18);
	    all_cout.remove(cyc_info_begin, cyc_info_end-cyc_info_begin+16);
	} 
    } while (cyc_info_begin>=0);
    if (all_cout.length()>0) stdcout->append(all_cout);
    if (cyc_info_text.length()>0) cycle_info->setText(cyc_info_text);
}


void RobotControl_CtrWidget::read_cerr()
{
    stdcerr->append(rc_process->readStderr ());
}


void RobotControl_CtrWidget::button_I_slot()
{
     if (!rc_process->isRunning()) return;
     rc_process->writeToStdin("I");
}


void RobotControl_CtrWidget::button_m_slot()
{
    if (!rc_process->isRunning()) return;
    rc_process->writeToStdin("m");
}


void RobotControl_CtrWidget::button_stop_robot_slot()
{
    if (!rc_process->isRunning()) return;
    rc_process->writeToStdin(" ");
}


bool RobotControl_CtrWidget::isRunning()
{
    return rc_process->isRunning();
}
