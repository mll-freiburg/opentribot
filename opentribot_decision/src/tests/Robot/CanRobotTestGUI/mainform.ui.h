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
#define REPORTTEST( __ok__, __out__, __test_name__, __error_desc__, __error_hint__,__fatal__) ReportTest(__ok__,__out__, __test_name__, __error_desc__, __error_hint__); if(__fatal__ && !__ok__) return false;

void MainForm::init()
{
    com = 0;

    r_timer = new QTimer(this);
    s_timer = new QTimer(this);
    
    p.DevNodeStr  = "/dev/pcan32";
    p.canbaud        = CAN_BAUD_1M;
    
    connect( r_timer , SIGNAL(timeout()) , this, SLOT(receive_tmccancom_data()));
    connect( s_timer , SIGNAL(timeout()) , this, SLOT(send_tmccancom_cmd()));
    
    
    update_config_views();
}


void MainForm::destroy()
{
    if (com!=0) delete com;
}


bool MainForm::test_handle()
{
    HANDLE h;
    char dev_buf[1000];
    sprintf(dev_buf, "%s" ,  p.DevNodeStr.c_str());
    
    h= LINUX_CAN_Open ( dev_buf, O_RDWR );
    
    REPORTTEST( h,
		msg_out, 
		"Opening can handle [" + dev_path->text() + "]",
		"\n\t- unable to get a pcan handle",
		"\n\t- wrong device node;  \n\t- pcan kernel module not loaded (modprobe pcan); \n\t- device not present; \n\t- device not readable;", true);
    
    int err;
    err = CAN_Close(h);
    REPORTTEST(!err,
	       msg_out, 
	       "Closing can handle [" + dev_path->text() + "]",
	       "\n\tUnknown.",
	       "\n\tUnknown.", true);
    return true;
}

void MainForm::ReportTest( bool ok, QTextEdit*  out_disp , std::string test_name, std::string error_desc, std::string error_hint )
{
    std::stringstream out;    
if (ok) 
    out << "<font color=green>(OK)<\font>    \t"; 
  else 
    out << "<font color=red>(FAILED)<\font>\t";
  out << test_name;
  if (!ok) {
    out << "\n\t ErrDesc: " << error_desc;
    out << "\n\t Hint: " << error_hint;
  }
  out_disp->append(out.str());
}

void MainForm::set_config_file( std::string  fname )
{
    config_file_name = fname;
    
    Tribots::ConfigReader cr;
    std::string sdum;
    double ddum;
    int    idum;
    long ldum;
    std::vector< int > vidum;

    if (!cr.append_from_file(fname.c_str(), true)) {
	std::cerr << "Could not read from " << fname << "\n";
	return;
    }
    
    if (!cr.get("RobotCommunication::device", p.DevNodeStr)) {
      std::cerr << "# Can't read [RobotCommunication::device] from config file [" 
	      << fname << "]\n";
    }	
    if (cr.get("RobotCommunication::baud", ldum)) {
	switch (ldum) {
	case 250000:  p.canbaud = CAN_BAUD_250K; break;
	case 1000000: p.canbaud = CAN_BAUD_1M;   break;
	default:
	    std::cerr << "Wrong can baud specification.";
	}
    }

    if (cr.get("RobotHardware::MaxMotorVel", idum)) p_ctr.setMaxMotorVel(idum);
    if (cr.get("RobotHardware::GearFactor", ddum)) p_ctr.setGearFactor(ddum);
    if (cr.get("RobotHardware::PIDFactors", vidum)) {
      if (vidum.size() != 3) std::cerr << "!!! Wrong number of paprams for pid\n";
      else p_ctr.setPIDParams(vidum[0],vidum[1],vidum[2]); 
    }

    update_config_views();
}

void MainForm::update_config_views()
{
    dev_path->setText(p.DevNodeStr);
    
}


void MainForm::update_config_from_view()
{
    if (comboBoxBaud->currentText () == "CAN_BAUD_250K")
	p.canbaud = CAN_BAUD_250K;
    else if (comboBoxBaud->currentText () == "CAN_BAUD_1M")
	p.canbaud = CAN_BAUD_1M;
    else
	std::cerr << "Wrong baud rate: " << comboBoxBaud->currentText () << "\n";
    
    p.DevNodeStr = dev_path->text().ascii();
    
    update_config_views();
}


bool MainForm::connect_tmc_can_com()
{
    unsigned int confec = 0;
    if (com!=0) return false;
    
    bool res = true;
    std::string errstr;

    try {
	com = new RobotCtr2::TmcCanCom(p);    
    }
    catch (RobotCtr2::TmcCanException& e) {
	errstr = e.what();
	com = 0;
	res = false;
	pushButton_tmcCanCom_connect->setPaletteBackgroundColor(Qt::red);
	
    }
    REPORTTEST(res,
	       msg_out, 
	       "Opening TmcCanCom",
	       "\n\t." + errstr,
	       "\n\t- Tmc not connected;\n\t- Tmc not switched on;\n\t- Wrong device node; \n\t- ...", true);
  
    std::string version;
    res = com->ping(&version);    
    REPORTTEST(res,
	       msg_out, 
	       "Pinging [" + version + "]",
	       "\n\t Unknown",
	       "\n\t Unknown", true);
    lineEdit_fw_version->setText(version);
    if (res) {
	pushButton_tmcCanCom_connect->setPaletteBackgroundColor(Qt::green);
	tab1->setEnabled(false);
	tab3->setEnabled(false);
	
	confec |= com->setCyclicSendMode(CSM_ALL);	
	if (confec!=0) 
	    msg_out->append( "Starting message collection: " 
			     + QString(com->resolveCONFEC(confec)));
	
	r_timer->start(33,false);
    }
    
    return res;
}


bool MainForm::disconnect_tmc_can_com()
{
    unsigned int confec = 0;
    if (com!=0) {
	confec |= com->setCyclicSendMode(CSM_NOTHING);
	if (confec!=0) {
	    msg_out->append( "Stopping message collection: " 
			     + QString(com->resolveCONFEC(confec)));
	}
    }
    r_timer->stop();
    
    pushButton_tmcCanCom_connect->setPaletteBackgroundColor(Qt::gray);
    tab1->setEnabled(true);
    tab3->setEnabled(true);
    
    if (com!=0) delete com;
    com=0;
    
    return true;
}




void MainForm::receive_tmccancom_data()
{
    if (com==0) return;
    bool res = true;
    RobotCtr2::CanCyclicDataFrame candata;
    res &= com->getNextCyclicFrame(candata, 20000);
    if (!res) {
	msg_out->append("<font color=res>Error in receivibg data.<\font>");
	return;
    }
    
    lineEdit_v1->setText(QString::number(candata.vel.motorVel0));
    lineEdit_v2->setText(QString::number(candata.vel.motorVel1));
    lineEdit_v3->setText(QString::number(candata.vel.motorVel2));
    
}


void MainForm::send_tmccancom_cmd()
{
    if (com==0) return;
    com->sendMotorVel( spinBox_u1->value(), 
		       spinBox_u2->value(),
		       spinBox_u3->value(),
		       MOTOR_CTR_MODE_PWM, 0, 0);
    
    
}


void MainForm::enale_sending_tmccancom_cmd( bool a)
{
    if (a) {
	s_timer->start(30, false);
	pushButton_tmccancom_send->setPaletteBackgroundColor(Qt::green);
    }
    else
    {
	s_timer->stop();
	pushButton_tmccancom_send->setPaletteBackgroundColor(Qt::gray);
    }
}
