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


void MonitorForm::init()
{
    monitor = 0;
    monitor = new Tribots::OnlineRobotMonitorClient();
    monitoring = false;
    
    dataWindowLength = 1000;
    plotcounter                = 0;
    for (int i=0; i<3; i++) {
	wvist_plotbuf[i] = (double*) malloc(dataWindowLength * sizeof(double));
	wvsoll_plotbuf[i] = (double*) malloc(dataWindowLength * sizeof(double));
	rvist_plotbuf[i] = (double*) malloc(dataWindowLength * sizeof(double));
	rvsoll_plotbuf[i] = (double*) malloc(dataWindowLength * sizeof(double));
	for (int h=0; h<dataWindowLength; h++) {
	    wvist_plotbuf[i][h] = 0.0;
	    wvsoll_plotbuf[i][h] = 0.0;
	    rvist_plotbuf[i][h]  = 0.0;
	    rvsoll_plotbuf[i][h]  = 0.0;
	}
    }
    x_plotbuf = (double* ) malloc(dataWindowLength * sizeof(double));
    for (int i=0; i< dataWindowLength; i++) x_plotbuf[i] = i;
   
    qwtPlotWV1->setAxisScale(QwtPlot::yLeft, -70.0, 70.0);
    qwtPlotWV2->setAxisScale(QwtPlot::yLeft, -70.0, 70.0);  
    qwtPlotWV3->setAxisScale(QwtPlot::yLeft, -70.0, 70.0);
    
    qwtPlotRVX->setAxisScale(QwtPlot::yLeft, -4.0, 4.0);
    qwtPlotRVY->setAxisScale(QwtPlot::yLeft, -4.0, 4.0);
    qwtPlotRVPhi->setAxisScale(QwtPlot::yLeft, -12.0, 12.0);
     
    qwtzoomerWV1 = new QwtPlotZoomer(qwtPlotWV1->canvas() );
    qwtzoomerWV1->setSelectionFlags( QwtPicker::DragSelection );
    qwtzoomerWV2 = new QwtPlotZoomer(qwtPlotWV2->canvas() );
    qwtzoomerWV2->setSelectionFlags( QwtPicker::DragSelection );
    qwtzoomerWV3 = new QwtPlotZoomer(qwtPlotWV3->canvas() );
    qwtzoomerWV3->setSelectionFlags( QwtPicker::DragSelection );
    qwtzoomerRVX = new QwtPlotZoomer(qwtPlotRVX->canvas() );
    qwtzoomerRVX->setSelectionFlags( QwtPicker::DragSelection );
    qwtzoomerRVY = new QwtPlotZoomer(qwtPlotRVY->canvas() );
    qwtzoomerRVY->setSelectionFlags( QwtPicker::DragSelection );
    qwtzoomerRVPhi = new QwtPlotZoomer(qwtPlotRVPhi->canvas() );
    qwtzoomerRVPhi->setSelectionFlags( QwtPicker::DragSelection );
    
    connect(qwtzoomerWV1, SIGNAL(zoomed(const QwtDoubleRect &)), qwtzoomerWV2, SLOT(zoom (const QwtDoubleRect &)));
    connect(qwtzoomerWV1, SIGNAL(zoomed(const QwtDoubleRect &)), qwtzoomerWV3, SLOT(zoom (const QwtDoubleRect &)));
    connect(qwtzoomerRVX, SIGNAL(zoomed(const QwtDoubleRect &)), qwtzoomerRVY, SLOT(zoom (const QwtDoubleRect &)));
   
    wvist_curve[0] = qwtPlotWV1->insertCurve("Ist");
    qwtPlotWV1->setCurvePen(wvist_curve[0], QPen(QColor(0,0,0))); 
    wvist_curve[1] = qwtPlotWV2->insertCurve("Ist");	
    wvist_curve[2] = qwtPlotWV3->insertCurve("Ist");
    
    wvsoll_curve[0] = qwtPlotWV1->insertCurve("soll");
    qwtPlotWV1->setCurvePen(wvsoll_curve[0], QPen(QColor(255,0,0))); 
    wvsoll_curve[1] = qwtPlotWV2->insertCurve("soll");	
    qwtPlotWV2->setCurvePen(wvsoll_curve[1], QPen(QColor(255,0,0))); 
    wvsoll_curve[2] = qwtPlotWV3->insertCurve("soll"); 
    qwtPlotWV3->setCurvePen(wvsoll_curve[2], QPen(QColor(255,0,0))); 
    
    rvist_curve[0] = qwtPlotRVX->insertCurve("Ist");
    rvist_curve[1] = qwtPlotRVY->insertCurve("Ist");	
    rvist_curve[2] = qwtPlotRVPhi->insertCurve("Ist");
    
    rvsoll_curve[0] = qwtPlotRVX->insertCurve("soll");
    qwtPlotRVX->setCurvePen(rvsoll_curve[0], QPen(QColor(255,0,0))); 
    rvsoll_curve[1] = qwtPlotRVY->insertCurve("soll");	
    qwtPlotRVY->setCurvePen(rvsoll_curve[1], QPen(QColor(255,0,0))); 
    rvsoll_curve[2] = qwtPlotRVPhi->insertCurve("soll");
     qwtPlotRVPhi->setCurvePen(rvsoll_curve[2], QPen(QColor(255,0,0))); 
     
    qwtPlotWV1->replot();
    qwtPlotWV2->replot();
    qwtPlotWV3->replot();   
    qwtPlotRVX->replot();
    qwtPlotRVY->replot();
    qwtPlotRVPhi->replot();
     
    timer = new QTimer(this);
    connect( timer, SIGNAL(timeout()), this, SLOT(updateData()) );
}


void MonitorForm::destroy()
{
    monitor->stopcom();
    if (monitor != 0) delete monitor;
    std::cerr << "Bye!\n";
}

void MonitorForm::updateData()
{
    std::vector< Tribots::RobotMonitorData > data;
    monitor->getNewData(data);
    
    for (unsigned int i=0; i<data.size(); i++) {
	//x_plotbuf[plotcounter%dataWindowLength] = plotcounter;
	for (int h=0; h<3; h++) {
	    wvist_plotbuf[h][plotcounter%dataWindowLength] = data[i].wheel_vel_ist[h];
	    wvsoll_plotbuf[h][plotcounter%dataWindowLength] = data[i].wheel_vel_soll[h];	
	    rvist_plotbuf[h][plotcounter%dataWindowLength] = data[i].robot_vel_ist[h];
	    rvsoll_plotbuf[h][plotcounter%dataWindowLength] = data[i].robot_vel_soll[h];
	}		
	plotcounter++;	
    }	
    
    qwtPlotWV1->setCurveData( wvist_curve[0], x_plotbuf, wvist_plotbuf[0], dataWindowLength);
    qwtPlotWV2->setCurveData( wvist_curve[1], x_plotbuf, wvist_plotbuf[1], dataWindowLength);
    qwtPlotWV3->setCurveData( wvist_curve[2], x_plotbuf, wvist_plotbuf[2], dataWindowLength);
    
    qwtPlotWV1->setCurveData( wvsoll_curve[0], x_plotbuf, wvsoll_plotbuf[0], dataWindowLength);
    qwtPlotWV2->setCurveData( wvsoll_curve[1], x_plotbuf, wvsoll_plotbuf[1], dataWindowLength);
    qwtPlotWV3->setCurveData( wvsoll_curve[2], x_plotbuf, wvsoll_plotbuf[2], dataWindowLength);
    
    qwtPlotRVX->setCurveData( rvist_curve[0], x_plotbuf, rvist_plotbuf[0], dataWindowLength);
    qwtPlotRVY->setCurveData( rvist_curve[1], x_plotbuf, rvist_plotbuf[1], dataWindowLength);
    qwtPlotRVPhi->setCurveData( rvist_curve[2], x_plotbuf, rvist_plotbuf[2], dataWindowLength);
    
    qwtPlotRVX->setCurveData( rvsoll_curve[0], x_plotbuf, rvsoll_plotbuf[0], dataWindowLength);
    qwtPlotRVY->setCurveData( rvsoll_curve[1], x_plotbuf, rvsoll_plotbuf[1], dataWindowLength);
    qwtPlotRVPhi->setCurveData( rvsoll_curve[2], x_plotbuf, rvsoll_plotbuf[2], dataWindowLength);
    
    qwtPlotWV1->replot();
    qwtPlotWV2->replot();
    qwtPlotWV3->replot(); 
    qwtPlotRVX->replot();
    qwtPlotRVY->replot();
    qwtPlotRVPhi->replot();
}




void MonitorForm::startcom()
{
    monitor->startcom();
    monitoring = true;
    timer->start(300);
}


void MonitorForm::stopcom()
{
    monitor->stopcom();
    monitoring=false;
    timer->stop();
}


void MonitorForm::setPIDValues()
{
    monitor->sendPIDSettings( spinBoxKP->value(), spinBoxKI->value(), spinBoxKD->value());
}
