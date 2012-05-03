/*
 *  qtCost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtHrics.hpp"
#include "ui_qtHrics.h"

#include <iostream>
#include <tr1/memory>
#include <boost/bind.hpp>
#include <sys/time.h>
#include <QMessageBox>

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "qtLibrary.hpp"
#include "planner_handler.hpp"

#ifdef USE_QWT
#include "qtPlot/basicPlot.hpp"
#include "qtPlot/multiPlot.hpp"
#include "qtPlot/tempWin.hpp"
#endif

#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"

#include "qtMotionPlanner.hpp"

#include "HRI_costspace/HRICS_costspace.hpp"
#include "planner/planEnvironment.hpp"
#include "utils/ConfGenerator.h"

using namespace std;
using namespace tr1;

extern Eigen::Vector3d global_DrawnSphere;

HricsWidget::HricsWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::HricsWidget)
{
	m_ui->setupUi(this);
	
	//this->initHRI();
	//this->initHumanLike();
}

HricsWidget::~HricsWidget()
{
	delete m_ui;
	//#ifdef USE_QWT
	//    delete this->plot;
	//#endif
}

//---------------------------------------------------------------------
// HRI
//---------------------------------------------------------------------
void HricsWidget::initHRI()
{	
//  if (ENV.getBool(Env::isCostSpace)) 
//  {
//    std::cout << "Initializing HRI Cost Function" << std::endl;
//    global_costSpace->addCost("costHRI",boost::bind(HRICS_getConfigCost,_1));
//    global_costSpace->setCost("costHRI");
//  }
  
  cout << "Init HRICS widget" << endl;
  
  connect(this, SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));
	
	m_mainWindow->connectCheckBoxToEnv(m_ui->enableHri_2,										Env::enableHri);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawGrid,							Env::drawGrid);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxEntireGrid,						Env::drawEntireGrid);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawDistance,					Env::drawDistance);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawRandPoints,				Env::drawPoints);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawGaze,							Env::drawGaze);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawVectorField,				Env::drawVectorField);
  m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawOnlyOneLine,				Env::drawOnlyOneLine);
  m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawBox,               Env::drawBox);
  m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawHumanColorFromConf,PlanParam::hriSetColorFromConfig);
  
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRICS_MOPL,						Env::HRIPlannerWS);
//	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxBBDist,								Env::useBoxDist);
//	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxBallDist,							Env::useBallDist);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriPlannerTRRT,				Env::HRIPlannerTRRT);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriPathDistance,				Env::HRIPathDistance);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCameraBehindHuman,			Env::HRIcameraBehindHuman);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRINoRobot,						Env::HRINoRobot);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxAutoLoadGrid,					Env::HRIAutoLoadGrid);
	
	connect(m_ui->checkBoxDrawGrid,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));
	connect(m_ui->checkBoxEntireGrid,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));
  
  connect(m_ui->pushButtonInitGrids,SIGNAL(clicked()),this,SLOT(initGrids()));
  connect(m_ui->pushButtonDeleteGrids,SIGNAL(clicked()),this,SLOT(deleteGrids()));
  connect(m_ui->pushButtonComputeAllCellCost,SIGNAL(clicked()),this,SLOT(computeAllCellCost()));
	
  // Load and Save agent grids
  connect(m_ui->pushButtonLoad,SIGNAL(clicked()),this,SLOT(loadGrid()));
  connect(m_ui->pushButtonSave,SIGNAL(clicked()),this,SLOT(saveGrid()));
  
  // Simple Otp test
  connect(m_ui->pushButtonComputeOtpConfig,SIGNAL(clicked()),this,SLOT(computeOtpConfig()));
  
  // A*
  connect(m_ui->pushButtonAStarInGrid,SIGNAL(clicked()),this,SLOT(computeAStarGrid()));
  
	// -------------------------------
	// K Sliders
	// -------------------------------
	m_k_distance = new QtShiva::SpinBoxSliderConnector(this, 
                                                     m_ui->doubleSpinBoxDistance,		
                                                     m_ui->horizontalSliderDistance,
                                                     Env::Kdistance);
  
  cout << "Env::Kdistance : " << ENV.getDouble(Env::Kdistance) << endl;
	
	m_k_visbility = new QtShiva::SpinBoxSliderConnector(this,
                                                      m_ui->doubleSpinBoxVisibility,
                                                      m_ui->horizontalSliderVisibility,
                                                      Env::Kvisibility );
	
  cout << "Env::Kvisibility : " << ENV.getDouble(Env::Kvisibility) << endl;
  
	m_k_naturality = new QtShiva::SpinBoxSliderConnector(this, 
                                                       m_ui->doubleSpinBoxNatural,
                                                       m_ui->horizontalSliderNatural,
                                                       Env::Knatural );
	
  cout << "Env::Knatural : " << ENV.getDouble(Env::Knatural) << endl;
  
	m_k_reachability = new QtShiva::SpinBoxSliderConnector(this, 
                                                         m_ui->doubleSpinBoxReachable,		
                                                         m_ui->horizontalSliderReachable,	
                                                         Env::Kreachable );
  
  cout << "Env::Kreachable : " << ENV.getDouble(Env::Kreachable) << endl;
	
//	connect(m_k_distance,SIGNAL(valueChanged(double)),this,SLOT(KDistance(double)));
//	connect(m_k_visbility,SIGNAL(valueChanged(double)),this,SLOT(KVisibility(double)));
  
  // -------------------------------
	// Cells	
	// -------------------------------
  new QtShiva::SpinBoxSliderConnector(this, 
                                      m_ui->doubleSpinBoxCellSize,		
                                      m_ui->horizontalSliderCellSize,
                                      Env::CellSize);
  
  new QtShiva::SpinBoxSliderConnector(this, 
                                      m_ui->doubleSpinBoxColor1,		
                                      m_ui->horizontalSliderColor1,
                                      Env::colorThreshold1);
	
  new QtShiva::SpinBoxSliderConnector(this, 
                                      m_ui->doubleSpinBoxColor2,		
                                      m_ui->horizontalSliderColor2,
                                      Env::colorThreshold2);
  
	// -------------------------------
	// Wich Test	
	// -------------------------------
	connect(m_ui->whichTestBox, SIGNAL(currentIndexChanged(int)),ENV.getObject(Env::hriCostType), SLOT(set(int)), Qt::DirectConnection);
	connect(ENV.getObject(Env::hriCostType), SIGNAL(valueChanged(int)),this, SLOT(setWhichTestSlot(int)), Qt::DirectConnection);
	m_ui->whichTestBox->setCurrentIndex(ENV.getInt(Env::hriCostType));
	
	// -------------------------------
	// All widgets
	// -------------------------------
	initVectorField();
	initDrawOneLineInGrid();
//  initGrids();
//  initObjectTransferPoint();
  
//  setGroupBoxDisabled(true);
}

void HricsWidget::setGroupBoxDisabled(bool disable)
{
  m_ui->groupBoxCostFunctions->setDisabled( disable );
  m_ui->groupBoxDraw->setDisabled( disable );
  m_ui->groupBoxGrids->setDisabled( disable );
  m_ui->groupBoxHRICSPlanner->setDisabled( disable );
  m_ui->groupBoxMisc->setDisabled( disable );
}

void HricsWidget::drawAllWinActive()
{
	m_mainWindow->drawAllWinActive();	
}

/*!
 * Zone size (protective bubble) changed
 */
void HricsWidget::zoneSizeChanged()
{
	if(ENV.getBool(Env::HRIPlannerWS))
	{
		HRICS_activeDist = HRICS_MotionPL->getDistance();
		HRICS_activeDist->parseHumans();
	}
	else if(ENV.getBool(Env::HRIPlannerCS))
	{
		HRICS_activeDist = HRICS_MotionPL->getDistance();
		HRICS_activeDist->parseHumans();
	}
	
	m_mainWindow->drawAllWinActive();
	cout << "Zone Size Changed" << endl;
}

void HricsWidget::initVectorField()
{
	connect(m_ui->pushButtonVectorField,SIGNAL(clicked()),this,SLOT(computeVectorField()));
}

void HricsWidget::computeVectorField()
{
  dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid()->computeVectorField();
  API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid();
	ENV.setBool(Env::drawGrid,true);
	ENV.setBool(Env::drawVectorField,true);
	m_mainWindow->drawAllWinActive();
}

//-------------------------------------------------------------
// Object Transfer Point
//-------------------------------------------------------------
void HricsWidget::initObjectTransferPoint()
{
	connect(m_k_distance,		SIGNAL(valueChanged(double)),this,SLOT(computeObjectTransferPoint()));
	connect(m_k_visbility,		SIGNAL(valueChanged(double)),this,SLOT(computeObjectTransferPoint()));
	connect(m_k_reachability,	SIGNAL(valueChanged(double)),this,SLOT(computeObjectTransferPoint()));
}

// Compute the object transfer point
void HricsWidget::computeObjectTransferPoint()
{
        qDebug() << "HricsWidget::computeObjectTransferPoint";

        ENV.setBool(Env::HRIComputeOTP,true);

        if ( ENV.getBool(Env::HRIComputeOTP) )
	{
		Eigen::Vector3d WSPoint;
                qDebug() << "HricsWidget::computeObjectTransferPoint Eigen";
		if( dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computeBestTransferPoint(WSPoint) )
		{
			HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();
                        reachSpace->computeIsReachableAndMove(WSPoint,reachSpace->getGrid()->isReachableWithLA(WSPoint));
			
			Robot* Object = global_Project->getActiveScene()->getRobotByNameContaining("OBJECT");
			
			shared_ptr<Configuration> q_curr = Object->getCurrentPos();
			
			(*q_curr)[6] = WSPoint[0];
			(*q_curr)[7] = WSPoint[1];
			(*q_curr)[8] = WSPoint[2];
			
//                        qDebug() << "| z : "<< WSPoint[0] <<"| z : "<<  WSPoint[1] <<  "| z : "<< WSPoint[2] ;

			Object->setAndUpdate(*q_curr);
			
			cout << "Set and update : " << Object->getName() << endl << WSPoint << endl;
			
			m_mainWindow->drawAllWinActive();
		}
	}
}

//-------------------------------------------------------------
// Select One Slice
//-------------------------------------------------------------
void HricsWidget::initDrawOneLineInGrid()
{
	connect(m_ui->radioButtonXline, SIGNAL(toggled(bool)), this, SLOT(set_X_line(bool)));
	connect(m_ui->radioButtonYline, SIGNAL(toggled(bool)), this, SLOT(set_Y_line(bool)));
	connect(m_ui->radioButtonZline, SIGNAL(toggled(bool)), this, SLOT(set_Z_line(bool)));
	
	QtShiva::SpinBoxSliderConnector* connector =  new QtShiva::SpinBoxSliderConnector(
																																										this, m_ui->doubleSpinBoxWhichGridLine, m_ui->horizontalSliderWhichGridLine , Env::hriShownGridLine );
	
	connect(connector,SIGNAL(valueChanged(double)),m_mainWindow,SLOT(drawAllWinActive()));
	
}

void HricsWidget::set_X_line(bool enable)
{	
	if (enable && API_activeGrid) 
	{
		unsigned int maxLines = dynamic_cast<API::ThreeDGrid*>(API_activeGrid)->getXNumberOfCells();
		m_ui->doubleSpinBoxWhichGridLine->setMaximum(maxLines);
	}
	
	ENV.setInt(Env::lineToShow,0);
}

void HricsWidget::set_Y_line(bool enable)
{
	if (enable && API_activeGrid) 
	{
		unsigned int maxLines = dynamic_cast<API::ThreeDGrid*>(API_activeGrid)->getYNumberOfCells();
		m_ui->doubleSpinBoxWhichGridLine->setMaximum(maxLines);
	}
	
	ENV.setInt(Env::lineToShow,1);
}


void HricsWidget::set_Z_line(bool enable)
{
	if (enable && API_activeGrid) 
	{
		unsigned int maxLines = dynamic_cast<API::ThreeDGrid*>(API_activeGrid)->getZNumberOfCells();
		m_ui->doubleSpinBoxWhichGridLine->setMaximum(maxLines);
	}
	
	ENV.setInt(Env::lineToShow,2);
}


//-------------------------------------------------------------
// IK Grids
//-------------------------------------------------------------
void HricsWidget::setWhichTestSlot(int test)
{
	cout << "Change test to :" << test << endl;
	
	if(ENV.getBool(Env::HRIPlannerTS))
	{
#ifdef HRI_PLANNER
		//hriSpace->changeTest(test);
		cout << "HricsWidget::setWhichTestSlot::WARNING" << endl;
#else
		cout << "HRI Planner not compiled nor linked" << endl;
#endif
	}
}

//-------------------------------------------------------------
// Plan Workspace HRI
//-------------------------------------------------------------
void HricsWidget::make3DHriGrid()
{
  Robot* Human = global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");
  
  shared_ptr<Configuration> q = Human->getCurrentPos();
  
	HRICS_MotionPL = new HRICS::Workspace;
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initGrid();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initDistance();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initVisibility();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initNatural();
  dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initAgentGrids(ENV.getDouble(Env::CellSize));
  
  // Protective bubble
  QtShiva::SpinBoxSliderConnector* connectorZoneSize  = new QtShiva::SpinBoxSliderConnector(
                                                                                            this, m_ui->doubleSpinBoxZoneSize, m_ui->horizontalSliderZoneSize ,Env::zone_size );
  connect(connectorZoneSize,SIGNAL(valueChanged(double)),this,SLOT(zoneSizeChanged()),Qt::DirectConnection);
	
	/*if (PointsToDraw != NULL) 
	 {
	 delete PointsToDraw;
	 }*/
	
	PointsToDraw = new PointCloud;
	
	//m_ui->HRICSPlanner->setDisabled(false);
	ENV.setBool(Env::HRIPlannerWS,true);
	//    ENV.setBool(Env::biDir,false);
	ENV.setDouble(Env::zone_size,0.5);
	HRICS_activeDist = HRICS_MotionPL->getDistance();
	
  //API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid();
	//API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid();
	//    enableHriSpace();
	
	m_ui->pushButtonMakeGrid->setDisabled(true);
	m_ui->pushButtonDeleteGrid->setDisabled(false);
	
	ENV.setBool(Env::enableHri,true);
	ENV.setBool(Env::isCostSpace,true);
	
	if( ENV.getBool(Env::HRIAutoLoadGrid) )
	{
		// Reads a traj from a file
    //		string filename("/Users/jmainpri/workspace/BioMove3DDemos/CostHriFunction/JidoTrajectory/JidoK.traj");
    //		qt_fileName = filename.c_str();
    //		qt_readTraj();
		
    QString fileName("/statFiles/Cost3DGrids/Cost3DGrid2.grid");
    QString home( getenv("HOME_MOVE3D") );
    fileName = home + fileName;
		// Reads the grid from XML and sets it ti the HRICS_MotionPL
		qt_load_HRICS_Grid(fileName.toStdString());
		
    //		m_ui->HRICSNatural->setDisabled(false);
    //		m_ui->pushButtonNewNaturalCostSpace->setDisabled(true);
    //		m_ui->activeGridsBox->addItem(dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid)->getName().c_str());
    
//    api_store_new_grid( dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid) );
//		HRICS_activeNatu->setGrid(dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid));
//		ENV.setBool(Env::drawGrid,false);
	}
	
//	ENV.setInt(Env::hriCostType,HRICS_Combine);
  m_mainWindow->drawAllWinActive();
  
  Human->setAndUpdate( *q );
  //        m_mainWindow->Ui()->tabCost->initCostFunctions();
	m_mainWindow->Ui()->tabCost->setCostFunction("costHRI");
  m_mainWindow->drawAllWinActive();
  
  cout << "new HRI Workspace" << endl;
  cout << "set Agent Grid as API_activeGrid" << endl;
  API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getAgentGrids()[0];
}

void HricsWidget::delete3DHriGrid()
{
	ENV.setBool(Env::drawGrid,false);
	ENV.setBool(Env::HRIPlannerWS,false);
	
	delete HRICS_MotionPL;
	HRICS_MotionPL = NULL;
	
	//m_ui->HRICSPlanner->setDisabled(true);
	
	m_mainWindow->drawAllWinActive();
	
	m_ui->pushButtonMakeGrid->setDisabled(false);
	m_ui->pushButtonDeleteGrid->setDisabled(true);
}

//-------------------------------------------------------------
// Human Cost Space
//-------------------------------------------------------------
void HricsWidget::initGrids()
{
  // Get the HRICS workspace
  HRICS::Workspace* ws = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL);
  
  // If a costspace allready exists delete it!
  if(HRICS_humanCostMaps!=NULL)
    delete HRICS_humanCostMaps;

  // Build a costspace from the Robot and Humans in the workspace
  // This costspace will hold the agents grids
  HRICS_humanCostMaps = new HRICS::HumanCostSpace(ws->getRobot(),ws->getHumans(), HRICS_activeNatu, ENV.getDouble(Env::CellSize));
}

void HricsWidget::deleteGrids()
{
  delete HRICS_humanCostMaps;
  HRICS_humanCostMaps = NULL;
}

void HricsWidget::computeAllCellCost()
{
  emit(selectedPlanner(QString("ComputeAgentGridCost")));
}

void HricsWidget::loadGrid()
{
  emit(selectedPlanner(QString("LoadAgentGrid")));
}

void HricsWidget::saveGrid()
{
  emit(selectedPlanner(QString("SaveAgentGrid")));
}

//-------------------------------------------------------------
// OTHERS
//-------------------------------------------------------------
void HricsWidget::resetRandomPoints()
{
	ENV.setBool(Env::drawPoints,false);
	if(PointsToDraw != NULL)
	{
		delete PointsToDraw;
	}
}

void HricsWidget::AStarIn3DGrid()
{
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computeAStarIn3DGrid();
	ENV.setBool(Env::drawTraj,true);
	m_mainWindow->drawAllWinActive();
}

void HricsWidget::HRICSRRT()
{
	//    std::string str = "runHRICSRRT";
	//    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
	
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initHriRRT();
	ENV.setBool(Env::drawTraj,true);
	m_mainWindow->drawAllWinActive();
}

void HricsWidget::computeOtpConfig()
{
  Scene* sce = global_Project->getActiveScene();
  
  Robot* rob = sce->getRobotByNameContaining("ROBOT");
  Robot* hum = sce->getRobotByNameContaining("HUMAN");

  ConfGenerator generator( rob, hum );

  Eigen::Vector3d point = hum->getJoint("rPalm")->getVectorPos();
  point[2] += 0.10;
  
  configPt q; // = q_rob->getConfigStruct();
  
  timeval tim;
  gettimeofday(&tim, NULL);
  double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);
  
  if( generator.computeRobotGikForGrabing( q, point ) )
  {
    gettimeofday(&tim, NULL);
    double dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
    cout << "Ik computed in : " << dt << " sec" << endl;
    
    confPtr_t q_rob(new Configuration(rob,q));
    rob->setAndUpdate(*q_rob);
    m_mainWindow->drawAllWinActive();
  }
  else
  {
    cout << "Could not find a robot OTP configuration" << endl;
  }
}

void HricsWidget::on_pushButton_initColPos_clicked()
{
    HRICS_activeNatu->setRobotToConfortPosture();
    HRICS_activeNatu->setRobotColorFromConfiguration(false);
    m_mainWindow->drawAllWinActive();
}

void HricsWidget::on_checkBoxPosOr_toggled(bool checked)
{
    if (checked)
    {
        m_mh = new MovingHuman();
        m_mh->setMainWindow(m_mainWindow);
        m_mh->show();
    }
    else
    {
        delete m_mh;
    }
}

void HricsWidget::computeAStarGrid()
{
  emit(selectedPlanner(QString("ComputeAStar")));
}

