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
#include <QMessageBox>

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "qtLibrary.hpp"
#include "planner_handler.hpp"

#ifdef USE_QWT
#include "qtPlot/basicPlot.hpp"
#include "qtPlot/doublePlot.hpp"
#include "qtPlot/tempWin.hpp"
#endif

#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"

#include "qtMotionPlanner.hpp"

#include "HRI_costspace/HRICS_costspace.hpp"

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
	
	m_mainWindow->connectCheckBoxToEnv(m_ui->enableHri_2,										Env::enableHri);
	m_mainWindow->connectCheckBoxToEnv(m_ui->enableHriTS,										Env::HRIPlannerTS);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawGrid,							Env::drawGrid);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxEntireGrid,							Env::drawEntireGrid);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawDistance,					Env::drawDistance);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawRandPoints,				Env::drawPoints);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawGaze,							Env::drawGaze);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawVectorField,				Env::drawVectorField);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRICS_MOPL,						Env::HRIPlannerWS);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxBBDist,								Env::useBoxDist);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxBallDist,							Env::useBallDist);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRIGoalBiased,					Env::isGoalBiased);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxInverseKinematics,			Env::isInverseKinematics);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawOnlyOneLine,				Env::drawOnlyOneLine);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxEnableHRIConfigSpace,  Env::HRIPlannerCS);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriPlannerTRRT,				Env::HRIPlannerTRRT);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriPathDistance,				Env::HRIPathDistance);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriLeftArmVsRightArm,	Env::HRIleftArmVsRightArm);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCameraBehindHuman,			Env::HRIcameraBehindHuman);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxComputeOTP,						Env::HRIComputeOTP);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRINoRobot,						Env::HRINoRobot);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxAutoLoadGrid,					Env::HRIAutoLoadGrid);
	
	connect(m_ui->checkBoxDrawGrid,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));
	connect(m_ui->checkBoxEntireGrid,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));
	connect(m_ui->pushButtonHRITS,SIGNAL(clicked()),this,SLOT(enableHriSpace()));
	
	// -------------------------------
	// K Sliders
	// -------------------------------
	m_k_distance = new QtShiva::SpinBoxSliderConnector(
																										 this, m_ui->doubleSpinBoxDistance,		m_ui->horizontalSliderDistance,		Env::Kdistance);
	
	m_k_visbility = new QtShiva::SpinBoxSliderConnector(
																											this, m_ui->doubleSpinBoxVisibility,	m_ui->horizontalSliderVisibility,	Env::Kvisibility );
	
	m_k_naturality = new QtShiva::SpinBoxSliderConnector(
																											 this, m_ui->doubleSpinBoxNatural,		m_ui->horizontalSliderNatural ,		Env::Knatural );
	
	m_k_reachability = new QtShiva::SpinBoxSliderConnector(
																												 this, m_ui->doubleSpinBoxReachable,		m_ui->horizontalSliderReachable ,	Env::Kreachable );
	
	connect(m_k_distance,SIGNAL(valueChanged(double)),this,SLOT(KDistance(double)));
	connect(m_k_visbility,SIGNAL(valueChanged(double)),this,SLOT(KVisibility(double)));
	
	
	// -------------------------------
	// Wich Test	
	// -------------------------------
	connect(m_ui->whichTestBox, SIGNAL(currentIndexChanged(int)),ENV.getObject(Env::hriCostType), SLOT(set(int)), Qt::DirectConnection);
	connect(ENV.getObject(Env::hriCostType), SIGNAL(valueChanged(int)),this, SLOT(setWhichTestSlot(int)), Qt::DirectConnection);
	m_ui->whichTestBox->setCurrentIndex(ENV.getInt(Env::hriCostType));
	
	// -------------------------------
	// Active Joint
	// -------------------------------
	connect(m_ui->spinBoxJoint, SIGNAL(valueChanged(int)),ENV.getObject(Env::akinJntId), SLOT(set(int)), Qt::DirectConnection);
	connect(ENV.getObject(Env::akinJntId), SIGNAL(valueChanged(int)),m_ui->spinBoxJoint, SLOT(setValue(int)), Qt::DirectConnection);
	m_ui->spinBoxJoint->setValue(ENV.getInt(Env::akinJntId));
	
	connect(m_ui->pushButtonWorkspacePath, SIGNAL(clicked()),this, SLOT(computeWorkspacePath()), Qt::DirectConnection);
	connect(m_ui->pushButtonHoleMotion, SIGNAL(clicked()),this, SLOT(computeHoleMotion()), Qt::DirectConnection);
	
	connect(m_ui->pushButtonSaveActiveGridToFile,SIGNAL(clicked()),this,SLOT(saveActiveGridToFile()));
	connect(m_ui->pushButtonLoadActiveGridFromFile,SIGNAL(clicked()),this,SLOT(loadActiveGridFromFile()));
	
	// -------------------------------
	// All widgets
	// -------------------------------
	initVectorField();
	initNaturalSpace();
	initConfigSpace();
	initTaskSpace();
	initDrawOneLineInGrid();
  initGrids();
	//initObjectTransferPoint();

	m_ui->groupBox_gridPreperties->hide();
	m_ui->NaturalParameters->hide();
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
// Select One line
//-------------------------------------------------------------
void HricsWidget::initDrawOneLineInGrid()
{
	connect(m_ui->radioButtonXline, SIGNAL(toggled(bool)), this, SLOT(set_X_line(bool)));
	connect(m_ui->radioButtonYline, SIGNAL(toggled(bool)), this, SLOT(set_Y_line(bool)));
	connect(m_ui->radioButtonZline, SIGNAL(toggled(bool)), this, SLOT(set_Z_line(bool)));
	
	QtShiva::SpinBoxSliderConnector* connector =  new QtShiva::SpinBoxSliderConnector(
																																										this, m_ui->doubleSpinBoxWhichGridLine, m_ui->horizontalSliderWhichGridLine , Env::hriShownGridLine );
	
	connect(connector,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
	
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
// Active Grids
//-------------------------------------------------------------
Ui::HricsWidget* static_ui;

void add_grid_to_ui( API::BaseGrid* grid)
{
  static_ui->activeGridsBox->addItem( grid->getName().c_str() );
}

void HricsWidget::initGrids()
{
	connect(m_ui->activeGridsBox, SIGNAL(currentIndexChanged(int)),this, SLOT(setActiveGrid(int)), Qt::DirectConnection);
	connect(ENV.getObject(Env::hriActiveGrid), SIGNAL(valueChanged(int)),this, SLOT(setActiveGrid(int)), Qt::DirectConnection);
	m_ui->whichTestBox->setCurrentIndex(ENV.getInt(Env::hriActiveGrid));
	
	connect(m_ui->pushButtonMergeGrids, SIGNAL(clicked()), this, SLOT(mergeGrids()));
	connect(m_ui->spinBoxCellToShow, SIGNAL(valueChanged(int)),ENV.getObject(Env::cellToShow),SLOT(set(int)), Qt::DirectConnection);
	connect(m_ui->spinBoxCellToShow, SIGNAL(valueChanged(int)),this,SLOT(cellToShowChanged()), Qt::QueuedConnection);  
  
  static_ui = m_ui;
  ext_add_grid_to_ui = add_grid_to_ui;
}

void HricsWidget::setActiveGrid(int ith_grid)
{
	cout << "Change grid to :" << ith_grid << endl;
	
	if (ith_grid == 0) 
	{
		ENV.setBool(Env::drawGrid,false);
		API_activeGrid = NULL;
	}
	else
	{
		ENV.setBool(Env::drawGrid,true);
    vector<API::BaseGrid*> API_allGrids = api_get_all_grids();
		API_activeGrid = API_allGrids[ith_grid-1];
	}
	
	m_mainWindow->drawAllWinActive();
}

void HricsWidget::mergeGrids()
{
  vector<API::BaseGrid*> API_allGrids = api_get_all_grids();
  
	HRICS::NaturalGrid* first = dynamic_cast<HRICS::NaturalGrid*>(API_allGrids[0]);
	HRICS::NaturalGrid* secon = dynamic_cast<HRICS::NaturalGrid*>(API_allGrids[1]);
	
	HRICS::NaturalGrid* third = first->mergeWith(secon);
	
	m_ui->activeGridsBox->addItem(third->getName().c_str());
	API_allGrids.push_back(third);
	
	API_activeGrid = third;	
	
	m_mainWindow->drawAllWinActive();
}

void HricsWidget::cellToShowChanged()
{
	HRICS::NaturalGrid* myGrid = HRICS_activeNatu->getGrid();
	
//	vector<HRICS::NaturalCell*> ReachableCells = myGrid->getAllReachableCells();
	
	vector<pair<double,HRICS::NaturalCell*> > ReachableCells = myGrid->getAllReachableCellsSorted();
	if (ReachableCells.empty()) 
	{
		cout << "Warning :: ReachableCells is empty!!!" << endl;
	}
	m_ui->spinBoxCellToShow->setMaximum(ReachableCells.size()-1);
	HRICS_activeNatu->setRobotToConfortPosture();
	
	int ith = ENV.getInt(Env::cellToShow);
	//	ReachableCells[ith]->setBlankCost();
	//	ReachableCells[ith]->getCost();
	cout << "Showing reachable cell : " << ith << endl;
	cout << "Number of reachable cell : " << ReachableCells.size()  << endl;

	if ( (ith >= 0) && ( ReachableCells.size() > ((unsigned int)ith)) ) 
	{
		shared_ptr<Configuration> q_init = myGrid->getRobot()->getInitialPosition();
		HRICS_activeNatu->setRobotToConfortPosture();
		
		//global_DrawnSphere = ReachableCells[ith]->getWorkspacePoint();
		
#ifdef HRI_PLANNER
		if( HRICS_activeNatu->computeIsReachableAndMove(ReachableCells[ith].second->getWorkspacePoint(),
                                                    ReachableCells[ith].second->isReachableWithLA()))
    {
     cout << "Compute is reachable succeded" << endl; 
    }
    else 
    {
      HRICS_activeNatu->setRobotToConfortPosture();
      if( HRICS_activeNatu->computeIsReachableAndMove( ReachableCells[ith].second->getWorkspacePoint(),
                                                      !ReachableCells[ith].second->isReachableWithLA()))
      {
            cout << "Compute is reachable succeded" << endl; 
      }
      else {
        HRICS_activeNatu->setRobotToConfortPosture();
        cout << "IK failed" << endl;
      }
    }
    
    
		
//		cout << "Cell Cost = " << ReachableCells[ith]->getCost() << endl;
		cout << "Cell Cost = " << ReachableCells[ith].second->getCost() << endl;
#else
		cout << "Warning: HRI_PLANNER: Not compiled" << endl;
#endif
		g3d_set_draw_coll(myGrid->getRobot()->isInCollision());
		
		m_mainWindow->drawAllWinActive();
	}
	else 
	{
		cout << "Exede the number of cells" << endl;
	}
}

void HricsWidget::saveActiveGridToFile()
{
    string docname;
    QString fileName = QFileDialog::getSaveFileName(this,
                                tr("Save Grid"),
                                "Cost3DGrid.grid",
                                tr("Grid Files (*.grid);;All Files (*)"));
    if (!fileName.isEmpty())
        docname = fileName.toStdString();
        API_activeGrid->writeToXmlFile(docname);
  
//	string docname("Cost3DGrid.grid");
//	API_activeGrid->writeToXmlFile(docname);
}

void HricsWidget::loadActiveGridFromFile()
{
	if (HRICS_MotionPL == NULL) 
	{
		//cout << "Warning :: No HRICS Motion Planner" << endl;
		QMessageBox msgBox;
		msgBox.setText("No HRICS Motion Planner\n Do you wish to continue?");
		msgBox.setStandardButtons( QMessageBox::Cancel | QMessageBox::Ok );
		msgBox.setIcon( QMessageBox::Critical );
		
		if ( msgBox.exec() == QMessageBox::Cancel ) 
		{
			return;
		}
	}
	
	QString fileName = QFileDialog::getOpenFileName(this);
	
	if (!fileName.isEmpty())
	{
		m_ui->HRICSNatural->setDisabled(false);
		m_ui->pushButtonNewNaturalCostSpace->setDisabled(true);
		
		// Reads the grid from XML and sets it ti the HRICS_MotionPL
		qt_load_HRICS_Grid(fileName.toStdString());
		
		m_ui->activeGridsBox->addItem( API_activeGrid->getName().c_str() );
		api_store_new_grid( dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid) );
		HRICS_activeNatu->setGrid( dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid) );
		ENV.setBool(Env::drawGrid,true);
		m_mainWindow->drawAllWinActive();
	}
}

//-------------------------------------------------------------
// INIT (HRICS_Natural)
//-------------------------------------------------------------
void HricsWidget::initNaturalSpace()
{
	m_ui->HRICSNatural->setDisabled(true);
	
	connect(m_ui->pushButtonNewNaturalCostSpace,SIGNAL(clicked()),this,SLOT(newNaturalCostSpace()));
	connect(m_ui->pushButtonDeleteNaturalCostSpace,SIGNAL(clicked()),this,SLOT(deleteNaturalCostSpace()));
	
	connect(m_ui->pushButtonRecomputeReach,SIGNAL(clicked()),this,SLOT(recomputeReachability()));
	connect(m_ui->pushButtonComputeAllNatCost,SIGNAL(clicked()),this,SLOT(computeAllCellNaturalCost()));
	connect(m_ui->pushButtonComputeReachability,SIGNAL(clicked()),this,SLOT(computeReachability()));
	connect(m_ui->pushButtonGetSortedCells,SIGNAL(clicked()),this,SLOT(getSortedReachableWSPoint()));
	
	new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxNeutralDist, m_ui->horizontalSliderNeutralDist , Env::coeffJoint );
	
	new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxEnergy, m_ui->horizontalSliderEnergy , Env::coeffEnerg );
	
	new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxDiscomfort, m_ui->horizontalSliderDiscomfort , Env::coeffConfo );

	new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxArmPref, m_ui->horizontalSliderArmPref , Env::coeffArmPr );

	ENV.setDouble(Env::coeffJoint,0.5);
	ENV.setDouble(Env::coeffEnerg,1.0);
	ENV.setDouble(Env::coeffConfo,10.0);
	ENV.setDouble(Env::coeffArmPr,0.05);

}
//-------------------------------------------------------------
// Natural Cost Space (HRICS_Natural)
//-------------------------------------------------------------
void HricsWidget::newNaturalCostSpace()
{
	Robot* robotPt = global_Project->getActiveScene()->getActiveRobot();
	HRICS_activeNatu  = new HRICS::Natural(robotPt);
	HRICS_activeNatu->computeNaturalGrid();
	
	API_activeGrid = HRICS_activeNatu->getGrid();
	
	ENV.setBool(Env::drawGrid,true);
	m_mainWindow->drawAllWinActive();
	
	m_ui->HRICSNatural->setDisabled(false);
	m_ui->pushButtonNewNaturalCostSpace->setDisabled(true);
	
	ENV.setBool(Env::isCostSpace,true);
	ENV.setBool(Env::enableHri,true);
	
	/*if (PointsToDraw != NULL) {
	 delete PointsToDraw;
	 }*/
	
	PointsToDraw = new PointCloud;
	
	
	//	for (int i=0 ; i<HRICS_Natural->getGrid()->getNumberOfCells(); i++) 
	//	{
	//		int config = HRICS_Natural->getGrid()->robotConfigInCell(i);
	//		
	//		if (config != -1) 
	//		{
	//			cout << "Respect Constraint = " << config << endl;
	//		}
	//		
	//		m_mainWindow->drawAllWinActive();
	//	}
}

void HricsWidget::deleteNaturalCostSpace()
{
	ENV.setBool(Env::drawGrid,false);
	API_activeGrid = NULL;
	delete HRICS_activeNatu;
	HRICS_activeNatu = NULL;
	
	/*if (PointsToDraw != NULL) {
	 delete PointsToDraw;
	 PointsToDraw = NULL;
	 }*/
	
	m_ui->HRICSNatural->setDisabled(true);
	m_ui->pushButtonNewNaturalCostSpace->setDisabled(false);
	
	m_mainWindow->drawAllWinActive();
}

void HricsWidget::recomputeReachability()
{
	vector<HRICS::NaturalCell*> cells = dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid)->getAllReachableCells();
	
	/*for (unsigned int i=0; i<cells.size(); i++) 
	 {
	 cells[i]->computeReachability(true);
	 cells[i]->computeReachability(false);
	 }*/
	
	for (unsigned int i=0; i<cells.size(); i++) 
	{
		if ( cells[i]->getCost() <= 0.0	)
		{
			cells[i]->setIsReachable(false);
		}
	}
	
	cout << "Reachability recomputed" << endl;
}

void HricsWidget::computeAllCellNaturalCost()
{
	HRICS::NaturalGrid* grid = dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid);
	
	grid->resetCellCost(); 
	grid->computeAllCellCost();
	
	m_mainWindow->drawAllWinActive();

//	unsigned int j=0;

//	for (unsigned int i=0; i<grid->getNumberOfCells(); i++)
//	{
//		HRICS::NaturalCell* cell = dynamic_cast<HRICS::NaturalCell*>(grid->BaseGrid::getCell(i));
		
//		if( cell->isReachable() )
//		{
//			if (cell->getCost() == numeric_limits<double>::max() )
//			{
//				cell->setIsReachable(false);
//				j++;
//			}
//		}
//	}
//	cout << j << " cells lost!!!" << endl;

	cout << "All Cell Cost Computed" << endl;
//	cout << nbCells - cells.size() << " cells lost!!!" << endl;
	m_mainWindow->drawAllWinActive();

}

void HricsWidget::computeReachability()
{
	HRICS_activeNatu->getGrid()->resetCellCost();
	HRICS_activeNatu->getGrid()->resetReachability();

//	HRICS_activeNatu->setRobotToConfortPosture();

#ifdef HRI_PLANNER
//	HRICS_activeNatu->getGrid()->computeReachability(ENV.getBool(Env::HRIleftArmVsRightArm));
	HRICS_activeNatu->getGrid()->computeReachability();
#else
	cout << "HRI planner not defined" << endl;
#endif

	vector<HRICS::NaturalCell*> cells = HRICS_activeNatu->getGrid()->getAllReachableCells();

	for (unsigned int i=0; i<cells.size(); i++)
	{
		cout << "Cost of cell = " << cells[i]->getCost() << endl;
	}

	m_mainWindow->drawAllWinActive();
}

void HricsWidget::getSortedReachableWSPoint()
{
	HRICS_activeNatu->getSortedReachableWSPoint();
}

//-------------------------------------------------------------
// Config Cost Space HRICS_CSpaceMPL
//-------------------------------------------------------------
void HricsWidget::newHRIConfigSpace()
{
	m_ui->HRIConfigSpace->setDisabled(false);
	HRICS_MotionPLConfig  = new HRICS::ConfigSpace;
	HRICS_activeDist = HRICS_MotionPL->getDistance();
	
	ENV.setBool(Env::HRIPlannerCS,true);
	ENV.setBool(Env::enableHri,true);
	ENV.setBool(Env::isCostSpace,true);
	
	ENV.setBool(Env::useBallDist,false);
	ENV.setBool(Env::useBoxDist,true);
	
	m_ui->pushButtonNewHRIConfigSpace->setDisabled(true);
	m_ui->pushButtonDeleteHRICSpace->setDisabled(false);
	
	m_mainWindow->drawAllWinActive();
}

void HricsWidget::deleteHRIConfigSpace()
{
	ENV.setBool(Env::drawGrid,false);
	ENV.setBool(Env::HRIPlannerCS,false);
	
	delete HRICS_MotionPL;
	
	m_ui->pushButtonDeleteHRICSpace->setDisabled(true);
	m_ui->pushButtonNewHRIConfigSpace->setDisabled(false);
	m_ui->HRIConfigSpace->setDisabled(true);
	m_mainWindow->drawAllWinActive();
}

//-------------------------------------------------------------
// INIT (HRICS_CSpaceMPL)
//-------------------------------------------------------------
void HricsWidget::initConfigSpace()
{
	m_ui->HRIConfigSpace->setDisabled(true);
	
	connect(m_ui->pushButtonNewHRIConfigSpace,SIGNAL(clicked()),this,SLOT(newHRIConfigSpace()));
	connect(m_ui->pushButtonDeleteHRICSpace,SIGNAL(clicked()),this,SLOT(deleteHRIConfigSpace()));
	m_ui->pushButtonDeleteHRICSpace->setDisabled(true);
	
	connect(m_ui->pushButtonCreateGrid,SIGNAL(clicked()),this,SLOT(makeGridHRIConfigSpace()));
	connect(m_ui->pushButtonCreatePlan,SIGNAL(clicked()),this,SLOT(makePlanHRIConfigSpace()));
	connect(m_ui->pushButtonAStarIn2DGrid,SIGNAL(clicked()),this,SLOT(AStarInPlanHRIConfigSpace()));
	
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRecomputeCost, Env::RecomputeCellCost);
	
	QtShiva::SpinBoxSliderConnector* connectorColor1 = new QtShiva::SpinBoxSliderConnector(
																																												 this, m_ui->doubleSpinBoxColor1, m_ui->horizontalSliderColor1 , Env::colorThreshold1 );
	QtShiva::SpinBoxSliderConnector* connectorColor2 = new QtShiva::SpinBoxSliderConnector(
																																												 this, m_ui->doubleSpinBoxColor2, m_ui->horizontalSliderColor2 , Env::colorThreshold2 );
	
	connect(connectorColor1,SIGNAL(valueChanged(double)),m_mainWindow,SLOT(drawAllWinActive()),Qt::DirectConnection);
	connect(connectorColor2,SIGNAL(valueChanged(double)),m_mainWindow,SLOT(drawAllWinActive()),Qt::DirectConnection);
	
	connect(m_ui->pushButtonWriteToObPlane,SIGNAL(clicked()),this,SLOT(writeToOBPlane()));
	connect(m_ui->pushButtonHRIPlanRRT,SIGNAL(clicked()),this,SLOT(hriPlanRRT()));	
}

//-------------------------------------------------------------
// Plan Grid in HRICS_CSpaceMPL
//-------------------------------------------------------------
void HricsWidget::makeGridHRIConfigSpace()
{
	if(ENV.getBool(Env::HRIPlannerCS))
	{
		if(ENV.getInt(Env::hriCostType)==0)
		{
			cout << " Compute Distance Grid"  << endl;
			dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->computeDistanceGrid();
			API_activeGrid = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->getGrid();
			ENV.setBool(Env::drawGrid,true);
			m_mainWindow->drawAllWinActive();
		}
		if(ENV.getInt(Env::hriCostType)==1)
		{
			cout << " Compute Visibility Grid"  << endl;
			dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->computeVisibilityGrid();
			API_activeGrid = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->getGrid();
			ENV.setBool(Env::drawGrid,true);
			m_mainWindow->drawAllWinActive();
		}
		else
		{
			cout << "Warning : Only Distance or Visib in Env::hriCostType : No grid made" << endl;
		}
	}
}

void HricsWidget::makePlanHRIConfigSpace()
{
	if(ENV.getBool(Env::HRIPlannerCS))
	{
		API_activeGrid = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->getPlanGrid();

		ENV.setBool(Env::drawGrid,true);
		m_mainWindow->setBoolFloor(false);
		m_mainWindow->drawAllWinActive();
	}
}

void HricsWidget::AStarInPlanHRIConfigSpace()
{
	cout << "Computing 2D HRI A*" << endl;
	if(ENV.getBool(Env::HRIPlannerCS))
	{
		dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->computeAStarIn2DGrid();
		ENV.setBool(Env::drawTraj,true);
	}
}

void HricsWidget::writeToOBPlane()
{
	if( ENV.getBool(Env::HRIPlannerCS) && dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->getPlanGrid() )
	{
		dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->getPlanGrid()->writeToOBPlane();
	}
}

void HricsWidget::hriPlanRRT()
{
	dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->initHriRRT();
	m_mainWindow->drawAllWinActive();
}

//-----------------------
// INIT. Task Space
//-----------------------
void HricsWidget::initTaskSpace()
{
	m_ui->HRITaskSpace->setDisabled(true);
	
	
	connect(m_ui->pushButtonMake2DGrid,SIGNAL(clicked()),this,SLOT(make2DGrid()));
	
	connect(m_ui->pushButtonMakeGrid,SIGNAL(clicked()),this,SLOT(make3DHriGrid()));
	connect(m_ui->pushButtonDeleteGrid,SIGNAL(clicked()),this,SLOT(delete3DHriGrid()));
	m_ui->pushButtonDeleteGrid->setDisabled(true);
	
	connect(m_ui->pushButtonComputeCost,SIGNAL(clicked()),this,SLOT(computeGridCost()));
	connect(m_ui->pushButtonResetCost,SIGNAL(clicked()),this,SLOT(resetGridCost()));
	
	connect(m_ui->pushButtonAStarIn3DGrid,SIGNAL(clicked()),this,SLOT(AStarIn3DGrid()));
	connect(m_ui->pushButtonHRICSRRT,SIGNAL(clicked()),this,SLOT(HRICSRRT()));
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxCellSize, m_ui->horizontalSliderCellSize ,Env::CellSize );
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxPlanCellSize, m_ui->horizontalSliderPlanCellSize ,Env::PlanCellSize );
	
	m_ui->HRICSPlanner->setDisabled(true);
	
	connect(m_ui->pushButtonResetRandPoints,SIGNAL(clicked()),this,SLOT(resetRandomPoints()));
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
  
  // Protective bubble
  QtShiva::SpinBoxSliderConnector* connectorZoneSize  = new QtShiva::SpinBoxSliderConnector(
  this, m_ui->doubleSpinBoxZoneSize, m_ui->horizontalSliderZoneSize ,Env::zone_size );
  connect(connectorZoneSize,SIGNAL(valueChanged(double)),this,SLOT(zoneSizeChanged()),Qt::DirectConnection);
	
	/*if (PointsToDraw != NULL) 
	 {
	 delete PointsToDraw;
	 }*/
	
	PointsToDraw = new PointCloud;
	
	m_ui->HRICSPlanner->setDisabled(false);
	ENV.setBool(Env::HRIPlannerWS,true);
	//    ENV.setBool(Env::biDir,false);
	ENV.setDouble(Env::zone_size,0.5);
	HRICS_activeDist = HRICS_MotionPL->getDistance();
	
	API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid();
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
		
		m_ui->HRICSNatural->setDisabled(false);
		m_ui->pushButtonNewNaturalCostSpace->setDisabled(true);
		m_ui->activeGridsBox->addItem(dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid)->getName().c_str());
    api_store_new_grid( dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid) );
		HRICS_activeNatu->setGrid(dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid));
		ENV.setBool(Env::drawGrid,false);
	}
	
	ENV.setInt(Env::hriCostType,HRICS_Combine);
        m_mainWindow->drawAllWinActive();
	
        cout << "new HRI Workspace" << endl;

  Human->setAndUpdate( *q );
//        m_mainWindow->Ui()->tabCost->initCostFunctions();
	m_mainWindow->Ui()->tabCost->setCostFunction("costHRI");
  m_mainWindow->drawAllWinActive();
}

void HricsWidget::delete3DHriGrid()
{
	ENV.setBool(Env::drawGrid,false);
	ENV.setBool(Env::HRIPlannerWS,false);
	
	delete HRICS_MotionPL;
	HRICS_MotionPL = NULL;
	
	m_ui->HRICSPlanner->setDisabled(true);
	
	m_mainWindow->drawAllWinActive();
	
	m_ui->pushButtonMakeGrid->setDisabled(false);
	m_ui->pushButtonDeleteGrid->setDisabled(true);
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

void HricsWidget::computeGridCost()
{
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid()->computeAllCellCost();
	API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid();
}

void HricsWidget::resetGridCost()
{
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid()->resetCellCost();
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

//-------------------------------------------------------------
// Akin Hamp
//-------------------------------------------------------------
void HricsWidget::computeWorkspacePath()
{
	std::string str = "computeWorkspacePath";
	//write(qt_fl_pipe[1],str.c_str(),str.length()+1);
	cout << "HricsWidget : widget not implemented" << endl;
}

void HricsWidget::computeHoleMotion()
{
	std::string str = "computeHoleManipulationPath";
	//write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void HricsWidget::make2DGrid()
{
	vector<double>  envSize(4);
	envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
	envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
	
	ENV.setBool(Env::drawGrid,false);
	API::TwoDGrid* grid = new API::TwoDGrid(ENV.getDouble(Env::CellSize),envSize);
	grid->createAllCells();
	ENV.setBool(Env::drawGrid,true);
	API_activeGrid = grid;
}

void HricsWidget::KDistance(double value)
{
#ifdef HRI_PLANNER
	//    cout << "HRI_WEIGHTS[0] = " <<  ENV.getDouble(Env::Kdistance) << endl;
	HRI_WEIGHTS[0] = ENV.getDouble(Env::Kdistance);
#endif
}

void HricsWidget::KVisibility(double value)
{
#ifdef HRI_PLANNER
	//    cout << "HRI_WEIGHTS[1] = " <<  ENV.getDouble(Env::Kvisibility) << endl;
	HRI_WEIGHTS[1] = ENV.getDouble(Env::Kvisibility);
#endif
}

///////////////////////////////////////////////////////////////
void HricsWidget::enableHriSpace()
{
	//    if(hriSpace)
	//    {
	//        delete hriSpace;
	//    }
	//    hriSpace = new HriSpaceCost(XYZ_ROBOT,ENV.getInt(Env::akinJntId));
	
#ifdef HRI_PLANNER
  GLOBAL_AGENTS = hri_create_agents();
  char robotName[] = "JIDOKUKA";
  hri_assign_source_agent( robotName , GLOBAL_AGENTS);
	cout << "GLOBAL_AGENTS created!!!" << endl;
	
	ENV.setBool(Env::drawDistance,true);
#endif
	
#ifdef HRI_COSTSPACE
//	ENV.setBool(Env::isCostSpace,true);
//	ENV.setBool(Env::enableHri,true);
//	ENV.setBool(Env::HRIPlannerTS,true);
//	cout << "Env::enableHri is set to true, joint number is :"<< ENV.getInt(Env::akinJntId) << endl;
//	cout << "Robot is :" << XYZ_ROBOT->name << endl;
//	m_ui->HRITaskSpace->setDisabled(false);
#endif
}

void HricsWidget::drawAllWinActive()
{
	m_mainWindow->drawAllWinActive();	
}

void HricsWidget::on_pushButtonInitBaseGrid_clicked()
{
	HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();
	vector<double> box(6);

	box[0]  = -m_ui->horizontalSlider_xmin->value()/10.0; box[1]  = m_ui->horizontalSlider_xmax->value()/10.0;
	box[2]  = -m_ui->horizontalSlider_ymin->value()/10.0; box[3]  = m_ui->horizontalSlider_ymax->value()/10.0;
	box[4]  = -m_ui->horizontalSlider_zmin->value()/10.0; box[5]  = m_ui->horizontalSlider_zmax->value()/10.0;
	reachSpace->initHumanBaseGrid(box);
}

void HricsWidget::on_horizontalSlider_xmin_sliderMoved(int position)
{
	m_ui->label_xmin->setText(QString::number(-position/10.0) );

}

void HricsWidget::on_horizontalSlider_ymin_sliderMoved(int position)
{
	m_ui->label_ymin->setText(QString::number(-position/10.0) );
}

void HricsWidget::on_horizontalSlider_zmin_sliderMoved(int position)
{
	m_ui->label_zmin->setText(QString::number(-position/10.0) );
}

void HricsWidget::on_horizontalSlider_xmax_sliderMoved(int position)
{
	m_ui->label_xmax->setText(QString::number(position/10.0) );
}

void HricsWidget::on_horizontalSlider_ymax_sliderMoved(int position)
{
	m_ui->label_ymax->setText(QString::number(position/10.0) );
}

void HricsWidget::on_horizontalSlider_zmax_sliderMoved(int position)
{
	m_ui->label_zmax->setText(QString::number(position/10.0) );
}

void HricsWidget::on_pushButton_gridProperties_toggled(bool checked)
{
	if (checked)
	{
		m_ui->groupBox_gridPreperties->show();
		m_ui->pushButton_gridProperties->setText("Hide base grid properties");
	}
	else
	{
		m_ui->groupBox_gridPreperties->hide();
		m_ui->pushButton_gridProperties->setText("Show base grid properties");
	}
}




void HricsWidget::on_pushButton_NaturalParam_toggled(bool checked)
{
	if (checked)
	{
		m_ui->NaturalParameters->show();
		m_ui->pushButton_NaturalParam->setText("Hide Natural Parameters");
	}
	else
	{
		m_ui->NaturalParameters->hide();
		m_ui->pushButton_NaturalParam->setText("Show Natural Parameters");
	}
}

void HricsWidget::on_pushButton_initColPos_clicked()
{
    HRICS_activeNatu->setRobotToConfortPosture();
    HRICS_activeNatu->setRobotColorFromConfiguration(false);
    m_mainWindow->drawAllWinActive();
}
