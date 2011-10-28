//
//  qtNatural.cpp
//  move3d-studio
//
//  Created by Jim Mainprice on 21/09/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "qtNatural.hpp"
#include "ui_qtNatural.h"

#include <iostream>
#include <tr1/memory>
#include <boost/bind.hpp>
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

using namespace std;
using namespace tr1;

extern Eigen::Vector3d global_DrawnSphere;

NaturalWidget::NaturalWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::NaturalWidget)
{
	m_ui->setupUi(this);
	//this->initHRI();
	//this->initHumanLike();
}

NaturalWidget::~NaturalWidget()
{
	delete m_ui;
	//#ifdef USE_QWT
	//    delete this->plot;
	//#endif
}

void NaturalWidget::initNatural()
{
//  m_mainWindow->connectCheckBoxToEnv(m_ui->enableHriTS,										Env::HRIPlannerTS);
//  m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRIGoalBiased,					Env::isGoalBiased);
//  m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxInverseKinematics,			Env::isInverseKinematics);
//  m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxComputeOTP,						Env::HRIComputeOTP);
  m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriLeftArmVsRightArm,	Env::HRIleftArmVsRightArm);
//  m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxEnableHRIConfigSpace,  Env::HRIPlannerCS);
  
//  connect(m_ui->pushButtonHRITS,SIGNAL(clicked()),this,SLOT(enableHriSpace()));
  
  m_ui->groupBox_gridPreperties->hide();
  m_ui->NaturalParameters->hide();
  
  
  // -----------------------------------------------------------------
  // Active Joint
  // -----------------------------------------------------------------
//  static_ui->activeGridsBox->addItem( grid->getName().c_str() );
//  
//  connect(m_ui->spinBoxJoint, SIGNAL(valueChanged(int)),ENV.getObject(Env::akinJntId), SLOT(set(int)), Qt::DirectConnection);
//  connect(ENV.getObject(Env::akinJntId), SIGNAL(valueChanged(int)),m_ui->spinBoxJoint, SLOT(setValue(int)), Qt::DirectConnection);
//  m_ui->spinBoxJoint->setValue(ENV.getInt(Env::akinJntId));
//  
//  connect(m_ui->pushButtonWorkspacePath, SIGNAL(clicked()),this, SLOT(computeWorkspacePath()), Qt::DirectConnection);
//  connect(m_ui->pushButtonHoleMotion, SIGNAL(clicked()),this, SLOT(computeHoleMotion()), Qt::DirectConnection);
  
  connect(m_ui->pushButtonSaveActiveGridToFile,SIGNAL(clicked()),this,SLOT(saveActiveGridToFile()));
  connect(m_ui->pushButtonLoadActiveGridFromFile,SIGNAL(clicked()),this,SLOT(loadActiveGridFromFile()));
  
  connect(m_ui->activeGridsBox, SIGNAL(currentIndexChanged(int)),this, SLOT(setActiveGrid(int)), Qt::DirectConnection);
  
  connect(m_ui->pushButtonMergeGrids, SIGNAL(clicked()), this, SLOT(mergeGrids()));
  connect(m_ui->spinBoxCellToShow, SIGNAL(valueChanged(int)),ENV.getObject(Env::cellToShow),SLOT(set(int)), Qt::DirectConnection);
  connect(m_ui->spinBoxCellToShow, SIGNAL(valueChanged(int)),this,SLOT(cellToShowChanged()), Qt::QueuedConnection);  
  
//  connect(m_ui->pushButtonWriteToObPlane,SIGNAL(clicked()),this,SLOT(writeToOBPlane()));
//  connect(m_ui->pushButtonHRIPlanRRT,SIGNAL(clicked()),this,SLOT(hriPlanRRT()));
//  
//  connect(m_ui->pushButtonResetRandPoints,SIGNAL(clicked()),this,SLOT(resetRandomPoints()));
  
  m_ui->HRICSNatural->setDisabled(false);
  m_ui->pushButtonNewNaturalCostSpace->setDisabled(true);
  m_ui->activeGridsBox->addItem(dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid)->getName().c_str());
  
//  m_ui->HRIConfigSpace->setDisabled(true);
//  
//  connect(m_ui->pushButtonNewHRIConfigSpace,SIGNAL(clicked()),this,SLOT(newHRIConfigSpace()));
//  connect(m_ui->pushButtonDeleteHRICSpace,SIGNAL(clicked()),this,SLOT(deleteHRIConfigSpace()));
//  m_ui->pushButtonDeleteHRICSpace->setDisabled(true);
//  
//  connect(m_ui->pushButtonCreateGrid,SIGNAL(clicked()),this,SLOT(makeGridHRIConfigSpace()));
//  connect(m_ui->pushButtonCreatePlan,SIGNAL(clicked()),this,SLOT(makePlanHRIConfigSpace()));
//  connect(m_ui->pushButtonAStarIn2DGrid,SIGNAL(clicked()),this,SLOT(AStarInPlanHRIConfigSpace()));
//  
//  m_ui->pushButtonNewHRIConfigSpace->setDisabled(true);
//  m_ui->pushButtonDeleteHRICSpace->setDisabled(false);
//  
//  m_ui->pushButtonDeleteHRICSpace->setDisabled(true);
//  m_ui->pushButtonNewHRIConfigSpace->setDisabled(false);
//  m_ui->HRIConfigSpace->setDisabled(true);
//  
//  m_ui->HRIConfigSpace->setDisabled(false);
//  
	HRICS_MotionPLConfig  = new HRICS::ConfigSpace;
  
  m_ui->HRICSNatural->setDisabled(true);
  m_ui->pushButtonNewNaturalCostSpace->setDisabled(false);
}

void NaturalWidget::cellToShowChanged()
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

//-------------------------------------------------------------
// INIT (HRICS_Natural)
//-------------------------------------------------------------
void NaturalWidget::initNaturalSpace()
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

void NaturalWidget::newNaturalCostSpace()
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

void NaturalWidget::deleteNaturalCostSpace()
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

void NaturalWidget::recomputeReachability()
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

void NaturalWidget::computeAllCellNaturalCost()
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

void NaturalWidget::computeReachability()
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

void NaturalWidget::getSortedReachableWSPoint()
{
	HRICS_activeNatu->getSortedReachableWSPoint();
}

//-------------------------------------------------------------
// Active Grids
//-------------------------------------------------------------
Ui::NaturalWidget* static_ui;

void add_grid_to_ui( API::BaseGrid* grid)
{
  static_ui->activeGridsBox->addItem( grid->getName().c_str() );
}

void NaturalWidget::initGrids()
{
	connect(m_ui->activeGridsBox, SIGNAL(currentIndexChanged(int)),this, SLOT(setActiveGrid(int)), Qt::DirectConnection);
	connect(ENV.getObject(Env::hriActiveGrid), SIGNAL(valueChanged(int)),this, SLOT(setActiveGrid(int)), Qt::DirectConnection);
//	m_ui->whichTestBox->setCurrentIndex(ENV.getInt(Env::hriActiveGrid));
	
	connect(m_ui->pushButtonMergeGrids, SIGNAL(clicked()), this, SLOT(mergeGrids()));
	connect(m_ui->spinBoxCellToShow, SIGNAL(valueChanged(int)),ENV.getObject(Env::cellToShow),SLOT(set(int)), Qt::DirectConnection);
	connect(m_ui->spinBoxCellToShow, SIGNAL(valueChanged(int)),this,SLOT(cellToShowChanged()), Qt::QueuedConnection);  
  
  static_ui = m_ui;
  ext_add_grid_to_ui = add_grid_to_ui;
}

void NaturalWidget::setActiveGrid(int ith_grid)
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

void NaturalWidget::mergeGrids()
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

void NaturalWidget::saveActiveGridToFile()
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

void NaturalWidget::loadActiveGridFromFile()
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
    //		m_ui->HRICSNatural->setDisabled(false);
    //		m_ui->pushButtonNewNaturalCostSpace->setDisabled(true);
		
		// Reads the grid from XML and sets it ti the HRICS_MotionPL
		qt_load_HRICS_Grid(fileName.toStdString());
		
    //		m_ui->activeGridsBox->addItem( API_activeGrid->getName().c_str() );
		api_store_new_grid( dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid) );
		HRICS_activeNatu->setGrid( dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid) );
		ENV.setBool(Env::drawGrid,true);
		m_mainWindow->drawAllWinActive();
	}
}


void NaturalWidget::on_pushButtonInitBaseGrid_clicked()
{
	HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();
	vector<double> box(6);
  
	box[0]  = -m_ui->horizontalSlider_xmin->value()/10.0; box[1]  = m_ui->horizontalSlider_xmax->value()/10.0;
	box[2]  = -m_ui->horizontalSlider_ymin->value()/10.0; box[3]  = m_ui->horizontalSlider_ymax->value()/10.0;
	box[4]  = -m_ui->horizontalSlider_zmin->value()/10.0; box[5]  = m_ui->horizontalSlider_zmax->value()/10.0;
	reachSpace->initHumanBaseGrid(box);
}

void NaturalWidget::on_horizontalSlider_xmin_sliderMoved(int position)
{
	m_ui->label_xmin->setText(QString::number(-position/10.0) );
  
}

void NaturalWidget::on_horizontalSlider_ymin_sliderMoved(int position)
{
	m_ui->label_ymin->setText(QString::number(-position/10.0) );
}

void NaturalWidget::on_horizontalSlider_zmin_sliderMoved(int position)
{
	m_ui->label_zmin->setText(QString::number(-position/10.0) );
}

void NaturalWidget::on_horizontalSlider_xmax_sliderMoved(int position)
{
	m_ui->label_xmax->setText(QString::number(position/10.0) );
}

void NaturalWidget::on_horizontalSlider_ymax_sliderMoved(int position)
{
	m_ui->label_ymax->setText(QString::number(position/10.0) );
}

void NaturalWidget::on_horizontalSlider_zmax_sliderMoved(int position)
{
	m_ui->label_zmax->setText(QString::number(position/10.0) );
}

void NaturalWidget::on_pushButton_gridProperties_toggled(bool checked)
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

void NaturalWidget::on_pushButton_NaturalParam_toggled(bool checked)
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

