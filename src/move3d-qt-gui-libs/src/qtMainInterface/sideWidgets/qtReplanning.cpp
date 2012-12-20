/*
 *  qtReplanning.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtReplanning.hpp"
#include "ui_qtReplanning.h"

#include "planner_handler.hpp"

#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/policy_improvement_loop.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/replanningAlgorithms.hpp"
#include "planner/replanningSimulators.hpp"

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#if defined(USE_QWT)
#include "qtPlot/basicPlot.hpp"
#include "qtPlot/multiPlot.hpp"
#include "qtPlot/replottingVectors.hpp"
#endif

using namespace std;
using namespace tr1;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;
using namespace QtShiva;

ReplanningWidget::ReplanningWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::ReplanningWidget)
{
	m_ui->setupUi(this);
	
	init();
}

ReplanningWidget::~ReplanningWidget()
{
	delete m_ui;
}

void ReplanningWidget::init()
{
  // This function enables multi-threading
  connect(this, SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));
  
  connect(m_ui->pushButtonRunStomp, SIGNAL(clicked()), this, SLOT(runStomp()));
  connect(m_ui->pushButtonRunChomp, SIGNAL(clicked()), this, SLOT(runChomp()));
  connect(m_ui->pushButtonRunNoReset, SIGNAL(clicked()), this, SLOT(runNoReset()));
  
  connect(m_ui->pushButtonExecuteSimpleSim, SIGNAL(clicked()), this, SLOT(executeSimpleSimu()));
  
  // Enable multi-thread graphical mode
  connect(m_ui->checkBoxMultiThreadGraphical, SIGNAL(toggled(bool)), this, SLOT(multiThreadGraphicalMode(bool)));
  
  // Active Joints
  connect(m_ui->radioButtonNavigation,    SIGNAL(toggled(bool)), this, SLOT(setActiveJoints()));
	connect(m_ui->radioButtonManipulation,  SIGNAL(toggled(bool)), this, SLOT(setActiveJoints()));
	connect(m_ui->radioButtonMobileManip,   SIGNAL(toggled(bool)), this, SLOT(setActiveJoints()));
  connect( ENV.getObject(Env::setOfActiveJoints), SIGNAL(valueChanged(int)), this, SLOT(setActiveJointsRadioButtons(int)), Qt::DirectConnection );
  
  // Init Method
  connect(m_ui->radioButtonRRT,             SIGNAL(toggled(bool)), this, SLOT(setInitMethod()));
	connect(m_ui->radioButtonStraightLine,    SIGNAL(toggled(bool)), this, SLOT(setInitMethod()));
  connect( PlanEnv->getObject(PlanParam::replanningInitMethod), SIGNAL(valueChanged(int)), this, SLOT(setInitMethodRadioButtons(int)), Qt::DirectConnection );
  
  // replanner Type
  connect(m_ui->comboBoxReplanner, SIGNAL(currentIndexChanged(int)), PlanEnv->getObject(PlanParam::replanningAlgorithm),SLOT(set(int)));
	connect( PlanEnv->getObject(PlanParam::replanningAlgorithm), SIGNAL(valueChanged(int)) ,m_ui->comboBoxReplanner, SLOT(setCurrentIndex(int)));
	m_ui->comboBoxReplanner->setCurrentIndex( 0 /*Simple*/ );
	// 0 => Simple
	// 1 => SoftMotion
	// 2 => RRT
  
  // Plot Noisy trajectories
#ifdef USE_QWT
  connect(m_ui->pushButtonPlotNoise, SIGNAL(clicked()), this, SLOT(plotNoisyTrajectories()));
  connect(m_ui->pushButtonPlotTraj, SIGNAL(clicked()), this, SLOT(plotMultiVectors()));
  connect(m_ui->pushButtonPlotConvergence, SIGNAL(clicked()), this, SLOT(plotConvergence()));
  m_plot = new BasicPlotWindow();
#endif
  
  connect(m_ui->pushButtonComputeSM, SIGNAL(clicked()), this, SLOT(computeSoftMotion()));
  connect(m_ui->pushButtonFixJoints, SIGNAL(clicked()), this, SLOT(setMlpCntrtsAndFixJoints()));
  
  // Smooth And Obstacle Weigth
  SpinBoxConnector(this,m_ui->doubleSpinBoxSmoothWeight,PlanParam::trajOptimSmoothWeight);
  SpinBoxConnector(this,m_ui->doubleSpinBoxObstacWeight,PlanParam::trajOptimObstacWeight);
  
  //---------------------------------------
  // Test the multi gaussian
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxActiveJointsSetAtStart, Env::setActiveJointsGroup );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxInitStomp,              Env::setStompPlanner );
  
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxTestMultiGauss, PlanParam::trajOptimTestMultiGauss );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDrawTraj, Env::drawTraj );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxOptimizeCurrentTraj, PlanParam::withCurrentTraj );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDoReplanning, PlanParam::doReplanning );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxMoveHuman, PlanParam::trajMoveHuman );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxRecomputeOtp, PlanParam::trajUseOtp );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxSelectedDuration, PlanParam::useSelectedDuration );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxTimeLimit, PlanParam::trajStompWithTimeLimit );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxMMatrix, PlanParam::trajStompMultiplyM );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxWithRRT, PlanParam::trajStompWithRRT );
  
  // Stomp draw iteration
  SpinBoxConnector(this,m_ui->spinBoxStompDrawIteration,PlanParam::stompDrawIteration);
  
  // Time Limit
  SpinBoxConnector(this,m_ui->doubleSpinBoxTimeLimit,PlanParam::trajStompTimeLimit);
  
  // Set the number of point to be optimized
  SpinBoxConnector(this,m_ui->spinBoxNbPoints,PlanParam::nb_pointsOnTraj);
  
  // Set the duration of the optimized trajectory
  SpinBoxConnector(this,m_ui->doubleSpinBoxDuration,PlanParam::trajDuration);
  
  // Set the standard deviation of the perturbations
  SpinBoxConnector(this,m_ui->doubleSpinBoxStdDev,PlanParam::trajOptimStdDev);
  
  // The replanning window 
  SpinBoxSliderConnector( this, m_ui->doubleSpinBoxReplanningWindow, m_ui->horizontalSliderReplanningWindow , PlanParam::trajReplanningWindow );
  SpinBoxSliderConnector( this, m_ui->doubleSpinBoxTotalTrajDuration, m_ui->horizontalSliderTotalTrajDuration , PlanParam::trajReplanningTotalTime );
}

//---------------------------------------------------------
// Trajectory Optimization
//---------------------------------------------------------
void ReplanningWidget::computeHandOver()
{
  emit(selectedPlanner(QString("computeHandover")));
}

void ReplanningWidget::runStomp()
{
  emit(selectedPlanner(QString("runStomp")));
}

void ReplanningWidget::runChomp()
{
  emit(selectedPlanner(QString("runChomp")));
}

void ReplanningWidget::runNoReset()
{
  emit(selectedPlanner(QString("runNoReset")));
}

void ReplanningWidget::plotNoisyTrajectories()
{  
#if defined(USE_QWT)
	ReplottingVectors* myPlot = new ReplottingVectors(m_plot);
	myPlot->setGeometry(m_plot->getPlot()->geometry());
  
//  vector<double> cost;
  
//  for( int i=0;i<int(global_noiseTrajectory.size()); i++)
//	{
//    cout << "global_noiseTrajectory[" << i << "] = " << global_noiseTrajectory[i] << endl;
//  }
  
  vector< const vector<double>* > toPlot;
  toPlot.push_back( &global_noiseTrajectory1 );
	toPlot.push_back( &global_noiseTrajectory2 );
  
  myPlot->addData( toPlot );
  
	delete m_plot->getPlot();
	m_plot->setPlot(myPlot);
	m_plot->show();
#endif
}

void ReplanningWidget::plotMultiVectors()
{
#if defined(USE_QWT)
  MultiPlot* myPlot = new MultiPlot(m_plot);
	myPlot->setGeometry(m_plot->getPlot()->geometry());
	
	vector< string > plotNames;
	plotNames.push_back( "X SoftMotion" );
	plotNames.push_back( "Y SoftMotion" );
	plotNames.push_back( "X Initial" );
  plotNames.push_back( "Y Initial" );
	
	myPlot->setData( plotNames , traj_optim_to_plot );
  myPlot->setTitle("Initial and smoothed trajectory");
  
	delete m_plot->getPlot();
	m_plot->setPlot(myPlot);
	m_plot->show();
#endif
}

// static bool test_plot = true;
void ReplanningWidget::plotConvergence()
{
#if defined(USE_QWT)
  if (traj_optim_convergence.size() != 0 && traj_optim_convergence[0].size() != 0) 
  {
    MultiPlot* myPlot = new MultiPlot(m_plot);
    myPlot->setGeometry(m_plot->getPlot()->geometry());
    
    vector<string> plotNames;
    plotNames.push_back( "Costs" );
    
//    vector< vector<double> > to_plot = traj_optim_convergence;
//    
//    if ( test_plot )
//    {
//      to_plot[0].resize(100);
//      test_plot = false;
//    }
//    else {
//      test_plot = true;
//    }
    
    myPlot->setData( plotNames , traj_optim_convergence );
    myPlot->setTitle("Convergence of the traj optim");
    myPlot->setAxisScale( QwtPlot::yLeft, 0, 8000 );
    
    delete m_plot->getPlot();
    m_plot->setPlot(myPlot);
    m_plot->show();
  }
#endif
}

void ReplanningWidget::computeSoftMotion()
{
  emit(selectedPlanner(QString("convertToSoftMotion")));
  return; 
}
//---------------------------------------------------------
// Replanning
//---------------------------------------------------------
void ReplanningWidget::setActiveJoints()
{
  if( m_ui->radioButtonNavigation->isChecked() )
  {
    cout << "radioButtonNavigation->isChecked()" << endl;
    ENV.setInt(Env::setOfActiveJoints,0);
  }
	if ( m_ui->radioButtonManipulation->isChecked() )
  {
    cout << "radioButtonManipulation->isChecked()" << endl;
    ENV.setInt(Env::setOfActiveJoints,1);
  }
	if ( m_ui->radioButtonMobileManip->isChecked() )
  {
    cout << "radioButtonMobileManip->isChecked()" << endl;
    ENV.setInt(Env::setOfActiveJoints,2);
  }
}

void ReplanningWidget::setActiveJointsRadioButtons(int type)
{
  if( type == 0 && !m_ui->radioButtonNavigation->isChecked() )
  {
    cout << "radioButtonNavigation->toggle()" << endl;
    m_ui->radioButtonNavigation->toggle();
  }
	if( type == 1 && !m_ui->radioButtonManipulation->isChecked())
  {
    cout << "radioButtonManipulation->toggle()" << endl;
    m_ui->radioButtonManipulation->toggle();
  }
	if( type == 2 && !m_ui->radioButtonMobileManip->isChecked())
  {
    cout << "radioButtonMobileManip->toggle()" << endl;
    m_ui->radioButtonMobileManip->toggle();
  }
}

void ReplanningWidget::setInitMethod()
{
  if( m_ui->radioButtonStraightLine->isChecked() )
  {
    PlanEnv->setInt(PlanParam::replanningInitMethod,0);
    cout << "radioButtonStraightLine->isChecked()" << endl;
  }
	if ( m_ui->radioButtonRRT->isChecked() )
  {
    PlanEnv->setInt(PlanParam::replanningInitMethod,1);
    cout << "radioButtonRRT->isChecked()" << endl;
  }
}

void ReplanningWidget::setInitMethodRadioButtons(int type)
{
  if( type == 0 && !m_ui->radioButtonStraightLine->isChecked() )
  {
    cout << "radioButtonNavigation->toggle()" << endl;
    m_ui->radioButtonStraightLine->toggle();
  }
	if( type == 1 && !m_ui->radioButtonRRT->isChecked())
  {
    cout << "radioButtonManipulation->toggle()" << endl;
    m_ui->radioButtonRRT->toggle();
  }
}

void ReplanningWidget::multiThreadGraphicalMode(bool enable)
{
  global_rePlanningEnv->set_multithread_graphical( enable );
}

void ReplanningWidget::setMlpCntrtsAndFixJoints()
{
  emit(selectedPlanner(QString("initMlpCntrtsAndFixJoints")));
  return;
}

void ReplanningWidget::executeSimpleSimu()
{
  emit(selectedPlanner(QString("ExecuteSimpleSimu")));
}

