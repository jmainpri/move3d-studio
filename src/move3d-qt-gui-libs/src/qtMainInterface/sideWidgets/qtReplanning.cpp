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
#include "planner/replanning.hpp"

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#if defined(USE_QWT)
#include "qtPlot/basicPlot.hpp"
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
  connect(m_ui->pushButtonReplan, SIGNAL(clicked()), this, SLOT(mainReplanFunction()));
  
  connect(m_ui->pushButtonRunStomp, SIGNAL(clicked()), this, SLOT(runStomp()));
  connect(m_ui->pushButtonRunChomp, SIGNAL(clicked()), this, SLOT(runChomp()));
  
  connect(m_ui->pushButtonInitialize, SIGNAL(clicked()), this, SLOT(initReplanning()));
  connect(m_ui->pushButtonExecuteSimu, SIGNAL(clicked()), this, SLOT(executeReplanTraj()));

  
#ifdef USE_QWT
  connect(m_ui->pushButtonPlotNoise, SIGNAL(clicked()), this, SLOT(plotNoisyTrajectories()));
  m_plot = new BasicPlotWindow();
#endif
  
  connect(m_ui->pushButtonComputeSM, SIGNAL(clicked()), this, SLOT(computeSoftMotion()));
  
  // Smooth And Obstacle Weigth
  SpinBoxConnector(this,m_ui->doubleSpinBoxSmoothWeight,PlanParam::trajOptimSmoothWeight);
  SpinBoxConnector(this,m_ui->doubleSpinBoxObstacWeight,PlanParam::trajOptimObstacWeight);
  
  //---------------------------------------
  // Test the multi gaussian
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxTestMultiGauss, 
                                     PlanParam::trajOptimTestMultiGauss );
  
  // Draw the traj
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDrawTraj, 
                                      Env::drawTraj );
  
  // Use the current trajectory
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxOptimizeCurrentTraj, 
                                      PlanParam::withCurrentTraj );
  
  // Do the replanning sequences
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDoReplanning,
                                      PlanParam::doReplanning );
  
  // Use the selected duration
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxSelectedDuration,   
                                      PlanParam::useSelectedDuration );
  
  // Set the number of point to be optimized
  SpinBoxConnector(this,m_ui->spinBoxNbPoints,PlanParam::nb_pointsOnTraj);
  
  // Set the duration of the optimized trajectory
  SpinBoxConnector(this,m_ui->doubleSpinBoxDuration,PlanParam::trajDuration);
  
  // Set the standard deviation of the perturbations
  SpinBoxConnector(this,m_ui->doubleSpinBoxStdDev,PlanParam::trajOptimStdDev);
}

//---------------------------------------------------------
// Trajectory Optimization
//---------------------------------------------------------
void ReplanningWidget::computeHandOver()
{
  emit(selectedPlanner(QString("computeHandover")));
  return;
}

void ReplanningWidget::runStomp()
{
  emit(selectedPlanner(QString("runStomp")));
}

void ReplanningWidget::runChomp()
{
  emit(selectedPlanner(QString("runChomp")));
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

void ReplanningWidget::computeSoftMotion()
{
  emit(selectedPlanner(QString("convertToSoftMotion")));
  return; 
}
//---------------------------------------------------------
// Replanning
//---------------------------------------------------------
void ReplanningWidget::initReplanning()
{
  replann_initialize();
}

void ReplanningWidget::mainReplanFunction()
{
  emit(selectedPlanner(QString("Replanning")));
  return; 
}

void ReplanningWidget::executeReplanTraj()
{
  emit(selectedPlanner(QString("ExecuteReplanTraj")));
}


