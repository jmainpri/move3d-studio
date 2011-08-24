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
#include "planner/planEnvironment.hpp"
#include "planner/replanning.hpp"

using namespace std;
using namespace tr1;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

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
  
  connect(m_ui->pushButtonSetLocalpath, SIGNAL(clicked()), this, SLOT(setLocalpath()));
  
  connect(m_ui->pushButtonInitialize, SIGNAL(clicked()), this, SLOT(initReplanning()));
  connect(m_ui->pushButtonExecuteSimu, SIGNAL(clicked()), this, SLOT(executeReplanTraj()));
  
  // Draw the traj
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDrawTraj, Env::drawTraj );
  
  // Use the current trajectory
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxOptimizeCurrentTraj,    PlanParam::withCurrentTraj );
  
  // Do the replanning sequences
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDoReplanning,    PlanParam::doReplanning );
  
  // Set the number of point to be optimized
  connect(m_ui->spinBoxNbPoints,SIGNAL(valueChanged(int)),PlanEnv->getObject(PlanParam::nb_pointsOnTraj),SLOT(set(int)));
	m_ui->spinBoxNbPoints->setValue(PlanEnv->getInt(PlanParam::nb_pointsOnTraj));
  
  // Set the duration of the optimized trajectory
  connect(m_ui->doubleSpinBoxDuration,SIGNAL(valueChanged(double)),PlanEnv->getObject(PlanParam::trajDuration),SLOT(set(double)));
	m_ui->doubleSpinBoxDuration->setValue(PlanEnv->getDouble(PlanParam::trajDuration));
}

//---------------------------------------------------------
// Trajectory Optimization
//---------------------------------------------------------
void ReplanningWidget::setLocalpath()
{
  traj_optim_set_localpath_and_cntrts();
}

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


