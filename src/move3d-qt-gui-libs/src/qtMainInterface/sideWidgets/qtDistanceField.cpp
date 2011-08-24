/*
 *  qtDistanceField.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtDistanceField.hpp"
#include "ui_qtDistanceField.h"

#include "planner/Greedy/CollisionSpace.hpp"

#include "API/project.hpp"

#include "planner_handler.hpp"

using namespace std;
using namespace tr1;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

extern string global_ActiveRobotName;

DistFieldWidget::DistFieldWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::DistFieldWidget)
{
	m_ui->setupUi(this);
	
	initDistField();
}

DistFieldWidget::~DistFieldWidget()
{
	delete m_ui;
}

void DistFieldWidget::initDistField()
{
  // This function enables multi-threading
//  connect(this, SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));
  connect(m_ui->pushButtonCreateDistanceField, SIGNAL(clicked()), this, SLOT(createDistanceField()));
  connect(m_ui->pushButtonDeleteDistanceField, SIGNAL(clicked()), this, SLOT(deleteDistanceField()));
  connect(m_ui->pushButtonAddAllPoints,        SIGNAL(clicked()), this, SLOT(addAllPointsToField()));
  connect(m_ui->pushButtonGenerateRobotBoundingVolumes, SIGNAL(clicked()), this, SLOT(generateRobotBoundingVolumes()));
  
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDrawOccupVoxels,    PlanParam::drawOccupVoxels );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDrawSampledPoints,  PlanParam::drawSampledPoints );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDrawStaticCells,    PlanParam::drawStaticVoxels );
  m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDrawBoundingVolumes,PlanParam::drawBoundingVolumes );
  
  // Set the number of cells in the grid
  connect(m_ui->spinBoxNbMaxCells,SIGNAL(valueChanged(int)),ENV.getObject(Env::nbCells),SLOT(set(int)));
	m_ui->spinBoxNbMaxCells->setValue(ENV.getInt(Env::nbCells));
  
  // Set the distance at which to draw the voxels
  connect(m_ui->doubleSpinBoxDrawingDistance,SIGNAL(valueChanged(double)),PlanEnv->getObject(PlanParam::distMinToDraw),SLOT(set(double)));
	m_ui->doubleSpinBoxDrawingDistance->setValue(PlanEnv->getDouble(PlanParam::distMinToDraw));
}

void DistFieldWidget::createDistanceField()
{
  if(global_CollisionSpace)
    delete global_CollisionSpace;
  
  Robot* rob = global_Project->getActiveScene()->getActiveRobot();
  
  global_CollisionSpace = new CollisionSpace(rob);
}

void DistFieldWidget::addAllPointsToField()
{
  if (global_CollisionSpace) 
  {
    // Add all the static (moveable objects also)
    global_CollisionSpace->addAllPointsToField();
    
    // Add all the humans
    Scene* sc = global_Project->getActiveScene();
    
    for (unsigned int i=0; i<sc->getNumberOfRobots(); i++) 
    {
      Robot* rob = sc->getRobot(i);
      
      if ( rob->getName().find("HERAKLES") != string::npos )
      {
        global_CollisionSpace->addRobot( rob );
      }
    }
  }
}

void DistFieldWidget::deleteDistanceField()
{
  if (global_CollisionSpace!= NULL) 
  {
    delete global_CollisionSpace;
    global_CollisionSpace = NULL;
    cout << "Distance field Deleted" << endl;
  }
  else 
  {
    cout << "Distance field doesn't exist!!!" << endl;
  }
}

void DistFieldWidget::generateRobotBoundingVolumes()
{
  if (global_CollisionSpace == NULL) 
  {
    cout << "Distance field doesn't exist!!!" << endl;
    return;
  }
  
  // Set the active joints (links)
  std::vector<int> active_joints;
  active_joints.clear();
  active_joints.push_back( 6 );
  active_joints.push_back( 7 );
  active_joints.push_back( 8 );
  active_joints.push_back( 9 );
  active_joints.push_back( 10 );
  active_joints.push_back( 11 );
  active_joints.push_back( 12 );
  
  active_joints.push_back( 14 );
  active_joints.push_back( 15 );
  
  // Set the planner joints
  std::vector<int> planner_joints;
  planner_joints.clear();
  planner_joints.push_back( 6 );
  planner_joints.push_back( 7 );
  planner_joints.push_back( 8 );
  planner_joints.push_back( 9 );
  planner_joints.push_back( 10 );
  planner_joints.push_back( 11 );
  planner_joints.push_back( 12 );
  
  // Generate Bounding volumes for active joints
  BodySurfaceSampler* sampler = global_CollisionSpace->getBodySampler();
  
  // Generate bounding volumes for all joints active in the motion planning
  Robot* rob = global_CollisionSpace->getRobot();
  
  // Generate cylinders
  vector<Joint*> joints;
  joints.clear();
  for (unsigned int i=0; i<active_joints.size(); i++) 
  {
    joints.push_back( rob->getJoint( active_joints[i] ) );
  }
  
  sampler->generateRobotBoudingCylinder( rob, joints );
  
  cout << "Bounding volumes generated for robot " << rob->getName() << endl;
  
  // Generate collision points
  vector<int> planner_joints_id;
  for (unsigned int i=0; i<planner_joints.size(); i++) 
  {
    planner_joints_id.push_back( planner_joints[i] );
  }
  
  sampler->generateRobotCollisionPoints( rob, active_joints, planner_joints_id );
  
  cout << "Collision points generated for robot " << rob->getName() << endl;
}

//void DistFieldWidget::mainReplanFunction()
//{
//  emit(selectedPlanner(QString("Replanning")));
//  return; 
//}

//void DistFieldWidget::computeHandOver()
//{
//  emit(selectedPlanner(QString("computeHandover")));
//  return;
//}
