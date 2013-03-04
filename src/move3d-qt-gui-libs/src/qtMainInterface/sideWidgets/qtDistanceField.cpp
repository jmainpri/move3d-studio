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

#include "planner_handler.hpp"

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "collision_space/CollisionSpace.hpp"

#include "API/project.hpp"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;
using namespace QtShiva;

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
    
    new connectCheckBoxToEnv( m_ui->checkBoxDrawOccupVoxels,     PlanEnv->getObject(PlanParam::drawOccupVoxels ));
    new connectCheckBoxToEnv( m_ui->checkBoxDrawSampledPoints,   PlanEnv->getObject(PlanParam::drawSampledPoints ));
    new connectCheckBoxToEnv( m_ui->checkBoxDrawStaticCells,     PlanEnv->getObject(PlanParam::drawStaticVoxels ));
    new connectCheckBoxToEnv( m_ui->checkBoxDrawBoundingVolumes, PlanEnv->getObject(PlanParam::drawBoundingVolumes ));
    
    // Set the number of cells in the grid
    SpinBoxConnector(this,m_ui->spinBoxNbMaxCells, ENV.getObject(Env::nbCells));
    //  connect(m_ui->spinBoxNbMaxCells,SIGNAL(valueChanged(int)),ENV.getObject(Env::nbCells),SLOT(set(int)));
    //	m_ui->spinBoxNbMaxCells->setValue(ENV.getInt(Env::nbCells));
    
    // Set the distance at which to draw the voxels
    SpinBoxConnector(this,m_ui->doubleSpinBoxDrawingDistance, PlanEnv->getObject(PlanParam::distMinToDraw));
    //  connect(m_ui->doubleSpinBoxDrawingDistance,SIGNAL(valueChanged(double)),PlanEnv->getObject(PlanParam::distMinToDraw),SLOT(set(double)));
    //	m_ui->doubleSpinBoxDrawingDistance->setValue(PlanEnv->getDouble(PlanParam::distMinToDraw));
}

void DistFieldWidget::createDistanceField()
{
    if(global_collisionSpace)
        delete global_collisionSpace;
    
    Scene* sce = global_Project->getActiveScene();
    
    global_collisionSpace = new CollisionSpace( sce->getActiveRobot(), double(ENV.getInt(Env::nbCells))/100, sce->getBounds() );
    
    cout << "You can now add static to the distance field!!!" << endl;
}

void DistFieldWidget::addAllPointsToField()
{
    if (global_collisionSpace) 
    {
        // Add all the static (moveable objects also)
        global_collisionSpace->addAllPointsToField();
        
        // Add all the humans
        Scene* sc = global_Project->getActiveScene();
        
        for (unsigned int i=0; i<sc->getNumberOfRobots(); i++) 
        {
            Robot* rob = sc->getRobot(i);
            
            if ( rob->getName().find("HERAKLES") != string::npos )
            {
                global_collisionSpace->addRobot( rob );
            }
        }
    }
}

void DistFieldWidget::deleteDistanceField()
{
    if (global_collisionSpace != NULL) 
    {
        delete global_collisionSpace;
        global_collisionSpace = NULL;
        cout << "Distance field Deleted" << endl;
    }
    else 
    {
        cout << "Distance field doesn't exist!!!" << endl;
    }
}

void DistFieldWidget::generateRobotBoundingVolumes()
{
    if (global_collisionSpace == NULL) 
    {
        cout << "Distance field doesn't exist!!!" << endl;
        return;
    }
    
    Robot* rob = global_collisionSpace->getRobot();
    
    if (rob == NULL) 
    {
        cout << "No robot in collision space!!!" << endl;
        return;
    }
    
    BodySurfaceSampler* sampler = global_collisionSpace->getBodySampler();
    
    if (sampler == NULL) 
    {
        cout << "No body sampler in  collision space!!!" << endl;
        return;
    }
    
    cout << "Generate All Robot Collision Points" << endl;  
    sampler->generateRobotBoudingCylinder( rob, rob->getAllJoints() );
    sampler->generateAllRobotCollisionPoints( rob );
    return;
    
    // Set the active joints (links)
    std::vector<int> active_joints;
    active_joints.clear();
    active_joints.push_back( 1 );
    // active_joints.push_back( 2 );
    //  active_joints.push_back( 6 );
    //  active_joints.push_back( 7 );
    //  active_joints.push_back( 8 );
    //  active_joints.push_back( 9 );
    //  active_joints.push_back( 10 );
    //  active_joints.push_back( 11 );
    //  active_joints.push_back( 12 );
    //  
    //  active_joints.push_back( 14 );
    //  active_joints.push_back( 15 );
    
    // Set the planner joints
    std::vector<int> planner_joints;
    planner_joints.clear();
    planner_joints.push_back( 1 );
    //  planner_joints.push_back( 6 );
    //  planner_joints.push_back( 7 );
    //  planner_joints.push_back( 8 );
    //  planner_joints.push_back( 9 );
    //  planner_joints.push_back( 10 );
    //  planner_joints.push_back( 11 );
    //  planner_joints.push_back( 12 );
    
    // Generate cylinders
    if (!active_joints.empty()) 
    {
        vector<Joint*> joints;
        joints.clear();
        for (unsigned int i=0; i<active_joints.size(); i++) 
        {
            joints.push_back( rob->getJoint( active_joints[i] ) );
        }
        
        sampler->generateRobotBoudingCylinder( rob, joints );
        cout << "Bounding volumes generated for robot " << rob->getName() << endl;
    }
    else
        cout << "No robot bounding cylinder generated!!!" << endl;
    
    
    // Generate collision points  
    if (!planner_joints.empty()) 
    {
        vector<int> planner_joints_id;
        for (unsigned int i=0; i<planner_joints.size(); i++) 
        {
            planner_joints_id.push_back( planner_joints[i] );
        }
        
        sampler->generateRobotCollisionPoints( rob, active_joints, planner_joints_id );
        cout << "Collision points generated for robot " << rob->getName() << endl;
    }
    else
        cout << "No robot collision points generated!!!" << endl;
    
}
