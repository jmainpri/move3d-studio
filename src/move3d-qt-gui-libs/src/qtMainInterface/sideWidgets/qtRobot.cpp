/*
 *  qtRobot.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CRNS. All rights reserved.
 *
 */

#include "qtRobot.hpp"
#include "ui_qtRobot.h"

#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

#include "planner_handler.hpp"

#include "planner/plannerFunctions.hpp"
#include "planner/replanning.hpp"

#if defined( MOVE3D_CORE ) 
#include "MultiRun.hpp"
#include "SaveContext.hpp"
#include "testModel.hpp"
#include "API/project.hpp"
//#include "Greedy/GridCollisionChecker.h"
#endif

#ifdef HRI_PLANNER
#include <libmove3d/hri/hri.h>
#endif

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

using namespace std;
using namespace tr1;

extern string global_ActiveRobotName;
#ifdef MULTILOCALPATH
extern ManipulationTestFunctions* global_manipPlanTest;
#endif

shared_ptr<Configuration> qInit;
shared_ptr<Configuration> qGoal;


RobotWidget::RobotWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::RobotWidget),
m_mainWindow(NULL)
{
	m_ui->setupUi(this);
	
	initModel();
	initManipulation();
	//initVoxelCollisionChecker();
  initTrajectoryFromConfig();
}

RobotWidget::~RobotWidget()
{
	delete m_ui;
}

//---------------------------------------------------------------------
// Robot
//---------------------------------------------------------------------
void RobotWidget::initRobot()
{
	m_mainWindow->Ui()->formRobot->initAllForms(m_mainWindow->getOpenGL());	
}

MoveRobot*  RobotWidget::getMoveRobot()
{
	return m_mainWindow->Ui()->formRobot;
}

//---------------------------------------------------------------------
// TEST MODEL
//---------------------------------------------------------------------
void RobotWidget::initModel()
{
	connect(m_ui->pushButtonCollision,SIGNAL(clicked()),this,SLOT(collisionsTest()));
	connect(m_ui->pushButtonLocalPath,SIGNAL(clicked()),this,SLOT(localpathsTest()));
	connect(m_ui->pushButtonCost,SIGNAL(clicked()),this,SLOT(costTest()));
	connect(m_ui->pushButtonTestAll,SIGNAL(clicked()),this,SLOT(allTests()));
	connect(m_ui->pushButtonSetObjectToCarry,SIGNAL(clicked()),this,SLOT(SetObjectToCarry()));

	//Traj from via point
	connect(m_ui->pushButtonLaunchSoft,SIGNAL(clicked()),this,SLOT(makeSoftmotionTraj()));
  connect(m_ui->pushButtonLaunchNormal,SIGNAL(clicked()),this,SLOT(makeNormalTraj()));
	connect(m_ui->pushButtonSaveConf,SIGNAL(clicked()),this,SLOT(saveConfig()));
	connect(m_ui->pushButtonClearTraj,SIGNAL(clicked()),this,SLOT(clearConfigs()));
	connect(m_ui->pushButtonDeleteConf,SIGNAL(clicked()),this,SLOT(deleteConfig()));
	// end Traj from via point
	
	connect(ENV.getObject(Env::numberOfCollisionPerSec),SIGNAL(valueChanged(QString)),m_ui->labelCollision,SLOT(setText(QString)));
	connect(ENV.getObject(Env::numberOfLocalPathPerSec),SIGNAL(valueChanged(QString)),m_ui->labelLocalPath,SLOT(setText(QString)));
	connect(ENV.getObject(Env::numberOfCostPerSec),SIGNAL(valueChanged(QString)),m_ui->labelTimeCost,SLOT(setText(QString)));
	
	connect(m_ui->pushButtonAttMat,SIGNAL(clicked()),this,SLOT(setAttMatrix()));
	
	QString RobotObjectToCarry("No Object");
	
	ENV.setString(Env::ObjectToCarry,RobotObjectToCarry);
	
	// Grab Object
	for(int i =0;i<XYZ_ENV->nr;i++)
	{
		if(XYZ_ENV->robot[i]->joints[1]->type == P3D_FREEFLYER )
		{
			if( XYZ_ENV->robot[i]->njoints == 1 )
			{
				QString FFname(XYZ_ENV->robot[i]->name);
				m_ui->comboBoxGrabObject->addItem(FFname);
				m_FreeFlyers.push_back(FFname);
				//                cout<< " FreeFlyer = "  << XYZ_ENV->robot[i]->name << endl;
			}
		}
	}
	
	m_ui->comboBoxGrabObject->setCurrentIndex(0);
	connect(m_ui->comboBoxGrabObject, SIGNAL(currentIndexChanged(int)),this, SLOT(currentObjectChange(int))/*, Qt::DirectConnection*/);
	
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxIsWeightedRot,	Env::isWeightedRotation);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxFKSampling,		Env::FKShoot);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxFKDistance,		Env::FKDistance);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawBox,				Env::drawBox);
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxWeightedRot, m_ui->horizontalSliderWeightedRot , Env::RotationWeight );
	
	connect(m_ui->pushButtonGrabObject,SIGNAL(clicked()),					this,SLOT(GrabObject()));
	connect(m_ui->pushButtonReleaseObject,SIGNAL(clicked()),			this,SLOT(ReleaseObject()));
	
#ifdef HRI_PLANNER
	connect(m_ui->pushButtonComputeLeftArmGIK,SIGNAL(clicked()),	this,SLOT(computeHriGikLARM()));
	connect(m_ui->pushButtonComputeRightArmGIK,SIGNAL(clicked()),	this,SLOT(computeHriGikRARM()));
#endif
	
  connect(m_ui->pushButtonPrintAbsPos,SIGNAL(clicked()),        this,SLOT(printAbsPos()));
	connect(m_ui->pushButtonPrintCurrentPos,SIGNAL(clicked()),		this,SLOT(printCurrentPos()));
  
	
#if defined(LIGHT_PLANNER)
	connect(m_ui->pushButtonSwitchFKIK,SIGNAL(clicked()),					this,SLOT(switchFKIK()));
#endif
	
	connect(m_ui->pushButtonPrintPQPColPair,SIGNAL(clicked()),		this,SLOT(printPQPColPair()));
	
	void	setMaximum ( int max );
	void	setMinimum ( int min );
	
}

void RobotWidget::printAbsPos()
{
	Robot* r = global_Project->getActiveScene()->getActiveRobot();
  
  cout << "Print Matrix for robot " << r->getName() << endl;
  
  unsigned int joint = m_ui->spinBoxPrintAbsPos->value();
  
  if( joint <= r->getNumberOfJoints() )
     cout << r->getJoint(joint)->getMatrixPos().matrix() << endl;
  else
    cout << "Cannont print for this robot" << endl;

}

void RobotWidget::costTest()
{
	if(ENV.getBool(Env::isCostSpace))
	{
		TestModel tests;
		tests.nbOfCostPerSeconds();
	}
}

void RobotWidget::collisionsTest()
{
	TestModel tests;
	tests.nbOfColisionsPerSeconds();
}

void RobotWidget::localpathsTest()
{
	TestModel tests;
	tests.nbOfLocalPathsPerSeconds();
}

void RobotWidget::allTests()
{	
	TestModel tests;
	tests.runAllTests();
}

void RobotWidget::setAttMatrix()
{
#ifdef LIGHT_PLANNER
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	//  p3d_compute_attached_matrix_from_virt_obj(robotPt->ccCntrts[0]);
	for(int i = 0; i < robotPt->nbCcCntrts; i++)
	{
		//p3d_compute_Tatt(robotPt->ccCntrts[i]);
		
		cout << "Tatt = " << endl;
		for (i = 0; i < 4; i++)
		{
			cout << robotPt->ccCntrts[0]->Tatt[i][0] << " " <<
			" " << robotPt->ccCntrts[0]->Tatt[i][1] <<
			" " << robotPt->ccCntrts[0]->Tatt[i][2] <<
			" " << robotPt->ccCntrts[0]->Tatt[i][3] << endl;
		}
		cout << endl;
	}
#endif
}

void RobotWidget::computeHriGik(bool leftArm)
{	
	int i=0;
	for(i=0; i<XYZ_ENV->nr; i++){
		if( strcasestr(XYZ_ENV->robot[i]->name,"VISBALL") )
			break;
	}
	if(i==XYZ_ENV->nr){
		printf("No visball in the environment\n");
		return;
	}
	
#ifdef HRI_PLANNER
	// 1 - Select Goto point
	p3d_vector3 Tcoord;
	
	Tcoord[0] = XYZ_ENV->robot[i]->joints[1]->abs_pos[0][3];
	Tcoord[1] = XYZ_ENV->robot[i]->joints[1]->abs_pos[1][3];
	Tcoord[2] = XYZ_ENV->robot[i]->joints[1]->abs_pos[2][3];
	
	
	// 2 - Select Task
	HRI_GIK_TASK_TYPE task;
	
	if (leftArm == true) 
	{
                task = GIK_LATREACH; // Left Arm GIK
	}
	else 
	{
                task = GIK_RATREACH; // Left Arm GIK
	}
	
  cout << "HRI_AGENTS * agents = hri_create_agents()" << endl;
  
	// 3 - Select Agent
	HRI_AGENTS * agents = hri_create_agents();
	
	configPt q;
  
	
	double distance_tolerance = 0.05;
	
	/** if(	agents->humans_no > 0 ) // Humans
	{
		q = p3d_get_robot_config(agents->humans[0]->robotPt);
		hri_agent_single_task_manip_move(agents->humans[0], task, &Tcoord, distance_tolerance, &q);
		p3d_set_and_update_this_robot_conf(agents->humans[0]->robotPt,q);
	}
	else */ 
  //if ( agents->robots_no > 0) // Robots
	//{
  HRI_AGENT* agent = hri_get_one_agent_of_type(agents, HRI_HERAKLES);
  q = p3d_get_robot_config(agent->robotPt);
  if( hri_agent_single_task_manip_move(agent, task, &Tcoord, distance_tolerance, &q) )
  {
    cout << "GIK succeded" << endl; 
  }
  else {
    cout << "GIK failed" << endl;
  }
  p3d_set_and_update_this_robot_conf(agent->robotPt, q);
  
  
  
  agent = hri_get_one_agent_of_type(agents, HRI_ACHILE);
  q = p3d_get_robot_config(agent->robotPt);
  if( hri_agent_single_task_manip_move(agent, task, &Tcoord, distance_tolerance, &q) )
  {
    cout << "GIK succeded" << endl; 
  }
  else {
    cout << "GIK failed" << endl;
  }
  
  p3d_set_and_update_this_robot_conf(agent->robotPt, q);
  
//	}
//	else 
//  {
//		cout << "Warning: No Agent for GIK" << endl;
//	}
	
#else
	cout << "HRI_PLANNER not defined" << endl;
#endif
	
	
	//delete_config(robotPt,q);	
	m_mainWindow->drawAllWinActive();
}

void RobotWidget::currentObjectChange(int i)
{
	if((m_FreeFlyers.size() > 0) && (i != 0))
	{
		//        cout << "Env::ObjectToCarry  is "<< m_FreeFlyers[i-1] << endl;
		ENV.setString(Env::ObjectToCarry,m_FreeFlyers[i-1]);
	}
}

void RobotWidget::SetObjectToCarry()
{
#ifdef LIGHT_PLANNER
	if(m_FreeFlyers.size() > 0)
	{
		p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
                //p3d_set_object_to_carry(robotPt,ENV.getString(Env::ObjectToCarry).toStdString().c_str());
		
		// Set the dist of the object to the radius of the carried object
		//robotPt->curObjectJnt->dist = robotPt->carriedObject->joints[1]->dist;
		//Warning TODO this has changed
    
		double radius = 1.5;
		//take only x and y composantes of the base
		double dof[2][2];
		for(int i = 0; i < 2; i++){
			dof[i][0] = p3d_jnt_get_dof(robotPt->joints[1], i) - radius;
			dof[i][1] = p3d_jnt_get_dof(robotPt->joints[1], i) + radius;
		}
		for(int i = 0; i < 2; i++){
			p3d_jnt_set_dof_rand_bounds(robotPt->curObjectJnt, i, dof[i][0], dof[i][1]);
		}
		
	}
	else
	{
		p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
		// Set the dist of the object to the radius of the carried object
		
		cout << "Setting Dist of Joint : " << robotPt->joints[6]->name << endl;
		cout << "To Joint : "  << robotPt->joints[7]->name << endl;
		
		cout << robotPt->joints[7]->dist << "  Takes "  << robotPt->joints[6]->dist << endl;
		
		robotPt->joints[7]->dist = robotPt->joints[6]->dist;
	}
#endif
}

void RobotWidget::GrabObject()
{
	
#if defined( LIGHT_PLANNER ) && defined( PQP )
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	cout << "Robot = " << robotPt->name <<  endl;
        //p3d_set_object_to_carry(robotPt,ENV.getString(Env::ObjectToCarry).toStdString().c_str());
	p3d_grab_object2(robotPt,0);
	
	//    if(m_FreeFlyers.size() > 0)
	//    {
	//        p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	//		//        p3d_rob *carriedObject;
	//		
	//        p3d_set_object_to_carry(robotPt,ENV.getString(Env::ObjectToCarry).toStdString().c_str());
	//		//        p3d_matrix4 saved;
	//		//        p3d_mat4Copy(robotPt->curObjectJnt->abs_pos,saved);
	//        p3d_mat4Copy(robotPt->carriedObject->joints[1]->abs_pos,robotPt->curObjectJnt->abs_pos);
	//        p3d_grab_object(robotPt,0);
	//		//        p3d_mat4Copy(saved,robotPt->curObjectJnt->abs_pos);
	//		//        configPt q = p3d_get_robot_config(robotPt);
	//		
	//		//        robotPt->ROBOT_POS = q;
	//		//        p3d_set_and_update_robot_conf(q);
	//        p3d_mat4Print(robotPt->ccCntrts[0]->Tatt,"curObject Grab");
	//    }
#endif
}

void RobotWidget::ReleaseObject()
{
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	cout << "Robot = " << robotPt->name <<  endl;
	/*
	 p3d_release_object(robotPt);
	 deactivateCcCntrts(robotPt, -1);
	 //	configPt qi = p3d_alloc_config(robotPt);
	 //	p3d_copy_config_into(robotPt, _robotPt->ROBOT_POS, &qi);
	 x
	 //p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qi);
	 double tx, ty, tz, ax, ay, az;
	 p3d_mat4ExtractPosReverseOrder2(robotPt->CurObjtJnt->abs_pos, &tx, &ty, &tz, &ax, &ay, &az);
	 
	 //	p3d_set_and_update_this_robot_conf(robotPt, qi);
	 //	p3d_destroy_config(_robotPt, qi);
	 
	 qi = p3d_get_robot_config(robotPt);
	 p3d_copy_config_into(robotPt, qi, &_robotPt->ROBOT_POS);
	 p3d_destroy_config(_robotPt, qi);
	 m_ui->mainWindow->drawAllWindowActive();
	 */
	
#if defined ( LIGHT_PLANNER ) && defined( PQP )
	//    m_ui->comboBoxGrabObject-
	//    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	p3d_release_object(robotPt);
	//    m_ui->comboBoxGrabObject->setCurrentIndex(0);
#endif
};

void RobotWidget::printCurrentPos()
{
	Robot currRobot((p3d_rob*) p3d_get_desc_curid(P3D_ROBOT));
	shared_ptr<Configuration> q = currRobot.getCurrentPos();
	q->print(true );
}

#ifdef LIGHT_PLANNER
void RobotWidget::switchFKIK()
{
	cout << "Switching FK to IK" << endl;
	
	Robot* ptrRob = global_Project->getActiveScene()->getActiveRobot();
	
	if ( ptrRob->isActiveCcConstraint() ) 
	{
		ptrRob->deactivateCcConstraint();
	}
	else
	{
		ptrRob->activateCcConstraint();
	}
	
	getMoveRobot()->setRobotConstraintedDof(ptrRob);
	
}
#endif


void RobotWidget::printPQPColPair()
{
  cout << "print collision pair:" << endl;
	p3d_print_col_pair();
}
/*void RobotWidget::initVoxelCollisionChecker()
 {
 m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawVoxelGrid,				Env::drawGrid);
 
 connect(m_ui->pushButtonCreateVoxelCollisionChecker,SIGNAL(clicked()),this,SLOT(createVoxelCC()));
 connect(m_ui->pushButtonDeleteVoxelCollisionChecker,SIGNAL(clicked()),this,SLOT(deleteVoxelCC()));
 
 connect(m_ui->pushButtonVoxelCC,SIGNAL(clicked()),this,SLOT(voxelCCTest()));
 connect(ENV.getObject(Env::numberOfCollisionPerSec),SIGNAL(valueChanged(QString)),m_ui->labelVoxelTime,SLOT(setText(QString)));
 
 connect(m_ui->spinBoxVoxelCCnbCells,SIGNAL(valueChanged(int)),ENV.getObject(Env::nbCells),SLOT(set(int)));
 connect(ENV.getObject(Env::nbCells),SIGNAL(valueChanged(int)),m_ui->spinBoxVoxelCCnbCells,SLOT(setValue(int)));
 }
 
 void RobotWidget::createVoxelCC()
 {
 API_activeGrid = global_GridCollisionChecker = new GridCollisionChecker;
 ENV.setBool(Env::drawGrid, true);
 }
 
 void RobotWidget::deleteVoxelCC()
 {
 ENV.setBool(Env::drawAll,false);
 delete API_activeGrid;
 API_activeGrid = NULL;
 }
 
 void RobotWidget::voxelCCTest()
 {
 TestModel tests;
 tests.nbOfVoxelCCPerSeconds();
 }*/

// ------------------------------------------------------------------------------
// Arm manipulations
// ------------------------------------------------------------------------------
void RobotWidget::initManipulation()
{
  connect(m_ui->checkBoxIsDebugManip, SIGNAL(toggled(bool)),              this,SLOT(isDebugManip(bool)));
  connect(m_ui->checkBoxIsCartesianMode, SIGNAL(toggled(bool)),           this,SLOT(isCartesianMode(bool)));
  
  connect(m_ui->pushButtonSetStart,SIGNAL(clicked()), this,SLOT(setRobotToInitConfig()));
  connect(m_ui->pushButtonSetGoal,SIGNAL(clicked()), this,SLOT(setRobotToGoalConfig()));
  
  connect(m_ui->pushButtonResetManipulationData,SIGNAL(clicked()),        this,SLOT(resetManipulationData()));
  connect(m_ui->pushButtonRunTest,SIGNAL(clicked()),                      this,SLOT(runManipTest()));
	connect(m_ui->pushButtonArmFree,SIGNAL(clicked()),                      this,SLOT(armFree()));
	connect(m_ui->pushButtonArmPickGoto,SIGNAL(clicked()),                  this,SLOT(armPickGoto()));
	connect(m_ui->pushButtonArmPickTakeToFree,SIGNAL(clicked()),            this,SLOT(armPickTakeToFree()));
  connect(m_ui->pushButtonArmPickTakeToFreePoint,SIGNAL(clicked()),       this,SLOT(armPickTakeToFreePoint()));
	connect(m_ui->pushButtonArmPickGotoAndTakeToFree,SIGNAL(clicked()),     this,SLOT(armPickGotoAndTakeToFree()));
  connect(m_ui->pushButtonReplanning,SIGNAL(clicked()),                   this,SLOT(armReplanTask()));
  connect(m_ui->pushButtonLoadWorkspaceFile,SIGNAL(clicked()),            this,SLOT(loadWorkspace()));
  connect(m_ui->pushButtonOptimizeRedundantCost,SIGNAL(clicked()),        this,SLOT(optimizeRedundantCost()));
  
  connect(this, SIGNAL(selectedPlanner(QString)),
          global_plannerHandler, SLOT(startPlanner(QString)));
  
  initObjectSupportAndPlacementCombo();
}

void RobotWidget::initObjectSupportAndPlacementCombo()
{
  m_ui->comboBoxObjectName->addItem( "No Object" );
  m_ui->comboBoxPlacementName->addItem( "No Object" );
  m_ui->comboBoxSupportName->addItem( "No Object" );
  
  // Object To Grasp 
	for(int i =0;i<int(m_FreeFlyers.size());i++)
	{
    m_ui->comboBoxObjectName->addItem( m_FreeFlyers[i] );
    m_ui->comboBoxPlacementName->addItem( m_FreeFlyers[i] );
    m_ui->comboBoxSupportName->addItem( m_FreeFlyers[i] );
	}
	
	m_ui->comboBoxObjectName->setCurrentIndex(0);
	connect(m_ui->comboBoxObjectName, SIGNAL(currentIndexChanged(int)),this, SLOT(objectNameChanged(int)));
  
	m_ui->comboBoxPlacementName->setCurrentIndex(0);
	connect(m_ui->comboBoxPlacementName, SIGNAL(currentIndexChanged(int)),this, SLOT(placementNameChanged(int)));
  
  m_ui->comboBoxSupportName->setCurrentIndex(0);
	connect(m_ui->comboBoxSupportName, SIGNAL(currentIndexChanged(int)),this, SLOT(supportNameChanged(int)));
}

void RobotWidget::setRobotToInitConfig()
{
  if( !global_manipPlanTest )
  {
    cout << "global_manipPlanTest is not initialized!!!" << endl;
    return;
  }
  
  cout << "Set to qInit configuration" << endl;
  qInit->getRobot()->setAndUpdate( *qInit );
}

void RobotWidget::setRobotToGoalConfig()
{
  if( !global_manipPlanTest )
  {
    cout << "global_manipPlanTest is not initialized!!!" << endl;
    return;
  }
  
  cout << "Set to qGoal configuration" << endl;
  qGoal->getRobot()->setAndUpdate( *qGoal );
}

std::string RobotWidget::getNameOfFreeFlyerFromIndex(int id)
{
  string name("");
  
  if((id >= 0) && (id < int(m_FreeFlyers.size())))
	{
    if( id == 0 )
    {
      name = "No Object";
    }
    else
    {
      name = m_FreeFlyers[id-1].toStdString();
    }
    
    cout << "Set object name to : " << name << endl;
  }
  
  return name;
}


#ifdef MULTILOCALPATH
void RobotWidget::objectNameChanged(int id)
{
  string name = getNameOfFreeFlyerFromIndex( id );
 
  if( name == "")
  {
    cout << "Object doesn't exist" << endl;
    return;
  }
  
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  if( name == "No Object" )
  {
    global_manipPlanTest->resetObject();
  }
  else
  {
    global_manipPlanTest->setObject( name );
  }
}

void RobotWidget::placementNameChanged(int id)
{
  string name = getNameOfFreeFlyerFromIndex( id );
  
  if( name == "")
  {
    cout << "Placement doesn't exist" << endl;
    return;
  }
  
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  if( name == "No Object" )
  {
    global_manipPlanTest->resetPlacement();
  }
  else
  {
    global_manipPlanTest->setPlacement( name );
  }
}

void RobotWidget::supportNameChanged(int id)
{
  string name = getNameOfFreeFlyerFromIndex( id );
  
  if( name == "")
  {
    cout << "Support doesn't exist" << endl;
    return;
  }
  
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  if( name == "No Object" )
  {
    global_manipPlanTest->resetSupport();
  }
  else
  {
    global_manipPlanTest->setSupport( name );
  }
}

// ------------------------------------------------------------------------------
// The Manip namespace holds the function
// to call the Manipulation Planner
// ------------------------------------------------------------------------------
#ifdef MULTILOCALPATH
namespace Manip
{
  ManipulationPlanner* manipulation = NULL;
  
  // Manip locals
  typedef enum ManipulationType
	{
		armFree,
		pickGoto,
		takeToFree,
    takeToFreePoint,
		pickGotoAndTakeToFree,
    rePlanning
	} 
	ManipulationType;
  
  bool firstRun = true;
  bool isCartesianMode = false;
  
  int currentIdTest=0;
  
  ManipulationType Phase;
  
  void runCurrentTest()
  {
    global_manipPlanTest->runTest(currentIdTest);
  }
  
  //! Main function for launching the manipulation planner
  //! it is based on the testing class
  //! it reads the Cartesian mode variable to determine 
  //! which type of method will be used (it is called from the planner thread)
  void runManipulation()
  {
    p3d_rob* rob =  global_manipPlanTest->getManipulationPlanner()->robot();
    
    if(Manip::isCartesianMode)
    {
      for(unsigned int i=0; i < rob->armManipulationData->size(); i++) 
        global_manipPlanTest->getManipulationPlanner()->setArmCartesian(i, true);
    }
    else 
    {
      for(unsigned int i=0; i < rob->armManipulationData->size(); i++) 
        global_manipPlanTest->getManipulationPlanner()->setArmCartesian(i, false);
    }
    
    switch (Phase) 
    {
      case Manip::armFree :
      {
        cout << "Manip::armFree" << endl;
        global_manipPlanTest->setInitConfiguration(qInit->getConfigStructCopy());
        global_manipPlanTest->setGoalConfiguration(qGoal->getConfigStructCopy());
        
        global_manipPlanTest->runTest(1);
      }
        break;
        
      case Manip::pickGoto :
      {
        cout << "Manip::pickGoto" << endl;
        global_manipPlanTest->setInitConfiguration(qInit->getConfigStructCopy());
        global_manipPlanTest->setGoalConfiguration(qGoal->getConfigStructCopy());
        
        global_manipPlanTest->runTest(2);
      }
        break;
        
      case Manip::rePlanning :
      {
        p3d_vector3 otp;
        otp[0] = 4.250;
        otp[1] = -2.60;
        otp[2] = 1.000;
        
        SM_TRAJ traj;
        
        int id_localpath;
        const double t_rep = 0.0; // in second
        const double tau = 0.0;
        p3d_getQSwitchIDFromMidCVS(tau, t_rep, &id_localpath); 
        
        global_manipPlanTest->getManipulationPlanner()->armReplan(otp,id_localpath,traj);
      }
        break;
        
      default:
        cout << "Manip::Test not implemented" << endl;
        break;
    }
    
    g3d_draw_allwin_active();
    ENV.setBool(Env::isRunning,false);
    cout << "Ends Manipulation Thread" << endl;
  }


  //! Main function for launching the manipulation planner
  //! it is based on the testing class
  //! it reads the Cartesian mode variable to determine
  //! which type of method will be used (it is called from the planner thread)
  void runNavigation()
  {
    p3d_rob* rob =  global_manipPlanTest->getManipulationPlanner()->robot();

  //  if(Manip::isCartesianMode)
  //  {
  //    for(unsigned int i=0; i < rob->armManipulationData->size(); i++)
  //      global_manipPlanTest->getManipulationPlanner()->setArmCartesian(i, true);
  //  }
  //  else
  //  {
  //    for(unsigned int i=0; i < rob->armManipulationData->size(); i++)
  //      global_manipPlanTest->getManipulationPlanner()->setArmCartesian(i, false);
  //  }
  //
  //  switch (Phase)
  //  {
  //    case Manip::armFree :
  //    {
  //      cout << "Manip::armFree" << endl;
  //      global_manipPlanTest->runTest(1);
  //    }
  //      break;
  //
  //    case Manip::pickGoto :
  //    {
  //      cout << "Manip::pickGoto" << endl;
  //      global_manipPlanTest->runTest(2);
  //    }
  //      break;
  //
  //    case Manip::rePlanning :
  //    {
  //      p3d_vector3 otp;
  //      otp[0] = 4.250;
  //      otp[1] = -2.60;
  //      otp[2] = 1.000;
  //
  //      SM_TRAJ traj;
  //
  //      int id_localpath;
  //      const double t_rep = 0.0; // in second
  //      const double tau = 0.0;
  //      p3d_getQSwitchIDFromMidCVS(tau, t_rep, &id_localpath);
  //
  //      global_manipPlanTest->getManipulationPlanner()->armReplan(otp,id_localpath,traj);
        std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
        std::vector <SM_TRAJ> smTrajs;

        global_manipPlanTest->getManipulationPlanner()->planNavigation(rob->ROBOT_POS, rob->ROBOT_GOTO, confs, smTrajs);
  //    }
  //      break;
  //
  //    default:
  //      cout << "Manip::Test not implemented" << endl;
  //      break;
  //  }
  //
  //  g3d_draw_allwin_active();
  //  ENV.setBool(Env::isRunning,false);
    cout << "Ends Manipulation Thread" << endl;
  }

};
#endif


// ------------------------------------------------------------------------------
// All buttons callbacks are defined here
// for manipulation planning 
// ------------------------------------------------------------------------------
void RobotWidget::isCartesianMode(bool on)
{
#ifdef MULTILOCALPATH
  if(on){
    Manip::isCartesianMode = true;
  }else{
    Manip::isCartesianMode = false;
  }
  
  cout << "Cartesian mode switched to : " << Manip::isCartesianMode << endl;
  
#else
  cout << "Error : use MultiLocalPath" << endl;
#endif
}

void RobotWidget::isDebugManip(bool value)
{
  if(global_manipPlanTest)
  {
    global_manipPlanTest->setDebugMode(value);
    
    cout << "Debug mode switched to : " << value << endl;
  }
  else {
    cout << "Manip test is NULL" << endl;
  }
}

void RobotWidget::resetManipulationData()
{
  cout << "-----------------------------------------------------" << endl;
  cout << " RobotWidget::resetManipulationData -----------------" << endl;
  
  if (!global_manipPlanTest) 
  {
//    Robot* rob = global_Project->getActiveScene()->getRobotByNameContaining( global_ActiveRobotName );
//    
//    qInit = rob->getInitialPosition();
//    qGoal = rob->getGoTo();
//    
//    if(qInit->equal(*qGoal))
//    {
//      cout << "Manip::firstRun : qInit->equal(*qGoal)" << endl;
//    }
    
    global_manipPlanTest = new ManipulationTestFunctions( global_ActiveRobotName  );
  }
  
//  p3d_rob* rob1 = qInit->getRobot()->getRobotStruct();
//  p3d_rob* rob2 = qGoal->getRobot()->getRobotStruct();
//  
//  if (rob1 != rob2) 
//  {
//    cout << "Error in resetManipulationData robot is not the same in init and goal" << endl;
//  }
  
//  p3d_copy_config_into(rob1,qInit->getConfigStruct(),&(rob1->ROBOT_POS));
//  p3d_copy_config_into(rob1,qGoal->getConfigStruct(),&(rob1->ROBOT_GOTO));
  
  Robot* rob = global_Project->getActiveScene()->getRobotByNameContaining( global_ActiveRobotName );
//  Robot* rob = global_Project->getActiveScene()->getRobotByName(rob1->name);
  
  qInit = rob->getInitialPosition();
  qGoal = rob->getGoTo();
  
  rob->setAndUpdate(*qInit);
  
  cout << "global_manipPlanTest->getManipPlanner()->setPlanningMethod" << endl;
  // Set Planning functions

  global_manipPlanTest->initManipulationGenom ();
  
  configPt q1 = p3d_alloc_config(rob->getRobotStruct());
  configPt q2 = p3d_alloc_config(rob->getRobotStruct());
  
  p3d_copy_config_into(rob->getRobotStruct(), qInit->getConfigStruct(), &(q1));
  p3d_copy_config_into(rob->getRobotStruct(), qGoal->getConfigStruct(), &(q2));
  
//  p3d_copy_config_into(rob->getRobotStruct(),qInit->getConfigStruct(),&(rob->getRobotStruct()->ROBOT_POS));
//  p3d_copy_config_into(rob->getRobotStruct(),qGoal->getConfigStruct(),&(rob->getRobotStruct()->ROBOT_GOTO));
//  
//  p3d_set_and_update_this_robot_conf(rob->getRobotStruct(),rob->getRobotStruct()->ROBOT_POS);
//  
//  m_mainWindow->drawAllWinActive();
  
  global_manipPlanTest->setInitConfiguration (q1);
  global_manipPlanTest->setGoalConfiguration (q2);
  
  global_manipPlanTest->getManipPlanner()->setPlanningMethod( planner_Function );
  global_manipPlanTest->getManipPlanner()->setSmoothingMethod( smoothing_Function );
  global_manipPlanTest->getManipPlanner()->setReplanningMethod( replanning_Function );
  
  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  
  m_mainWindow->drawAllWinActive();
}

void RobotWidget::optimizeRedundantCost()
{
//  p3d_rob* robot = global_manipPlanTest->getManipulationPlanner->robot();
//  int redJnt = 
//  double cost = optimizeRedundentJointConfigCost( robot, redJntId, q, objectPos, tAtt, grasp, armId, 10000 );
//  cout << "Cost" << cost << endl;
}

void RobotWidget::armFree()
{
	cout << "Manipulation : free" << endl;
	
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  Manip::Phase = Manip::armFree;
  m_mainWindow->isPlanning();
  
  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  
  emit(selectedPlanner(QString("Manipulation")));
}

void RobotWidget::armPickGoto()
{
	cout << "Manipulation : pick goto" << endl;
	
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  Manip::Phase = Manip::pickGoto;
  m_mainWindow->isPlanning();
  
  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  
	emit(selectedPlanner(QString("Manipulation")));
}

void RobotWidget::armPickTakeToFree()
{
	cout << "Manipulation : take to free" << endl;
	
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  Manip::Phase = Manip::takeToFree;
  m_mainWindow->isPlanning();
  resetManipulationData();
  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  
	emit(selectedPlanner(QString("Manipulation")));
}

void RobotWidget::armPickTakeToFreePoint()
{
  cout << "Manipulation : take to free point" << endl;
	
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  Manip::Phase = Manip::takeToFreePoint;
  m_mainWindow->isPlanning();

  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  
	emit(selectedPlanner(QString("Manipulation")));
}

void RobotWidget::armPickGotoAndTakeToFree()
{
	cout << "Manipulation : pick goto and take to free" << endl;

  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  Manip::Phase = Manip::pickGotoAndTakeToFree;
  m_mainWindow->isPlanning();

  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  
	emit(selectedPlanner(QString("Manipulation")));
}

void RobotWidget::armReplanTask()
{
  cout << "Manipulation : rePlanning" << endl;
	
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  Manip::Phase = Manip::rePlanning;
  m_mainWindow->isPlanning();
  
  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  
  emit(selectedPlanner(QString("Manipulation")));
}

void RobotWidget::runManipTest()
{
 	cout << "Run Current Manipulation Tests: " << endl;
  
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  Manip::currentIdTest = m_ui->spinBoxManipTestId->value();
  m_mainWindow->isPlanning();
  
  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  
  emit(selectedPlanner(QString("ManipCurrentTest")));
}

void RobotWidget::loadWorkspace()
{
  if (!global_manipPlanTest) 
  {
    resetManipulationData();
  }
  
  std::string fileName( "./statFiles/workspace.csv" );
 	cout << "Loading file : " << fileName << endl;
  
  global_manipPlanTest->readWorkspaceFromFile( fileName );
}
#endif // MULTILOCALPATH

//-----------------------------------------------------------------------
//---------
//---------  Trajectory generation
//---------
//-----------------------------------------------------------------------

//  connect(this, SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));

void RobotWidget::initTrajectoryFromConfig()
{
  m_configNum = 0;
  on_pushButtonrefrech_clicked();
}

void RobotWidget::makeSoftmotionTraj()
{
	m_mainWindow->isPlanning();
	emit(selectedPlanner(QString("makeTrajFromViaPoints")));
}

void RobotWidget::makeNormalTraj()
{
  Robot* robot = global_Project->getActiveScene()->getActiveRobot();
  API::Trajectory traj(robot);
  
  for(int i=(robot->getRobotStruct()->nconf-1) ; i>0; i--) 
  {
    std::tr1::shared_ptr<Configuration> q(new Configuration(robot,robot->getRobotStruct()->conf[i]->q));
    traj.push_back(q);
  }
  
  if(traj.replaceP3dTraj())
    cout << "Traj generated successfully for robot : " << robot->getName() 
    << " with " << traj.getNbOfViaPoints() << " via points" << endl;
  else
    cout << "Error : replacing p3d traj" << endl;
}

void RobotWidget::saveConfig()
{
	config_namePt config;

	string name( "Config_" );
	string num;

	stringstream out;
	out << m_configNum++;
	num = out.str();

	name = name + num;

	p3d_rob* robot = p3d_get_robot_by_name(global_ActiveRobotName.c_str());
	configPt q = p3d_get_robot_config( robot );

        p3d_set_new_robot_config(robot, name.c_str() , q, NULL, config);
	m_ui->spinBoxNavigate->setMaximum(robot->nconf - 1);
}

void RobotWidget::clearConfigs()
{
	p3d_rob* robot = p3d_get_robot_by_name(global_ActiveRobotName.c_str());
	int num = robot->nconf;
	for(int i=0 ; i<num; i++)
	{
		p3d_del_config(robot, robot->conf[0]);
	}
	m_ui->spinBoxNavigate->setMaximum(0);
}

void RobotWidget::deleteConfig()
{
	p3d_rob* robot = p3d_get_robot_by_name(global_ActiveRobotName.c_str());
	p3d_del_config(robot, robot->conf[m_ui->spinBoxNavigate->value()]);
	m_ui->spinBoxNavigate->setMaximum(robot->nconf - 1);
//	on_spinBoxNavigate_valueChanged(m_ui->spinBoxNavigate->getValue() - 1);
}


void RobotWidget::on_spinBoxNavigate_valueChanged(int value)
{
	p3d_rob* robot = p3d_get_robot_by_name(global_ActiveRobotName.c_str());
	Robot* rob = new Robot(robot);
	if ( (value < robot->nconf) && (value > 0)) 
	{
		configPt conf = robot->conf[value]->q;
		shared_ptr<Configuration> q(new Configuration(rob,conf));
		q->print();
		rob->setAndUpdate(*q);
    
    if( m_mainWindow )
      m_mainWindow->drawAllWinActive();
	}
}

void RobotWidget::on_pushButtonrefrech_clicked()
{
	p3d_rob* robot = p3d_get_robot_by_name(global_ActiveRobotName.c_str());
  
  if (robot != NULL) {
    m_ui->spinBoxNavigate->setMaximum(robot->nconf - 1);
    
    if( m_mainWindow )
      m_mainWindow->drawAllWinActive();
  }
}

void RobotWidget::on_pushButtonPlanNavigation_clicked()
{
    cout << "Navigation softmotion" << endl;

if (!global_manipPlanTest)
{
    resetManipulationData();
}

//Manip::Phase = Manip::armFree;
m_mainWindow->isPlanning();

global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );

emit(selectedPlanner(QString("NavigationSM")));
}
