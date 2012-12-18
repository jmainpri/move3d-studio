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
#include "planner/replanningAlgorithms.hpp"
#include "planner/replanningSimulators.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"

#include "API/Trajectory/trajectory.hpp" 

#include "HRI_costspace/HRICS_Miscellaneous.hpp"

#include <QMessageBox>
#include <QString>

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
shared_ptr<Configuration> qOpen;

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
  
#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
  cout << "Set grasp tab" << endl;
  // Initialize the grasp planner tab
  m_tabGrasp = new GraspPlannerWidget(m_ui->GraspPlanner);
  m_tabGrasp->setObjectName(QString::fromUtf8("tabGraspPlanner"));
  m_ui->graspplannerLayout->addWidget(m_tabGrasp);
#endif
}

RobotWidget::~RobotWidget()
{
    delete m_ui;
}

void RobotWidget::setMainWindow(MainWindow *ptrMW) 
{ 
  m_mainWindow = ptrMW; 
#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
  m_tabGrasp->setMainWindow( m_mainWindow );
#endif
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
  //connect(m_ui->pushButtonSetObjectToCarry,SIGNAL(clicked()),this,SLOT(SetObjectToCarry()));
  
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
  
  new QtShiva::SpinBoxConnector( this, m_ui->spinBoxArmId, Env::currentArmId );
  
  QString RobotObjectToCarry("No Object");
  
  ENV.setString(Env::ObjectToCarry,RobotObjectToCarry);
  
  // List all robots with only one 
  // Freeflyer joint
  for(int i =0;i<XYZ_ENV->nr;i++)
  {
    if( XYZ_ENV->robot[i]->njoints == 1 )
    {
      if(XYZ_ENV->robot[i]->joints[1]->type == P3D_FREEFLYER )
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

    connect(m_ui->pushButtonVirtualJointBounds,SIGNAL(clicked()),	this,SLOT(SetBounds()));
  
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

//    void	setMaximum ( int max );
//    void	setMinimum ( int min );
}

void RobotWidget::printAbsPos()
{
  Robot* r = global_Project->getActiveScene()->getActiveRobot();
  
  unsigned int joint = m_ui->spinBoxPrintAbsPos->value();
  
  if( joint <= r->getNumberOfJoints() ) {
    cout << "Print Matrix of joint (" << r->getJoint(joint)->getName() ;
    cout << ") for robot " << r->getName() << endl;
    cout << r->getJoint(joint)->getMatrixPos().matrix() << endl;
  }
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
        //cout << "Env::ObjectToCarry  is "<< m_FreeFlyers[i-1] << endl;
        ENV.setString(Env::ObjectToCarry,m_FreeFlyers[i-1]);
    }
}

//void RobotWidget::SetObjectToCarry()
//{
//#ifdef LIGHT_PLANNER
//  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
//  
//  if(m_FreeFlyers.size() > 0)
//  {
//    double radius = 1.5;
//    //take only x and y composantes of the base
//    double dof[2][2];
//    for(int i = 0; i < 2; i++){
//      dof[i][0] = p3d_jnt_get_dof(robotPt->joints[1], i) - radius;
//      dof[i][1] = p3d_jnt_get_dof(robotPt->joints[1], i) + radius;
//    }
//    for(int i = 0; i < 2; i++){
//      p3d_jnt_set_dof_rand_bounds(robotPt->curObjectJnt, i, dof[i][0], dof[i][1]);
//    }
//  }
//  else
//  {
//    cout << "Setting Dist of Joint : " << robotPt->joints[6]->name << endl;
//    cout << "To Joint : "  << robotPt->joints[7]->name << endl;
//    cout << robotPt->joints[7]->dist << "  Takes "  << robotPt->joints[6]->dist << endl;
//    
//    robotPt->joints[7]->dist = robotPt->joints[6]->dist;
//  }
//#endif
//}

void RobotWidget::GrabObject()
{
  if( GLOBAL_AGENTS == NULL ) 
  {
    GLOBAL_AGENTS = hri_create_agents();
  }
  
  Robot* robot = global_Project->getActiveScene()->getRobotByNameContaining("_ROBOT");
  
  int armId = ENV.getInt( Env::currentArmId );
  string robot_name = robot->getName();
  string object_name = ENV.getString(Env::ObjectToCarry).toStdString();
  cout << "Robot " << robot_name << " grasp object " << object_name;
  cout << " with arm " << armId << endl;
  
  HRI_AGENT* agent = hri_get_agent_by_name( GLOBAL_AGENTS, robot_name.c_str() );
  
  if( agent ) {
    hri_agent_is_grasping_obj( agent, true , object_name.c_str() , armId );
  }
  else {
    cout << "Agent not found!!!" << endl;
  }
}

void RobotWidget::ReleaseObject()
{
  if( GLOBAL_AGENTS == NULL ) 
  {
    GLOBAL_AGENTS = hri_create_agents();
  }
  
  Robot* robot = global_Project->getActiveScene()->getRobotByNameContaining("_ROBOT");
  
  int armId = ENV.getInt( Env::currentArmId );
  string robot_name = robot->getName();
  cout << "Robot " << robot_name << " release object with arm " << armId << endl;
  
  HRI_AGENT* agent = hri_get_agent_by_name( GLOBAL_AGENTS, robot_name.c_str() );
  
  if( agent ) {
    hri_agent_is_grasping_obj( agent, false, NULL, armId );
  }
  else {
    cout << "Agent not found!!!" << endl;
  }
}

void RobotWidget::SetBounds()
{
  Robot* robot = global_Project->getActiveScene()->getRobotByNameContaining("_ROBOT");
  
  confPtr_t q = robot->getCurrentPos();
  
  double dist = 2.3; // 4.5 metres 
  
  double limits[6];
  
  limits[0] = (*q)[6] - dist;
  limits[1] = (*q)[6] + dist;
  limits[2] = (*q)[7] - dist;
  limits[3] = (*q)[7] + dist;
  limits[4] = 0;
  limits[5] = 2.0;
    
  for ( int j=2; j<int(robot->getNumberOfJoints()); j++ )
  {
    if( robot->getJoint(j)->getJointStruct()->type == P3D_FREEFLYER )
    {
      cout << "set joint : " << robot->getJoint(j)->getName() << " bounds" << endl;
      
      for ( int k=0; k<3; k++ )
      {
        p3d_jnt_set_dof_bounds_deg (      robot->getRobotStruct()->joints[j], k, limits[2*k], limits[2*k+1] );
        p3d_jnt_set_dof_rand_bounds_deg ( robot->getRobotStruct()->joints[j], k, limits[2*k], limits[2*k+1] );
      }
      
      robot->getRobotStruct()->joints[j]->dist = 0.10;
    }
  }
}

void RobotWidget::printCurrentPos()
{
  Robot* currRobot = global_Project->getActiveScene()->getActiveRobot();
  currRobot->getCurrentPos()->print(true);
  
  HRICS::printPr2Config();
}

#ifdef LIGHT_PLANNER
static bool switch_cart_mode=false;

void RobotWidget::switchFKIK()
{
  traj_optim_switch_cartesian_mode( switch_cart_mode );
  switch_cart_mode = !switch_cart_mode;
}
#endif

void RobotWidget::printPQPColPair()
{
    Robot* currRobot = global_Project->getActiveScene()->getActiveRobot();

    if(  currRobot->getCurrentPos()->isInCollision() )
    {
        cout << "print collision pair:" << endl;
        p3d_print_col_pair();
    }
    else
        cout << "not colliding" << endl;
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
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// ARM MANIPULATION GUI
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
// The Manip namespace holds the function
// to call the Manipulation Planner
// ------------------------------------------------------------------------------
//#ifdef MULTILOCALPATH
namespace Manip
{
ManipulationPlanner* manipulation = NULL;

bool firstRun = true;
bool isCartesianMode = false;

int currentIdTest=0;

MANIPULATION_TASK_TYPE_STR Phase;

void runCurrentTest()
{
    global_manipPlanTest->runTest(currentIdTest);
}

//ARM_FREE  /*!< move the arm from a free configuration (in the air) to another free configuration */
//ARM_PICK_GOTO   /*!< move the arm from a free configuration to a grasping configuration of the object placed on a support */
//ARM_TAKE_TO_FREE   /*!< move the arm from a grasping configuration (of the object placed on a support) to a free configuration */
//ARM_TAKE_TO_PLACE   /*!< move the arm from a grasping configuration to a configuration with the same grasp but a different object placement */
//ARM_PLACE_FROM_FREE  /*!< move the arm from a free configuration to a placement configuration */
//ARM_EXTRACT  /*!< move the arm over Z axis to escape from collision */
//ARM_ESCAPE_OBJECT  /*!< move the arm to escape from a placed object */

//! Main function for launching the manipulation planner
//! it is based on the testing class
//! it reads the Cartesian mode variable to determine
//! which type of method will be used (it is called from the planner thread)
void runManipulation()
{
  if(Manip::isCartesianMode)
  {
    //for(unsigned int i=0; i < rob->armManipulationData->size(); i++)
    global_manipPlanTest->getManipulationPlanner()->setArmCartesian(0, true);
  }
  else
  {
    //for(unsigned int i=0; i < rob->armManipulationData->size(); i++)
    global_manipPlanTest->getManipulationPlanner()->setArmCartesian(0, false);
  }
  
  switch (Phase)
  {
    case ARM_FREE :
    {
      cout << "ARM_FREE" << endl;
      global_manipPlanTest->setInitConfiguration(qInit->getConfigStructCopy());
      global_manipPlanTest->setGoalConfiguration(qGoal->getConfigStructCopy());
      
      global_manipPlanTest->runTest(1);
    }
      break;
      
    case ARM_PICK_GOTO :
    {
      cout << "ARM_PICK_GOTO" << endl;
      global_manipPlanTest->setInitConfiguration(qInit->getConfigStructCopy());
      global_manipPlanTest->setGoalConfiguration(qGoal->getConfigStructCopy());
      
      global_manipPlanTest->runTest(2);
    }
      break;
      
    case ARM_TAKE_TO_FREE :
    {
      cout << "ARM_TAKE_TO_FREE" << endl;
      global_manipPlanTest->setInitConfiguration(qInit->getConfigStructCopy());
      global_manipPlanTest->setGoalConfiguration(qGoal->getConfigStructCopy());
      
      global_manipPlanTest->runTest(3);
    }
      break;
      
    case ARM_TAKE_TO_PLACE :
    {
      cout << "ARM_TAKE_TO_PLACE" << endl;
      global_manipPlanTest->setInitConfiguration(qInit->getConfigStructCopy());
      global_manipPlanTest->setGoalConfiguration(qGoal->getConfigStructCopy());
      
      global_manipPlanTest->runTest(4);
    }
      break;
      
    case ARM_ESCAPE_OBJECT :
    {
      cout << "ARM_ESCAPE_OBJECT" << endl;
      global_manipPlanTest->setInitConfiguration(qInit->getConfigStructCopy());
      global_manipPlanTest->setGoalConfiguration(qGoal->getConfigStructCopy());
      
      global_manipPlanTest->runTest(7);
    }
      break;
      
      //      case Manip::rePlanning :
      //      {
      //        p3d_vector3 otp;
      //        otp[0] = 4.250;
      //        otp[1] = -2.60;
      //        otp[2] = 1.000;
      //
      //        SM_TRAJ traj;
      //
      //        int id_localpath;
      //        const double t_rep = 0.0; // in second
      //        const double tau = 0.0;
      //        p3d_getQSwitchIDFromMidCVS(tau, t_rep, &id_localpath);
      //
      //        global_manipPlanTest->getManipulationPlanner()->armReplan(otp,id_localpath,traj);
      //      }
      //        break;
      
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
    
    global_manipPlanTest->getManipulationPlanner()->planNavigation(rob->ROBOT_POS, rob->ROBOT_GOTO, false ,confs, smTrajs);
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
//#endif

// ------------------------------------------------------------------------------
// The Manip Buttons
// to call the Manipulation Planner
// ------------------------------------------------------------------------------
void RobotWidget::initManipulation()
{
    connect(m_ui->checkBoxIsDebugManip, SIGNAL(toggled(bool)),              this,SLOT(isDebugManip(bool)));
    connect(m_ui->checkBoxIsCartesianMode, SIGNAL(toggled(bool)),           this,SLOT(isCartesianMode(bool)));
    connect(m_ui->checkBoxOnlySampleRZ, SIGNAL(toggled(bool)),              this,SLOT(isOnlySamplingRZ(bool)));
  

    connect(m_ui->pushButtonSetStart, SIGNAL(clicked()),                    this,SLOT(setRobotAtInitConfig()));
    connect(m_ui->pushButtonSetGoal, SIGNAL(clicked()),                     this,SLOT(setRobotAtGoalConfig()));
    connect(m_ui->pushButtonStoreStart, SIGNAL(clicked()),                  this,SLOT(setInitConfigAtCurrent()));
    connect(m_ui->pushButtonStoreGoal, SIGNAL(clicked()),                   this,SLOT(setGoalConfigAtCurrent()));
    connect(m_ui->pushButtonSetOpenConfig, SIGNAL(clicked()),               this,SLOT(setRobotAtOpenConfig()));

    connect(m_ui->pushButtonResetManipulationData,SIGNAL(clicked()),        this,SLOT(resetManipulationData()));
    connect(m_ui->pushButtonRunTest,SIGNAL(clicked()),                      this,SLOT(runManipTest()));

    connect(m_ui->pushButtonArmFree,SIGNAL(clicked()),                      this,SLOT(armFree()));
    connect(m_ui->pushButtonArmPickGoto,SIGNAL(clicked()),                  this,SLOT(armPickGoto()));
    connect(m_ui->pushButtonArmTakeToFree,SIGNAL(clicked()),                this,SLOT(armTakeToFree()));
    connect(m_ui->pushButtonArmTakeToPlace,SIGNAL(clicked()),               this,SLOT(armTakeToPlace()));
    connect(m_ui->pushButtonArmPlaceFromFree,SIGNAL(clicked()),             this,SLOT(armPlaceFromFree()));
    connect(m_ui->pushButtonArmExtract,SIGNAL(clicked()),                   this,SLOT(armExtract()));
    connect(m_ui->pushButtonArmEscape,SIGNAL(clicked()),                    this,SLOT(armEscapeObject()));

    connect(m_ui->pushButtonReplanning,SIGNAL(clicked()),                   this,SLOT(armReplanTask()));
    connect(m_ui->pushButtonLoadWorkspaceFile,SIGNAL(clicked()),            this,SLOT(loadWorkspace()));
    connect(m_ui->pushButtonOptimizeRedundantCost,SIGNAL(clicked()),        this,SLOT(optimizeRedundantCost()));

    connect(this, SIGNAL(selectedPlanner(QString)),
            global_plannerHandler, SLOT(startPlanner(QString)));

    initObjectSupportAndPlacementCombo();

    QString text("<FONT COLOR=Red>Unitizialized</Font>");
    m_ui->labelManipRobotName->setText(text);
  
  
  // X,Y,Z Place!!!
  vector<double> envSize = global_Project->getActiveScene()->getBounds();
	
	m_ui->doubleSpinBoxXPlace->setMinimum(envSize[0]);
	m_ui->doubleSpinBoxXPlace->setMaximum(envSize[1]);
	
  m_ui->doubleSpinBoxYPlace->setMinimum(envSize[2]);
	m_ui->doubleSpinBoxYPlace->setMaximum(envSize[3]);
  
	m_ui->doubleSpinBoxZPlace->setMinimum(envSize[4]);
	m_ui->doubleSpinBoxZPlace->setMaximum(envSize[5]);
  
  // To free point sliders sliders
  new QtShiva::SpinBoxSliderConnector(
  this, m_ui->doubleSpinBoxXPlace, m_ui->horizontalSliderXPlace );
  new QtShiva::SpinBoxSliderConnector(
  this, m_ui->doubleSpinBoxYPlace, m_ui->horizontalSliderYPlace );
  new QtShiva::SpinBoxSliderConnector(
  this, m_ui->doubleSpinBoxZPlace, m_ui->horizontalSliderZPlace );
  
  m_ui->doubleSpinBoxXPlace->setValue((envSize[1]+envSize[0])/2);
	m_ui->doubleSpinBoxYPlace->setValue((envSize[3]+envSize[2])/2);
	m_ui->doubleSpinBoxZPlace->setValue((envSize[5]+envSize[4])/2);
  
  connect(m_ui->checkBoxToFreePoint, SIGNAL(toggled(bool)), this,SLOT(isToFreePoint(bool)));
  
  // Get Visball Position
  connect(m_ui->pushButtonGetVisballPos, SIGNAL(clicked()), this,SLOT(getVisballPos()));
  
  connect(m_ui->checkBoxIsUsingMobileBase, SIGNAL(toggled(bool)), this,SLOT(isUsingMobileBase(bool)));
  
  setGroupBoxDisabled(true);
}

void RobotWidget::setGroupBoxDisabled(bool disable)
{
  m_ui->groupBoxTest->setDisabled( disable );
  m_ui->groupBoxArmPlanTask->setDisabled( disable );
  m_ui->groupBoxSettingRobotConf->setDisabled( disable );
  m_ui->groupBoxStoringRobotConf->setDisabled( disable );
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

void RobotWidget::setRobotAtInitConfig()
{
    if( !global_manipPlanTest )
    {
        cout << "global_manipPlanTest is not initialized!!!" << endl;
        return;
    }

    cout << "Set to qInit configuration" << endl;
    qInit->getRobot()->setAndUpdate( *qInit );
    m_mainWindow->drawAllWinActive();
}

void RobotWidget::setRobotAtGoalConfig()
{
    if( !global_manipPlanTest )
    {
        cout << "global_manipPlanTest is not initialized!!!" << endl;
        return;
    }

    cout << "Set to qGoal configuration" << endl;
    qGoal->getRobot()->setAndUpdate( *qGoal );
    m_mainWindow->drawAllWinActive();
}

void RobotWidget::setRobotAtOpenConfig()
{
    if( !global_manipPlanTest )
    {
        cout << "global_manipPlanTest is not initialized!!!" << endl;
        return;
    }

    cout << "Set to qOpen configuration" << endl;
    qOpen->getRobot()->setAndUpdate( *qOpen , true );
    m_mainWindow->drawAllWinActive();
}

void RobotWidget::setInitConfigAtCurrent()
{
    if( !global_manipPlanTest )
    {
        cout << "global_manipPlanTest is not initialized!!!" << endl;
        return;
    }

    cout << "Set to qInit configuration to current" << endl;
    qInit = qInit->getRobot()->getCurrentPos();
}

void RobotWidget::setGoalConfigAtCurrent()
{  
    if( !global_manipPlanTest )
    {
        cout << "global_manipPlanTest is not initialized!!!" << endl;
        return;
    }

    cout << "Set to qGoal configuration to current" << endl;
    qGoal = qGoal->getRobot()->getCurrentPos();
}

std::string RobotWidget::getNameOfFreeFlyerFromIndex(int id)
{
    string name("");

    if((id >= 0) && (id < int(m_FreeFlyers.size())))
    {
        if( id == 0 )
            name = "No Object";
        else
            name = m_FreeFlyers[id-1].toStdString();

        cout << "Set object name to : " << name << endl;
    }

    return name;
}


//#ifdef MULTILOCALPATH
void RobotWidget::objectNameChanged(int id)
{
    string name = getNameOfFreeFlyerFromIndex( id );

    if( name == "")
    {
        cout << "Object doesn't exist" << endl;
        return;
    }

    if (!global_manipPlanTest) resetManipulationData();

    if( name == "No Object" )
        global_manipPlanTest->resetObject();
    else
        global_manipPlanTest->setObject( name );
}

void RobotWidget::placementNameChanged(int id)
{
    string name = getNameOfFreeFlyerFromIndex( id );

    if( name == "")
    {
        cout << "Placement doesn't exist" << endl;
        return;
    }

    if (!global_manipPlanTest) resetManipulationData();

    if( name == "No Object" )
        global_manipPlanTest->resetPlacement();
    else
        global_manipPlanTest->setPlacement( name );
}

void RobotWidget::supportNameChanged(int id)
{
    string name = getNameOfFreeFlyerFromIndex( id );

    if( name == "")
    {
        cout << "Support doesn't exist" << endl;
        return;
    }

    if (!global_manipPlanTest) resetManipulationData();

    if( name == "No Object" )
        global_manipPlanTest->resetSupport();
    else
        global_manipPlanTest->setSupport( name );
}


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

void RobotWidget::isUsingMobileBase(bool value)
{
  if(global_manipPlanTest)
  {
    global_manipPlanTest->getManipPlanner()->setUseBaseMotion(value);
  }
  else {
    cout << "Manip test is NULL" << endl;
  }
}

// ---------------------------------------------------------------------
// ---------------------------------------------------------------------
void RobotWidget::isOnlySamplingRZ(bool value)
{
  if(global_manipPlanTest)
  {
    global_manipPlanTest->getManipPlanner()->setUseBaseMotion(value);
  }
  else {
    cout << "Manip test is NULL" << endl;
  }
}

void RobotWidget::isToFreePoint(bool value)
{
  if( global_manipPlanTest != NULL )
  {
    if( value )
    {
      vector<double> point(6,P3D_HUGE);
      point[0] = m_ui->doubleSpinBoxXPlace->value();
      point[1] = m_ui->doubleSpinBoxYPlace->value();
      point[2] = m_ui->doubleSpinBoxZPlace->value();
      
      global_manipPlanTest->setToPoint( point );
    }
    else
      global_manipPlanTest->resetToPoint();
  }
  else {
    cout << "Manip test is NULL" << endl;
  }
}

void RobotWidget::getVisballPos()
{
  if( global_manipPlanTest != NULL )
  {
    Robot* rob = global_Project->getActiveScene()->getRobotByName("VISBALL_INTERNAL");
    
    if( rob != NULL )
    {
      cout << "getVisballPos" << endl;
      
      Eigen::Vector3d pos = rob->getJoint(1)->getVectorPos();
      
      m_ui->doubleSpinBoxXPlace->setValue( pos[0] );
      m_ui->doubleSpinBoxYPlace->setValue( pos[1] );
      m_ui->doubleSpinBoxZPlace->setValue( pos[2] );
      
      m_ui->checkBoxToFreePoint->toggle();
      
      vector<double> point(6,P3D_HUGE);
      point[0] = pos[0];
      point[1] = pos[1];
      point[2] = pos[2];
      
      global_manipPlanTest->setToPoint( point );
    }
    else {
      cout << "No robot is named VISBALL_INTERNAL" << endl;
    }
  }
  else {
    cout << "Manip test is NULL" << endl;
  }
}

// ---------------------------------------------------------------------
// ---------------------------------------------------------------------
void RobotWidget::resetManipulationData()
{
  cout << "-----------------------------------------------------" << endl;
  cout << " RobotWidget::resetManipulationData -----------------" << endl;
  
  if (!global_manipPlanTest)
  {
    global_manipPlanTest = new ManipulationTestFunctions( global_ActiveRobotName  );
    
    global_manipPlanTest->resetObject();
    global_manipPlanTest->resetPlacement();
    global_manipPlanTest->resetSupport();
    global_manipPlanTest->resetToPoint();
  }
  
  Robot* rob = global_Project->getActiveScene()->getRobotByNameContaining( global_ActiveRobotName );
  //  Robot* rob = global_Project->getActiveScene()->getRobotByName(rob1->name);
  
  qInit = rob->getInitialPosition();
  qGoal = rob->getGoTo();
  qOpen = shared_ptr<Configuration>( new Configuration( rob, rob->getRobotStruct()->openChainConf ));
  
  rob->setAndUpdate(*qInit);
  
  cout << "global_manipPlanTest->getManipPlanner()->setPlanningMethod" << endl;
  // Set Planning functions
  
  global_manipPlanTest->initManipulationGenom ();
  
  global_manipPlanTest->setInitConfiguration (qInit->getConfigStructCopy());
  global_manipPlanTest->setGoalConfiguration (qGoal->getConfigStructCopy());
  
  global_manipPlanTest->getManipPlanner()->setPlanningMethod( p3d_planner_function );
  global_manipPlanTest->getManipPlanner()->setSmoothingMethod( p3d_smoothing_function );
  global_manipPlanTest->getManipPlanner()->setCleanningRoadmaps( false );
  //global_manipPlanTest->getManipPlanner()->setReplanningMethod( replanning_Function );
  
  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  
  m_mainWindow->drawAllWinActive();
  
  std::string t1 = "<FONT COLOR=Green>";
  std::string t2 = "</FONT>";
  t1 = t1 + rob->getName() + t2;
  
  m_ui->labelManipRobotName->setText(QString(t1.c_str()));
  
  setGroupBoxDisabled(false);
}

void RobotWidget::optimizeRedundantCost()
{
    //  p3d_rob* robot = global_manipPlanTest->getManipulationPlanner->robot();
    //  int redJnt =
    //  double cost = optimizeRedundentJointConfigCost( robot, redJntId, q, objectPos, tAtt, grasp, armId, 10000 );
    //  cout << "Cost" << cost << endl;
}

//ARM_FREE  /*!< move the arm from a free configuration (in the air) to another free configuration */
//ARM_PICK_GOTO   /*!< move the arm from a free configuration to a grasping configuration of the object placed on a support */
//ARM_TAKE_TO_FREE   /*!< move the arm from a grasping configuration (of the object placed on a support) to a free configuration */
//ARM_TAKE_TO_PLACE   /*!< move the arm from a grasping configuration to a configuration with the same grasp but a different object placement */
//ARM_PLACE_FROM_FREE  /*!< move the arm from a free configuration to a placement configuration */
//ARM_EXTRACT  /*!< move the arm over Z axis to escape from collision */
//ARM_ESCAPE_OBJECT  /*!< move the arm to escape from a placed object */

void RobotWidget::callToManipulationPlanner()
{
    if ( global_manipPlanTest == NULL )
    {
        resetManipulationData();
    }

    m_mainWindow->isPlanning();
  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
  emit(selectedPlanner(QString("Manipulation")));
}

void RobotWidget::armFree()
{
  cout << "Manipulation : free" << endl;
  
  double center[2];

  center[0] = ((*qInit)[6] + (*qGoal)[6]) / 2;
  center[1] = ((*qInit)[7] + (*qGoal)[7]) / 2;
  
  double dist = sqrt( pow( (*qInit)[6]-(*qGoal)[6] ,2)  + pow( (*qInit)[7]-(*qGoal)[7], 2 ));
  
  double limits[6];
  
  limits[0] = center[0] - 2*dist;
  limits[1] = center[0] + 2*dist;
  limits[2] = center[1] - 2*dist;
  limits[3] = center[1] + 2*dist;
  limits[4] = 0;
  limits[5] = 0;
  
  p3d_change_ff_translation_bounds( qInit->getRobot()->getRobotStruct(), limits );
  
  Manip::Phase = ARM_FREE;
  callToManipulationPlanner();
}

void RobotWidget::armPickGoto()
{
    cout << "Manipulation : pick goto" << endl;
    Manip::Phase = ARM_PICK_GOTO;
    callToManipulationPlanner();
}

void RobotWidget::armTakeToFree()
{
    cout << "Manipulation : take to free" << endl;
    Manip::Phase = ARM_TAKE_TO_FREE;
    callToManipulationPlanner();
}

void RobotWidget::armTakeToPlace()
{
    cout << "Manipulation : take to place" << endl;
    Manip::Phase = ARM_TAKE_TO_PLACE;
    callToManipulationPlanner();
}

void RobotWidget::armPlaceFromFree()
{
    cout << "Manipulation : place from free" << endl;
    Manip::Phase = ARM_PLACE_FROM_FREE;
    callToManipulationPlanner();
}

void RobotWidget::armExtract()
{
    cout << "Manipulation : Extract" << endl;
    Manip::Phase = ARM_EXTRACT;
    callToManipulationPlanner();
}

void RobotWidget::armEscapeObject()
{
    cout << "Manipulation : Escape Object" << endl;
    Manip::Phase = ARM_ESCAPE_OBJECT;
    callToManipulationPlanner();
}

void RobotWidget::armReplanTask()
{
    //  cout << "Manipulation : rePlanning" << endl;
    //
    //  if (!global_manipPlanTest)
    //  {
    //    resetManipulationData();
    //  }
    //
    //  Manip::Phase = Manip::rePlanning;
    //  m_mainWindow->isPlanning();
    //
    //  global_manipPlanTest->setDebugMode( m_ui->checkBoxIsDebugManip->isChecked() );
    //
    //  emit(selectedPlanner(QString("Manipulation")));
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
//#endif // MULTILOCALPATH

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

void RobotWidget::on_pushButtonSaveCurConf_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save configuration to XML file"));
    if (!fileName.isEmpty())
    {
        p3d_rob* robot = p3d_get_robot_by_name(global_ActiveRobotName.c_str());
        configPt q = p3d_get_robot_config( robot );
        p3d_saveConfigTofile(robot,q,fileName.toAscii().data());
    }
}

void RobotWidget::on_pushButtonLoadConf_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Load configuration from XML file"));
    if (!fileName.isEmpty())
    {
        p3d_rob* robot = p3d_get_robot_by_name(global_ActiveRobotName.c_str());
        configPt q = p3d_loadConfigFromfile(robot,fileName.toAscii().data());
        p3d_set_robot_config(robot,q);
    }

}
