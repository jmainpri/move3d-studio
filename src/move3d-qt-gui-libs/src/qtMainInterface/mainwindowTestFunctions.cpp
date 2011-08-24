/*
 *  mainwindowTestFunctions.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/08/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "mainwindowTestFunctions.hpp"
#include "mainwindowGenerated.hpp"

#include <iostream>
#include <tr1/memory>
#include <vector>

#if defined( MOVE3D_CORE ) 
#include "testModel.hpp"
#include "SaveContext.hpp"
#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
//Warning contains boos function that conlicts with Qt
//#include "API/Trajectory/RoboptimTrajectory.h"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "API/Grids/GridToGraph/gridtograph.hpp"
#include "API/Search/GraphState.hpp"
#include "API/Grids/PointCloud.hpp"
#include "API/Grids/BaseGrid.hpp"
#include "API/Grids/TwoDGrid.hpp"
#include "planner/Greedy/CostMapRRTs.hpp"
#include "planner/cost_space.hpp"
#endif

#ifdef HRI_COSTSPACE
#include "ui_qtHrics.h"
#include "HRI_costspace/HRICS_costspace.hpp"
#endif

#ifdef HRI_PLANNER
#include "P3d-pkg.h"
#include <hri/hri.h>
#endif

using namespace std;
using namespace tr1;

MainWindowTestFunctions::MainWindowTestFunctions(MainWindow* MainWinPt) : m_mainWindow(MainWinPt)
{
  cout << "Init test Functions" << endl;
  
	connect(m_mainWindow->Ui()->pushButtonTest1,SIGNAL(clicked()),this,SLOT(test1()));
	connect(m_mainWindow->Ui()->pushButtonTest2,SIGNAL(clicked()),this,SLOT(test2()));
	connect(m_mainWindow->Ui()->pushButtonTest3,SIGNAL(clicked()),this,SLOT(test3()));	
}

p3d_matrix4 mKinectToOrigin;

void MainWindowTestFunctions::test1()
{
/**
  Scene* sc = global_Project->getActiveScene();
  Robot* rob = sc->getRobotByName("JIDOKUKA_ROBOT");
  
  configPt q1,q2;
  q1 = p3d_get_robot_config(rob->getRobotStruct());
  q2 = p3d_get_robot_config(rob->getRobotStruct());
  
  q1[0] = 0;
  q1[1] = 0;
  q1[2] = 0;
  q1[3] = 0;
  q1[4] = 0;
  q1[5] = 0;
  q1[6] = 3.54;
  q1[7] = -2.163;
  q1[8] = 0;
  q1[9] = 0;
  q1[10] = 0;
  q1[11] = 0;
  q1[12] = 0;
  q1[13] = -0.03264;
  q1[14] = 1.869;
  q1[15] = -0.598;
  q1[16] = 0.04165;
  q1[17] = 1.96;
  q1[18] = -2.337;
  q1[19] = -1.51;
  q1[20] = -2.86;
  q1[21] = 0.8578;
  q1[22] = -0.1801;
  q1[23] = 0.3808;
  q1[24] = 0;
  q1[25] = 0;
  q1[26] = 0;
  q1[27] = 0.475;
  q1[28] = 0.02554;
  q1[29] = 0.02554;
  q1[30] = 0;
  q1[31] = 0.5153;
  q1[32] = 0;
  q1[33] = 0;
  q1[34] = 0;
  q1[35] = 0.4952;
  q1[36] = 0;
  q1[37] = 0;
  q1[38] = 4.894;
  q1[39] = 5.769;
  q1[40] = 4.388;
  q1[41] = -2.337;
  q1[42] = 0.9222;
  q1[43] = 0.6171;
  q1[44] = 0.1507;
  q1[45] = -1.029;
  
  q2[0] = 0;
  q2[1] = 0;
  q2[2] = 0;
  q2[3] = 0;
  q2[4] = 0;
  q2[5] = 0;
  q2[6] = 3.54;
  q2[7] = -2.163;
  q2[8] = 0;
  q2[9] = 0;
  q2[10] = 0;
  q2[11] = 0;
  q2[12] = 0;
  q2[13] = -0.03264;
  q2[14] = 2.199;
  q2[15] = 0.1433;
  q2[16] = -0.195;
  q2[17] = 1.452;
  q2[18] = -0.6663;
  q2[19] = -0.8717;
  q2[20] = 0.6909;
  q2[21] = 0.8578;
  q2[22] = -0.1801;
  q2[23] = 0.3808;
  q2[24] = 0;
  q2[25] = 0;
  q2[26] = 0;
  q2[27] = 0.475;
  q2[28] = 0.02554;
  q2[29] = 0.02554;
  q2[30] = 0;
  q2[31] = 0.5153;
  q2[32] = 0;
  q2[33] = 0;
  q2[34] = 0;
  q2[35] = 0.4952;
  q2[36] = 0;
  q2[37] = 0;
  q2[38] = 1.636;
  q2[39] = 7.337;
  q2[40] = 4.25;
  q2[41] = -2.6;
  q2[42] = 1;
  q2[43] = -0;
  q2[44] = 0;
  q2[45] = -0.013;
  
  shared_ptr<Configuration> q_source( new Configuration( rob,q1 ));
  shared_ptr<Configuration> q_target( new Configuration( rob,q2 ));
  
  rob->activateCcConstraint();
  
  LocalPath* pathPt = new LocalPath(q_source,q_target);
  
  p3d_multilocalpath_switch_to_linear_groups (rob->getRobotStruct());
  pathPt->getLocalpathStruct();
  
  if( pathPt->isValid() )
  {
    cout << "LocalPath valid" << endl;
  }
  
  vector<LocalPath*> paths;
  paths.push_back( pathPt );
  
  
  API::Trajectory traj(rob);
  traj.replacePortion((unsigned int)0,(unsigned int)0,paths);
  traj.replaceP3dTraj();
 */
 
  /*
  global_FramesToDraw.clear();
  
  Scene* sce = global_Project->getActiveScene();
  Robot* rob = sce->getRobotByNameContaining("PR2");
  
//  global_FramesToDraw.push_back( rob->getJoint(1)->getAbsPos() );
  
  p3d_matrix4 kinect_to_head;
  double baseX = 0.05;
  double baseY = -0.20;
  double baseZ = 0.00;
  
  kinect_to_head[0][0] = 0.0;  kinect_to_head[0][1] = 0.0;  kinect_to_head[0][2] = 1.0;  kinect_to_head[0][3] = baseX;
  kinect_to_head[1][0] = 0.0;  kinect_to_head[1][1] = -1.0; kinect_to_head[1][2] = 0.0;  kinect_to_head[1][3] = baseY;
  kinect_to_head[2][0] = 1.0;  kinect_to_head[2][1] = 0.0;  kinect_to_head[2][2] = 0.0;  kinect_to_head[2][3] = baseZ;
  kinect_to_head[3][0] = 0.0;  kinect_to_head[3][1] = 0.0;  kinect_to_head[3][2] = 0.0;  kinect_to_head[3][3] = 1.0;
  
  p3d_mat4Mult( *rob->getJoint(4)->getAbsPos() , kinect_to_head , mKinectToOrigin );

  global_FramesToDraw.push_back( &mKinectToOrigin );
  global_FramesToDraw.push_back( rob->getJoint(4)->getAbsPos() );
  
  cout << "Add 2 Matrix to draw" << endl;
   */
  /**
  Scene* sce = global_Project->getActiveScene();
  Robot* rob = sce->getRobotByNameContaining("HERAKLES");
  
  configPt gRHHumanQ = p3d_alloc_config( rob->getRobotStruct() );
  
  gRHHumanQ[18] = 1.39626;
  gRHHumanQ[19] = 0.10472;
  gRHHumanQ[20] = -0.346273;
  gRHHumanQ[21] = 0;
  gRHHumanQ[22] = 0.174533;
  gRHHumanQ[23] = 0;
  gRHHumanQ[24] = 0;
  gRHHumanQ[25] = 0;
  gRHHumanQ[26] = 0;
  gRHHumanQ[27] = -1.39626;
  gRHHumanQ[28] = -0.10472;
  gRHHumanQ[29] = -0.174533;
  gRHHumanQ[30] = 0;
  gRHHumanQ[31] = -0.174533;
  gRHHumanQ[32] = 0;
  gRHHumanQ[33] = 0;
  
  shared_ptr<Configuration> q( new Configuration(rob,gRHHumanQ) );
  
  rob->setAndUpdate(*q);
   */
  
  global_Project->getActiveScene()->setActiveRobot("PR2_ROBOT");
  
  HRI_AGENTS* agents = hri_create_agents();
  HRI_AGENT* pr2 = hri_get_one_agent_of_type(agents, HRI_PR2);
  
  p3d_matrix4 mat_ID = {{1, 0, 0, 0.10}, {0, 1, 0, 0.10}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  
  hri_agent_is_grasping_obj_at_center( pr2, "VISBALL" , 0 , mat_ID);
}

void MainWindowTestFunctions::test2()
{
	using namespace std;
	
	cout << "------------------- test2 -------------------" << endl;
	cout << "---------------------------------------------" << endl;
	
#ifdef HRI_COSTSPACE
	ENV.setBool(Env::drawGraph,false);
	ENV.setBool(Env::drawTraj,true);
	ENV.setBool(Env::biDir,true);
	ENV.setBool(Env::costStarRRT,false);
	ENV.setBool(Env::useBoxDist,true);
	ENV.setBool(Env::useBallDist,false);
	ENV.setBool(Env::costStarRRT,false);
	ENV.setDouble(Env::extensionStep,5.0);
	
	ENV.setDouble(Env::Knatural,0.0);
	ENV.setDouble(Env::Kreachable,0.0);
	ENV.setDouble(Env::CellSize,0.2);
	
	if (HRICS_MotionPL) 
	{
		delete HRICS_MotionPL;
		HRICS_MotionPL= NULL;
	}
	
	HRICS_MotionPL = new HRICS::Workspace;
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initGrid();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initDistance();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initVisibility();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initNatural();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initReachable();
	
	HRICS_activeDist = HRICS_MotionPL->getDistance();
	API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid();
	
	PointsToDraw = new PointCloud;
	
	m_mainWindow->Ui()->tabCost->getHriWidget()->Ui()->HRICSPlanner->setDisabled(false);
	m_mainWindow->Ui()->tabCost->getHriWidget()->Ui()->pushButtonMakeGrid->setDisabled(true);
	m_mainWindow->Ui()->tabCost->getHriWidget()->Ui()->pushButtonDeleteGrid->setDisabled(false);
	m_mainWindow->Ui()->tabCost->setCostFunction("costHRI");
	
	ENV.setBool(Env::HRIPlannerWS,true);
	ENV.setBool(Env::enableHri,true);
	ENV.setBool(Env::isCostSpace,true);

	ENV.setDouble(Env::zone_size,1.0);
	
	ENV.setBool(Env::drawDistance,false);
	//ENV.setBool(Env::drawGrid,true);
	m_mainWindow->drawAllWinActive();
	
	//	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid()->computeVectorField();
	//	ENV.setBool(Env::drawGrid,true);
	//	ENV.setBool(Env::drawVectorField,true);
	//	drawAllWinActive();
#endif
}

/*roboptim::RoboptimTrajectory OptimTraj;
 OptimTraj.run_CostMap();*/

//boost_test_1();
//boost_test_2();
//boost_test_3();
///boost_test_4();

//	Graph* G = new Graph(XYZ_GRAPH);

Testthread::Testthread(QObject* parent) :
QThread(parent)
{
	
}

void Testthread::run()
{	
//	try 
//	{
//		Scene* sc = global_Project->getActiveScene();
//		
//		Robot* rob = sc->getRobotByNameContaining("ROBOT");
//		sc->setActiveRobot(rob->getName());
//		
//		ENV.setBool(Env::isRunning,true);
//		
//		API_activeGraph = new Graph(rob);
//		CostmapPlanner rrt(rob,API_activeGraph);
//		rrt.init();
//		rrt.run();
//		
//		ENV.setBool(Env::isRunning,false);
//		
//		cout << "Ends Planner Thread" << endl;
//	}
//	catch (string str) 
//	{
//		cout << "Problem in Testthread : " << str << endl;
//	}
//	catch (...) 
//	{
//		cout << "Problem in Testthread" << endl;
//	}
	cout << "GLOBAL_AGENTS created" << endl;
	
//	p3d_rob * robot;
	p3d_rob * object;
	p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	int i;
	//ENV.setBool(Env::tRrtComputeGradient,true);
	//ENV.setDouble(Env::extensionStep,3.0);
	//global_costSpace->setCost("costMap2D");
	GLOBAL_AGENTS = hri_create_agents();
	for(i=0; i<env->nr; i++){
		if( strcasestr(env->robot[i]->name,"trash") ){
			object = env->robot[i];
			continue;
		}    
	}
	
	hri_is_object_visible(GLOBAL_AGENTS->humans[0], object, 50, TRUE, FALSE);
}

void MainWindowTestFunctions::test3()
{
//  cout << "------------------- test3 -------------------" << endl;
//	cout << "---------------------------------------------" << endl;
//  
//	m_mainWindow->isPlanning();
//	Testthread* ptrPlan = new Testthread; ptrPlan->start();
  
  G3D_DRAW_TRACE = !G3D_DRAW_TRACE;
  m_mainWindow->drawAllWinActive();
}
