#include "planner_handler.hpp"

#include <iostream>
#include <iostream>
#include <iomanip>
#include <iosfwd>
#include <sstream>
#include <fstream>
#include <string>
//#include <vector>
#include <set>
#include <map>
#include <list>
#include <utility>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <tr1/memory>

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"

#include "move3d-headless.h"
#include "move3d-gui.h"

#include "qtFormRobot/moverobot.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/sideWidgets/qtRobot.hpp"

#include "API/project.hpp"
#include "API/Search/Dijkstra/dijkstra.hpp"

#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
#include "planner/replanning.hpp"
#endif

#include "utils/MultiRun.hpp"
#include "utils/MultiRun.hpp"

#include "qtLibrary.hpp"
#include <QtCore/QMutexLocker>
#include <QtCore/QTime>

#ifdef ENERGY
#include "bio/BioEnergy/include/Energy-pkg.h"
#endif

#if defined( HRI_COSTSPACE )
#include "HRI_costspace/HRICS_costspace.hpp"
#include "HRI_costspace/HRICS_Workspace.hpp"
#if defined( HRI_PLANNER )
#include "HRI_costspace/HRICS_HAMP.hpp"
#include "HRI_costspace/HRICS_otpmotionpl.hpp"
#endif
#endif

#ifdef LIGHT_PLANNER
#include "../lightPlanner/proto/ManipulationViaConfPlanner.hpp"
#endif

const char *qt_fileName = NULL;

using namespace std;
using namespace tr1;

/*
 *
 *      This file implements a pipe to read out commands from qt and
 *      pass it on to the XForms thread, the main loop runs in a
 *      distinct thread using X11 and XForms doesn't permit qt's thread to act upon X11
 *      at the same time (causing a segmentation fault).
 *
 *      This can be solved by passing the 3D display to Ogre or having the OpenGl in a
 *      Qt window.
 */


//------------------------------------------------------------------------------
//  Callback functions  
//------------------------------------------------------------------------------

void qt_resetGraph()
{
	try 
	{
		if (API_activeGraph) 
		{
			delete API_activeGraph;
			API_activeGraph = NULL;
			cerr << "Delete C++ API Graph" << endl;
		}
		
#ifdef P3D_PLANNER
		if( !p3d_del_graph(XYZ_GRAPH) )
		{
			cerr << "XYZ_GRAPH allready deleted" << endl;
		}
		
		cerr <<  "XYZ_GRAPH = " << XYZ_GRAPH << endl;
		
#endif
	}
	catch (string str) 
	{
		cerr << str << endl;
	}
	catch (...) 
	{
		cerr << "Exeption in qt_resetGraph()" << endl;
	}
}

/**
 * Draw All Win Active
 */
void qt_drawAllWinActive()
{
	g3d_draw_allwin_active();
}

/**
 * Run Diffusion
 */
void qt_runDiffusion()
{
	cout << "Diffusion" << endl;
	
	try 
	{
#ifdef P3D_PLANNER
		p3d_SetStopValue(FALSE);
#endif
		ChronoOn();
		
		int res;
		cout << "ENV.getBool(Env::Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
		if (ENV.getBool(Env::treePlannerIsEST))
		{
#if defined( MOVE3D_CORE )
			res = p3d_run_est(XYZ_GRAPH, fct_stop, fct_draw);
		}
		else
		{
			res = p3d_run_rrt(XYZ_GRAPH, fct_stop, fct_draw);
#endif
		}
		ChronoPrint("");
		ChronoOff();
		
		g3d_draw_allwin_active();
	}
	catch (string str) 
	{
		cerr << "Exeption in run qt_runDiffusion : " << endl;
		cerr << str << endl;
		ENV.setBool(Env::isRunning,false);
	}
	catch (...) 
	{
		cerr << "Exeption in run qt_runDiffusion" << endl;
		ENV.setBool(Env::isRunning,false);
	}
}

/**
 * Run PRM
 */
void qt_runPRM()
{
	try 
	{
#ifdef P3D_PLANNER
		p3d_SetStopValue(FALSE);
#endif
		
		int res;
		int fail;
		
		ChronoOn();
		
		//        cout << "ENV.getInt(Env::PRMType)  = "  << ENV.getInt(Env::PRMType) << endl;
		
		switch(ENV.getInt(Env::PRMType))
		{
#ifdef MOVE3D_CORE
			case 0:
				res = p3d_run_prm(XYZ_GRAPH, &fail, fct_stop, fct_draw);
				break;
			case 1:
				res = p3d_run_vis_prm(XYZ_GRAPH, &fail, fct_stop, fct_draw);
				break;
			case 2:
				res = p3d_run_acr(XYZ_GRAPH, &fail, fct_stop, fct_draw);
				break;
#endif
			default:
				cout << "Error No Other PRM"  << endl;
				ChronoPrint("");
				ChronoOff();
				return;
		}
		
		
		ChronoPrint("");
		ChronoOff();
		
		if (ENV.getBool(Env::expandToGoal))
		{
			if (res)
			{
				if (ENV.getBool(Env::isCostSpace))
				{
#ifdef P3D_PLANNER
					p3d_ExtractBestTraj(XYZ_GRAPH);
#endif
				}
				else
				{
					if (p3d_graph_to_traj(XYZ_ROBOT))
					{
						g3d_add_traj((char*) "Globalsearch",
												 p3d_get_desc_number(P3D_TRAJ));
					}
					else
					{
						printf("Problem during trajectory extraction\n");
					}
				}
				g3d_draw_allwin_active();
			}
		}
	}
	catch (string str) 
	{
		cerr << "Exeption in run qt_runPRM : " << endl;
		cerr << str << endl;
	}
	catch (...) 
	{
		cerr << "Exeption in run qt_runPRM" << endl;
	}
	
	ENV.setBool(Env::isRunning,false);
}


#ifdef MULTILOCALPATH

void qt_runManipTest()
{
  Manip::runCurrentTest();
}

void qt_runManipulation()
{
  Manip::runManipulation();
}

void qt_runNavigation()
{
    Manip::runNavigation();
}



void qt_runReplanning()
{
  Robot* rob =	global_Project->getActiveScene()->getActiveRobot();
  p3d_vector3 otp;
  otp[0] = 4.250;
  otp[1] = -2.60;
  otp[2] = 1.000;
  
  replanning_Function(rob->getRobotStruct(), rob->getRobotStruct()->tcur, otp, 5);
}
#endif

void qt_handover()
{
  //  compute_handOver();
}


#ifdef HRI_PLANNER

void qtRealTimeOtp()
{
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getInputs();
    int tmp = PlanEnv->getInt(PlanParam::env_timeShow);
    PlanEnv->setInt(PlanParam::env_timeShow,0);
    ENV.setBool(Env::drawGraph,false);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->setAsNotSorted();
    PlanEnv->setBool(PlanParam::env_drawHumanModel,true);
    while (PlanEnv->getBool(PlanParam::env_realTime))
    {
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);
//        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getInputs();
        Eigen::Vector3d pos;
        pos[0] = PlanEnv->getDouble(PlanParam::env_futurX);
        pos[1] = PlanEnv->getDouble(PlanParam::env_futurY);
        pos[2] = PlanEnv->getDouble(PlanParam::env_futurRZ);
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setInputs(pos,
                                                                           dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getRobotPos(),
                                                                           PlanEnv->getBool(PlanParam::env_isStanding),
                                                                           PlanEnv->getDouble(PlanParam::env_objectNessecity));

        if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->newComputeOTP())
        {
            dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showBestConf();
            g3d_draw_allwin_active();
            ENV.setBool(Env::drawOTPTraj,true);
        }
    }

    PlanEnv->setBool(PlanParam::env_drawHumanModel,false);
    ENV.setBool(Env::drawGraph,true);
    PlanEnv->setInt(PlanParam::env_timeShow,tmp);
}

void qtOTP()
{
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();

//	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->initGrid();
//	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->simpleComputeBaseAndOTP();
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->setAsNotSorted();
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->newComputeOTP();
	ENV.setBool(Env::drawOTPTraj,true);
  //	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computePR2GIK();
  
}
#endif

#ifdef MULTILOCALPATH
void qt_runStomp()
{
  if( traj_optim_runStomp() )
  {
    cout << "Stomp has run succesfully!!!" << endl;
  }
  else {
    cout << "Stomp fail!!!" << endl;
  }
}

void qt_runChomp()
{
  if( traj_optim_runChomp() )
  {
    cout << "Chomp has run succesfully!!!" << endl;
  }
  else {
    cout << "Chomp fail!!!" << endl;
  }
  
  //ChompTrajectory ( T, 2 );
  //cout << "Warning : CHOMP Not yet implemented" << endl;
}
#endif // MULTILOCALPATH

#ifdef HRI_PLANNER
void qtMultipleOTP()
{
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
	ENV.setBool(Env::drawOTPTraj,true);
}
#endif

static int default_drawtraj_fct_qt_pipe(p3d_rob* robot, p3d_localpath* curLp)
{
	g3d_draw_allwin_active();
	usleep(20000);
	//cout << "ENV.getDouble(Env::showTrajFPS) = " << ENV.getDouble(Env::showTrajFPS) << endl;
	return TRUE;
}

#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
void qt_executeReplanSimu()
{
  if( replan_plan_initial_path() )
  {
    replann_execute_simulation_traj(default_drawtraj_fct_qt_pipe);
  }
	ENV.setBool(Env::isRunning,false);
}
#endif

void qt_showTraj()
{
	p3d_rob *hum_robotPt;
	Robot* rob =	global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");
	if (rob)
	{
		hum_robotPt = rob->getRobotStruct();
	}

#ifdef HRI_PLANNER
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  
	if (PlanEnv->getBool(PlanParam::env_showHumanTraj))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,false);
		g3d_show_tcur_both_rob(robotPt,default_drawtraj_fct_qt_pipe,hum_robotPt,default_drawtraj_fct_qt_pipe);
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showBestConf();
	}
	else
	{
    //		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,false);
		g3d_show_tcur_rob(robotPt,default_drawtraj_fct_qt_pipe);
    //		g3d_show_tcur_rob(hum_robotPt,default_drawtraj_fct_qt_pipe);
    //		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showBestConf();
	}
#endif

	ENV.setBool(Env::isRunning,false);
}

/**
 * Shortcut optimization
 */
void qt_shortCut()
{
	cout << "Random : ShortCut "  << endl;
	//ENV.setBool(Env::isRunning,true);
#ifdef MOVE3D_CORE
	Robot* trajRobot = global_Project->getActiveScene()->getActiveRobot();
	API::Smoothing optimTrj(trajRobot->getCurrentTraj());
	optimTrj.runShortCut(ENV.getInt(Env::nbCostOptimize));
	optimTrj.replaceP3dTraj();
#endif
	g3d_draw_allwin_active();
	ENV.setBool(Env::isRunning,false);
}

/**
 * Deformation optimization
 */
void qt_optimize()
{
	cout << "Random : Deformation "  << endl;
	//ENV.setBool(Env::isRunning,true);
#ifdef MOVE3D_CORE
	Robot* trajRobot = global_Project->getActiveScene()->getActiveRobot();
	API::CostOptimization optimTrj(trajRobot->getCurrentTraj());
	optimTrj.runDeformation(ENV.getInt(Env::nbCostOptimize));
	optimTrj.replaceP3dTraj();
#endif
	g3d_draw_allwin_active();
	ENV.setBool(Env::isRunning,false);
}

/**
 * Optimize one step
 */
void qt_oneStepOptim()
{
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
	
	//	  	p3d_SetIsCostFuncSpace(TRUE);
#ifdef MOVE3D_CORE
	Robot* trajRobot = new Robot(robotPt);
	API::CostOptimization optimTrj(trajRobot,CurrentTrajPt);
	
	optimTrj.oneLoopDeform();
	//		optimTrj.removeRedundantNodes();
	optimTrj.replaceP3dTraj(CurrentTrajPt);
#endif
	g3d_draw_allwin_active();
	
	if (CurrentTrajPt == NULL)
	{
		PrintInfo(("Warning: no current trajectory to optimize\n"));
	}
	return;	
}

void qt_removeRedundantNodes()
{
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
	
	if (CurrentTrajPt == NULL)
	{
		PrintInfo(("Warning: no current trajectory to optimize\n"));
	}
#ifdef MOVE3D_CORE
	Robot* trajRobot = new Robot(robotPt);
	API::CostOptimization optimTrj(trajRobot,CurrentTrajPt);
	delete trajRobot;
	
	optimTrj.removeRedundantNodes();
	optimTrj.replaceP3dTraj(CurrentTrajPt);
	g3d_draw_allwin_active();
	
	if (optimTrj.isValid())
	{
		cout << "Trajectory valid" << endl;
  }
  else
  {
    cout << "Trajectory not valid" << endl;
  }
#endif
  
  return;	
}

#ifdef LIGHT_PLANNER
/**
 * Make traj from via points
 */
void qt_makeTrajFromViaPoints()
{
  //    p3d_rob * robotPt = p3d_get_robot_by_name("PR2_ROBOT");//justin//JIDOKUKA_ROBOT
  p3d_rob * robotPt =  global_Project->getActiveScene()->getActiveRobot()->getRobotStruct();
  ManipulationViaConfPlanner m_viaConfPlan(robotPt);
  std::vector<SM_TRAJ> smTrajs;
  
  // 	if(FORMGENOM_CARTESIAN == 1) {
  //   	for(int i=0; i<m_viaConfPlan.robot()->armManipulationData->size(); i++) {
  //     		m_viaConfPlan.setArmCartesian(i,true);
  //   	}
  // 	} else {
  //   	for(int i=0; i<m_viaConfPlan.robot()->armManipulationData->size(); i++) {
  //     		m_viaConfPlan.setArmCartesian(i,false);
  //   	}
  // 	}
  
  m_viaConfPlan.planTrajFromConfigArrayInRobotTheForm(smTrajs);
  //    MainWindow::planningFinished();
  
}
#endif

/**
 * Read Scenario
 */
void qt_readScenario()
{
	std::string fileToOpen(qt_fileName);
	cout <<" Should Open scenarion " << fileToOpen << endl;
	
	p3d_rw_scenario_init_name();
	p3d_read_scenario(qt_fileName);
}

/**
 * Save Scenario
 */
void qt_saveScenario()
{
	std::string fileToOpen(qt_fileName);
	cout <<" Should Open scenarion " << fileToOpen << endl;
	
	p3d_rw_scenario_init_name();
	p3d_save_scenario(qt_fileName);
}

void qt_readTraj()
{
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	int ir;
	configPt qi, qf;
	pp3d_traj trajPt;
	
	if (qt_fileName!=NULL) 
	{
		if (p3d_read_traj(qt_fileName)) 
		{
			trajPt = (p3d_traj *) p3d_get_desc_curid(P3D_TRAJ);
			ir = p3d_get_desc_curnum(P3D_ROBOT);
      //			g3d_add_traj(p3d_get_desc_curname(P3D_TRAJ),
      //						 p3d_get_desc_number(P3D_TRAJ));
			qi = p3d_alloc_config(robotPt);
			qf = p3d_alloc_config(robotPt);
			p3d_ends_and_length_traj(trajPt, &qi, &qf);   
			p3d_copy_config_into(robotPt, qf, &(robotPt->ROBOT_GOTO));
			p3d_copy_config_into(robotPt, qi, &(robotPt->ROBOT_POS));
			
      //			g3d_draw_allwin_active();
			p3d_destroy_config(robotPt, qi);
			p3d_destroy_config(robotPt, qf);
		}
	}
	
	cout << "Apres lecture de la trajectoire" << endl;
}

void qt_load_HRICS_Grid(std::string docname)
{
#ifdef HRI_COSTSPACE
	ENV.setBool(Env::drawGrid,false);
	
	HRICS_activeNatu  = new HRICS::Natural;
	
  //	string docname(qt_fileName);
	
	HRICS::NaturalGrid* myGrid = new HRICS::NaturalGrid;
	myGrid->setNaturalCostSpace(HRICS_activeNatu);
	
	bool reading_OK=false;
	
	for (int i=0; (i<5)&&(!reading_OK) ; i++) 
	{
		cout << "Reading grid at : " << docname << endl;
		reading_OK = myGrid->loadFromXmlFile(docname);
	}
	//		myGrid->initReachable();
	//		myGrid->resetCellCost();
	API_activeGrid = myGrid;
	
	if( HRICS_MotionPL != NULL )
	{
		if( HRICS_activeNatu->IsHuman() )
		{
			cout << "Set Reachability space" << endl;
			HRICS_MotionPL->setReachability(HRICS_activeNatu);
		}
		else 
		{
			cout << "Set Natural space" << endl;
			HRICS_MotionPL->setNatural(HRICS_activeNatu);
		}
	}
	
	ENV.setBool(Env::drawGrid,true);
#else
  cout << "HRICS is not compiled!!!" << endl;
#endif
}

// Add a trajectory to the interface
void qt_add_traj(char* name,int id,p3d_rob* rob,p3d_traj* traj)
{
	std::ostringstream oss;
	oss << name << " (" << id - 1 << " )";
	
  cout << "traj = " << traj << endl;
  
#ifdef QT_GL
  if(rob != NULL)
  {
    FormRobot* form = global_w->getMoveRobot()->getRobotFormByName( rob->name );
    
    std::string str = oss.str();
    form->addTraj(str,traj);
  }
#endif
}

void qt_add_config_to_ui(char* name,p3d_rob* robot,double* q)
{
#ifdef QT_GL
	FormRobot* form = global_w->getMoveRobot()->getRobotFormByName( robot->name );
  
	std::string str(name);
	form->addConfig(str,q);
#endif
}

#ifdef HRI_PLANNER
void qt_test()
{
  cout << "Running test" << endl; 
  
  HRI_AGENTS* agents = hri_create_agents();
  HRI_AGENT* pr2 = hri_get_one_agent_of_type(agents, HRI_PR2);
  
  p3d_matrix4 mat_ID = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  
  hri_agent_is_grasping_obj_at_center( pr2, "VISBALL" , 0 , mat_ID);
}
#endif

//------------------------------------------------------------------------------
// Planner handler
//------------------------------------------------------------------------------
#include <pthread.h>
extern pthread_attr_t attr;
extern int mainMhp(int argc, char** argv);


PlannerHandler::PlannerHandler(int argc, char** argv) :
mState(none),
mArgc(argc),
mArgv(argv)
{
  
}

void PlannerHandler::init()
{
  mainMhp(mArgc, mArgv);
  emit initIsDone();
}

void PlannerHandler::startPlanner(QString plannerName)
{
  if(mState == running) // already running, do nothing
  {
    printf("Error: PlannerHandler::startPlanner called, but a planner \
           is already running.\n");
    return;
  }
  mState = running;
#ifdef P3D_PLANNER
  p3d_SetStopValue(FALSE);
#endif
  ENV.setBool(Env::isRunning, true);
  try
  {
    if(plannerName == "Diffusion")
    {
      std::cout << "Planning thread : starting diffusion." << std::endl;
      qt_runDiffusion();
      //qt_test();
    }
    else if(plannerName == "PRM")
    {
      std::cout << "Planning thread : starting PRM." << std::endl;
      qt_runPRM();
    }
#ifdef LIGHT_PLANNER
    else if(plannerName == "Manipulation")
    {
      std::cout << "Planning thread : starting Manipulation." << std::endl;
      //      size_t stacksize;
      //      pthread_attr_init(&attr);
      //      pthread_attr_getstacksize (&attr, &stacksize);
      Manip::runManipulation();
    }
    else if(plannerName == "NavigationSM")
    {
      std::cout << "Navigation thread : starting navigation." << std::endl;
      qt_runNavigation();
    }
    else if (plannerName == "ManipCurrentTest"){
      std::cout << "Manipulation Test :" << std::endl;
      Manip::runCurrentTest();
    }
#endif
#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
    else if(plannerName == "Replanning")
    {
      std::cout << "Re-Planning thread : starting Re-planning." << std::endl;
      qt_runReplanning();
    }
    else if(plannerName == "ExecuteReplanTraj")
    {
      std::cout << "Re-Planning thread : starting simulation." << std::endl;
      qt_executeReplanSimu();
    }
#endif
    //    else if(plannerName == "Replanning")
    //    {
    //      std::cout << "Compute Handover thread : starting Re-planning." << std::endl;
    //      qt_handover();
    //    }
    else if (plannerName == "ShowTraj" ){
      std::cout << "Show trajectory : " << std::endl;
      qt_showTraj();
    }
    else if (plannerName == "Optimize" ){
      std::cout << "Optimize trajectory : " << std::endl;
      qt_optimize();
    }
    else if (plannerName == "Shortcut" ){
      std::cout << "Shortcut trajectory : " << std::endl;
      qt_shortCut();
    }

#ifdef HRI_PLANNER

    else if( plannerName == "realTimeOtp"){
          qtRealTimeOtp();
    }
    else if( plannerName == "otp"){
      qtOTP();
    }
    else if( plannerName == "MultipleOtp"){
      qtMultipleOTP();
    }
#endif
#ifdef GRASP_PLANNING
    else if( plannerName == "makeTrajFromViaPoints"){
      qt_makeTrajFromViaPoints();
    }
#endif
#ifdef MULTILOCALPATH
    else if( plannerName == "runStomp"){
      qt_runStomp();
    }
    else if( plannerName == "runChomp"){
      qt_runChomp();
    }
    else if( plannerName == "convertToSoftMotion"){
      traj_optim_generate_softMotion();
    }
#endif
  }
  catch(std::string what)
  {
    std::cerr << "Planner thread : caught exception : " << what << std::endl;
  }
  catch(std::exception& e)
  {
    std::cerr << "Planner thread : caught exception : " << e.what() << std::endl;
  }
  catch(...)
  {
    std::cerr << "Planner thread : caught exception of unknown type." << std::endl;
  }
  ENV.setBool(Env::isRunning, false);
  mState = stopped;
  emit plannerIsStopped();
}

void PlannerHandler::stopPlanner()
{
  if(mState != running) // not running, do nothing
  {
    printf("Error: PlannerHandler::stopPlanner called, but there is no planner \
           running.\n");
    //   return;
  }
#ifdef P3D_PLANNER
  p3d_SetStopValue(true);
#endif
  ENV.setBool( Env::isRunning, false );
  PlanEnv->setBool( PlanParam::stopPlanner, true );
}

void PlannerHandler::resetPlanner()
{
  if(mState == running) // running, do nothing
  {
    printf("Error: PlannerHandler::resetPlanner called, but there is a planner \
           currently running.\n");
    return;
  }
  if(mState == none) // no planner, do nothing
  {
    printf("Error: PlannerHandler::resetPlanner called, but there is no planner.\n");
    return;
  }
  mState = none;
  qt_resetGraph();
#ifdef P3D_PLANNER
  p3d_SetStopValue(false);
#endif
  PlanEnv->setBool( PlanParam::stopPlanner, false );
  emit plannerIsReset();
}

