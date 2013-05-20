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
#include <sys/time.h>

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "LightPlanner-pkg.h"

#include "move3d-headless.h"
#include "move3d-gui.h"

#include "qtFormRobot/moverobot.hpp"
#include "qtFormRobot/sliderfunction.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"
#include "qtMainInterface/sideWidgets/qtRobot.hpp"

#include "API/project.hpp"
#include "API/Search/Dijkstra/dijkstra.hpp"

#include "planner/planner.hpp"
#include "planner/Diffusion/Variants/Star-RRT.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"

#include "utils/MultiRun.hpp"

#include "qtLibrary.hpp"
#include <QtCore/QMutexLocker>
#include <QtCore/QTime>

#ifdef ENERGY
#include "bio/BioEnergy/include/Energy-pkg.h"
#endif

#if defined( HRI_COSTSPACE )
#include "hri_costspace/HRICS_costspace.hpp"
#include "hri_costspace/HRICS_Workspace.hpp"
#include "hri_costspace/HRICS_Miscellaneous.hpp"
#include "hri_costspace/HRICS_Navigation.hpp"
#include "hri_costspace/Gestures/HRICS_GestParameters.hpp"
#include "hri_costspace/Gestures/HRICS_WorkspaceOccupancy.hpp"
#include "hri_costspace/Gestures/HRICS_RecordMotion.hpp"
#include "hri_costspace/Gestures/HRICS_HumanPredictionCostSpace.hpp"
#include "hri_costspace/Gestures/HRICS_HumanPredictionSimulator.hpp"

#include "hri_costspace/HumanTrajectories/HRICS_IOCSimulator.hpp"

#if defined( HRI_PLANNER )
#include "hri_costspace/HRICS_HAMP.hpp"
#include "hri_costspace/HRICS_otpmotionpl.hpp"
#endif
#endif

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#include "../lightPlanner/proto/ManipulationViaConfPlanner.hpp"

#if defined(MULTILOCALPATH)
#include "planner/replanningAlgorithms.hpp"
#include "planner/replanningSimulators.hpp"

extern ManipulationTestFunctions* global_manipPlanTest;
#endif
#endif

const char *qt_fileName = NULL;

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

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

extern void* GroundCostObj;

//------------------------------------------------------------------------------
//  Init Functions
//------------------------------------------------------------------------------

//! This function initialises the basic costspace
void qt_init_costspace()
{
    GlobalCostSpace::initialize();

    std::string function;

    if( GroundCostObj != NULL )
        function = "costMap2D";
    else
        function = "costDistToObst";

    global_costSpace->setCost( function );
}

//! This function initialises the software
//! after the parameter file has been loaded
void qt_init_after_params()
{
    cout << "p3d_set_user_drawnjnt : " << ENV.getInt(Env::jntToDraw) << endl;
    p3d_set_user_drawnjnt( ENV.getInt(Env::jntToDraw) );

    if(ENV.getBool(Env::isCostSpace) || ENV.getBool(Env::useTRRT) )
    {
        qt_init_costspace();
    }

    if ( ENV.getBool(Env::enableHri) )
    {
        HRICS_init();
    }

    if( ENV.getBool(Env::setActiveJointsGroup) )
    {
        traj_optim_init_mlp_cntrts_and_fix_joints();
    }

    if( ENV.getBool(Env::setStompPlanner) )
    {
        traj_optim_initStomp();
    }

    if( PlanEnv->getBool(PlanParam::setRobotIK))
    {
        traj_optim_switch_cartesian_mode( true );
    }

    if( GestEnv->getBool(GestParam::init_module_at_start) )
    {
        HRICS_initOccupancyPredictionFramework();
    }
}

//------------------------------------------------------------------------------
//  Callback functions  
//------------------------------------------------------------------------------
void qt_drawAllWinActive()
{
    g3d_draw_allwin_active();
}

static HRICS::Navigation* m_navigation = NULL;

void qt_test1()
{
    delete global_workspaceOccupancy;
    global_workspaceOccupancy = NULL;

    //  Scene* sce = global_Project->getActiveScene();
    //  Robot* robot = sce->getRobotByName("PR2_ROBOT");
    //  if( robot == NULL ) {
    //    cout << "NO robot PR2_ROBOT in the scene" << endl;
    //    return;
    //  }

    //  if( m_navigation == NULL ) {
    //    m_navigation = new HRICS::Navigation( robot );
    //  }
    //  else {
    //    m_navigation->reset();
    //  }

    //  confPtr_t q_init = robot->getInitPos();
    //  confPtr_t q_goal = robot->getGoalPos();

    //  API::Trajectory* path_ = m_navigation->computeRobotTrajectory( q_init, q_goal );

    //  if( path_ )
    //    path_->replaceP3dTraj();





    //HRICS::generateGraspConfigurations();
    //string robotName("PR2_ROBOT");
    //  Node* node = global_w->Ui()->tabMotionPlanner->getIthNodeInBestTraj();
    //
    //  double t=0.0;
    //  ChronoTimeOfDayOn();
    //
    //  dynamic_cast<StarRRT*>(global_Move3DPlanner)->pruneTreeFromNode( node );
    //
    //  ChronoTimeOfDayTimes( &t );
    //  ChronoTimeOfDayOff();
    //
    //  cout << "Time to erase graph : " << t << endl;
    //  HRICS::printHumanConfig();
}

void qt_test2()
{
    //  cout << "Plan param 1 : " << PlanEnv->getBool(PlanParam::starRRT) << endl;
    //  cout << "Plan param 2 : " << PlanEnv->getBool(PlanParam::starRewire) << endl;

    //HRICS::printHumanConfig();
    //HRICS::setTenAccessiblePositions();

    //  p3d_set_goal_solution_function( manipulation_get_free_holding_config );
    //  HRICS::setSimulationRobotsTransparent();
    //HRICS_humanCostMaps->loadAgentGrids();

    //  if (HRICS::initShelfScenario())
    //  {
    //    HRICS::execShelfScenario();
    //  }

    cout << "Clear traj" << endl;
    Robot* robot = global_Project->getActiveScene()->getActiveRobot();
    p3d_destroy_traj( robot->getRobotStruct(), robot->getRobotStruct()->tcur );

    if( global_rePlanningEnv != NULL )
        global_rePlanningEnv->resetTrajectoriesToDraw();
}

static bool recompute_cost=false;
static bool init_generator=false;
static bool use_list_generator=false;
static ConfGenerator* generatorPtr=NULL;

void qt_test3()
{
    //  HRICS_humanCostMaps->testCostFunction();
    //  HRICS_humanCostMaps->saveAgentGrids();

    //qt_set_otp_cost_recompute();

    //  API_activeGrid = HRICS_activeNatu->getGrid();

    //  timeval tim;
    //  gettimeofday(&tim, NULL);
    //  double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);
    //
    //  Scene* sce = global_Project->getActiveScene();
    //  Robot* robot = sce->getRobotByNameContaining("ROBOT");
    //  Robot* human = sce->getRobotByNameContaining("HUMAN");
    //
    //  if( !init_generator )
    //  {
    //    string dir(getenv("HOME_MOVE3D"));
    //    string file("/statFiles/OtpComputing/confHerakles.xml");
    //
    //    generatorPtr = new ConfGenerator( robot, human );
    //    generatorPtr->initialize( dir+file, HRICS_activeNatu );
    //
    //    init_generator = true;
    //  }
    //
    //  if( use_list_generator )
    //  {
    //    // Compute LIST
    //    pair<confPtr_t,confPtr_t> best_handover_conf;
    //    double best_cost=0.0;
    //
    //    if( generatorPtr->computeHandoverConfigFromList( best_handover_conf, best_cost ))
    //    {
    //      human->setAndUpdate( *best_handover_conf.first );
    //      robot->setAndUpdate( *best_handover_conf.second );
    //    }
    //    cout << "Cost of configuration is : " << best_cost << endl;
    //  }
    //  else
    //  {
    //    // Compute IK
    //    std::vector<Eigen::Vector3d> points;
    //    HRICS_humanCostMaps->getHandoverPointList( points, recompute_cost, true );
    //
    //    configPt q;
    //    bool found_ik = false;
    //    for (int i=0; i<int(points.size()) && found_ik==false; i++) {
    //      found_ik = generatorPtr->computeRobotIkForGrabing( q, points[i] );
    //    }
    //
    //    recompute_cost = !recompute_cost;
    //
    //    // Disable cntrts
    //    p3d_cntrt* ct;
    //    p3d_rob* rob = robot->getRobotStruct();
    //    for(int i=0; i<rob->cntrt_manager->ncntrts; i++) {
    //      ct = rob->cntrt_manager->cntrts[i];
    //      p3d_desactivateCntrt( rob, ct );
    //    }
    //
    //    if( found_ik ) {
    //      confPtr_t target_new_ = confPtr_t(new Configuration( robot, q ));
    //      robot->setAndUpdate( *target_new_ );
    //    }
    //  }
    //
    //  gettimeofday(&tim, NULL);
    //  double dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
    //  cout << "Handover computed in : " << dt << " sec" << endl;
    //  g3d_draw_allwin_active();

    //  Eigen::Vector3d point = human->getJoint("rPalm")->getVectorPos();
    //  point[2] += 0.10;
    //
    //  configPt q;
    //  if( generator.computeRobotIkForGrabing( q, point ) )
    //  {
    //    // Deactivate cntrts
    //    p3d_rob* rob = robot->getRobotStruct();
    //    for(int i=0; i<rob->cntrt_manager->ncntrts; i++) {
    //      p3d_cntrt* ct = rob->cntrt_manager->cntrts[i];
    //      p3d_desactivateCntrt( rob, ct );
    //    }
    //
    //    cout << "Ik computed " << endl;
    //    confPtr_t q_rob(new Configuration(robot,q));
    //
    //    // Generate trajectory
    //    API::CostOptimization optim( robot->getCurrentTraj() );
    //    double step = optim.getRangeMax() / 10;
    //    cout << "Connect configuration " << endl;
    //    optim.connectConfiguration( q_rob, step );
    //    optim.replaceP3dTraj();
    //    qt_drawAllWinActive();
    //  }
    //  else {
    //    cout << "Could not find a robot OTP configuration" << endl;
    //  }
}

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

        //    XYZ_GRAPH = NULL;

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
 * Run Diffusion
 */
void qt_runDiffusion()
{
    cout << "Run Diffusion" << endl;
    Robot* robot = global_Project->getActiveScene()->getActiveRobot();

    try
    {
#ifdef P3D_PLANNER
        p3d_SetStopValue(FALSE);
#endif
        timeval tim;
        gettimeofday(&tim, NULL);
        double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);

        int res;
        cout << "ENV.getBool(Env::Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
        if (ENV.getBool(Env::treePlannerIsEST))
        {
            res = p3d_run_est(robot->getRobotStruct());
        }
        else
        {
            res = p3d_run_rrt(robot->getRobotStruct());
        }

        gettimeofday(&tim, NULL);
        double dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
        cout << "RRT computed in : " << dt << " sec" << endl;

        if( !ENV.getBool(Env::drawDisabled) ) {
            g3d_draw_allwin_active();
        }
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
    cout << "Run Probabilistic Road Map" << endl;
    Robot* robot = global_Project->getActiveScene()->getActiveRobot();

    try
    {
#ifdef P3D_PLANNER
        p3d_SetStopValue(FALSE);
#endif

        int res;

        ChronoOn();

        switch(ENV.getInt(Env::PRMType))
        {
        case 0:
            res = p3d_run_prm(robot->getRobotStruct());
            break;
        case 1:
            res = p3d_run_vis_prm(robot->getRobotStruct());
            break;
        case 2:
            res = p3d_run_acr(robot->getRobotStruct());
            break;
        case 3:
            res = p3d_run_perturb_prm(robot->getRobotStruct());
            break;
        default:
            cout << "Error No Other PRM"  << endl;
            ChronoPrint("");
            ChronoOff();
            return;
        }

        ChronoPrint("");
        ChronoOff();

        if( !ENV.getBool(Env::drawDisabled) ) {
            g3d_draw_allwin_active();
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

void qt_runMultiRRT()
{
    MultiRun pool;
    pool.runMultiRRT();
}

void qt_runMultiSmooth()
{
    MultiRun pool;
    pool.runMultiSmooth();
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

#endif

void qt_handover()
{

    //  compute_handOver();
}

//----------------------------------------------------------
// Gesture Functions
//----------------------------------------------------------

void qt_show_recorded_motion()
{
    cout << "Show recorded motion" << endl;

    if(!global_motionRecorder) {
        cout << "recorder not initialized" << endl;
        return;
    }

    global_motionRecorder->showStoredMotion();
    cout << "End recorded motion" << endl;
}

void qt_workspace_occupancy()
{
    cout << "Loading regressed motion and computing the occupancy" << endl;
    global_motionRecorder->loadRegressedFromCSV();
    global_workspaceOccupancy->setRegressedMotions( global_motionRecorder->getStoredMotions() );
    global_workspaceOccupancy->computeOccpancy();
}

void qt_classify_motions()
{
    if( global_humanPredictionSimulator == NULL || global_motionRecorder == NULL )
    {
        cout << "global_humanPredictionSimulator or global_motionRecorder are not initilized" << endl;
        return;
    }

    global_motionRecorder->loadFolder();

    const std::vector<motion_t>& stored_motions = global_motionRecorder->getStoredMotions();

    for(int i=0;i<8;i++)
    {
        for(int j=0;j<5;j++)
        {
            int index = i*25+j;
            cout << std::setw( 3 ) << std::setfill( '0' ) << index  << " : " ;
            int id_class = global_humanPredictionSimulator->classifyMotion( global_motionRecorder->resample( stored_motions[index], 100 ) );
            //cout << std::setw( 3 ) << std::setfill( ' ' ) << i << " : " << id_class << endl;
        }
    }
    cout << "End!!!" << endl;
}

void qt_voxel_occupancy_simulation()
{
    cout << "Starting Simulator" << endl;
    if( global_humanPredictionSimulator == NULL )
    {
        cout << "global_humanPredictionSimulator == NULL" << endl;
        return;
    }
    global_humanPredictionSimulator->runVoxelOccupancy();
}

void qt_human_prediction_simulation_tests()
{
    cout << "Starting Simulator" << endl;
    if( global_humanPredictionSimulator == NULL )
    {
        cout << "global_humanPredictionSimulator == NULL" << endl;
        return;
    }

    int nb_motions = 1;
    int nb_runs = 1;
    std::vector<int> human_motion_id( nb_motions );
    std::vector< std::vector<double> > costs( nb_motions );

    human_motion_id[0] = 77;

    // (25,26,27,28,29)
//    human_motion_id[0] = 25;
//    human_motion_id[1] = 26;
//    human_motion_id[2] = 27;
//    human_motion_id[3] = 28;
//    human_motion_id[4] = 29;

    for(int i=0;i<nb_motions;i++)
    {
        GestEnv->setInt( GestParam::human_traj_id, human_motion_id[i] );

        costs[i].resize(nb_runs);

        for(int j=0;j<nb_runs;j++)
        {
            cout << "----------------------------------------" << endl;
            cout << " TEST ("<< i << " , " << j << " )"<< endl;
            cout << "----------------------------------------" << endl;

            costs[i][j] = global_humanPredictionSimulator->run();
        }
    }

    for(int i=0;i<nb_motions;i++)
    {
        cout << "-------------------" << endl;
        cout << " Human motion " << i << endl;
        cout << "-------------------" << endl;
        for(int j=0;j<nb_runs;j++)
        {
            // Matlab
            cout << "costs_" << i << "(" << j+1 << ") = " << costs[i][j] << endl;
        }
    }
}

void qt_human_prediction_simulation()
{
    cout << "Starting Simulator" << endl;
    if( global_humanPredictionSimulator == NULL )
    {
        cout << "global_humanPredictionSimulator == NULL" << endl;
        return;
    }

    double cost = global_humanPredictionSimulator->run();

    cout << "cost of trajectory : " << cost << endl;
}

//----------------------------------------------------------
// Inverse Optimal Control Functions
//----------------------------------------------------------
void qt_runHumanIOC()
{
    HRICS::IOCSimulator sim;
    sim.run();
}

//----------------------------------------------------------
// Navigation Functions
//----------------------------------------------------------
HRICS::Navigation* navPlanner = NULL;

void qt_computeAStar()
{
    cout << "Compute AStar" <<  endl;
    Robot* rob;

    if (navPlanner == NULL)
    {
        rob = global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");
        navPlanner = new HRICS::Navigation(rob);
    }
    else {
        navPlanner->reset();
        rob = navPlanner->getRobot();
    }

    confPtr_t q_init = rob->getInitPos();
    confPtr_t q_goal = rob->getGoalPos();

    navPlanner->computeRobotTrajectory( q_init, q_goal );
}

#ifdef HRI_PLANNER

void qtRealTimeOtp()
{
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setRobotPos();
    int tmp = PlanEnv->getInt(PlanParam::env_timeShow);
    PlanEnv->setInt(PlanParam::env_timeShow,0);
    ENV.setBool(Env::drawGraph,false);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->setAsNotSorted();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->clearRealTreajectory();
    //    PlanEnv->setBool(PlanParam::env_drawHumanModel,true);
    //    PlanEnv->setBool(PlanParam::env_showText,false);
    bool first = true;
    while (PlanEnv->getBool(PlanParam::env_realTime))
    {
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);
        //        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getInputs();
        Eigen::Vector3d pos;
        pos[0] = PlanEnv->getDouble(PlanParam::env_futurX);
        pos[1] = PlanEnv->getDouble(PlanParam::env_futurY);
        pos[2] = PlanEnv->getDouble(PlanParam::env_futurRZ);

        Eigen::Vector2d v(pos[0],pos[1]);
        v[0] = pos[0];
        v[1] = pos[1];
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->addVectorToRealTreajectory(v);

        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setInputs(pos,
                                                                           dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getRobotPos(),
                                                                           PlanEnv->getBool(PlanParam::env_isStanding),
                                                                           PlanEnv->getDouble(PlanParam::env_objectNessecity));


        if (!dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->isTheRealNearThePredicted(0.2) || first)
        {
            first = false;
            dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->clearRealTreajectory();

            if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->newComputeOTP())
            {
                dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showBestConf();

                ENV.setBool(Env::drawOTPTraj,true);
            }
        }
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showBestConf();
        g3d_draw_allwin_active();
    }

    PlanEnv->setBool(PlanParam::env_showText,true);
    //    PlanEnv->setBool(PlanParam::env_drawHumanModel,false);
    ENV.setBool(Env::drawGraph,true);
    PlanEnv->setInt(PlanParam::env_timeShow,tmp);
}

void qtOTP()
{
    //    global_w->getOpenGL()
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();

    //	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->initGrid();
    //	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->simpleComputeBaseAndOTP();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->setAsNotSorted();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->newComputeOTP();
    ENV.setBool(Env::drawOTPTraj,true);
    //	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computePR2GIK();

}

void qt_simpleNav()
{
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->navigate();
    qt_drawAllWinActive();

}
#endif

void qt_runParallelStomp()
{
//    srompRun_MultipleParallel();
    srompRun_OneParallel();
}

#ifdef MULTILOCALPATH
void qt_runStomp()
{
    if( traj_optim_runStomp(0) )
    {
        cout << "Stomp has run succesfully!!!" << endl;
    }
    else {
        cout << "Stomp fail!!!" << endl;
    }
}


void qt_runStompNoReset()
{
    if( traj_optim_runStompNoReset(0) )
    {
        cout << "Stomp has run succesfully!!!" << endl;
    }
    else {
        cout << "Stomp fail!!!" << endl;
    }
}

void qt_init_mlp_cntrts_and_fixjoints() 
{
    traj_optim_init_mlp_cntrts_and_fix_joints();
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
void qt_executeSimpleSimu()
{
    cout << "qt_executeSimpleSimu" << endl;

    if( global_rePlanningEnv == NULL )
    {
        global_rePlanningEnv = new ReplanningSimulator();
    }
    global_rePlanningEnv->execute_simple_simulation( default_drawtraj_fct_qt_pipe );
}

void qt_executePlan()
{
    if( HRICS::initShelfScenario() )
    {
        HRICS::execShelfScenario();
    }
}
#endif

void qt_showTraj()
{
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    g3d_show_tcur_rob(robotPt,default_drawtraj_fct_qt_pipe);
    ENV.setBool(Env::isRunning,false);
    if (PlanEnv->getBool(PlanParam::env_showHumanTraj))
    {
        p3d_rob *hum_robotPt;
        Robot* rob = global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");
        if (rob)
        {
            hum_robotPt = rob->getRobotStruct();
            // dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,false);
            double tmp = ENV.getDouble(Env::showTrajFPS);
            ENV.setDouble(Env::showTrajFPS,tmp*5);
            g3d_show_tcur_rob(hum_robotPt,default_drawtraj_fct_qt_pipe);
            ENV.setDouble(Env::showTrajFPS,tmp);
            // dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showBestConf();
        }
    }
}

/**
 * Shortcut optimization
 */
void qt_shortCut()
{
    Robot* robot = global_Project->getActiveScene()->getActiveRobot();

    cout << "Random shortCut for robot : " << robot->getName() << endl;
    API::Smoothing optimTrj(robot->getCurrentTraj());
    optimTrj.runShortCut(PlanEnv->getInt(PlanParam::smoothMaxIterations));
    optimTrj.replaceP3dTraj();

    g3d_draw_allwin_active();
    ENV.setBool(Env::isRunning,false);
}

/**
 * Deformation optimization
 */
void qt_optimize()
{
    Robot* robot = global_Project->getActiveScene()->getActiveRobot();

    cout << "Random deformation for robot : " << robot ->getName()<< endl;
    API::CostOptimization optimTrj(robot->getCurrentTraj());
    optimTrj.runDeformation(PlanEnv->getInt(PlanParam::smoothMaxIterations));
    optimTrj.replaceP3dTraj();

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
    //	std::string fileToOpen(qt_fileName);
    cout <<" Should Open scenarion " /*<< fileToOpen << " or "*/ << qt_fileName << endl;

    p3d_rw_scenario_init_name();
    p3d_read_scenario(qt_fileName);
}

/**
 * Save Scenario
 */
void qt_saveScenario()
{
    std::string fileToOpen(qt_fileName);
    cout <<" Should Save scenario " << fileToOpen << endl;

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
            // g3d_add_traj(p3d_get_desc_curname(P3D_TRAJ),
            // p3d_get_desc_number(P3D_TRAJ));
            qi = p3d_alloc_config(robotPt);
            qf = p3d_alloc_config(robotPt);
            p3d_ends_and_length_traj(trajPt, &qi, &qf);
            p3d_copy_config_into(robotPt, qf, &(robotPt->ROBOT_GOTO));
            p3d_copy_config_into(robotPt, qi, &(robotPt->ROBOT_POS));

            // g3d_draw_allwin_active();
            p3d_destroy_config(robotPt, qi);
            p3d_destroy_config(robotPt, qf);
        }
    }

    cout << "Apres lecture de la trajectoire" << endl;
}

//------------------------------------------------------------------
//------------------------------------------------------------------
void qt_recompute_agent_grid_cost()
{
    // If a costspace exists re compute all cell cost
    if( HRICS_humanCostMaps != NULL )
    {
        HRICS_humanCostMaps->computeAllCellCost();
    }
    else {
        cout << "HRICS_humanCostMaps is not initialized!!!" << endl;
    }
}

void qt_save_agent_grid()
{
    if( HRICS_humanCostMaps != NULL )
    {
        HRICS_humanCostMaps->saveAgentGrids();
    }
    else {
        cout << "HRICS_humanCostMaps is not initialized!!!" << endl;
    }
}

void qt_load_agent_grid()
{
    if( HRICS_humanCostMaps != NULL )
    {
        const char* move3d_home = getenv("HOME_MOVE3D");

        if( move3d_home == NULL )
        {
            cout << "HOME_MOVE3D is undefined" << endl;
            return;
        }

        std::string home( move3d_home );
        std::string filename = "/statFiles/Cost3DGrids/human_grids_0.grid";

        filename = home + filename;

        HRICS_humanCostMaps->loadAgentGrids( filename );
    }
}
//------------------------------------------------------------------
//------------------------------------------------------------------
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

//------------------------------------------------------------------
//------------------------------------------------------------------

// Add a trajectory to the interface
void qt_add_traj(char* name,int id,p3d_rob* rob,p3d_traj* traj)
{
    std::ostringstream oss;
    oss << name << " (" << id - 1 << " )";
    //cout << "traj = " << traj << endl;

#ifdef QT_GL
    if(global_manipPlanTest)
    {
        global_manipPlanTest->addTraj( oss.str(), traj );
        return;
    }

    if(rob != NULL)
    {
        FormRobot* form = global_w->getMoveRobot()->getRobotFormByName( rob->name );

        std::string str = oss.str();
        form->addTraj(str,traj);
    }
#endif
}

void qt_add_config_to_ui(char* name,p3d_rob* rob,double* q)
{
#ifdef QT_GL
    if(global_manipPlanTest)
    {
        global_manipPlanTest->addConf( name, q );
        return;
    }

    if(rob != NULL)
    {
        FormRobot* form = global_w->getMoveRobot()->getRobotFormByName( rob->name );

        std::string str(name);
        form->addConfig(str,q);
    }
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
        else if(plannerName == "MultiRRT")
        {
            std::cout << "Planning thread : starting MultiRRT." << std::endl;
            qt_runMultiRRT();
        }
        else if(plannerName == "MultiSmooth")
        {
            std::cout << "Planning thread : starting MultiSmooth." << std::endl;
            qt_runMultiSmooth();
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
        else if (plannerName == "ComputeAStar" )
        {
            qt_computeAStar();
        }
        else if (plannerName == "ManipCurrentTest")
        {
            std::cout << "Manipulation Test :" << std::endl;
            Manip::runCurrentTest();
        }
#endif
#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
        else if(plannerName == "ExecuteManipulationPlan")
        {
            std::cout << "execute manipulation thread : starting plan." << std::endl;
            qt_executePlan();
        }
        else if(plannerName == "ExecuteSimpleSimu")
        {
            qt_executeSimpleSimu();
        }
#endif
        //    else if(plannerName == "Replanning")
        //    {
        //      std::cout << "Compute Handover thread : starting Re-planning." << std::endl;
        //      qt_handover();
        //    }
        else if (plannerName == "ShowTraj" ){
            std::cout << "Show trajectory : ";
            std::cout.flush();
            qt_showTraj();
            std::cout << "Done" << std::endl;
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
        else if( plannerName == "simpleNav"){
            qt_simpleNav();
        }
        else if( plannerName == "MultipleOtp"){
            qtMultipleOTP();
        }
        else if( plannerName == "ComputeAgentGridCost"){
            qt_recompute_agent_grid_cost();
        }
        else if( plannerName == "LoadAgentGrid"){
            qt_load_agent_grid();
        }
        else if( plannerName == "SaveAgentGrid"){
            qt_save_agent_grid();
        }
#endif
#ifdef GRASP_PLANNING
        else if( plannerName == "makeTrajFromViaPoints"){
            qt_makeTrajFromViaPoints();
        }
#endif
        else if( plannerName == "runParallelStomp" ){
            qt_runParallelStomp();
        }
#ifdef MULTILOCALPATH
        else if( plannerName == "runStomp"){
            qt_runStomp();
        }
        else if( plannerName == "runChomp"){
            qt_runChomp();
        }
        else if( plannerName == "runNoReset") {
            qt_runStompNoReset();
        }
        else if( plannerName == "convertToSoftMotion"){
            //traj_optim_generate_softMotion();
            traj_optim_generate_pointsOnTraj();
        }
        else if( plannerName == "initMlpCntrtsAndFixJoints" ){
            qt_init_mlp_cntrts_and_fixjoints();
        }
#endif
        else if( plannerName == "ShowRecordedMotion"){
            qt_show_recorded_motion();
        }
        else if(plannerName == "WorkspaceOccupancy")
        {
            qt_workspace_occupancy();
        }
        else if(plannerName == "ClassifyMotions")
        {
            qt_classify_motions();
        }
        else if(plannerName == "VoxelOccupancy")
        {
            qt_voxel_occupancy_simulation();
        }
        else if(plannerName == "PredictionSimulation")
        {
            qt_human_prediction_simulation();
        }
        else if( plannerName == "HumanIOC" )
        {
            qt_runHumanIOC();
        }
        else if(plannerName == "test1")
        {
            qt_test1();
        }
        else if(plannerName == "test2")
        {
            qt_test2();
        }
        else if(plannerName == "test3")
        {
            qt_test3();
        }
        else
        {
            cout << "planner not define" << endl;
        }
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
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

