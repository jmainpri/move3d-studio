/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
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
#include "API/Roadmap/graphSampler.hpp"
#include "API/Graphic/drawModule.hpp"
#include "API/Graphic/drawCost.hpp"

#include "planner/planner.hpp"
#include "planner/AStar/AStarPlanner.hpp"
#include "planner/Diffusion/Variants/Star-RRT.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#include "planner/plannerSequences.hpp"

#include "utils/MultiRun.hpp"
#include "utils/multilocalpath_utils.hpp"
#include "utils/misc_functions.hpp"
#include "utils/ik_generator.hpp"
#include "utils/pr2_mapping.hpp"

#include "qtLibrary.hpp"
#include <QtCore/QMutexLocker>
#include <QtCore/QTime>

#ifdef ENERGY
#include "bio/BioEnergy/include/Energy-pkg.h"
#endif

#include "feature_space/features.hpp"
#include "feature_space/spheres.hpp"
#include "feature_space/spheres_3d.hpp"
#include "feature_space/squares.hpp"
#include "feature_space/boxes.hpp"
#include "feature_space/clearance.hpp"

#if defined( HRI_COSTSPACE )
#include "hri_costspace/HRICS_parameters.hpp"
#include "hri_costspace/HRICS_costspace.hpp"
#include "hri_costspace/HRICS_workspace.hpp"
#include "hri_costspace/HRICS_miscellaneous.hpp"
#include "hri_costspace/HRICS_navigation.hpp"

#include "hri_costspace/gestures/HRICS_gest_parameters.hpp"
#include "hri_costspace/gestures/HRICS_workspace_occupancy.hpp"
#include "hri_costspace/gestures/HRICS_record_motion.hpp"
#include "hri_costspace/gestures/HRICS_play_motion.hpp"
#include "hri_costspace/gestures/HRICS_human_prediction_cost_space.hpp"
#include "hri_costspace/gestures/HRICS_human_prediction_simulator.hpp"

#include "hri_costspace/human_trajectories/HRICS_ioc_sequences.hpp"
#include "hri_costspace/human_trajectories/HRICS_ioc.hpp"
#include "hri_costspace/human_trajectories/HRICS_human_ioc.hpp"
#include "hri_costspace/human_trajectories/HRICS_human_simulator.hpp"
#include "hri_costspace/human_trajectories/HRICS_detours.hpp"
#include "hri_costspace/human_trajectories/HRICS_run_multiple_planners.hpp"
#include "hri_costspace/human_trajectories/HRICS_dynamic_time_warping.hpp"

#if defined( HRI_PLANNER )
#include "hri_costspace/HRICS_hamp.hpp"
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
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

/*
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

//! This function initialises the software
//! after the parameter file has been loaded
void qt_init_after_params()
{
    cout << "p3d_set_user_drawnjnt : " << ENV.getInt(Env::jntToDraw) << endl;
    p3d_set_user_drawnjnt( ENV.getInt(Env::jntToDraw) );

    if( ENV.getBool(Env::isCostSpace) || ENV.getBool(Env::useTRRT) )
    {
        GlobalCostSpace::initialize();
    }

    if ( ENV.getBool(Env::enableHri) )
    {
        HRICS_init();
    }

    if( ENV.getBool(Env::setActiveJointsGroup) )
    {
        traj_optim_init_mlp_cntrts_and_fix_joints( global_Project->getActiveScene()->getActiveRobot() );
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
        HRICS_initOccupancyPredictionFramework( "PR2_ROBOT", "HERAKLES_HUMAN1" );
    }

    if( PlanEnv->getBool(PlanParam::initColisionSpace))
    {
        traj_optim_initScenario();

        if( global_Project->getActiveScene()->getActiveRobot()->getName() == "PR2_ROBOT")
            FEATURES_init_Clearance_cost();
    }

    if( HriEnv->getBool(HricsParam::init_spheres_cost) || HriEnv->getBool(HricsParam::init_human_trajectory_cost) )
    {
        if( !ENV.getBool(Env::isCostSpace) )
        {
            GlobalCostSpace::initialize();
        }

        bool init_cost = false;

        if( HriEnv->getBool(HricsParam::init_spheres_cost) )
        {
            cout << "init_cost : " << init_cost << endl;
            init_cost |= HRICS_init_boxes_cost();
            cout << "init_cost : " << init_cost << endl;
            init_cost |= HRICS_init_shperes_3d_cost();
            cout << "init_cost : " << init_cost << endl;
            init_cost |= HRICS_init_sphere_cost();
            cout << "init_cost : " << init_cost << endl;
            init_cost |= HRICS_init_square_cost();
        }
        if( HriEnv->getBool(HricsParam::init_human_trajectory_cost) )
        {
            init_cost |= HRICS_init_human_trajectory_cost();
        }

        if( init_cost == true )
        {
            ENV.setBool( Env::isCostSpace, true ); // will initilize the cost space with cost space function
        }
    }

    if( ENV.getBool(Env::isCostSpace) )
    {
        std::string cost_function = PlanEnv->getString(PlanParam::active_cost_function);
        global_costSpace->setCost( cost_function );
        cout << "SET COST FUNCTION : " << cost_function << endl;
    }

    //    if( GestEnv->getBool(GestParam::init_module_ioc) )
    //    {
    //        HRICS_initIverseOptimalControlFramework();
    //    }

//    print_joint_mapping( global_Project->getActiveScene()->getRobotByNameContaining("ROBOT") );
//    print_joint_anchors( global_Project->getActiveScene()->getActiveRobot() );
}

//------------------------------------------------------------------------------
//  Callback functions  
//------------------------------------------------------------------------------
void qt_drawAllWinActive()
{
    g3d_draw_allwin_active();
}

//static HRICS::Navigation* m_navigation = NULL;


bool qt_showTraj();
bool qt_showMotion( const Move3D::Trajectory& motion1, const Move3D::Trajectory& motion2 );
bool qt_showMotion2( const Move3D::Trajectory& motion1, const Move3D::Trajectory& motion2, bool save_video );

static bool save_images=true;

void qt_test1()
{
    Move3D::Robot* pr2      = global_Project->getActiveScene()->getRobotByName("PR2_ROBOT");
    Move3D::Robot* human    = global_Project->getActiveScene()->getRobotByName("HERAKLES_HUMAN1");

//    std::string pr2_traj_file = "/home/pr2command/catkin_ws/src/hrics-or-rafi/python_module/bioik/joint_state_traj.csv";
//    std::string human_traj_file = "/home/pr2command/workspace/test_mocap/no_split_human1_.csv";

    std::string folder = "/home/pr2command/workspace/test_mocap/ricoun_test/0";
    std::string pr2_traj_file   = folder + "/joint_state_traj.csv";
    std::string human_traj_file = folder + "/no_split_human1_.csv";

    cout << "load file 1 (pr2)" << endl;
    pr2_mapping mapping( pr2 );
    Move3D::Trajectory traj1 = mapping.load_trajectory( pr2_traj_file );

    cout << "load file 2 (human)" << endl;
    HRICS::RecordMotion rec_motion( human );
    rec_motion.useBioFormat( true );
    motion_t motion = rec_motion.loadFromCSV( human_traj_file );
    Move3D::Trajectory traj2 = HRICS::motion_to_traj( motion, human );


    qt_showMotion( traj1, traj2 );
//    qt_showMotion( traj1, Move3D::Trajectory() );




//    Move3D::Robot* active_human  = global_ht_simulator->getActiveHuman();
//    Move3D::Robot* passive_human = global_ht_simulator->getPassiveHuman();

//    if( active_human == NULL || passive_human == NULL ){
//        cout << "Did not get robots" << endl;
//        return;
//    }

//    active_human->getP3dRobotStruct()->tcur = NULL;

//    if( save_images )
//    {
//        global_w->getOpenGL()->setSaveOnDisk( false );
//        global_w->getOpenGL()->setSaveTraj( true );
//        save_images = false;
//    }
//    else
//    {
//        global_w->getOpenGL()->saveImagesToDisk();
//        global_w->getOpenGL()->setSaveTraj( false );
//    }






//    std::cout << "Load file to global motion recorder" << std::endl;

//    cout.precision(4);
//    cout << "motion duration : " << HRICS::motion_duration( global_motionRecorders[1]->getStoredMotions()[0] ) << endl;

//    Move3D::Trajectory active_traj  = global_ht_simulator->get
//    Move3D::Trajectory passive_traj = HRICS::motion_to_traj( global_motionRecorders[1]->getStoredMotions()[0], active_human );

//    if( !qt_showMotion( active_traj, passive_traj ) ){
//        return;
//    }
}

std::vector< std::vector<Move3D::Trajectory> > load_trajs(std::string folder, int nb_demos, int demo_id, Eigen::Vector3d color, bool draw )
{
    Move3D::Robot* active_human  = global_ht_simulator->getActiveHuman();
    std::vector< std::vector<Move3D::Trajectory> > trajs( nb_demos );

    int nb_runs = 10;

    for( int d=0; d<nb_demos; d++ )
        for( int k=0; k<nb_runs; k++ )
        {
            std::stringstream ss;
            ss.str("");
            ss << "run_simulator_" << std::setw(3) << std::setfill( '0' ) << d;
            ss <<              "_" << std::setw(3) << std::setfill( '0' ) << k << ".traj";


            Move3D::Trajectory traj( active_human );
            traj.loadFromFile( folder + ss.str() );

            cout << "loading trajectory : " << folder + ss.str() << " nb of waypoints : " << traj.getNbOfViaPoints() << endl;

            // traj.setColor( 0 );
            // double alpha = double(d)/double(nb_demos);

            if( d == demo_id && draw )
                global_linesToDraw.push_back( std::make_pair( color, traj.getJointPoseTrajectory( active_human->getJoint(45) ) ) );
            // global_trajToDraw.push_back( traj );
            trajs[d].push_back( traj );
        }

    return trajs;
}

std::vector<Move3D::Trajectory> demos;
std::vector< std::vector<Move3D::Trajectory> > baseline;
std::vector< std::vector<Move3D::Trajectory> > recovered;
std::vector< std::vector<Move3D::Trajectory> > noreplan_baseline;
std::vector< std::vector<Move3D::Trajectory> > noreplan_recovered;

void qt_test2()
{
//    Move3D::Scene* sce = global_Project->getActiveScene();
//    Move3D::Robot* robot = sce->getActiveRobot();

//    if( robot != NULL ){
//        cout << "Got robot" << endl;
//    }

//    Move3D::Trajectory traj(robot);
//    traj.loadFromFile("tmp_traj_file.m3dtraj");
//    traj.replaceP3dTraj();

    std::string folder_demos = "loo_trajectories/demos/";

//    std::string folder_base_line = "loo_trajectories/paper_icra_0/baseline/";
//    std::string folder_recovered = "loo_trajectories/paper_icra_0/recovered/";
//    std::string folder_no_replan_base_line = "loo_trajectories/paper_icra_0/no_replan_baseline/";
//    std::string folder_no_replan_recovered = "loo_trajectories/paper_icra_0/no_replan_recovered/";


    std::string folder_base_line = "loo_trajectories/with_collisions_final/zero_baseline_8/";
    std::string folder_recovered = "loo_trajectories/with_collisions_final/recovered_8/";
    std::string folder_no_replan_base_line = "loo_trajectories/with_collisions_final/no_replan_zero_baseline_8/";
    std::string folder_no_replan_recovered = "loo_trajectories/with_collisions_final/no_replan_recovered_8/";


    Move3D::Robot* active_human  = global_ht_simulator->getActiveHuman();
    Move3D::Robot* passive_human = global_ht_simulator->getPassiveHuman();

    active_human->getP3dRobotStruct()->tcur = NULL;

    std::vector<int> active_joint_id;
    std::vector<Move3D::Joint*> active_joints = global_ht_simulator->getActiveJoints();
    for( int i=0; i<int(active_joints.size()); i ++){
        active_joint_id.push_back( active_joints[i]->getId() );
        cout << "active_joints[" << i << "] : " << active_joints[i]->getName() << endl;
    }

    ChompPlanningGroup* plangroup = new ChompPlanningGroup( active_human, active_joint_id );

    int nb_demos = 8;
    int demo_id = HriEnv->getInt(HricsParam::ioc_sample_iteration); // TODO change that
    if( demo_id >= nb_demos ){
        demo_id = nb_demos-1;
    }

    global_linesToDraw.clear();

    if( global_DrawModule )
    {
        global_DrawModule->addDrawFunction( "Draw3DTrajs", boost::bind( &g3d_draw_3d_lines ) );
        global_DrawModule->enableDrawFunction( "Draw3DTrajs" );
    }

//    std::vector<int> active_dofs;

//    active joints dof (Pelvis) [0] : 6
//    active joints dof (TorsoX) [1] : 12
//    active joints dof (TorsoZ) [2] : 13
//    active joints dof (TorsoY) [3] : 14
//    active joints dof (rShoulderTransX) [4] : 18
//    active joints dof (rShoulderTransY) [5] : 19
//    active joints dof (rShoulderTransZ) [6] : 20
//    active joints dof (rShoulderY1) [7] : 21
//    active joints dof (rShoulderX) [8] : 22
//    active joints dof (rShoulderY2) [9] : 23
//    active joints dof (rArmTrans) [10] : 24
//    active joints dof (rElbowZ) [11] : 25
//    active joints dof (rElbowX) [12] : 26
//    active joints dof (rElbowY) [13] : 27
//    active joints dof (lPoint) [14] : 28
//    active joints dof (rWristZ) [15] : 29
//    active joints dof (rWristX) [16] : 30
//    active joints dof (rWristY) [17] : 31

//    active_dofs.push_back( 6 );
//    active_dofs.push_back( 7 );
//    active_dofs.push_back( 8 );
//    active_dofs.push_back( 9 );

//    active_dofs.push_back( 12 );
//    active_dofs.push_back( 13 );
//    active_dofs.push_back( 14 );

//    active_dofs.push_back( 21 );
//    active_dofs.push_back( 22 );
//    active_dofs.push_back( 23 );

//    active_dofs.push_back( 25 );
//    active_dofs.push_back( 26 );
//    active_dofs.push_back( 27 );

//    active_dofs.push_back( 29 );
//    active_dofs.push_back( 30 );
//    active_dofs.push_back( 31 );

//    global_ht_simulator->getActiveDofs();


    if( true || demos.empty() )
    {
        for( int d=0; d<nb_demos; d++ )
            //    if( true )
        {
            std::stringstream ss;
            ss.str("");
            ss << "trajectory_human_trajs_" << std::setw(3) << std::setfill( '0' ) << d;
            ss <<                       "_" << std::setw(3) << std::setfill( '0' ) << int(0) << ".traj";

            Move3D::Trajectory traj( active_human );
            traj.loadFromFile( folder_demos + ss.str() );

            cout << "loading trajectory : " << folder_demos + ss.str() << " nb of waypoints : " << traj.getNbOfViaPoints() << endl;

            traj.setColor( d );

//            double alpha = double(d)/double(nb_demos);

            if( d == demo_id )
                global_linesToDraw.push_back( std::make_pair( Eigen::Vector3d(1, 0, 0), traj.getJointPoseTrajectory( active_human->getJoint(45) ) ) );

            global_trajToDraw.push_back( traj );
            demos.push_back( traj );
        }
    }

//    int nb_runs = 10;

    if( true || baseline.empty() )
    {
        baseline            = load_trajs( folder_base_line, nb_demos, demo_id, Eigen::Vector3d(0, 1, 0), false );
        recovered           = load_trajs( folder_recovered, nb_demos, demo_id, Eigen::Vector3d(0, 0, 1), true );
        noreplan_baseline   = load_trajs( folder_no_replan_base_line, nb_demos, demo_id, Eigen::Vector3d(0, 1, 0), false );
        noreplan_recovered  = load_trajs( folder_no_replan_recovered, nb_demos, demo_id, Eigen::Vector3d(0, 1, 0), true );
    }

    // SET HUMAN CONFIGURAION

    cout << "size : " << global_ht_simulator->getDemonstrationsPassive()[demo_id].size() << endl;

    active_human->setAndUpdate( *demos[demo_id].getEnd() );
    passive_human->setAndUpdate( *global_ht_simulator->getDemonstrationsPassive()[demo_id].back().second );

    // Show trajectory
    // Comment to compute DTW
    Move3D::Trajectory passive_traj( HRICS::motion_to_traj( global_ht_simulator->getDemonstrationsPassive()[demo_id], passive_human ) );
    Move3D::Trajectory& active_traj = recovered[demo_id][9];
    global_linesToDraw.push_back( std::make_pair( Eigen::Vector3d(1, 0, 0), active_traj.getJointPoseTrajectory( active_human->getJoint(45) ) ) );

    cout << "passive human traj size : " << passive_traj.size() << endl;
//    qt_showMotion2( active_traj, passive_traj, true );
    return;

    std::vector< std::vector<Eigen::VectorXd> > costs( nb_demos );
    for( int d=0; d<nb_demos; d++ )
        costs[d].resize( 8 );

//    std::vector<Eigen::VectorXd> stats1( 4 );
//    for( int i=0; i<int(stats1.size()); i++ )
//        stats1[i] = Eigen::VectorXd::Zero( 2 );

    std::vector<Move3D::Joint*> joints;
    joints.push_back( active_human->getJoint(45) );

    active_joints.clear();
    active_joints.push_back( active_human->getJoint("Pelvis") );       // joint name : Pelvis
    active_joints.push_back( active_human->getJoint("TorsoX" )  );
    active_joints.push_back( active_human->getJoint("rShoulderX") );   // joint name : rShoulderX
    active_joints.push_back( active_human->getJoint("rElbowZ") );      // joint name : rElbowZ
    active_joints.push_back( active_human->getJoint("rWristX") );      // joint name : rWristX


    for( int d=0; d<nb_demos; d++ )
    {
        std::vector<double> costs_tmp = dtw_compare_performance( plangroup, demos[d], baseline[d], active_joints );
        costs[d][0] = Eigen::VectorXd::Zero( costs_tmp.size() );
        for( int k=0; k<int(costs_tmp.size()); k++ )
            costs[d][0][k] = costs_tmp[k];
    }

    for( int d=0; d<nb_demos; d++ )
    {
        std::vector<double> costs_tmp = dtw_compare_performance( plangroup, demos[d], baseline[d], joints  );
        costs[d][1] = Eigen::VectorXd::Zero( costs_tmp.size() );
        for( int k=0; k<int(costs_tmp.size()); k++ )
            costs[d][1][k] = costs_tmp[k];
    }


    for( int d=0; d<nb_demos; d++ )
    {
        std::vector<double> costs_tmp = dtw_compare_performance( plangroup, demos[d], recovered[d], active_joints );
        costs[d][2] = Eigen::VectorXd::Zero( costs_tmp.size() );
        for( int k=0; k<int(costs_tmp.size()); k++ )
            costs[d][2][k] = costs_tmp[k];
   }

    for( int d=0; d<nb_demos; d++ )
    {
        std::vector<double> costs_tmp = dtw_compare_performance( plangroup, demos[d], recovered[d], joints );
        costs[d][3] = Eigen::VectorXd::Zero( costs_tmp.size() );
        for( int k=0; k<int(costs_tmp.size()); k++ )
            costs[d][3][k] = costs_tmp[k];
    }

    for( int d=0; d<nb_demos; d++ )
    {
        std::vector<double> costs_tmp = dtw_compare_performance( plangroup, demos[d], noreplan_baseline[d], active_joints );
        costs[d][4] = Eigen::VectorXd::Zero( costs_tmp.size() );
        for( int k=0; k<int(costs_tmp.size()); k++ )
            costs[d][4][k] = costs_tmp[k];
    }

    for( int d=0; d<nb_demos; d++ )
    {
        std::vector<double> costs_tmp = dtw_compare_performance( plangroup, demos[d], noreplan_baseline[d], joints  );
        costs[d][5] = Eigen::VectorXd::Zero( costs_tmp.size() );
        for( int k=0; k<int(costs_tmp.size()); k++ )
            costs[d][5][k] = costs_tmp[k];
    }


    for( int d=0; d<nb_demos; d++ )
    {
        std::vector<double> costs_tmp = dtw_compare_performance( plangroup, demos[d], noreplan_recovered[d], active_joints );
        costs[d][6] = Eigen::VectorXd::Zero( costs_tmp.size() );
        for( int k=0; k<int(costs_tmp.size()); k++ )
            costs[d][6][k] = costs_tmp[k];
   }

    for( int d=0; d<nb_demos; d++ )
    {
        std::vector<double> costs_tmp = dtw_compare_performance( plangroup, demos[d], noreplan_recovered[d], joints );
        costs[d][7] = Eigen::VectorXd::Zero( costs_tmp.size() );
        for( int k=0; k<int(costs_tmp.size()); k++ )
            costs[d][7][k] = costs_tmp[k];
    }

    // SET HUMAN CONFIGURAION
    active_human->setAndUpdate( *demos[demo_id].getEnd() );
    passive_human->setAndUpdate( *global_ht_simulator->getDemonstrationsPassive()[demo_id].back().second );


    std::vector<std::string> names;
    names.push_back( "1 (1.31) & " );
    names.push_back( "2 (1.31) & " );
    names.push_back( "3 (1.40) & " );
    names.push_back( "4 (0.99) & " );
    names.push_back( "5 (0.90) & " );
    names.push_back( "6 (0.70) & " );
    names.push_back( "7 (1.11) & " );
    names.push_back( "8 & " );

    std::vector< std::vector<Eigen::VectorXd> > stats(nb_demos);

    for( int d=0; d<nb_demos; d++ )
    {
        stats[d].resize(8);

        for( int i=0; i<8; i++ )
        {
            double mean = costs[d][i].mean();
            double sq_sum = costs[d][i].transpose()*costs[d][i];
            double stdev = std::sqrt( sq_sum / double(costs[d][i].size()) - (mean * mean) ); // Moyenne de carrés moins le carré de la moyenne
            double min = costs[d][i].minCoeff();
            double max = costs[d][i].maxCoeff();

            stats[d][i].resize( 4 );
            stats[d][i][0] = mean;
            stats[d][i][1] = stdev;
            stats[d][i][2] = min;
            stats[d][i][3] = max;
         }

        // BASELINE
//        cout << stats[d][0][0] << "  " << stats[d][0][1] << "  " << stats[d][0][2] << "  " << stats[d][0][3] << "  " ;
//        cout << stats[d][1][0] << "  " << stats[d][1][1] << "  " << stats[d][1][2] << "  " << stats[d][1][3] << "  " ;

//        cout << stats[d][4][0] << "  " << stats[d][4][1] << "  " << stats[d][4][2] << "  " << stats[d][4][3] << "  " ;
//        cout << stats[d][5][0] << "  " << stats[d][5][1] << "  " << stats[d][5][2] << "  " << stats[d][5][3] << "  " << endl;

        // RECOVERED
        cout << stats[d][2][0] << "  " << stats[d][2][1] << "  " << stats[d][2][2] << "  " << stats[d][2][3] << "  " ;
        cout << stats[d][3][0] << "  " << stats[d][3][1] << "  " << stats[d][3][2] << "  " << stats[d][3][3] << "  " ;

        cout << stats[d][6][0] << "  " << stats[d][6][1] << "  " << stats[d][6][2] << "  " << stats[d][6][3] << "  " ;
        cout << stats[d][7][0] << "  " << stats[d][7][1] << "  " << stats[d][7][2] << "  " << stats[d][7][3] << "  " << endl;
    }

    cout << endl;

    for( int d=0; d<nb_demos; d++ )
    {
        stats[d].resize(8);

        for( int i=0; i<8; i++ )
        {
            double mean = costs[d][i].mean();
            double sq_sum = costs[d][i].transpose()*costs[d][i];
            double stdev = std::sqrt( sq_sum / double(costs[d][i].size()) - (mean * mean) ); // Moyenne de carrés moins le carré de la moyenne
            double min = costs[d][i].minCoeff();
            double max = costs[d][i].maxCoeff();

            stats[d][i].resize( 4 );
            stats[d][i][0] = mean;
            stats[d][i][1] = stdev;
            stats[d][i][2] = min;
            stats[d][i][3] = max;
         }

        cout << names[d] ;

        // BASELINE
//        cout << stats[d][0][0] << " & " << stats[d][0][1] << " & " << stats[d][0][2] << " & " << stats[d][0][3] << " & " ;
//        cout << stats[d][1][0] << " & " << stats[d][1][1] << " & " << stats[d][1][2] << " & " << stats[d][1][3] << " & " ;

//        cout << stats[d][4][0] << " & " << stats[d][4][1] << " & " << stats[d][4][2] << " & " << stats[d][4][3] << " & " ;
//        cout << stats[d][5][0] << " & " << stats[d][5][1] << " & " << stats[d][5][2] << " & " << stats[d][5][3] << " \\" << endl;

        // RECOVERED
        cout << stats[d][2][0] << " & " << stats[d][2][1] << " & " << stats[d][2][2] << " & " << stats[d][2][3] << " & " ;
        cout << stats[d][3][0] << " & " << stats[d][3][1] << " & " << stats[d][3][2] << " & " << stats[d][3][3] << " & " ;

        cout << stats[d][6][0] << " & " << stats[d][6][1] << " & " << stats[d][6][2] << " & " << stats[d][6][3] << " & " ;
        cout << stats[d][7][0] << " & " << stats[d][7][1] << " & " << stats[d][7][2] << " & " << stats[d][7][3] << "  " << endl;

    }


        double mean = costs[demo_id][0].mean();
        double sq_sum = costs[demo_id][0].transpose()*costs[demo_id][0];
        double stdev = std::sqrt( sq_sum / double(costs[demo_id][0].size()) - (mean * mean) ); // Moyenne de carrés moins le carré de la moyenne
        double min = costs[demo_id][0].minCoeff();
        double max = costs[demo_id][0].maxCoeff();

        double mean1 = costs[demo_id][1].mean();
        double sq_sum1 = costs[demo_id][1].transpose()*costs[demo_id][1];
        double stdev1 = std::sqrt( sq_sum1 / double(costs[demo_id][1].size()) - (mean1 * mean1) ); // Moyenne de carrés moins le carré de la moyenne
        double min1 = costs[demo_id][1].minCoeff();
        double max1 = costs[demo_id][1].maxCoeff();

        cout << "replanning baseline joint / task " << endl;
        cout << mean << " & " << stdev << " & " << min << " & " << max << " &  " << mean1 << " & " << stdev1 << " & " << min1 << " & " << max1 << endl;

        mean = costs[demo_id][2].mean();
        sq_sum = costs[demo_id][2].transpose()*costs[demo_id][2];
        stdev = std::sqrt( sq_sum / double(costs[demo_id][2].size()) - (mean * mean) ); // Moyenne de carrés moins le carré de la moyenne
        min = costs[demo_id][2].minCoeff();
        max = costs[demo_id][2].maxCoeff();

        mean1 = costs[demo_id][3].mean();
        sq_sum1 = costs[demo_id][3].transpose()*costs[demo_id][3];
        stdev1 = std::sqrt( sq_sum1 / double(costs[demo_id][3].size()) - (mean1 * mean1) ); // Moyenne de carrés moins le carré de la moyenne
        min1 = costs[demo_id][3].minCoeff();
        max1 = costs[demo_id][3].maxCoeff();
        cout << "replanning recovered joint / task " << endl;
        cout << mean << " & " << stdev << " & " << min << " & " << max << " &  " << mean1 << " & " << stdev1 << " & " << min1 << " & " << max1 << endl;


        mean = costs[demo_id][4].mean();
        sq_sum = costs[demo_id][4].transpose()*costs[demo_id][4];
        stdev = std::sqrt( sq_sum / double(costs[demo_id][4].size()) - (mean * mean) ); // Moyenne de carrés moins le carré de la moyenne
        min = costs[demo_id][4].minCoeff();
        max = costs[demo_id][4].maxCoeff();

        mean1 = costs[demo_id][5].mean();
        sq_sum1 = costs[demo_id][5].transpose()*costs[demo_id][5];
        stdev1 = std::sqrt( sq_sum1 / double(costs[demo_id][5].size()) - (mean1 * mean1) ); // Moyenne de carrés moins le carré de la moyenne
        min1 = costs[demo_id][5].minCoeff();
        max1 = costs[demo_id][5].maxCoeff();

        cout << "one iteration baseline joint / task " << endl;
        cout << mean << " & " << stdev << " & " << min << " & " << max << " &  " << mean1 << " & " << stdev1 << " & " << min1 << " & " << max1 << endl;

        mean = costs[demo_id][6].mean();
        sq_sum = costs[demo_id][6].transpose()*costs[demo_id][6];
        stdev = std::sqrt( sq_sum / double(costs[demo_id][6].size()) - (mean * mean) ); // Moyenne de carrés moins le carré de la moyenne
        min = costs[demo_id][6].minCoeff();
        max = costs[demo_id][6].maxCoeff();

        mean1 = costs[demo_id][7].mean();
        sq_sum1 = costs[demo_id][7].transpose()*costs[demo_id][7];
        stdev1 = std::sqrt( sq_sum1 / double(costs[demo_id][7].size()) - (mean1 * mean1) ); // Moyenne de carrés moins le carré de la moyenne
        min1 = costs[demo_id][7].minCoeff();
        max1 = costs[demo_id][7].maxCoeff();
        cout << "one iteration recovered joint / task " << endl;
        cout << mean << " & " << stdev << " & " << min << " & " << max << " &  " << mean1 << " & " << stdev1 << " & " << min1 << " & " << max1 << endl;

//    cout << endl;
//    cout << " MEAN mean  : "  << stats1[0].mean() << endl;
//    cout << " MEAN stdev : " << stats1[1].mean() << endl;
//    cout << " MEAN min   : "   << stats1[2].mean() << endl;
//    cout << " MEAN max   : "   << stats1[3].mean() << endl;

    //**********************************************************

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



//    cout << "Clear traj" << endl;
//    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();
//    p3d_destroy_traj( robot->getP3dRobotStruct(), robot->getP3dRobotStruct()->tcur );

//    if( global_rePlanningEnv != NULL )
//        global_rePlanningEnv->resetTrajectoriesToDraw();


}

//static bool recompute_cost=false;
//static bool init_generator=false;
//static bool use_list_generator=false;
//static ConfGenerator* generatorPtr=NULL;

void qt_test3()
{
    //*****************************************************************
    /// READ TRAJS

    std::stringstream ss;
    std::vector<std::string> files;
    std::string cur_dir(get_current_dir_name());

    cout << cur_dir << endl;

    int nb_trajs = 12;
    for(int i=0; i<nb_trajs; i++ )
    {
        ss.str("");
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";
        files.push_back( cur_dir + "/" + ss.str() );
    }

    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();
    Move3D::SequencesPlanners pool( robot );
    pool.loadTrajsFromFile( files );
    pool.playTrajs();


    //*****************************************************************
    /// IK
//    Move3D::Robot* object = global_Project->getActiveScene()->getRobotByNameContaining( "VISBALL" );
//    if( object == NULL )
//        return;

////    HRICS_activeNatu->computeIsReachableAndMove( object->getJoint(1)->getVectorPos(), false );

//    Move3D::Robot* robot = global_Project->getActiveScene()->getRobotByName("BIOMECH_HUMAN");
//    if( !robot ){
//        cout << "no human" << endl;
//        robot = global_Project->getActiveScene()->getRobotByName("manip_3dofs_ROBOT");
//        if( !robot ){
//            cout << "no robot" << endl;
//            return;
//        }
//    }

//    Move3D::Joint* eef = NULL;
//    if( HRICS_activeNatu != NULL )
//    {
//        eef = robot->getJoint("rPalm");
//        if( eef == NULL )
//            return;
//    }
//    else
//    {
//        eef = robot->getJoint("J4");
//        if( eef == NULL )
//            return;
//    }

//    std::vector<Move3D::Joint*> active_joints;
//    //    active_joints.push_back( robot->getJoint( "Pelvis" ) );
//    active_joints.push_back( robot->getJoint( "TorsoX" ) );
//    active_joints.push_back( robot->getJoint( "TorsoZ" ) );
//    active_joints.push_back( robot->getJoint( "TorsoY" ) );
//    active_joints.push_back( robot->getJoint( "rShoulderTransX" ) );
//    active_joints.push_back( robot->getJoint( "rShoulderTransY" ) );
//    active_joints.push_back( robot->getJoint( "rShoulderTransZ" ) );
//    active_joints.push_back( robot->getJoint( "rShoulderY1" ) );
//    active_joints.push_back( robot->getJoint( "rShoulderX" ) );
//    active_joints.push_back( robot->getJoint( "rShoulderY2" ) );
//    active_joints.push_back( robot->getJoint( "rArmTrans" ) );
//    active_joints.push_back( robot->getJoint( "rElbowZ" ) );
//    active_joints.push_back( robot->getJoint( "rElbowX" ) );
//    active_joints.push_back( robot->getJoint( "rElbowY" ) );
//    active_joints.push_back( robot->getJoint( "lPoint" ) );
//    active_joints.push_back( robot->getJoint( "rWristZ" ) );
//    active_joints.push_back( robot->getJoint( "rWristX" ) );
//    active_joints.push_back( robot->getJoint( "rWristY" ) );

//    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "rShoulderTransX" )->getP3dJointStruct(), 0, .016, .020 );
//    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "rShoulderTransY" )->getP3dJointStruct(), 0, .32, .34 );
//    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "rShoulderTransZ" )->getP3dJointStruct(), 0, .24, .26 );
//    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "rArmTrans" )->getP3dJointStruct(), 0, .38, .40 );
//    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "lPoint" )->getP3dJointStruct(), 0, .23, .25 );

//    Eigen::VectorXd xdes = object->getJoint(1)->getXYZPose();
//    //    Eigen::VectorXd xdes = object->getJoint(1)->getVectorPos();

//    Move3D::confPtr_t q_tmp = robot->getCurrentPos();
//    //    robot->setAndUpdate( *HRICS_activeNatu->getComfortPosture() );
//    //    q_tmp = HRICS_activeNatu->getComfortPosture()->copy();

//    bool succeed = false;
//    bool simple_ik = false;
//    if( !simple_ik )
//    {
//        Move3D::IKGenerator ik( robot );
//        ik.initialize( active_joints, eef );

//        succeed = ik.generate( xdes );
//        cout << "diff = " << ( xdes - eef->getXYZPose() ).norm() << endl;

//        if( !succeed ){
//            robot->setAndUpdate( *q_tmp );
//        }
//    }

//    int nb_iter = 200;

//    global_w->getOpenGL()->setSaveOnDisk( false );
//    global_w->getOpenGL()->setSaveTraj( true );

//    for( int i=0; i<113; i++)
//    {
//        g3d_rotate_win_camera_rz( G3D_WIN->vs, M_PI/double(nb_iter) );
//        g3d_draw_allwin_active();
//        usleep(1000);
//    }

//    global_w->getOpenGL()->saveImagesToDisk();
//    global_w->getOpenGL()->setSaveTraj( false );
//*****************************************************************






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
    //    p3d_rob* rob = robot->getP3dRobotStruct();
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
    //    p3d_rob* rob = robot->getP3dRobotStruct();
    //    for(int i=0; i<rob->cntrt_manager->ncntrts; i++) {
    //      p3d_cntrt* ct = rob->cntrt_manager->cntrts[i];
    //      p3d_desactivateCntrt( rob, ct );
    //    }
    //
    //    cout << "Ik computed " << endl;
    //    confPtr_t q_rob(new Configuration(robot,q));
    //
    //    // Generate trajectory
    //    Move3D::CostOptimization optim( robot->getCurrentTraj() );
    //    double step = optim.getParamMax() / 10;
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
        if( API_activeGraph )
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
    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();

    // Remove displayed trajectory
    p3d_destroy_traj( robot->getP3dRobotStruct(), robot->getP3dRobotStruct()->tcur );

    try
    {
#ifdef P3D_PLANNER
        p3d_SetStopValue(FALSE);
#endif
        timeval tim;
        gettimeofday(&tim, NULL);
        double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);

        cout << "ENV.getBool(Env::Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
        if (ENV.getBool(Env::treePlannerIsEST))
        {
            p3d_run_est( robot->getP3dRobotStruct() );
        }
        else
        {
            p3d_run_rrt( robot->getP3dRobotStruct() );
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
    }
    catch (...)
    {
        cerr << "Exeption in run qt_runDiffusion" << endl;
    }

    // TEST
//    if( global_activeFeatureFunction != NULL )
//        global_activeFeatureFunction->extractAllTrajectories( API_activeGraph, robot->getInitPos(), robot->getGoalPos(), 10 );
}

/**
 * Run PRM
 */
void qt_runPRM()
{
    cout << "Run Probabilistic Road Map" << endl;
    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();

    // Remove displayed trajectory
    p3d_destroy_traj( robot->getP3dRobotStruct(), robot->getP3dRobotStruct()->tcur );

    try
    {
#ifdef P3D_PLANNER
        p3d_SetStopValue(FALSE);
#endif
        ChronoOn();

        switch(ENV.getInt(Env::PRMType))
        {
        case 0:
        case 1:
        case 2:
        case 4:
        case 5:
            p3d_run_prm(robot->getP3dRobotStruct());
            break;

        case 3:
            p3d_run_perturb_prm(robot->getP3dRobotStruct());
            break;
        default:
            cout << "Error No Other PRM"  << endl;
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

    // TEST
//    if( global_activeFeatureFunction != NULL )
//        global_activeFeatureFunction->extractAllTrajectories( API_activeGraph, robot->getInitPos(), robot->getGoalPos(), 7 );
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

void qt_runPlanSequence()
{
    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();
    Move3D::SequencesPlanners pool(robot);
    pool.setPlannerType( Move3D::stomp );

    std::vector<Move3D::confPtr_t> configs = pool.getStoredConfig();
    // Config 0 -> green
    // Config 1 -> yellow
    // Config 2 -> red
    // Config 3 -> center area

    std::vector<Move3D::confPtr_t> sequence;
    sequence.push_back( configs[3] );
    sequence.push_back( configs[0] );
    sequence.push_back( configs[3] );
    sequence.push_back( configs[1] );
    sequence.push_back( configs[3] );
    sequence.push_back( configs[0] );
    sequence.push_back( configs[3] );
    sequence.push_back( configs[1] );
    sequence.push_back( configs[3] );
    sequence.push_back( configs[0] );
    sequence.push_back( configs[3] );
    sequence.push_back( configs[2] );
    sequence.push_back( configs[3] );
    sequence.push_back( configs[0] );
    sequence.push_back( configs[3] );
    sequence.push_back( configs[1] );
    sequence.push_back( configs[3] );
    sequence.push_back( configs[0] );

    pool.runSequence( sequence );
    pool.playTrajs();
    pool.saveTrajsToFile( "." );
}

void qt_runAStarPlanning()
{
    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();
    Move3D::confPtr_t q_init = robot->getInitPos();
    Move3D::confPtr_t q_goal = robot->getGoalPos();

    // Warning leaking
    Move3D::AStarPlanner* planner = new Move3D::AStarPlanner(robot);
    planner->set_pace( PlanEnv->getDouble(PlanParam::grid_pace) );
    planner->init();

    Move3D::Trajectory* traj = planner->computeRobotTrajectory( q_init, q_goal );
    if( traj != NULL )
    {
        traj->replaceP3dTraj();
    }
    else{
        cout << "Error in run AStar planner" << endl;
    }
}

void qt_runAStarInCurrentGraph()
{
    if( API_activeGraph)
    {
        Move3D::Robot* robot = API_activeGraph->getRobot();

        // API_activeGraph->extractBestAStarPathSoFar( robot->getInitPos(), robot->getGoalPos() );
        API_activeGraph->extractDijkstraShortestPathsTraj( robot->getInitPos(), robot->getGoalPos() );

        g3d_draw_allwin_active();
    }
    else cout << "Graph Empty" << endl;
}

void qt_sampleGraph()
{
    cout << "Sample Graph" << endl;
    graphSampler sampler;
    sampler.initialize();
    sampler.sample();
}

void qt_makeGridGraph()
{
    cout << "Make Grid Graph" << endl;
    graphSampler sampler;
    sampler.makeGrid(6);
}

void qt_generateDetours()
{
    cout << "Sample Graph" << endl;
    HRICS::Detours* det = new HRICS::Detours;
    det->planAStar();
}

void qt_extractAllTrajectories()
{
    ENV.setBool(Env::drawGraph,false);
    ENV.setBool(Env::drawTrajVector,false);
    ENV.setBool(Env::drawGrid,false);

    if( dynamic_cast<PlanarFeature*>(global_activeFeatureFunction) != NULL ){
        dynamic_cast<PlanarFeature*>(global_activeFeatureFunction)->generateRandomEnvironment();
    }

    g3d_draw_allwin_active();

    ChronoTimeOfDayOn();

    if( API_activeGraph == NULL )
        qt_runPRM();
    else
        API_activeGraph->resetAllEdgesFeatures();

    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();
    robot->removeCurrentTraj();

    if( global_activeFeatureFunction != NULL )
        global_activeFeatureFunction->extractAllTrajectories( API_activeGraph, robot->getInitPos(), robot->getGoalPos(), 7 );

    double time;
    ChronoTimeOfDayTimes( &time );
    ChronoTimeOfDayOff();

    cout << "time to compute : " << time << endl;

    ENV.setBool(Env::drawTrajVector,true);
    ENV.setBool(Env::drawGrid,true);

    if( robot->getNumberOfActiveDoF() == 2 )
    {
        Move3D::WeightVect w( Move3D::WeightVect::Constant( global_activeFeatureFunction->getNumberOfFeatures(), 0.5 ));
        cout << " w.transpose() : " << w.transpose() << endl;
        global_activeFeatureFunction->setWeights( w );

        // Create grid and set as active grid
        Move3D::PlanGrid* grid = new Move3D::PlanGrid( robot, PlanEnv->getDouble(PlanParam::grid_pace),
                                                       global_Project->getActiveScene()->getBounds(), false );

        grid->setCostBounds( 0.0, 1.0 );

        if( API_activeGrid != NULL ) delete API_activeGrid;
        API_activeGrid = grid;
    }

    g3d_draw_allwin_active();
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

bool remove_motion;

void qt_show_recorded_motion()
{
    if( global_motionRecorders.empty() ) {
        cout << "recorder not initialized" << endl;
        return;
    }

    if( global_motionRecorders.size() == 1 )
    {
        global_motionRecorders[0]->showStoredMotion();
        cout << "End recorded motion" << endl;
    }
    else
    {
        cout << "Start motion player" << endl;

        HRICS::PlayMotion* player = NULL;

        if( global_ht_simulator == NULL )
        {
            cout << "USE MOTION RECORDER" << endl;
            player = new HRICS::PlayMotion( global_motionRecorders );
        }
        else
        {
            cout << "USE MOTION VECTOR" << endl;
            player = new HRICS::PlayMotion( global_ht_simulator->getMotions() );
            player->setMotionsNames( global_ht_simulator->getMotionsNames() );
        }

        if( player->getNumberOfMotions() == 0 ){
            cout << "no motions to play" << endl;
        }

        std::vector<std::string> names;

        //int i=3;
        for( int i=0; i<player->getNumberOfMotions(); i++ )
        {
            if( global_ht_simulator == NULL )
            {
                cout << "play motion " << i;
                cout << " , from file : " << global_motionRecorders[0]->getStoredMotionName(i) << endl;
            }

//            if ( i > 0 ){
//                break;
//            }

            player->play(i);


            if( GestEnv->getBool( GestParam::play_repeat ) ){
                i--;
            }
            bool use_button = false;
            while( !GestEnv->getBool(GestParam::play_next) ) {
                usleep(100);
                use_button = true;
            }
            if( use_button ){
                GestEnv->setBool( GestParam::play_next, false );
            }

            else if( global_ht_simulator == NULL )
            {
                if( remove_motion ){
                    names.push_back( global_motionRecorders[0]->getStoredMotionName(i) );
                    remove_motion = false;
                }
            }
        }

        if( !names.empty() )
        {
            cout << "remove" << endl;
            for( size_t i=0; i<names.size(); i++ )
            {
                cout << names[i] << endl;
            }
        }
    }

    cout << "End!!!" << endl;
}

void qt_workspace_occupancy()
{
    cout << "Loading regressed motion and computing the occupancy" << endl;
    std::string foldername = "/home/jmainpri/Dropbox/workspace/gesture-recognition/gmm/gmm-gmr-gesture-recognition/";
    global_motionRecorders[0]->loadRegressedFromCSV( foldername );
    global_workspaceOccupancy->setRegressedMotions( global_motionRecorders[0]->getStoredMotions(), true );
    global_workspaceOccupancy->computeOccpancy();
}

void qt_classify_motions()
{
    if( global_humanPredictionSimulator == NULL || global_motionRecorders.empty() )
    {
        cout << "global_humanPredictionSimulator or global_motionRecorder are not initilized" << endl;
        return;
    }

    std::string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/";
    global_motionRecorders[0]->loadXMLFolder();

    const std::vector<motion_t>& stored_motions = global_motionRecorders[0]->getStoredMotions();

    for(int i=0;i<8;i++)
    {
        for(int j=0;j<5;j++)
        {
            int index = i*25+j;
            cout << std::setw( 3 ) << std::setfill( '0' ) << index  << " : " ;
            /*int id_class =*/ global_humanPredictionSimulator->classifyMotion( global_motionRecorders[0]->resample( stored_motions[index], 100 ) );
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
void qt_runIOC()
{
    // TODO BUG WITH THE destructor...

//    HRICS::IocSequences* ioc = new HRICS::IocSequences;
//    ioc->run();

//    global_w->getOpenGL()->setSaveOnDisk( false );
//    global_w->getOpenGL()->setSaveTraj( true );


    HRICS::IocSequences ioc;
    ioc.run();

    cout << "End IOC RUN!!!" << endl;

    if( HriEnv->getBool(HricsParam::ioc_exit_after_run) )
    {
        cout << "exit move3d" << endl;
        exit(1);
    }

//    global_w->getOpenGL()->saveImagesToDisk();
//    global_w->getOpenGL()->setSaveTraj( false );
}

//void qt_runHumanIOC()
//{
//    HRICS_run_human_ioc_evaluation();
//    HRICS_run_human_ioc();
//}

//----------------------------------------------------------
// Navigation Functions
//----------------------------------------------------------
HRICS::Navigation* navPlanner = NULL;

void qt_computeAStar()
{
    cout << "Compute AStar" <<  endl;
    Move3D::Robot* rob;

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
    if( PlanEnv->getBool(PlanParam::trajStompRunMultiple) )
    {
        cout << "RUN MULTIPLE PARALLEL STOMP" << endl;
        stomp_motion_planner::srompRun_MultipleParallel();

//        HRICS::MultiplePlanners planner( global_Project->getActiveScene()->getRobotByNameContaining("ROBOT") );
//        planner.loadTrajsFromFile( "/home/jmainpri/workspace/move3d/move3d-launch/matlab/stomp_trajs/per_feature_square/ASTAR" );
//        planner.multipleRun( 100 );
    }
    else
    {
        cout << "RUN ONE PARALLEL STOMP" << endl;
        stomp_motion_planner::srompRun_OneParallel();
    }
}

void qt_runStomp()
{
    cout << "-----------------------------------------------------------" << endl;

    if( !PlanEnv->getBool(PlanParam::trajStompRunParallel) )
    {
        cout << "RUN NORMAL STOMP" << endl;
    }
    else {
        qt_runParallelStomp();
        return;
    }

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
    cout << "-----------------------------------------------------------" << endl;
    cout << "RUN NORMAL STOMP (no reset)" << endl;

    if( traj_optim_runStompNoReset(0) )
    {
        cout << "Stomp has run succesfully!!!" << endl;
    }
    else {
        cout << "Stomp fail!!!" << endl;
    }
}
#ifdef MULTILOCALPATH
void qt_init_mlp_cntrts_and_fixjoints() 
{
    traj_optim_init_mlp_cntrts_and_fix_joints(  global_Project->getActiveScene()->getActiveRobot() );
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

bool qt_showMotion2( const Move3D::Trajectory& motion1, const Move3D::Trajectory& motion2, bool save_video )
{
    cout << __PRETTY_FUNCTION__ << endl;

    if( motion1.size() == 0 ) {
        cout << "warning : motion is empty" << endl;
        return false;
    }

    cout << "time length : " << motion1.getTimeLength() << endl;

    Move3D::Robot* robot = motion1.getRobot();
    bool StopRun = false;
    double t = 0.0;
    double delta = 0.01;

    if( save_video )
    {
        global_w->getOpenGL()->setSaveOnDisk( false );
        global_w->getOpenGL()->setSaveTraj( true );
    }

    while ( !StopRun )
    {
        robot->setAndUpdate( *motion1.configAtTime( t ) );

        if( motion2.size() > 1 )
        {
            cout << "Set robot 2 in configuration" << endl;
            motion2.getRobot()->setAndUpdate( *motion2.configAtTime( t ) );
        }

        g3d_draw_allwin_active();

        t += delta;

        if ( t >= motion1.getTimeLength() )
            StopRun = true;

        if ( PlanEnv->getBool(PlanParam::stopPlanner) )
            StopRun = true;
    }

    if( save_video )
    {
        global_w->getOpenGL()->saveImagesToDisk();
        global_w->getOpenGL()->setSaveTraj( false );
    }

    cout << "end " << __PRETTY_FUNCTION__ << endl;

    return true;
}

bool qt_showMotion( const Move3D::Trajectory& motion1, const Move3D::Trajectory& motion2 )
{
    cout << __PRETTY_FUNCTION__ << endl;

    if( motion1.size() == 0 ) {
        cout << "warning : motion is empty" << endl;
        return false;
    }

    cout << "time length : " << motion1.getTimeLength() << endl;

    Move3D::Robot* robot = motion1.getRobot();
    bool StopRun = false;
    double tu_init = 0.0, tu = 0.0, t = 0.0;
    timeval tim;

    bool record_video = false;

    if( record_video )
        global_w->getOpenGL()->setSaveOnDisk( false );

    while ( !StopRun )
    {
        t = tu - tu_init;

        robot->setAndUpdate( *motion1.configAtTime( t ) );

        if( motion2.size() > 1 )
        {
            cout << "Set robot 2 in configuration" << endl;
            motion2.getRobot()->setAndUpdate( *motion2.configAtTime( t ) );
        }

//        bool ncol = false;

//        if( global_collisionSpace )
//        {
//            double distance = numeric_limits<double>::max();
//            double potential = numeric_limits<double>::max();

//            ncol = global_collisionSpace->isRobotColliding( distance, potential );

////            cout << "Distance to nearest obstacle = " << distance << " and potential = " << potential << endl;

//            if( global_optimizer && ( global_optimizer->getRobot() == robot ) ) {
//                global_optimizer->getCollisionSpaceCost( *robot->getCurrentPos() );
//            }
//        }
//        else
//        {
//            ncol = robot->isInCollision();
//        }

//        if( ncol ){
//            cout << "Robot in collision " << endl;
//        }

        //HRICS::setThePlacemateInIkeaShelf();
//        g3d_set_draw_coll( ncol );

        if( record_video )
            global_w->getOpenGL()->addCurrentImage();

        g3d_draw_allwin_active();

        gettimeofday(&tim, NULL);
        tu = tim.tv_sec + (tim.tv_usec/1000000.0);

        if( tu_init == 0.0 )
            tu_init = tu;

        if ( t >= motion1.getTimeLength() )
            StopRun = true;

        if ( PlanEnv->getBool(PlanParam::stopPlanner) )
            StopRun = true;
    }

    if( record_video )
        global_w->getOpenGL()->saveImagesToDisk();

    cout << "end " << __PRETTY_FUNCTION__ << endl;

    return true;
}

bool qt_showTraj()
{
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

    Move3D::Robot* rob = global_Project->getActiveScene()->getRobotByNameContaining( robotPt->name );
    if( rob->getCurrentMove3DTraj().size() != 0 ) {
        qt_showMotion( rob->getCurrentMove3DTraj(), Move3D::Trajectory() );
        ENV.setBool(Env::isRunning ,false);
        return true;
    }
    else {
        cout << "no getCurrentMove3DTraj for robot :  " << rob->getName() << endl;
    }

    g3d_show_tcur_rob( robotPt, default_drawtraj_fct_qt_pipe );
    ENV.setBool(Env::isRunning,false);

    if (PlanEnv->getBool(PlanParam::env_showHumanTraj))
    {
        p3d_rob *hum_robotPt;
        Move3D::Robot* rob = global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");
        if (rob)
        {
            hum_robotPt = rob->getP3dRobotStruct();
            // dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,false);
            double tmp = ENV.getDouble(Env::showTrajFPS);
            ENV.setDouble(Env::showTrajFPS,tmp*5);
            g3d_show_tcur_rob(hum_robotPt,default_drawtraj_fct_qt_pipe);
            ENV.setDouble(Env::showTrajFPS,tmp);
            // dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showBestConf();
        }
    }

    return true;
}

/**
 * Shortcut optimization
 */
void qt_shortCut()
{
    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();

    cout << "Random shortCut for robot : " << robot->getName() << endl;
    Move3D::Smoothing optimTrj(robot->getCurrentTraj());
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
    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();

    cout << "Random deformation for robot : " << robot ->getName()<< endl;
    Move3D::CostOptimization optimTrj(robot->getCurrentTraj());
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
    Move3D::Robot* trajRobot = new Move3D::Robot(robotPt);
    Move3D::CostOptimization optimTrj(trajRobot,CurrentTrajPt);

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
    Move3D::Robot* trajRobot = new Move3D::Robot(robotPt);
    Move3D::CostOptimization optimTrj(trajRobot,CurrentTrajPt);
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

void qt_recomputeValidEdges()
{
    if( API_activeGraph )
    {
        // Graph* tmpGraph = new Graph( *API_activeGraph );

        API_activeGraph->resetAllEdgesValid();

        if( API_activeGraph->checkAllEdgesValid() )
        {
            cout << "Graph valid" << endl;
        }
        else {
            cout << "Graph Not valid" << endl;
        }
    }
    else cout << "Graph Empty" << endl;
}

#ifdef LIGHT_PLANNER
/**
 * Make traj from via points
 */
void qt_makeTrajFromViaPoints()
{
    //    p3d_rob * robotPt = p3d_get_robot_by_name("PR2_ROBOT");//justin//JIDOKUKA_ROBOT
    p3d_rob * robotPt =  global_Project->getActiveScene()->getActiveRobot()->getP3dRobotStruct();
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
    //int ir;
    configPt qi, qf;
    pp3d_traj trajPt;

    if (qt_fileName!=NULL)
    {
        if (p3d_read_traj(qt_fileName))
        {
            trajPt = (p3d_traj *) p3d_get_desc_curid(P3D_TRAJ);
            // ir = p3d_get_desc_curnum(P3D_ROBOT);
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

    Move3D::Robot* human = global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");

    HRICS_activeNatu  = new HRICS::Natural( human );

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

void PlannerHandler::setExternalFunction(boost::function<void(void)> f)
{
    func_ = f;
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
//    try
    {
        if(plannerName == "BoostFunction")
        {
            std::cout << "Planning thread : starting boost function." << std::endl;
            func_();
        }
        else if(plannerName == "Diffusion")
        {
            std::cout << "Planning thread : starting diffusion." << std::endl;
            qt_runDiffusion();
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
        else if(plannerName == "PlanSequence")
        {
            std::cout << "Planning thread : starting plan in sequence." << std::endl;
            qt_runPlanSequence();
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
        else if (plannerName == "checkAllEdges")
        {
            qt_recomputeValidEdges();
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
        else if( plannerName == "initMlpCntrtsAndFixJoints" ){
            qt_init_mlp_cntrts_and_fixjoints();
        }
#endif
        else if( plannerName == "ShowRecordedMotion")
        {
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
        else if( plannerName == "SphereIOC" ||  plannerName == "HumanIOC"  || plannerName == "RunIOC" )
        {
            qt_runIOC();
            cout << "END : qt_runIOC" << endl;
//            qt_runShereIOC();
//            qt_runHumanIOC();
        }
        else if( plannerName == "HumanPlanning" )
        {
//            qt_runHumanPlanning();
            cout << "Not implemented anymore" << endl;
        }
        else if( plannerName == "AStarPlanning" )
        {
            qt_runAStarPlanning();
        }
        else if( plannerName == "ComputeAStarInCurrentGraph" )
        {
            qt_runAStarInCurrentGraph();
        }
        else if( plannerName == "SampleGraph" )
        {
            qt_sampleGraph();
        }
        else if( plannerName == "MakeGridGraph" )
        {
            qt_makeGridGraph();
        }
        else if( plannerName == "Detours" )
        {
            qt_generateDetours();
        }
        else if(plannerName == "ExtractAllTrajectories" )
        {
            qt_extractAllTrajectories();
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
//    catch(std::string what)
//    {
//        std::cerr << "Planner thread : caught exception : " << what << std::endl;
//    }
//    catch(std::exception& e)
//    {
//        std::cerr << "Planner thread : caught exception : " << e.what() << std::endl;
//    }
//    catch(...)
//    {
//        std::cerr << "Planner thread : caught exception of unknown type." << std::endl;
//    }


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

