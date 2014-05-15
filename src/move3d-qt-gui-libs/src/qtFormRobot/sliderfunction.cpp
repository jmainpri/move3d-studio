//
//  sliderfunction.cpp
//  MOVE3DSTUDIO
//
//  Created by Jim Mainprice on 20/10/11.
//  Copyright 2011 LAAS-CNRS. All rights reserved.
//

#include "sliderfunction.hpp"

#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"

#include "collision_space/collision_space.hpp"

#ifdef HRI_COSTSPACE
#include "hri_costspace/HRICS_costspace.hpp"
#include "hri_costspace/HRICS_miscellaneous.hpp"
#endif

#include "feature_space/features.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "API/Device/robot.hpp"
#include "API/project.hpp"

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Collision-pkg.h>

#include <sys/time.h>

using namespace Move3D;
using namespace std;

MOVE3D_USING_SHARED_PTR_NAMESPACE

static bool recompute_cost=true;
static bool init_generator=false;
static bool use_list_generator=false;
static ConfGenerator* generatorPtr=NULL;

void qt_set_arm_along_body( Robot* robot )
{  
    confPtr_t q_curr = robot->getCurrentPos();
    confPtr_t q_init = robot->getInitPos();
    (*q_init)[6] =  (*q_curr)[6];
    (*q_init)[7] =  (*q_curr)[7];
    (*q_init)[8] =  (*q_curr)[8];
    (*q_init)[9] =  (*q_curr)[9];
    (*q_init)[10] = (*q_curr)[10];
    (*q_init)[11] = (*q_curr)[11];
    robot->setAndUpdate( *q_init );
}

void qt_gik( p3d_rob* robot, const Eigen::Vector3d& point )
{  
    if( GLOBAL_AGENTS == NULL ) {
        cout << "No GLOBAL_AGENTS in " << __func__ << endl;
        return;
    }

    // 1 - Select Goto point
    p3d_vector3 Tcoord;
    Tcoord[0] = point[0];
    Tcoord[1] = point[1];
    Tcoord[2] = point[2];

    // 2 - Select Task
    HRI_GIK_TASK_TYPE task;
    const bool leftArm = false;

    if (leftArm == true)
        task = GIK_LATREACH; // Left Arm GIK
    else
        task = GIK_RATREACH; // Right Arm GIK

    HRI_AGENT* agent = hri_get_agent_by_name( GLOBAL_AGENTS, robot->name );
    double distance_tolerance = 0.05;
    configPt q = p3d_get_robot_config(agent->robotPt);

    if( hri_agent_single_task_manip_move(agent, task, &Tcoord, distance_tolerance, &q) )
    {
        cout << "GIK for " << agent->robotPt->name << " succeded" << endl;
        p3d_set_and_update_this_robot_conf(agent->robotPt, q);
    }
    else {
        cout << "GIK for " << agent->robotPt->name << " failed"<< endl;
    }
}

void qt_set_otp_cost_recompute()
{
    recompute_cost = true;
}

void qt_otp()
{
    //  HRICS_humanCostMaps->testCostFunction();
    //  HRICS_humanCostMaps->saveAgentGrids();

    timeval tim;
    gettimeofday(&tim, NULL);
    double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);

    Scene* sce = global_Project->getActiveScene();
    Robot* robot = sce->getRobotByNameContaining("ROBOT");
    Robot* human = sce->getRobotByNameContaining("HUMAN");

    if( !init_generator )
    {
        string dir(getenv("HOME_MOVE3D"));
        string file("/statFiles/OtpComputing/confHerakles.xml");

        generatorPtr = new ConfGenerator( robot, human );
        generatorPtr->initialize( dir+file, HRICS_activeNatu );

        init_generator = true;
    }

    if( use_list_generator )
    {
        // Compute LIST
        pair<confPtr_t,confPtr_t> best_handover_conf;
        double best_cost=0.0;

        if( generatorPtr->computeHandoverConfigFromList( best_handover_conf, best_cost ))
        {
            human->setAndUpdate( *best_handover_conf.first );
            robot->setAndUpdate( *best_handover_conf.second );
        }
        cout << "Cost of configuration is : " << best_cost << endl;
    }
    else
    {
        // Compute IK
        std::vector<Eigen::Vector3d> points;
        HRICS_humanCostMaps->getHandoverPointList( points, recompute_cost, true );
        recompute_cost = false;

        gettimeofday(&tim, NULL);
        double time = tim.tv_sec + (tim.tv_usec/1000000.0) - t_init;
        cout << "time after getHandoverPointList : " << time << endl;

        qt_set_arm_along_body( human );

        configPt q;
        bool found_ik = false;
        int i=0;

        for (i=0; i<int(points.size()) && found_ik==false && time<0.5; i++)
        {
            found_ik = generatorPtr->computeRobotIkForGrabing( q, points[i] );
            gettimeofday(&tim, NULL);
            time = tim.tv_sec + (tim.tv_usec/1000000.0) - t_init;
        }

        if( found_ik ) {
            HRI_AGENT* robot_agent = hri_get_agent_by_name( GLOBAL_AGENTS, robot->getName().c_str() );
            hri_activate_coll_robot_and_all_humans_arms( robot_agent, GLOBAL_AGENTS, false );
            //qt_gik( human->getP3dRobotStruct(), points[i] );
        }
        cout << "Ik(" << found_ik << ") at iteration : " << i << endl;

        // Disable cntrts
        p3d_cntrt* ct;
        p3d_rob* rob = robot->getP3dRobotStruct();
        for(int i=0; i<rob->cntrt_manager->ncntrts; i++) {
            ct = rob->cntrt_manager->cntrts[i];
            p3d_desactivateCntrt( rob, ct );
        }

        if( found_ik ) {
            confPtr_t target_new_ = confPtr_t(new Configuration( robot, q ));
            robot->setAndUpdate( *target_new_ );
        }
    }

    gettimeofday(&tim, NULL);
    double dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
    cout << "Handover computed in : " << dt << " sec" << endl;
}

//! function that computes a cost 
//! on the robot actual configuration
void qtSliderFunction(p3d_rob* robotPt, configPt p)
{  
    // No cost is computed for the other robots
    if (ENV.getBool(Env::isCostSpace) && (XYZ_ENV->active_robot == robotPt ) )
    {
#ifdef P3D_PLANNER
        p3d_rob* costRobot = robotPt;
        configPt cost_q = p;
#ifdef HRI_COSTSPACE
        // Compute kinematic the object transfer point
        if ( ENV.getBool(Env::HRIComputeOTP) )
        {
            Eigen::Vector3d WSPoint;

            if( dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computeBestFeasableTransferPoint(WSPoint) )
            {
                Robot* Object = global_Project->getActiveScene()->getRobotByNameContaining("OBJECT");

                confPtr_t q_curr = Object->getCurrentPos();
                (*q_curr)[6] = WSPoint[0];
                (*q_curr)[7] = WSPoint[1];
                (*q_curr)[8] = WSPoint[2];

                Object->setAndUpdate(*q_curr);

                cout << "Set and update : " << Object->getName() << endl << WSPoint << endl;

                p3d_col_deactivate_rob_rob(Object->getP3dRobotStruct(),
                                           dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman()->getP3dRobotStruct());
            }
        }

        std::string robotName(robotPt->name);

        if( robotName == "HERAKLES_HUMAN1" ) {
            // Change here for drawing the otp grid (JIM THESIS)
            //qt_otp();
        }

        // Change the cost robot
        if ( ENV.getBool(Env::enableHri) && (!ENV.getBool(Env::HRINoRobot)))
        {
            std::string robotName(costRobot->name);

            // If the Robot moved is not ROBOT
            if( robotName.find("ROBOT") == string::npos &&(global_Project->getActiveScene()->getNumberOfRobots() > 1)  ) // Does not contain Robot
            {
                p3d_rob* robTmp = p3d_get_robot_by_name_containing("ROBOT");

                if (robTmp)
                {
                    costRobot = robTmp;
                    cost_q = p3d_get_robot_config(costRobot);
                }
                cout << "Change the robot position = " << robTmp->name << endl;
            }

            // Compute kinematic transfer point
            if ( ENV.getBool(Env::HRIcameraBehindHuman) )
            {
                //cout << "choseBestTransferPoint" << endl;
                if( HRICS_MotionPL != NULL )
                {
                    Eigen::Vector3d WSPoint;

                    if( dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->chooseBestTransferPoint(WSPoint, false, 0) )
                    {
                        HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();
                        reachSpace->computeIsReachableAndMove(WSPoint,reachSpace->getGrid()->isReachableWithLA(WSPoint));
                    }
                }
            }
        }

#endif
        Robot* costR(global_Project->getActiveScene()->getRobotByName(costRobot->name));

        if ( /*true ||*/ global_costSpace && (!ENV.getBool(Env::HRINoRobot)))
        {
            Configuration costConfig(costR,cost_q);
            std::cout << "Cost = " << global_costSpace->cost(costConfig) << std::endl;
        }

    }

    Robot* robot(global_Project->getActiveScene()->getRobotByName(robotPt->name));
    if ( PlanEnv->getBool(PlanParam::drawNaturalColor) && (robot->getName().find("HUMAN") != string::npos) && HRICS_activeNatu )
    {
        HRICS_activeNatu->setRobotColorFromConfiguration(true);
    }

#endif

//    cout << "features : " << API_activeFeatureSpace->getFeatures( *robot->getCurrentPos() ).transpose() << endl;

    /*

    int ncol = false;

    if( global_collisionSpace )
    {
        double distance = numeric_limits<double>::max();
        double potential = numeric_limits<double>::max();

        ncol = global_collisionSpace->isRobotColliding( distance, potential );

        cout << "Distance to nearest obstacle = " << distance << " and potential = " << potential << endl;

        if( global_optimizer && ( global_optimizer->getRobot() == robot ) ) {
            global_optimizer->getCollisionSpaceCost( *robot->getCurrentPos() );
        }

        if( ncol )
        {
            double colorvector[4];

            GroundColorMixGreenToRed( colorvector , 1.0 );

            g3d_set_custom_color_draw( robotPt, true );
            g3d_set_custom_color_vect( colorvector );
            cout << "Robot colliding!!!" << endl;
        }
        else {
            g3d_set_custom_color_draw( robotPt, false );
            cout << "Robot not colliding" << endl;
        }
    }
    else
    {
        Robot* robot(global_Project->getActiveScene()->getRobotByName(robotPt->name));
        ncol = robot->isInCollision();
    }

    //HRICS::setThePlacemateInIkeaShelf();
    g3d_set_draw_coll( ncol );

    */
//    cout << "ncol : " << ncol << endl;
}
