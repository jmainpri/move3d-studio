//
//  sliderfunction.cpp
//  MOVE3DSTUDIO
//
//  Created by Jim Mainprice on 20/10/11.
//  Copyright 2011 LAAS-CNRS. All rights reserved.
//

#include "sliderfunction.hpp"

#include "planner/cost_space.hpp"
#include "planner/Greedy/CollisionSpace.hpp"

#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_costspace.hpp"
#include "HRI_costspace/HRICS_Miscellaneous.hpp"
#endif

#include "API/ConfigSpace/configuration.hpp"
#include "API/Device/robot.hpp"

#include "p3d/env.hpp"

#include "P3d-pkg.h"
#include "Collision-pkg.h"

using namespace std;
using namespace tr1;

//! function that computes a cost 
//! on the robot actual configuration
void qtSliderFunction(p3d_rob* robotPt, configPt p)
{
  //cout << "qtSliderFunction" << endl;
  
  /*  if ( mRobot->getName().find("HUMAN") != string::npos ) 
   {
   shared_ptr<Configuration> qSeated = mRobot->getCurrentPos();
   HRI_AGENTS* agents = hri_create_agents();
   //		Eigen::Vector3d head = mRobot->getJoint(5)->getVectorPos();
   
   hri_agent_set_human_seated_posture(agents->humans[0], qSeated->getConfigStruct());
   //hri_agent_compute_posture(agents->humans[0],head[2],1.3,qSeated->getConfigStruct());
   mRobot->setAndUpdate(*qSeated);
   //hri_destroy_agents(agents);
   }
   */
  
  if (ENV.getBool(Env::isCostSpace))
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
        
        shared_ptr<Configuration> q_curr = Object->getCurrentPos();
        
        (*q_curr)[6] = WSPoint[0];
        (*q_curr)[7] = WSPoint[1];
        (*q_curr)[8] = WSPoint[2];
        
        Object->setAndUpdate(*q_curr);
        
        cout << "Set and update : " << Object->getName() << endl << WSPoint << endl;
        
        p3d_col_deactivate_rob_rob(Object->getRobotStruct(), 
                                   dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman()->getRobotStruct());
			}
    }
    
    // Change the cost robot
    if ( ENV.getBool(Env::enableHri) && (!ENV.getBool(Env::HRINoRobot))) 
    {
      std::string robotName(costRobot->name);
      
      // If the Robot moved is not ROBOT
      if( robotName.find("JIDOKUKA") == string::npos && 
         (global_Project->getActiveScene()->getNumberOfRobots() > 1)  ) // Does not contain Robot
      {
        p3d_rob* robTmp = p3d_get_robot_by_name_containing("JIDOKUKA");
        
        if (robTmp) 
        {
          cost_q = p3d_get_robot_config(costRobot);
          costRobot = robTmp;
        }
        //cout << "Change the robot position = " << robotPt->name << endl;
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
      //std::cout << "Cost = " << p3d_GetConfigCost(costRobot,cost_q) << std::endl;
      std::cout << "Cost = " << global_costSpace->cost(costConfig) << std::endl;
    }
    
        if ( (costR->getName().find("HUMAN") != string::npos) && HRICS_activeNatu ) 
        {
          if (HRICS_MotionPL) {
            cout << "reachSpace->setRobotColorFromConfiguration(true)" << endl;;
            HRICS::Natural* reachSpace = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getReachability();
          }
          HRICS_activeNatu->setRobotColorFromConfiguration(true);
        }
#endif
  }
  //	cout << "robotPt->name = " << robotPt->name << endl;
  //	cout << "XYZ_ROBOT = " << XYZ_ROBOT->name << endl;
  
#ifdef P3D_COLLISION_CHECKING
  int ncol = false;
  
  if( global_collisionSpace )
  {
    double distance = numeric_limits<double>::max();
    
    ncol = global_collisionSpace->isRobotColliding( distance );
    
    cout << "Distance Potential To Nearest Obstacle = " << distance << endl;
    
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
#endif
  
  //HRICS::setThePlacemateInIkeaShelf();
  g3d_set_draw_coll( ncol );
}
