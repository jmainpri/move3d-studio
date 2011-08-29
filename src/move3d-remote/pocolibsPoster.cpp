/*
 *  pocolibsPoster.cpp
 *  Move3D-Qt-Gui
 *
 *  Created by Jim Mainprice on 28/01/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#include "pocolibsPoster.hpp"

#include "qtLibrary.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#include <portLib.h>
#include <posterLib.h>

using namespace std;

static POSTER_ID posterEnvID=NULL;

//----------------------------------------------------
// SPARK
//----------------------------------------------------
#define M3D_MAX_DOF 65
#define SPARK_MAX_AGENT_NB 5
#define SPARK_MAX_FREEFLYER_NB 30

typedef struct GEN_STRING64 {
  char name[64];
} GEN_STRING64;

typedef struct STRUCT_M3D_ROBOT {
  GEN_STRING64 name;
  double q[M3D_MAX_DOF];
  int length;
  int unused;
} M3D_ROBOT;

typedef struct STRUCT_M3D_FREEFLYER {
  GEN_STRING64 name;
  double q[6];
} M3D_FREEFLYER;

typedef struct STRUCT_SPARK_CURRENT_ENVIRONMENT {
  GEN_STRING64 envName;
  M3D_ROBOT robot[SPARK_MAX_AGENT_NB];
  M3D_FREEFLYER freeflyer[SPARK_MAX_FREEFLYER_NB];
  int robotNb;
  int freeflyerNb;
  int time;
  int unused;
} SPARK_CURRENT_ENVIRONMENT;


GEN_STRING64 EnvPoster = { "sparkEnvironment"};

//----------------------------------------------------
// FETCH ENVIRONMENT
//----------------------------------------------------

FetchEnvironment::FetchEnvironment(QWidget* obj)
{
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(refresh()));
  timer->start(50);
}
  

bool FetchEnvironment::init(MainWindowRemote *win)
{ 
  if(posterFind(EnvPoster.name, &posterEnvID)==ERROR)
    {
      printf("Can't set Environment Poster\n");
      return false;
    }

  m_win = win;
  cout << "Poster Found!!!!" << endl;
  return true;
}


bool FetchEnvironment::refresh()
{
  SPARK_CURRENT_ENVIRONMENT envPoster;
  int i,j;
  p3d_rob *robotPt;

  // if(posterFind(EnvPoster.name, &posterEnvID)==ERROR)
  //   {
  //     printf("Can't find Environment Poster\n");
  //     return false;
  //   }

  int size = posterRead(posterEnvID,0, &envPoster, sizeof(envPoster));
  
  if( size != sizeof(envPoster)) 
    {
      cout << "poster size mismatch (" << size << " , " << sizeof(envPoster) <<  " )"  << endl;
      return false;
    }
  else
    { 
      if( strcmp(envPoster.envName.name, XYZ_ENV->name) || (envPoster.robotNb!=XYZ_ENV->nr)) 
	{
	  cout << "S_mhp_POSTER_NOT_COMPATIBLE (env name is wrong or num robots)" << endl;
	  cout << envPoster.envName.name << endl;
	  cout << XYZ_ENV->name << endl;
	  cout << "---" << endl;
	  cout << envPoster.robotNb << endl;
	  cout << XYZ_ENV->nr << endl;
	  return false;
	}
      else 
	{
	  // Set all Robots and Agents configurations
	  for(i=0; i<envPoster.robotNb; i++) 
	    {
	      robotPt = p3d_get_robot_by_name(envPoster.robot[i].name.name);
      
	      if( robotPt == NULL || (envPoster.robot[i].length!=robotPt->nb_dof)) {
	
		return false;
	      }

	      for(j=0; j<robotPt->nb_dof; j++) {
		robotPt->ROBOT_POS[j] = envPoster.robot[i].q[j];
	      }

	      p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_POS);
	    }    

	  // Set all FreeFlyers and Objects configurations
	  for(i=0; i<envPoster.freeflyerNb; i++) 
	    {
	      robotPt = p3d_get_robot_by_name(envPoster.freeflyer[i].name.name);
      
	      if( robotPt == NULL || (envPoster.robot[i].length!=robotPt->nb_dof)) {
	
		return false;
	      }

	      for(j=0; j<6; j++) {
		robotPt->ROBOT_POS[j+6] = envPoster.robot[i].q[j];
	      }

	      p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_POS);
	    }
   

	}
      m_win->drawAllWinActive();
      return true;
    }
}
