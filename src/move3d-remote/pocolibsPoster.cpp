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



using namespace std;


FetchEnvironment::FetchEnvironment(QWidget* obj)
{
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(refresh()));
  timer->start(100);
}
  
bool FetchEnvironment::init(MainWindowRemote *win)
{ 
  m_win = win;
  _EnvPoster = "sparkEnvironment";
  return findPoster(_EnvPoster, &_EnvPosterID);
}

bool FetchEnvironment::findPoster(std::string str, POSTER_ID *posterId)
{
    if(posterFind(str.c_str(), posterId)==ERROR)
      {
        printf("ERROR: FetchEnvironment::init() can't find poster %s\n", str.c_str());
        *posterId = NULL;
        return false;
      }

    cout << "Poster " << str << " found !!!!" << endl;
    return true;
}

 void FetchEnvironment::setSparkRefresh(bool checked)
 {
   _sparkRefrech = checked;

 }

bool FetchEnvironment::refresh()
{
  SPARK_CURRENT_ENVIRONMENT envPoster;
  int i,j;
  p3d_rob *robotPt;

  if(!_sparkRefrech)
  {
      _sparkStatus = false;
      return false;
  }

  if(_EnvPosterID == NULL)
  {
      if(init(m_win) == false)
      {
          /* spark is not updating */
          _sparkStatus = false;
          return false;
      }
  }

  /* spark is updating */
  _sparkStatus = true;

  int size = posterRead(_EnvPosterID,0, &envPoster, sizeof(envPoster));
  if( size != sizeof(envPoster))
    {
      cout << "ERROR: FetchEnvironment::refresh() poster size mismatch (" << size << " , " << sizeof(envPoster) <<  " )"  << endl;
      return false;
    }
  else
    { 
      if( strcmp(envPoster.envName.name, XYZ_ENV->name))
	{
	  cout << "S_mhp_POSTER_NOT_COMPATIBLE (env name is wrong or num robots)" << endl;
          cout << envPoster.envName.name << endl;
	  cout << XYZ_ENV->name << endl;
	  cout << "---" << endl;
          cout << "env.robotnb " << envPoster.robotNb << endl;
	  cout << XYZ_ENV->nr << endl;
	  return false;
	}
      else 
	{
	  // Set all Robots and Agents configurations
          for(i=0; i<envPoster.robotNb; i++)
	    {
              robotPt = p3d_get_robot_by_name(envPoster.robot[i].name.name);

              if( robotPt == NULL) {
                  printf("robot not found %s toto\n",envPoster.robot[i].name.name );
                  return false;
              }
              if(envPoster.robot[i].length!=robotPt->nb_dof) {
                printf("length is false for robot %s (%d) %d \n",envPoster.robot[i].name.name, envPoster.robot[i].length, robotPt->nb_dof);
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
      
              if(robotPt == NULL) {
		return false;
	      }
              for(j=0; j<6; j++) {
                robotPt->ROBOT_POS[j+6] = envPoster.freeflyer[i].q[j];
	      }

	      p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_POS);
	    }
   

	}
      m_win->drawAllWinActive();
      return true;
    }
}
