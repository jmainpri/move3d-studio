#include "posterreader.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include <unistd.h>

#include "../lightPlanner/proto/lightPlanner.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/ManipulationPlanner.hpp"


using namespace std;


PosterReader::PosterReader(MainWindowRemote* obj)
{
    m_win = obj;
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(10);


    /* declaration of the poster reader threads */
  _sparkPoster = new GenomPoster("sparkEnvironment", (char*)(&_sparkPosterStruct), sizeof(SPARK_CURRENT_ENVIRONMENT), 10);
  _sparkPoster->setRefreshStatus(false);

  // _viamImagePoster = new GenomImagePoster("vimanBridgeImage", 1000);
   _viamImagePoster = new GenomImagePoster("vimanImages", 10);
   _viamImagePoster->setRefreshStatus(true);
  
  // Niut reader
  _niutPoster = new GenomPoster("niutHuman", (char*)(&_niutPosterStruct), sizeof(NIUT_HUMAN_LIST), 10);
  _niutPoster->setRefreshStatus(false);
  _niutWatchDog = 0;
  _niutDeathCounter = 0;
  _niutPrevId = 0;
}

PosterReader::~PosterReader()
{
    delete _sparkPoster;
    delete _viamImagePoster;
    delete _niutPoster;
}

void PosterReader::init()
{
    cout << "start thread for spark ..." << endl;
    _sparkPoster->start();
    cout << "   ... spark thread started" << endl;
    cout << "start thread for viamImage ..." << endl;
    _viamImagePoster->start();
    cout << "   ... viamImage thread started" << endl;
}

void PosterReader::update()
{
  updateSparkEnv();
  updateNiut();
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////  Add after here the functions specific to each poster reading /////////////
/////////////////////////////////////////////////////////////////////////////////////////////

bool PosterReader::updateSparkEnv()
{
  int i,j;
  p3d_rob *robotPt = NULL;
  
  if(_sparkPoster == NULL)
  {
    return false;
  }
  
  if( _sparkPoster->getPosterStuct((char *)(&_sparkPosterStruct)) == false )
  {
    emit sparkStatus(false);
    return false;
  }
  else
  {
    if( strcmp(_sparkPosterStruct.envName.name, XYZ_ENV->name))
    {
      cout << "sparkPoster POSTER_NOT_COMPATIBLE (poster.envName= "
      << _sparkPosterStruct.envName.name << " p3d.envName= " << XYZ_ENV->name
      << " poster.robotNb= " << (_sparkPosterStruct.robotNb+_sparkPosterStruct.freeflyerNb) << " p3d.robotNb= " << XYZ_ENV->nr << ")" << endl;
      sleep(2);
      return false;
    }
    
    else
    {
      // Set all Robots and Agents configurations
      for(i=0; i<_sparkPosterStruct.robotNb; i++)
      {
        robotPt = p3d_get_robot_by_name(_sparkPosterStruct.robot[i].name.name);
        
        if( robotPt == NULL) {
          //printf("robot not found %s\n",_sparkPosterStruct.robot[i].name.name );
          continue;
        }
        if(_sparkPosterStruct.robot[i].length!=robotPt->nb_dof) {
          printf("Length is false for robot %s (%d) %d \n",_sparkPosterStruct.robot[i].name.name, _sparkPosterStruct.robot[i].length, robotPt->nb_dof);
          continue;
        }
        for (uint k = 0; k < (*robotPt->armManipulationData).size(); k++) {     
          deactivateCcCntrts(robotPt, k);
        }
        for(j=0; j<robotPt->nb_dof; j++) {
          robotPt->ROBOT_POS[j] = _sparkPosterStruct.robot[i].q[j];
        }
        
        p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_POS);
	    }
      
      // Set all FreeFlyers and Objects configurations
      for(i=0; i<_sparkPosterStruct.freeflyerNb; i++)
      {
        robotPt = p3d_get_robot_by_name(_sparkPosterStruct.freeflyer[i].name.name);
        
        if(robotPt == NULL) {
          //printf("freeflyer %s not found\n",_sparkPosterStruct.freeflyer[i].name.name);
          continue;
        }
        for(j=0; j<6; j++) {
          robotPt->ROBOT_POS[j+6] = _sparkPosterStruct.freeflyer[i].q[j];
        }
        
        p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_POS);
      }
    }
    emit sparkStatus(true);
    m_win->drawAllWinActive();
    return true;
  }
}

bool PosterReader::updateNiut()
{
  if( _niutPoster == NULL )
  {
    m_win->setNiutIsAlive( false );
    return false;
  }
  
  // We keep track of niut's counter
  // if the value doesn't change for a while it's set as dead
  _niutPrevId = _niutPosterStruct.watch_dog;
  
  if(_niutPosterStruct.watch_dog != _niutWatchDog) 
  { _niutDeathCounter = 0; }
  else
  { _niutDeathCounter++; }
  
  _niutWatchDog = _niutPosterStruct.watch_dog;
  
  if( _niutDeathCounter > 100 )
  {
    m_win->setNiutIsAlive( false );
    return false;
  }
  
  // Associtating mode
  for(unsigned int i=0; i<16;i++)
  {
    switch (_niutPosterStruct.users[i].state) 
    {
      case NIUT_NO_TRACKING:
        m_win->setNiutColorLabel(i,0);
        break;
      
      case NIUT_POSE_SEARCH:
        m_win->setNiutColorLabel(i,1);
        break;
        
      case NIUT_CALIBRATE:
        m_win->setNiutColorLabel(i,2);
        break;
        
      case NIUT_TRACKING:
        m_win->setNiutColorLabel(i,2);
        break;
        
      default:
        cout << "Error : Kinect user state is of wrong type" << endl;
        break;
    }
  }
  
  m_win->setNiutIsAlive( true );
  return true;
}



