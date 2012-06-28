#include "posterreader.hpp"

#include "qtOpenGL/glwidget.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include <unistd.h>

#include "planner_handler.hpp"
#include "move3d-gui.h"
#include "API/scene.hpp"
#include "API/project.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"


#include "../lightPlanner/proto/lightPlanner.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/ManipulationArmData.hpp"
#include "../lightPlanner/proto/ManipulationPlanner.hpp"
#include <stdlib.h>

#include <QMessageBox>
#include "softMotion/Sm_Traj.h"

using namespace std;

PosterReader* ptrPosterReader=NULL;

vector<vector<double> > _2dTraj;

void draw_smtraj_tace()
{
    if(ptrPosterReader)
        ptrPosterReader->drawSmTraj();
    ptrPosterReader->drawGoToPos();
}

PosterReader::PosterReader()
{

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(10);

    _drawTraj = false;
    _drawGotoPos = false;
    /* declaration of the poster reader threads */
    _sparkPoster = new GenomPoster("sparkEnvironment", (char*)(&_sparkPosterStruct), sizeof(SPARK_CURRENT_ENVIRONMENT), 10);
    _sparkPoster->setRefreshStatus(true);

    _dt = 0.2;
    /* declaration of the poster reader threads */
    _softmotionPoster = new GenomPoster("mhpArmTraj", (char*)(&_softmotionPosterStruct), sizeof(SM_TRAJ_STR), 10);
    _softmotionPoster->setRefreshStatus(true);
    // Camera

    std::string host;
    char * posterPath;
    posterPath = getenv ("POSTER_PATH");
    printf("posterPath is %s", posterPath);
    if(strncmp(posterPath,"jido-arm",4) == 0) {
        host = "jido-arm.laas.fr";
        _picowebLeftImg = new PicowebImage(QString(host.c_str()), 8080, QString("/viman?bank=Images&image=CameraTopLeft&quality=25"));
        _picowebRightImg = new PicowebImage(QString(host.c_str()), 8080, QString("/viman?bank=Images&image=CameraTopRight&quality=25"));
        cout << "poster path is jido" << endl;
    } else if(strncmp(getenv("POSTER_PATH"),"pr2c2",5) == 0) {
        host = "pr2c2.laas.fr";
        _picowebLeftImg = new PicowebImage(QString(host.c_str()), 8080, QString("/viman?bank=Images&image=CAMERA_TOP_LEFT&quality=50"));
        _picowebRightImg = new PicowebImage(QString(host.c_str()), 8080, QString("/viman?bank=Images&image=CAMERA_TOP_RIGHT&quality=50"));
        cout << "poster path is pr2" << endl;
    } else {
        cout << "ERROR CANNOT DETERMINE POSTER_PATH" << endl;

    }

    // Niut reader
    _niutPoster = new GenomPoster("niutHuman", (char*)(&_niutPosterStruct), sizeof(NIUT_HUMAN_LIST), 2000);
    _niutPoster->setRefreshStatus(true);
    _niutWatchDog = 0;
    _niutDeathCounter = 0;
    _niutPrevId = 0;
 #ifdef ATTENTIONAL_REMOTE
    // Attentional reader
    _attentionalPoster = new GenomPoster("attentionalReport", (char*)(&_attentionalPosterStruct), sizeof(ATTENTIONAL_REPORT_STR), 10);
    _attentionalPoster->setRefreshStatus(true);

    _attentionalOutputPoster = new GenomPoster("attentionalOutput", (char*)(&_attentionalOutputPosterStruct), sizeof(ATTENTIONAL_OUTPUT_STR), 10);
    _attentionalOutputPoster->setRefreshStatus(true);
#endif
    ptrPosterReader = this;


    /* poster for robot in mhp */
    _mhpPoster = new GenomPoster("mhprobotFuturPos", (char*)(&_mhpRobotGoTo), sizeof(MHP_ROBOTFUTURPOS_POSTER_STR), 10);
    _mhpPoster->setRefreshStatus(false);
}

void PosterReader::changesoftmotiondt(double dt) {

    _dt = dt;
}

PosterReader::~PosterReader()
{
    delete _sparkPoster;
    delete _niutPoster;
    delete _picowebLeftImg;
    delete _picowebRightImg;
    delete _softmotionPoster;
    delete _mhpPoster;
   #ifdef ATTENTIONAL_REMOTE
    delete _attentionalPoster;
    delete _attentionalOutputPoster;
#endif
}


void PosterReader::init()
{
    cout << "start thread for spark ..." << endl;
    _sparkPoster->start();
    cout << "   ... spark thread started" << endl;
    cout << "start thread for picowebLeftImg ..." << endl;
    _picowebLeftImg->start();
    cout << "   ... picowebLeftImg thread started" << endl;
    cout << "start thread for _picowebRightImg ..." << endl;
    _picowebRightImg->start();
    cout << "   ... _picowebRightImg thread started" << endl;
    cout << "start thread for niut ..." << endl;
    _niutPoster->start();
    cout << "   ... niut thread started" << endl;

    _softmotionPoster->start();
    cout << "   ... niut thread started" << endl;
     #ifdef ATTENTIONAL_REMOTE
    _attentionalPoster->start();
    cout << "   ... attentional thread started" << endl;
    _attentionalOutputPoster->start();
    cout << "   ... attentionalOutput thread started" << endl;
#endif
    _mhpPoster->start();
    cout << "   ... mhp thread started" << endl;
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
                if(_mhpPoster!= NULL)
                {
                    if(_mhpPoster->getPosterStuct((char *)(&_mhpRobotGoTo)) != false)
                    {
                        if (!strcmp(_sparkPosterStruct.robot[i].name.name , _mhpRobotGoTo.robot_to_draw.robot_name.name))
                        {
                            for(j=0; j<robotPt->nb_dof; j++) {
                                robotPt->ROBOT_GOTO[j] = _mhpRobotGoTo.robot_to_draw.q[j];
                            }
                            _2dTraj.clear();
                            for(int m=0; m< _mhpRobotGoTo.robot_to_draw.traj.nbPts ; m++) {
                                vector<double> v;
                                v.push_back(_mhpRobotGoTo.robot_to_draw.traj.points[m].x);
                                v.push_back(_mhpRobotGoTo.robot_to_draw.traj.points[m].y);
                                v.push_back(_mhpRobotGoTo.robot_to_draw.traj.points[m].theta);
                                _2dTraj.push_back(v);
                            }
                        }
                    }

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
        emit drawAllWinActive();
        return true;
    }
}

bool PosterReader::updateNiut()
{
    if( _niutPoster == NULL )
    {
        cout << "Niut : NULL Poster" << endl;
        emit niutIsAlive(false);
        return false;
    }

    // We keep track of niut's counter
    // if the value doesn't change for a while it's set as dead
    //_niutPrevId = _niutPosterStruct.watch_dog;

    if(_niutPosterStruct.watch_dog != _niutWatchDog)
    { _niutDeathCounter = 0; }
    else
    { _niutDeathCounter++; }

    //  cout << "_niutWatchDog : " << _niutWatchDog ;
    //  cout << " , _niutPosterStruct.watch_dog : " << _niutPosterStruct.watch_dog << endl;

    _niutWatchDog = _niutPosterStruct.watch_dog;

    if( _niutDeathCounter > 1000 )
    {
        emit niutIsAlive(false);
        return false;
    }
    // Associtating mode
    for(unsigned int i=0; i<16;i++)
    {
        emit setNiutColorLabel(i,0);
    }
    // Associtating mode
    for(unsigned int i=0; i<16;i++)
    {
        int id = _niutPosterStruct.users[i].id;

        switch (_niutPosterStruct.users[i].state)
        {
        case NIUT_NO_TRACKING:
            emit setNiutColorLabel(id,0);
            break;

        case NIUT_POSE_SEARCH:
            emit setNiutColorLabel(id,1);
            break;

        case NIUT_CALIBRATE:
            emit setNiutColorLabel(id,2);
            break;

        case NIUT_TRACKING:
            emit setNiutColorLabel(id,3);
            break;

        default:
            cout << "Error : Kinect user state is of wrong type" << endl;
            break;
        }
    }
    //  cout << "Niut is alive association done!!!" <<endl;
    emit niutIsAlive(true);
    return true;
}

void PosterReader::softmotionPlotTraj()
{

    if( _softmotionPoster == NULL )
    {
        cout << "softmotion : NULL Poster" << endl;
        return;
    }
    //_softmotionPoster->update();
    SM_TRAJ smTraj;
    if(_softmotionPoster->getPosterStuct((char *)(&_softmotionPosterStruct)) == false) {
        cout << " PosterReader::softmotionPlotTraj() ERROR " << endl;
    }

    smTraj.importFromSM_TRAJ_STR(&_softmotionPosterStruct);

    if( smTraj.getDuration() <= 0) {

        // QMessageBox  toto(QMessageBox::Information, ,QMessageBox::Close, NULL,Qt::Dialog);
        QMessageBox::about (NULL, QString("SoftMotion info"), QString("Trajectory is empty"));

        return;
    }

    cout << "duration " << smTraj.getDuration() << endl;
    smTraj.plot();
    return;
}



void PosterReader::drawSmTraj()
{
    if(_drawTraj == true) {
        p3d_rob * robotPt = NULL;
        if(_softmotionPoster->getPosterStuct((char *)(&_softmotionPosterStruct)) == false) {
            cout << " PosterReader::softmotionPlotTraj() ERROR " << endl;
        }
        for(int i=0; i<_sparkPosterStruct.robotNb; i++) {
            robotPt = p3d_get_robot_by_name("PR2_ROBOT");
            if( robotPt == NULL) {
                //printf("robot not found %s\n",_sparkPosterStruct.robot[i].name.name );
                continue;
            }
        }

        if( robotPt == NULL) {
            printf("robot not found %s\n","PR2_ROBOT");
            return;
        }

        _smTraj.importFromSM_TRAJ_STR(&_softmotionPosterStruct);


        if( _smTraj.getDuration()  <= 0 ) {
                return;
        }

        std::vector<SM_COND> cond;

        configPt q = p3d_alloc_config(robotPt);
        robotPt->draw_transparent = true;

        for (double t=0; t< _smTraj.getDuration() ; t = t +  _dt) {
            // t= _smTraj.getDuration() + 1;
            _smTraj.getMotionCond(t, cond);
            p3d_get_robot_config_into(robotPt, &q);

            q[6] = cond[0].x;
            q[7] = cond[1].x;
            q[11] = cond[5].x;
            // torso
            q[12] = cond[6].x;
            //head
            q[13] = cond[7].x;
            q[14] = cond[8].x;
            //leftarm
            q[25] = cond[19].x;
            q[26] = cond[20].x;
            q[27] = cond[21].x;
            q[28] = cond[22].x;
            q[29] = cond[23].x;
            q[30] = cond[24].x;
            q[31] = cond[25].x;
            //rightarm
            q[16] = cond[10].x;
            q[17] = cond[11].x;
            q[18] = cond[12].x;
            q[19] = cond[13].x;
            q[20] = cond[14].x;
            q[21] = cond[15].x;
            q[22] = cond[16].x;
            //G3D_WIN->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;

            p3d_set_and_update_this_robot_conf(robotPt, q);
            g3d_draw_robot(robotPt->num, G3D_WIN, 0);
        }

       _smTraj.getMotionCond(_smTraj.getDuration(), cond);
        p3d_get_robot_config_into(robotPt, &q);

        q[6] = cond[0].x;
        q[7] = cond[1].x;
        q[11] = cond[5].x;
        // torso
        q[12] = cond[6].x;
        //head
        q[13] = cond[7].x;
        q[14] = cond[8].x;
        //leftarm
        q[25] = cond[19].x;
        q[26] = cond[20].x;
        q[27] = cond[21].x;
        q[28] = cond[22].x;
        q[29] = cond[23].x;
        q[30] = cond[24].x;
        q[31] = cond[25].x;
        //rightarm
        q[16] = cond[10].x;
        q[17] = cond[11].x;
        q[18] = cond[12].x;
        q[19] = cond[13].x;
        q[20] = cond[14].x;
        q[21] = cond[15].x;
        q[22] = cond[16].x;
        //G3D_WIN->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;

        p3d_set_and_update_this_robot_conf(robotPt, q);
        g3d_draw_robot(robotPt->num, G3D_WIN, 0);
        robotPt->draw_transparent = false;
        p3d_destroy_config(robotPt, q);
    }
    return;
}

void PosterReader::softmotionDrawTraj(bool b)
{
    if( _softmotionPoster == NULL )
    {
        cout << "softmotion : NULL Poster" << endl;
        return;
    }
    _drawTraj = b;
    if(b) {

        cout << "_drawTraj = true " << endl;
    } else {
        cout << "_drawTraj = false " << endl;
    }
    return;
}

void PosterReader::drawGoToPos()
{
    if(_drawGotoPos) {

        p3d_rob * robotPt = NULL;
        for(int i=0; i<_sparkPosterStruct.robotNb; i++) {
            robotPt = p3d_get_robot_by_name("PR2_ROBOT");
            if( robotPt == NULL) {
                //printf("robot not found %s\n",_sparkPosterStruct.robot[i].name.name );
                continue;
            }
        }

        if( robotPt == NULL) {
            printf("robot not found %s\n","PR2_ROBOT");
            return;
        }


        configPt q = p3d_alloc_config(robotPt);
        configPt q_cur = robotPt->ROBOT_POS;
        configPt q_todraw = robotPt->ROBOT_GOTO;
        robotPt->draw_transparent = true;

        p3d_set_and_update_this_robot_conf(robotPt, q_todraw);
        g3d_draw_robot(robotPt->num, G3D_WIN, 0);

        for (unsigned int i = 0; i < _2dTraj.size(); i++)
        {
            q_cur[6] = _2dTraj.at(i).at(0);
            q_cur[7] = _2dTraj.at(i).at(1);
            q_cur[11] = _2dTraj.at(i).at(2);
            p3d_set_and_update_this_robot_conf(robotPt, q_cur);
            g3d_draw_robot(robotPt->num, G3D_WIN, 0);
        }
        robotPt->draw_transparent = false;
        p3d_destroy_config(robotPt, q);
    }
    return;
}

void PosterReader::setDrawGoTo(bool b)
{
    if( _mhpPoster == NULL )
    {
        cout << "mhp ERROR : NULL Poster" << endl;
        return;
    }
    _drawGotoPos = b;
    if(b) {
        _mhpPoster->setRefreshStatus(true);
        cout << "_drawGotoPos = true " << endl;
    } else {
        _mhpPoster->setRefreshStatus(false);
        cout << "_drawGotoPos = false " << endl;
    }
    return;
}


