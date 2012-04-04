#ifndef POSTERREADER_HPP
#define POSTERREADER_HPP

#include "qtLibrary.hpp"
#include "mainwindow-remote.hpp"


#include "genomposter.hpp"
#include "picowebimage.hpp"

#include "duplicatedGenomStruct.hpp"

#include "softMotion/Sm_Traj.h"


extern void draw_smtraj_tace();

class PosterReader : public QObject
{
      Q_OBJECT
public:
    PosterReader();
    ~PosterReader();

    void init();
    //const GenomPoster & getSparkPoster(){return *_sparkPoster;}
    GenomPoster *getSparkPoster(){return _sparkPoster;}
    GenomPoster *getNiutPoster(){return _niutPoster;}
    GenomPoster *getSoftmotionPoster(){return _softmotionPoster;}
//    GenomPoster *getVimanImageLeftPoster(){return _vimanImageLeftPoster;}
//    GenomPoster *getVimanImageRightPoster(){return _vimanImageRightPoster;}

public slots:
    void update();
    void softmotionPlotTraj();
    void softmotionDrawTraj(bool b);
    void drawSmTraj();
    void drawGoToPos();
    void setDrawGoTo(bool b);

    void changesoftmotiondt(double dt);

private:
    // Spark ftc and data
    bool updateSparkEnv();

    bool _drawTraj;
    bool _drawGotoPos;
  
    // Niut fct and data
    bool updateNiut();
    void setNiutIsAlive(bool state);
    int _niutWatchDog;
    int _niutDeathCounter;
    int _niutPrevId;
  
signals:
    void sparkStatus(bool st);
    void niutIsAlive(bool st);
    void setNiutColorLabel(int i, int v);

    void drawAllWinActive();

public:
    //! Posters
    GenomPoster * _sparkPoster;
    SPARK_CURRENT_ENVIRONMENT _sparkPosterStruct;

    GenomPoster * _softmotionPoster;
    SM_TRAJ_STR _softmotionPosterStruct;

    PicowebImage * _picowebLeftImg;
    PicowebImage * _picowebRightImg;

    GenomPoster * _niutPoster;
    NIUT_HUMAN_LIST _niutPosterStruct;
 #ifdef ATTENTIONAL_REMOTE
    GenomPoster * _attentionalPoster;
    ATTENTIONAL_REPORT_STR _attentionalPosterStruct;

    GenomPoster * _attentionalOutputPoster;
    ATTENTIONAL_OUTPUT_STR _attentionalOutputPosterStruct;
#endif
    GenomPoster * _mhpPoster;
    MHP_ROBOT_GO_TO _mhpRobotGoTo;
double _dt;
    SM_TRAJ _smTraj;
};

#endif // POSTERREADER_HPP
