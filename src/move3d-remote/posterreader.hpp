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
#ifndef POSTERREADER_HPP
#define POSTERREADER_HPP

#include "qtLibrary.hpp"
#include "mainwindow-remote.hpp"


#include "genomposter.hpp"
#include "picowebimage.hpp"

#include "duplicatedGenomStruct.hpp"

#include "softMotion/Sm_Traj.h"


extern void draw_smtraj_tace();
extern void draw_monitoring_spheres();


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
    void monitoringSpheresDraw(bool b);
    void drawSmTraj();
    void drawGoToPos();
    void drawMS();
    void setDrawGoTo(bool b);

    void changesoftmotiondt(double dt);
    void changesOpacity(double opacity);

private:
    // Spark ftc and data
    bool updateSparkEnv();

    bool _drawTraj;
    bool _drawGotoPos;
    bool _drawMonitoringSphere;
  
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

    GenomPoster * _monitoringSpheresPoster;
    SPARK_ALL_MONITORING_SPHERES _monitoringSpheresPosterStruct;

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
    MHP_ROBOTFUTURPOS_POSTER_STR _mhpRobotGoTo;
double _dt;
double _opacity;
    SM_TRAJ _smTraj;
};

#endif // POSTERREADER_HPP
