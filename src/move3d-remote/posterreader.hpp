#ifndef POSTERREADER_HPP
#define POSTERREADER_HPP

#include "qtLibrary.hpp"
#include "mainwindow-remote.hpp"


#include "genomposter.hpp"
#include "picowebimage.hpp"

#include "duplicatedGenomStruct.hpp"


class PosterReader : public QObject
{
      Q_OBJECT
public:
    PosterReader();
    ~PosterReader();

    void init();
    //const GenomPoster & getSparkPoster(){return *_sparkPoster;}
    GenomPoster *getSparkPoster(){return _sparkPoster;}
    GenomPoster *getSoftmotionPoster(){return _softmotionPoster;}
//    GenomPoster *getVimanImageLeftPoster(){return _vimanImageLeftPoster;}
//    GenomPoster *getVimanImageRightPoster(){return _vimanImageRightPoster;}

private slots:
    void update();
    void softmotionPlotTraj();

private:
    // Spark ftc and data
    bool updateSparkEnv();
  
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
};

#endif // POSTERREADER_HPP
