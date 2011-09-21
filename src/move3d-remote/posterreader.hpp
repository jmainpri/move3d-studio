#ifndef POSTERREADER_HPP
#define POSTERREADER_HPP

#include "qtLibrary.hpp"
#include "mainwindow-remote.hpp"
#include "genomposter.hpp"
#include "genomimageposter.hpp"

#include "duplicatedGenomStruct.hpp"


class PosterReader : public QObject
{
      Q_OBJECT
public:
    PosterReader(MainWindowRemote* win);
    ~PosterReader();

    void init();
    //const GenomPoster & getSparkPoster(){return *_sparkPoster;}
    GenomPoster *getSparkPoster(){return _sparkPoster;}

//    GenomPoster *getVimanImageLeftPoster(){return _vimanImageLeftPoster;}
//    GenomPoster *getVimanImageRightPoster(){return _vimanImageRightPoster;}

private slots:
    void update();

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

public:
    //! Display functions
    MainWindowRemote* m_win;
  
    //! Posters
    GenomPoster * _sparkPoster;
    SPARK_CURRENT_ENVIRONMENT _sparkPosterStruct;

    GenomImagePoster * _viamImagePoster;
  
    GenomPoster * _niutPoster;
    NIUT_HUMAN_LIST _niutPosterStruct;
};

#endif // POSTERREADER_HPP
