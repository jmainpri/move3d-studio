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
    bool updateSparkEnv();

signals:
    void sparkStatus(bool st);

public:
    //! Display functions
    MainWindowRemote* m_win;

    GenomPoster * _sparkPoster;
    SPARK_CURRENT_ENVIRONMENT _sparkPosterStruct;

    GenomImagePoster * _viamImagePoster;

};

#endif // POSTERREADER_HPP
