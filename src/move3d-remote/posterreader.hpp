#ifndef POSTERREADER_HPP
#define POSTERREADER_HPP

#include "qtLibrary.hpp"
#include "mainwindow-remote.hpp"
#include "genomposter.hpp"
#include "environmentStruct.h"


class PosterReader : public QObject
{
      Q_OBJECT
public:
    PosterReader(MainWindowRemote* win);
    ~PosterReader();

    void init();
    GenomPoster *getSparkPoster(){return _sparkPoster;}

private slots:
    void update();



private:
    bool updateSparkEnv();

signals:
    void sparkStatus(bool st);

private:
    //! Display functions
    MainWindowRemote* m_win;

    GenomPoster * _sparkPoster;
    SPARK_CURRENT_ENVIRONMENT _sparkPosterStruct;

};

#endif // POSTERREADER_HPP
