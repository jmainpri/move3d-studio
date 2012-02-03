/*
 *  genomposter.hpp
 *  move3d-remote
 *
 *  Created by Xavier BROQUERE on 31/08/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef GENOMPOSTER_HPP
#define GENOMPOSTER_HPP
#include <QSemaphore>
#include <portLib.h>
#include <posterLib.h>

 #include <string>
 #include <QThread>

extern QSemaphore mySem;

class GenomPoster : public QThread
{
      Q_OBJECT
public:
    GenomPoster(std::string name, char* posterStruct,
                int posterSize, unsigned long rate);

    /* the main loop of the thread */
    void run();
    virtual void update();

    void setRefreshStatus(bool b);
    //! Find the poster given a posterName
    //! returns true if OK and fill posterId
    bool findPoster();

    bool isRefreshing(){return _refreshStatus;}
    bool isPosterUpdating(){return _updatingStatus;}
    bool isPosterFound(){return _posterFound;}
    bool getRate(){return _posterFound;}
    bool setRate(int r);
    bool getPosterStuct(char *posterStruct);

public slots:
    //void setRefresh(bool checked);
    void stop();

signals:
    void dataReady();


protected:
    std::string _posterName;
    POSTER_ID _posterID;
     /* _refresh status is specified by the user to
         activate or deactivate poster reading */
    bool _refreshStatus;
    /* _updating status is automatically updated */
    bool _updatingStatus;
    /* updating period in ms */
    bool _posterFound;
    /* updating rate in ms */
    int _rate;
    char* _posterStruct;
    int _posterSize;
    bool _stopThread;
};

#endif // GENOMPOSTER_HPP

