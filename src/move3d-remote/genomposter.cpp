#include "genomposter.hpp"
#include <iostream>
#include <unistd.h>
#include <QTimer>

#include <stdio.h>
using namespace std;

QSemaphore mySem(1);

GenomPoster::GenomPoster(std::string name, char* posterStruct, int posterSize, unsigned long rate)
{
    _posterName = name;
    //setStackSize (10000000);
    _posterStruct = posterStruct;
    _posterSize = posterSize;
    /* rate is 100 ms by default */
    _rate = rate;
    _posterID = NULL;
    _updatingStatus = false;
    _refreshStatus = false;
    _stopThread = false;
}

void GenomPoster::setRefreshStatus(bool b) {
     _refreshStatus = b;
}

void GenomPoster::run()
{
    while(!_stopThread)
    {
        if( _stopThread )
        {
            return;
        }
        if(_refreshStatus)
        {
            update();
        } else {
            _updatingStatus = false;
        }
        QThread::msleep(_rate);
    }
}

void GenomPoster::stop()
{
    _stopThread = true;
}

void GenomPoster::update() {
  mySem.acquire();
    if(_posterID == NULL)
    {
        if(findPoster() == false)
        {
            _updatingStatus = false;
        }
    } else {
      //printf("read poster %p \n", _posterID);
            int size = posterRead(_posterID,0, _posterStruct, _posterSize);
 
            if( size != _posterSize)
            {
                _updatingStatus = false;
                cout << "ERROR: GenomPoster::refresh() poster " << _posterName << " size mismatch (" << size << " , " << _posterSize << " ) for posterID " << _posterID << endl;
                QThread::msleep(2000);
            }
            /* poster is updated */
            _updatingStatus = true;
        }
    mySem.release();
    return;
}

bool GenomPoster::getPosterStuct(char *posterStruct) {
    posterStruct = _posterStruct;
    return _updatingStatus;
}

bool GenomPoster::setRate(int r)
{
    if(r <= 0)
    {
        cout << "ERROR: GenomPoster::setRate " << r << " wrong rate" << endl;
        return false;
    }                
    _rate = r;
    return true;
}


bool GenomPoster::findPoster()
{
    if(posterFind(_posterName.c_str(), &(_posterID))==ERROR)
    {
        cout << "ERROR: GenomPoster::findPoster can't find poster \""<< _posterName << "\", retry within 2 sec. " << endl;
        _posterID = NULL;
        QThread::msleep(2000);
        return false;
    }
    cout << "*********************  INFO: GenomPoster::findPoster: Poster " <<  _posterName << " found ***********************" << endl;
    return true;
}
