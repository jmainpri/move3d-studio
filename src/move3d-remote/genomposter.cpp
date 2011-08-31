#include "genomposter.hpp"
#include <iostream>
#include <unistd.h>

#include <QTimer>

using namespace std;

GenomPoster::GenomPoster(std::string name, char* posterStruct, int posterSize, unsigned long rate)
{
    _posterName = name;
    _posterStruct = posterStruct;
    _posterSize = posterSize;
    /* rate is 100 ms by default */
    _rate = rate;
    _posterID = NULL;

    _updatingStatus = false;
}

void GenomPoster::run()
{
    cout << " run thread poster toto " << endl;
    while(!_stopThread)
    {

        if( _stopThread )
        {
            return;
        }
        update();

        QThread::msleep(_rate);
    }
}

void GenomPoster::stop()
{
    _stopThread = true;
}

void GenomPoster::update() {
    if(_posterID == NULL)
    {
        if(findPoster() == false)
        {
            emit statusChanged(false);
            _updatingStatus = false;
        }
    } else {

        int size = posterRead(_posterID,0, _posterStruct, _posterSize);
        if( size != _posterSize)
        {
            emit statusChanged(false);
            _updatingStatus = false;
            cout << "ERROR: GenomPoster::refresh() poster size mismatch (" << size << " , " << sizeof(_posterStruct) <<  " )"  << endl;
        }
        /* poster is updated */
        emit statusChanged(true);
        _updatingStatus = true;
    }   
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
        cout << "ERROR: GenomPoster::findPoster can't find poster " << _posterName << endl;
        _posterID = NULL;
        return false;
    }
    cout << "INFO: GenomPoster::findPoster: Poster " <<  _posterName << " found" << endl;
    return true;
}
