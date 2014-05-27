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
            if (this)
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
            emit dataReady();
            //cout << "GenomPoster::refresh() poster " << _posterName << endl;
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
