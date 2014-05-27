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
#include "picowebimage.hpp"

#include <iostream>
#include "images/jimmy-xpm.h"
using namespace std;

PicowebImage::PicowebImage(QString host, int port, QString pathWithQuery)
{
    _host = host;
    _port = port;
    _pathWithQuery = pathWithQuery;
    _http = new QHttp();
    connect(_http, SIGNAL(requestFinished(int, bool)),this, SLOT(flushImage(int, bool)));
    _buffer = new QBuffer(&_bytes);
    _buffer->open(QIODevice::ReadWrite);
    _http->setHost(_host,QHttp::ConnectionModeHttp, _port);

    _update = false;
}

PicowebImage::~PicowebImage()
{

}

void PicowebImage::run()
{
    _update = true;
}

void PicowebImage::flushImage(int requestId, bool error)
{
    if(_update && error==false)
    {
        QPixmap pm;
        QImage qimg;
        _bytes = _http->readAll();
        qimg.loadFromData(_bytes);
        _image = _image.fromImage(qimg, 0);
        QThread::msleep(60);
        _Request =_http->get(_pathWithQuery);
    } else if ( !_update && error==false){
        _image = QPixmap(jimmy_img);
    } else {
        _image = QPixmap(jimmy_img);
        QThread::msleep(2000);
        _Request =_http->get(_pathWithQuery);
    }
    emit imageReady();
}

void PicowebImage::quit()
{
    _update = false;
}

void PicowebImage::stop()
{
    _update = false;
}


