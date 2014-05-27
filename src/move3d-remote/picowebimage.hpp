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
#ifndef PICOWEBIMAGE_HPP
#define PICOWEBIMAGE_HPP

#include "qtLibrary.hpp"

#include <QUrl>
#include <QBuffer>
#include <QtNetwork/QHttp>
#include <QThread>


class PicowebImage : public QThread
{
    Q_OBJECT
public:
    PicowebImage(QString host, int port, QString pathWithQuery);
    ~PicowebImage();

private:
    /* the main loop of the thread */
    void run();

public slots:
    void quit();
    void stop();

signals:
    void imageReady();

private slots:
    void flushImage(int requestId, bool error);

public:
    const QPixmap &image(){return _image;}

private:
    QPixmap _image;
    QString _host;
    int _port;
    QString _pathWithQuery;
    QBuffer *_buffer;
    QByteArray _bytes;
    QHttp *_http;
    int _Request;
    bool _update;
};

#endif // PICOWEBIMAGE_HPP
