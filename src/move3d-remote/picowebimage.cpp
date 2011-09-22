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
        _image = _image.scaledToHeight(200);
        QThread::msleep(60);
        _Request =_http->get(_pathWithQuery);
        cout << "request send" << endl;
    } else {
        _image = QPixmap(jimmy_img);
        _image = _image.scaledToHeight(200);
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




