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
