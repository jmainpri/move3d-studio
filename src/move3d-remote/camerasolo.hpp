#ifndef CAMERASOLO_HPP
#define CAMERASOLO_HPP

#include <QWidget>
#include "qtLibrary.hpp"

#include "posterreader.hpp"

#include "ui_mainwindow-remote.h"

namespace Ui {
    class CameraSolo;
}

class CameraSolo : public QWidget
{
    Q_OBJECT

public:
    CameraSolo(PosterReader *pr, Ui::MainWindowRemote *m_ui_parent, QWidget *parent = 0);
    ~CameraSolo();

protected:
    void changeEvent(QEvent *e);

public slots:
    void updateImage();

public:
    QLabel *labelImg(){return _labelImage;}
    QLabel* _labelImage;

private:
    Ui::CameraSolo *m_ui;
    Ui::MainWindowRemote *m_ui_p;
    PosterReader *m_pr;
    uchar *_dataImage;
    QImage* _qimage;
};

#endif // CAMERASOLO_HPP
