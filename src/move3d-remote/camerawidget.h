#ifndef CAMERAWIDGET_H
#define CAMERAWIDGET_H

#include <QWidget>
#include "qtLibrary.hpp"

#include "posterreader.hpp"

#include "ui_mainwindow-remote.h"

namespace Ui {
    class cameraWidget;
}

class cameraWidget : public QWidget {
    Q_OBJECT
public:
    cameraWidget(PosterReader *pr, Ui::MainWindowRemote *m_ui_parent, QWidget *parent = 0);
    ~cameraWidget();

protected:
    void changeEvent(QEvent *e);

public slots:
    void updateImageLeft();
    void updateImageRight();


public:

    QLabel *labelImgLeft(){return _labelImageLeft;}
    QLabel *labelImgRight(){return _labelImageRight;}
    QLabel* _labelImageLeft;
    QLabel* _labelImageRight;

private:
    Ui::cameraWidget *m_ui;
    Ui::MainWindowRemote *m_ui_p;
    PosterReader *m_pr;
    uchar *_dataImageLeft;
    uchar *_dataImageRight;
    QImage* _qimageLeft;
    QImage* _qimageRight;




};

#endif // CAMERAWIDGET_H
