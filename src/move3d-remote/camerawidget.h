#ifndef CAMERAWIDGET_H
#define CAMERAWIDGET_H

#include <QWidget>
#include "qtLibrary.hpp"

#include "posterreader.hpp"

#include "ui_mainwindow-remote.h"

namespace Ui {
    class cameraWidget;
}

class cameraWidget : public QWidget 
{
    Q_OBJECT
public:
    cameraWidget(PosterReader *pr = NULL, Ui::MainWindowRemote *ui_parent = NULL, QWidget *parent = 0);
    ~cameraWidget();
    void init(PosterReader *pr, Ui::MainWindowRemote *ui_parent);

protected:
    void changeEvent(QEvent *e);

public slots:
    void updateImageLeft();
    void updateImageRight();

public:
    QLabel* labelImgLeft(){return _labelImageLeft;}
    QLabel* labelImgRight(){return _labelImageRight;}
    QLabel* _labelImageLeft;
    QLabel* _labelImageRight;

private:
    PosterReader *m_pr;
    Ui::MainWindowRemote *m_ui_p;
    Ui::cameraWidget *m_ui;
    
    uchar *_dataImageLeft;
    uchar *_dataImageRight;
    QImage* _qimageLeft;
    QImage* _qimageRight;
};

#endif // CAMERAWIDGET_H
