#ifndef CAMERAWIDGET_H
#define CAMERAWIDGET_H

#include <QWidget>
#include "qtLibrary.hpp"

#include "posterreader.hpp"


namespace Ui {
    class cameraWidget;
    class ParamWidget;
}

class cameraWidget : public QWidget 
{
    Q_OBJECT
public:
    cameraWidget(QWidget *parent = 0);
    ~cameraWidget();
    void init(PosterReader *pr, Ui::ParamWidget *ui_param);

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
    Ui::ParamWidget *m_ui_p;
    Ui::cameraWidget *m_ui;
    
    uchar *_dataImageLeft;
    uchar *_dataImageRight;
    QImage* _qimageLeft;
    QImage* _qimageRight;
};

#endif // CAMERAWIDGET_H
