#ifndef SPARKWIDGET_H
#define SPARKWIDGET_H

#include "qtOpenGL/glwidget.hpp"
#include <QWidget>
#include "posterreader.hpp"
#include "ui_paramwidget.h"


namespace Ui {
    class sparkWidget;
}

class sparkWidget : public QWidget 
{
    Q_OBJECT
public:
    sparkWidget(QWidget *parent = 0);
    ~sparkWidget();
    GLWidget* getOpenGL();
   void init(PosterReader *pr, Ui::ParamWidget *m_ui_param);

private slots:
    void drawAllWinActive();

private slots:
    void setBoolGhost(bool value);
    void setBoolBb(bool value);
    void setBoolFloor(bool value);
    void setBoolSky(bool value);
    void setBoolTiles(bool value);
    void setBoolWalls(bool value);
    void setBoolSmooth(bool value);
    void setBoolShadows(bool value);
    void setBoolFilaire(bool value);
    void setBoolJoints(bool value);
    void setBoolContour(bool value);
    void setBoolEnableLight(bool value);
    void setBoolEnableShaders(bool value);
    void restoreView();


    void changeLightPosX();
    void changeLightPosY();
    void changeLightPosZ();

protected:
    void changeEvent(QEvent *e);

private:
    PosterReader *m_pr;
    Ui::ParamWidget *m_ui_p;
    Ui::sparkWidget *m_ui;
    
    void initLightSource();
    /**
     * Function tro create sliders and checkboxes TODO move somwhere else
     */
    void connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p);
    void connectCheckBoxes();
};

#endif // SPARKWIDGET_H
