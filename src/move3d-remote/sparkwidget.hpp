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
