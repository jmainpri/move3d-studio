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

private slots:
    void on_checkBox_toggled(bool checked);
};

#endif // CAMERAWIDGET_H
