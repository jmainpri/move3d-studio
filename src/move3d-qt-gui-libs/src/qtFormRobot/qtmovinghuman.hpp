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
#ifndef QTMOVINGHUMAN_HPP
#define QTMOVINGHUMAN_HPP

#include <QWidget>

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtBase/SpinBoxSliderConnector_p.hpp"
#include "planner/planEnvironment.hpp"

namespace Ui {
    class MovingHuman;
}

class MovingHuman : public QWidget {
    Q_OBJECT
public:
    MovingHuman(QWidget *parent = 0);
    MovingHuman(double x, double y, double rz, QWidget *parent = 0);
    ~MovingHuman();

    // getters and setters
    void setX(double x);
    void setY(double y);
    void setZ(double z);
    void setRZ(double rz);

    double getX();
    double getY();
    double getZ();
    double getRZ();

    void init(double x, double y, double z, double rz);
    void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MovingHuman *ui;
    MainWindow* m_mainWindow;

    QtShiva::SpinBoxSliderConnector*	m_k_x;
    QtShiva::SpinBoxSliderConnector*	m_k_y;
    QtShiva::SpinBoxSliderConnector*	m_k_z;
    QtShiva::SpinBoxSliderConnector*	m_k_rz;


private slots:
    void updateMainWindow(double );
    void printValues();
};

#endif // QTMOVINGHUMAN_HPP
