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
/*
 * Header File for the qtOpenGL Widget
 */

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "p3d_matrix.h"
#include "qtLibrary.hpp"

#ifndef MAINWINDOW_HPP
class MainWindow;
#endif

class Move3D2OpenGl;

#ifndef WITH_XFORMS
#include "Graphic-pkg.h"
#endif

void qt_get_win_mouse(int* i, int* j);
void qt_ui_calc_param(g3d_cam_param& p);

/**
  * @ingroup qtWindow
  * @brief Open GL viewer implemetation in Qt
  */
class GLWidget: public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();

    void setMainWindow(MainWindow* w) { m_mainWindow = w; }
    void setWinSize(double size);
    void setSaveTraj(bool save_traj) { m_save_traj = save_traj; }
    void setSaveOnDisk(bool save_on_disk) { m_save_on_disk = save_on_disk; }
    void resetImageVector();
    void setThreadWorking(bool isWorking);
    void newG3dWindow();
    void initG3DFunctions();
    // void updateGL();

public slots:

    void saveView();
    //void reinitGraphics();
    void addCurrentImage();
    void saveImagesToDisk();
    void myPaintGL();

signals:

    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);
    void zoomChanged(int value);

protected:

    // OpenGL functions
    void initializeGL();
    void paintGL(); // This is called when changing environments.
    void resizeGL(int width, int height);
    void computeNewVectors(p3d_vector4& Xc,p3d_vector4& Xw,p3d_vector4& up);

    // Mouse events
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
    void mouseDoubleClickEvent(QMouseEvent *event);

private:

    // Viewer ID
    int m_id;

    bool m_save_traj;
    bool m_save_on_disk;

    // Pointer that allows resizing
    MainWindow*	m_mainWindow;

    // OpenGl variables
    GLdouble   x,y,z,el,az,zo;

    // size of the OpenGl scene
    double size;

    p3d_vector4  up;

    QPoint lastPos;

    // Colors for background
    QColor trolltechGreen;
    QColor trolltechPurple;
    QColor trolltechGrey;
    QColor trolltechBlack;
    QColor trolltechWhite;

    bool _light;
    bool _watingMouseRelease;

    // Do not draw when this
    // variable is true
    bool _isThreadWorking;

    // Vector of recorded images
    QVector<QImage*> _pictures;
    int picsId;

    // Counts the number of draw
    int paintNum;

#ifndef WITH_XFORMS
    qtG3DWindow* mG3DOld;
#endif
};

extern int mouse_mode;

#endif
