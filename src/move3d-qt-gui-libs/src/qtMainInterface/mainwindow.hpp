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
#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <libmove3d/include/p3d_sys.h>
#include <libmove3d/p3d/env.hpp>
#include <libmove3d/p3d/ParametersEnv.hpp>

#include <libmove3d/planners/API/Trajectory/trajectory.hpp>

#include "qtMainInterface/kcdpropertieswindow.hpp"
#include "qtBase/qt_widgets.hpp"
#include "qtFormRobot/moverobot.hpp"

#ifdef USE_QWT
#include "qtPlot/histoWin.hpp"
#endif

#ifndef GLWIDGET_H
class GLWidget;
#endif

#ifndef MAIN_WINDOW_TEST_FUNCTIONS
class MainWindowTestFunctions;
#endif

#include <vector>
#include <QTimer>

namespace Ui
{
class MainWindow;
}

class VideoRecorder : public QThread
{
    Q_OBJECT

public:
    //VideoRecorder(QObject* parent = 0);
    VideoRecorder(double msec, GLWidget* display) {
        m_msec = msec;
        m_display = display;
    }
    ~VideoRecorder() { }

signals:
    void timeout();

protected:
    void run();

    double m_msec;
    GLWidget* m_display;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Main Window container
 * Tow Widget are derived from other classes The GLWidget widget and the MoveRobot Widget
 \image html Designer.png
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    LabeledSlider* createSlider(QString s, Env::intParameter p,int lower, int upper);
    LabeledDoubleSlider* createDoubleSlider(QString s,Env::doubleParameter p, double lower, double upper);

    /* returns the Ui auto genrated structure */
    Ui::MainWindow* Ui() { return m_ui; }

    GLWidget*	getOpenGL();
    MoveRobot*	getMoveRobot();

public slots:
    void drawAllWinActive();
    void isPlanning();
    void planningFinished();

    void setBoolGhost(bool value);
    void setBoolBb(bool value);
    void setBoolFloor(bool value);
    void setBoolTiles(bool value);
    void setBoolWalls(bool value);
    void setBoolSmooth(bool value);
    void setBoolShadows(bool value);
    void setBoolFilaire(bool value);
    void setBoolJoints(bool value);
    void setBoolContour(bool value);
    void setBoolEnableLight(bool value);
    void setBoolEnableShaders(bool value);
    void setBoolFlatBox(bool value);

    void setCurrentTraj(p3d_traj* traj);
    void refreshConstraintedDoFs();

    void addTab(QWidget* tab, std::string name);

signals:
    void runClicked();
    void stopClicked();
    void resetClicked();

protected:
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);

public slots:
    void loadParametersQuick();

private slots:
    void switchSpeedVsPosition(bool enable);
    void setRobotAlongTraj(double param);

    void saveInterfaceParameters();
    void loadInterfaceParameters();
    void saveParametersQuick();

    void initShortcuts();

    void initRobotsMenu();
    void setRobotAsCurrent();

    void openScenario();
    void saveScenario();

    void setDrawColl();

    void loadGraph();
    void saveGraph();
    void saveXYZGraphToDot();

    void loadTraj();
    void saveTraj();

    void changeLightPosX();
    void changeLightPosY();
    void changeLightPosZ();

    void setTrajToDrawAsCurrent();
    void addglobal_trajToDraw();
    void clearglobal_trajToDraw();
    void colorTrajChange(int color);

    void enableRunAndResetButtons();
    void enableStopButton();
    void enableRunButton();
    void showTraj();
    void showTrace();
    void restoreView();
    void mobileCamera();
    void changeCamera();

    void test();
    void saveVideo();
    void saveVideoTimer();

    // Global
    //    void setLineEditWithNumber(Env::intParameter p , int val );
    void changeEvent(QEvent *e);
    void envSelectedPlannerTypeChanged(bool isPRMvsDiffusion);

signals:
    void selectedPlanner(QString);

private:

    Ui::MainWindow*					m_ui;

    KCDpropertiesWindow*			mKCDpropertiesWindow;

    MainWindowTestFunctions*		m_testFunctions;

    //! Robot menu
    std::vector<QAction*>			m_RobotsInMenu;

    //! New tabs
    std::vector<QWidget*> new_tabs_;

    void connectCheckBoxes();

    void initRunButtons();
    void initViewerButtons();
    void initLightSource();

    // Screen recording
    QTimer *timer;
    bool isRecording;

    // Show traj
    Move3D::Trajectory current_traj_;
    double traj_fps_tmp_;
    long int traj_id_;


};

// Global MainWindow Pointer 
extern MainWindow* global_w;


#endif // MAINWINDOW_H
