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
#ifndef MAINWINDOWREMOTE_HPP
#define MAINWINDOWREMOTE_HPP


class GLWidget;
class PosterReader;

#include "qtLibrary.hpp"
#include <QSettings>
#include <QTimer>
#include "p3d_sys.h"
#include "../p3d/env.hpp"
#include "../p3d/ParametersEnv.hpp"

namespace Ui {
        class MainWindowRemote;
}

#ifdef ATTENTIONAL_REMOTE
class AttentionalWidget;
#endif

/**
 * @ingroup qtMainWindowRemote
 * @brief Qt Main Window container
 * Tow Widget are derived from other classes The GLWidget widget and the MoveRobot Widget
 \image html Designer.png
 */
class MainWindowRemote : public QMainWindow
{
        Q_OBJECT

public:
        MainWindowRemote(PosterReader *pr, QWidget *parent = 0);
        ~MainWindowRemote();

public slots:
        void loadParametersQuick();
  
private slots:
        void saveInterfaceParameters();
        void loadInterfaceParameters();
        void saveParametersQuick();
  
        //void on_switchSparkView_clicked(bool checked);
        void on_pushButtonSaveVideo_toggled(bool checked);
        void on_pushButtonSaveSettings_clicked();
        void on_pushButtonLoadSettings_clicked();
        void setRobotAsCurrent();
        void saveVideoTimer();
        //void on_checkBox_clicked(bool checked);

protected:
        //void keyPressEvent(QKeyEvent *e);
        //void keyReleaseEvent(QKeyEvent *e);
        void closeEvent(QCloseEvent *event);

private:
        PosterReader *m_pr;
  
public:
        Ui::MainWindowRemote*	m_ui;

private:  
        std::vector<QAction*> m_RobotsInMenu;


        void initRobotsMenu();

        bool userReallyWantsToQuit();
        bool userWantsToLoadSettings();
        void initCamera();
        void saveDockSettings(QSettings & settings, QString dockName, QDockWidget * dockWidget);
        void loadDockSettings(QSettings & settings, QString dockName, QDockWidget * dockWidget);

        QTimer* timer;
#ifdef ATTENTIONAL_REMOTE
        QDockWidget *dockWidgetAttentional;
        AttentionalWidget *dockAttentional;
#endif
};

#endif // MAINWINDOWREMOTE_HPP
