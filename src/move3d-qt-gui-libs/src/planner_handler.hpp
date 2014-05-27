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
#ifndef PLANNER_HANDLER_HPP_INCLUDED
#define PLANNER_HANDLER_HPP_INCLUDED

#include <QtCore/QObject>
#include <QtCore/QString>

#include "P3d-pkg.h"

#include <string>

// UI functions
void qt_add_traj(char* name, int id, p3d_rob* rob, p3d_traj* traj);
void qt_add_config_to_ui(char* name, p3d_rob* rob, double* q);

// Read/Write functions
void qt_readScenario();
void qt_saveScenario();
void qt_readTraj();
void qt_load_HRICS_Grid(std::string gridName);
void qt_init_after_params();

// Variable used for file reading
extern const char *qt_fileName;

// Planner Handler object
class PlannerHandler : public QObject
{
    Q_OBJECT;

public:
    enum state {
        running, // A planning algorithm instance is running
        stopped, // A planning algorithm instance has been created, but is stopped
        none }; // There is no instance created

    PlannerHandler(int argc, char** argv);

public slots:
    void init();
    void startPlanner(QString plannerName);
    void stopPlanner();
    void resetPlanner();

signals:
    void initIsDone();
    void plannerIsStopped();
    void plannerIsReset();

protected:
    state mState;
    int mArgc;
    char** mArgv;
};

extern PlannerHandler* global_plannerHandler;

#endif
