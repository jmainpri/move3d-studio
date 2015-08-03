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

#include "mainwindowTestFunctions.hpp"
#include "mainwindowGenerated.hpp"

#include "planner_handler.hpp"

using namespace std;

MainWindowTestFunctions::MainWindowTestFunctions(MainWindow* MainWinPt) : m_mainWindow(MainWinPt)
{
    connect(this, SIGNAL(selectedPlanner(QString)),global_plannerHandler, SLOT(startPlanner(QString)));

    connect(m_mainWindow->Ui()->pushButtonTest1,SIGNAL(clicked()),this,SLOT(test1()));
    connect(m_mainWindow->Ui()->pushButtonTest2,SIGNAL(clicked()),this,SLOT(test2()));
    connect(m_mainWindow->Ui()->pushButtonTest3,SIGNAL(clicked()),this,SLOT(test3()));
}



void MainWindowTestFunctions::test1()
{
    cout << "------------------- test1 -------------------" << endl;
    cout << "---------------------------------------------" << endl;

    emit(selectedPlanner(QString("test1")));
}

void MainWindowTestFunctions::test2()
{
    using namespace std;

    cout << "------------------- test2 -------------------" << endl;
    cout << "---------------------------------------------" << endl;
    emit(selectedPlanner(QString("test2")));
}

void MainWindowTestFunctions::test3()
{
    cout << "------------------- test3 -------------------" << endl;
    cout << "---------------------------------------------" << endl;
    emit(selectedPlanner(QString("test3")));
}
