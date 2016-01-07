/*
 *  qtMotionPlanner.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_MOTIONPLANNER_H
#define QT_MOTIONPLANNER_H

#include "../p3d/env.hpp"

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"

#include "qtFormRobot/qtmovinghuman.hpp"

// Foward declaration of class Node
// problem with boost graph and moc (until fix is not node)
// do not include boost graph in Qt headers
namespace Move3D
{
    class Node;
}

#ifdef USE_QWT
#include "qtPlot/basicPlotWindow.hpp"
#endif

namespace Ui
{
class MotionPlanner;
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class MotionPlanner : public QWidget
{
    Q_OBJECT

public:
    MotionPlanner(QWidget *parent = 0);
    ~MotionPlanner();

    void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }

    Move3D::Node* getIthNodeInActiveGraph();
    Ui::MotionPlanner* getMui(){return m_ui;}

public slots:
    Move3D::Node* getIthNodeInBestTraj();

private slots:

    // Optim -----------------------------
    void on_checkBoxSmooth_toggled(bool checked);
    void on_pushButtonComputeNavigation_clicked();
    void on_pushButtonSetGoal_toggled(bool checked);
    void test(double value);
    void computeGrid();
    void runMultiSmooth();
    void optimizeCost();
    void shortCutCost();
    void removeRedundant();
    void extractBestTraj();
    void setCostCriterium(int choise);
    void eraseDebugTraj();
    void cutTrajInSmallLP();
    void cutTrajAndOptimizeSM();

    // Multi-Run -------------------------
    void saveContext();
    void printContext();
    void deleteSelected();
    void printAllContext();
    void resetContext();
    void setToSelected();
    void runMultiRRT();
    void runAllRRT();
    void runAllGreedy();
    void showHistoWindow();

    // General ---------------------------
    void checkAllEdges();
    void computeAStarInCurrentGraph();
    void planASequenceOfPlans();
    void envDmaxSpinBoxValueChanged( double dmax );
    void envIsWithGoalValueChanged( bool state );
    void envBiDirValueChanged( bool state );
    void envRandomConnectionToGoalValueChanged( bool state );
    void envTryClosestValueChanged( bool state );
    void envMultiRRTValueChanged( bool state );
    void envPRMTypeChanged(int type );
    void testParam(double param);

    // Show ------------------------------
    void nodeToShowChanged();
    void edgeToShowChanged();
    void removeNode();

    // AStar -----------------------------
    void planAStarPath();

    // Sample Graph ----------------------
    void sampleGraph();
    void makeGridGraph();

signals:
    void selectedPlanner(QString);

private:
    Ui::MotionPlanner *m_ui;

    MainWindow *m_mainWindow;

    QListWidget* contextList;
    std::vector<QListWidgetItem*> itemList;

#ifdef USE_QWT_5
    BasicPlotWindow *plot;
    HistoWindow* histoWin;
#endif

    void initDiffusion();
    void initMultiRRT();
    void initPRM();
    void initMultiRun();
    void initOptim();
    void initGeneral();
    void initShowGraph();
    void initAStar();
    void initGraphSampling();
    MovingHuman* m_mh;
};

/**
 * @ingroup qtWindow
 * @brief Multi Planner thread class
 */
class MultiThread: public QThread
{
    Q_OBJECT

public:
    MultiThread(bool isRRT,QObject* parent = 0);

protected:
    void run();
    bool m_isRRT;
};

#endif
